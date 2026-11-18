import { test } from "node:test";
import assert from "node:assert/strict";
import { EventEmitter } from "node:events";
import { PassThrough } from "node:stream";
import type { ChildProcess } from "node:child_process";
import WebSocket from "ws";
import { validateBluetoothDiagnostic } from "../../communication/bluetoothDiagnostic";
import { startBluetoothNativeBridge } from "../../communication/bluetoothNative";
import { startServer, stopServer } from "../../server";
import { ingestBluetoothMessage } from "../telemetry.gateway";

const packet=()=>({type:"bt_telemetry_test",seq:7,uptime_ms:1750,heap_bytes:90000,motors_enabled:false,ultrasonic_enabled:false,
 imu:{valid:true,yaw_deg:12.5,gyro_z_dps:0.06,accel_g:{x:0.01,y:0.02,z:1}}});
test("valida medições reais e não transforma IMU indisponível em zero",()=>{
 const input=packet();const value=validateBluetoothDiagnostic(input,100);
 assert.equal(value.received_at_ms,100);assert.equal(value.imu.yaw_deg,12.5);
 input.imu.valid=false;
 assert.equal(validateBluetoothDiagnostic(input).imu.yaw_deg,null);
 assert.throws(()=>validateBluetoothDiagnostic({...packet(),motors_enabled:true}));
 assert.throws(()=>validateBluetoothDiagnostic({...packet(),seq:-1}));
 const invalid=packet();invalid.imu.yaw_deg=NaN;assert.throws(()=>validateBluetoothDiagnostic(invalid));
});

class Child extends EventEmitter {
 stdin=new PassThrough();stdout=new PassThrough();stderr=new PassThrough();exitCode:number|null=null;signalCode:string|null=null;
 kill(signal:string){this.signalCode=signal;queueMicrotask(()=>this.emit("close",null,signal));return true;}
}
test("ponte nativa enquadra chunks, reconecta e encerra sem deixar processo",async()=>{
 const children:Child[]=[];const lines:string[]=[];const statuses:string[]=[];
 const bridge=startBluetoothNativeBridge({address:"AA:BB:CC:DD:EE:FF",reconnectDelayMs:5,
  onLine:line=>lines.push(line),onStatus:status=>statuses.push(status),log:()=>{},
  spawnProcess:()=>{const child=new Child();children.push(child);return child as unknown as ChildProcess;}});
 children[0].stdout.write('{"type":"bt_tele');children[0].stdout.write('metry_test"}\ninvalid\n');
 assert.equal(lines.length,1);assert.deepEqual(statuses,["connecting","connected"]);
 let command="";children[0].stdin.on("data",chunk=>{command+=chunk.toString();});
 assert.equal(bridge.send('{"type":"robot_command","action":"stop"}'),true);
 assert.ok(command.endsWith("\n"));
 assert.equal(bridge.send("invalid\nframe"),false);
 children[0].stdout.write('{"type":"robot_status"}\n{"robot":{},"cell":{}}\n');
 assert.equal(lines.length,3);
 children[0].exitCode=1;children[0].emit("close",1);
 await new Promise(resolve=>setTimeout(resolve,15));assert.equal(children.length,2);
 await bridge.close();await new Promise(resolve=>setTimeout(resolve,10));assert.equal(children.length,2);
 assert.equal(children[1].signalCode,"SIGTERM");
 assert.throws(()=>startBluetoothNativeBridge({address:"invalid",onLine:()=>{},onStatus:()=>{}}));
});

function connect(url:string){return new Promise<{socket:WebSocket;next:()=>Promise<any>}>((resolve,reject)=>{
 const socket=new WebSocket(url);const queue:any[]=[];const waiting:Array<(value:any)=>void>=[];
 socket.on("message",raw=>{const value=JSON.parse(raw.toString());const next=waiting.shift();if(next)next(value);else queue.push(value);});
 socket.once("error",reject);socket.once("open",()=>resolve({socket,next:()=>{
  if(queue.length)return Promise.resolve(queue.shift());
  return new Promise((done,fail)=>{const timer=setTimeout(()=>fail(new Error("timeout")),1000);waiting.push(value=>{clearTimeout(timer);done(value);});});
 }}));
});}
test("monitor real HTTP/WS dispensa banco, retransmite Bluetooth e preserva idade no replay",async()=>{
 const backend=await startServer({port:0,liveOnly:true,databaseFactory:async()=>{throw new Error("não deve abrir banco");},log:()=>{}});
 const clients:WebSocket[]=[];
 try {
  const addr=backend.server.address();assert.ok(addr&&typeof addr!=="string");
  const base=`http://127.0.0.1:${addr.port}`;
  assert.equal((await fetch(base+"/health")).status,200);
  assert.equal((await fetch(base+"/api/sessions")).status,503);
  const first=await connect(base.replace("http:","ws:")+"/ws?role=dashboard");clients.push(first.socket);
  ingestBluetoothMessage(backend.gateway,JSON.stringify({type:"bt_link_status",status:"connected"}));
  assert.equal((await first.next()).status,"connected");
  ingestBluetoothMessage(backend.gateway,JSON.stringify(packet()));
  const reading=await first.next();assert.equal(reading.imu.yaw_deg,12.5);assert.ok(reading.received_at_ms>0);
  const second=await connect(base.replace("http:","ws:")+"/ws?role=dashboard");clients.push(second.socket);
  assert.equal((await second.next()).type,"bt_link_status");
  assert.equal((await second.next()).received_at_ms,reading.received_at_ms);
  const device=await connect(base.replace("http:","ws:")+"/ws?role=device");clients.push(device.socket);
  device.socket.send(JSON.stringify({...packet(),motors_enabled:true}));
  assert.equal((await device.next()).type,"gateway_error");
 } finally {for(const socket of clients)socket.terminate();await stopServer(backend);}
});
