import {test} from "node:test";
import assert from "node:assert/strict";
import {createRobotControl} from "../../communication/robotControl";
import {createBackendServer,stopServer} from "../../server";
import {createApp} from "../../app";
import {ingestBluetoothMessage} from "../telemetry.gateway";
const status=(extra={})=>({type:"robot_status",firmware:"maze",boot_id:123,remote_enabled:true,state:"ready",width:4,corner:0,heading:1,...extra});
const start={action:"start",width:4,corner:0,heading:1};
test("partida exige estado recente, firmware habilitado e configuração válida",async()=>{
 let now=1000;const sent:string[]=[];
 const c=createRobotControl({send:line=>{sent.push(line);return true;},now:()=>now});
 await assert.rejects(c.command(start),/Sem comunicação/);
 c.observe(JSON.stringify(status({remote_enabled:false})));
 await assert.rejects(c.command(start),/bloqueada/);
 c.observe(JSON.stringify(status()));
 await assert.rejects(c.command({...start,width:5}),/4×4/);
 now+=2600;await assert.rejects(c.command(start),/Sem comunicação/);
 assert.equal(sent.length,0);c.close();
});
test("só confirma após ACK, bloqueia partida simultânea e aceita parada",async()=>{
 const sent:any[]=[];const c=createRobotControl({send:line=>{sent.push(JSON.parse(line));return true;}});
 c.observe(JSON.stringify(status()));
 let settled=false;const running=c.command(start).then(ack=>{settled=true;return ack;});
 await Promise.resolve();assert.equal(settled,false);
 assert.equal(sent[0].boot_id,123);assert.equal(sent[0].width,4);
 await assert.rejects(c.command(start),/andamento/);
 const stopping=c.command({action:"stop"});
 c.observe(JSON.stringify({type:"robot_ack",id:sent[1].id,accepted:true,message:"Parado"}));
 assert.equal((await stopping).message,"Parado");
 c.observe(JSON.stringify({type:"robot_ack",id:sent[0].id,accepted:true,message:"Armada"}));
 assert.equal((await running).accepted,true);c.close();
});
test("timeout solicita parada e não repete nem guarda início para reconexão",async()=>{
 const sent:any[]=[];const c=createRobotControl({send:line=>{sent.push(JSON.parse(line));return true;},ackTimeoutMs:15});
 c.observe(JSON.stringify(status()));
 await assert.rejects(c.command(start),/não confirmou/);
 assert.deepEqual(sent.map(p=>p.action),["start","stop"]);
 assert.equal(c.snapshot().connected,false);
 c.observe(JSON.stringify(status()));assert.equal(sent.length,2);c.close();
});
test("firmware diagnóstico nunca recebe partida e desconexão bloqueia comandos",async()=>{
 const sent:string[]=[];const c=createRobotControl({send:line=>{sent.push(line);return true;}});
 c.observe(JSON.stringify({type:"bt_telemetry_test",motors_enabled:false}));
 await assert.rejects(c.command(start),/bloqueada/);
 c.observe(JSON.stringify({type:"bt_link_status",status:"disconnected"}));
 await assert.rejects(c.command({action:"stop"}),/Sem comunicação/);
 assert.equal(sent.length,0);c.close();
});
test("API local rejeita origem externa e só retorna sucesso depois da resposta Bluetooth",async()=>{
 const backend=createBackendServer(undefined,createApp());
 const sent:any[]=[];
 backend.bluetoothBridge={close:async()=>{},send:line=>{
  const command=JSON.parse(line);sent.push(command);
  setTimeout(()=>ingestBluetoothMessage(backend.gateway,JSON.stringify({type:"robot_ack",id:command.id,accepted:true,message:"Armada"})),10);
  return true;
 }};
 await new Promise<void>(done=>backend.server.listen(0,"127.0.0.1",done));
 const address=backend.server.address();assert.ok(address&&typeof address!=="string");
 const url=`http://127.0.0.1:${address.port}/api/robot`;
 try {
  ingestBluetoothMessage(backend.gateway,JSON.stringify(status()));
  assert.equal((await (await fetch(url+"/status")).json()).connected,true);
  const denied=await fetch(url+"/command",{method:"POST",headers:{"Content-Type":"application/json",Origin:"https://external.example"},body:JSON.stringify(start)});
  assert.equal(denied.status,403);assert.equal(sent.length,0);
  const response=await fetch(url+"/command",{method:"POST",headers:{"Content-Type":"application/json",Origin:"http://127.0.0.1:5173"},body:JSON.stringify(start)});
  assert.equal(response.status,200);assert.equal((await response.json()).accepted,true);
  assert.equal(sent.length,1);
 } finally {await stopServer(backend);}
});

test("mapa e telemetria chegam sem banco e uma nova missão limpa o cache anterior",async()=>{
 const {default:WebSocket}=await import("ws");
 const backend=createBackendServer(undefined,createApp());
 await new Promise<void>(done=>backend.server.listen(0,"127.0.0.1",done));
 const address=backend.server.address();assert.ok(address&&typeof address!=="string");
 const url=`ws://127.0.0.1:${address.port}/ws?role=dashboard`;
 const sockets:InstanceType<typeof WebSocket>[]=[];
 const connect=async()=>{
  const socket=new WebSocket(url);sockets.push(socket);const frames:any[]=[];
  socket.on("message",data=>frames.push(JSON.parse(data.toString())));
  await new Promise<void>(done=>socket.once("open",done));return frames;
 };
 try {
  const first=await connect();
  ingestBluetoothMessage(backend.gateway,JSON.stringify({type:"maze",width:4,height:4}));
  ingestBluetoothMessage(backend.gateway,JSON.stringify({robot:{x:2,y:1,heading:"EAST"},cell:{x:2,y:1,visited:true,walls:{north:true,south:null,east:false,west:true}}}));
  ingestBluetoothMessage(backend.gateway,JSON.stringify({type:"telemetry",session_id:9,t_ms:500,battery_voltage:null,pid_error:0,wheels_speed:{left:.1,right:.1},heading_deg:0,robot_state:"NAVIGATING"}));
  await new Promise(resolve=>setTimeout(resolve,20));
  assert.equal(first[0].width,4);assert.equal(first[1].cell.x,2);assert.equal(first[2].robot_state,"NAVIGATING");
  ingestBluetoothMessage(backend.gateway,JSON.stringify({type:"maze",width:8,height:4}));
  await new Promise(resolve=>setTimeout(resolve,20));
  const replay=await connect();await new Promise(resolve=>setTimeout(resolve,20));
  assert.equal(replay.length,1);assert.equal(replay[0].width,8);
 } finally {for(const socket of sockets)socket.terminate();await stopServer(backend);}
});

test("preserva o estado de recuperação de eco sem transformar a pausa em nova partida",()=>{
 const c=createRobotControl({send:()=>{throw new Error("não deve transmitir");}});
 c.observe(JSON.stringify(status({state:"running",recovering_front_echo:true})));
 assert.equal(c.snapshot().robot?.recovering_front_echo,true);
 c.observe(JSON.stringify(status({state:"running",recovering_front_echo:false})));
 assert.equal(c.snapshot().robot?.recovering_front_echo,false);c.close();
});
