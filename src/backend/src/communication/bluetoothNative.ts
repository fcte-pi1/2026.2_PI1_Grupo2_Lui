import { spawn, type ChildProcess } from "node:child_process";
import { fileURLToPath } from "node:url";
import { StringDecoder } from "node:string_decoder";
export interface BluetoothNativeOptions {
  address: string;
  onLine: (line: string) => void;
  onStatus: (status: "connecting" | "connected" | "disconnected", message: string) => void;
  executable?: string;
  reconnectDelayMs?: number;
  log?: (message: string) => void;
  spawnProcess?: (command: string, args: string[]) => ChildProcess;
}
export function startBluetoothNativeBridge(options: BluetoothNativeOptions) {
  if (!/^[\da-f]{2}(?::[\da-f]{2}){5}$/i.test(options.address)) throw new Error("MM_BLUETOOTH_ADDRESS inválido; use AA:BB:CC:DD:EE:FF");
  const executable=options.executable ?? fileURLToPath(new URL("../../../firmwareTest/BluetoothTelemetry/tools/receber_mac",import.meta.url));
  const launch=options.spawnProcess ?? ((command,args)=>spawn(command,args,{stdio:["pipe","pipe","pipe"]}));
  const log=options.log ?? console.log;
  let child: ChildProcess | undefined;
  let stopped=false;
  let linkConnected=false;
  let retry: ReturnType<typeof setTimeout> | undefined;
  function connect() {
    if(stopped)return;
    options.onStatus("connecting","Conectando à ESP32");
    let connected=false, failed=false, pending="";
    const decoder=new StringDecoder("utf8");
    const disconnect=(message: string)=>{
      if(failed)return;failed=true;linkConnected=false;
      options.onStatus("disconnected",message);
      if(!stopped){retry=setTimeout(connect,options.reconnectDelayMs ?? 2000);retry.unref();}
    };
    try { child=launch(executable,[options.address,"/dev/null","0","--json"]); }
    catch(error){disconnect(String(error));return;}
    child.stdout?.on("data",(chunk: Buffer)=>{
      pending+=decoder.write(chunk);
      if(pending.length>65536){pending="";log("Bluetooth: linha excessiva descartada");return;}
      let end: number;
      while((end=pending.indexOf("\n"))>=0){
        const line=pending.slice(0,end).trim();pending=pending.slice(end+1);
        if(!line)continue;
        try {
          const packet=JSON.parse(line) as {type?:string};
          if(!["bt_test_info","bt_telemetry_test","pong","robot_status","robot_ack","maze","cell","position","telemetry","session_start","session_end","log"].includes(packet?.type ?? "") && !(packet && typeof packet === "object" && "cell" in packet))continue;
          if(!connected){connected=true;linkConnected=true;options.onStatus("connected","ESP32 conectada por Bluetooth");}
          options.onLine(line);
        } catch {log("Bluetooth: JSON inválido descartado");}
      }
    });
    child.stdin?.on("error",(error)=>disconnect(`Envio Bluetooth falhou: ${error.message}`));
    child.stderr?.on("data",(chunk: Buffer)=>log(`Bluetooth: ${chunk.toString().trim()}`));
    child.once("error",(error)=>disconnect(`Receptor indisponível: ${error.message}`));
    child.once("close",()=>disconnect("Bluetooth desconectado; tentando novamente"));
  }
  connect();
  return {
    send(line: string) {
      if(stopped || !linkConnected || !child?.stdin?.writable || line.length>512 || /[\r\n]/.test(line))return false;
      if(child.stdin.writableLength>4096)return false;
      child.stdin.write(line+"\n");return true;
    },
    async close() {
    stopped=true;if(retry)clearTimeout(retry);
    const active=child;
    if(!active || active.exitCode!==null || active.signalCode!==null)return;
    await new Promise<void>((resolve)=>{
      const fallback=setTimeout(()=>{active.kill("SIGKILL");resolve();},1500);fallback.unref();
      active.once("close",()=>{clearTimeout(fallback);resolve();});
      active.kill("SIGTERM");
    });
  }};
}
