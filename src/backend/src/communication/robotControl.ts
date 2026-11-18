import { randomUUID } from "node:crypto";
import { Router } from "express";

export type RobotStatus = {
  firmware: "maze" | "diagnostic";
  boot_id: number;
  remote_enabled: boolean;
  state: "ready" | "armed" | "running" | "completed" | "stopped" | "error";
  width: number; corner: number; heading: number;
  received_at_ms: number;
  error_reason?: string;
  recovering_front_echo?: boolean;
  front_echo_recoveries?: number;
  sensor_age_ms?: number;
  peak_sensor_age_ms?: number;
};
type Ack = { type: "robot_ack"; id: string; accepted: boolean; message: string };
export class RobotCommandError extends Error {
  constructor(message: string, readonly status = 409) { super(message); }
}
const record = (value: unknown): value is Record<string, unknown> =>
  typeof value === "object" && value !== null && !Array.isArray(value);
const integer = (value: unknown, min: number, max: number) =>
  Number.isInteger(value) && Number(value) >= min && Number(value) <= max;

/** No command replay, no offline queue. An HTTP success requires the ESP ACK. */
export function createRobotControl(options: {
  send: (line: string) => boolean;
  now?: () => number;
  ackTimeoutMs?: number;
}) {
  const now = options.now ?? Date.now;
  let latest: RobotStatus | null = null;
  let disconnected = true;
  let starting = false;
  let closed = false;
  const pending = new Map<string, { resolve: (ack: Ack) => void; reject: (error: Error) => void; timer: ReturnType<typeof setTimeout> }>();
  const snapshot = () => ({
    robot: latest,
    connected: !disconnected && latest !== null && now() - latest.received_at_ms < 2500,
  });
  function observe(line: string) {
    let p: unknown;
    try { p = JSON.parse(line); } catch { return; }
    if (!record(p)) return;
    if (p.type === "bt_link_status" && p.status !== "connected") { disconnected = true; return; }
    if (p.type === "bt_telemetry_test" && p.motors_enabled === false) {
      latest = {firmware:"diagnostic", boot_id:latest?.firmware === "diagnostic" ? latest.boot_id : 0, remote_enabled:false, state:"ready", width:4, corner:0, heading:1, received_at_ms:now()};
      disconnected = false;
    }
    if (p.type === "robot_status" && ["maze","diagnostic"].includes(String(p.firmware)) &&
        integer(p.boot_id,1,0xffffffff) && typeof p.remote_enabled === "boolean" &&
        ["ready","armed","running","completed","stopped","error"].includes(String(p.state)) &&
        [4,8,12].includes(Number(p.width)) && integer(p.corner,0,3) && integer(p.heading,0,3)) {
      latest = {firmware:p.firmware as RobotStatus["firmware"], boot_id:Number(p.boot_id), remote_enabled:p.remote_enabled,
        state:p.state as RobotStatus["state"], width:Number(p.width), corner:Number(p.corner), heading:Number(p.heading), received_at_ms:now(),
        ...(typeof p.error_reason==="string"?{error_reason:p.error_reason.slice(0,240)}:{}),
        ...(typeof p.recovering_front_echo==="boolean"?{recovering_front_echo:p.recovering_front_echo}:{}),
        ...(integer(p.front_echo_recoveries,0,65535)?{front_echo_recoveries:Number(p.front_echo_recoveries)}:{}),
        ...(integer(p.sensor_age_ms,0,0xffffffff)?{sensor_age_ms:Number(p.sensor_age_ms)}:{}),
        ...(integer(p.peak_sensor_age_ms,0,0xffffffff)?{peak_sensor_age_ms:Number(p.peak_sensor_age_ms)}:{})};
      disconnected = false;
    }
    if (p.type === "robot_ack" && typeof p.id === "string" && typeof p.accepted === "boolean" && typeof p.message === "string") {
      const request = pending.get(p.id);
      if (!request) return;
      pending.delete(p.id); clearTimeout(request.timer);
      request.resolve({type:"robot_ack",id:p.id,accepted:p.accepted,message:p.message.slice(0,240)});
    }
  }
  async function command(raw: unknown): Promise<Ack> {
    if (closed) throw new RobotCommandError("Controle encerrado",503);
    if (!record(raw) || !["start","stop"].includes(String(raw.action)))
      throw new RobotCommandError("Comando inválido",400);
    const status = snapshot();
    if (!status.connected) throw new RobotCommandError("Sem comunicação recente com a ESP",503);
    const start = raw.action === "start";
    if (start) {
      if (![4,8,12].includes(raw.width as number) || !integer(raw.corner,0,3) || !integer(raw.heading,0,3))
        throw new RobotCommandError("Use 4×4, 8×4 ou 12×4 e orientação/canto válidos",400);
      if (!latest!.remote_enabled || latest!.firmware !== "maze")
        throw new RobotCommandError("Partida bloqueada: firmware de navegação habilitado e sensores corrigidos são necessários");
      if (starting || ["armed","running"].includes(latest!.state))
        throw new RobotCommandError("Já existe uma partida ou missão em andamento");
    }
    if (pending.size >= 4 && start) throw new RobotCommandError("Aguarde a resposta da ESP");
    const id = randomUUID().replaceAll("-", "");
    const packet = {type:"robot_command",id,boot_id:latest!.boot_id,action:raw.action,
      ...(start ? {width:raw.width,corner:raw.corner,heading:raw.heading} : {})};
    if (start) starting = true;
    try {
      return await new Promise<Ack>((resolve,reject) => {
        const timer = setTimeout(() => {
          pending.delete(id);
          // A lost ACK is not permission to retry a start. Request a stop instead.
          if (start) options.send(JSON.stringify({type:"robot_command",id:randomUUID().replaceAll("-", ""),action:"stop",boot_id:packet.boot_id}));
          disconnected = true;
          reject(new RobotCommandError("ESP não confirmou o comando; partida não será repetida. Confira o robô antes de tentar novamente.",504));
        }, options.ackTimeoutMs ?? 10000);
        pending.set(id,{resolve,reject,timer});
        if (!options.send(JSON.stringify(packet))) {
          clearTimeout(timer);pending.delete(id);
          reject(new RobotCommandError("Transporte Bluetooth indisponível",503));
        }
      });
    } finally { if (start) starting = false; }
  }
  return {observe,snapshot,command,close() {
    closed=true;
    for(const request of pending.values()){clearTimeout(request.timer);request.reject(new RobotCommandError("Controle encerrado",503));}
    pending.clear();
  }};
}
export type RobotControl = ReturnType<typeof createRobotControl>;
export function createRobotControlRouter(control: RobotControl) {
  const router=Router();
  router.get("/status",(_req,res)=>res.json(control.snapshot()));
  router.post("/command",async(req,res)=>{
    // Control is local to this Mac. Reject commands originating from other sites.
    const origin=req.get("origin");
    const peer=req.socket.remoteAddress ?? "";
    if (!["127.0.0.1","::1","::ffff:127.0.0.1"].includes(peer)) {
      res.status(403).json({erro:"Controle disponível somente neste computador"});return;
    }
    if(origin){
      try {const url=new URL(origin);if(!["localhost","127.0.0.1","[::1]"].includes(url.hostname)||!["http:","https:"].includes(url.protocol))throw new Error();}
      catch {res.status(403).json({erro:"Origem do comando não permitida"});return;}
    }
    try {
      const ack=await control.command(req.body);
      res.status(ack.accepted?200:409).json(ack);
    } catch(error) {
      res.status(error instanceof RobotCommandError?error.status:500).json({erro:error instanceof Error?error.message:"Falha no comando"});
    }
  });
  return router;
}
