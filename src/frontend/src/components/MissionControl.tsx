import { useEffect, useState } from "react";
const API = import.meta.env.VITE_API_URL ?? "http://localhost:3000";
interface Status {
  connected: boolean;
  robot: null | {
    firmware: "maze" | "diagnostic"; remote_enabled: boolean;
    state: "ready" | "armed" | "running" | "completed" | "stopped" | "error";
    width: number; corner: number; heading: number; received_at_ms: number; error_reason?:string; recovering_front_echo?:boolean;
  };
}
const names = {ready:"Pronto",armed:"Partida em 5 segundos",running:"Explorando o labirinto",completed:"Concluído",stopped:"Parado",error:"Falha — confira o robô"};
export function MissionControl() {
  const [status,setStatus]=useState<Status|null>(null);
  const [width,setWidth]=useState(4);
  const [corner,setCorner]=useState(0);
  const [heading,setHeading]=useState(1);
  const [startPending,setStartPending]=useState(false);
  const [stopPending,setStopPending]=useState(false);
  const [feedback,setFeedback]=useState("");
  const [error,setError]=useState("");
  const [submitted,setSubmitted]=useState(false);
  useEffect(()=>{
    let active=true;
    let timer:ReturnType<typeof setTimeout>;
    const controller=new AbortController();
    async function poll() {
      try {
        const response=await fetch(`${API}/api/robot/status`,{signal:controller.signal});
        if(!response.ok)throw new Error("Controle indisponível");
        const data=await response.json() as Status;
        if(active){
          setStatus(data);
          if(data.connected && data.robot && ["completed","stopped","error"].includes(data.robot.state))setSubmitted(false);
        }
      } catch {if(active)setStatus(null);}
      finally {if(active)timer=setTimeout(poll,750);}
    }
    void poll();
    return()=>{active=false;controller.abort();clearTimeout(timer);};
  },[]);
  const connected=status?.connected===true;
  const robot=status?.robot;
  const busy=startPending||submitted||robot?.state==="armed"||robot?.state==="running";
  const canStart=connected&&robot?.firmware==="maze"&&robot.remote_enabled&&!busy;
  const reason=!connected ? "Sem comunicação recente com a ESP." : robot?.firmware!=="maze"
    ? "A ESP está com o teste Bluetooth. Partida indisponível neste firmware."
    : !robot.remote_enabled ? "Partida bloqueada no firmware enquanto a alimentação e os sinais ECHO dos sensores não forem validados." : "";
  async function command(action:"start"|"stop") {
    if(action==="start"&&!canStart)return;
    if(action==="start")setStartPending(true);else setStopPending(true);
    setError("");setFeedback(action==="start"?"Aguardando confirmação da ESP…":"Solicitando parada à ESP…");
    try {
      const response=await fetch(`${API}/api/robot/command`,{
        method:"POST",headers:{"Content-Type":"application/json"},
        body:JSON.stringify({action,...(action==="start"?{width,corner,heading}:{})}),
        signal:AbortSignal.timeout(13000),
      });
      const result=await response.json() as {accepted?:boolean;message?:string;erro?:string};
      if(!response.ok || result.accepted!==true)throw new Error(result.erro??result.message??"Comando não confirmado");
      setFeedback(`ESP confirmou: ${result.message ?? action}`);
      setSubmitted(action==="start");
    } catch(cause) {
      setFeedback("");setError(cause instanceof Error?cause.message:"Falha ao enviar comando");
    } finally {if(action==="start")setStartPending(false);else setStopPending(false);}
  }
  return <section aria-label="Comandos do robô" className="rounded-2xl border border-slate-200 bg-white p-4 dark:border-slate-800 dark:bg-slate-900">
    <div className="flex flex-wrap items-center justify-between gap-3">
      <h2 className="font-semibold">Missão no labirinto</h2>
      <p className="text-sm text-slate-500" data-testid="mission-state">{connected&&robot?(robot.recovering_front_echo?"Freado · aguardando eco frontal":names[robot.state]):"Aguardando ESP"}</p>
    </div>
    <div className="mt-3 flex flex-wrap items-end gap-3">
      <label className="text-sm">Labirinto<select aria-label="Labirinto da missão" disabled={busy} value={width} onChange={e=>setWidth(Number(e.target.value))} className="mt-1 block rounded-md border border-slate-300 bg-transparent p-2">
        {[4,8,12].map(w=><option key={w} value={w}>{w}×4</option>)}
      </select></label>
      <label className="text-sm">Canto inicial<select aria-label="Canto inicial" disabled={busy} value={corner} onChange={e=>setCorner(Number(e.target.value))} className="mt-1 block rounded-md border border-slate-300 bg-transparent p-2">
        {["Superior esquerdo","Superior direito","Inferior esquerdo","Inferior direito"].map((label,i)=><option key={label} value={i}>{label}</option>)}
      </select></label>
      <label className="text-sm">Frente apontada para<select aria-label="Orientação inicial" disabled={busy} value={heading} onChange={e=>setHeading(Number(e.target.value))} className="mt-1 block rounded-md border border-slate-300 bg-transparent p-2">
        {["Norte ↑","Leste →","Sul ↓","Oeste ←"].map((label,i)=><option key={label} value={i}>{label}</option>)}
      </select></label>
      <button type="button" disabled={!canStart} onClick={()=>void command("start")} className="rounded-md bg-emerald-700 px-4 py-2 font-medium text-white disabled:cursor-not-allowed disabled:opacity-40">{startPending?"Enviando…":"Iniciar missão"}</button>
      <button type="button" disabled={!connected||stopPending} onClick={()=>void command("stop")} className="rounded-md bg-red-700 px-4 py-2 font-medium text-white disabled:cursor-not-allowed disabled:opacity-40">{stopPending?"Parando…":"Parar robô"}</button>
    </div>
    <p className="mt-3 text-sm text-slate-500">Posicione o robô no início antes de cada partida. Destino: canto diagonalmente oposto. A ESP calibra imóvel e espera 5 segundos; limite de percurso: 180 segundos.</p>
    {robot&&connected&&<p className="mt-1 text-sm text-slate-500">Configuração informada pela ESP: {robot.width}×4. A posição no mapa é estimada pelos sensores e encoders.</p>}
    {connected&&robot?.state==="error"&&robot.error_reason&&<p role="alert" className="mt-2 text-sm text-red-700">Motivo da parada: {robot.error_reason}</p>}
    {reason&&<p className="mt-2 text-sm text-amber-700">{reason}</p>}
    {feedback&&<p role="status" aria-label="Resposta do comando" className="mt-2 text-sm">{feedback}</p>}
    {error&&<p role="alert" className="mt-2 text-sm text-red-700">{error}</p>}
    <p className="mt-2 text-xs text-slate-500">Parar depende da conexão Bluetooth. Mantenha acesso à chave da bateria. Reconectar o painel não inicia movimentos.</p>
  </section>;
}
