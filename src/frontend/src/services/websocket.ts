/** URL substituível no ambiente; o encerramento explícito cancela reconexão. */
const WS_URL=import.meta.env.VITE_WS_URL ?? "ws://localhost:3000/ws?role=dashboard";
export function connectTelemetry(onMessage:(data:unknown)=>void) {
  let disposed=false;
  let socket:WebSocket;
  let retry:ReturnType<typeof setTimeout>|undefined;
  const connect=()=>{
    if(disposed)return;
    socket=new WebSocket(WS_URL);
    socket.onmessage=event=>{try{onMessage(JSON.parse(event.data) as unknown);}catch{console.error("Mensagem WebSocket inválida; JSON descartado");}};
    socket.onerror=()=>console.error("Falha na conexão WebSocket do dashboard");
    socket.onclose=()=>{if(!disposed)retry=setTimeout(connect,1000);};
  };
  connect();
  return {close(){disposed=true;if(retry)clearTimeout(retry);socket?.close();}};
}
