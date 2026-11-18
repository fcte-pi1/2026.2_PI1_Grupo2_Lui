import { startServer, stopServer } from "./server";

void startServer().then((backend)=>{
  let stopping=false;
  const stop=()=>{if(stopping)return;stopping=true;void stopServer(backend).catch((error)=>{console.error(error);process.exitCode=1;});};
  process.once("SIGINT",stop);
  process.once("SIGTERM",stop);
}).catch((error: unknown)=>{console.error("Falha ao iniciar o backend:",error);process.exitCode=1;});
