import http from "node:http";
import app from "./app";

const PORT = Number(process.env.PORT) || 3000;

const server = http.createServer(app);

server.listen(PORT, () => {
  console.log(`Backend executando em http://localhost:${PORT}`);
});