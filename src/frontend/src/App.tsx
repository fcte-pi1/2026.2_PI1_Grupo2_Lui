import { useCallback, useEffect, useState } from "react";
import { Dashboard } from "./pages/Dashboard";
import Historico from "./pages/Historico";
import { getSessions, type Sessao } from "./services/api";

type Tela = "dashboard" | "historico";

function mensagemErro(erro: unknown): string {
  return erro instanceof Error ? erro.message : "Erro ao comunicar com o servidor";
}

/**
 * Coordena a navegação e mantém uma única fonte de dados para a lista de sessões.
 * As duas telas permanecem montadas: navegar para o histórico não interrompe
 * o WebSocket nem descarta o estado vivo do mapa e da telemetria (HU-08/09).
 */
function App() {
  const diagnosticOnly=new URLSearchParams(window.location.search).get("fonte")==="bluetooth";
  const [tela, setTela] = useState<Tela>("dashboard");
  const [sessoes, setSessoes] = useState<Sessao[] | null>(null);
  const [erroSessoes, setErroSessoes] = useState<string | null>(null);
  const [carregandoSessoes, setCarregandoSessoes] = useState(!diagnosticOnly);
  const [versaoSessoes, setVersaoSessoes] = useState(0);

  const [pedidoSessoes,setPedidoSessoes]=useState(0);
  const atualizarSessoes=useCallback(()=>{
    if(diagnosticOnly)return;
    setCarregandoSessoes(true);
    setErroSessoes(null);
    setPedidoSessoes(value=>value+1);
  },[diagnosticOnly]);
  useEffect(()=>{
    if(diagnosticOnly)return;
    let active=true;
    void getSessions().then(dados=>{
      if(!active)return;
      setSessoes(dados);setVersaoSessoes(value=>value+1);
    }).catch(erro=>{
      if(!active)return;
      const mensagem=mensagemErro(erro);
      // Backend em monitor ao vivo (sem banco) responde 503: não é falha, só não há histórico.
      if(mensagem.includes("(503)")){setSessoes([]);setVersaoSessoes(value=>value+1);return;}
      setErroSessoes(mensagem);
    })
      .finally(()=>{if(active)setCarregandoSessoes(false);});
    return()=>{active=false;};
  },[diagnosticOnly,pedidoSessoes]);

  return (
    <>
      <div className={tela === "dashboard" ? "block" : "hidden"}>
        <Dashboard
          diagnosticOnly={diagnosticOnly}
          onAbrirHistorico={() => setTela("historico")}
          sessoes={sessoes}
          erroSessoes={erroSessoes}
          carregandoSessoes={carregandoSessoes}
          versaoSessoes={versaoSessoes}
          onAtualizarSessoes={atualizarSessoes}
        />
      </div>
      <div className={tela === "historico" ? "block" : "hidden"}>
        <Historico
          ativo={tela === "historico"}
          sessoes={sessoes}
          erroSessoes={erroSessoes}
          carregandoSessoes={carregandoSessoes}
          onAtualizarSessoes={atualizarSessoes}
          onVoltar={() => setTela("dashboard")}
          versaoSessoes={versaoSessoes}
        />
      </div>
    </>
  );
}

export default App;
