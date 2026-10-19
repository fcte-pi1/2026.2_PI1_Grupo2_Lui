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
  const [tela, setTela] = useState<Tela>("dashboard");
  const [sessoes, setSessoes] = useState<Sessao[] | null>(null);
  const [erroSessoes, setErroSessoes] = useState<string | null>(null);
  const [carregandoSessoes, setCarregandoSessoes] = useState(true);
  const [versaoSessoes, setVersaoSessoes] = useState(0);

  const atualizarSessoes = useCallback(async () => {
    setCarregandoSessoes(true);
    setErroSessoes(null);
    try {
      const dados = await getSessions();
      setSessoes(dados);
      setVersaoSessoes((versao) => versao + 1);
    } catch (erro) {
      setErroSessoes(mensagemErro(erro));
    } finally {
      setCarregandoSessoes(false);
    }
  }, []);

  useEffect(() => {
    void atualizarSessoes();
  }, [atualizarSessoes]);

  return (
    <>
      <div className={tela === "dashboard" ? "block" : "hidden"}>
        <Dashboard
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
