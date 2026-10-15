import { useState } from 'react'
import Historico from './pages/Historico'

/**
 * Abas do sistema. Aqui só existe o Histórico (HU-12); a aba do Dashboard
 * entra quando a branch do dashboard (HU-09) for integrada.
 */
const ABAS = [{ id: 'historico', titulo: 'Histórico', pagina: Historico }]

function App() {
  const [abaAtiva, setAbaAtiva] = useState(ABAS[0].id)
  const Pagina = ABAS.find((a) => a.id === abaAtiva)!.pagina

  return (
    <div className="flex min-h-screen flex-col bg-[#0d0f14] text-gray-200">
      <header className="flex flex-wrap items-center justify-between gap-4 border-b border-white/10 px-6 py-4">
        <div>
          <h1 className="text-lg font-semibold text-gray-100">MicroMouse</h1>
          <p className="text-xs text-gray-500">Sistema Web</p>
        </div>
        <nav aria-label="Abas" className="flex gap-2">
          {ABAS.map((aba) => (
            <button
              key={aba.id}
              type="button"
              onClick={() => setAbaAtiva(aba.id)}
              aria-current={aba.id === abaAtiva ? 'page' : undefined}
              className={`rounded-full px-4 py-1.5 text-sm font-medium transition ${
                aba.id === abaAtiva
                  ? 'bg-orange-500 text-white'
                  : 'bg-white/5 text-gray-300 hover:bg-white/10'
              }`}
            >
              {aba.titulo}
            </button>
          ))}
        </nav>
      </header>
      <main className="flex min-h-0 flex-1 flex-col">
        <Pagina />
      </main>
    </div>
  )
}

export default App
