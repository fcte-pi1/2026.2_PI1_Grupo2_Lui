# _Frontend_

Esta pasta deverá armazenar arquivos referentes a:

- Código-fonte da interface: componentes, páginas, estilos e lógica de apresentação, organizados conforme o framework utilizado ([React](https://react.dev/), [Vue](https://vuejs.org/), [Angular](https://angular.io/) etc.).
- Arquivos de marcação e estilo estáticos: `index.html`, arquivos `.css`, `.scss` ou `.sass` de estilização global.
- Arquivos de definição de dependências: `package.json` e `package-lock.json` (ou `yarn.lock`) com todas as bibliotecas utilizadas.
- Arquivos de configuração do bundler/toolchain: `vite.config.js`, `webpack.config.js`, `tsconfig.json` etc.
- Arquivos de configuração de ambiente: `.env.example` com as variáveis de ambiente públicas necessárias (ex.: URL base da API).
- Arquivos de containerização: `Dockerfile` e `docker-compose.yml`, caso a aplicação seja servida via contêiner.

Evite incluir:

- Dependências instaladas: a pasta `node_modules/` deve ser gerada localmente via `npm install` ou equivalente e nunca incluída no repositório.
- Artefatos de build: diretórios como `dist/`, `build/` ou `.next/` são gerados pelo processo de compilação e não devem ser versionados.
- Arquivos de configuração pessoal: arquivos como `.directory` (Linux/KDE) ou configurações locais de editor (ex.: `.vscode/settings.json`), salvo configurações compartilhadas como `.editorconfig`.
- Credenciais e segredos: arquivos `.env` com valores reais nunca devem ser versionados.

> [!WARNING]
> **Não acrescente arquivos referentes ao _backend_ nesta pasta.** Eles deverão ser armazenados na pasta [backend](https://github.com/fcte-pi1/template/tree/main/src/backend) deste repositório.

## Painel Bluetooth real

Execute o backend em modo monitor, depois `npm run dev:bluetooth` nesta pasta. Abra `http://127.0.0.1:5173/?fonte=bluetooth` para ver as medidas reais da IMU, sequência e estado de atualização, sem simular bateria/mapa ou criar uma corrida.

[Configuração e guia completo](../backend/BLUETOOTH.md). O modo normal de corrida/histórico e a opção de simulação foram preservados.

## Iniciar e parar pelo dashboard original

Abra `/` na fonte Robô. O cartão **Missão no labirinto** permite selecionar tamanho, canto e orientação para a próxima missão. **Iniciar missão** depende de estado recente da ESP, firmware de navegação e liberação no firmware; **Parar robô** aguarda confirmação Bluetooth. Reconectar painel não inicia nem zera a ESP. A simulação continua separada e não apresenta os comandos físicos.

Enquanto a ESP estiver com BluetoothTelemetry, o início fica desabilitado e a parada pode ser testada com os motores já desativados. Consulte `../backend/BLUETOOTH.md` para preparação do firmware final e limites. O destino é o canto oposto ao início. A seleção para uma próxima missão não altera o mapa recebido da atual.
