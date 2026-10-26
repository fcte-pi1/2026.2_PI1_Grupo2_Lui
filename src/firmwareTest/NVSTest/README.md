# NVSTest — gravação e persistência da flash/NVS da ESP32

Projeto PlatformIO independente para verificar se a NVS (Non-Volatile Storage), armazenada na memória flash interna da ESP32, aceita gravações, permite leitura de retorno e preserva um registro após reinicialização por software e desligamento completo da alimentação.

O repositório já utiliza NVS no firmware principal para persistir o mapa (`namespace "maze"`, chave `"map"`). Este teste usa exclusivamente o namespace `"nvstest"` e a chave `"record"`, sem apagar nem modificar os dados de produção. Ele não chama `nvs_flash_erase()` e não formata a partição NVS.

## O que é verificado

1. **Pré-verificação da NVS:** encontra a partição NVS padrão, registra tamanho/endereço da flash, abre o namespace exclusivo do teste e consegue consultar metadados antes de gravar.
2. **Gravação e leitura imediata:** grava um registro binário com identificador, versão, sequência, padrão de bytes e checksum; fecha a sessão e reabre a NVS para validar os bytes gravados.
3. **Persistência após reinicialização por software:** o primeiro registro é seguido por `ESP.restart()`. No boot seguinte, o teste lê e valida o registro persistido e grava uma segunda versão.
4. **Persistência após desligamento completo:** depois da segunda gravação, desligue completamente a ESP32, aguarde pelo menos 5 segundos e ligue-a novamente. O teste valida o registro e usa o motivo `POWERON` como evidência; se a interface serial causar um reset adicional, solicita confirmação manual explícita do operador.
5. **Integridade:** valida comprimento, campos, padrão dos bytes e checksum FNV-1a antes de indicar aprovação.

## Como executar

Na raiz do repositório:

```bash
cd src/firmwareTest/NVSTest
pio run
pio run --target upload --upload-port /dev/ttyUSB0
pio device monitor --port /dev/ttyUSB0 --baud 115200
```

Ajuste a porta serial conforme o dispositivo (`/dev/ttyACM0` ou `COMx`, por exemplo). O monitor serial usa 115200 baud.

## Procedimento físico necessário

- Depois do upload, mantenha o monitor serial aberto. O teste fará a primeira gravação e reiniciará a ESP32 automaticamente uma vez.
- Confira no log a aprovação da persistência após reinicialização por software e a solicitação para desligar a placa.
- Desconecte **todas** as fontes de alimentação da ESP32 (USB e qualquer alimentação externa que mantenha a placa energizada). Aguarde pelo menos 5 segundos e ligue-a novamente.
- Abra/reconecte o monitor serial, se necessário. A aprovação final exige que o registro da segunda versão e o estado do teste estejam íntegros após a energização.
- Se o motivo de reset indicar `POWERON`, a placa usa esse sinal como evidência do ciclo de energia. Se a interface serial tiver provocado um reset adicional e o motivo for outro, o programa pedirá confirmação manual. Responda `s` **somente** se realmente removeu todas as fontes por pelo menos 5 segundos; caso contrário, responda `n` e repita o desligamento completo.

Quando o ciclo já estiver aprovado, envie `r` seguido de ENTER pelo monitor serial para iniciar novamente. Se o estado de teste estiver inválido, o mesmo comando permite descartar esse estado dedicado. Em ambos os casos, ele limpa **somente o namespace `nvstest`**, reinicia a ESP32 e executa um ciclo novo. Ele não limpa a NVS inteira nem toca no namespace `maze`.

## Limites do teste

O teste comprova gravação/leitura e persistência lógica de um pequeno registro na NVS padrão; não mede desgaste da flash, desempenho máximo, tolerância a falha durante uma escrita interrompida nem valida a persistência do sistema de arquivos LittleFS usado para sessões. O teste deve ser gravado temporariamente, como os outros testes de bancada; ao terminar, grave novamente o firmware principal a partir de `src/firmware/`.
