#include <Arduino.h>
#include <Preferences.h>
#include <esp_partition.h>
#include <esp_system.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "diagnostic_logger.hpp"

namespace {

constexpr char NVS_NAMESPACE[] = "nvstest";
constexpr char RECORD_KEY[] = "record";
constexpr char PHASE_KEY[] = "phase";
constexpr uint32_t RECORD_MAGIC = 0x4E565354UL; // "NVST"
constexpr uint16_t RECORD_VERSION = 1;
constexpr size_t PAYLOAD_LENGTH = 32;

// Estado persistente do roteiro. Esses valores ficam apenas no namespace de teste.
constexpr uint8_t PHASE_INITIAL = 0;
constexpr uint8_t PHASE_AFTER_SOFTWARE_REBOOT = 1;
constexpr uint8_t PHASE_WAITING_FOR_POWER_CYCLE = 2;
constexpr uint8_t PHASE_COMPLETE = 3;

struct PersistedRecord {
    uint32_t magic;
    uint16_t version;
    uint16_t payloadLength;
    uint32_t sequence;
    uint8_t payload[PAYLOAD_LENGTH];
    uint32_t checksum;
};

static_assert(sizeof(PersistedRecord) == 48, "PersistedRecord layout changed; update the diagnostic deliberately");

esp_reset_reason_t resetReason;
uint8_t currentPhase = PHASE_INITIAL;

const char* resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON: return "POWERON";
        case ESP_RST_EXT: return "EXT (pino de reset)";
        case ESP_RST_SW: return "SW (reinicializacao por software)";
        case ESP_RST_PANIC: return "PANIC";
        case ESP_RST_INT_WDT: return "INT_WDT";
        case ESP_RST_TASK_WDT: return "TASK_WDT";
        case ESP_RST_WDT: return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        default: return "OUTRO/INDEFINIDO";
    }
}

uint32_t checksumBytes(const uint8_t* bytes, size_t length) {
    // FNV-1a 32-bit: checksum simples para detectar escrita/leitura corrompida.
    uint32_t hash = 2166136261UL;
    for (size_t i = 0; i < length; ++i) {
        hash ^= bytes[i];
        hash *= 16777619UL;
    }
    return hash;
}

uint32_t calculateChecksum(const PersistedRecord& record) {
    return checksumBytes(reinterpret_cast<const uint8_t*>(&record), offsetof(PersistedRecord, checksum));
}

PersistedRecord makeRecord(uint32_t sequence) {
    PersistedRecord record{};
    record.magic = RECORD_MAGIC;
    record.version = RECORD_VERSION;
    record.payloadLength = PAYLOAD_LENGTH;
    record.sequence = sequence;

    for (size_t i = 0; i < PAYLOAD_LENGTH; ++i) {
        record.payload[i] = static_cast<uint8_t>((i * 37U + sequence * 19U) ^ 0xA5U);
    }

    record.checksum = calculateChecksum(record);
    return record;
}

bool recordIsValid(const PersistedRecord& record, uint32_t expectedSequence) {
    if (record.magic != RECORD_MAGIC || record.version != RECORD_VERSION ||
        record.payloadLength != PAYLOAD_LENGTH || record.sequence != expectedSequence) {
        return false;
    }

    const PersistedRecord expected = makeRecord(expectedSequence);
    return record.checksum == calculateChecksum(record) &&
           memcmp(record.payload, expected.payload, PAYLOAD_LENGTH) == 0;
}

bool saveRecordAndPhase(const PersistedRecord& record, uint8_t phase) {
    Preferences preferences;
    if (!preferences.begin(NVS_NAMESPACE, false)) {
        DiagnosticLog::log("ERROR", "Nao foi possivel abrir namespace '%s' para escrita.", NVS_NAMESPACE);
        return false;
    }

    const size_t recordBytes = preferences.putBytes(RECORD_KEY, &record, sizeof(record));
    if (recordBytes != sizeof(record)) {
        DiagnosticLog::log("ERROR", "Gravacao do registro falhou: bytes gravados=%u, esperado=%u.",
            static_cast<unsigned>(recordBytes), static_cast<unsigned>(sizeof(record)));
        preferences.end();
        return false;
    }

    const size_t phaseBytes = preferences.putUChar(PHASE_KEY, phase);
    if (phaseBytes != sizeof(uint8_t)) {
        DiagnosticLog::log("ERROR", "Gravacao da fase persistente falhou (retorno=%u).",
            static_cast<unsigned>(phaseBytes));
        preferences.end();
        return false;
    }

    preferences.end();
    return true;
}

bool savePhase(uint8_t phase) {
    Preferences preferences;
    if (!preferences.begin(NVS_NAMESPACE, false)) {
        DiagnosticLog::log("ERROR", "Nao foi possivel abrir namespace '%s' para atualizar fase.", NVS_NAMESPACE);
        return false;
    }
    const size_t written = preferences.putUChar(PHASE_KEY, phase);
    preferences.end();
    if (written != sizeof(uint8_t)) {
        DiagnosticLog::log("ERROR", "Atualizacao da fase %u falhou (retorno=%u).",
            phase, static_cast<unsigned>(written));
        return false;
    }
    return true;
}

bool readStoredData(uint8_t& phase, bool& phaseExists, PersistedRecord& record, bool& recordExists) {
    Preferences preferences;
    if (!preferences.begin(NVS_NAMESPACE, true)) {
        DiagnosticLog::log("ERROR", "Falha ao reabrir namespace '%s' somente para leitura.", NVS_NAMESPACE);
        return false;
    }

    phaseExists = preferences.isKey(PHASE_KEY);
    phase = phaseExists ? preferences.getUChar(PHASE_KEY, 0xFF) : PHASE_INITIAL;
    recordExists = preferences.isKey(RECORD_KEY);
    bool readOk = true;

    if (recordExists) {
        const size_t storedLength = preferences.getBytesLength(RECORD_KEY);
        if (storedLength != sizeof(record)) {
            DiagnosticLog::log("ERROR", "Tamanho do registro salvo incorreto: %u bytes; esperado=%u.",
                static_cast<unsigned>(storedLength), static_cast<unsigned>(sizeof(record)));
            readOk = false;
        } else {
            memset(&record, 0, sizeof(record));
            const size_t bytesRead = preferences.getBytes(RECORD_KEY, &record, sizeof(record));
            if (bytesRead != sizeof(record)) {
                DiagnosticLog::log("ERROR", "Leitura do registro incompleta: %u de %u bytes.",
                    static_cast<unsigned>(bytesRead), static_cast<unsigned>(sizeof(record)));
                readOk = false;
            }
        }
    }

    preferences.end();
    return readOk;
}

bool verifyRecordAndPhase(uint32_t expectedSequence, uint8_t expectedPhase, const char* context) {
    uint8_t phase = PHASE_INITIAL;
    bool phaseExists = false;
    bool recordExists = false;
    PersistedRecord record{};

    if (!readStoredData(phase, phaseExists, record, recordExists)) {
        DiagnosticLog::log("ERROR", "%s: leitura da NVS falhou.", context);
        return false;
    }
    if (!phaseExists || phase != expectedPhase) {
        DiagnosticLog::log("ERROR", "%s: fase persistida=%u (existe=%s), esperado=%u.",
            context, phase, phaseExists ? "sim" : "nao", expectedPhase);
        return false;
    }
    if (!recordExists) {
        DiagnosticLog::log("ERROR", "%s: chave '%s' nao existe na NVS.", context, RECORD_KEY);
        return false;
    }
    if (!recordIsValid(record, expectedSequence)) {
        DiagnosticLog::log("ERROR", "%s: registro invalido (magic=0x%08lX, versao=%u, sequencia=%lu, checksum salvo=0x%08lX, calculado=0x%08lX).",
            context, static_cast<unsigned long>(record.magic), record.version,
            static_cast<unsigned long>(record.sequence), static_cast<unsigned long>(record.checksum),
            static_cast<unsigned long>(calculateChecksum(record)));
        return false;
    }

    DiagnosticLog::log("PASS", "%s: registro de %u bytes integro; sequencia=%lu; checksum=0x%08lX; fase=%u.",
        context, static_cast<unsigned>(sizeof(record)), static_cast<unsigned long>(record.sequence),
        static_cast<unsigned long>(record.checksum), phase);
    return true;
}

bool runConnectionPrecheck(uint8_t& phase, bool& phaseExists) {
    DiagnosticLog::log("STAGE", "FASE 1 - PRE-CHECK NVS: verificar particao flash, abertura do namespace e leitura de metadados antes de gravar.");

    const esp_partition_t* partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, nullptr);
    if (partition == nullptr) {
        DiagnosticLog::log("ERROR", "Particao NVS padrao nao encontrada na tabela de particoes. Gravacao e diagnostico BLOQUEADOS.");
        return false;
    }
    DiagnosticLog::log("PASS", "Particao NVS encontrada: label=%s, offset=0x%08lX, tamanho=%lu bytes.",
        partition->label, static_cast<unsigned long>(partition->address),
        static_cast<unsigned long>(partition->size));

    Preferences preferences;
    if (!preferences.begin(NVS_NAMESPACE, false)) {
        DiagnosticLog::log("ERROR", "Nao foi possivel abrir NVS namespace '%s'. Diagnostico funcional BLOQUEADO.", NVS_NAMESPACE);
        DiagnosticLog::log("WARN", "Este teste NAO apaga nem formata a NVS. Verifique a tabela de particoes e os erros de inicializacao antes de qualquer recuperacao destrutiva.");
        return false;
    }

    phaseExists = preferences.isKey(PHASE_KEY);
    phase = phaseExists ? preferences.getUChar(PHASE_KEY, 0xFF) : PHASE_INITIAL;
    const bool recordExists = preferences.isKey(RECORD_KEY);
    const size_t freeEntries = preferences.freeEntries();
    preferences.end();

    if (phase > PHASE_COMPLETE) {
        DiagnosticLog::log("ERROR", "Estado persistente desconhecido (%u). Por seguranca, o teste nao vai gravar sobre esse estado.", phase);
        DiagnosticLog::log("ACTION", "Se deseja descartar somente os dados deste teste, envie 'r' e ENTER para limpar exclusivamente o namespace '%s'.", NVS_NAMESPACE);
        return false;
    }

    DiagnosticLog::log("PASS", "Namespace '%s' aberto e metadados consultados; phase=%u (%s), record=%s, entradas livres reportadas=%u.",
        NVS_NAMESPACE, phase, phaseExists ? "chave existente" : "primeira execucao",
        recordExists ? "presente" : "ausente", static_cast<unsigned>(freeEntries));
    DiagnosticLog::log("INFO", "Flash detectada: %lu bytes; sketch=%lu bytes; espaco livre para sketch=%lu bytes; heap livre=%u bytes.",
        static_cast<unsigned long>(ESP.getFlashChipSize()),
        static_cast<unsigned long>(ESP.getSketchSize()),
        static_cast<unsigned long>(ESP.getFreeSketchSpace()),
        static_cast<unsigned>(ESP.getFreeHeap()));
    return true;
}

void startFreshCycle() {
    DiagnosticLog::log("STAGE", "FASE 2 - gravacao de um registro de teste novo na NVS.");
    const PersistedRecord record = makeRecord(1);
    DiagnosticLog::log("INFO", "Registro: magic=0x%08lX, versao=%u, sequencia=1, payload=%u bytes, total=%u bytes.",
        static_cast<unsigned long>(record.magic), record.version,
        static_cast<unsigned>(PAYLOAD_LENGTH), static_cast<unsigned>(sizeof(record)));

    if (!saveRecordAndPhase(record, PHASE_AFTER_SOFTWARE_REBOOT)) {
        DiagnosticLog::log("ERROR", "Gravacao/commit do registro ou da fase falhou. Ciclo interrompido.");
        return;
    }
    DiagnosticLog::log("PASS", "putBytes() e putUChar() concluiram; namespace fechado para forcar a leitura em nova abertura.");
    if (!verifyRecordAndPhase(1, PHASE_AFTER_SOFTWARE_REBOOT, "Leitura imediata apos gravacao")) {
        DiagnosticLog::log("ERROR", "Verificacao de leitura imediata falhou; reinicializacao automatica cancelada.");
        return;
    }

    DiagnosticLog::log("ACTION", "Primeira fase gravada. A ESP32 sera reiniciada por software em 2 segundos para verificar a persistencia apos reboot.");
    delay(2000);
    ESP.restart();
}

void afterSoftwareReboot() {
    DiagnosticLog::log("STAGE", "FASE 3 - verificar os dados gravados antes da reinicializacao por software.");
    if (!verifyRecordAndPhase(1, PHASE_AFTER_SOFTWARE_REBOOT, "Persistencia apos reinicializacao por software")) {
        DiagnosticLog::log("ERROR", "Nao e seguro prosseguir para o teste de desligamento completo; revise o estado da NVS.");
        return;
    }
    if (resetReason == ESP_RST_SW) {
        DiagnosticLog::log("PASS", "Motivo de reset confirma reinicializacao por software (%s).", resetReasonName(resetReason));
    } else {
        DiagnosticLog::log("WARN", "O registro sobreviveu, mas o motivo de reset foi %s, nao SW. A verificacao dos dados passou; causa do reboot automatico nao confirmada.", resetReasonName(resetReason));
    }

    const PersistedRecord secondRecord = makeRecord(2);
    DiagnosticLog::log("STAGE", "FASE 4 - gravar segunda versao para validar a persistencia apos desligamento completo.");
    if (!saveRecordAndPhase(secondRecord, PHASE_WAITING_FOR_POWER_CYCLE)) {
        DiagnosticLog::log("ERROR", "Falha ao gravar a segunda versao do registro. Diagnostico interrompido.");
        return;
    }
    if (!verifyRecordAndPhase(2, PHASE_WAITING_FOR_POWER_CYCLE, "Leitura imediata da segunda versao")) {
        DiagnosticLog::log("ERROR", "Segunda versao nao passou na leitura imediata; nao execute o teste de power-cycle ate corrigir a falha.");
        return;
    }

    Serial.println();
    DiagnosticLog::log("ACTION", "DESLIGUE COMPLETAMENTE a ESP32: remova o USB e qualquer outra fonte que mantenha a placa alimentada.");
    DiagnosticLog::log("ACTION", "Aguarde pelo menos 5 segundos e ligue novamente. Depois, abra/reconecte o monitor serial a 115200 baud.");
    DiagnosticLog::log("INFO", "A proxima inicializacao vai procurar a fase 2 e validar os dados gravados na flash.");
}

void afterBootWithPowerCyclePending() {
    DiagnosticLog::log("STAGE", "FASE 5 - verificar o registro apos a segunda inicializacao.");
    if (!verifyRecordAndPhase(2, PHASE_WAITING_FOR_POWER_CYCLE, "Persistencia da segunda versao")) {
        DiagnosticLog::log("ERROR", "Registro nao persistiu corretamente. Teste de flash reprovado.");
        return;
    }

    bool powerCycleConfirmedByResetReason = (resetReason == ESP_RST_POWERON);
    if (!powerCycleConfirmedByResetReason) {
        DiagnosticLog::log("WARN", "Os dados persistem apos este reset (%s), mas o motivo de reset nao comprova sozinho o corte total de alimentacao.", resetReasonName(resetReason));
        if (!DiagnosticLog::answerYes("Voce realmente removeu TODAS as fontes de alimentacao da ESP32 por pelo menos 5 segundos antes deste boot?")) {
            DiagnosticLog::log("WARN", "Power-cycle fisico NAO confirmado. A fase 2 permanece salva; desligue todas as fontes, aguarde 5 segundos e ligue novamente.");
            return;
        }
        DiagnosticLog::log("INFO", "Power-cycle confirmado pelo operador. O reset reportado pode refletir reset adicional causado pela interface serial.");
    } else {
        DiagnosticLog::log("PASS", "Motivo de reset POWERON compativel com energizacao apos desligamento completo.");
    }

    if (!savePhase(PHASE_COMPLETE)) {
        DiagnosticLog::log("ERROR", "Registro persistiu apos o ciclo, mas nao foi possivel gravar o estado final do teste.");
        return;
    }
    if (!verifyRecordAndPhase(2, PHASE_COMPLETE, "Verificacao final apos power-cycle")) {
        DiagnosticLog::log("ERROR", "Falha na verificacao final do estado de teste.");
        return;
    }

    currentPhase = PHASE_COMPLETE;
    if (powerCycleConfirmedByResetReason) {
        DiagnosticLog::log("PASS", "TESTE NVS APROVADO: gravacao, leitura, integridade, reboot por software e persistencia apos desligamento/ligamento foram verificadas.");
    } else {
        DiagnosticLog::log("PASS", "TESTE NVS APROVADO com confirmacao manual do operador: gravacao, leitura, integridade e persistencia apos ciclo de desligamento/ligamento.");
    }
    DiagnosticLog::log("INFO", "Nenhuma particao foi formatada; somente as chaves '%s' e '%s' do namespace '%s' foram utilizadas.",
        RECORD_KEY, PHASE_KEY, NVS_NAMESPACE);
    DiagnosticLog::log("ACTION", "Para repetir o ciclo, envie 'r' e ENTER. Isso limpa apenas o namespace dedicado '%s'.", NVS_NAMESPACE);
}

void showCompletedState() {
    if (!verifyRecordAndPhase(2, PHASE_COMPLETE, "Registro do ultimo ciclo aprovado")) {
        DiagnosticLog::log("ERROR", "O estado marcado como concluido nao passou na verificacao. Nao declarar persistencia valida.");
        return;
    }
    DiagnosticLog::log("PASS", "O ultimo ciclo completo ja foi aprovado anteriormente. O registro continua presente na NVS.");
    DiagnosticLog::log("ACTION", "Envie 'r' e ENTER para apagar apenas os dados do namespace de teste e iniciar um novo ciclo.");
}

void handleRerunRequest() {
    if (!Serial.available()) return;
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (!command.equalsIgnoreCase("r")) {
        if (!command.isEmpty()) {
            DiagnosticLog::log("INFO", "Comando '%s' ignorado. Use 'r' para limpar somente o namespace de teste e reiniciar o ciclo.", command.c_str());
        }
        return;
    }

    Preferences preferences;
    if (!preferences.begin(NVS_NAMESPACE, false)) {
        DiagnosticLog::log("ERROR", "Nao foi possivel abrir namespace '%s' para reiniciar o ciclo.", NVS_NAMESPACE);
        return;
    }
    const bool clearOk = preferences.clear(); // limpa somente o namespace nvstest
    preferences.end();
    if (!clearOk) {
        DiagnosticLog::log("ERROR", "Falha ao limpar as chaves do namespace '%s'; nenhuma limpeza global foi solicitada.", NVS_NAMESPACE);
        return;
    }

    DiagnosticLog::log("PASS", "Somente o namespace '%s' foi limpo. Reiniciando para iniciar um ciclo novo.", NVS_NAMESPACE);
    delay(500);
    ESP.restart();
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(350);
    Serial.setTimeout(30000);

    resetReason = esp_reset_reason();
    DiagnosticLog::banner("NVS / memoria flash interna da ESP32");
    DiagnosticLog::log("INFO", "Motivo de reset interpretado: %s.", resetReasonName(resetReason));
    DiagnosticLog::log("INFO", "Area isolada do teste: namespace='%s', chaves='%s' e '%s'. Namespace de mapa 'maze' nao sera alterado.",
        NVS_NAMESPACE, RECORD_KEY, PHASE_KEY);

    uint8_t phase = PHASE_INITIAL;
    bool phaseExists = false;
    if (!runConnectionPrecheck(phase, phaseExists)) {
        currentPhase = phase;
        DiagnosticLog::log("ERROR", "Pre-verificacao NVS falhou. As etapas de gravacao/leitura foram bloqueadas.");
        return;
    }
    currentPhase = phaseExists ? phase : PHASE_INITIAL;

    switch (currentPhase) {
        case PHASE_INITIAL:
            startFreshCycle();
            break;
        case PHASE_AFTER_SOFTWARE_REBOOT:
            afterSoftwareReboot();
            break;
        case PHASE_WAITING_FOR_POWER_CYCLE:
            afterBootWithPowerCyclePending();
            break;
        case PHASE_COMPLETE:
            showCompletedState();
            break;
        default:
            DiagnosticLog::log("ERROR", "Fase invalida; diagnostico interrompido por seguranca.");
            break;
    }
}

void loop() {
    if (currentPhase == PHASE_COMPLETE || currentPhase > PHASE_COMPLETE) {
        handleRerunRequest();
    }
    delay(20);
}
