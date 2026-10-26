#pragma once

#include <cstddef>

namespace mm {

/**
 * Interface para transporte de logs de eventos (HU-12).
 *
 * Implementada concretamente pelo módulo de comunicação (WebSocket/WiFi)
 * ou por um mock de testes. Não deve bloquear tarefas críticas de controle.
 */
class ILogTransport {
public:
    virtual ~ILogTransport() = default;

    /**
     * Envia o buffer serializado de log (JSON).
     * @param payload String JSON terminada em null.
     * @param length Tamanho do payload em bytes.
     * @return true se o envio foi enfileirado/executado com sucesso.
     */
    virtual bool sendLog(const char *payload, size_t length) = 0;
};

}  // namespace mm
