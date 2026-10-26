#pragma once

#include "communication/ILogTransport.h"
#include <string>
#include <vector>

namespace mm::test {

// Duplo de teste para ILogTransport: armazena as mensagens enviadas em memória
// para asserções durante os testes unitários.
class MockLogTransport : public ILogTransport {
public:
    std::vector<std::string> sentMessages;
    bool shouldFail = false;
    size_t callCount = 0;

    bool sendLog(const char *payload, size_t length) override {
        callCount++;
        if (shouldFail) {
            return false;
        }
        sentMessages.emplace_back(payload, length);
        return true;
    }

    void reset() {
        sentMessages.clear();
        shouldFail = false;
        callCount = 0;
    }
};

}  // namespace mm::test
