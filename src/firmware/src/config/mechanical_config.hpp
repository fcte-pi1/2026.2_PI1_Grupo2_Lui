#pragma once

// Calibracao provisoria de bancada em 12/11/2026.
// Diametro e bitola medidos pelo usuario; contagens/volta estimadas por video.
// Validar no piso antes de considerar estes valores definitivos.
namespace MechanicalSettings {
constexpr float kPi = 3.14159265358979323846f;
constexpr float WHEEL_DIAMETER_MM = 34.0f;
constexpr float WHEEL_TRACK_MM = 120.0f;
constexpr float LEFT_COUNTS_PER_REVOLUTION = 292.0f;
constexpr float RIGHT_COUNTS_PER_REVOLUTION = 284.0f;
constexpr float LEFT_COUNTS_PER_MM = LEFT_COUNTS_PER_REVOLUTION / (kPi * WHEEL_DIAMETER_MM);
constexpr float RIGHT_COUNTS_PER_MM = RIGHT_COUNTS_PER_REVOLUTION / (kPi * WHEEL_DIAMETER_MM);
}
