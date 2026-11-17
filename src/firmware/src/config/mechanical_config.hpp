#pragma once

#include <stdint.h>

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

// Centralizacao lateral pelas paredes (ultrassons esquerdo/direito).
// 0,12 grau/mm: 20 mm fora do centro pedem ~2,4 graus de rumo, limitado a 4.
constexpr float WALL_ALIGNMENT_KP_DEG_PER_MM = 0.12f;
constexpr float MAX_WALL_HEADING_DEG = 4.0f;
// Leitura lateral acima disto = sem parede adjacente (corredor de 168 mm).
constexpr float LATERAL_WALL_MAX_DISTANCE_MM = 100.0f;
// 0 = aprendido na largada (o beco tem as duas paredes) e durante o percurso.
constexpr float LATERAL_CENTER_DISTANCE_MM = 0.0f;
constexpr uint32_t WALL_FEEDBACK_MAX_AGE_MS = 100;

// Distancia do sensor frontal ate a parede com o robo no centro da celula.
// 0 DESATIVA a parada pela parede frontal. Para calibrar: centralizar o robo
// numa celula de frente para uma parede, enviar 's' no serial e copiar F (cm x 10).
// Deve ser >= 30 mm (alcance minimo confiavel do ultrassom).
constexpr float FRONT_WALL_CENTERED_DISTANCE_MM = 0.0f;
constexpr float FRONT_MAX_CORRECTION_MM = 30.0f;
}
