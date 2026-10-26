// Reutiliza a implementação ultrassônica já existente no firmware principal.
// O protocolo Trigger/Echo usado pelo HC-SR04 é compatível com o pulso enviado
// por UltrasonicUS100; neste projeto de teste, a faixa fica limitada a 2-400 cm.
// Manter a implementação original evita duplicar o driver do componente.
#include "../../../firmware/src/hal/ultrasonic_us100.cpp"
