#pragma once
#include <cstdint>
constexpr int OUTPUT=1, INPUT=0, LOW=0, HIGH=1;
extern uint32_t fakeNow;
extern int fakePwm[64], fakeDigital[64];
inline uint32_t millis(){return fakeNow;}
inline void delay(uint32_t ms){fakeNow+=ms;}
inline void pinMode(uint8_t,int){}
inline void digitalWrite(uint8_t pin,int value){fakeDigital[pin]=value;}
inline bool ledcAttach(uint8_t,uint32_t,uint8_t){return true;}
inline void ledcWrite(uint8_t pin,uint32_t value){fakePwm[pin]=value;}

extern uint32_t fakeEchoPulse;
inline void delayMicroseconds(uint32_t){}
inline uint32_t pulseIn(uint8_t,int,uint32_t){return fakeEchoPulse;}
