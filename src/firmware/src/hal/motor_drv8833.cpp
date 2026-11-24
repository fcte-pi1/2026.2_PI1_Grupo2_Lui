#include "motor_drv8833.hpp"

bool MotorDRV8833::begin(uint8_t l1, uint8_t l2, uint8_t r1, uint8_t r2,
    uint8_t sleep, uint32_t hz, uint8_t bits, bool il, bool ir) {
    if (!hz || bits != 8) return false;
    _leftIn1=l1; _leftIn2=l2; _rightIn1=r1; _rightIn2=r2; _nSleepPin=sleep;
    _pwmFrequencyHz=hz; _pwmResolutionBits=bits; _maxDuty=255; _invertLeft=il; _invertRight=ir;
    pinMode(sleep, OUTPUT); digitalWrite(sleep, LOW);
    pinMode(l2, OUTPUT); pinMode(r2, OUTPUT); digitalWrite(l2, LOW); digitalWrite(r2, LOW);
    _leftAttached=ledcAttach(l1, hz, bits); _rightAttached=ledcAttach(r1, hz, bits);
    _initialized=_leftAttached && _rightAttached;
    setEnabled(false);
    return _initialized;
}
void MotorDRV8833::setEnabled(bool enabled) {
    if (!enabled) {
        stop(); digitalWrite(_nSleepPin, LOW); _enabled=false; return;
    }
    if (!_initialized) return;
    if (_enabled) return;
    digitalWrite(_nSleepPin, HIGH); delay(10); _enabled=true;
}
bool MotorDRV8833::isEnabled() const { return _enabled; }
void MotorDRV8833::writeMotor(uint8_t pwm, uint8_t dir, int16_t speed, bool invert) {
    int32_t value=speed;
    if (value>255) value=255; if (value< -255) value= -255;
    if (invert) value= -value;
    digitalWrite(dir, value < 0 ? HIGH : LOW);
    ledcWrite(pwm, value < 0 ? 255+value : value);
}
void MotorDRV8833::setLeftSpeed(int16_t value) { if (_initialized && _enabled) writeMotor(_leftIn1,_leftIn2,value,_invertLeft); }
void MotorDRV8833::setRightSpeed(int16_t value) { if (_initialized && _enabled) writeMotor(_rightIn1,_rightIn2,value,_invertRight); }
void MotorDRV8833::setSpeeds(int16_t l, int16_t r) { setControlledSpeeds(l,r,false,false); }
void MotorDRV8833::setControlledSpeeds(int16_t l,int16_t r,bool bl,bool br) {
    if (!_initialized || !_enabled) return;
    if (bl) { ledcWrite(_leftIn1,255); digitalWrite(_leftIn2,HIGH); } else setLeftSpeed(l);
    if (br) { ledcWrite(_rightIn1,255); digitalWrite(_rightIn2,HIGH); } else setRightSpeed(r);
}
void MotorDRV8833::stop() {
    if (_leftAttached) ledcWrite(_leftIn1,0);
    if (_rightAttached) ledcWrite(_rightIn1,0);
    digitalWrite(_leftIn2,LOW); digitalWrite(_rightIn2,LOW);
}
void MotorDRV8833::brake() { setControlledSpeeds(0,0,true,true); }
