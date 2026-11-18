#include "../src/relative_yaw.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
 DiagnosticYaw yaw;
 float value=0;
 for(int i=0;i<3000;++i) value=yaw.update(.06f,0,0,1,.02f);
 assert(yaw.stationary());assert(std::fabs(value)<.07f);
 const float before=value;
 for(int i=0;i<3000;++i) value=yaw.update(.09f,0,0,1,.02f);
 assert(value==before);
 for(int i=0;i<100;++i) value=yaw.update(45.09f,0,0,1,.02f);
 assert(!yaw.stationary());assert(std::fabs((value-before)-90)<.1f);
 for(int i=0;i<3000;++i) value=yaw.update(.09f,0,0,1,.02f);
 const float after=value;
 for(int i=0;i<1000;++i) value=yaw.update(.09f,0,0,1,.02f);
 assert(after==value);
 yaw.reset();
 for(int i=0;i<200;++i) value=yaw.update(.5f,0,0,1,.02f);
 assert(std::fabs(value-2)<.01f); assert(!yaw.stationary());
 value=yaw.update(std::numeric_limits<float>::quiet_NaN(),0,0,1,.02f);
 assert(std::isfinite(value));
 assert(yaw.update(100,0,0,1,2)==value);
 yaw.reset();
 for(int i=0;i<1000;++i) value=yaw.update(-90,0,0,1,.02f);
 assert(value>=-180 && value<180);
 std::cout<<"PASS: repouso, variacao de bias, giro 90, retorno ao repouso, giro lento, leitura invalida, pausa e wrap\n";
}
