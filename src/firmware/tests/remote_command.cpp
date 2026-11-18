#include "../src/communication/RemoteCommand.h"
#include <cassert>
#include <iostream>
int main(){
 RemoteCommand c;
 const char* valid=R"({"type":"robot_command","id":"0123456789abcdef0123456789abcdef","action":"start","boot_id":123,"width":4,"corner":0,"heading":1})";
 assert(parseRemoteCommand(valid,c));assert(c.action==RemoteAction::Start&&c.width==4&&c.bootId==123);
 for(int width:{4,8,12}) {
  std::string text=valid;const auto pos=text.find("\"width\":4");text.replace(pos,9,"\"width\":"+std::to_string(width));
  assert(parseRemoteCommand(text.c_str(),c));assert(c.width==width);
 }
 assert(parseRemoteCommand(R"({"type":"robot_command","id":"0123456789abcdef0123456789abcdef","action":"stop"})",c));
 assert(c.action==RemoteAction::Stop);
 for(const char* invalid:{"bad","{}",R"({"type":"robot_command","id":"abc","action":"stop"})",R"({"type":"robot_command","id":"0123456789abcdef0123456789abcdef","action":"restart"})",R"({"type":"robot_command","id":"0123456789abcdef0123456789abcdef","action":"start","boot_id":123,"width":5,"corner":0,"heading":1})",R"({"type":"robot_command","id":"0123456789abcdef0123456789abcdef","action":"start","boot_id":123,"width":4,"corner":-1,"heading":1})",R"({"type":"robot_command","id":"0123456789abcdef0123456789abcdef","action":"start","boot_id":123,"width":4,"corner":0,"heading":4})"})assert(!parseRemoteCommand(invalid,c));
 std::cout<<"PASS: parser compartilhado, tres formatos, parada sem configuracao e rejeicao de mensagens invalidas\n";
}
