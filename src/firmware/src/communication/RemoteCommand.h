#pragma once
#include <ArduinoJson.h>
#include <cstdint>
#include <cstring>

enum class RemoteAction { Start, Stop };
struct RemoteCommand {
    char id[33]{};
    RemoteAction action = RemoteAction::Stop;
    uint32_t bootId = 0;
    uint8_t width = 4, corner = 0, heading = 1;
};
// Shared wire validation. STOP deliberately needs no boot/config fields.
inline bool parseRemoteCommand(const char* line, RemoteCommand& output) {
    JsonDocument doc;
    if(deserializeJson(doc,line) || doc["type"]!="robot_command" || !doc["id"].is<const char*>())return false;
    const char* id=doc["id"];
    if(strlen(id)!=32)return false;
    for(size_t i=0;i<32;++i) {
        const char c=id[i];
        if(!((c>='0'&&c<='9')||(c>='a'&&c<='f')||(c>='A'&&c<='F')))return false;
    }
    RemoteCommand command;
    memcpy(command.id,id,33);
    if(doc["action"]=="stop"){output=command;return true;}
    if(doc["action"]!="start" || !doc["boot_id"].is<uint32_t>() ||
       !doc["width"].is<int>() || !doc["corner"].is<int>() || !doc["heading"].is<int>())return false;
    const int width=doc["width"],corner=doc["corner"],heading=doc["heading"];
    if((width!=4 && width!=8 && width!=12) || corner<0 || corner>3 || heading<0 || heading>3)return false;
    command.action=RemoteAction::Start;command.bootId=doc["boot_id"];
    command.width=width;command.corner=corner;command.heading=heading;
    output=command;return true;
}
