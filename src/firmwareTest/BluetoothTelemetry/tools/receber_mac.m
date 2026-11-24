#import <Foundation/Foundation.h>
#import <IOBluetooth/IOBluetooth.h>
#include <stdio.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>

@interface Receiver : NSObject
@property NSMutableData *pending;
@property NSString *token;
@property FILE *capture;
@property NSUInteger frames, validImu, bad, gaps, previous;
@property BOOL pong, closed, jsonOutput;
@end
@implementation Receiver
- (instancetype)init { if((self=[super init])) _pending=[NSMutableData data]; return self; }
- (void)rfcommChannelData:(IOBluetoothRFCOMMChannel*)channel data:(void*)bytes length:(size_t)length {
    [self.pending appendBytes:bytes length:length];
    if(self.pending.length>65536){self.bad++;[self.pending setLength:0];return;}
    for(;;){
        const uint8_t *buffer=self.pending.bytes;NSUInteger end=NSNotFound;
        for(NSUInteger i=0;i<self.pending.length;i++)if(buffer[i]=='\n'){end=i;break;}
        if(end==NSNotFound)break;
        NSData *line=[self.pending subdataWithRange:NSMakeRange(0,end)];
        [self.pending replaceBytesInRange:NSMakeRange(0,end+1) withBytes:NULL length:0];
        if(!line.length)continue;
        NSError *error=nil;id object=[NSJSONSerialization JSONObjectWithData:line options:0 error:&error];
        if(![object isKindOfClass:[NSDictionary class]]){self.bad++;continue;}
        NSDictionary *packet=object;
        if(self.jsonOutput){fwrite(line.bytes,1,line.length,stdout);fputc('\n',stdout);}
        if(self.capture){fwrite(line.bytes,1,line.length,self.capture);fputc('\n',self.capture);fflush(self.capture);}
        if([packet[@"type"] isEqual:@"pong"]&&[packet[@"token"] isEqual:self.token])self.pong=YES;
        if(![packet[@"type"] isEqual:@"bt_telemetry_test"])continue;
        if(!packet[@"motors_enabled"]||!packet[@"ultrasonic_enabled"]||[packet[@"motors_enabled"] boolValue]||[packet[@"ultrasonic_enabled"] boolValue]){self.bad++;continue;}
        NSUInteger seq=[packet[@"seq"] unsignedIntegerValue];
        if(self.frames&&seq<=self.previous){self.bad++;continue;}
        if(self.frames)self.gaps+=seq-self.previous-1;
        self.previous=seq;self.frames++;
        NSDictionary *imu=packet[@"imu"];if([imu[@"valid"] boolValue])self.validImu++;
        if(!self.jsonOutput&&(self.frames<=3||self.frames%10==0))
            printf("seq=%lu uptime=%s IMU=%s yaw=%s motores=OFF sonar=OFF\n",(unsigned long)seq,[[packet[@"uptime_ms"] description] UTF8String],[imu[@"valid"] boolValue]?"OK":"indisponivel",[[imu[@"yaw_deg"] description] UTF8String]);
    }
}
- (void)rfcommChannelClosed:(IOBluetoothRFCOMMChannel*)channel { self.closed=YES;fprintf(self.jsonOutput?stderr:stdout,"Canal Bluetooth encerrado.\n"); }
@end

// Only explicit JSON robot commands pass from stdin to RFCOMM. Never replay.
static void forwardCommands(IOBluetoothRFCOMMChannel *channel) {
    static char pending[513];static size_t length=0;static BOOL dropping=NO;
    char chunk[256];const ssize_t count=read(STDIN_FILENO,chunk,sizeof(chunk));
    for(ssize_t i=0;i<count;++i) {
        const char c=chunk[i];if(c=='\r')continue;
        if(c=='\n') {
            if(!dropping&&length) {
                NSData *data=[NSData dataWithBytes:pending length:length];
                id object=[NSJSONSerialization JSONObjectWithData:data options:0 error:nil];
                if([object isKindOfClass:[NSDictionary class]]&&[object[@"type"] isEqual:@"robot_command"]) {
                    pending[length]='\n';
                    const IOReturn status=[channel writeSync:pending length:(UInt16)(length+1)];
                    if(status!=kIOReturnSuccess)fprintf(stderr,"Falha no envio RFCOMM: %d\n",status);
                }
            }
            length=0;dropping=NO;
        } else if(!dropping) {
            if(length<sizeof(pending)-1)pending[length++]=c;
            else {length=0;dropping=YES;}
        }
    }
}

int main(int argc,const char *argv[]){@autoreleasepool{
    setvbuf(stdout,NULL,_IONBF,0);
    if(argc<3){fprintf(stderr,"Uso: receber_mac ENDERECO_ESP arquivo.jsonl [segundos]\n");return 2;}
    NSString *address=[NSString stringWithUTF8String:argv[1]];
    double duration=argc>3?atof(argv[3]):15;
    if(duration<0||duration>300){fprintf(stderr,"Duracao deve ficar entre0 e300s;0=continuo.\n");return 2;}
    IOBluetoothDevice *device=[IOBluetoothDevice deviceWithAddressString:address];
    if(!device||![device isPaired]){fprintf(stderr,"ESP nao pareada; conferir Ajustes do Sistema > Bluetooth.\n");return 2;}
    IOBluetoothSDPServiceRecord *service=[device getServiceRecordForUUID:[IOBluetoothSDPUUID uuid16:0x1101]];
    BluetoothRFCOMMChannelID channelID=0;
    if(!service||[service getRFCOMMChannelID:&channelID]!=kIOReturnSuccess){fprintf(stderr,"ServicoSPP nao encontrado no cadastro.\n");return 2;}
    Receiver *receiver=[Receiver new];
    receiver.jsonOutput=argc>4&&!strcmp(argv[4],"--json");
    receiver.token=[@"pi1-" stringByAppendingString:[[NSUUID UUID].UUIDString stringByReplacingOccurrencesOfString:@"-" withString:@""]];
    receiver.capture=fopen(argv[2],"w");if(!receiver.capture){perror("captura");return 2;}
    IOBluetoothRFCOMMChannel *channel=nil;
    fprintf(receiver.jsonOutput?stderr:stdout,"Abrindo canal Bluetooth RFCOMM %u da ESP...\n",channelID);
    IOReturn status=[device openRFCOMMChannelSync:&channel withChannelID:channelID delegate:receiver];
    if(status!=kIOReturnSuccess||!channel){fprintf(stderr,"FalhaRFCOMM status=%d\n",status);fclose(receiver.capture);return 2;}
    fprintf(receiver.jsonOutput?stderr:stdout,"Canal Bluetooth aberto; recebendo dados reais.\n");
    NSData *ping=[[NSString stringWithFormat:@"PING %@\n",receiver.token] dataUsingEncoding:NSUTF8StringEncoding];
    [channel writeSync:(void*)ping.bytes length:(UInt16)ping.length];
    NSDate *end=[NSDate dateWithTimeIntervalSinceNow:duration];
    if(receiver.jsonOutput)fcntl(STDIN_FILENO,F_SETFL,fcntl(STDIN_FILENO,F_GETFL)|O_NONBLOCK);
    while(!receiver.closed&&(duration==0||[end timeIntervalSinceNow]>0)) {
        if(receiver.jsonOutput)forwardCommands(channel);
        [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.05]];
    }
    fprintf(receiver.jsonOutput?stderr:stdout,"{\"frames\":%lu,\"imu_valid_frames\":%lu,\"invalid_frames\":%lu,\"sequence_gaps\":%lu,\"ping_pong_ok\":%s}\n",(unsigned long)receiver.frames,(unsigned long)receiver.validImu,(unsigned long)receiver.bad,(unsigned long)receiver.gaps,receiver.pong?"true":"false");
    const BOOL passed=receiver.frames>=8&&receiver.bad==0&&receiver.pong&&!receiver.closed;
    [channel closeChannel];fclose(receiver.capture);receiver.capture=NULL;
    return passed?0:1;
}}
