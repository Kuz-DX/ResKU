#include "protocol2_packet_handler.h"
#include <vector>
#include <cstring>
#include <algorithm>
#include <cassert>
#include <cstdio>
using namespace dynamixel;
struct Fake:PortHandler {
 std::vector<uint8_t> bytes; size_t offset=0; bool fail=false; int chunk=1024, idle_reads=0;
 Fake(){is_using_=false;}
 bool openPort(){return true;} void closePort(){} void clearPort(){}
 void setPortName(const char*){} char* getPortName(){return nullptr;}
 bool setBaudRate(int){return true;} int getBaudRate(){return 1000000;}
 int getBytesAvailable(){return bytes.size()-offset;}
 int readPort(uint8_t*p,int n){if(fail)return -1; if(idle_reads>0){--idle_reads;return 0;} assert(n>=0); n=std::min(n,chunk);int k=std::min(size_t(n),bytes.size()-offset);if(k)memcpy(p,bytes.data()+offset,k);offset+=k;return k;}
 int writePort(uint8_t*,int n){return n;}
 void setPacketTimeout(uint16_t){} void setPacketTimeout(double){}
 bool isPacketTimeout(){return offset==bytes.size();}
};
std::vector<uint8_t> packet(int length){std::vector<uint8_t> b(length+7,0);b[0]=255;b[1]=255;b[2]=253;b[4]=4;b[5]=length&255;b[6]=length>>8;b[7]=85;uint16_t crc=0;for(size_t i=0;i<b.size()-2;++i){crc^=uint16_t(b[i])<<8;for(int j=0;j<8;++j)crc=(crc&0x8000)?(crc<<1)^0x8005:crc<<1;}b[b.size()-2]=crc&255;b.back()=crc>>8;return b;}
int main(){auto h=Protocol2PacketHandler::getInstance();uint8_t error=0;
 for(int len:{4,40,1017,1018,1024}){Fake p;p.bytes=packet(len);int r=h->write1ByteTxRx(&p,4,64,0,&error);assert(r==(len==4?COMM_SUCCESS:COMM_RX_CORRUPT));}
 for(int len:{0,1,2,3}){Fake p;p.bytes=packet(4);p.bytes[5]=len;assert(h->write1ByteTxRx(&p,4,64,0,&error)!=COMM_SUCCESS);}
 Fake p;p.fail=true;assert(h->write1ByteTxRx(&p,4,64,0,&error)==COMM_RX_FAIL);

 int cases=10;
 for(int chunk:{1,3,1024}) {
   Fake ping;ping.chunk=chunk;ping.idle_reads=3;ping.bytes=packet(7);uint16_t model=65535;
   assert(h->ping(&ping,4,&model,&error)==COMM_SUCCESS);assert(model==0);++cases;
   Fake read;read.chunk=chunk;read.idle_reads=2;read.bytes=packet(8);uint32_t value=123;
   assert(h->read4ByteTxRx(&read,4,132,&value,&error)==COMM_SUCCESS);assert(value==0);++cases;
 }
 for(int len:{4,5,6,8}) {Fake p;p.bytes=packet(len);uint16_t model=123;
   assert(h->ping(&p,4,&model,&error)==COMM_RX_CORRUPT);assert(model==123);++cases;}
 for(int len:{4,5,7,9}) {Fake p;p.bytes=packet(len);uint32_t value=123;
   assert(h->read4ByteTxRx(&p,4,132,&value,&error)==COMM_RX_CORRUPT);++cases;}
 {Fake p;p.bytes=packet(4);p.bytes.back()^=1;assert(h->write1ByteTxRx(&p,4,64,0,&error)==COMM_RX_CORRUPT);++cases;}
 {Fake p;p.bytes=packet(4);p.bytes.resize(9);assert(h->write1ByteTxRx(&p,4,64,0,&error)==COMM_RX_CORRUPT);++cases;}
 {Fake p;assert(h->write1ByteTxRx(&p,4,64,0,&error)==COMM_RX_TIMEOUT);++cases;}
 {Fake p;p.bytes=packet(8);uint8_t data[4]={99,99,99,99};assert(h->readRx(&p,4,4,data,&error)==COMM_SUCCESS);assert(data[0]==0);++cases;}
 {Fake p;p.bytes=packet(4);uint8_t data[4]={99,99,99,99};assert(h->readRx(&p,4,4,data,&error)==COMM_RX_CORRUPT);assert(data[0]==99);++cases;}
 printf("%d packet/error cases passed\n",cases);}
