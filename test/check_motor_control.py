"""Host regression check of the actual motor/control functions, without powering hardware.
Run: python3 test/check_motor_control.py (requires a C++11 compiler).
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[1] / 'src/main.cpp').read_text()
functions = source[source.index('void writeDuty('):source.index('void safetyTask(')]
stubs = r'''
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <map>
#include <string>
#include <cstring>
class String : public std::string {
public:
  using std::string::string;
  String() = default;
  String(const std::string &s): std::string(s) {}
  long toInt() const { return std::strtol(c_str(), nullptr, 10); }
  String substring(size_t from, size_t to) const { return substr(from, to-from); }
  void replace(const char *from, const char *to) {
    size_t pos=0; while((pos=find(from,pos))!=npos){std::string::replace(pos,strlen(from),to);pos+=strlen(to);}
  }
};
struct Server {
  std::map<std::string,String> args; int code=0;
  String arg(const char *key){return args[key];}
  bool hasArg(const char *key){return args.count(key);}
  void sendHeader(const char *, const char *){}
  void send(int c,const char *,const char *){code=c;}
} server;
constexpr uint8_t A_IN1=10,A_IN2=9,B_IN1=21,B_IN2=20;
constexpr uint8_t PINS[]={A_IN1,A_IN2,B_IN1,B_IN2};
constexpr bool INVERT_A=false,INVERT_B=false;
constexpr int HIGH=1,LOW=0,OUTPUT=1,portMAX_DELAY=0;
uint32_t commandedDuty[4]={},pending[4]={},applied[4]={};
bool pwmAttached[4]={},physicalPwm[4]={},moving=false;
int requestedPower[2]={}; bool boostActive[2]={}; uint32_t boostStarted[2]={};
constexpr uint32_t START_BOOST_MS=180,COMMAND_TIMEOUT_MS=450;
uint32_t fakeNow=100;
int levels[22]={},motorMutex=0,writeCount=0;
uint32_t lastCommand=0;
void logMessage(const char *,...){}
void xSemaphoreTake(int,int){} void xSemaphoreGive(int){}
uint32_t millis(){return fakeNow;}
void ledcWrite(uint8_t c,uint32_t d){pending[c]=d;++writeCount;}
uint32_t ledcRead(uint8_t c){return applied[c];}
void ledcAttachPin(uint8_t p,uint8_t c){assert(p==PINS[c]);physicalPwm[c]=true;}
void ledcDetachPin(uint8_t p){for(int c=0;c<4;++c)if(PINS[c]==p)physicalPwm[c]=false;}
void pinMode(uint8_t,int){}
void digitalWrite(uint8_t p,int level){levels[p]=level;++writeCount;}
int digitalRead(uint8_t p){return levels[p];}
void delayMicroseconds(int us){assert(us>=100);for(int c=0;c<4;++c)applied[c]=pending[c];}
'''
# Keep the physical pin mapping under test in sync with the firmware.
pin_declaration = re.search(r'constexpr uint8_t A_IN1 =[^;]+;', source).group(0)
stubs = stubs.replace('constexpr uint8_t A_IN1=10,A_IN2=9,B_IN1=21,B_IN2=20;', pin_declaration)
checks = r''' 
void command(const char *dir,const char *speed="100",const char *target="both"){
  server.args={{"dir",dir},{"speed",speed},{"motor",target},{"source","test"}};
  control();
}
void settle(){fakeNow+=START_BOOST_MS;updateMotorTimers();delayMicroseconds(100);}
void expected(int a1,int a2,int b1,int b2){
  const int values[]={a1,a2,b1,b2};
  for(int c=0;c<4;++c){assert(commandedDuty[c]==(uint32_t)values[c]);assert(readDuty(c)==(uint32_t)values[c]);}
  assert(!(commandedDuty[0]&&commandedDuty[1]));
  assert(!(commandedDuty[2]&&commandedDuty[3]));
}
int main(){
  stopMotors();
  command("forward"); expected(255,0,255,0);
  for(int c=0;c<4;++c){assert(!physicalPwm[c]);assert(levels[PINS[c]]==(c%2==0?HIGH:LOW));}
  command("backward"); expected(0,255,0,255);
  command("left"); expected(0,255,255,0);
  command("right"); expected(255,0,0,255);
  command("forward","31"); settle(); expected(79,0,79,0);
  assert(physicalPwm[0]&&physicalPwm[2]);
  int count=writeCount;
  for(int i=0;i<100;++i)command("forward","31");
  assert(count==writeCount); // Heartbeats never reset or interrupt PWM.
  command("backward","31"); settle(); expected(0,79,0,79);
  command("left","31"); settle(); expected(0,79,79,0);
  command("right","31"); settle(); expected(79,0,0,79);
  command("forward","100","a"); expected(255,0,0,0);
  command("backward","100","a"); expected(0,255,0,0);
  command("forward","100","b"); expected(0,0,255,0);
  command("backward","100","b"); expected(0,0,0,255);
  command("stop"); expected(0,0,0,0); assert(!moving);
  command("forward","0"); expected(0,0,0,0); assert(!moving);
  command("forward","101"); assert(server.code==400); expected(0,0,0,0);
  command("forward","bad"); assert(server.code==400); expected(0,0,0,0);
  command("unknown"); assert(server.code==400); expected(0,0,0,0);
  command("forward","100","bad"); assert(server.code==400); expected(0,0,0,0);
  motor(0,255,true); expected(0,255,0,0);
  stopMotors();
  command("backward","40","a");expected(0,255,0,0);
  assert(boostActive[0]&&!boostActive[1]);
  const uint32_t started=boostStarted[0];
  fakeNow+=100;command("backward","40","a");
  assert(boostStarted[0]==started);expected(0,255,0,0);
  fakeNow+=80;updateMotorTimers();delayMicroseconds(100);expected(0,102,0,0);
  assert(!boostActive[0]);
  command("forward","40","a");assert(boostActive[0]);expected(255,0,0,0);
  command("stop");settle();expected(0,0,0,0);assert(!boostActive[0]);
  command("forward","40","b");fakeNow+=COMMAND_TIMEOUT_MS;
  assert(updateMotorTimers());expected(0,0,0,0);assert(!boostActive[1]);
  fakeNow=UINT32_MAX-50;command("backward","40","b");
  settle();expected(0,0,0,102);assert(!boostActive[1]);
  stopMotors();
  command("forward","40","a");fakeNow+=30;command("forward","55","a");
  settle();expected(140,0,0,0);
  puts("PASS: startup pulse duration, heartbeat does not extend pulse, stop cancellation, timeout, timer wraparound, speed adjustment;");
  puts("PASS: direction table, equal PWM, steady GPIO at 100%, PWM transitions, independent channels, stop, invalid commands");
}
'''
with tempfile.TemporaryDirectory(prefix='esp-car-motor-') as folder:
    cpp = Path(folder) / 'check.cpp'
    binary = Path(folder) / 'check'
    cpp.write_text(stubs + functions + checks)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
