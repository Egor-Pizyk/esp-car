#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>
#include <stdarg.h>
#include "secrets.h"

#if !ARDUINO_USB_CDC_ON_BOOT || !ARDUINO_USB_MODE
#error "Motor GPIO20/21 require native USB CDC logging, not UART Serial"
#endif

// GPIO numbers, not physical header pin numbers.
// Current wiring after the diagnostic pair swap: labels follow physical driver A/B.
constexpr uint8_t A_IN1 = 21, A_IN2 = 20, B_IN1 = 10, B_IN2 = 9;
// Assumption: A is the left wheel, B is the right wheel.
constexpr bool INVERT_A = false, INVERT_B = false;

// Workaround for WiFi authentication failures on some ESP32-C3 boards.
// https://github.com/espressif/arduino-esp32/issues/6551
constexpr wifi_power_t WIFI_TX_POWER = WIFI_POWER_8_5dBm;
constexpr uint32_t COMMAND_TIMEOUT_MS = 450;
constexpr uint32_t START_BOOST_MS = 180;
constexpr uint8_t PINS[] = {A_IN1, A_IN2, B_IN1, B_IN2};
WebServer server(80);
uint32_t lastCommand = 0;
bool moving = false;
uint32_t commandedDuty[4] = {};
bool pwmAttached[4] = {};
int requestedPower[2] = {};
bool boostActive[2] = {};
uint32_t boostStarted[2] = {};
SemaphoreHandle_t motorMutex;

// Drop logs when USB is absent/full: diagnostics must not delay motor safety.
void logMessage(const char *format, ...) {
  if (!Serial) return;
  char line[256];
  int prefix = snprintf(line, sizeof(line), "[%lu ms] ", (unsigned long)millis());
  va_list args;
  va_start(args, format);
  vsnprintf(line + prefix, sizeof(line) - prefix - 2, format, args);
  va_end(args);
  size_t length = strlen(line);
  line[length++] = '\n';
  if (Serial.availableForWrite() >= (int)length)
    Serial.write(reinterpret_cast<const uint8_t *>(line), length);
}

const char *wifiStatusName(wl_status_t status) {
  switch (status) {
    case WL_CONNECTED: return "connected";
    case WL_NO_SSID_AVAIL: return "network not found";
    case WL_CONNECT_FAILED: return "connection failed";
    case WL_CONNECTION_LOST: return "connection lost";
    case WL_DISCONNECTED: return "disconnected";
    case WL_IDLE_STATUS: return "connecting";
    default: return "not ready";
  }
}

const char *wifiReasonName(uint8_t reason) {
  switch (reason) {
    case 2: return "AUTH_EXPIRE";
    case 202: return "AUTH_FAIL";
    case 205: return "CONNECTION_FAIL";
    default: return "OTHER";
  }
}

void logNetwork() {
  const wl_status_t status = WiFi.status();
  logMessage("[WiFi] SSID=%s | %s (status=%d) | TX=%.2f dBm",
             WIFI_SSID, wifiStatusName(status), (int)status, (int)WiFi.getTxPower() / 4.0);
  if (status == WL_CONNECTED) {
    logMessage("[HTTP] Open http://%s/ | RSSI=%d dBm | channel=%d",
               WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.channel());
  }
  logMessage("[BUILD] %s %s | STA MAC=%s", __DATE__, __TIME__, WiFi.macAddress().c_str());
}

void fatalError(const char *message) {
  // Repeat so the error remains visible when Monitor is opened after boot.
  for (;;) {
    logMessage("[ERROR] %s", message);
    delay(2000);
  }
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ESP Car</title><style>
*{box-sizing:border-box}body{margin:0;background:#101827;color:#edf4ff;font:18px system-ui;text-align:center}main{max-width:440px;margin:auto;padding:24px}h1{margin-bottom:8px}.hint{color:#b6c6df;font-size:15px}.joystick{position:relative;width:260px;height:260px;margin:28px auto;border:2px solid #405775;border-radius:50%;background:radial-gradient(circle,#203550,#162338);touch-action:none;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none}.joystick::before,.joystick::after{content:"";position:absolute;background:#405775;pointer-events:none}.joystick::before{width:1px;height:100%;left:50%;top:0;transform:translateX(-50%)}.joystick::after{height:1px;width:100%;top:50%;left:0;transform:translateY(-50%)}.knob{position:absolute;left:50%;top:50%;width:86px;height:86px;margin:-43px;border-radius:50%;background:#4f719b;border:3px solid #91b9e7;box-shadow:0 6px 18px #0008;pointer-events:none;z-index:1}.joystick.active .knob{background:#178bd0}.joystick .up,.joystick .down,.joystick .left,.joystick .right{position:absolute;width:0;height:0;font-size:0;border:8px solid transparent;pointer-events:none}.joystick .up{top:8px;left:50%;transform:translateX(-50%);border-top:0;border-bottom-color:#b6c6df}.joystick .down{bottom:8px;left:50%;transform:translateX(-50%);border-bottom:0;border-top-color:#b6c6df}.joystick .left{left:10px;top:50%;transform:translateY(-50%);border-left:0;border-right-color:#b6c6df}.joystick .right{right:10px;top:50%;transform:translateY(-50%);border-right:0;border-left-color:#b6c6df}button{height:90px;border:0;border-radius:20px;background:#273c5a;color:white;font-size:38px;touch-action:none;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none}button.active{background:#178bd0}#stop{background:#af3548;font-size:19px;width:100%;height:56px}input[type=range]{width:100%;margin:20px 0}.mode{display:block;font-size:16px;margin:12px 0}.mode input{width:auto;margin-right:8px}select{font:inherit;color:inherit;background:#273c5a;border:0;border-radius:8px;padding:8px;margin:8px}#status{min-height:28px;font-size:15px}
</style><main><h1>ESP Car</h1><div class="hint">Отклоните стик в нужную сторону</div>
<div id="joystick" class="joystick" role="group" aria-label="Джойстик: перетащите стик для движения; также доступны стрелки клавиатуры"><span class="up">▲</span><span class="down">▼</span><span class="left">◀</span><span class="right">▶</span><div class="knob" id="knob"></div></div>
<button id="stop">СТОП</button>
<label for="motorSelection">Моторы:</label><select id="motorSelection"><option value="both">Оба</option><option value="a">Только A (AO1/AO2)</option><option value="b">Только B (BO1/BO2)</option></select><br>
<label for="speed">Скорость: <b id="value">60</b>%</label><input id="speed" type="range" min="0" max="100" value="60">
<label class="mode"><input id="latch" type="checkbox">Фиксировать направление после отпускания</label>
<p class="hint">Старт: импульс 100% на 180 мс, затем выбранная мощность.</p>
<div id="status" role="status">Готово</div><p class="hint">При потере команд моторы отключаются.<br>Можно использовать стрелки клавиатуры; пробел — стоп.</p></main>
<script>
const speed=document.querySelector('#speed'),status=document.querySelector('#status'),latch=document.querySelector('#latch'),motorSelection=document.querySelector('#motorSelection');
let direction='stop',generation=0,busy=false,controller=null,activePointer=null,activeKey=null,inputSource='initial';
const joystick=document.querySelector('#joystick'),knob=document.querySelector('#knob');
let stickX=0,stickY=0;
function paint(){
 const positions={stop:[0,0],forward:[0,-82],backward:[0,82],left:[-82,0],right:[82,0]};
 const [x,y]=activePointer!==null?[stickX,stickY]:positions[direction];
 knob.style.transform=`translate(${x}px,${y}px)`;
 joystick.classList.toggle('active',direction!=='stop');
}
async function send(){
 if(busy)return;
 busy=true;const gen=generation,dir=direction,target=motorSelection.value,percent=speed.value;
 const request=new AbortController();controller=request;
 const timeout=setTimeout(()=>request.abort(),300);
 try{const r=await fetch('/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({dir,speed:percent,source:inputSource,motor:target}),signal:request.signal,cache:'no-store'});if(!r.ok)throw Error();if(gen===generation)status.textContent=dir==='stop'?'Команда СТОП принята':'Команда принята: '+({both:'оба',a:'A',b:'B'}[target])+', '+({forward:'вперёд',backward:'назад',left:'влево',right:'вправо'}[dir])+', '+percent+'%';}
 catch(e){if(gen===generation){if(direction!=='stop'){direction='stop';inputSource='network-error';generation++;}activePointer=null;activeKey=null;paint();status.textContent='Нет связи — нажмите направление заново';}}
 finally{clearTimeout(timeout);if(controller===request){controller=null;busy=false;}if(gen!==generation)send();}
}
function setDirection(dir,source='direction'){inputSource=source;generation++;direction=dir;paint();status.textContent='Отправка команды…';if(controller){controller.abort();}else send();}
function stop(source='stop'){activePointer=null;activeKey=null;setDirection('stop',source);}
function moveStick(e){
 const rect=joystick.getBoundingClientRect(),radius=rect.width/2-45;
 let x=e.clientX-rect.left-rect.width/2,y=e.clientY-rect.top-rect.height/2;
 const length=Math.hypot(x,y),scale=length>radius?radius/length:1;
 stickX=x*scale;stickY=y*scale;
 const dir=length<radius*0.2?'stop':Math.abs(x)>Math.abs(y)?(x<0?'left':'right'):(y<0?'forward':'backward');
 if(dir!==direction)setDirection(dir,'joystick');else paint();
}
joystick.addEventListener('pointerdown',e=>{
 if(e.pointerType==='mouse'&&e.button!==0)return;
 e.preventDefault();if(activePointer!==null)return;
 activePointer=e.pointerId;activeKey=null;
 try{joystick.setPointerCapture(e.pointerId);}catch(e){}
 moveStick(e);
});
addEventListener('pointermove',e=>{if(activePointer===e.pointerId){e.preventDefault();moveStick(e);}},{passive:false});
function releaseStick(e){
 if(activePointer!==e.pointerId)return;
 activePointer=null;
 if(e.type==='pointerup'&&latch.checked){paint();return;}
 stop(e.type);
}
for(const event of ['pointerup','pointercancel'])addEventListener(event,releaseStick);
joystick.addEventListener('lostpointercapture',releaseStick);
joystick.addEventListener('contextmenu',e=>e.preventDefault());
joystick.addEventListener('dragstart',e=>e.preventDefault());
document.querySelector('#stop').addEventListener('pointerdown',()=>stop('stop-button'));
document.querySelector('#stop').addEventListener('click',()=>stop('stop-button'));
latch.addEventListener('change',()=>stop('mode-change'));
motorSelection.addEventListener('change',()=>stop('motor-change'));
speed.addEventListener('input',()=>{document.querySelector('#value').textContent=speed.value;});
const keys={ArrowUp:'forward',ArrowDown:'backward',ArrowLeft:'left',ArrowRight:'right'};
addEventListener('keydown',e=>{
 if(keys[e.key]||e.code==='Space'){
  e.preventDefault();if(e.repeat)return;
  if(e.code==='Space'){stop('space');return;}
  if(latch.checked){setDirection(direction===keys[e.key]?'stop':keys[e.key],'latch-key');return;}
  activePointer=null;activeKey=e.key;setDirection(keys[e.key],'keydown');
 }
});
addEventListener('keyup',e=>{if(keys[e.key]){e.preventDefault();if(activeKey===e.key)stop('keyup');}});
addEventListener('blur',()=>stop('blur'));document.addEventListener('visibilitychange',()=>{if(document.hidden)stop('hidden');});
addEventListener('pagehide',()=>{navigator.sendBeacon('/control',new URLSearchParams({dir:'stop',speed:'0',source:'pagehide'}));});
setInterval(()=>{if(direction!=='stop')send();},100);
</script></html>)HTML";

void writeDuty(uint8_t channel, uint32_t duty) {
  // At the endpoints bypass LEDC completely: full power is steady HIGH/LOW.
  if (duty == 0 || duty == 255) {
    if (pwmAttached[channel]) {
      ledcDetachPin(PINS[channel]);
      pwmAttached[channel] = false;
    }
    digitalWrite(PINS[channel], duty == 255 ? HIGH : LOW);
    pinMode(PINS[channel], OUTPUT);
  } else {
    ledcWrite(channel, duty);
    if (!pwmAttached[channel]) {
      // ledcAttachPin reads current duty; let the new value reach the hardware first.
      delayMicroseconds(100);
      ledcAttachPin(PINS[channel], channel);
      pwmAttached[channel] = true;
    }
  }
  commandedDuty[channel] = duty;
}

uint32_t readDuty(uint8_t channel) {
  if (!pwmAttached[channel]) return digitalRead(PINS[channel]) == HIGH ? 255 : 0;
  const uint32_t duty = ledcRead(channel);
  return duty > 255 ? 255 : duty;
}

void stopMotors() {
  for (uint8_t index = 0; index < 2; ++index) {
    requestedPower[index] = 0;
    boostActive[index] = false;
  }
  for (uint8_t channel = 0; channel < 4; ++channel) writeDuty(channel, 0);
  moving = false;
}

void applyMotorPower(uint8_t firstChannel, int power) {
  const uint32_t forward = power > 0 ? power : 0;
  const uint32_t backward = power < 0 ? -power : 0;
  if (commandedDuty[firstChannel] == forward &&
      commandedDuty[firstChannel + 1] == backward) return;
  const bool reversing = (commandedDuty[firstChannel] && backward) ||
                         (commandedDuty[firstChannel + 1] && forward);
  if (reversing) {
    writeDuty(firstChannel, 0);
    writeDuty(firstChannel + 1, 0);
    // LEDC applies changes on the next cycle (20 kHz = 50 us).
    delayMicroseconds(100);
  }
  if (commandedDuty[firstChannel] != forward) writeDuty(firstChannel, forward);
  if (commandedDuty[firstChannel + 1] != backward) writeDuty(firstChannel + 1, backward);
}

void motor(uint8_t firstChannel, int power, bool invert) {
  if (invert) power = -power;
  const uint8_t index = firstChannel / 2;
  const int previous = requestedPower[index];
  const bool starting = power != 0 &&
      (previous == 0 || (previous < 0) != (power < 0));
  requestedPower[index] = power;
  if (power == 0 || power == 255 || power == -255) {
    boostActive[index] = false;
  } else if (starting && START_BOOST_MS > 0) {
    boostActive[index] = true;
    boostStarted[index] = millis();
  }
  // Heartbeats and speed adjustments never extend an existing pulse.
  if (boostActive[index] && millis() - boostStarted[index] >= START_BOOST_MS)
    boostActive[index] = false;
  const int output = boostActive[index] ? (power > 0 ? 255 : -255) : power;
  applyMotorPower(firstChannel, output);
}

bool updateMotorTimers() {
  // Stop takes precedence over pulse completion; timers cannot restart stopped motors.
  if (moving && millis() - lastCommand >= COMMAND_TIMEOUT_MS) {
    stopMotors();
    return true;
  }
  for (uint8_t index = 0; index < 2; ++index) {
    if (boostActive[index] && millis() - boostStarted[index] >= START_BOOST_MS) {
      boostActive[index] = false;
      applyMotorPower(index * 2, requestedPower[index]);
    }
  }
  return false;
}

void control() {
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  const String dir = server.arg("dir");
  const String target = server.hasArg("motor") ? server.arg("motor") : "both";
  if (dir == "stop") {
    stopMotors();
  } else {
    const String speed = server.arg("speed");
    bool validSpeed = speed.length() > 0 && speed.length() <= 3;
    for (size_t i = 0; i < speed.length(); ++i)
      if (speed[i] < '0' || speed[i] > '9') validSpeed = false;
    if ((target != "both" && target != "a" && target != "b") ||
        !validSpeed || speed.toInt() > 100 ||
        (dir != "forward" && dir != "backward" && dir != "left" && dir != "right")) {
      stopMotors();
      xSemaphoreGive(motorMutex);
      logMessage("[WARN] Invalid control command; motors stopped");
      server.send(400, "text/plain", "Invalid command");
      return;
    }
    const int power = speed.toInt() * 255 / 100;
    const int a = target == "b" ? 0 : ((dir == "backward" || dir == "left") ? -power : power);
    const int b = target == "a" ? 0 : ((dir == "backward" || dir == "right") ? -power : power);
    motor(0, a, INVERT_A);
    motor(2, b, INVERT_B);
    moving = power > 0;
  }
  lastCommand = millis();
  // Immediate ledcRead() can still show the previous cycle.
  delayMicroseconds(100);
  uint32_t duty[4];
  for (uint8_t channel = 0; channel < 4; ++channel) duty[channel] = readDuty(channel);
  const bool boosting = boostActive[0] || boostActive[1];
  xSemaphoreGive(motorMutex);
  static String previousDirection;
  static int previousSpeed = -1;
  static String previousTarget;
  const int percent = dir == "stop" ? 0 : server.arg("speed").toInt();
  if (dir != previousDirection || percent != previousSpeed || target != previousTarget) {
    logMessage("[MOTOR] %s %d%% target=%s | A GPIO%u=%u GPIO%u=%u | B GPIO%u=%u GPIO%u=%u",
               dir.c_str(), percent, target.c_str(), A_IN1, (unsigned)duty[0], A_IN2, (unsigned)duty[1],
               B_IN1, (unsigned)duty[2], B_IN2, (unsigned)duty[3]);
    if (boosting) logMessage("[MOTOR] Startup pulse 100%% for %lu ms, then %d%%",
                            (unsigned long)START_BOOST_MS, percent);
    if (dir == "stop") {
      String source = server.arg("source");
      source.replace("\r", " "); source.replace("\n", " ");
      logMessage("[INPUT] Stop source=%s", source.substring(0, 32).c_str());
    }
    previousDirection = dir;
    previousSpeed = percent;
    previousTarget = target;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "OK");
}

void safetyTask(void *) {
  for (;;) {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    const bool wasBoosting = boostActive[0] || boostActive[1];
    const bool timedOut = updateMotorTimers();
    const bool pulseFinished = wasBoosting && !boostActive[0] && !boostActive[1] && !timedOut;
    xSemaphoreGive(motorMutex);
    if (pulseFinished) logMessage("[MOTOR] Startup pulse finished; requested PWM applied");
    if (timedOut) logMessage("[SAFETY] No command for %lu ms; motors stopped",
                             (unsigned long)COMMAND_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  logMessage("[BOOT] ESP-Car starting; reset reason=%d; free heap=%u",
             (int)esp_reset_reason(), ESP.getFreeHeap());
  motorMutex = xSemaphoreCreateMutex();
  if (!motorMutex) {
    logMessage("[ERROR] Cannot allocate motor mutex");
    for (uint8_t pin : PINS) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
    while (true) delay(100);
  }
  // Serial is native USB; UART GPIO20/21 remain motor outputs.
  for (uint8_t channel = 0; channel < 4; ++channel) {
    digitalWrite(PINS[channel], LOW);
    pinMode(PINS[channel], OUTPUT);
    if (ledcSetup(channel, 20000, 8) == 0) {
      stopMotors();
      fatalError("Cannot initialize motor PWM");
    }
  }
  stopMotors();
  if (xTaskCreate(safetyTask, "motor-safety", 3072, nullptr, 2, nullptr) != pdPASS) {
    logMessage("[ERROR] Cannot create motor safety task");
    while (true) { stopMotors(); delay(100); }
  }
  logMessage("[MOTOR] Ready: A=%u/%u B=%u/%u; PWM=20kHz, 100%%=DC GPIO; timeout=%lu ms",
             A_IN1, A_IN2, B_IN1, B_IN2, (unsigned long)COMMAND_TIMEOUT_MS);
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED)
      logMessage("[WiFi] Router joined; waiting for DHCP IP");
    if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      logMessage("[WiFi] DHCP IP=%s gateway=%s", WiFi.localIP().toString().c_str(),
                 WiFi.gatewayIP().toString().c_str());
    }
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED || event == ARDUINO_EVENT_WIFI_STA_LOST_IP) {
      xSemaphoreTake(motorMutex, portMAX_DELAY);
      stopMotors();
      xSemaphoreGive(motorMutex);
      if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
        logMessage("[WiFi] Disconnected, reason=%u (%s); motors stopped; reconnecting",
                   info.wifi_sta_disconnected.reason, wifiReasonName(info.wifi_sta_disconnected.reason));
      else logMessage("[WiFi] IP lost; motors stopped");
    }
  });
  if (!WiFi.mode(WIFI_STA)) fatalError("Cannot enable WiFi station mode");
  WiFi.setAutoReconnect(true);
  if (!WiFi.setSleep(false)) logMessage("[WARN] Cannot disable WiFi sleep");
  logMessage("[WiFi] Connecting to router SSID=%s (2.4 GHz)", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Apply after begin(): the WiFi interface must already be started.
  if (!WiFi.setTxPower(WIFI_TX_POWER)) {
    logMessage("[ERROR] Cannot set WiFi TX power to 8.5 dBm");
  } else {
    logMessage("[WiFi] TX power workaround applied: %.2f dBm", (int)WiFi.getTxPower() / 4.0);
  }
  server.on("/", HTTP_GET, []() {
    logMessage("[HTTP] GET / from %s", server.client().remoteIP().toString().c_str());
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html; charset=utf-8", PAGE);
  });
  server.on("/control", HTTP_POST, control);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  logMessage("[HTTP] Routes ready; waiting for WiFi connection");
  logNetwork();
}

void loop() {
  static bool usbWasConnected = false;
  static uint32_t lastStatus = 0;
  const bool usbConnected = (bool)Serial;
  if ((usbConnected && !usbWasConnected) || millis() - lastStatus >= 10000) {
    logMessage("[STATUS] uptime=%lu s; reset=%d; heap=%u", (unsigned long)(millis()/1000),
               (int)esp_reset_reason(), ESP.getFreeHeap());
    logNetwork();
    lastStatus = millis();
  }
  usbWasConnected = usbConnected;
  static bool serverStarted = false;
  static uint32_t lastRetry = millis();
  if (WiFi.status() == WL_CONNECTED) {
    if (!serverStarted) {
      server.begin();
      serverStarted = true;
      logMessage("[HTTP] Server started: http://%s/", WiFi.localIP().toString().c_str());
    }
    lastRetry = millis();
    server.handleClient();
  } else {
    // A disconnected controller must never leave a motor powered.
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    stopMotors();
    xSemaphoreGive(motorMutex);
    if (millis() - lastRetry >= 15000) {
      lastRetry = millis();
      logMessage("[WiFi] Retrying connection to %s", WIFI_SSID);
      WiFi.reconnect();
    }
  }
  delay(1);
}
