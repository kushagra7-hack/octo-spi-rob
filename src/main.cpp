#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const char* SSID     = "POCO F7";
const char* PASSWORD = "12345678";

#define I2C_SDA        8
#define I2C_SCL        9
#define PCA9685_ADDR   0x40
#define PCA9685_OSC    27000000UL
#define SERVO_FREQ_HZ  50

#define M_CABLE_A     0
#define M_CABLE_B     1
#define M_CABLE_C     2
#define M_UPDOWN_1    3
#define M_UPDOWN_2    4
#define M_BEND        5
#define M_BASE        6
#define TOTAL_MOTORS  7

#define ACS712_SENS     0.185f
#define ACS712_ZERO_V   1.65f
#define ADC_VREF        3.3f
#define ADC_RES         4095.0f
#define CURRENT_SAMPLES 80
#define NUM_SENSORS     3

const uint8_t ACS_PIN[NUM_SENSORS] = {1, 2, 3};

#define FORCE_SOFT      0.15f
#define FORCE_MEDIUM    0.38f
#define FORCE_HARD      0.72f

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR);
WebServer server(80);

int    motorState[TOTAL_MOTORS] = {-1,-1,-1,-1,-1,-1,-1};
float  cabCurrent[NUM_SENSORS]  = {0,0,0};
float  forceLimit               = FORCE_MEDIUM;
bool   autoGrabRunning          = false;
String grabMode                 = "medium";

#define MAX_FRAMES 1000
struct Frame { int val[TOTAL_MOTORS]; };
Frame* recording   = nullptr;
int      recordCount = 0;
bool     isRecording = false;
bool     isReplaying = false;

uint32_t replayDelayMs = 50;

void captureFrame();
void disableMotor(uint8_t id);
void disableAll();
String buildJSON();

void disableMotor(uint8_t id) {
  if (id >= TOTAL_MOTORS) return;
  pca.setPWM(id, 0, 4096);
  motorState[id] = -1;
}

void disableAll() {
  for (uint8_t i = 0; i < TOTAL_MOTORS; i++) disableMotor(i);
  autoGrabRunning = false;
}

void setMotor(uint8_t id, int val) {
  if (id >= TOTAL_MOTORS) return;
  
  if (val < 0) { 
    disableMotor(id); 
    if (id != M_UPDOWN_2) captureFrame(); 
    return; 
  }
  
  uint32_t us;
  if (id == M_UPDOWN_1 || id == M_UPDOWN_2) {
    us = (uint32_t)constrain(val, 500, 2500);
    if (id == M_UPDOWN_2) us = 3000 - us;
  } else {
    us = (uint32_t)constrain(val, 1000, 2000);
  }
  motorState[id] = (int)us;
  uint16_t tick = (uint16_t)(us * ((uint32_t)SERVO_FREQ_HZ * 4096UL) / 1000000UL);
  pca.setPWM(id, 0, tick);
  
  if (id != M_UPDOWN_2) captureFrame();
}

void setUpDown(int val) {
  setMotor(M_UPDOWN_1, val);
  setMotor(M_UPDOWN_2, val);
}

void homeAll() { disableAll(); }

void captureFrame() {
  if (!isRecording || !recording) return;
  if (recordCount >= MAX_FRAMES) { isRecording = false; return; }
  for (uint8_t i = 0; i < TOTAL_MOTORS; i++)
    recording[recordCount].val[i] = motorState[i];
  recordCount++;
}

void startRecording() {
  recordCount  = 0;
  isRecording  = true;
  isReplaying  = false;
}

void stopRecording() {
  isRecording = false;
  isReplaying = false;
}

void replayRecording() {
  if (recordCount == 0) return;
  isRecording = false; // [FIX v13] prevent buffer corruption during replay
  isReplaying = true;
  while (isReplaying) {
    for (int f = 0; f < recordCount; f++) {
      if (!isReplaying) break;

      for (uint8_t i = 0; i < TOTAL_MOTORS; i++) {
        if (i == M_UPDOWN_2) continue;
        setMotor(i, recording[f].val[i]);
      }
      setMotor(M_UPDOWN_2, recording[f].val[M_UPDOWN_1]);

      uint32_t t0 = millis();
      while (isReplaying && (millis() - t0 < replayDelayMs)) {
        server.handleClient();
        yield();
        delay(5);
      }
    }
  }
  isReplaying = false;
}

void stopReplay() { isReplaying = false; }

float readCurrent(uint8_t pin) {
  long sum = 0;
  for (int i = 0; i < CURRENT_SAMPLES; i++) { sum += analogRead(pin); delayMicroseconds(80); }
  float v = ((float)sum / CURRENT_SAMPLES / ADC_RES) * ADC_VREF;
  return (v - ACS712_ZERO_V) / ACS712_SENS;
}

void updateCurrents() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) cabCurrent[i] = readCurrent(ACS_PIN[i]);
}

void autoGrab() {
  autoGrabRunning = true;
  bool a_run=true, b_run=true, c_run=true;
  setMotor(M_CABLE_A,2000); setMotor(M_CABLE_B,2000); setMotor(M_CABLE_C,2000);
  unsigned long startT = millis();
  delay(300);
  while (autoGrabRunning && (a_run||b_run||c_run)) {
    delay(25); server.handleClient(); updateCurrents();
    
    if (a_run && fabsf(cabCurrent[0])>=forceLimit) { disableMotor(M_CABLE_A); a_run=false; }
    if (b_run && fabsf(cabCurrent[1])>=forceLimit) { disableMotor(M_CABLE_B); b_run=false; }
    if (c_run && fabsf(cabCurrent[2])>=forceLimit) { disableMotor(M_CABLE_C); c_run=false; }
    if (millis()-startT > 12000) break;
  }
  disableMotor(M_CABLE_A); disableMotor(M_CABLE_B); disableMotor(M_CABLE_C);
  autoGrabRunning = false;
}

void openGripper() {
  setMotor(M_CABLE_A,1000); setMotor(M_CABLE_B,1000); setMotor(M_CABLE_C,1000);
  delay(1200);
  disableMotor(M_CABLE_A); disableMotor(M_CABLE_B); disableMotor(M_CABLE_C);
  autoGrabRunning = false;
}

String buildJSON() {
  updateCurrents();
  String j = "{";
  j += "\"ip\":\""          + WiFi.localIP().toString() + "\"";
  j += ",\"rssi\":"         + String(WiFi.RSSI());
  j += ",\"uptime\":"       + String(millis()/1000);
  j += ",\"grabMode\":\""   + grabMode + "\"";
  j += ",\"forceLimit\":"   + String(forceLimit,3);
  j += ",\"autoGrab\":"     + String(autoGrabRunning?"true":"false");
  j += ",\"replayDelay\":"  + String(replayDelayMs);
  j += ",\"motors\":[";
  for (uint8_t i=0; i<TOTAL_MOTORS; i++) {
    if (i) j+=",";
    j+="{\"id\":"+String(i)+",\"val\":"+String(motorState[i]);
    if (i<NUM_SENSORS) j+=",\"current\":"+String(cabCurrent[i],4);
    else j+=",\"current\":null";
    j+="}";
  }
  j+="]}";
  return j;
}

void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent(R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OctoGrip Robot</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap');
:root{--bg:#080c14;--panel:#0d1422;--border:#1c2a45;--ca:#00ffc8;--arm:#7b6cff;
  --text:#ccd8ee;--dim:#3a5070;--red:#ff3355;--orange:#ff8800;}
*{box-sizing:border-box;margin:0;padding:0;user-select:none}
body{background:var(--bg);color:var(--text);font-family:'Rajdhani',sans-serif;
  min-height:100vh;padding:14px}
body::before{content:'';position:fixed;inset:0;
  background:radial-gradient(ellipse at 50% 0%,rgba(0,255,200,.05),transparent 65%);
  pointer-events:none;z-index:0}
h1{text-align:center;font-size:clamp(1.3rem,4vw,2rem);font-weight:700;
  letter-spacing:.15em;margin-bottom:4px;position:relative;z-index:1;
  background:linear-gradient(90deg,var(--ca),var(--arm));
  -webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{text-align:center;font-family:'Share Tech Mono',monospace;font-size:.7rem;
  color:var(--dim);margin-bottom:16px;position:relative;z-index:1}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:14px;
  padding:18px;max-width:980px;margin:0 auto 14px;position:relative;z-index:1}
.ph{font-size:.75rem;letter-spacing:.12em;color:var(--dim);
  font-family:'Share Tech Mono',monospace;margin-bottom:12px}
.mode-row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px}
.modeBtn{flex:1;min-width:80px;padding:10px 6px;border-radius:8px;
  border:2px solid var(--border);background:transparent;color:var(--text);
  font-family:'Rajdhani',sans-serif;font-weight:700;font-size:.88rem;
  cursor:pointer;transition:all .2s;text-align:center;line-height:1.4}
.modeBtn.soft{border-color:#00ffc8;color:#00ffc8}.modeBtn.soft.active{background:#00ffc818}
.modeBtn.medium{border-color:#ffcc00;color:#ffcc00}.modeBtn.medium.active{background:#ffcc0018}
.modeBtn.hard{border-color:#ff5e3a;color:#ff5e3a}.modeBtn.hard.active{background:#ff5e3a18}
.grab-btns{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:10px}
.gbtn{flex:1;min-width:100px;padding:12px;border-radius:10px;border:none;
  font-family:'Rajdhani',sans-serif;font-weight:700;font-size:1rem;
  cursor:pointer;transition:all .2s;letter-spacing:.08em}
.gbtn:hover{filter:brightness(1.15)}
.btn-grab{background:linear-gradient(135deg,var(--ca),var(--arm));color:#000}
.btn-open{background:transparent;border:2px solid var(--ca);color:var(--ca)}
.btn-kill{background:transparent;border:2px solid var(--red);color:var(--red)}
.btn-home{background:transparent;border:2px solid var(--dim);color:var(--dim)}
.combo-section{margin-top:6px;background:#0a1020;border:1px solid var(--border);
  border-radius:10px;padding:12px 14px;margin-bottom:10px}
.combo-lbl-hdr{font-family:'Share Tech Mono',monospace;font-size:.65rem;
  color:var(--dim);letter-spacing:.1em;margin-bottom:10px}
.combo-row{display:flex;gap:8px;margin-bottom:8px;align-items:center}
.combo-row:last-child{margin-bottom:0}
.combo-tag{font-family:'Share Tech Mono',monospace;font-size:.65rem;
  color:var(--dim);min-width:58px;flex-shrink:0}
.cbtn{flex:1;padding:11px 6px;border-radius:9px;border:2px solid;
  background:transparent;font-family:'Rajdhani',sans-serif;font-weight:700;
  font-size:.88rem;cursor:pointer;transition:all .15s;text-align:center;
  line-height:1.3;-webkit-tap-highlight-color:transparent;touch-action:none}
.cbtn:active,.cbtn.held{filter:brightness(1.3);transform:scale(.97)}
.cbtn.fwd-ab{border-color:#00ffc8;color:#00ffc8}.cbtn.fwd-ab.held{background:#00ffc820}
.cbtn.fwd-bc{border-color:#ffcc00;color:#ffcc00}.cbtn.fwd-bc.held{background:#ffcc0020}
.cbtn.fwd-ac{border-color:#a78bfa;color:#a78bfa}.cbtn.fwd-ac.held{background:#a78bfa20}
.cbtn.rev{border-color:#ff3355;color:#ff3355}.cbtn.rev.held{background:#ff335520}
.curr-row{display:flex;gap:8px;flex-wrap:wrap}
.curr-box{flex:1;min-width:70px;background:#0a1020;border:1px solid var(--border);
  border-radius:8px;padding:10px;text-align:center}
.curr-label{font-size:.58rem;color:var(--dim);font-family:'Share Tech Mono',monospace;margin-bottom:4px}
.curr-val{font-size:1rem;font-weight:700;font-family:'Share Tech Mono',monospace;color:var(--ca)}
.curr-bar{height:4px;background:var(--border);border-radius:2px;margin-top:6px;overflow:hidden}
.curr-fill{height:100%;border-radius:2px;transition:width .4s}
.force-wrap{margin-top:14px;background:#0a1020;border:1px solid var(--border);
  border-radius:10px;padding:12px 14px}
.force-row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.force-label{font-family:'Share Tech Mono',monospace;font-size:.7rem;color:var(--dim);min-width:80px}
.force-val{font-family:'Share Tech Mono',monospace;font-size:1rem;font-weight:700;
  color:#7b6cff;min-width:55px;text-align:right}
.sec-lbl{font-family:'Share Tech Mono',monospace;font-size:.7rem;color:var(--dim);
  letter-spacing:.1em;max-width:980px;margin:14px auto 8px;position:relative;z-index:1}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));
  gap:12px;max-width:980px;margin:0 auto 0;position:relative;z-index:1}
.card{background:var(--panel);border:1px solid var(--border);border-radius:12px;
  padding:16px;position:relative;overflow:hidden}
.card::before{content:'';position:absolute;top:0;left:0;right:0;height:3px;background:var(--mc,#3a5070)}
.card-head{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:8px}
.cname{font-size:.9rem;font-weight:700}
.csub{font-size:.62rem;color:var(--dim);margin-top:2px}
.cch{font-family:'Share Tech Mono',monospace;font-size:.62rem;color:var(--dim);
  background:#0a1020;padding:2px 6px;border-radius:4px;border:1px solid var(--border)}
.sl-wrap{margin:8px 0 6px}
.sl-labels{display:flex;justify-content:space-between;
  font-family:'Share Tech Mono',monospace;font-size:.58rem;color:var(--dim);margin-bottom:3px}
.sl-val{text-align:center;font-family:'Share Tech Mono',monospace;font-size:1.3rem;
  font-weight:700;color:var(--mc);margin-bottom:6px}
input[type=range]{width:100%;-webkit-appearance:none;height:6px;
  background:linear-gradient(to right,#ff3355 0%,#1c2a45 45%,#1c2a45 55%,var(--mc) 100%);
  border-radius:3px;outline:none;cursor:pointer}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;
  border-radius:50%;background:var(--mc);box-shadow:0 0 8px var(--mc);cursor:pointer;transition:.1s}
input[type=range]::-webkit-slider-thumb:active{transform:scale(1.25)}
.sl-legend{display:flex;justify-content:space-between;
  font-family:'Share Tech Mono',monospace;font-size:.58rem;margin-top:3px}
.kill-row{display:flex;gap:6px;margin-top:8px}
.kill-btn{flex:1;padding:9px;border-radius:8px;border:1px solid var(--red);
  background:transparent;color:var(--red);font-family:'Rajdhani',sans-serif;
  font-weight:700;font-size:.82rem;cursor:pointer;transition:all .15s}
.kill-btn:hover{background:#ff335522}
.curt{font-size:.68rem;font-family:'Share Tech Mono',monospace;color:#3dffa0;
  text-align:right;margin-bottom:4px;height:1.1em}
.dual-badge{font-size:.58rem;background:#1a2540;padding:2px 6px;border-radius:4px;
  border:1px solid #7b6cff;color:#7b6cff;margin-left:4px}
.ondemand-badge{font-size:.58rem;background:#1a1a10;padding:2px 6px;border-radius:4px;
  border:1px solid #ff8800;color:#ff8800;margin-left:4px}
.ch6-notice{font-family:'Share Tech Mono',monospace;font-size:.62rem;color:#ff8800;
  text-align:center;margin-bottom:6px;padding:5px 8px;background:#1a1a0a;
  border-radius:6px;border:1px solid #ff880033}
.rec-dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:4px}
.rec-dot.recording{background:var(--red);animation:blink .5s infinite}
.rec-dot.replaying{background:var(--ca);animation:blink .5s infinite}
.rec-dot.idle{background:var(--dim)}
.delay-row{display:flex;align-items:center;gap:10px;flex-wrap:wrap;
  background:#0a1020;border:1px solid var(--border);border-radius:8px;
  padding:10px 14px;margin-top:12px}
.delay-label{font-family:'Share Tech Mono',monospace;font-size:.68rem;color:var(--dim);min-width:100px}
.delay-val{font-family:'Share Tech Mono',monospace;font-size:1rem;font-weight:700;
  color:var(--ca);min-width:50px;text-align:right}
.sbar{background:#06080f;border:1px solid var(--border);border-radius:8px;
  padding:9px 14px;max-width:980px;margin:12px auto;position:relative;z-index:1;
  font-family:'Share Tech Mono',monospace;font-size:.68rem;color:var(--dim);
  display:flex;flex-wrap:wrap;gap:10px;justify-content:space-between}
.sbar span{color:var(--text)}
.dot{width:6px;height:6px;border-radius:50%;background:#3dffa0;
  display:inline-block;margin-right:4px;animation:blink 2s infinite}
.log{background:#06080f;border:1px solid var(--border);border-radius:8px;
  padding:9px 14px;max-width:980px;margin:10px auto;position:relative;z-index:1;
  font-family:'Share Tech Mono',monospace;font-size:.66rem;max-height:90px;overflow-y:auto}
.le{color:var(--dim);margin-bottom:1px}
.ok{color:#3dffa0}.er{color:var(--red)}.warn{color:#ffcc00}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.2}}
</style>
</head>
<body>
<h1>🐙 OCTOGRIP ROBOT</h1>
<p class="sub">ESP32-S3 · 7× Continuous · FT5330M CH3+CH4+CH5 · ACS712 Force · v12</p>

<div class="panel">
  <div class="ph">// AUTO GRAB — PARALLEL FORCE CONTROL</div>
  <div class="mode-row">
    <button class="modeBtn soft"          onclick="setMode('soft')">🌿 SOFT<br><small>0.15A</small></button>
    <button class="modeBtn medium active" onclick="setMode('medium')">🥤 MEDIUM<br><small>0.38A</small></button>
    <button class="modeBtn hard"          onclick="setMode('hard')">📦 HARD<br><small>0.72A</small></button>
  </div>
  <div class="grab-btns">
    <button class="gbtn btn-grab" onclick="autoGrab()">🤙 AUTO GRAB</button>
    <button class="gbtn btn-open" onclick="openGripper()">✋ OPEN</button>
    <button class="gbtn btn-kill" onclick="powerOff()">⚡ ALL OFF</button>
    <button class="gbtn btn-home" onclick="homeAll()">⌂ HOME</button>
  </div>

  <div class="combo-section">
    <div class="combo-lbl-hdr">// COMBO HOLD — HOLD TO RUN · RELEASE TO STOP</div>
    <div class="combo-row">
      <span class="combo-tag">CH0+CH1</span>
      <button class="cbtn fwd-ab"
        onmousedown="if(!window._touch)comboStart([0,1],2000,'cab01f')"
        ontouchstart="window._touch=true; comboStart([0,1],2000,'cab01f'); event.preventDefault()"
        onmouseup="comboStop([0,1],'cab01f')"
        ontouchend="window._touch=false; comboStop([0,1],'cab01f'); event.preventDefault()"
        onmouseleave="comboStop([0,1],'cab01f')" id="cab01f">
        ▶ FWD<br><small style="font-size:.65rem">Hold</small>
      </button>
      <button class="cbtn rev"
        onmousedown="if(!window._touch)comboStart([0,1],1000,'cab01r')"
        ontouchstart="window._touch=true; comboStart([0,1],1000,'cab01r'); event.preventDefault()"
        onmouseup="comboStop([0,1],'cab01r')"
        ontouchend="window._touch=false; comboStop([0,1],'cab01r'); event.preventDefault()"
        onmouseleave="comboStop([0,1],'cab01r')" id="cab01r">
        ◀ REV<br><small style="font-size:.65rem">Hold</small>
      </button>
    </div>
    <div class="combo-row">
      <span class="combo-tag">CH1+CH2</span>
      <button class="cbtn fwd-bc"
        onmousedown="if(!window._touch)comboStart([1,2],2000,'cab12f')"
        ontouchstart="window._touch=true; comboStart([1,2],2000,'cab12f'); event.preventDefault()"
        onmouseup="comboStop([1,2],'cab12f')"
        ontouchend="window._touch=false; comboStop([1,2],'cab12f'); event.preventDefault()"
        onmouseleave="comboStop([1,2],'cab12f')" id="cab12f">
        ▶ FWD<br><small style="font-size:.65rem">Hold</small>
      </button>
      <button class="cbtn rev"
        onmousedown="if(!window._touch)comboStart([1,2],1000,'cab12r')"
        ontouchstart="window._touch=true; comboStart([1,2],1000,'cab12r'); event.preventDefault()"
        onmouseup="comboStop([1,2],'cab12r')"
        ontouchend="window._touch=false; comboStop([1,2],'cab12r'); event.preventDefault()"
        onmouseleave="comboStop([1,2],'cab12r')" id="cab12r">
        ◀ REV<br><small style="font-size:.65rem">Hold</small>
      </button>
    </div>
    <div class="combo-row">
      <span class="combo-tag">CH0+CH2</span>
      <button class="cbtn fwd-ac"
        onmousedown="if(!window._touch)comboStart([0,2],2000,'cab02f')"
        ontouchstart="window._touch=true; comboStart([0,2],2000,'cab02f'); event.preventDefault()"
        onmouseup="comboStop([0,2],'cab02f')"
        ontouchend="window._touch=false; comboStop([0,2],'cab02f'); event.preventDefault()"
        onmouseleave="comboStop([0,2],'cab02f')" id="cab02f">
        ▶ FWD<br><small style="font-size:.65rem">Hold</small>
      </button>
      <button class="cbtn rev"
        onmousedown="if(!window._touch)comboStart([0,2],1000,'cab02r')"
        ontouchstart="window._touch=true; comboStart([0,2],1000,'cab02r'); event.preventDefault()"
        onmouseup="comboStop([0,2],'cab02r')"
        ontouchend="window._touch=false; comboStop([0,2],'cab02r'); event.preventDefault()"
        onmouseleave="comboStop([0,2],'cab02r')" id="cab02r">
        ◀ REV<br><small style="font-size:.65rem">Hold</small>
      </button>
    </div>
  </div>

  <div class="curr-row">
    <div class="curr-box"><div class="curr-label">CABLE-A·CH0</div>
      <div class="curr-val" id="cc0">0.000A</div>
      <div class="curr-bar"><div class="curr-fill" id="cb0" style="background:#00ffc8;width:0%"></div></div></div>
    <div class="curr-box"><div class="curr-label">CABLE-B·CH1</div>
      <div class="curr-val" id="cc1">0.000A</div>
      <div class="curr-bar"><div class="curr-fill" id="cb1" style="background:#ff5e3a;width:0%"></div></div></div>
    <div class="curr-box"><div class="curr-label">CABLE-C·CH2</div>
      <div class="curr-val" id="cc2">0.000A</div>
      <div class="curr-bar"><div class="curr-fill" id="cb2" style="background:#ffcc00;width:0%"></div></div></div>
    <div class="curr-box"><div class="curr-label">FORCE LIMIT</div>
      <div class="curr-val" id="flim">0.38A</div>
      <div class="curr-bar"><div class="curr-fill" id="flimbar" style="background:#7b6cff;width:53%"></div></div></div>
  </div>
  <div class="force-wrap">
    <div class="force-row">
      <span class="force-label">⚡ FORCE</span>
      <input type="range" id="forceSlider" min="5" max="200" value="38"
             oninput="updateForceSlider(this.value)"
             style="flex:1;min-width:120px;accent-color:#7b6cff;background:#1c2a45">
      <span class="force-val" id="forceSliderVal">0.38A</span>
      <button onclick="sendForce()" style="padding:8px 14px;border-radius:8px;border:none;
        background:#7b6cff;color:#000;font-family:'Rajdhani',sans-serif;
        font-weight:700;cursor:pointer">SET</button>
    </div>
  </div>
</div>

<div class="panel">
  <div class="ph">// MOTION CAPTURE — FIXED DELAY LOOP REPLAY</div>
  <div style="background:#0a1020;border:1px solid var(--border);border-radius:10px;
    padding:14px;margin-bottom:12px;text-align:center">
    <div style="display:flex;align-items:center;justify-content:center;gap:10px">
      <span class="rec-dot idle" id="recDot"></span>
      <span id="recStatus" style="font-size:1.1rem;font-weight:700;
        font-family:'Rajdhani',sans-serif;color:var(--dim)">IDLE</span>
    </div>
    <div style="margin-top:8px;font-family:'Share Tech Mono',monospace;
      font-size:.7rem;color:var(--dim)">
      Frames: <span id="recFrames" style="color:var(--text)">0</span> / 1000
    </div>
    <div style="background:var(--border);border-radius:3px;height:5px;
      overflow:hidden;margin-top:5px">
      <div id="recBar" style="height:100%;background:var(--red);
        width:0%;transition:width .3s;border-radius:3px"></div>
    </div>
  </div>

  <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:12px">
    <button id="btnRecord" onclick="recStart()" style="padding:18px;border-radius:12px;
      border:2px solid var(--red);background:transparent;color:var(--red);
      font-family:'Rajdhani',sans-serif;font-weight:700;font-size:1.1rem;cursor:pointer">
      ⏺<br><span style="font-size:.8rem">START REC</span></button>
    <button onclick="recStop()" style="padding:18px;border-radius:12px;
      border:2px solid var(--orange);background:transparent;color:var(--orange);
      font-family:'Rajdhani',sans-serif;font-weight:700;font-size:1.1rem;cursor:pointer">
      ⏹<br><span style="font-size:.8rem">STOP REC</span></button>
    <button id="btnPlay" onclick="recPlay()" style="padding:18px;border-radius:12px;
      border:2px solid var(--ca);background:transparent;color:var(--ca);
      font-family:'Rajdhani',sans-serif;font-weight:700;font-size:1.1rem;cursor:pointer">
      ▶<br><span style="font-size:.8rem">PLAY LOOP</span></button>
    <button onclick="recStopPlay()" style="padding:18px;border-radius:12px;
      border:2px solid var(--dim);background:transparent;color:var(--dim);
      font-family:'Rajdhani',sans-serif;font-weight:700;font-size:1.1rem;cursor:pointer">
      ⏸<br><span style="font-size:.8rem">STOP PLAY</span></button>
  </div>

  <div class="delay-row">
    <span class="delay-label">⏱ REPLAY SPEED</span>
    <input type="range" id="delaySlider" min="10" max="200" value="50"
           oninput="updateDelaySlider(this.value)"
           style="flex:1;min-width:120px;--mc:var(--ca);accent-color:var(--ca)">
    <span class="delay-val" id="delayVal">50ms</span>
    <button onclick="sendDelay()" style="padding:8px 14px;border-radius:8px;border:none;
      background:var(--ca);color:#000;font-family:'Rajdhani',sans-serif;
      font-weight:700;cursor:pointer">SET</button>
  </div>
  <div style="display:flex;justify-content:space-between;font-family:'Share Tech Mono',monospace;
    font-size:.6rem;color:var(--dim);padding:3px 2px 0">
    <span>10ms — FAST</span>
    <span style="color:var(--dim)">No recorded timing — fixed delay per frame</span>
    <span>200ms — SLOW</span>
  </div>
</div>

<div class="sec-lbl">// TENTACLE CABLE WINCHES (CH0–CH2) — LIVE SPEED SLIDER</div>
<div class="grid" id="tentGrid"></div>

<div class="sec-lbl">// UP/DOWN FT5330M (CH3+CH4) — STARTS 63° · CH4 AUTO-INVERTED</div>
<div class="grid" id="udGrid"></div>

<div class="sec-lbl">// BEND (CH5 · starts 1000µs) &amp; BASE (CH6 · on-demand power)</div>
<div class="grid" id="armContGrid"></div>

<div class="sbar">
  <div><span class="dot"></span>IP: <span id="sip">—</span></div>
  <div>RSSI: <span id="srssi">—</span> dBm</div>
  <div>UP: <span id="sup">—</span>s</div>
  <div>MODE: <span id="smode">medium</span></div>
  <div>GRAB: <span id="sgrab">idle</span></div>
  <div>DELAY: <span id="sdelay">50</span>ms</div>
</div>
<div class="log" id="log"></div>

<script>
const MOTORS=[
  {id:0,name:'Cable-A', sub:'Left curl',    color:'#00ffc8'},
  {id:1,name:'Cable-B', sub:'Right curl',   color:'#ff5e3a'},
  {id:2,name:'Cable-C', sub:'Grab/Wrap',    color:'#ffcc00'},
  {id:5,name:'Bend',    sub:'FT5330M CH5',  color:'#a78bfa'},
  {id:6,name:'Base',    sub:'Rotation CH6', color:'#60a5fa'},
];
const FORCES={soft:0.15,medium:0.38,hard:0.72};
let currentMode='medium';

function buildSliderCard(id, cid, opts={}) {
  const m      = MOTORS.find(x=>x.id===id) || {name:'Motor'+id,sub:'',color:'#7b6cff'};
  const color  = opts.color || m.color;
  const label  = opts.label || m.name;
  const sub    = opts.sub   || m.sub;
  const ch     = opts.ch    || 'CH'+id;
  const defVal = opts.defVal !== undefined ? opts.defVal : 1500;
  const defLbl = opts.defLbl || usToLabel(defVal);
  const showCurrent = id < 3;
  document.getElementById(cid).innerHTML += `
  <div class="card" style="--mc:${color}">
    <div class="card-head">
      <div>
        <div class="cname" style="color:${color}">${label}${opts.badge||''}</div>
        <div class="csub">${sub}</div>
      </div>
      <div class="cch">${ch}</div>
    </div>
    ${showCurrent ? `<div class="curt" id="curt${id}">— A</div>` : '<div style="height:1.1em"></div>'}
    <div class="sl-val" id="av${id}">${defLbl}</div>
    <div class="sl-wrap">
      <div class="sl-labels">
        <span style="color:#ff5e3a">◀ REV</span>
        <span>STOP</span>
        <span style="color:${color}">FWD ▶</span>
      </div>
      <input type="range" id="sl${id}"
             min="1000" max="2000" value="${defVal}"
             oninput="liveMotor(${id}, this.value)"
             style="--mc:${color}">
      <div class="sl-legend">
        <span style="color:#ff5e3a">1000µs</span>
        <span style="color:var(--dim)">1500µs</span>
        <span style="color:${color}">2000µs</span>
      </div>
    </div>
    <div class="kill-row">
      <button class="kill-btn" onclick="killMotor(${id})">🛑 KILL CH${id}</button>
      <button onclick="snapCenter(${id})" style="flex:1;padding:9px;border-radius:8px;
        border:1px solid ${color};background:transparent;color:${color};
        font-family:'Rajdhani',sans-serif;font-weight:700;font-size:.82rem;cursor:pointer">
        ⊙ CENTER
      </button>
    </div>
  </div>`;
}

function buildBaseCard(cid) {
  const color = '#60a5fa';
  document.getElementById(cid).innerHTML += `
  <div class="card" style="--mc:${color}">
    <div class="card-head">
      <div>
        <div class="cname" style="color:${color}">Base
          <span class="ondemand-badge">ON-DEMAND</span>
        </div>
        <div class="csub">Rotation CH6</div>
      </div>
      <div class="cch">CH6</div>
    </div>
    <div class="ch6-notice" id="ch6notice">⚡ NO POWER — move slider or press FWD/REV to activate</div>
    <div class="sl-val" id="av6">IDLE</div>
    <div class="sl-wrap">
      <div class="sl-labels">
        <span style="color:#ff5e3a">◀ REV</span>
        <span>STOP/OFF</span>
        <span style="color:${color}">FWD ▶</span>
      </div>
      <input type="range" id="sl6" min="1000" max="2000" value="1500"
             oninput="liveBase(this.value)" style="--mc:${color}">
      <div class="sl-legend">
        <span style="color:#ff5e3a">1000µs</span>
        <span style="color:var(--dim)">1500µs = OFF</span>
        <span style="color:${color}">2000µs</span>
      </div>
    </div>
    <div class="kill-row">
      <button class="kill-btn" onclick="killBase()">🛑 KILL CH6</button>
      <button onmousedown="if(!window._touch)baseHold(2000,'bfwd')" 
        ontouchstart="window._touch=true; baseHold(2000,'bfwd'); event.preventDefault()"
        onmouseup="baseRelease('bfwd')" 
        ontouchend="window._touch=false; baseRelease('bfwd'); event.preventDefault()"
        onmouseleave="baseRelease('bfwd')" id="bfwd"
        style="flex:1;padding:9px;border-radius:8px;border:1px solid ${color};
          background:transparent;color:${color};font-family:'Rajdhani',sans-serif;
          font-weight:700;font-size:.82rem;cursor:pointer;-webkit-tap-highlight-color:transparent">
        ▶ HOLD FWD
      </button>
      <button onmousedown="if(!window._touch)baseHold(1000,'brev')" 
        ontouchstart="window._touch=true; baseHold(1000,'brev'); event.preventDefault()"
        onmouseup="baseRelease('brev')" 
        ontouchend="window._touch=false; baseRelease('brev'); event.preventDefault()"
        onmouseleave="baseRelease('brev')" id="brev"
        style="flex:1;padding:9px;border-radius:8px;border:1px solid #ff3355;
          background:transparent;color:#ff3355;font-family:'Rajdhani',sans-serif;
          font-weight:700;font-size:.82rem;cursor:pointer;-webkit-tap-highlight-color:transparent">
        ◀ HOLD REV
      </button>
    </div>
  </div>`;
}

function buildUDCard(cid) {
  document.getElementById(cid).innerHTML += `
  <div class="card" style="--mc:#7b6cff">
    <div class="card-head">
      <div>
        <div class="cname" style="color:#7b6cff">Up / Down (FT5330M)
          <span class="dual-badge">CH3+CH4</span>
        </div>
        <div class="csub">same command · CH4 auto-inverted · starts 63°</div>
      </div>
      <div class="cch" style="color:#7b6cff;border-color:#7b6cff">DUAL</div>
    </div>
    <div class="sl-val" id="av3">63°</div>
    <div class="sl-wrap">
      <div class="sl-labels">
        <span style="color:#ff5e3a">0°</span>
        <span>90° CENTER</span>
        <span style="color:#7b6cff">180°</span>
      </div>
      <input type="range" id="sl3" min="500" max="2500" value="1200"
             oninput="liveUD(this.value)" style="--mc:#7b6cff">
      <div class="sl-legend">
        <span style="color:#ff5e3a">0° (500µs)</span>
        <span style="color:var(--dim)">90° (1500µs)</span>
        <span style="color:#7b6cff">180° (2500µs)</span>
      </div>
    </div>
    <div class="kill-row">
      <button class="kill-btn" onclick="killUD()">🛑 KILL BOTH</button>
      <button onclick="snapUD()" style="flex:1;padding:9px;border-radius:8px;
        border:1px solid #7b6cff;background:transparent;color:#7b6cff;
        font-family:'Rajdhani',sans-serif;font-weight:700;font-size:.82rem;cursor:pointer">
        ⊙ 90° CENTER
      </button>
    </div>
  </div>`;
}

[0,1,2].forEach(id => buildSliderCard(id, 'tentGrid'));
buildUDCard('udGrid');
buildSliderCard(5, 'armContGrid', {defVal:1000, defLbl:'◀ REV 1000µs'});
buildBaseCard('armContGrid');

// ── Helpers ───────────────────────────────────────────────
function lg(msg,cls=''){
  const d=document.getElementById('log'),e=document.createElement('div');
  e.className='le '+cls;
  e.textContent=new Date().toLocaleTimeString()+' › '+msg;
  d.prepend(e);while(d.children.length>40)d.removeChild(d.lastChild);
}
async function post(ep,body){
  return fetch(ep,{method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},body});
}
function usToLabel(us) {
  us=parseInt(us);
  if(us===1500) return 'STOP';
  if(us<1500)   return `◀ REV ${us}µs`;
  return `FWD ${us}µs ▶`;
}

// ── Motor sliders ─────────────────────────────────────────
function liveMotor(id, val) {
  val=parseInt(val);
  document.getElementById('av'+id).textContent=usToLabel(val);
  clearTimeout(window['t'+id]);
  window['t'+id]=setTimeout(async()=>{
    try{await post('/api/motor',`id=${id}&angle=${val}`);}
    catch(e){lg('err motor '+id,'er');}
  },25);
}
function liveUD(val) {
  val=parseInt(val);
  const deg=Math.round((val-500)/2000*180);
  document.getElementById('av3').textContent=deg+'°';
  clearTimeout(window.tud);
  window.tud=setTimeout(async()=>{
    try{await post('/api/updown',`angle=${val}`);}
    catch(e){lg('err updown','er');}
  },25);
}
function liveBase(val) {
  val=parseInt(val);
  const notice=document.getElementById('ch6notice');
  if(val===1500){
    document.getElementById('av6').textContent='IDLE';
    if(notice){notice.textContent='⚡ NO POWER — move slider or press FWD/REV to activate';notice.style.color='#ff8800';}
    post('/api/motor','id=6&angle=-1');
  } else {
    document.getElementById('av6').textContent=usToLabel(val);
    if(notice){notice.textContent='✅ POWERED — '+usToLabel(val);notice.style.color='#3dffa0';}
    clearTimeout(window.t6);
    window.t6=setTimeout(async()=>{
      try{await post('/api/motor',`id=6&angle=${val}`);}
      catch(e){lg('err base','er');}
    },25);
  }
}
function baseHold(speed,btnId){
  const sl=document.getElementById('sl6');if(sl)sl.value=speed;
  const notice=document.getElementById('ch6notice');
  document.getElementById('av6').textContent=usToLabel(speed);
  if(notice){notice.textContent='✅ POWERED — '+usToLabel(speed);notice.style.color='#3dffa0';}
  const btn=document.getElementById(btnId);if(btn)btn.style.opacity='0.6';
  post('/api/motor',`id=6&angle=${speed}`);
  lg('Base CH6 → '+(speed===2000?'FWD':'REV'),'warn');
}
function baseRelease(btnId){
  const sl=document.getElementById('sl6');if(sl)sl.value=1500;
  const notice=document.getElementById('ch6notice');
  document.getElementById('av6').textContent='IDLE';
  if(notice){notice.textContent='⚡ NO POWER — move slider or press FWD/REV to activate';notice.style.color='#ff8800';}
  const btn=document.getElementById(btnId);if(btn)btn.style.opacity='1';
  post('/api/motor','id=6&angle=-1');
}
function killBase(){
  const sl=document.getElementById('sl6');if(sl)sl.value=1500;
  const notice=document.getElementById('ch6notice');
  document.getElementById('av6').textContent='IDLE';
  if(notice){notice.textContent='⚡ NO POWER — move slider or press FWD/REV to activate';notice.style.color='#ff8800';}
  post('/api/motor','id=6&angle=-1');
  lg('Kill CH6','warn');
}
function killMotor(id){
  const sl=document.getElementById('sl'+id);if(sl)sl.value=1500;
  document.getElementById('av'+id).textContent='STOP';
  post('/api/motor',`id=${id}&angle=-1`);
  lg('Kill CH'+id,'warn');
}

function killUD(){
  document.getElementById('av3').textContent='OFF';
  post('/api/updown','angle=-1');
  lg('Kill UpDown','warn');
}

function snapCenter(id){
  document.getElementById('sl'+id).value=1500;
  document.getElementById('av'+id).textContent='STOP';
  post('/api/motor',`id=${id}&angle=1500`);
}
function snapUD(){
  document.getElementById('sl3').value=1500;
  document.getElementById('av3').textContent='90°';
  post('/api/updown','angle=1500');
}

// ── Combo hold ────────────────────────────────────────────
function comboStart(ids,speed,btnId){
  const btn=document.getElementById(btnId);if(btn)btn.classList.add('held');
  ids.forEach(id=>{
    const sl=document.getElementById('sl'+id);if(sl)sl.value=speed;
    const av=document.getElementById('av'+id);if(av)av.textContent=usToLabel(speed);
    post('/api/motor',`id=${id}&angle=${speed}`).catch(()=>{});
  });
  lg((speed===2000?'▶ FWD':'◀ REV')+' Combo CH'+ids.join('+CH')+' ON','warn');
}
function comboStop(ids,btnId){
  const btn=document.getElementById(btnId);if(btn)btn.classList.remove('held');
  ids.forEach(id=>{
    const sl=document.getElementById('sl'+id);if(sl)sl.value=1500;
    const av=document.getElementById('av'+id);if(av)av.textContent='STOP';
    post('/api/motor',`id=${id}&angle=-1`).catch(()=>{});
  });
  lg('⏹ Combo CH'+ids.join('+CH')+' OFF','ok');
}

// [FIX v12] window._touch reset on touchcancel prevents the permanent lockout bug
document.addEventListener('touchcancel',()=>{
  window._touch = false; 
  ['cab01f','cab01r','cab12f','cab12r','cab02f','cab02r'].forEach(id=>{
    const b=document.getElementById(id);if(b)b.classList.remove('held');
  });
  comboStop([0,1],'cab01f');comboStop([0,1],'cab01r');
  comboStop([1,2],'cab12f');comboStop([1,2],'cab12r');
  comboStop([0,2],'cab02f');comboStop([0,2],'cab02r');
  baseRelease('bfwd');baseRelease('brev');
});
window.addEventListener('beforeunload',()=>{ navigator.sendBeacon('/api/poweroff',''); });

// ── Force ─────────────────────────────────────────────────
function updateForceSlider(v){
  const a=(parseInt(v)/100).toFixed(2);
  document.getElementById('forceSliderVal').textContent=a+'A';
  document.getElementById('flim').textContent=a+'A';
  document.getElementById('flimbar').style.width=(parseInt(v)/200*100)+'%';
}
async function sendForce(){
  const v=document.getElementById('forceSlider').value;
  const a=(parseInt(v)/100).toFixed(2);
  try{await post('/api/force',`limit=${a}`);lg('Force → '+a+'A','warn');}
  catch(e){lg('Error: '+e,'er');}
}

// ── Replay delay slider ───────────────────────────────────
function updateDelaySlider(v){
  document.getElementById('delayVal').textContent=v+'ms';
}
async function sendDelay(){
  const v=parseInt(document.getElementById('delaySlider').value);
  try{
    await post('/api/rec/delay',`ms=${v}`);
    document.getElementById('sdelay').textContent=v;
    lg('Replay delay → '+v+'ms','ok');
  }catch(e){lg('Error: '+e,'er');}
}

// ── Mode ──────────────────────────────────────────────────
function setMode(mode){
  currentMode=mode;
  document.querySelectorAll('.modeBtn').forEach(b=>b.classList.remove('active'));
  document.querySelector('.modeBtn.'+mode).classList.add('active');
  document.getElementById('flim').textContent=FORCES[mode].toFixed(2)+'A';
  document.getElementById('flimbar').style.width=(FORCES[mode]/0.72*100)+'%';
  post('/api/mode',`mode=${mode}`);
  lg('Mode → '+mode,'warn');
}

// ── Grab actions ──────────────────────────────────────────
async function autoGrab(){
  lg('▶ Auto grab — '+currentMode,'warn');
  try{await post('/api/grab',`mode=${currentMode}`);lg('Done','ok');}
  catch(e){lg('Error: '+e,'er');}
}
async function openGripper(){
  try{await post('/api/open','');lg('Open','ok');}
  catch(e){lg('Error: '+e,'er');}
}
function resetAllSliders(){
  [0,1,2].forEach(id=>{
    const sl=document.getElementById('sl'+id);
    if(sl){sl.value=1500;document.getElementById('av'+id).textContent='STOP';}
  });
  document.getElementById('sl3').value=1200;
  document.getElementById('av3').textContent='63°';
  const sl5=document.getElementById('sl5');
  if(sl5){sl5.value=1000;document.getElementById('av5').textContent='◀ REV 1000µs';}
  const sl6=document.getElementById('sl6');if(sl6)sl6.value=1500;
  document.getElementById('av6').textContent='IDLE';
  const notice=document.getElementById('ch6notice');
  if(notice){notice.textContent='⚡ NO POWER — move slider or press FWD/REV to activate';notice.style.color='#ff8800';}
}
async function homeAll(){
  try{await post('/api/home','');resetAllSliders();lg('All → zero power','ok');}
  catch(e){lg('Error: '+e,'er');}
}
async function powerOff(){
  try{await post('/api/poweroff','');resetAllSliders();lg('⚡ ALL ZERO POWER','warn');}
  catch(e){lg('Error: '+e,'er');}
}

async function recStart(){
  try{await post('/api/rec/start','');lg('⏺ Recording started — press buttons now','warn');}
  catch(e){lg('Error: '+e,'er');}
}
async function recStop(){
  try{
    await post('/api/rec/stop','');
    lg('⏹ Recording stopped','ok');
  }catch(e){lg('Error: '+e,'er');}
}
async function recPlay(){
  try{await post('/api/rec/play','');lg('▶ Loop replay started','ok');}
  catch(e){lg('Error: '+e,'er');}
}
async function recStopPlay(){
  try{await post('/api/rec/stopplay','');lg('⏸ Replay stopped','warn');}
  catch(e){lg('Error: '+e,'er');}
}

// ── Poll status ───────────────────────────────────────────
async function poll(){
  try{
    const d=await(await fetch('/api/status')).json();
    document.getElementById('sip').textContent=d.ip;
    document.getElementById('srssi').textContent=d.rssi;
    document.getElementById('sup').textContent=d.uptime;
    document.getElementById('smode').textContent=d.grabMode;
    document.getElementById('sgrab').textContent=d.autoGrab?'GRABBING':'idle';
    if(d.replayDelay!==undefined){
      document.getElementById('sdelay').textContent=d.replayDelay;
      const sl=document.getElementById('delaySlider');
      if(sl && document.activeElement!==sl){
        sl.value=d.replayDelay;
        document.getElementById('delayVal').textContent=d.replayDelay+'ms';
      }
    }
    d.motors.forEach(m=>{
      if(m.current!==null && m.id<3){
        const a=Math.abs(m.current),pct=Math.min(100,(a/0.72)*100);
        document.getElementById('cc'+m.id).textContent=m.current.toFixed(3)+'A';
        document.getElementById('cb'+m.id).style.width=pct+'%';
        const cu=document.getElementById('curt'+m.id);
        if(cu)cu.textContent=m.current.toFixed(3)+' A';
      }
    });
  }catch(e){}
}
setInterval(poll,1500); poll();

setInterval(async()=>{
  try{
    const d=await(await fetch('/api/rec/status')).json();
    document.getElementById('recFrames').textContent=d.frames;
    document.getElementById('recBar').style.width=(d.frames/1000*100)+'%';
    const dot=document.getElementById('recDot');
    const sta=document.getElementById('recStatus');
    const bR=document.getElementById('btnRecord');
    const bP=document.getElementById('btnPlay');
    bR.style.background='transparent';bP.style.background='transparent';
    if(d.recording){
      dot.className='rec-dot recording';sta.textContent='● RECORDING... press your buttons now';
      sta.style.color='#ff3355';bR.style.background='#ff335533';
    }else if(d.replaying){
      dot.className='rec-dot replaying';sta.textContent='▶ LOOPING...';
      sta.style.color='#00ffc8';bP.style.background='#00ffc833';
    }else{
      dot.className='rec-dot idle';
      sta.textContent=d.frames>0?`READY — ${d.frames} frames`:'IDLE';
      sta.style.color=d.frames>0?'#00ffc8':'var(--dim)';
    }
  }catch(e){}
},600);

window.addEventListener('load',()=>{
  post('/api/motor','id=5&angle=1000').catch(()=>{});
  post('/api/updown','angle=1200').catch(()=>{});
  lg('OctoGrip v12 🐙 — Critical loop & touch bugs patched','ok');
});
</script>
</body>
</html>)HTML");
  server.sendContent("");
}

void handleStatus()    { server.send(200,"application/json",buildJSON()); }
void handleSetMotor()  {
  if(!server.hasArg("id")||!server.hasArg("angle")){
    server.send(400,"application/json","{\"error\":\"Missing args\"}");return;}
  if(isReplaying){isReplaying=false;}
  setMotor(server.arg("id").toInt(),server.arg("angle").toInt());
  server.send(200,"application/json","{\"ok\":1}");
}
void handleSetUpDown() {
  if(!server.hasArg("angle")){
    server.send(400,"application/json","{\"error\":\"Missing angle\"}");return;}
  if(isReplaying){isReplaying=false;}
  setUpDown(server.arg("angle").toInt());
  server.send(200,"application/json","{\"ok\":1}");
}
void handleSetMode() {
  if(server.hasArg("mode")){
    grabMode=server.arg("mode");
    if(grabMode=="soft")        forceLimit=FORCE_SOFT;
    else if(grabMode=="medium") forceLimit=FORCE_MEDIUM;
    else if(grabMode=="hard")   forceLimit=FORCE_HARD;
  }
  server.send(200,"application/json","{\"ok\":1}");
}

// [FIX v12] Prevent stack overflow by adding autoGrabRunning guard
void handleAutoGrab() {
  if (autoGrabRunning) { 
    server.send(200,"application/json","{\"ok\":1}"); 
    return; 
  }
  isReplaying = false; // [FIX v13] stop replay before grab
  
  if(server.hasArg("mode")){
    grabMode=server.arg("mode");
    if(grabMode=="soft")        forceLimit=FORCE_SOFT;
    else if(grabMode=="medium") forceLimit=FORCE_MEDIUM;
    else if(grabMode=="hard")   forceLimit=FORCE_HARD;
  }
  server.send(200,"application/json","{\"ok\":1}");
  autoGrab();
}

void handleOpen()     { isReplaying=false; autoGrabRunning=false; server.send(200,"application/json","{\"ok\":1}"); openGripper(); } // [FIX v13]
void handleHome()     { isReplaying=false; autoGrabRunning=false; server.send(200,"application/json","{\"ok\":1}"); homeAll(); } // [FIX v13]
void handleSetForce() {
  if(server.hasArg("limit")) forceLimit=constrain(server.arg("limit").toFloat(),0.05f,2.0f);
  server.send(200,"application/json","{\"ok\":1}");
}
void handleRecStart()    { startRecording(); server.send(200,"application/json","{\"ok\":1}"); }
void handleRecStop()     { stopRecording();  server.send(200,"application/json","{\"ok\":1}"); }
void handleRecPlay()     { 
  if (isReplaying) { server.send(200,"application/json","{\"ok\":1}"); return; } 
  server.send(200,"application/json","{\"ok\":1}"); 
  replayRecording(); 
}

void handleRecStopPlay() { stopReplay();     server.send(200,"application/json","{\"ok\":1}"); }
void handleRecDelay() {
  if(server.hasArg("ms"))
    replayDelayMs = (uint32_t)constrain(server.arg("ms").toInt(), 10, 200);
  server.send(200,"application/json","{\"ok\":1}");
}
void handleRecStatus() {
  String j="{\"recording\":"+String(isRecording?"true":"false");
  j+=",\"replaying\":"+String(isReplaying?"true":"false");
  j+=",\"frames\":"+String(recordCount);
  j+=",\"delayMs\":"+String(replayDelayMs)+"}";
  server.send(200,"application/json",j);
}
void handleNotFound() { server.send(404,"application/json","{\"error\":\"Not found\"}"); }

void setup() {
  Serial.begin(115200);
  unsigned long _t=millis();
  while(!Serial&&millis()-_t<3000);
  delay(200);
  Serial.println("\n>>> OCTOGRIP v12 <<<");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  recording = psramFound()
    ? (Frame*)ps_malloc(MAX_FRAMES*sizeof(Frame))
    : new Frame[MAX_FRAMES];
  if(!recording) Serial.println("[MEM] ERROR");
  else Serial.printf("[MEM] %d frames OK (PSRAM:%d)\n", MAX_FRAMES, (int)psramFound());

  Wire.begin(I2C_SDA,I2C_SCL);
  Wire.setClock(100000);

  bool pcaOK=false;
  for(int a=1;a<=5;a++){ delay(100); if(pca.begin()){pcaOK=true;break;} }
  if(!pcaOK){ Serial.println("[PCA9685] ERROR"); }
  else {
    pca.setOscillatorFrequency(PCA9685_OSC);
    pca.setPWMFreq(SERVO_FREQ_HZ);
    delay(20);
    disableAll();
    setMotor(M_BEND, 1000);
    setUpDown(1200);
    Serial.println("[MOTORS] Boot: CH5=1000µs · CH3+4=63° · CH6=OFF");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID,PASSWORD);
  Serial.printf("[WiFi] Connecting to %s",SSID);
  unsigned long t=millis();
  while(WiFi.status()!=WL_CONNECTED){
    if(millis()-t>20000){Serial.println("\n[WiFi] FAILED");break;}
    delay(500);Serial.print(".");
  }
  if(WiFi.status()==WL_CONNECTED)
    Serial.printf("\n[WiFi] http://%s/\n",WiFi.localIP().toString().c_str());

  server.on("/",                HTTP_GET,  handleRoot);
  server.on("/api/status",      HTTP_GET,  handleStatus);
  server.on("/api/rec/status",  HTTP_GET,  handleRecStatus);
  server.on("/api/motor",       HTTP_POST, handleSetMotor);
  server.on("/api/updown",      HTTP_POST, handleSetUpDown);
  server.on("/api/mode",        HTTP_POST, handleSetMode);
  server.on("/api/grab",        HTTP_POST, handleAutoGrab);
  server.on("/api/open",        HTTP_POST, handleOpen);
  server.on("/api/home",        HTTP_POST, handleHome);
  server.on("/api/poweroff",    HTTP_POST, [](){ isReplaying=false; autoGrabRunning=false; disableAll(); server.send(200,"application/json","{\"ok\":1}"); }); // [FIX v13]
  server.on("/api/force",       HTTP_POST, handleSetForce);
  server.on("/api/rec/start",   HTTP_POST, handleRecStart);
  server.on("/api/rec/stop",    HTTP_POST, handleRecStop);
  server.on("/api/rec/play",    HTTP_POST, handleRecPlay);
  server.on("/api/rec/stopplay",HTTP_POST, handleRecStopPlay);
  server.on("/api/rec/delay",   HTTP_POST, handleRecDelay);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[HTTP] Port 80 ready");
}

void loop() {
  server.handleClient();
  static unsigned long lastLog=0;
  if(millis()-lastLog>=4000){
    lastLog=millis();
    updateCurrents();
    Serial.printf("[CURR] A=%.3f B=%.3f C=%.3f Limit=%.2f Delay=%dms\n",
      cabCurrent[0],cabCurrent[1],cabCurrent[2],forceLimit,(int)replayDelayMs);
  }
  static unsigned long lastWifi=0;
  if(millis()-lastWifi>=10000){
    lastWifi=millis();
    if(WiFi.status()!=WL_CONNECTED) WiFi.reconnect();
  }
}