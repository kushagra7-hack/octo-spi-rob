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

const char* MOTOR_NAME[TOTAL_MOTORS] = {
  "Cable-A (Left curl)",
  "Cable-B (Right curl)",
  "Cable-C (Grab/Wrap)",
  "UpDown Motor 1 (FT5330M)",
  "UpDown Motor 2 (FT5330M inv)",
  "Bend (FT5330M toggle)",
  "Base (Rotation)"
};

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

int    motorState[TOTAL_MOTORS] = { -1, -1, -1, -1, -1, -1, -1 };
float  cabCurrent[NUM_SENSORS]  = { 0, 0, 0 };
float  forceLimit               = FORCE_MEDIUM;
bool   autoGrabRunning          = false;
String grabMode                 = "medium";

#define MAX_FRAMES 1000
struct Frame { int val[TOTAL_MOTORS]; uint32_t delayMs; };
Frame*   recording   = nullptr;
int      recordCount = 0;
bool     isRecording = false;
bool     isReplaying = false;
uint32_t lastFrameMs = 0;

void captureFrame();
void disableMotor(uint8_t id);
void disableAll();
String buildJSON();

void disableMotor(uint8_t id) {
  if (id >= TOTAL_MOTORS) return;
  pca.setPWM(id, 0, 4096);
  motorState[id] = -1;
  Serial.printf("[MOT] CH%d -> ZERO POWER\n", id);
}

void disableAll() {
  for (uint8_t i = 0; i < TOTAL_MOTORS; i++) disableMotor(i);
  autoGrabRunning = false;
}

// CH4 auto-inverted: 3000-us mirrors around 1500
// So CH3=2000 → CH4=1000 (opposite spin)
// And CH3=1000 → CH4=2000 (opposite spin)
void setMotor(uint8_t id, int val) {
  if (id >= TOTAL_MOTORS) return;
  if (val < 0) { disableMotor(id); captureFrame(); return; }
  uint32_t us = (uint32_t)constrain(val, 1000, 2000);
  if (id == M_UPDOWN_2) us = 3000 - us;
  motorState[id] = (int)us;
  Serial.printf("[MOT] CH%d -> %uus\n", id, (unsigned)us);
  uint16_t tick = (uint16_t)(us * ((uint32_t)SERVO_FREQ_HZ * 4096UL) / 1000000UL);
  pca.setPWM(id, 0, tick);
  captureFrame();
}

void setUpDown(int val) {
  setMotor(M_UPDOWN_1, val);
  setMotor(M_UPDOWN_2, val);
}

void homeAll() { disableAll(); }

void captureFrame() {
  if (!isRecording || !recording) return;
  if (recordCount >= MAX_FRAMES) { isRecording = false; return; }
  uint32_t now = millis();
  uint32_t dt  = (recordCount == 0) ? 0 : now - lastFrameMs;
  lastFrameMs  = now;
  for (uint8_t i = 0; i < TOTAL_MOTORS; i++)
    recording[recordCount].val[i] = motorState[i];
  recording[recordCount].delayMs = dt;
  recordCount++;
}

void startRecording() { recordCount=0; isRecording=true; isReplaying=false; lastFrameMs=millis(); }
void stopRecording()  { isRecording=false; }

void replayRecording() {
  if (recordCount == 0) return;
  isReplaying = true;
  for (int f = 0; f < recordCount; f++) {
    if (!isReplaying) break;
    server.handleClient();
    if (recording[f].delayMs > 0) delay(constrain(recording[f].delayMs, 10, 2000));
    for (uint8_t i = 0; i < TOTAL_MOTORS; i++) {
      if (i == M_UPDOWN_2) continue;
      setMotor(i, recording[f].val[i]);
    }
    setMotor(M_UPDOWN_2, recording[f].val[M_UPDOWN_1]);
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
  setMotor(M_CABLE_A, 2000); setMotor(M_CABLE_B, 2000); setMotor(M_CABLE_C, 2000);
  unsigned long startT = millis();
  delay(300);
  while (autoGrabRunning && (a_run||b_run||c_run)) {
    delay(25); server.handleClient(); updateCurrents();
    if (a_run && abs(cabCurrent[0])>=forceLimit) { disableMotor(M_CABLE_A); a_run=false; }
    if (b_run && abs(cabCurrent[1])>=forceLimit) { disableMotor(M_CABLE_B); b_run=false; }
    if (c_run && abs(cabCurrent[2])>=forceLimit) { disableMotor(M_CABLE_C); c_run=false; }
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
  j += "\"ip\":\""        + WiFi.localIP().toString() + "\"";
  j += ",\"rssi\":"       + String(WiFi.RSSI());
  j += ",\"uptime\":"     + String(millis()/1000);
  j += ",\"grabMode\":\"" + grabMode + "\"";
  j += ",\"forceLimit\":" + String(forceLimit,3);
  j += ",\"autoGrab\":"   + String(autoGrabRunning?"true":"false");
  j += ",\"motors\":[";
  for (uint8_t i=0; i<TOTAL_MOTORS; i++) {
    if (i) j+=",";
    j+="{\"id\":"+String(i)+",\"name\":\""+String(MOTOR_NAME[i])+"\",\"val\":"+String(motorState[i]);
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
:root{--bg:#080c14;--panel:#0d1422;--border:#1c2a45;--ca:#00ffc8;--arm:#7b6cff;--text:#ccd8ee;--dim:#3a5070;--red:#ff3355;--orange:#ff8800;}
*{box-sizing:border-box;margin:0;padding:0;user-select:none}
body{background:var(--bg);color:var(--text);font-family:'Rajdhani',sans-serif;min-height:100vh;padding:14px}
body::before{content:'';position:fixed;inset:0;background:radial-gradient(ellipse at 50% 0%,rgba(0,255,200,.05),transparent 65%);pointer-events:none;z-index:0}
h1{text-align:center;font-size:clamp(1.3rem,4vw,2rem);font-weight:700;letter-spacing:.15em;margin-bottom:4px;position:relative;z-index:1;background:linear-gradient(90deg,var(--ca),var(--arm));-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{text-align:center;font-family:'Share Tech Mono',monospace;font-size:.7rem;color:var(--dim);margin-bottom:16px;position:relative;z-index:1}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:14px;padding:18px;max-width:980px;margin:0 auto 14px;position:relative;z-index:1}
.ph{font-size:.75rem;letter-spacing:.12em;color:var(--dim);font-family:'Share Tech Mono',monospace;margin-bottom:12px}
.mode-row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px}
.modeBtn{flex:1;min-width:80px;padding:10px 6px;border-radius:8px;border:2px solid var(--border);background:transparent;color:var(--text);font-family:'Rajdhani',sans-serif;font-weight:700;font-size:.88rem;cursor:pointer;transition:all .2s;text-align:center;line-height:1.4}
.modeBtn.soft{border-color:#00ffc8;color:#00ffc8}.modeBtn.soft.active{background:#00ffc818}
.modeBtn.medium{border-color:#ffcc00;color:#ffcc00}.modeBtn.medium.active{background:#ffcc0018}
.modeBtn.hard{border-color:#ff5e3a;color:#ff5e3a}.modeBtn.hard.active{background:#ff5e3a18}
.grab-btns{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:14px}
.gbtn{flex:1;min-width:100px;padding:12px;border-radius:10px;border:none;font-family:'Rajdhani',sans-serif;font-weight:700;font-size:1rem;cursor:pointer;transition:all .2s;letter-spacing:.08em}
.btn-grab{background:linear-gradient(135deg,var(--ca),var(--arm));color:#000}
.btn-grab:hover{filter:brightness(1.15)}
.btn-open{background:transparent;border:2px solid var(--ca);color:var(--ca)}
.btn-kill{background:transparent;border:2px solid var(--red);color:var(--red)}
.btn-home{background:transparent;border:2px solid var(--dim);color:var(--dim)}
.curr-row{display:flex;gap:8px;flex-wrap:wrap}
.curr-box{flex:1;min-width:70px;background:#0a1020;border:1px solid var(--border);border-radius:8px;padding:10px;text-align:center}
.curr-label{font-size:.58rem;color:var(--dim);font-family:'Share Tech Mono',monospace;margin-bottom:4px}
.curr-val{font-size:1rem;font-weight:700;font-family:'Share Tech Mono',monospace;color:var(--ca)}
.curr-bar{height:4px;background:var(--border);border-radius:2px;margin-top:6px;overflow:hidden}
.curr-fill{height:100%;border-radius:2px;transition:width .4s}
.force-wrap{margin-top:14px;background:#0a1020;border:1px solid var(--border);border-radius:10px;padding:12px 14px}
.force-row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.force-label{font-family:'Share Tech Mono',monospace;font-size:.7rem;color:var(--dim);min-width:80px}
.force-val{font-family:'Share Tech Mono',monospace;font-size:1rem;font-weight:700;color:#7b6cff;min-width:55px;text-align:right}
input#forceSlider{flex:1;min-width:120px;accent-color:#7b6cff}
.rec-dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:4px}
.rec-dot.recording{background:var(--red);animation:blink .5s infinite}
.rec-dot.replaying{background:var(--ca);animation:blink .5s infinite}
.rec-dot.idle{background:var(--dim)}
.arm-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px}
.arm-ctrl{background:#0a1020;border:1px solid var(--border);border-radius:10px;padding:14px}
.arm-name{font-size:.85rem;font-weight:700;margin-bottom:8px}
.arm-btns{display:flex;gap:5px;flex-wrap:wrap}
.abtn{flex:1;min-width:44px;padding:9px 4px;border-radius:7px;border:none;font-family:'Rajdhani',sans-serif;font-weight:700;font-size:.78rem;cursor:pointer;transition:all .15s;text-align:center;line-height:1.3}
.abtn:active{transform:scale(.96)}
.kbtn{padding:9px 8px;border-radius:7px;border:1px solid var(--red);background:transparent;color:var(--red);font-family:'Rajdhani',sans-serif;font-weight:700;font-size:.82rem;cursor:pointer}
.arm-val{font-family:'Share Tech Mono',monospace;font-size:.78rem;color:var(--dim);text-align:center;margin-top:7px}
.badge{font-size:.58rem;background:#1a2540;padding:2px 6px;border-radius:4px;border:1px solid #7b6cff;color:#7b6cff;margin-left:4px}
/* ── TOGGLE BUTTON ── */
.tog{width:100%;margin-top:8px;padding:11px;border-radius:8px;border:2px solid #a78bfa;background:transparent;color:#a78bfa;font-family:'Rajdhani',sans-serif;font-weight:700;font-size:.92rem;cursor:pointer;transition:all .2s;letter-spacing:.05em}
.tog.active{background:#a78bfa33;color:#fff;border-color:#fff}
.sec-lbl{font-family:'Share Tech Mono',monospace;font-size:.7rem;color:var(--dim);letter-spacing:.1em;max-width:980px;margin:14px auto 8px;position:relative;z-index:1}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:12px;max-width:980px;margin:0 auto;position:relative;z-index:1}
.card{background:var(--panel);border:1px solid var(--border);border-radius:12px;padding:16px;position:relative;overflow:hidden}
.card::before{content:'';position:absolute;top:0;left:0;right:0;height:3px;background:var(--mc,#3a5070)}
.card-head{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:8px}
.cname{font-size:.9rem;font-weight:700}
.csub{font-size:.62rem;color:var(--dim);margin-top:2px}
.cch{font-family:'Share Tech Mono',monospace;font-size:.62rem;color:var(--dim);background:#0a1020;padding:2px 6px;border-radius:4px;border:1px solid var(--border)}
.ang-disp{text-align:center;font-size:1.5rem;font-weight:700;font-family:'Share Tech Mono',monospace;margin:6px 0}
.curt{font-size:.68rem;font-family:'Share Tech Mono',monospace;color:#3dffa0;text-align:right;margin-bottom:6px;height:1.1em}
.hrow{display:flex;gap:5px;margin-top:6px}
.hbtn{flex:1;padding:10px 3px;border-radius:7px;border:1px solid;background:transparent;font-family:'Rajdhani',sans-serif;font-weight:700;font-size:.78rem;cursor:pointer;text-align:center;line-height:1.3}
.hbtn:active{opacity:.65}
.sbar{background:#06080f;border:1px solid var(--border);border-radius:8px;padding:9px 14px;max-width:980px;margin:12px auto;position:relative;z-index:1;font-family:'Share Tech Mono',monospace;font-size:.68rem;color:var(--dim);display:flex;flex-wrap:wrap;gap:10px;justify-content:space-between}
.sbar span{color:var(--text)}
.dot{width:6px;height:6px;border-radius:50%;background:#3dffa0;display:inline-block;margin-right:4px;animation:blink 2s infinite}
.log{background:#06080f;border:1px solid var(--border);border-radius:8px;padding:9px 14px;max-width:980px;margin:10px auto;position:relative;z-index:1;font-family:'Share Tech Mono',monospace;font-size:.66rem;max-height:90px;overflow-y:auto}
.le{color:var(--dim);margin-bottom:1px}
.ok{color:#3dffa0}.er{color:var(--red)}.warn{color:#ffcc00}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.2}}
</style>
</head>
<body>
<h1>🐙 OCTOGRIP ROBOT</h1>
<p class="sub">ESP32-S3 · 7× Continuous · FT5330M CH3+CH4+CH5 · ACS712 Force</p>

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
  <div class="curr-row">
    <div class="curr-box"><div class="curr-label">CABLE-A·CH0</div><div class="curr-val" id="cc0">0.000A</div><div class="curr-bar"><div class="curr-fill" id="cb0" style="background:#00ffc8;width:0%"></div></div></div>
    <div class="curr-box"><div class="curr-label">CABLE-B·CH1</div><div class="curr-val" id="cc1">0.000A</div><div class="curr-bar"><div class="curr-fill" id="cb1" style="background:#ff5e3a;width:0%"></div></div></div>
    <div class="curr-box"><div class="curr-label">CABLE-C·CH2</div><div class="curr-val" id="cc2">0.000A</div><div class="curr-bar"><div class="curr-fill" id="cb2" style="background:#ffcc00;width:0%"></div></div></div>
    <div class="curr-box"><div class="curr-label">FORCE LIMIT</div><div class="curr-val" id="flim">0.38A</div><div class="curr-bar"><div class="curr-fill" id="flimbar" style="background:#7b6cff;width:53%"></div></div></div>
  </div>
  <div class="force-wrap">
    <div class="force-row">
      <span class="force-label">⚡ FORCE</span>
      <input type="range" id="forceSlider" min="5" max="200" value="38" oninput="updateForceSlider(this.value)">
      <span class="force-val" id="forceSliderVal">0.38A</span>
      <button onclick="sendForce()" style="padding:8px 14px;border-radius:8px;border:none;background:#7b6cff;color:#000;font-family:'Rajdhani',sans-serif;font-weight:700;cursor:pointer">SET</button>
    </div>
  </div>
</div>

<div class="panel">
  <div class="ph">// MOTION CAPTURE</div>
  <div style="background:#0a1020;border:1px solid var(--border);border-radius:10px;padding:14px;margin-bottom:14px;text-align:center">
    <div style="display:flex;align-items:center;justify-content:center;gap:10px">
      <span class="rec-dot idle" id="recDot"></span>
      <span id="recStatus" style="font-size:1.1rem;font-weight:700;font-family:'Rajdhani',sans-serif;color:var(--dim)">IDLE</span>
    </div>
    <div style="margin-top:8px;font-family:'Share Tech Mono',monospace;font-size:.7rem;color:var(--dim)">
      Frames: <span id="recFrames" style="color:var(--text)">0</span> / 1000
    </div>
    <div style="background:var(--border);border-radius:3px;height:5px;overflow:hidden;margin-top:5px">
      <div id="recBar" style="height:100%;background:var(--red);width:0%;transition:width .3s;border-radius:3px"></div>
    </div>
  </div>
  <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px">
    <button id="btnRecord" onclick="recStart()" style="padding:18px;border-radius:12px;border:2px solid var(--red);background:transparent;color:var(--red);font-family:'Rajdhani',sans-serif;font-weight:700;font-size:1.1rem;cursor:pointer">⏺<br><span style="font-size:.8rem">START REC</span></button>
    <button onclick="recStop()" style="padding:18px;border-radius:12px;border:2px solid var(--orange);background:transparent;color:var(--orange);font-family:'Rajdhani',sans-serif;font-weight:700;font-size:1.1rem;cursor:pointer">⏹<br><span style="font-size:.8rem">STOP REC</span></button>
    <button id="btnPlay" onclick="recPlay()" style="padding:18px;border-radius:12px;border:2px solid var(--ca);background:transparent;color:var(--ca);font-family:'Rajdhani',sans-serif;font-weight:700;font-size:1.1rem;cursor:pointer">▶<br><span style="font-size:.8rem">PLAY</span></button>
    <button onclick="recStopPlay()" style="padding:18px;border-radius:12px;border:2px solid var(--dim);background:transparent;color:var(--dim);font-family:'Rajdhani',sans-serif;font-weight:700;font-size:1.1rem;cursor:pointer">⏸<br><span style="font-size:.8rem">STOP PLAY</span></button>
  </div>
</div>

<div class="panel">
  <div class="ph">// ARM QUICK CONTROLS</div>
  <div class="arm-grid">

    <div class="arm-ctrl">
      <div class="arm-name" style="color:#7b6cff">⬆ UP / DOWN <span class="badge">CH3+CH4</span></div>
      <div class="arm-btns">
        <button class="abtn" style="background:#7b6cff;color:#000"
          onmousedown="setUD(2000)" onmouseup="setUD(-1)" onmouseleave="setUD(-1)"
          ontouchstart="setUD(2000);event.preventDefault()" ontouchend="setUD(-1)">▲ UP</button>
        <button class="abtn" style="background:#7b6cff;color:#000"
          onmousedown="setUD(1500)" onmouseup="setUD(-1)" onmouseleave="setUD(-1)"
          ontouchstart="setUD(1500);event.preventDefault()" ontouchend="setUD(-1)">⊙ STP</button>
        <button class="abtn" style="background:#7b6cff;color:#000"
          onmousedown="setUD(1000)" onmouseup="setUD(-1)" onmouseleave="setUD(-1)"
          ontouchstart="setUD(1000);event.preventDefault()" ontouchend="setUD(-1)">▼ DWN</button>
        <button class="kbtn" onclick="setUD(-1)">🛑</button>
      </div>
      <div class="arm-val" id="ud-val">OFF</div>
    </div>

    <div class="arm-ctrl">
      <div class="arm-name" style="color:#a78bfa">↕ BEND <span class="badge">CH5 · FT5330M</span></div>
      <div class="arm-btns">
        <button class="abtn" style="background:#a78bfa;color:#000"
          onmousedown="holdMotor(5,2000)" onmouseup="holdMotor(5,-1)" onmouseleave="holdMotor(5,-1)"
          ontouchstart="holdMotor(5,2000);event.preventDefault()" ontouchend="holdMotor(5,-1)">▲ FWD</button>
        <button class="abtn" style="background:#a78bfa;color:#000"
          onmousedown="holdMotor(5,1500)" onmouseup="holdMotor(5,-1)" onmouseleave="holdMotor(5,-1)"
          ontouchstart="holdMotor(5,1500);event.preventDefault()" ontouchend="holdMotor(5,-1)">⊙ STP</button>
        <button class="abtn" style="background:#a78bfa;color:#000"
          onmousedown="holdMotor(5,1000)" onmouseup="holdMotor(5,-1)" onmouseleave="holdMotor(5,-1)"
          ontouchstart="holdMotor(5,1000);event.preventDefault()" ontouchend="holdMotor(5,-1)">▼ BCK</button>
        <button class="kbtn" onclick="holdMotor(5,-1);stopBendToggle()">🛑</button>
      </div>
      <!-- TOGGLE BUTTON CH5 -->
      <button id="bendTog" class="tog" onclick="toggleBend()">🔁 TOGGLE RUN (OFF)</button>
      <div class="arm-val" id="bend-val">OFF</div>
    </div>

    <div class="arm-ctrl">
      <div class="arm-name" style="color:#60a5fa">↻ BASE <span class="badge">CH6</span></div>
      <div class="arm-btns">
        <button class="abtn" style="background:#60a5fa;color:#000"
          onmousedown="holdMotor(6,1000)" onmouseup="holdMotor(6,-1)" onmouseleave="holdMotor(6,-1)"
          ontouchstart="holdMotor(6,1000);event.preventDefault()" ontouchend="holdMotor(6,-1)">◀ LEFT</button>
        <button class="abtn" style="background:#60a5fa;color:#000"
          onmousedown="holdMotor(6,1500)" onmouseup="holdMotor(6,-1)" onmouseleave="holdMotor(6,-1)"
          ontouchstart="holdMotor(6,1500);event.preventDefault()" ontouchend="holdMotor(6,-1)">⊙ STP</button>
        <button class="abtn" style="background:#60a5fa;color:#000"
          onmousedown="holdMotor(6,2000)" onmouseup="holdMotor(6,-1)" onmouseleave="holdMotor(6,-1)"
          ontouchstart="holdMotor(6,2000);event.preventDefault()" ontouchend="holdMotor(6,-1)">▶ RIGHT</button>
        <button class="kbtn" onclick="holdMotor(6,-1)">🛑</button>
      </div>
      <div class="arm-val" id="base-val">OFF</div>
    </div>

  </div>
</div>

<div class="sec-lbl">// TENTACLE CABLE WINCHES (CH0–CH2) — HOLD TO RUN</div>
<div class="grid" id="tentGrid"></div>

<div class="sec-lbl">// UP/DOWN FT5330M (CH3+CH4) — HOLD TO RUN · CH4 AUTO-INVERTED</div>
<div class="grid" id="udGrid"></div>

<div class="sec-lbl">// BEND & BASE (CH5–CH6) — HOLD TO RUN</div>
<div class="grid" id="armContGrid"></div>

<div class="sbar">
  <div><span class="dot"></span>IP: <span id="sip">—</span></div>
  <div>RSSI: <span id="srssi">—</span> dBm</div>
  <div>UP: <span id="sup">—</span>s</div>
  <div>MODE: <span id="smode">medium</span></div>
  <div>GRAB: <span id="sgrab">idle</span></div>
</div>
<div class="log" id="log"></div>

<script>
const WINCH=[
  {id:0,name:'Cable-A',sub:'Left curl',color:'#00ffc8'},
  {id:1,name:'Cable-B',sub:'Right curl',color:'#ff5e3a'},
  {id:2,name:'Cable-C',sub:'Grab/Wrap',color:'#ffcc00'},
  {id:5,name:'Bend',sub:'FT5330M',color:'#a78bfa'},
  {id:6,name:'Base',sub:'Rotation',color:'#60a5fa'},
];
const FORCES={soft:0.15,medium:0.38,hard:0.72};
let currentMode='medium';

function buildWinchCard(id,cid){
  const m=WINCH.find(x=>x.id===id);
  const hc=id<3;
  document.getElementById(cid).innerHTML+=`
  <div class="card" style="--mc:${m.color}">
    <div class="card-head">
      <div><div class="cname" style="color:${m.color}">${m.name}</div><div class="csub">${m.sub} — hold to run</div></div>
      <div class="cch">CH${id}</div>
    </div>
    <div class="ang-disp" id="av${id}" style="color:${m.color}">OFF</div>
    ${hc?`<div class="curt" id="curt${id}">— A</div>`:'<div style="height:1.1em"></div>'}
    <div class="hrow">
      <button class="hbtn" style="border-color:${m.color};color:${m.color}"
        onmousedown="holdMotor(${id},1000)" onmouseup="holdMotor(${id},-1)" onmouseleave="holdMotor(${id},-1)"
        ontouchstart="holdMotor(${id},1000);event.preventDefault()" ontouchend="holdMotor(${id},-1)">⏪<br><small>REV</small></button>
      <button style="flex:1;padding:10px 3px;border-radius:7px;border:1px solid #ff3355;background:transparent;color:#ff3355;font-family:'Rajdhani',sans-serif;font-weight:700;font-size:.78rem;cursor:pointer"
        onclick="holdMotor(${id},-1)">🛑<br><small>KILL</small></button>
      <button class="hbtn" style="border-color:${m.color};color:${m.color}"
        onmousedown="holdMotor(${id},2000)" onmouseup="holdMotor(${id},-1)" onmouseleave="holdMotor(${id},-1)"
        ontouchstart="holdMotor(${id},2000);event.preventDefault()" ontouchend="holdMotor(${id},-1)">⏩<br><small>FWD</small></button>
    </div>
  </div>`;
}

function buildUDCard(cid){
  document.getElementById(cid).innerHTML+=`
  <div class="card" style="--mc:#7b6cff">
    <div class="card-head">
      <div><div class="cname" style="color:#7b6cff">Up / Down (FT5330M)</div><div class="csub">CH3+CH4 · same command · CH4 auto-inverted</div></div>
      <div class="cch" style="color:#7b6cff">DUAL</div>
    </div>
    <div class="ang-disp" id="av3b" style="color:#7b6cff">OFF</div>
    <div class="hrow">
      <button class="hbtn" style="border-color:#7b6cff;color:#7b6cff"
        onmousedown="setUD(2000)" onmouseup="setUD(-1)" onmouseleave="setUD(-1)"
        ontouchstart="setUD(2000);event.preventDefault()" ontouchend="setUD(-1)">▲ UP<br><small>2000µs</small></button>
      <button class="hbtn" style="border-color:#7b6cff;color:#7b6cff"
        onmousedown="setUD(1500)" onmouseup="setUD(-1)" onmouseleave="setUD(-1)"
        ontouchstart="setUD(1500);event.preventDefault()" ontouchend="setUD(-1)">⊙ STOP<br><small>1500µs</small></button>
      <button class="hbtn" style="border-color:#7b6cff;color:#7b6cff"
        onmousedown="setUD(1000)" onmouseup="setUD(-1)" onmouseleave="setUD(-1)"
        ontouchstart="setUD(1000);event.preventDefault()" ontouchend="setUD(-1)">▼ DOWN<br><small>1000µs</small></button>
      <button style="flex:1;padding:10px 3px;border-radius:7px;border:1px solid #ff3355;background:transparent;color:#ff3355;font-family:'Rajdhani',sans-serif;font-weight:700;font-size:.78rem;cursor:pointer"
        onclick="setUD(-1)">🛑<br><small>KILL</small></button>
    </div>
  </div>`;
}

[0,1,2].forEach(id=>buildWinchCard(id,'tentGrid'));
buildUDCard('udGrid');
[5,6].forEach(id=>buildWinchCard(id,'armContGrid'));

function lg(msg,cls=''){
  const d=document.getElementById('log');
  const e=document.createElement('div');
  e.className='le '+cls;
  e.textContent=new Date().toLocaleTimeString()+' › '+msg;
  d.prepend(e);
  while(d.children.length>40) d.removeChild(d.lastChild);
}
async function post(ep,body){
  return fetch(ep,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});
}

async function holdMotor(id,val){
  try{
    await post('/api/motor',`id=${id}&angle=${val}`);
    const lbl=val<0?'OFF':val+'µs';
    const el=document.getElementById('av'+id);
    if(el) el.textContent=lbl;
    if(id===5) document.getElementById('bend-val').textContent=lbl;
    if(id===6) document.getElementById('base-val').textContent=lbl;
  }catch(e){lg('Error: '+e,'er');}
}

async function setUD(val){
  const lbl=val<0?'OFF':val+'µs';
  document.getElementById('ud-val').textContent=lbl;
  const ab=document.getElementById('av3b');
  if(ab) ab.textContent=lbl;
  try{ await post('/api/updown',`angle=${val}`); }
  catch(e){lg('Error: '+e,'er');}
}

// ════ TOGGLE FOR CH5 BEND ════
let bendOn=false;
let bendIv=null;

function stopBendToggle(){
  if(!bendOn) return;
  bendOn=false;
  clearInterval(bendIv); bendIv=null;
  const btn=document.getElementById('bendTog');
  btn.className='tog';
  btn.textContent='🔁 TOGGLE RUN (OFF)';
  document.getElementById('bend-val').textContent='OFF';
}

function toggleBend(){
  const btn=document.getElementById('bendTog');
  if(!bendOn){
    bendOn=true;
    btn.className='tog active';
    btn.textContent='🔁 TOGGLE RUN (ON) — click to stop';
    document.getElementById('bend-val').textContent='2000µs';
    post('/api/motor','id=5&angle=2000');
    bendIv=setInterval(()=>{ post('/api/motor','id=5&angle=2000'); },200);
    lg('Bend toggle ON → running forward','warn');
  } else {
    post('/api/motor','id=5&angle=-1');
    stopBendToggle();
    lg('Bend toggle OFF → zero power','ok');
  }
}

function setMode(mode){
  currentMode=mode;
  document.querySelectorAll('.modeBtn').forEach(b=>b.classList.remove('active'));
  document.querySelector('.modeBtn.'+mode).classList.add('active');
  document.getElementById('flim').textContent=FORCES[mode].toFixed(2)+'A';
  document.getElementById('flimbar').style.width=(FORCES[mode]/0.72*100)+'%';
  post('/api/mode',`mode=${mode}`);
  lg('Mode → '+mode,'warn');
}
async function autoGrab(){
  lg('▶ Auto grab — '+currentMode,'warn');
  try{ await post('/api/grab',`mode=${currentMode}`); lg('Done','ok'); }
  catch(e){lg('Error: '+e,'er');}
}
async function openGripper(){
  try{ await post('/api/open',''); lg('Open','ok'); }catch(e){lg('Error: '+e,'er');}
}
async function homeAll(){
  stopBendToggle();
  try{
    await post('/api/home','');
    ['ud-val','bend-val','base-val'].forEach(id=>document.getElementById(id).textContent='OFF');
    const ab=document.getElementById('av3b'); if(ab) ab.textContent='OFF';
    [0,1,2,5,6].forEach(id=>{const e=document.getElementById('av'+id);if(e)e.textContent='OFF';});
    lg('All → zero power','ok');
  }catch(e){lg('Error: '+e,'er');}
}
async function powerOff(){
  stopBendToggle();
  try{
    await post('/api/poweroff','');
    ['ud-val','bend-val','base-val'].forEach(id=>document.getElementById(id).textContent='OFF');
    const ab=document.getElementById('av3b'); if(ab) ab.textContent='OFF';
    [0,1,2,5,6].forEach(id=>{const e=document.getElementById('av'+id);if(e)e.textContent='OFF';});
    lg('⚡ ALL ZERO POWER','warn');
  }catch(e){lg('Error: '+e,'er');}
}

function updateForceSlider(v){
  const a=(parseInt(v)/100).toFixed(2);
  document.getElementById('forceSliderVal').textContent=a+'A';
  document.getElementById('flim').textContent=a+'A';
  document.getElementById('flimbar').style.width=(parseInt(v)/200*100)+'%';
}
async function sendForce(){
  const v=document.getElementById('forceSlider').value;
  const a=(parseInt(v)/100).toFixed(2);
  try{ await post('/api/force',`limit=${a}`); lg('Force → '+a+'A','warn'); }
  catch(e){lg('Error: '+e,'er');}
}

async function recStart()   {try{await post('/api/rec/start','');   lg('⏺ Recording','warn');}catch(e){}}
async function recStop()    {try{await post('/api/rec/stop','');    lg('⏹ Stopped','ok');    }catch(e){}}
async function recPlay()    {try{await post('/api/rec/play','');    lg('▶ Replay','ok');      }catch(e){}}
async function recStopPlay(){try{await post('/api/rec/stopplay','');lg('⏸ Paused','warn');   }catch(e){}}

async function poll(){
  try{
    const d=await(await fetch('/api/status')).json();
    document.getElementById('sip').textContent=d.ip;
    document.getElementById('srssi').textContent=d.rssi;
    document.getElementById('sup').textContent=d.uptime;
    document.getElementById('smode').textContent=d.grabMode;
    document.getElementById('sgrab').textContent=d.autoGrab?'GRABBING':'idle';
    d.motors.forEach(m=>{
      if(m.current!==null&&m.id<3){
        const a=Math.abs(m.current),pct=Math.min(100,(a/0.72)*100);
        document.getElementById('cc'+m.id).textContent=m.current.toFixed(3)+'A';
        document.getElementById('cb'+m.id).style.width=pct+'%';
        const cu=document.getElementById('curt'+m.id);
        if(cu) cu.textContent=m.current.toFixed(3)+' A';
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
    bR.style.background='transparent'; bP.style.background='transparent';
    if(d.recording){dot.className='rec-dot recording';sta.textContent='● RECORDING...';sta.style.color='#ff3355';bR.style.background='#ff335533';}
    else if(d.replaying){dot.className='rec-dot replaying';sta.textContent='▶ REPLAYING...';sta.style.color='#00ffc8';bP.style.background='#00ffc833';}
    else{dot.className='rec-dot idle';sta.textContent=d.frames>0?`READY — ${d.frames} frames`:'IDLE';sta.style.color=d.frames>0?'#00ffc8':'var(--dim)';}
  }catch(e){}
},600);

lg('OctoGrip v6 ready 🐙','ok');
</script>
</body>
</html>)HTML");
  server.sendContent("");
}

void handleStatus()   { server.send(200,"application/json",buildJSON()); }
void handleSetMotor() {
  if(!server.hasArg("id")||!server.hasArg("angle")){server.send(400,"application/json","{\"error\":\"Missing args\"}");return;}
  setMotor(server.arg("id").toInt(),server.arg("angle").toInt());
  server.send(200,"application/json",buildJSON());
}
void handleSetUpDown() {
  if(!server.hasArg("angle")){server.send(400,"application/json","{\"error\":\"Missing angle\"}");return;}
  setUpDown(server.arg("angle").toInt());
  server.send(200,"application/json",buildJSON());
}
void handleSetMode() {
  if(server.hasArg("mode")){
    grabMode=server.arg("mode");
    if(grabMode=="soft")        forceLimit=FORCE_SOFT;
    else if(grabMode=="medium") forceLimit=FORCE_MEDIUM;
    else if(grabMode=="hard")   forceLimit=FORCE_HARD;
  }
  server.send(200,"application/json",buildJSON());
}
void handleAutoGrab() {
  if(server.hasArg("mode")){
    grabMode=server.arg("mode");
    if(grabMode=="soft")        forceLimit=FORCE_SOFT;
    else if(grabMode=="medium") forceLimit=FORCE_MEDIUM;
    else if(grabMode=="hard")   forceLimit=FORCE_HARD;
  }
  server.send(200,"application/json",buildJSON());
  autoGrab();
}
void handleOpen()     { server.send(200,"application/json",buildJSON()); openGripper(); }
void handleHome()     { server.send(200,"application/json",buildJSON()); homeAll(); }
void handleSetForce() {
  if(server.hasArg("limit")) forceLimit=constrain(server.arg("limit").toFloat(),0.05f,2.0f);
  server.send(200,"application/json",buildJSON());
}
void handleRecStart()    { startRecording();  server.send(200,"application/json",buildJSON()); }
void handleRecStop()     { stopRecording();   server.send(200,"application/json",buildJSON()); }
void handleRecPlay()     { server.send(200,"application/json",buildJSON()); replayRecording(); }
void handleRecStopPlay() { stopReplay();      server.send(200,"application/json",buildJSON()); }
void handleRecStatus() {
  String j="{\"recording\":"+String(isRecording?"true":"false")+",\"replaying\":"+String(isReplaying?"true":"false")+",\"frames\":"+String(recordCount)+",\"maxFrames\":"+String(MAX_FRAMES)+"}";
  server.send(200,"application/json",j);
}
void handleNotFound() { server.send(404,"application/json","{\"error\":\"Not found\"}"); }

void setup() {
  Serial.begin(115200);
  unsigned long _t=millis();
  while(!Serial&&millis()-_t<3000);
  delay(200);
  Serial.println("\n>>> OCTOGRIP v6 <<<");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  recording=psramFound()?(Frame*)ps_malloc(MAX_FRAMES*sizeof(Frame)):new Frame[MAX_FRAMES];
  if(!recording) Serial.println("[MEM] ERROR");
  else Serial.printf("[MEM] %d frames OK\n",MAX_FRAMES);

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
    Serial.println("[MOTORS] All 7 ZERO POWER at boot");
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

  server.on("/",               HTTP_GET,  handleRoot);
  server.on("/api/status",     HTTP_GET,  handleStatus);
  server.on("/api/motor",      HTTP_POST, handleSetMotor);
  server.on("/api/updown",     HTTP_POST, handleSetUpDown);
  server.on("/api/mode",       HTTP_POST, handleSetMode);
  server.on("/api/grab",       HTTP_POST, handleAutoGrab);
  server.on("/api/open",       HTTP_POST, handleOpen);
  server.on("/api/home",       HTTP_POST, handleHome);
  server.on("/api/poweroff",   HTTP_POST, [](){ disableAll(); server.send(200,"application/json",buildJSON()); });
  server.on("/api/force",      HTTP_POST, handleSetForce);
  server.on("/api/rec/start",  HTTP_POST, handleRecStart);
  server.on("/api/rec/stop",   HTTP_POST, handleRecStop);
  server.on("/api/rec/play",   HTTP_POST, handleRecPlay);
  server.on("/api/rec/stopplay",HTTP_POST,handleRecStopPlay);
  server.on("/api/rec/status", HTTP_GET,  handleRecStatus);
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
    Serial.printf("[CURR] A=%.3f B=%.3f C=%.3f Limit=%.2f\n",cabCurrent[0],cabCurrent[1],cabCurrent[2],forceLimit);
  }
  static unsigned long lastWifi=0;
  if(millis()-lastWifi>=10000){
    lastWifi=millis();
    if(WiFi.status()!=WL_CONNECTED) WiFi.reconnect();
  }
}