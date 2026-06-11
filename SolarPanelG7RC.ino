#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <esp_wifi.h>

// ==================== WiFi AP ====================
const char* ssid = "ESP32_Car";
const char* password = "password123";

WebServer server(80);

#define RELAY_PIN 17

// ==================== Motor Pins ====================
// Left motor  → GPIO 18 (PWM), 19 (DIR)
// Right motor → GPIO 25 (PWM), 26 (DIR)
const int PWM_L = 18;
const int DIR_L = 19;
const int PWM_R = 25;
const int DIR_R = 26;

bool isPumpOn = false;
// Helper function to control Active-LOW relay pin using High-Impedance trick
void setPumpState(bool turnOn) {
  isPumpOn = turnOn;
  if (turnOn) {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); // Relay active (Pump ON)
  } else {
    pinMode(RELAY_PIN, INPUT);    // High impedance to turn optocoupler OFF completely
  }
}


// ---- Cytron SPG20HP-34K Speed Limits ----
// Motor: 12V 580RPM — much faster than TT motors.
// MAX_PWM: hard ceiling on duty cycle (out of 255).
//   60  ≈ 24% duty — slow & controllable for a solar panel tracker.
//   Raise gradually if more speed is needed (max recommended: 100).
// MIN_PWM: minimum duty to overcome motor stiction and actually spin.
//   If motor hums but doesn't move, raise MIN_PWM slightly.
const int MAX_PWM = 60;   // ← main speed limiter (0–255)
const int MIN_PWM = 30;   // ← dead-band / stiction threshold

// ==================== FS80NK Cliff Sensors ====================
// Sensor pins assigned to free GPIOs with internal pull-up support
const int PIN_FS80NK_FL = 32; // Front-Left
const int PIN_FS80NK_FR = 33; // Front-Right
const int PIN_FS80NK_BL = 27; // Back-Left
const int PIN_FS80NK_BR = 14; // Back-Right

// Sensor logic: FS80NK outputs LOW when detecting surface (Safe), HIGH when open air (Cliff)
#define CLIFF_STATE HIGH

// ==================== Safety Evasion State Machine ====================
enum SafetyState {
  STATE_NORMAL,
  STATE_FRONT_EVADE,
  STATE_BACK_EVADE,
  STATE_LEFT_EVADE_TURN,
  STATE_LEFT_EVADE_STRAIGHT,
  STATE_RIGHT_EVADE_TURN,
  STATE_RIGHT_EVADE_STRAIGHT,
  STATE_STOP_WAIT
};

SafetyState safetyState = STATE_NORMAL;
unsigned long safetyTimer = 0;
bool safetyModeActive = false; // Normal Mode = false (Safety bypassed), Safety/Cleaning Mode = true

// ==================== Playback & Preset Globals ====================
#define MAX_STEPS 3000
struct DriveStep {
  int16_t left;
  int16_t right;
  uint8_t pump;
};
DriveStep recordedSequence[MAX_STEPS];
int sequenceLength = 0;
int playbackIndex = 0;
bool isPlayingPlayback = false;
bool isPausedPlayback = false;
unsigned long lastPlaybackStepTime = 0;
const unsigned long PLAYBACK_STEP_INTERVAL = 100; // 10 Hz (100ms)

// ==================== WiFi & Latency Diagnostics ====================
int lastLoggedRSSI = 0; // 0 = unlogged, 1 = normal, 2 = weak
int lastLoggedRTT = 0;  // 0 = normal, 1 = high latency

int getClientRSSI() {
  wifi_sta_list_t wifi_sta_list;
  memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
  if (esp_wifi_ap_get_sta_list(&wifi_sta_list) == ESP_OK) {
    if (wifi_sta_list.num > 0) {
      return wifi_sta_list.sta[0].rssi;
    }
  }
  return 0; // 0 dBm means unavailable
}

// ==================== Event Log Buffer ====================
struct LogEntry {
  char msg[80];
  unsigned long ts;
};
#define LOG_BUFFER_SIZE 20
LogEntry logBuffer[LOG_BUFFER_SIZE];
int logHead = 0;
int logCount = 0;

// ==================== Dashboard HTML ====================
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>G7 Solar Robot Safety Dashboard</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:'Segoe UI',sans-serif;background:#0a0a1a;color:#e0e0e0;
      min-height:100vh;padding:16px;background:linear-gradient(135deg,#0a0a1a,#1a1a2e,#0d0d20);
      background-size:400% 400%;animation:bg 15s ease infinite}
    @keyframes bg{0%,100%{background-position:0% 50%}50%{background-position:100% 50%}}
    h1{text-align:center;font-size:24px;margin-bottom:16px;
      background:linear-gradient(90deg,#bb86fc,#03dac6);-webkit-background-clip:text;
      -webkit-text-fill-color:transparent;letter-spacing:1px}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px;max-width:960px;margin:0 auto}
    .card{background:rgba(255,255,255,0.04);backdrop-filter:blur(12px);
      border:1px solid rgba(255,255,255,0.08);border-radius:16px;padding:18px;
      box-shadow:0 8px 32px rgba(0,0,0,0.3);transition:border-color .3s}
    .card:hover{border-color:rgba(0,212,255,0.25)}
    .card h2{font-size:15px;margin-bottom:12px;color:#bb86fc;display:flex;align-items:center;gap:8px}
    .card h2 span{font-size:18px}
    .full{grid-column:1/-1}

    /* Gamepad */
    #status{font-size:16px;color:#ff5252;padding:12px;border-radius:10px;background:rgba(0,0,0,0.3);
      text-align:center;margin-bottom:8px}
    #joystick{text-align:center;font-family:monospace;font-size:16px;color:#03dac6}
    .hint{text-align:center;font-size:12px;color:#555;margin-top:6px}

    /* Robot Visualizer styling */
    .visualizer-container {
      display: flex;
      justify-content: center;
      align-items: center;
      padding: 30px 0;
      background: rgba(0,0,0,0.25);
      border-radius: 12px;
      margin-bottom: 12px;
    }
    .robot-top-view {
      position: relative;
      width: 170px;
      height: 170px;
      background: linear-gradient(135deg, #1e1e38, #2a2a4e);
      border: 3px solid rgba(255,255,255,0.15);
      border-radius: 24px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.5), inset 0 0 15px rgba(255,255,255,0.05);
      display: flex;
      flex-direction: column;
      justify-content: center;
      align-items: center;
    }
    /* Front cleaning roller */
    .roller-brush.front {
      position: absolute;
      top: -12px;
      left: 20px;
      right: 20px;
      height: 12px;
      background: repeating-linear-gradient(45deg, #03dac6, #03dac6 6px, #018786 6px, #018786 12px);
      border: 1px solid rgba(255,255,255,0.2);
      border-radius: 6px;
      box-shadow: 0 4px 10px rgba(3, 218, 198, 0.4);
    }
    /* Solar cell grid pattern on the robot body */
    .solar-panel-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      grid-template-rows: 1fr 1fr;
      gap: 4px;
      width: 110px;
      height: 110px;
      opacity: 0.65;
    }
    .solar-cell {
      background: rgba(0, 212, 255, 0.15);
      border: 1px solid rgba(0, 212, 255, 0.3);
      border-radius: 4px;
      position: relative;
    }
    .solar-cell::after {
      content: '';
      position: absolute;
      top: 0; left: 0; right: 0; bottom: 0;
      background: linear-gradient(135deg, rgba(255,255,255,0.1) 0%, transparent 50%);
    }
    /* Direction text */
    .direction-indicator {
      position: absolute;
      bottom: 15px;
      font-size: 10px;
      font-weight: 800;
      color: rgba(255, 255, 255, 0.4);
      letter-spacing: 1.5px;
    }
    /* Sensor indicator lights at 4 corners */
    .sensor-indicator {
      position: absolute;
      width: 26px;
      height: 26px;
      border-radius: 8px;
      border: 2px solid rgba(255,255,255,0.3);
      display: flex;
      justify-content: center;
      align-items: center;
      font-size: 9px;
      font-weight: bold;
      color: #fff;
      text-shadow: 0 1px 2px rgba(0,0,0,0.5);
      transition: background-color 0.2s, box-shadow 0.2s, border-color 0.2s;
    }
    .sensor-indicator.fl { top: -8px; left: -8px; }
    .sensor-indicator.fr { top: -8px; right: -8px; }
    .sensor-indicator.bl { bottom: -8px; left: -8px; }
    .sensor-indicator.br { bottom: -8px; right: -8px; }

    /* Sensor States */
    .sensor-indicator.safe {
      background: #00e676;
      border-color: #ffffff;
      box-shadow: 0 0 12px #00e676;
      animation: breathe-green 2s infinite alternate;
    }
    .sensor-indicator.cliff {
      background: #ff1744;
      border-color: #ffffff;
      box-shadow: 0 0 18px #ff1744;
      animation: blink-red 0.3s infinite alternate;
    }

    @keyframes breathe-green {
      0% { opacity: 0.8; box-shadow: 0 0 6px rgba(0, 230, 118, 0.4); }
      100% { opacity: 1; box-shadow: 0 0 14px rgba(0, 230, 118, 0.9); }
    }
    @keyframes blink-red {
      0% { opacity: 0.35; box-shadow: 0 0 4px rgba(255, 23, 68, 0.3); }
      100% { opacity: 1; box-shadow: 0 0 22px rgba(255, 23, 68, 1); }
    }

    /* Diagnostics styling */
    .sensor-list {
      margin-bottom: 15px;
    }
    .sensor-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 8px 12px;
      background: rgba(255,255,255,0.02);
      border-radius: 8px;
      margin-bottom: 6px;
      border: 1px solid rgba(255,255,255,0.03);
    }
    .sensor-label {
      font-size: 13px;
      color: #aaa;
    }
    .sensor-badge {
      font-size: 11px;
      font-weight: bold;
      padding: 3px 8px;
      border-radius: 12px;
      text-transform: uppercase;
    }
    .badge-safe {
      background: rgba(0, 230, 118, 0.15);
      color: #00e676;
      border: 1px solid rgba(0, 230, 118, 0.3);
    }
    .badge-cliff {
      background: rgba(255, 23, 68, 0.15);
      color: #ff1744;
      border: 1px solid rgba(255, 23, 68, 0.3);
      animation: text-pulse 0.6s infinite alternate;
    }
    @keyframes text-pulse {
      0% { opacity: 0.7; }
      100% { opacity: 1; }
    }

    /* Safety status box styling */
    .safety-status-box {
      text-align: center;
      font-size: 14px;
      font-weight: bold;
      color: #ccc;
      padding: 10px;
      background: rgba(0,0,0,0.2);
      border-radius: 8px;
      border: 1px solid rgba(255,255,255,0.05);
    }
    .status-normal {
      color: #00e676;
    }
    .status-warning {
      color: #ffc107;
      animation: text-pulse 0.6s infinite alternate;
    }
    .status-danger {
      color: #ff1744;
      animation: text-pulse 0.4s infinite alternate;
    }

    /* Safety override alert banner */
    .safety-banner {
      position: fixed;
      top: 0; left: 0; right: 0;
      padding: 12px;
      text-align: center;
      background: linear-gradient(90deg, #d50000, #ff1744);
      color: #fff;
      font-weight: bold;
      font-size: 14px;
      transform: translateY(-100%);
      transition: transform 0.3s cubic-bezier(0.175, 0.885, 0.32, 1.275);
      z-index: 1000;
      box-shadow: 0 4px 20px rgba(213, 0, 0, 0.5);
      letter-spacing: 0.5px;
    }
    .safety-banner.show {
      transform: translateY(0);
    }

    /* Troubleshooting card lists */
    .troubleshoot-guide {
      margin-top: 15px;
      padding-top: 12px;
      border-top: 1px solid rgba(255,255,255,0.05);
    }
    .troubleshoot-guide h3 {
      font-size: 12px;
      color: #bb86fc;
      margin-bottom: 8px;
    }
    .troubleshoot-guide ul {
      list-style-type: none;
      font-size: 11px;
      color: #888;
      line-height: 1.5;
    }
    .troubleshoot-guide li {
      margin-bottom: 6px;
      padding-left: 10px;
      position: relative;
    }
    .troubleshoot-guide li::before {
      content: '•';
      color: #03dac6;
      position: absolute;
      left: 0;
    }

    /* Preset Recorder Controls */
    .preset-controls {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-bottom: 15px;
      justify-content: center;
    }
    .pbtn {
      padding: 10px 20px;
      font-size: 14px;
      font-weight: bold;
      border: none;
      border-radius: 8px;
      cursor: pointer;
      transition: all 0.2s ease;
      color: #fff;
      display: flex;
      align-items: center;
      gap: 6px;
    }
    .pbtn:disabled {
      opacity: 0.3;
      cursor: not-allowed;
      transform: none !important;
      box-shadow: none !important;
    }
    .record-btn {
      background: linear-gradient(135deg, #ff1744, #d50000);
      box-shadow: 0 4px 10px rgba(255, 23, 68, 0.3);
    }
    .record-btn.recording {
      animation: record-pulse 1s infinite alternate;
      background: #b71c1c;
    }
    .play-btn {
      background: linear-gradient(135deg, #00e676, #00c853);
      box-shadow: 0 4px 10px rgba(0, 230, 118, 0.3);
    }
    .play-btn.playing {
      background: linear-gradient(135deg, #ffc107, #ffb300);
      box-shadow: 0 4px 10px rgba(255, 193, 7, 0.3);
    }
    .stop-btn {
      background: linear-gradient(135deg, #2196f3, #1976d2);
      box-shadow: 0 4px 10px rgba(33, 150, 243, 0.3);
    }
    .clear-btn {
      background: linear-gradient(135deg, #757575, #424242);
    }
    .pbtn:hover:not(:disabled) {
      transform: translateY(-2px);
      box-shadow: 0 6px 15px rgba(255,255,255,0.1);
    }
    
    @keyframes record-pulse {
      0% { box-shadow: 0 0 4px rgba(255, 23, 68, 0.5); opacity: 0.8; }
      100% { box-shadow: 0 0 16px rgba(255, 23, 68, 1); opacity: 1; }
    }

    /* Timeline bar styling */
    .timeline-container {
      background: rgba(0,0,0,0.3);
      border: 1px solid rgba(255,255,255,0.06);
      border-radius: 10px;
      padding: 12px;
      margin-bottom: 12px;
      width: 100%;
    }
    .timeline-label {
      display: flex;
      justify-content: space-between;
      font-size: 13px;
      color: #ccc;
      margin-bottom: 8px;
      font-family: monospace;
    }
    .timeline-bar {
      height: 10px;
      background: rgba(255,255,255,0.06);
      border-radius: 5px;
      overflow: hidden;
      position: relative;
    }
    .timeline-progress {
      height: 100%;
      width: 0%;
      background: linear-gradient(90deg, #bb86fc, #03dac6);
      border-radius: 5px;
      transition: width 0.1s linear;
    }

    /* ===== System Health Bar ===== */
    .health-bar {
      display:flex;align-items:center;justify-content:space-between;gap:12px;
      max-width:960px;margin:0 auto 16px auto;padding:12px 18px;
      background:rgba(255,255,255,0.04);border:1px solid rgba(255,255,255,0.08);
      border-radius:14px;backdrop-filter:blur(12px);flex-wrap:wrap;
    }
    .health-pill {
      display:flex;align-items:center;gap:8px;font-size:14px;font-weight:bold;
      padding:6px 16px;border-radius:20px;border:2px solid;transition:all .3s;
    }
    .health-ok   { color:#00e676;border-color:#00e676;background:rgba(0,230,118,.1); }
    .health-warn { color:#ffc107;border-color:#ffc107;background:rgba(255,193,7,.1);animation:text-pulse .8s infinite alternate; }
    .health-fault{ color:#ff1744;border-color:#ff1744;background:rgba(255,23,68,.1);animation:text-pulse .4s infinite alternate; }
    .health-meta { display:flex;align-items:center;gap:18px;flex-wrap:wrap; }
    .meta-item   { display:flex;flex-direction:column;align-items:center;font-size:11px;color:#888;gap:3px; }
    .meta-value  { font-size:13px;font-weight:bold;color:#ccc;font-family:monospace; }
    .mbar-row    { display:flex;align-items:center;gap:6px;font-size:11px;color:#888; }
    .mbar-track  { width:55px;height:7px;background:rgba(255,255,255,0.08);border-radius:4px;overflow:hidden; }
    .mbar-fill   { height:100%;background:linear-gradient(90deg,#bb86fc,#03dac6);border-radius:4px;transition:width .12s;width:0%; }
    /* Signal / latency bars */
    .sig-bars { display:flex;align-items:flex-end;gap:3px;height:20px; }
    .sig-bar  { width:5px;border-radius:2px;background:rgba(255,255,255,0.1);transition:background .3s; }
    .sig-bar.s-on   { background:#00e676; }
    .sig-bar.s-warn { background:#ffc107; }
    .sig-bar.s-poor { background:#ff1744; }
    .sig-bar:nth-child(1){height:4px}.sig-bar:nth-child(2){height:8px}
    .sig-bar:nth-child(3){height:12px}.sig-bar:nth-child(4){height:16px}.sig-bar:nth-child(5){height:20px}
    /* E-STOP */
    .estop-btn {
      position:fixed;top:14px;right:14px;z-index:2000;
      padding:11px 22px;background:linear-gradient(135deg,#b71c1c,#ff1744);
      color:#fff;font-size:15px;font-weight:900;border:3px solid #ff5252;
      border-radius:12px;cursor:pointer;
      box-shadow:0 0 22px rgba(255,23,68,.55);letter-spacing:1px;transition:all .15s;
    }
    .estop-btn:hover  { transform:scale(1.06);box-shadow:0 0 32px rgba(255,23,68,.9); }
    .estop-btn:active { transform:scale(0.96); }
    @keyframes estop-flash {
      0%,100%{background:linear-gradient(135deg,#b71c1c,#ff1744);color:#fff;}
      50%{background:#fff;color:#ff1744;}
    }
    .estop-btn.fired { animation:estop-flash .2s 3; }
    /* Event Log */
    .log-wrap {
      max-height:200px;overflow-y:auto;background:rgba(0,0,0,0.25);
      border-radius:8px;padding:8px;font-family:monospace;font-size:12px;
      scrollbar-width:thin;scrollbar-color:rgba(255,255,255,0.1) transparent;
    }
    .log-entry {
      display:flex;gap:10px;padding:4px 8px;border-radius:4px;margin-bottom:3px;
      border-left:3px solid;animation:fade-in .3s ease;
    }
    @keyframes fade-in{from{opacity:0;transform:translateY(-4px)}to{opacity:1;transform:translateY(0)}}
    .ev-cliff  { border-color:#ff1744;background:rgba(255,23,68,.07);color:#ff8a80; }
    .ev-safety { border-color:#ffc107;background:rgba(255,193,7,.07);color:#ffd54f; }
    .ev-pump   { border-color:#3498db;background:rgba(52,152,219,.07);color:#90caf9; }
    .ev-ok     { border-color:#00e676;background:rgba(0,230,118,.07);color:#a5d6a7; }
    .ev-wifi   { border-color:#9c27b0;background:rgba(156,39,176,.07);color:#e040fb; }
    .ev-rec    { border-color:#e65100;background:rgba(230,81,0,.07);color:#ffb74d; }
    .ev-play   { border-color:#00e5ff;background:rgba(0,229,255,.07);color:#84ffff; }
    .log-ts    { color:#555;min-width:58px; }
    .log-acts  { display:flex;gap:8px;margin-top:10px;justify-content:flex-end; }
    .log-btn {
      padding:5px 12px;font-size:11px;border:1px solid rgba(255,255,255,0.1);
      border-radius:6px;background:rgba(255,255,255,0.05);color:#aaa;
      cursor:pointer;transition:all .2s;
    }
    .log-btn:hover { background:rgba(255,255,255,0.1);color:#fff; }
  </style>
</head>
<body>
  <div id="safetyBanner" class="safety-banner">&#x26A0;&#xFE0F; SAFETY INTERRUPT: ROBOT REPOSITIONING...</div>

  <button id="btnEstop" class="estop-btn" onclick="triggerEstop()">&#x26D4; E-STOP</button>

  <div class="health-bar">
    <div id="healthPill" class="health-pill health-ok">&#x1F7E2; ALL SYSTEMS OK</div>
    <div class="health-meta">
      <div class="meta-item">
        <span class="meta-value" id="uptimeVal">00:00:00</span>
        <span>Session Uptime</span>
      </div>
      <div class="meta-item">
        <div class="mbar-row"><span>L</span><div class="mbar-track"><div id="motorBarL" class="mbar-fill"></div></div></div>
        <div class="mbar-row"><span>R</span><div class="mbar-track"><div id="motorBarR" class="mbar-fill"></div></div></div>
        <span>Motor PWM</span>
      </div>
      <div class="meta-item">
        <div class="sig-bars">
          <div class="sig-bar" id="sig1"></div>
          <div class="sig-bar" id="sig2"></div>
          <div class="sig-bar" id="sig3"></div>
          <div class="sig-bar" id="sig4"></div>
          <div class="sig-bar" id="sig5"></div>
        </div>
        <span id="rttVal">-- ms</span>
        <span>Latency</span>
      </div>
      <div class="meta-item">
        <span class="meta-value" id="rssiVal">-- dBm</span>
        <span id="clientVal" style="font-size:10px;color:#888;font-family:monospace;">0 clients</span>
        <span>WiFi Signal</span>
      </div>
    </div>
  </div>

  <h1>&#x2699;&#xFE0F; G7 Solar Panel Robot Dashboard</h1>

  <div class="grid">
    <!-- Gamepad Card -->
    <div class="card full">
      <h2><span>🎮</span> Gamepad Control</h2>
      <div id="status">Waiting for Gamepad...<br>Press any button on your controller.</div>
      <div id="joystick">Left Motor: 0 | Right Motor: 0 | Pump: OFF</div>
      <div class="hint">Use Left Stick or D-Pad to drive. X: Pump ON, Y: Pump OFF</div>
    </div>

    <!-- Preset Recorder & Playback Card -->
    <div class="card full">
      <h2><span>🎙️</span> Preset Recorder &amp; Playback</h2>
      <div class="preset-controls">
        <button id="btnRecord" class="pbtn record-btn" onclick="toggleRecording()">Record Preset</button>
        <button id="btnPlay" class="pbtn play-btn" onclick="togglePlayback()" disabled>Play</button>
        <button id="btnStop" class="pbtn stop-btn" onclick="stopPlayback()" disabled>Stop</button>
        <button id="btnClear" class="pbtn clear-btn" onclick="clearPreset()" disabled>Clear</button>
      </div>
      <div class="timeline-container">
        <div class="timeline-label">
          <span>Status: <strong id="recStatus" style="color:#aaa">IDLE</strong></span>
          <span id="timeCounter">Steps: 0 / 3000 (0.0s)</span>
        </div>
        <div class="timeline-bar">
          <div id="timelineProgress" class="timeline-progress"></div>
        </div>
      </div>
      <div class="hint">
        <strong>Gamepad Shortcuts:</strong> Press <strong>Button A</strong> to start/stop recording. Press <strong>Button B</strong> to play/pause playback.
      </div>
    </div>

    <!-- Top-Down Visualizer Card -->
    <div class="card">
      <h2><span>🤖</span> Top-Down Robot Visualizer</h2>
      <div class="visualizer-container">
        <div class="robot-top-view">
          <div class="roller-brush front"></div>
          <div class="solar-panel-grid">
            <div class="solar-cell"></div>
            <div class="solar-cell"></div>
            <div class="solar-cell"></div>
            <div class="solar-cell"></div>
          </div>
          <div class="direction-indicator">▲ FRONT</div>
          <div id="sensor-fl" class="sensor-indicator fl" title="Front-Left Sensor">FL</div>
          <div id="sensor-fr" class="sensor-indicator fr" title="Front-Right Sensor">FR</div>
          <div id="sensor-bl" class="sensor-indicator bl" title="Back-Left Sensor">BL</div>
          <div id="sensor-br" class="sensor-indicator br" title="Back-Right Sensor">BR</div>
        </div>
      </div>
      <div class="safety-status-box" style="display:flex; flex-direction:column; gap:8px;">
        <div>System Status: <span id="safetyStateText" class="status-normal">NORMAL</span></div>
        <div>Safety Mode: <span id="safetyModeText" style="font-weight:bold; color:#ff5252;">BYPASSED (NORMAL)</span></div>
        <button id="btnToggleSafety" class="pbtn" style="background:linear-gradient(135deg,#00e676,#00c853); margin:0 auto 4px auto; padding:6px 12px; font-size:12px;" onclick="toggleSafetyMode()">Enable Safety/Cleaning</button>
        <div style="border-top:1px solid rgba(255,255,255,0.08); padding-top:6px; margin-top:4px;">
          <div>Water Pump: <span id="pumpStateText" style="font-weight:bold; color:#ff5252;">OFF</span></div>
          <button id="btnTogglePump" class="pbtn" style="background:linear-gradient(135deg,#3498db,#2980b9); margin:6px auto 0 auto; padding:6px 12px; font-size:12px;" onclick="togglePump()">Turn Pump ON</button>
        </div>
      </div>
    </div>

    <!-- Sensor Details & Troubleshooting Card -->
    <div class="card">
      <h2><span>🔍</span> Sensor Diagnostics & Troubleshooting</h2>
      <div class="sensor-list">
        <div class="sensor-row">
          <span class="sensor-label">Front-Left (GPIO 32)</span>
          <span id="txt-fl" class="sensor-badge badge-safe">SAFE</span>
        </div>
        <div class="sensor-row">
          <span class="sensor-label">Front-Right (GPIO 33)</span>
          <span id="txt-fr" class="sensor-badge badge-safe">SAFE</span>
        </div>
        <div class="sensor-row">
          <span class="sensor-label">Back-Left (GPIO 27)</span>
          <span id="txt-bl" class="sensor-badge badge-safe">SAFE</span>
        </div>
        <div class="sensor-row">
          <span class="sensor-label">Back-Right (GPIO 14)</span>
          <span id="txt-br" class="sensor-badge badge-safe">SAFE</span>
        </div>
      </div>
      <div class="troubleshoot-guide">
        <h3>Troubleshooting Guide:</h3>
        <ul>
          <li><strong>Green Indicator:</strong> Surface detected (Normal Operation).</li>
          <li><strong>Blinking Red Indicator:</strong> Cliff detected or sensor blocked.</li>
          <li>Ensure the infrared transmitter/receiver window is clean and free of dust.</li>
          <li>Use the potentiometer on the back of each FS80NK sensor to adjust the distance threshold.</li>
        </ul>
      </div>
    </div>

    <!-- Gamepad Input Visualizer & Guide Card -->
    <div class="card">
      <h2><span>🎮</span> Controller Diagnostics &amp; Guide</h2>
      <div style="display:flex; flex-direction:column; align-items:center; gap:12px;">
        <canvas id="controllerCanvas" width="320" height="160" style="background:rgba(0,0,0,0.2); border-radius:12px; border:1px solid rgba(255,255,255,0.05); width:100%; max-width:320px;"></canvas>
        <div style="width:100%; border-top:1px solid rgba(255,255,255,0.08); padding-top:10px;">
          <h3 style="font-size:12px; color:#bb86fc; margin-bottom:8px;">Gamepad Command Guide:</h3>
          <table style="width:100%; font-size:11px; color:#aaa; border-collapse:collapse; line-height:1.6;">
            <tr style="border-bottom:1px solid rgba(255,255,255,0.03);">
              <td style="padding:4px 0; font-weight:bold; color:#03dac6;">Left Stick / D-Pad</td>
              <td style="padding:4px 0; text-align:right; color:#eee;">Drive Robot (PWM Speed)</td>
            </tr>
            <tr style="border-bottom:1px solid rgba(255,255,255,0.03);">
              <td style="padding:4px 0; font-weight:bold; color:#3498db;">Button X (Blue)</td>
              <td style="padding:4px 0; text-align:right; color:#eee;">Water Pump ON</td>
            </tr>
            <tr style="border-bottom:1px solid rgba(255,255,255,0.03);">
              <td style="padding:4px 0; font-weight:bold; color:#ffb300;">Button Y (Yellow)</td>
              <td style="padding:4px 0; text-align:right; color:#eee;">Water Pump OFF</td>
            </tr>
            <tr style="border-bottom:1px solid rgba(255,255,255,0.03);">
              <td style="padding:4px 0; font-weight:bold; color:#2ecc71;">Button A (Green)</td>
              <td style="padding:4px 0; text-align:right; color:#eee;">Record/Stop Preset (A)</td>
            </tr>
            <tr style="border-bottom:1px solid rgba(255,255,255,0.03);">
              <td style="padding:4px 0; font-weight:bold; color:#e74c3c;">Button B (Red)</td>
              <td style="padding:4px 0; text-align:right; color:#eee;">Play/Pause Playback (B)</td>
            </tr>
            <tr>
              <td style="padding:4px 0; font-weight:bold; color:#bb86fc;">Button R1 (Bumper)</td>
              <td style="padding:4px 0; text-align:right; color:#eee;">Toggle Safety Mode (R1)</td>
            </tr>
          </table>
        </div>
      </div>
    </div>

    <!-- Event Log Card -->
    <div class="card full">
      <h2><span>&#x1F4CB;</span> Event Log</h2>
      <div class="log-wrap" id="logContainer">
        <div style="text-align:center;color:#444;padding:20px;font-size:12px;">No events yet. Drive the robot or trigger safety mode to see events here.</div>
      </div>
      <div class="log-acts">
        <button class="log-btn" onclick="clearLogDisplay()">&#x1F5D1; Clear Display</button>
        <button class="log-btn" onclick="downloadLogCSV()">&#x2B07; Download CSV</button>
      </div>
    </div>
  </div>

  <script>
    // =============== GAMEPAD (preserved) ===============
    let gamepadIndex = null;
    let lastSendTime = 0;
    let lastSentL = null;
    let lastSentR = null;
    let lastSentP = null;

    // Playback and recording button edge-triggers
    let lastBtn0 = false; // Button A
    let lastBtn1 = false; // Button B
    let lastBtn5 = false; // Button R1 (Right Bumper)
    let safetyModeActive = false;
    let currentL = 0;
    let currentR = 0;
    let isRecording = false;
    let recordedSteps = [];
    let recordIntervalId = null;

    // ===== New Dashboard State =====
    let logEntries = [];
    let lastRTT = 0;
    const sessionStart = Date.now();
    // Uptime counter
    setInterval(() => {
      const sec = Math.floor((Date.now() - sessionStart) / 1000);
      const h = String(Math.floor(sec / 3600)).padStart(2, '0');
      const m = String(Math.floor((sec % 3600) / 60)).padStart(2, '0');
      const s = String(sec % 60).padStart(2, '0');
      const el = EL('uptimeVal');
      if (el) el.textContent = `${h}:${m}:${s}`;
    }, 1000);

    window.addEventListener("gamepadconnected", (e) => {
      gamepadIndex = e.gamepad.index;
      document.getElementById("status").innerHTML = "Gamepad Connected:<br><span style='color:#03dac6'>" + e.gamepad.id + "</span>";
      document.getElementById("status").style.color = "#4caf50";
      requestAnimationFrame(updateLoop);
    });

    window.addEventListener("gamepaddisconnected", (e) => {
      if (e.gamepad.index === gamepadIndex) {
        gamepadIndex = null;
        document.getElementById("status").innerHTML = "Gamepad Disconnected.<br>Waiting for connection...";
        document.getElementById("status").style.color = "#ff5252";
        sendDrive(0, 0, 0);
        drawGamepadVisualizer();
      }
    });

    function updateLoop() {
      if (gamepadIndex !== null) {
        const gamepads = navigator.getGamepads();
        const gp = gamepads[gamepadIndex];
        if (gp) {
          let y = gp.axes[1] || 0; // Left Stick Y
          let x = (gp.axes.length > 2) ? gp.axes[2] : gp.axes[0]; // Right Stick X, fallback to Left Stick X
          if (x === undefined) x = 0;

          if (Math.abs(y) < 0.1) y = 0;
          if (Math.abs(x) < 0.1) x = 0;
          if (gp.buttons[12] && gp.buttons[12].pressed) y = -1;
          if (gp.buttons[13] && gp.buttons[13].pressed) y = 1;
          if (gp.buttons[14] && gp.buttons[14].pressed) x = -1;
          if (gp.buttons[15] && gp.buttons[15].pressed) x = 1;
          let leftSpeed = Math.max(-1, Math.min(1, -y + x));
          let rightSpeed = Math.max(-1, Math.min(1, -y - x));
          let leftPWM = Math.round(leftSpeed * 255);
          let rightPWM = Math.round(rightSpeed * 255);
          
          if (typeof window.pumpState === 'undefined') window.pumpState = 0;
          
          // Button X (2) turns ON, Button Y (3) turns OFF
          if (gp.buttons[2] && gp.buttons[2].pressed) window.pumpState = 1;
          if (gp.buttons[3] && gp.buttons[3].pressed) window.pumpState = 0;
          
          let pump = window.pumpState;

          document.getElementById("joystick").innerText = `Left: ${leftPWM} | Right: ${rightPWM} | Pump: ${pump ? "ON" : "OFF"}`;
          
          // Store current values for local recording
          currentL = leftPWM;
          currentR = rightPWM;

          // Gamepad edge-triggered button commands
          let btn0Pressed = gp.buttons[0] && gp.buttons[0].pressed; // Button A -> Record
          let btn1Pressed = gp.buttons[1] && gp.buttons[1].pressed; // Button B -> Play/Pause
          let btn5Pressed = gp.buttons[5] && gp.buttons[5].pressed; // Button R1 (Right Bumper) -> Toggle Safety Mode
          
          if (btn0Pressed && !lastBtn0) {
            toggleRecording();
          }
          lastBtn0 = btn0Pressed;

          if (btn1Pressed && !lastBtn1) {
            togglePlayback();
          }
          lastBtn1 = btn1Pressed;

          if (btn5Pressed && !lastBtn5) {
            toggleSafetyMode();
          }
          lastBtn5 = btn5Pressed;

          let now = Date.now();
          let changed = (leftPWM !== lastSentL || rightPWM !== lastSentR || pump !== lastSentP);
          let keepAlive = (now - lastSendTime > 200); // Send keep-alive EVERY 200ms, even if 0
          
          if ((changed || keepAlive) && (now - lastSendTime > 50)) {
            if (!window.isFetching || pump !== lastSentP) {
              sendDrive(leftPWM, rightPWM, pump);
              lastSentL = leftPWM;
              lastSentR = rightPWM;
              lastSentP = pump;
              lastSendTime = now;
            }
          }
        }
        drawGamepadVisualizer();
        requestAnimationFrame(updateLoop);
      }
    }

    window.isFetching = false;
    function sendDrive(l, r, p) {
      window.isFetching = true;
      fetch(`/drive?l=${l}&r=${r}&p=${p}`)
        .catch(e => console.error("Comms error:", e))
        .finally(() => { window.isFetching = false; });
    }

    // =============== STATUS LIVE POLLING ===============
    const EL = id => document.getElementById(id);

    function pollStatus() {
      const t0 = Date.now();
      fetch(`/status?rtt=${lastRTT}`)
        .then(r => r.json())
        .then(d => {
          const currentRTT = Date.now() - t0;
          lastRTT = currentRTT;
          updateSensorUI('sensor-fl', 'txt-fl', d.fl);
          updateSensorUI('sensor-fr', 'txt-fr', d.fr);
          updateSensorUI('sensor-bl', 'txt-bl', d.bl);
          updateSensorUI('sensor-br', 'txt-br', d.br);

          const stateEl = EL('safetyStateText');
          const bannerEl = EL('safetyBanner');
          
          stateEl.textContent = d.safety;
          
          if (d.safety === 'NORMAL') {
            stateEl.className = 'status-normal';
            bannerEl.classList.remove('show');
          } else {
            stateEl.className = (d.safety === 'SAFETY LOCK') ? 'status-danger' : 'status-warning';
            bannerEl.textContent = `⚠️ SAFETY INTERRUPT: ${d.safety} (RC DISABLED)`;
            bannerEl.classList.add('show');
          }

          if (d.safetyMode !== undefined) {
            safetyModeActive = d.safetyMode;
            updateSafetyModeUI(safetyModeActive);
          }

          if (d.pump !== undefined) {
            updatePumpUI(d.pump);
          }

          // Playback sync (only if not recording locally)
          if (!isRecording) {
            const statusTextEl = EL("recStatus");
            const timeCounterEl = EL("timeCounter");
            const progressBarEl = EL("timelineProgress");
            const btnPlayEl = EL("btnPlay");
            const btnStopEl = EL("btnStop");
            const btnClearEl = EL("btnClear");

            if (d.playStatus === "PLAYING") {
              statusTextEl.textContent = "PLAYING";
              statusTextEl.style.color = "#bb86fc";
              btnPlayEl.textContent = "Pause";
              btnPlayEl.classList.add("playing");
              btnPlayEl.disabled = false;
              btnStopEl.disabled = false;
              btnClearEl.disabled = true;
            } else if (d.playStatus === "PAUSED") {
              statusTextEl.textContent = "PAUSED";
              statusTextEl.style.color = "#ffc107";
              btnPlayEl.textContent = "Resume";
              btnPlayEl.classList.remove("playing");
              btnPlayEl.disabled = false;
              btnStopEl.disabled = false;
              btnClearEl.disabled = false;
            } else if (d.playStatus === "STOPPED") {
              statusTextEl.textContent = "PRESET LOADED";
              statusTextEl.style.color = "#00e676";
              btnPlayEl.textContent = "Play";
              btnPlayEl.classList.remove("playing");
              btnPlayEl.disabled = false;
              btnStopEl.disabled = true;
              btnClearEl.disabled = false;
            } else { // "NO_PRESET" or others
              statusTextEl.textContent = "IDLE";
              statusTextEl.style.color = "#aaa";
              btnPlayEl.textContent = "Play";
              btnPlayEl.classList.remove("playing");
              btnPlayEl.disabled = true;
              btnStopEl.disabled = true;
              btnClearEl.disabled = true;
            }

            if (d.playLen > 0) {
              const progressPct = (d.playIndex / d.playLen * 100);
              progressBarEl.style.width = progressPct + "%";
              const curSec = (d.playIndex * 0.1).toFixed(1);
              const totalSec = (d.playLen * 0.1).toFixed(1);
              timeCounterEl.textContent = `Playback: ${d.playIndex} / ${d.playLen} (${curSec}s / ${totalSec}s)`;
            }
          }
          if (d.rssi !== undefined) {
            const rssiValEl = EL('rssiVal');
            if (rssiValEl) {
              if (d.clients > 0 && d.rssi !== 0) {
                rssiValEl.textContent = d.rssi + ' dBm';
                if (d.rssi > -67) rssiValEl.style.color = '#00e676';
                else if (d.rssi > -80) rssiValEl.style.color = '#ffc107';
                else rssiValEl.style.color = '#ff1744';
              } else {
                rssiValEl.textContent = '--';
                rssiValEl.style.color = '#888';
              }
            }
          }
          if (d.clients !== undefined) {
            const clientValEl = EL('clientVal');
            if (clientValEl) {
              clientValEl.textContent = d.clients + ' station' + (d.clients === 1 ? '' : 's');
            }
          }
          updateHealthBar(d);
          updateConnectionUI(currentRTT);
          const barL = EL('motorBarL'), barR = EL('motorBarR');
          if (barL) barL.style.width = (Math.abs(currentL) / 255 * 100).toFixed(1) + '%';
          if (barR) barR.style.width = (Math.abs(currentR) / 255 * 100).toFixed(1) + '%';
        })
        .catch(() => {});
    }

    function updateSensorUI(indicatorId, textId, isCliff) {
      const ind = EL(indicatorId);
      const txt = EL(textId);
      const suffix = indicatorId.split('-')[1];
      if (isCliff) {
        ind.className = `sensor-indicator ${suffix} cliff`;
        txt.textContent = 'CLIFF';
        txt.className = 'sensor-badge badge-cliff';
      } else {
        ind.className = `sensor-indicator ${suffix} safe`;
        txt.textContent = 'SAFE';
        txt.className = 'sensor-badge badge-safe';
      }
    }

    function toggleSafetyMode() {
      const nextActive = safetyModeActive ? 0 : 1;
      fetch(`/set_safety?active=${nextActive}`)
        .then(r => r.text())
        .then(txt => {
          if (txt === "OK") {
            safetyModeActive = (nextActive === 1);
            updateSafetyModeUI(safetyModeActive);
          }
        })
        .catch(err => console.error("Toggle safety error:", err));
    }

    function updateSafetyModeUI(isActive) {
      const modeText = EL("safetyModeText");
      const btn = EL("btnToggleSafety");
      if (modeText && btn) {
        if (isActive) {
          modeText.textContent = "ACTIVE (CLEANING)";
          modeText.style.color = "#00e676"; // Green
          btn.textContent = "Disable Safety/Cleaning";
          btn.style.background = "linear-gradient(135deg, #ff1744, #d50000)"; // Red
        } else {
          modeText.textContent = "BYPASSED (NORMAL)";
          modeText.style.color = "#ff5252"; // Red
          btn.textContent = "Enable Safety/Cleaning";
          btn.style.background = "linear-gradient(135deg, #00e676, #00c853)"; // Green
        }
      }
    }

    let pumpActive = false;
    function togglePump() {
      const nextActive = pumpActive ? 0 : 1;
      fetch(`/set_pump?active=${nextActive}`)
        .then(r => r.text())
        .then(txt => {
          if (txt === "OK") {
            pumpActive = (nextActive === 1);
            window.pumpState = nextActive;
            updatePumpUI(pumpActive);
          }
        })
        .catch(err => console.error("Toggle pump error:", err));
    }

    function updatePumpUI(isActive) {
      pumpActive = isActive;
      window.pumpState = isActive ? 1 : 0;
      const modeText = EL("pumpStateText");
      const btn = EL("btnTogglePump");
      if (modeText && btn) {
        if (isActive) {
          modeText.textContent = "ON";
          modeText.style.color = "#00e676"; // Green
          btn.textContent = "Turn Pump OFF";
          btn.style.background = "linear-gradient(135deg, #ff1744, #d50000)"; // Red
        } else {
          modeText.textContent = "OFF";
          modeText.style.color = "#ff5252"; // Red
          btn.textContent = "Turn Pump ON";
          btn.style.background = "linear-gradient(135deg, #3498db, #2980b9)"; // Blue
        }
      }
    }

    // =============== PRESET RECORDER JS ===============
    function toggleRecording() {
      if (isRecording) {
        stopRecording();
      } else {
        startRecording();
      }
    }

    function startRecording() {
      if (isRecording) return;
      
      // Stop playback first
      fetch('/playback?action=stop').catch(() => {});
      
      isRecording = true;
      recordedSteps = [];
      document.getElementById("btnRecord").textContent = "Stop Recording";
      document.getElementById("btnRecord").classList.add("recording");
      document.getElementById("recStatus").textContent = "RECORDING";
      document.getElementById("recStatus").style.color = "#ff1744";
      
      document.getElementById("btnPlay").disabled = true;
      document.getElementById("btnStop").disabled = true;
      document.getElementById("btnClear").disabled = true;
      
      recordIntervalId = setInterval(() => {
        if (recordedSteps.length >= 3000) {
          stopRecording();
          return;
        }
        recordedSteps.push({
          l: currentL,
          r: currentR,
          p: window.pumpState || 0
        });
        
        const sec = (recordedSteps.length * 0.1).toFixed(1);
        document.getElementById("timeCounter").textContent = `Steps: ${recordedSteps.length} / 3000 (${sec}s)`;
        document.getElementById("timelineProgress").style.width = (recordedSteps.length / 3000 * 100) + "%";
      }, 100);
    }

    function stopRecording() {
      if (!isRecording) return;
      isRecording = false;
      clearInterval(recordIntervalId);
      
      document.getElementById("btnRecord").textContent = "Record Preset";
      document.getElementById("btnRecord").classList.remove("recording");
      document.getElementById("recStatus").textContent = "UPLOADING...";
      document.getElementById("recStatus").style.color = "#ffc107";
      
      const dataStr = recordedSteps.map(s => `${s.l},${s.r},${s.p}`).join(';');
      
      fetch('/upload_preset', {
        method: 'POST',
        headers: { 'Content-Type': 'text/plain' },
        body: dataStr
      })
      .then(r => r.text())
      .then(res => {
        document.getElementById("recStatus").textContent = "PRESET LOADED";
        document.getElementById("recStatus").style.color = "#00e676";
        document.getElementById("btnPlay").disabled = false;
        document.getElementById("btnClear").disabled = false;
        document.getElementById("timelineProgress").style.width = "0%";
        document.getElementById("timeCounter").textContent = `Steps: ${recordedSteps.length} (Preset Ready)`;
      })
      .catch(err => {
        console.error("Upload error:", err);
        document.getElementById("recStatus").textContent = "UPLOAD ERROR";
        document.getElementById("recStatus").style.color = "#ff1744";
      });
    }

    function togglePlayback() {
      const statusText = document.getElementById("recStatus").textContent;
      if (statusText === "PLAYING") {
        fetch('/playback?action=pause')
          .then(r => r.text())
          .then(txt => {
            document.getElementById("btnPlay").textContent = "Resume";
            document.getElementById("btnPlay").classList.remove("playing");
          });
      } else {
        fetch('/playback?action=play')
          .then(r => r.text())
          .then(txt => {
            if (txt === "PLAYING") {
              document.getElementById("btnPlay").textContent = "Pause";
              document.getElementById("btnPlay").classList.add("playing");
              document.getElementById("btnStop").disabled = false;
            }
          });
      }
    }

    function stopPlayback() {
      fetch('/playback?action=stop')
        .then(r => r.text())
        .then(txt => {
          document.getElementById("btnPlay").textContent = "Play";
          document.getElementById("btnPlay").classList.remove("playing");
          document.getElementById("btnStop").disabled = true;
        });
    }

    function clearPreset() {
      if (confirm("Are you sure you want to clear the preset?")) {
        fetch('/playback?action=clear')
          .then(r => r.text())
          .then(txt => {
            recordedSteps = [];
            document.getElementById("recStatus").textContent = "IDLE";
            document.getElementById("recStatus").style.color = "#aaa";
            document.getElementById("btnPlay").disabled = true;
            document.getElementById("btnStop").disabled = true;
            document.getElementById("btnClear").disabled = true;
            document.getElementById("timelineProgress").style.width = "0%";
            document.getElementById("timeCounter").textContent = "Steps: 0 / 3000 (0.0s)";
          });
      }
    }

    // =============== GAMEPAD CANVAS VISUALIZER LOGIC ===============
    const gpCanvas = document.getElementById("controllerCanvas");
    const gpCtx = gpCanvas.getContext("2d");

    function drawGamepadVisualizer() {
      gpCtx.clearRect(0, 0, gpCanvas.width, gpCanvas.height);
      
      const connected = (gamepadIndex !== null);
      let lx = 0, ly = 0;
      let rx = 0, ry = 0;
      let btnA = false, btnB = false, btnX = false, btnY = false;
      let btnL1 = false, btnR1 = false;
      let dpadUp = false, dpadDown = false, dpadLeft = false, dpadRight = false;
      
      if (connected) {
        const gamepads = navigator.getGamepads();
        const gp = gamepads[gamepadIndex];
        if (gp) {
          lx = gp.axes[0] || 0;
          ly = gp.axes[1] || 0;
          rx = (gp.axes.length > 2) ? gp.axes[2] : gp.axes[0];
          ry = (gp.axes.length > 3) ? gp.axes[3] : 0;
          
          if (Math.abs(lx) < 0.1) lx = 0;
          if (Math.abs(ly) < 0.1) ly = 0;
          if (Math.abs(rx) < 0.1) rx = 0;
          if (Math.abs(ry) < 0.1) ry = 0;
          
          btnA = gp.buttons[0]?.pressed || false;
          btnB = gp.buttons[1]?.pressed || false;
          btnX = gp.buttons[2]?.pressed || false;
          btnY = gp.buttons[3]?.pressed || false;
          btnL1 = gp.buttons[4]?.pressed || false;
          btnR1 = gp.buttons[5]?.pressed || false;
          
          dpadUp = gp.buttons[12]?.pressed || false;
          dpadDown = gp.buttons[13]?.pressed || false;
          dpadLeft = gp.buttons[14]?.pressed || false;
          dpadRight = gp.buttons[15]?.pressed || false;
          
          if (dpadUp) ly = -1;
          if (dpadDown) ly = 1;
          if (dpadLeft) lx = -1;
          if (dpadRight) lx = 1;
        }
      }
      
      const cx = 160;
      const cy = 80;
      
      // L2 / R2 Triggers
      gpCtx.fillStyle = connected ? 'rgba(255,255,255,0.05)' : 'rgba(255,255,255,0.02)';
      gpCtx.fillRect(cx - 90, cy - 60, 25, 15);
      gpCtx.fillRect(cx + 65, cy - 60, 25, 15);
      gpCtx.strokeStyle = connected ? 'rgba(255,255,255,0.2)' : 'rgba(255,255,255,0.08)';
      gpCtx.lineWidth = 1;
      gpCtx.strokeRect(cx - 90, cy - 60, 25, 15);
      gpCtx.strokeRect(cx + 65, cy - 60, 25, 15);
      
      // L1 / R1 Bumpers
      gpCtx.fillStyle = btnL1 ? 'rgba(3, 218, 198, 0.4)' : (connected ? 'rgba(255,255,255,0.1)' : 'rgba(255,255,255,0.03)');
      gpCtx.strokeStyle = btnL1 ? '#03dac6' : (connected ? 'rgba(255,255,255,0.3)' : 'rgba(255,255,255,0.1)');
      gpCtx.beginPath();
      if (gpCtx.roundRect) {
        gpCtx.roundRect(cx - 95, cy - 42, 35, 10, 3);
      } else {
        gpCtx.rect(cx - 95, cy - 42, 35, 10);
      }
      gpCtx.fill();
      gpCtx.stroke();
      
      gpCtx.fillStyle = btnR1 ? 'rgba(187, 134, 252, 0.5)' : (connected ? 'rgba(255,255,255,0.1)' : 'rgba(255,255,255,0.03)');
      gpCtx.strokeStyle = btnR1 ? '#bb86fc' : (connected ? 'rgba(255,255,255,0.3)' : 'rgba(255,255,255,0.1)');
      gpCtx.beginPath();
      if (gpCtx.roundRect) {
        gpCtx.roundRect(cx + 60, cy - 42, 35, 10, 3);
      } else {
        gpCtx.rect(cx + 60, cy - 42, 35, 10);
      }
      gpCtx.fill();
      gpCtx.stroke();

      gpCtx.fillStyle = connected ? 'rgba(255,255,255,0.4)' : 'rgba(255,255,255,0.15)';
      gpCtx.font = '8px monospace';
      gpCtx.fillText("L1", cx - 85, cy - 34);
      gpCtx.fillText("R1", cx + 75, cy - 34);
      
      // Gamepad Body Contour
      gpCtx.beginPath();
      gpCtx.moveTo(cx - 70, cy - 35);
      gpCtx.lineTo(cx + 70, cy - 35);
      gpCtx.quadraticCurveTo(cx + 95, cy - 35, cx + 100, cy - 20);
      gpCtx.quadraticCurveTo(cx + 115, cy + 20, cx + 80, cy + 60);
      gpCtx.quadraticCurveTo(cx + 60, cy + 60, cx + 50, cy + 40);
      gpCtx.quadraticCurveTo(cx, cy + 55, cx - 50, cy + 40);
      gpCtx.quadraticCurveTo(cx - 60, cy + 60, cx - 80, cy + 60);
      gpCtx.quadraticCurveTo(cx - 115, cy + 20, cx - 100, cy - 20);
      gpCtx.quadraticCurveTo(cx - 95, cy - 35, cx - 70, cy - 35);
      gpCtx.closePath();
      
      gpCtx.strokeStyle = connected ? 'rgba(3, 218, 198, 0.4)' : 'rgba(255, 255, 255, 0.1)';
      gpCtx.lineWidth = 3;
      gpCtx.fillStyle = 'rgba(255, 255, 255, 0.02)';
      gpCtx.fill();
      gpCtx.stroke();
      
      // Analog stick bases
      gpCtx.strokeStyle = connected ? 'rgba(255,255,255,0.15)' : 'rgba(255,255,255,0.05)';
      gpCtx.lineWidth = 2;
      gpCtx.beginPath();
      gpCtx.arc(cx - 40, cy + 15, 18, 0, Math.PI * 2);
      gpCtx.stroke();
      gpCtx.beginPath();
      gpCtx.arc(cx + 40, cy + 15, 18, 0, Math.PI * 2);
      gpCtx.stroke();
      
      // Analog knobs
      gpCtx.fillStyle = connected ? 'rgba(3, 218, 198, 0.8)' : 'rgba(255,255,255,0.15)';
      gpCtx.beginPath();
      gpCtx.arc(cx - 40 + lx * 10, cy + 15 + ly * 10, 10, 0, Math.PI * 2);
      gpCtx.fill();
      
      gpCtx.fillStyle = connected ? 'rgba(187, 134, 252, 0.8)' : 'rgba(255,255,255,0.15)';
      gpCtx.beginPath();
      gpCtx.arc(cx + 40 + rx * 10, cy + 15 + ry * 10, 10, 0, Math.PI * 2);
      gpCtx.fill();
      
      // D-Pad Cross
      const dpadX = cx - 75;
      const dpadY = cy - 5;
      gpCtx.fillStyle = connected ? 'rgba(255,255,255,0.08)' : 'rgba(255,255,255,0.03)';
      gpCtx.fillRect(dpadX - 6, dpadY - 18, 12, 36);
      gpCtx.fillRect(dpadX - 18, dpadY - 6, 36, 12);
      
      gpCtx.fillStyle = '#03dac6';
      if (dpadUp) gpCtx.fillRect(dpadX - 6, dpadY - 18, 12, 12);
      if (dpadDown) gpCtx.fillRect(dpadX - 6, dpadY + 6, 12, 12);
      if (dpadLeft) gpCtx.fillRect(dpadX - 18, dpadY - 6, 12, 12);
      if (dpadRight) gpCtx.fillRect(dpadX + 6, dpadY - 6, 12, 12);
      
      gpCtx.strokeStyle = connected ? 'rgba(255,255,255,0.2)' : 'rgba(255,255,255,0.06)';
      gpCtx.lineWidth = 1;
      gpCtx.beginPath();
      gpCtx.moveTo(dpadX - 6, dpadY - 18);
      gpCtx.lineTo(dpadX + 6, dpadY - 18);
      gpCtx.lineTo(dpadX + 6, dpadY - 6);
      gpCtx.lineTo(dpadX + 18, dpadY - 6);
      gpCtx.lineTo(dpadX + 18, dpadY + 6);
      gpCtx.lineTo(dpadX + 6, dpadY + 6);
      gpCtx.lineTo(dpadX + 6, dpadY + 18);
      gpCtx.lineTo(dpadX - 6, dpadY + 18);
      gpCtx.lineTo(dpadX - 6, dpadY + 6);
      gpCtx.lineTo(dpadX - 18, dpadY + 6);
      gpCtx.lineTo(dpadX - 18, dpadY - 6);
      gpCtx.lineTo(dpadX - 6, dpadY - 6);
      gpCtx.closePath();
      gpCtx.stroke();
      
      // Face Buttons (A, B, X, Y)
      const faceX = cx + 75;
      const faceY = cy - 5;
      
      // Y (Yellow)
      gpCtx.fillStyle = btnY ? '#ffb300' : (connected ? 'rgba(255,179,0,0.2)' : 'rgba(255,255,255,0.05)');
      gpCtx.strokeStyle = '#ffb300';
      gpCtx.lineWidth = 1.5;
      gpCtx.beginPath();
      gpCtx.arc(faceX, faceY - 12, 7, 0, Math.PI * 2);
      gpCtx.fill();
      gpCtx.stroke();
      gpCtx.fillStyle = btnY ? '#000' : '#ffb300';
      gpCtx.font = 'bold 8px Arial';
      gpCtx.textAlign = 'center';
      gpCtx.textBaseline = 'middle';
      gpCtx.fillText("Y", faceX, faceY - 12);
      
      // A (Green)
      gpCtx.fillStyle = btnA ? '#2ecc71' : (connected ? 'rgba(46,204,113,0.2)' : 'rgba(255,255,255,0.05)');
      gpCtx.strokeStyle = '#2ecc71';
      gpCtx.beginPath();
      gpCtx.arc(faceX, faceY + 12, 7, 0, Math.PI * 2);
      gpCtx.fill();
      gpCtx.stroke();
      gpCtx.fillStyle = btnA ? '#000' : '#2ecc71';
      gpCtx.fillText("A", faceX, faceY + 12);
      
      // X (Blue)
      gpCtx.fillStyle = btnX ? '#3498db' : (connected ? 'rgba(52,152,219,0.2)' : 'rgba(255,255,255,0.05)');
      gpCtx.strokeStyle = '#3498db';
      gpCtx.beginPath();
      gpCtx.arc(faceX - 12, faceY, 7, 0, Math.PI * 2);
      gpCtx.fill();
      gpCtx.stroke();
      gpCtx.fillStyle = btnX ? '#000' : '#3498db';
      gpCtx.fillText("X", faceX - 12, faceY);
      
      // B (Red)
      gpCtx.fillStyle = btnB ? '#e74c3c' : (connected ? 'rgba(231,76,60,0.2)' : 'rgba(255,255,255,0.05)');
      gpCtx.strokeStyle = '#e74c3c';
      gpCtx.beginPath();
      gpCtx.arc(faceX + 12, faceY, 7, 0, Math.PI * 2);
      gpCtx.fill();
      gpCtx.stroke();
      gpCtx.fillStyle = btnB ? '#000' : '#e74c3c';
      gpCtx.fillText("B", faceX + 12, faceY);
      
      // Status LED
      gpCtx.fillStyle = connected ? '#00e676' : '#ff1744';
      gpCtx.beginPath();
      gpCtx.arc(cx, cy - 15, 3, 0, Math.PI * 2);
      gpCtx.fill();
    }

    // ===== E-STOP =====
    function triggerEstop() {
      const btn = EL('btnEstop');
      if (btn) { btn.classList.add('fired'); setTimeout(() => btn.classList.remove('fired'), 700); }
      fetch('/estop').catch(() => {});
    }

    // ===== System Health Bar =====
    function updateHealthBar(d) {
      const pill = EL('healthPill');
      if (!pill) return;
      const anyCliff = d.fl || d.fr || d.bl || d.br;
      if (d.safety !== 'NORMAL') {
        pill.className = 'health-pill health-fault';
        pill.textContent = '\uD83D\uDD34 FAULT: ' + d.safety;
      } else if (!d.safetyMode && anyCliff) {
        pill.className = 'health-pill health-warn';
        pill.textContent = '\uD83D\uDFE1 WARNING: Cliff sensor active (safety bypassed)';
      } else {
        pill.className = 'health-pill health-ok';
        pill.textContent = '\uD83D\uDFE2 ALL SYSTEMS OK';
      }
    }

    // ===== Connection Latency (RTT) =====
    function updateConnectionUI(rtt) {
      const el = EL('rttVal');
      if (el) el.textContent = rtt + ' ms';
      let bars = 0, cls = '';
      if      (rtt < 50)  { bars = 5; cls = 's-on'; }
      else if (rtt < 100) { bars = 4; cls = 's-on'; }
      else if (rtt < 200) { bars = 3; cls = 's-warn'; }
      else if (rtt < 500) { bars = 2; cls = 's-warn'; }
      else                { bars = 1; cls = 's-poor'; }
      for (let i = 1; i <= 5; i++) {
        const b = EL('sig' + i);
        if (b) b.className = 'sig-bar' + (i <= bars ? ' ' + cls : '');
      }
    }

    // ===== Event Log =====
    function pollLog() {
      fetch('/log')
        .then(r => r.json())
        .then(data => {
          if (data.length === logEntries.length) return;
          logEntries = data;
          renderLog();
        })
        .catch(() => {});
    }

    function renderLog() {
      const c = EL('logContainer');
      if (!c) return;
      if (!logEntries.length) {
        c.innerHTML = '<div style="text-align:center;color:#444;padding:20px;font-size:12px;">No events yet. Drive the robot or trigger safety mode.</div>';
        return;
      }
      c.innerHTML = [...logEntries].reverse().map(e => {
        const ms = e.t;
        const h = String(Math.floor(ms / 3600000)).padStart(2, '0');
        const m = String(Math.floor((ms % 3600000) / 60000)).padStart(2, '0');
        const s = String(Math.floor((ms % 60000) / 1000)).padStart(2, '0');
        const up = e.msg.toUpperCase();
        let cls = 'ev-ok';
        if (up.includes('CLIFF') || up.includes('EVAD')) cls = 'ev-cliff';
        else if (up.includes('STOP') || up.includes('SAFETY') || up.includes('LOCK') || up.includes('E-STOP')) cls = 'ev-safety';
        else if (up.includes('PUMP')) cls = 'ev-pump';
        else if (up.includes('[WIFI]')) cls = 'ev-wifi';
        else if (up.includes('[REC]')) cls = 'ev-rec';
        else if (up.includes('[PLAY]')) cls = 'ev-play';
        return `<div class="log-entry ${cls}"><span class="log-ts">${h}:${m}:${s}</span><span>${e.msg}</span></div>`;
      }).join('');
    }

    function clearLogDisplay() { logEntries = []; renderLog(); }

    function downloadLogCSV() {
      if (!logEntries.length) { alert('No log entries to download.'); return; }
      const csv = 'Timestamp_ms,Message\n' + logEntries.map(e => `${e.t},"${e.msg.replace(/"/g, '""')}"`).join('\n');
      const a = document.createElement('a');
      a.href = URL.createObjectURL(new Blob([csv], {type: 'text/csv'}));
      a.download = 'g7_event_log.csv';
      a.click();
    }

    // Initial draw in disconnected state
    drawGamepadVisualizer();

    setInterval(pollStatus, 200);
    setInterval(pollLog, 2000);
  </script>
</body>
</html>
)rawliteral";

// ==================== Low-level Motor Output ====================
// Forward declarations
void setMotorsDirect(int left, int right);
void setMotors(int left, int right);

// ==================== Event Log Helper ====================
void addLog(const char* msg) {
  logBuffer[logHead].ts = millis();
  strncpy(logBuffer[logHead].msg, msg, 79);
  logBuffer[logHead].msg[79] = '\0';
  logHead = (logHead + 1) % LOG_BUFFER_SIZE;
  if (logCount < LOG_BUFFER_SIZE) logCount++;
  Serial.print("[LOG] ");
  Serial.println(msg);
}

// ==================== Safety Evasion Logic ====================
void updateSafety() {
  if (!safetyModeActive) {
    if (safetyState != STATE_NORMAL) {
      setMotorsDirect(0, 0);
      safetyState = STATE_NORMAL;
      Serial.println("Safety Mode disabled. Resetting safety state to NORMAL.");
    }
    return;
  }

  bool fl_cliff = (digitalRead(PIN_FS80NK_FL) == CLIFF_STATE);
  bool fr_cliff = (digitalRead(PIN_FS80NK_FR) == CLIFF_STATE);
  bool bl_cliff = (digitalRead(PIN_FS80NK_BL) == CLIFF_STATE);
  bool br_cliff = (digitalRead(PIN_FS80NK_BR) == CLIFF_STATE);

  unsigned long now = millis();

  switch (safetyState) {
    case STATE_NORMAL:
      // Check left side cliff first (both FL & BL detect cliff)
      if (fl_cliff && bl_cliff) {
        Serial.println("Safety Interrupt: LEFT CLIFF DETECTED! Evading right...");
        addLog("LEFT CLIFF detected - evading right");
        safetyState = STATE_LEFT_EVADE_TURN;
        safetyTimer = now;
        setMotorsDirect(255, -255); // Pivot Right (turn away from left edge)
      }
      // Check right side cliff (both FR & BR detect cliff)
      else if (fr_cliff && br_cliff) {
        Serial.println("Safety Interrupt: RIGHT CLIFF DETECTED! Evading left...");
        addLog("RIGHT CLIFF detected - evading left");
        safetyState = STATE_RIGHT_EVADE_TURN;
        safetyTimer = now;
        setMotorsDirect(-255, 255); // Pivot Left (turn away from right edge)
      }
      // Check front cliff (FL or FR detect cliff)
      else if (fl_cliff || fr_cliff) {
        Serial.println("Safety Interrupt: FRONT CLIFF DETECTED! Reversing...");
        addLog("FRONT CLIFF detected - reversing");
        safetyState = STATE_FRONT_EVADE;
        safetyTimer = now;
        setMotorsDirect(-255, -255); // Reverse
      }
      // Check back cliff (BL or BR detect cliff)
      else if (bl_cliff || br_cliff) {
        Serial.println("Safety Interrupt: BACK CLIFF DETECTED! Moving forward...");
        addLog("BACK CLIFF detected - moving forward");
        safetyState = STATE_BACK_EVADE;
        safetyTimer = now;
        setMotorsDirect(255, 255); // Forward
      }
      break;

    case STATE_LEFT_EVADE_TURN:
      // Turn right for 1000ms
      if (now - safetyTimer >= 1000) {
        Serial.println("Left Evasion: Turn complete. Moving straight to reposition...");
        safetyState = STATE_LEFT_EVADE_STRAIGHT;
        safetyTimer = now;
        setMotorsDirect(255, 255); // Move straight
      }
      break;

    case STATE_LEFT_EVADE_STRAIGHT:
      // Move straight for 800ms
      if (now - safetyTimer >= 800) {
        Serial.println("Left Evasion: Complete. Stopping.");
        setMotorsDirect(0, 0);
        safetyState = STATE_STOP_WAIT;
        safetyTimer = now;
      }
      break;

    case STATE_RIGHT_EVADE_TURN:
      // Turn left for 1000ms
      if (now - safetyTimer >= 1000) {
        Serial.println("Right Evasion: Turn complete. Moving straight to reposition...");
        safetyState = STATE_RIGHT_EVADE_STRAIGHT;
        safetyTimer = now;
        setMotorsDirect(255, 255); // Move straight
      }
      break;

    case STATE_RIGHT_EVADE_STRAIGHT:
      // Move straight for 800ms
      if (now - safetyTimer >= 800) {
        Serial.println("Right Evasion: Complete. Stopping.");
        setMotorsDirect(0, 0);
        safetyState = STATE_STOP_WAIT;
        safetyTimer = now;
      }
      break;

    case STATE_FRONT_EVADE:
      // Reverse for 1200ms
      if (now - safetyTimer >= 1200) {
        Serial.println("Front Evasion: Complete. Stopping.");
        setMotorsDirect(0, 0);
        safetyState = STATE_STOP_WAIT;
        safetyTimer = now;
      }
      break;

    case STATE_BACK_EVADE:
      // Move forward for 1200ms
      if (now - safetyTimer >= 1200) {
        Serial.println("Back Evasion: Complete. Stopping.");
        setMotorsDirect(0, 0);
        safetyState = STATE_STOP_WAIT;
        safetyTimer = now;
      }
      break;

    case STATE_STOP_WAIT:
      // Wait 500ms, then release lock ONLY if no cliffs are detected
      if (now - safetyTimer >= 500) {
        if (!fl_cliff && !fr_cliff && !bl_cliff && !br_cliff) {
          Serial.println("Safety clear. Control returned to RC.");
          addLog("Safety clear - control returned to RC");
          safetyState = STATE_NORMAL;
        } else {
          static unsigned long lastWarn = 0;
          if (now - lastWarn >= 2000) {
            Serial.println("Warning: Cliff still detected! Safety lock remains active.");
            lastWarn = now;
          }
        }
      }
      break;
  }
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(100); // Give serial monitor a moment
  Serial.println("\n\n--- Booting ESP32 ---");

  // Motor pin setup
  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  // Relay pin setup
  setPumpState(false); // Ensure pump starts OFF

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PWM_L, 5000, 8);
  ledcAttach(PWM_R, 5000, 8);
#else
  ledcSetup(2, 5000, 8);
  ledcAttachPin(PWM_L, 2);
  ledcSetup(3, 5000, 8);
  ledcAttachPin(PWM_R, 3);
#endif

  setMotors(0, 0);

  // I2C initialization (untouched on GPIO 21, 22 for future use)
  Wire.begin(21, 22);
  Serial.println("I2C initialized on GPIO 21, 22 (untouched for future use)");

  // FS80NK Cliff Sensors initialization
  pinMode(PIN_FS80NK_FL, INPUT_PULLUP);
  pinMode(PIN_FS80NK_FR, INPUT_PULLUP);
  pinMode(PIN_FS80NK_BL, INPUT_PULLUP);
  pinMode(PIN_FS80NK_BR, INPUT_PULLUP);
  Serial.println("FS80NK Cliff Sensors configured with INPUT_PULLUP");

  // WiFi Access Point
  Serial.println("Starting Access Point...");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Connect to WiFi: ");
  Serial.println(ssid);
  Serial.print("Dashboard: http://");
  Serial.println(IP);

  // Web routes
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", htmlPage);
  });

  server.on("/drive", HTTP_GET, []() {
    if (server.hasArg("l") && server.hasArg("r")) {
      int leftReq = server.arg("l").toInt();
      int rightReq = server.arg("r").toInt();
      int pumpReq = server.hasArg("p") ? server.arg("p").toInt() : 0;
      
      setMotors(leftReq, rightReq);

      if (pumpReq == 1) {
        setPumpState(true); // Relay active (Pump ON)
      } else {
        setPumpState(false); // Relay inactive (Pump OFF)
      }

      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Bad Request");
    }
  });

  server.on("/set_safety", HTTP_GET, []() {
    if (server.hasArg("active")) {
      int active = server.arg("active").toInt();
      safetyModeActive = (active == 1);
      Serial.print("Safety Mode changed: ");
      Serial.println(safetyModeActive ? "ACTIVE (Safety/Cleaning)" : "BYPASSED (Normal)");
      addLog(safetyModeActive ? "Safety mode ENABLED (Cleaning)" : "Safety mode DISABLED (Normal RC)");
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Bad Request");
    }
  });

  server.on("/set_pump", HTTP_GET, []() {
    if (server.hasArg("active")) {
      int active = server.arg("active").toInt();
      if (active == 1) {
        setPumpState(true); // Relay active (Pump ON)
        Serial.println("Pump turned ON via set_pump");
        addLog("Water pump turned ON");
      } else {
        setPumpState(false); // Relay inactive (Pump OFF)
        Serial.println("Pump turned OFF via set_pump");
        addLog("Water pump turned OFF");
      }
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Bad Request");
    }
  });

  // --- HARDWARE DEBUG ENDPOINTS ---
  server.on("/testleft", HTTP_GET, []() {
    if (safetyState != STATE_NORMAL) {
      server.send(200, "text/plain", "BLOCKED: Safety Override Active!");
      return;
    }
    digitalWrite(DIR_L, HIGH);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PWM_L, MAX_PWM);
#else
    ledcWrite(2, MAX_PWM);
#endif
    server.send(200, "text/plain", "TEST: Left Motor ON");
  });

  server.on("/testright", HTTP_GET, []() {
    if (safetyState != STATE_NORMAL) {
      server.send(200, "text/plain", "BLOCKED: Safety Override Active!");
      return;
    }
    digitalWrite(DIR_R, HIGH);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PWM_R, MAX_PWM);
#else
    ledcWrite(3, MAX_PWM);
#endif
    server.send(200, "text/plain", "TEST: Right Motor ON");
  });

  server.on("/teststop", HTTP_GET, []() {
    setMotors(0, 0);
    server.send(200, "text/plain", "TEST: Motors OFF");
  });
  // --------------------------------

  server.on("/upload_preset", HTTP_POST, []() {
    String payload = server.arg("plain");
    sequenceLength = 0;
    playbackIndex = 0;
    isPlayingPlayback = false;
    isPausedPlayback = false;

    int startIndex = 0;
    while (startIndex < payload.length() && sequenceLength < MAX_STEPS) {
      int nextSemi = payload.indexOf(';', startIndex);
      if (nextSemi == -1) nextSemi = payload.length();
      
      String stepStr = payload.substring(startIndex, nextSemi);
      startIndex = nextSemi + 1;
      
      if (stepStr.length() == 0) continue;
      
      int firstComma = stepStr.indexOf(',');
      int secondComma = stepStr.indexOf(',', firstComma + 1);
      if (firstComma != -1 && secondComma != -1) {
        int lVal = stepStr.substring(0, firstComma).toInt();
        int rVal = stepStr.substring(firstComma + 1, secondComma).toInt();
        int pVal = stepStr.substring(secondComma + 1).toInt();
        
        recordedSequence[sequenceLength].left = (int16_t)lVal;
        recordedSequence[sequenceLength].right = (int16_t)rVal;
        recordedSequence[sequenceLength].pump = (uint8_t)pVal;
        sequenceLength++;
      }
    }
    Serial.print("Uploaded preset steps: ");
    Serial.println(sequenceLength);
    char recLog[45];
    snprintf(recLog, sizeof(recLog), "[REC] Saved preset: %d steps", sequenceLength);
    addLog(recLog);
    server.send(200, "text/plain", "OK");
  });

  server.on("/playback", HTTP_GET, []() {
    if (server.hasArg("action")) {
      String action = server.arg("action");
      if (action == "play") {
        if (sequenceLength > 0) {
          isPlayingPlayback = true;
          isPausedPlayback = false;
          lastPlaybackStepTime = millis();
          Serial.println("Playback started/resumed");
          addLog("[PLAY] Started/resumed playback");
          server.send(200, "text/plain", "PLAYING");
        } else {
          server.send(200, "text/plain", "NO_PRESET");
        }
      } else if (action == "pause") {
        isPausedPlayback = true;
        isPlayingPlayback = false;
        setMotorsDirect(0, 0); // Stop motors but keep index
        Serial.println("Playback paused");
        char pauseLog[45];
        snprintf(pauseLog, sizeof(pauseLog), "[PLAY] Playback paused at step %d", playbackIndex);
        addLog(pauseLog);
        server.send(200, "text/plain", "PAUSED");
      } else if (action == "stop") {
        isPlayingPlayback = false;
        isPausedPlayback = false;
        playbackIndex = 0;
        setMotorsDirect(0, 0);
        Serial.println("Playback stopped");
        addLog("[PLAY] Playback stopped");
        server.send(200, "text/plain", "STOPPED");
      } else if (action == "clear") {
        isPlayingPlayback = false;
        isPausedPlayback = false;
        playbackIndex = 0;
        sequenceLength = 0;
        setMotorsDirect(0, 0);
        Serial.println("Playback cleared");
        addLog("[REC] Preset cleared");
        server.send(200, "text/plain", "CLEARED");
      }
    } else {
      server.send(400, "text/plain", "Missing action");
    }
  });

  server.on("/status", HTTP_GET, []() {
    bool fl = (digitalRead(PIN_FS80NK_FL) == CLIFF_STATE);
    bool fr = (digitalRead(PIN_FS80NK_FR) == CLIFF_STATE);
    bool bl = (digitalRead(PIN_FS80NK_BL) == CLIFF_STATE);
    bool br = (digitalRead(PIN_FS80NK_BR) == CLIFF_STATE);

    const char* safetyText = "NORMAL";
    if (safetyState == STATE_FRONT_EVADE) safetyText = "EVADING FRONT";
    else if (safetyState == STATE_BACK_EVADE) safetyText = "EVADING BACK";
    else if (safetyState == STATE_LEFT_EVADE_TURN || safetyState == STATE_LEFT_EVADE_STRAIGHT) safetyText = "EVADING LEFT";
    else if (safetyState == STATE_RIGHT_EVADE_TURN || safetyState == STATE_RIGHT_EVADE_STRAIGHT) safetyText = "EVADING RIGHT";
    else if (safetyState == STATE_STOP_WAIT) safetyText = "SAFETY LOCK";

    const char* playStatusText = "NO_PRESET";
    if (sequenceLength > 0) {
      if (isPlayingPlayback) playStatusText = "PLAYING";
      else if (isPausedPlayback) playStatusText = "PAUSED";
      else playStatusText = "STOPPED";
    }

    int rttParam = -1;
    if (server.hasArg("rtt")) {
      rttParam = server.arg("rtt").toInt();
    }

    if (rttParam > 350) {
      if (lastLoggedRTT == 0) {
        char latencyLog[45];
        snprintf(latencyLog, sizeof(latencyLog), "[WiFi] High latency: %d ms", rttParam);
        addLog(latencyLog);
        lastLoggedRTT = 1;
      }
    } else if (rttParam > 0 && rttParam <= 300) {
      if (lastLoggedRTT == 1) {
        addLog("[WiFi] Latency normal");
        lastLoggedRTT = 0;
      }
    }

    int clientRSSI = getClientRSSI();
    int stationNum = WiFi.softAPgetStationNum();

    if (stationNum > 0 && clientRSSI < -80 && clientRSSI > -120) {
      if (lastLoggedRSSI != 2) {
        char rssiLog[45];
        snprintf(rssiLog, sizeof(rssiLog), "[WiFi] Weak signal: %d dBm", clientRSSI);
        addLog(rssiLog);
        lastLoggedRSSI = 2;
      }
    } else if (stationNum > 0 && clientRSSI >= -75) {
      if (lastLoggedRSSI == 2) {
        addLog("[WiFi] Signal restored");
        lastLoggedRSSI = 1;
      }
    } else if (stationNum == 0) {
      lastLoggedRSSI = 0;
    }

    char json[512];
    snprintf(json, sizeof(json),
      "{\"fl\":%s,\"fr\":%s,\"bl\":%s,\"br\":%s,\"safety\":\"%s\","
      "\"playStatus\":\"%s\",\"playIndex\":%d,\"playLen\":%d,\"safetyMode\":%s,\"pump\":%s,\"uptime\":%lu,\"rssi\":%d,\"clients\":%d}",
      fl ? "true" : "false",
      fr ? "true" : "false",
      bl ? "true" : "false",
      br ? "true" : "false",
      safetyText,
      playStatusText,
      playbackIndex,
      sequenceLength,
      safetyModeActive ? "true" : "false",
      isPumpOn ? "true" : "false",
      millis(),
      clientRSSI,
      stationNum
    );
    server.send(200, "application/json", json);
  });

  // ===== E-STOP endpoint =====
  server.on("/estop", HTTP_GET, []() {
    setMotorsDirect(0, 0);
    setPumpState(false);
    isPlayingPlayback = false;
    isPausedPlayback = false;
    addLog("E-STOP activated - all motors and pump halted");
    server.send(200, "text/plain", "OK");
  });

  // ===== Event Log endpoint =====
  server.on("/log", HTTP_GET, []() {
    String json = "[";
    int start = (logCount < LOG_BUFFER_SIZE) ? 0 : logHead;
    for (int i = 0; i < logCount; i++) {
      int idx = (start + i) % LOG_BUFFER_SIZE;
      if (i > 0) json += ",";
      json += "{\"t\":";
      json += (unsigned long)logBuffer[idx].ts;
      json += ",\"msg\":\"";
      const char* m = logBuffer[idx].msg;
      while (*m) {
        if (*m == '"') json += "\\\"";
        else if (*m == '\\') json += "\\\\";
        else json += *m;
        m++;
      }
      json += "\"}";
    }
    json += "]";
    server.send(200, "application/json", json);
  });

  server.begin();
  Serial.println("HTTP server started");
}

// ==================== Loop ====================
void loop() {
  server.handleClient();
  updateSafety();

  // Non-blocking local playback sequence executor
  if (isPlayingPlayback && safetyState == STATE_NORMAL) {
    if (millis() - lastPlaybackStepTime >= PLAYBACK_STEP_INTERVAL) {
      lastPlaybackStepTime = millis();
      if (playbackIndex < sequenceLength) {
        int16_t lSpeed = recordedSequence[playbackIndex].left;
        int16_t rSpeed = recordedSequence[playbackIndex].right;
        uint8_t pState = recordedSequence[playbackIndex].pump;
        
        // Write direct to motors and pump
        setMotorsDirect(lSpeed, rSpeed);
        if (pState == 1) {
          setPumpState(true); // ON
        } else {
          setPumpState(false); // OFF
        }
        
        playbackIndex++;
      } else {
        // End of sequence reached
        isPlayingPlayback = false;
        playbackIndex = 0;
        setMotorsDirect(0, 0);
        Serial.println("Playback complete");
        addLog("[PLAY] Playback complete");
      }
    }
  } else if (isPlayingPlayback && safetyState != STATE_NORMAL) {
    // Auto-pause playback if safety override is active
    isPlayingPlayback = false;
    isPausedPlayback = true;
    setMotorsDirect(0, 0);
    Serial.println("Playback auto-paused due to Safety override!");
  }
}

// ==================== Motor Control Direct ====================
void setMotorsDirect(int left, int right) {
  int scaledLeft  = (left  != 0) ? map(abs(left),  0, 255, MIN_PWM, MAX_PWM) : 0;
  int scaledRight = (right != 0) ? map(abs(right), 0, 255, MIN_PWM, MAX_PWM) : 0;

  scaledLeft  = constrain(scaledLeft,  0, MAX_PWM);
  scaledRight = constrain(scaledRight, 0, MAX_PWM);

  // Print debug values to Serial Monitor
  Serial.print("Motor Control Direct -> Left: "); Serial.print(left);
  Serial.print(" | Scaled PWM: "); Serial.print(scaledLeft);
  Serial.print(" | Right: "); Serial.print(right);
  Serial.print(" | Scaled PWM: "); Serial.println(scaledRight);

  digitalWrite(DIR_L, left >= 0 ? LOW : HIGH);  // Inverted: left motor wired opposite
  digitalWrite(DIR_R, right >= 0 ? HIGH : LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWM_L, scaledLeft);
  ledcWrite(PWM_R, scaledRight);
#else
  ledcWrite(2, scaledLeft);
  ledcWrite(3, scaledRight);
#endif
}

// ==================== Motor Control (RC Controlled) ====================
void setMotors(int left, int right) {
  if (safetyState == STATE_NORMAL && !isPlayingPlayback) {
    setMotorsDirect(left, right);
  } else if (isPlayingPlayback) {
    Serial.println("RC Ignored: Autonomous Playback running!");
  } else {
    Serial.println("RC Ignored: Safety override active!");
  }
}
