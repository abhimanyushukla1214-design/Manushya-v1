/*
  =====================================================================
  ESP32 Brushed-Motor Quadcopter Flight Controller
  WiFi (UDP) control + MPU6050 stabilization + startup gyro calibration
  =====================================================================

  READ THIS FIRST:
  - This gives you a real, working starting point, NOT a guaranteed
    "ready to fly" file. PID gains, motor mixing signs, and motor
    direction ALWAYS need tuning per airframe. Do not skip the
    bench-test steps below.
  - ALWAYS test with propellers REMOVED first. Only fit propellers once
    you've confirmed each motor spins the correct direction and the
    stabilization response looks correct (test steps at the bottom).
  - If WiFi disconnects or no control packet arrives for FAILSAFE_MS,
    all motors are cut automatically. Do not rely on this as your only
    safety measure - always be ready to cut battery power by hand.

  ---------------------------------------------------------------------
  HARDWARE / PIN MAP (from your wiring diagram)
  ---------------------------------------------------------------------
    MPU6050   SDA  -> GPIO21
    MPU6050   SCL  -> GPIO22
    MPU6050   INT  -> GPIO14 (not used by this sketch, wired for later)
    M1 (front-right, CCW) -> GPIO4
    M2 (back-right,  CW ) -> GPIO33
    M3 (back-left,   CCW) -> GPIO32
    M4 (front-left,  CW ) -> GPIO25

  Frame layout (looking down from above, nose pointing up the page):

              M4 (CW)         M1 (CCW)
                 \\             /
                  \\           /
                       ESP32
                  /           \\
                 /             \\
              M3 (CCW)        M2 (CW)

  ---------------------------------------------------------------------
  CONTROL PROTOCOL (what you send from your phone)
  ---------------------------------------------------------------------
  The ESP32 creates its own WiFi network (see WIFI_SSID / WIFI_PASS
  below). Connect your phone to it directly - no home router needed.

  Send plain-text UDP packets to 192.168.4.1 : 4210

    "ARM\n"                 -> arms the motors (must be armed to fly)
    "DISARM\n"               -> disarms, motors stop immediately
    "T,R,P,Y\n"              -> control values, e.g. "50,0,0,0\n"
        T = throttle   0 to 100
        R = roll       -100 to 100  (negative = roll left)
        P = pitch      -100 to 100  (negative = pitch forward)
        Y = yaw rate   -100 to 100  (negative = yaw left)

  This works directly with apps like RoboRemo (Android/iOS) where you
  build a joystick UI and set each axis to send one of these strings
  over WiFi UDP - see the setup notes at the very end of this file.

  ---------------------------------------------------------------------
  NO APP NEEDED: BUILT-IN BROWSER CONTROL PAGE
  ---------------------------------------------------------------------
  This sketch also runs a small web server on the ESP32 itself. Once
  connected to the drone's WiFi network, just open a normal web browser
  (Chrome, Safari, whatever you already have) and go to:

      http://192.168.4.1

  You'll get two on-screen touch joysticks (left = throttle/yaw,
  right = pitch/roll) and an ARM/DISARM button - nothing to install.
  The UDP protocol above still works too if you later want a
  dedicated app like RoboRemo instead.
  =====================================================================
*/

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

// ---------------------------------------------------------------
// EDIT THESE: pins (already matched to your diagram)
// ---------------------------------------------------------------
#define SDA_PIN   21
#define SCL_PIN   22
#define INT_PIN   14

#define M1_PIN    32   // front-right, CCW
#define M2_PIN    33  // back-right,  CW
#define M3_PIN    26  // back-left,   CCW
#define M4_PIN    25  // front-left,  CW

// ---------------------------------------------------------------
// EDIT THESE: WiFi access point the drone creates
// ---------------------------------------------------------------
const char* WIFI_SSID = "MyDrone";
const char* WIFI_PASS = "drone1234";   // must be 8+ characters
const unsigned int UDP_PORT = 4210;

// ---------------------------------------------------------------
// Safety / tuning constants - EDIT AFTER BENCH TESTING
// ---------------------------------------------------------------
const unsigned long FAILSAFE_MS   = 500;   // cut motors if no packet in this long
const int   MAX_THROTTLE_PWM      = 255;   // 8-bit PWM ceiling
const int   MIN_SPIN_PWM          = 60;    // PWM below which motors barely spin (tune this!)
const float PID_KP_ROLL  = 1.8;
const float PID_KI_ROLL  = 0.02;
const float PID_KD_ROLL  = 0.8;
const float PID_KP_PITCH = 1.8;
const float PID_KI_PITCH = 0.02;
const float PID_KD_PITCH = 0.8;
const float PID_KP_YAW   = 2.0;

// ---------------------------------------------------------------
// MPU6050 registers
// ---------------------------------------------------------------
#define MPU6050_ADDR      0x68
#define REG_PWR_MGMT_1    0x6B
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B

// ---------------------------------------------------------------
// Globals
// ---------------------------------------------------------------
WiFiUDP udp;
char packetBuffer[64];
WebServer server(80);   // browser control page + endpoints, port 80 (default http)

volatile float throttleIn = 0;   // 0-100
volatile float rollIn     = 0;   // -100 to 100
volatile float pitchIn    = 0;   // -100 to 100
volatile float yawIn      = 0;   // -100 to 100
volatile bool  armed      = false;
unsigned long lastPacketTime = 0;

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

float pitchAngle = 0, rollAngle = 0;   // complementary-filter estimate (degrees)
unsigned long lastLoopTime = 0;

// PID state
float rollErrIntegral = 0, rollErrPrev = 0;
float pitchErrIntegral = 0, pitchErrPrev = 0;

// ESP32 LEDC PWM settings (core v3.x API: attach directly to the pin,
// no manual channel numbers needed)
const int PWM_FREQ = 20000;   // 20kHz, above audible range
const int PWM_RES  = 8;       // 0-255

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(INT_PIN, INPUT);

  setupMotors();
  setupMPU6050();
  calibrateGyro();
  setupWiFiAndUDP();
  setupWebServer();

  lastLoopTime = millis();
  Serial.println("Setup complete. Send ARM over UDP to enable motors.");
}

// =====================================================================
void loop() {
  handleIncomingUDP();
  server.handleClient();

  // Safety: if we haven't heard from the controller recently, disarm.
  if (armed && (millis() - lastPacketTime > FAILSAFE_MS)) {
    armed = false;
    Serial.println("FAILSAFE: no control packet received, motors disarmed.");
  }

  float dt = (millis() - lastLoopTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastLoopTime = millis();

  updateAttitudeEstimate(dt);

  if (armed) {
    stabilizeAndDrive(dt);
  } else {
    setAllMotors(0, 0, 0, 0);
  }
}

// =====================================================================
// Motor setup / output
// =====================================================================
void setupMotors() {
  ledcAttach(M1_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M2_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M3_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M4_PIN, PWM_FREQ, PWM_RES);

  setAllMotors(0, 0, 0, 0);
}

void setAllMotors(int m1, int m2, int m3, int m4) {
  ledcWrite(M1_PIN, constrain(m1, 0, MAX_THROTTLE_PWM));
  ledcWrite(M2_PIN, constrain(m2, 0, MAX_THROTTLE_PWM));
  ledcWrite(M3_PIN, constrain(m3, 0, MAX_THROTTLE_PWM));
  ledcWrite(M4_PIN, constrain(m4, 0, MAX_THROTTLE_PWM));
}

// =====================================================================
// MPU6050 setup / calibration / reading
// =====================================================================
void setupMPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  writeMPURegister(REG_PWR_MGMT_1, 0x00);   // wake up
  delay(100);
  writeMPURegister(REG_GYRO_CONFIG, 0x00);  // +/-250 deg/s
  writeMPURegister(REG_ACCEL_CONFIG, 0x00); // +/-2g

  uint8_t who = readMPURegister8(0x75);
  if (who == 0x68) {
    Serial.println("MPU6050 I2C connection [OK].");
  } else {
    Serial.println("MPU6050 I2C connection [FAIL]. Check wiring before flying!");
  }
}

void writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readMPURegister8(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU6050_ADDR, 1, true);
  return Wire.available() ? Wire.read() : 0xFF;
}

void readMPURaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU6050_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();          // skip temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// Averages gyro readings while the drone is still, to remove bias.
// KEEP THE DRONE COMPLETELY STILL AND LEVEL DURING THIS STEP.
void calibrateGyro() {
  Serial.println("Calibrating gyro - keep the drone completely still and level...");
  const int N = 1000;
  long sumX = 0, sumY = 0, sumZ = 0;
  int16_t ax, ay, az, gx, gy, gz;

  for (int i = 0; i < N; i++) {
    readMPURaw(ax, ay, az, gx, gy, gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(3);
  }
  gyroOffsetX = sumX / (float)N;
  gyroOffsetY = sumY / (float)N;
  gyroOffsetZ = sumZ / (float)N;

  Serial.println("Gyro calibration done.");
  Serial.print("Offsets: ");
  Serial.print(gyroOffsetX); Serial.print(", ");
  Serial.print(gyroOffsetY); Serial.print(", ");
  Serial.println(gyroOffsetZ);
}

// Complementary filter: combines accelerometer (stable long-term,
// noisy short-term) with gyro (smooth short-term, drifts long-term).
void updateAttitudeEstimate(float dt) {
  int16_t ax, ay, az, gx, gy, gz;
  readMPURaw(ax, ay, az, gx, gy, gz);

  float gxDegS = (gx - gyroOffsetX) / 131.0;   // 131 LSB/(deg/s) at +/-250dps
  float gyDegS = (gy - gyroOffsetY) / 131.0;

  float accelPitch = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 180.0 / PI;
  float accelRoll  = atan2(ay, az) * 180.0 / PI;

  const float ALPHA = 0.98;
  pitchAngle = ALPHA * (pitchAngle + gyDegS * dt) + (1 - ALPHA) * accelPitch;
  rollAngle  = ALPHA * (rollAngle  + gxDegS * dt) + (1 - ALPHA) * accelRoll;
}

// =====================================================================
// Stabilization + motor mixing
// =====================================================================
void stabilizeAndDrive(float dt) {
  // Desired angle comes from the stick input (small tilt range)
  float targetPitch = pitchIn * 0.3;   // stick 100 -> ~30 degrees target tilt
  float targetRoll   = rollIn  * 0.3;

  float pitchError = targetPitch - pitchAngle;
  float rollError  = targetRoll  - rollAngle;

  pitchErrIntegral += pitchError * dt;
  rollErrIntegral  += rollError  * dt;
  pitchErrIntegral = constrain(pitchErrIntegral, -50, 50); // anti-windup
  rollErrIntegral  = constrain(rollErrIntegral,  -50, 50);

  float pitchDeriv = (pitchError - pitchErrPrev) / dt;
  float rollDeriv  = (rollError  - rollErrPrev)  / dt;
  pitchErrPrev = pitchError;
  rollErrPrev  = rollError;

  float pitchCorrection = PID_KP_PITCH * pitchError
                         + PID_KI_PITCH * pitchErrIntegral
                         + PID_KD_PITCH * pitchDeriv;
  float rollCorrection  = PID_KP_ROLL  * rollError
                         + PID_KI_ROLL  * rollErrIntegral
                         + PID_KD_ROLL  * rollDeriv;
  float yawCorrection   = PID_KP_YAW * yawIn * 0.1;  // simple rate-only yaw

  // Base throttle: 0-100 input mapped to PWM range (with a floor so
  // motors don't stall out under correction at low throttle)
  float basePWM = map(throttleIn, 0, 100, 0, MAX_THROTTLE_PWM);

  // Standard X-frame mixing.
  // NOTE: signs here are a starting assumption based on your layout -
  // if a stick input tilts the drone the WRONG way during bench
  // testing (props off, watch motor speed changes), flip the sign of
  // that term.
  float m1 = basePWM + pitchCorrection - rollCorrection + yawCorrection; // front-right CCW
  float m2 = basePWM - pitchCorrection - rollCorrection - yawCorrection; // back-right  CW
  float m3 = basePWM - pitchCorrection + rollCorrection + yawCorrection; // back-left   CCW
  float m4 = basePWM + pitchCorrection + rollCorrection - yawCorrection; // front-left  CW

  // Only let motors spin at all once there's real throttle input,
  // and keep a minimum spin so corrections still have authority.
  if (throttleIn < 2) {
    setAllMotors(0, 0, 0, 0);
    return;
  }

  setAllMotors(
    (int)constrain(m1, MIN_SPIN_PWM, MAX_THROTTLE_PWM),
    (int)constrain(m2, MIN_SPIN_PWM, MAX_THROTTLE_PWM),
    (int)constrain(m3, MIN_SPIN_PWM, MAX_THROTTLE_PWM),
    (int)constrain(m4, MIN_SPIN_PWM, MAX_THROTTLE_PWM)
  );
}

// =====================================================================
// WiFi + UDP control link
// =====================================================================
void setupWiFiAndUDP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi AP started. SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("Connect your phone to this network, then send UDP to: ");
  Serial.println(WiFi.softAPIP());

  udp.begin(UDP_PORT);
}

void handleIncomingUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
  if (len <= 0) return;
  packetBuffer[len] = 0;

  String msg = String(packetBuffer);
  msg.trim();

  if (msg == "ARM") {
    armed = true;
    lastPacketTime = millis();
    Serial.println("ARMED");
    return;
  }
  if (msg == "DISARM") {
    armed = false;
    Serial.println("DISARMED");
    return;
  }

  // Expect "T,R,P,Y"
  int firstComma  = msg.indexOf(',');
  int secondComma = msg.indexOf(',', firstComma + 1);
  int thirdComma  = msg.indexOf(',', secondComma + 1);
  if (firstComma < 0 || secondComma < 0 || thirdComma < 0) return;

  throttleIn = constrain(msg.substring(0, firstComma).toFloat(), 0, 100);
  rollIn     = constrain(msg.substring(firstComma + 1, secondComma).toFloat(), -100, 100);
  pitchIn    = constrain(msg.substring(secondComma + 1, thirdComma).toFloat(), -100, 100);
  yawIn      = constrain(msg.substring(thirdComma + 1).toFloat(), -100, 100);

  lastPacketTime = millis();
}

// =====================================================================
// Browser-based control page - no app install needed.
// Connect your phone to the drone's WiFi, then open http://192.168.4.1
// in any browser (Chrome, Safari, etc). Two touch joysticks control
// throttle/yaw (left) and pitch/roll (right), same as a normal RC stick
// layout. This posts to the SAME control variables as the UDP path,
// so you can use either interchangeably.
// =====================================================================
const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Drone Control</title>
<style>
  html,body{margin:0;height:100%;background:#111;color:#eee;font-family:sans-serif;
             overflow:hidden;touch-action:none;user-select:none;}
  #top{display:flex;justify-content:space-between;align-items:center;padding:10px 16px;}
  #status{font-size:14px;}
  button{font-size:16px;padding:10px 18px;border-radius:8px;border:none;font-weight:bold;}
  #armBtn{background:#2ecc71;color:#111;}
  #armBtn.armed{background:#e74c3c;color:#fff;}
  #sticks{display:flex;justify-content:space-around;align-items:center;
          height:calc(100% - 60px);}
  .stickBase{width:42vw;height:42vw;max-width:260px;max-height:260px;border-radius:50%;
             background:#222;border:2px solid #444;position:relative;}
  .stickKnob{width:35%;height:35%;border-radius:50%;background:#3498db;
             position:absolute;top:32.5%;left:32.5%;}
  #readout{position:absolute;bottom:6px;width:100%;text-align:center;font-size:12px;color:#888;}
</style>
</head>
<body>
  <div id="top">
    <div id="status">Disarmed</div>
    <button id="armBtn" onclick="toggleArm()">ARM</button>
  </div>
  <div id="sticks">
    <div class="stickBase" id="leftBase"><div class="stickKnob" id="leftKnob"></div></div>
    <div class="stickBase" id="rightBase"><div class="stickKnob" id="rightKnob"></div></div>
  </div>
  <div id="readout">T:0 R:0 P:0 Y:0</div>

<script>
let armed = false;
let throttle = 0, yaw = 0, pitch = 0, roll = 0;

function toggleArm(){
  armed = !armed;
  fetch(armed ? '/arm' : '/disarm');
  document.getElementById('armBtn').classList.toggle('armed', armed);
  document.getElementById('armBtn').innerText = armed ? 'DISARM' : 'ARM';
  document.getElementById('status').innerText = armed ? 'ARMED' : 'Disarmed';
}

function setupStick(baseId, knobId, onMove, springBackX, springBackY){
  const base = document.getElementById(baseId);
  const knob = document.getElementById(knobId);
  let dragging = false;
  const radius = () => base.clientWidth / 2;

  function handle(x, y){
    const rect = base.getBoundingClientRect();
    let dx = x - (rect.left + rect.width/2);
    let dy = y - (rect.top + rect.height/2);
    const r = radius();
    const dist = Math.sqrt(dx*dx + dy*dy);
    if (dist > r){ dx = dx / dist * r; dy = dy / dist * r; }
    knob.style.left = (50 + (dx/r)*32.5) + '%';
    knob.style.top  = (50 + (dy/r)*32.5) + '%';
    onMove(dx/r, dy/r);   // normalized -1..1
  }

  function reset(){
    dragging = false;
    const nx = springBackX ? 0 : lastX;
    const ny = springBackY ? 0 : lastY;
    knob.style.left = (50 + nx*32.5) + '%';
    knob.style.top  = (50 + ny*32.5) + '%';
    onMove(nx, ny);
  }

  let lastX = 0, lastY = 0;
  base.addEventListener('touchstart', e => { dragging = true; e.preventDefault(); });
  base.addEventListener('touchmove', e => {
    if(!dragging) return;
    const t = e.touches[0];
    handle(t.clientX, t.clientY);
    e.preventDefault();
  });
  base.addEventListener('touchend', reset);
}

// Left stick: X = yaw (spring back), Y = throttle (NO spring back - stays where released)
setupStick('leftBase','leftKnob', (nx, ny) => {
  yaw = Math.round(nx * 100);
  throttle = Math.round((-ny + 1) / 2 * 100); // up = higher throttle
}, true, false);

// Right stick: X = roll (spring back), Y = pitch (spring back)
setupStick('rightBase','rightKnob', (nx, ny) => {
  roll = Math.round(nx * 100);
  pitch = Math.round(-ny * 100);
}, true, true);

// Continuously send current stick state so the drone's failsafe stays happy
setInterval(() => {
  document.getElementById('readout').innerText =
    'T:' + throttle + ' R:' + roll + ' P:' + pitch + ' Y:' + yaw;
  fetch('/control?t=' + throttle + '&r=' + roll + '&p=' + pitch + '&y=' + yaw);
}, 100);
</script>
</body>
</html>
)HTML";

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", CONTROL_PAGE);
  });

  server.on("/control", HTTP_GET, []() {
    if (server.hasArg("t")) throttleIn = constrain(server.arg("t").toFloat(), 0, 100);
    if (server.hasArg("r")) rollIn     = constrain(server.arg("r").toFloat(), -100, 100);
    if (server.hasArg("p")) pitchIn    = constrain(server.arg("p").toFloat(), -100, 100);
    if (server.hasArg("y")) yawIn      = constrain(server.arg("y").toFloat(), -100, 100);
    lastPacketTime = millis();
    server.send(200, "text/plain", "ok");
  });

  server.on("/arm", HTTP_GET, []() {
    armed = true;
    lastPacketTime = millis();
    Serial.println("ARMED (web)");
    server.send(200, "text/plain", "armed");
  });

  server.on("/disarm", HTTP_GET, []() {
    armed = false;
    Serial.println("DISARMED (web)");
    server.send(200, "text/plain", "disarmed");
  });

  server.begin();
  Serial.println("Web control page ready - open http://192.168.4.1 in your phone's browser.");
}

/*
  =====================================================================
  BENCH TEST CHECKLIST - do this before ever attaching propellers
  =====================================================================
  1. Flash this sketch, open Serial Monitor at 115200 baud.
     Confirm "MPU6050 I2C connection [OK]" and gyro calibration finish
     with the drone lying flat and still.

  2. Connect your phone to the "MyDrone" WiFi network (or whatever you
     set WIFI_SSID to).

  3. Send "ARM\n" via UDP to 192.168.4.1:4210 (RoboRemo, or any UDP
     test app / terminal). Then send a low throttle value, e.g.
     "10,0,0,0\n" and confirm all 4 motors spin at low, EQUAL speed.

  4. With props OFF, tilt the drone by hand and watch the motor speeds
     on the serial log or by ear - motors resisting the tilt (speeding
     up on the side going down) means the correction direction is
     correct. If a motor speeds up on the wrong side, flip that
     term's sign in stabilizeAndDrive().

  5. Only after step 4 checks out, fit propellers with correct
     CW/CCW orientation per the M1-M4 labels in your diagram, and do
     your first hover attempt on a soft surface with the throttle
     stick moved up slowly.

  6. Expect to nudge PID_KP_ROLL / PID_KP_PITCH (and D/I terms) up or
     down based on how it behaves - oscillating means too much P/D,
     sluggish/drifting means too little.

  =====================================================================
  ROBOREMO APP SETUP (or any similar WiFi/UDP joystick app)
  =====================================================================
  1. Install RoboRemo (Android/iOS).
  2. Connect setting: UDP, IP 192.168.4.1, Port 4210 (send), and same
     port to listen if you want telemetry back later.
  3. Add a joystick widget for Roll/Pitch: configure it to send text
     on move, formatted as a custom string combining 4 variables you
     define as sliders/joysticks: throttle, roll, pitch, yaw, joined
     with commas and ending in \n, e.g. "{throttle},{roll},{pitch},{yaw}\n"
     (RoboRemo supports templated send-strings referencing other
     widgets' current values - check its "Send text" widget docs).
  4. Add two buttons: one sends "ARM\n", one sends "DISARM\n".
  =====================================================================
*/
