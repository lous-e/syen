#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// IMPORTANT: Put your credentials/tokens in a separate, untracked file or
// define them via the build system. Do NOT check secrets into source
// control. The fields below are intentionally left empty.
const char* ssid      = "";
const char* password  = "";
const char* BOT_TOKEN = "";
const char* CHAT_ID   = "";

#define ENABLE_HEARTBEAT 1      // 1=send periodic status
#define ENABLE_DEBUG_LOG 0      // 1=serial debug prints
#define USE_IMPULSE      0      // 0=disable legacy impulse detector (recommended)
#define SDA_PIN 21
#define SCL_PIN 22

// -------------------- GLOBALS --------------------
Adafruit_LSM6DSOX imu;
WiFiClientSecure  secureClient;
HTTPClient        http;

// Timing (non-Morse infrastructure)
static const unsigned long HEARTBEAT_MS    = 10000;
static const unsigned long WIFI_LOG_MS     = 5000;
static const unsigned long SEND_SPACING_MS = 120;
static const uint32_t     SAMPLE_US        = 2500;   // ~400 Hz

// Legacy impulse path constants (kept but disabled by USE_IMPULSE)
static const unsigned long ALERT_GAP_MS = 160;
static const unsigned long PEAK_WIN_MS  = 150;
static const float A2_THR       = 5.0f;
static const float JERK_THR     = 60.0f;
static const float GATE_GYRO_RAD= 0.7f;

// Gravity HPF (only used for legacy impulse plotting)
static const float G_INIT_X = 0.0f, G_INIT_Y = 0.0f, G_INIT_Z = 9.80665f;
static const float TAU      = 0.35f;

// State
unsigned long lastHeartbeat = 0;
unsigned long lastWifiLog   = 0;
unsigned long lastSendMs    = 0;
uint32_t      nextSampleUS  = 0;

#if USE_IMPULSE
unsigned long lastAlert     = 0;
static unsigned long merge_until = 0;
float gX=G_INIT_X, gY=G_INIT_Y, gZ=G_INIT_Z;
float prevA2 = 0.0f;
unsigned long peakStart = 0;
float prevGyY = 0.0f;
#endif

// -------------------- SIMPLE MESSAGE QUEUE --------------------
#define MSG_QUEUE_LEN 8
struct MsgQueue {
  String items[MSG_QUEUE_LEN];
  int head=0, tail=0, count=0;
} mq;

void q_push(const String& s) {
  if (mq.count < MSG_QUEUE_LEN) {
    mq.items[mq.tail] = s;
    mq.tail = (mq.tail + 1) % MSG_QUEUE_LEN;
    mq.count++;
  } else {
    mq.items[mq.head] = s;
    mq.head = (mq.head + 1) % MSG_QUEUE_LEN;
    mq.tail = mq.head;
    mq.count = MSG_QUEUE_LEN;
  }
}

bool q_pop(String& out) {
  if (mq.count == 0) return false;
  out = mq.items[mq.head];
  mq.head = (mq.head + 1) % MSG_QUEUE_LEN;
  mq.count--;
  return true;
}

// -------------------- UTILS --------------------
String urlEncode(const String& s) {
  String out; out.reserve(s.length()*3);
  char buf[4];
  for (size_t i=0;i<s.length();i++) {
    char c = s[i];
    if (isalnum(c)||c=='-'||c=='_'||c=='.'||c=='~') out += c;
    else { snprintf(buf,sizeof(buf),"%%%02X",(unsigned char)c); out += buf; }
  }
  return out;
}

bool sendTelegram(const String& text) {
  if (!BOT_TOKEN[0] || !CHAT_ID[0]) return false;
  String endpoint = "https://api.telegram.org/bot" + String(BOT_TOKEN) +
                    "/sendMessage?chat_id=" + CHAT_ID +
                    "&text=" + urlEncode(text);
  secureClient.setInsecure();
  http.begin(secureClient, endpoint);
  int code = http.GET();
  http.end();
  return (code >= 200 && code < 300);
}

void pumpSends() {
  if (mq.count == 0) return;
  unsigned long now = millis();
  if (WiFi.status() != WL_CONNECTED) return;
  if (now - lastSendMs < SEND_SPACING_MS) return;
  String msg;
  if (q_pop(msg)) { (void)sendTelegram(msg); lastSendMs = now; }
}

// -------------------- TRUE MORSE FROM 6-DoF (ON-duration) --------------------
// ----- TRUE MORSE FROM 6-DoF (ON-duration) with anti-noise gating + relaxed timing -----
void updateMorse6DoF(const sensors_event_t& a, const sensors_event_t& g, unsigned long now_ms) {
  // (A) Activity thresholds (fixed, non-adaptive)
  const float THR_AX=0.8f, THR_AY=0.8f, THR_AZ=0.8f;      // m/s^2 deltas
  const float THR_WX=0.20f, THR_WY=0.20f, THR_WZ=0.20f;   // rad/s deltas
  const float ON_THR  = 1.0f;     // enter ON when D >= 1.0 (debounced)
  const float OFF_THR = 0.45f;    // leave ON when D <= 0.45 (debounced)
  const unsigned long ON_DEBOUNCE_MS  = 40;
  const unsigned long OFF_DEBOUNCE_MS = 180;

  // (B) Human-friendly timing windows (slightly relaxed)
  const unsigned long DOT_MIN  = 400,  DOT_MAX  = 1800;
  const unsigned long DASH_MIN = 2000, DASH_MAX = 4500;
  const unsigned long LET_MIN  = 4200;     // letter gap when OFF crosses >= 4.2 s (one-shot)
  const unsigned long MSG_END  = 7500;     // end of message when OFF >= 7.5 s

  // (C) Anti-noise gating
  const unsigned long PRE_SIL_MS     = 250;  // must be idle this long before starting a new ON
  const unsigned long MIN_INTRA_SIL  = 200;  // minimum OFF time between elements
  const float        MIN_PEAK_D      = 1.35f;// ON must reach at least 35% above ON_THR
  const unsigned long RISE_WIN_MS    = 150;  // look at the first 150ms of ON
  const float        MIN_RISE_DELTA  = 0.25f;// within RISE_WIN_MS, D must rise by at least 0.25

  // (D) Baseline & state
  static bool calibDone=false; static unsigned long calibEnd=0; static uint32_t nCal=0;
  static float ax0=0,ay0=0,az0=0, wx0=0,wy0=0,wz0=0;

  enum {IDLE=0, ON=1};
  static int state=IDLE;
  static unsigned long onStart=0, lastAbove=0, lastBelow=0;
  static unsigned long idleSince=0;        // time we truly entered IDLE
  static bool letterGapFired=false;

  static float D_onset=0.0f, D_peak=0.0f, D_risePeak=0.0f;

  static String letter="", morse="", decoded="";

  struct M { const char* code; char ch; };
  static const M MAP[] = {
    {".-", 'A'},   {"-...", 'B'}, {"-.-.", 'C'}, {"-..", 'D'},  {".", 'E'},
    {"..-.", 'F'}, {"--.", 'G'},  {"....", 'H'}, {"..", 'I'},   {".---", 'J'},
    {"-.-", 'K'},  {".-..", 'L'}, {"--", 'M'},   {"-.", 'N'},   {"---", 'O'},
    {".--.", 'P'}, {"--.-", 'Q'}, {".-.", 'R'},  {"...", 'S'},  {"-", 'T'},
    {"..-", 'U'},  {"...-", 'V'}, {".--", 'W'},  {"-..-", 'X'}, {"-.--", 'Y'},
    {"--..", 'Z'},
    {"-----", '0'}, {".----", '1'}, {"..---", '2'}, {"...--", '3'}, {"....-", '4'},
    {".....", '5'}, {"-....", '6'}, {"--...", '7'}, {"---..", '8'}, {"----.", '9'}
  };

  auto flushLetter = [&](){
    if (letter.length()==0) return;
    if (morse.length()>0) morse += ' ';
    morse += letter;
    char out='?';
    for (size_t i=0;i<sizeof(MAP)/sizeof(M);++i) if (letter==MAP[i].code) { out=MAP[i].ch; break; }
    decoded += out;
    letter = "";
  };
  auto pushMessage = [&](){
    if (morse.length()==0 && decoded.length()==0) return;
    String out = "📨 Morse (6DoF): " + morse;
    if (decoded.length()>0) out += "\n🔤 Text: " + decoded;
    q_push(out);
    morse=""; decoded=""; letter="";
  };

  // ---------- Calibration (~1.2 s still) ----------
  if (!calibDone) {
    if (calibEnd==0) calibEnd = now_ms + 3000;
    ax0=(ax0*nCal + a.acceleration.x)/(nCal+1);
    ay0=(ay0*nCal + a.acceleration.y)/(nCal+1);
    az0=(az0*nCal + a.acceleration.z)/(nCal+1);
    wx0=(wx0*nCal + g.gyro.x)        /(nCal+1);
    wy0=(wy0*nCal + g.gyro.y)        /(nCal+1);
    wz0=(wz0*nCal + g.gyro.z)        /(nCal+1);
    nCal++;
    if ((long)(now_ms - calibEnd) >= 0) calibDone = true;
    return;
  }

  // ---------- Deviation metric D(t) ----------
  float dax=fabsf(a.acceleration.x-ax0), day=fabsf(a.acceleration.y-ay0), daz=fabsf(a.acceleration.z-az0);
  float dwx=fabsf(g.gyro.x-wx0),        dwy=fabsf(g.gyro.y-wy0),        dwz=fabsf(g.gyro.z-wz0);
  float D=0.0f;
  D=fmaxf(D, dax/THR_AX); D=fmaxf(D, day/THR_AY); D=fmaxf(D, daz/THR_AZ);
  D=fmaxf(D, dwx/THR_WX); D=fmaxf(D, dwy/THR_WY); D=fmaxf(D, dwz/THR_WZ);

  // Allow tiny baseline creep ONLY when clearly idle for a while
  if (D < 0.25f && idleSince!=0 && (now_ms - idleSince) > 600) {
    const float BETA=0.002f;
    ax0=(1-BETA)*ax0 + BETA*a.acceleration.x;
    ay0=(1-BETA)*ay0 + BETA*a.acceleration.y;
    az0=(1-BETA)*az0 + BETA*a.acceleration.z;
    wx0=(1-BETA)*wx0 + BETA*g.gyro.x;
    wy0=(1-BETA)*wy0 + BETA*g.gyro.y;
    wz0=(1-BETA)*wz0 + BETA*g.gyro.z;
  }

  // ---------------- FSM ----------------
  if (state==IDLE) {
    // Only allow new ON after a quiet pre-silence
    bool preSilOK = (idleSince==0) ? true : ((now_ms - idleSince) >= PRE_SIL_MS);

    if (D >= ON_THR && preSilOK) {
      if (lastAbove==0) lastAbove=now_ms;
      if (now_ms - lastAbove >= ON_DEBOUNCE_MS) {
        state = ON;
        onStart = now_ms;
        lastAbove = 0;
        letterGapFired = false;
        // capture onset features
        D_onset = D;
        D_peak = D;
        D_risePeak = D;
        // mark that OFF timing is no longer running
        idleSince = 0;
      }
    } else {
      lastAbove = 0;
      // OFF-silence accounting (letter gap & message end)
      if (idleSince!=0) {
        unsigned long sil = now_ms - idleSince;
        if (!letterGapFired && sil >= LET_MIN) { flushLetter(); letterGapFired=true; }
        if (sil >= MSG_END) { flushLetter(); pushMessage(); idleSince=0; letterGapFired=false; }
      }
    }
  } else { // ON
    // Track peaks for gating
    if (D > D_peak) D_peak = D;
    if (now_ms - onStart <= RISE_WIN_MS && D > D_risePeak) D_risePeak = D;

    // Try to exit ON
    if (D <= OFF_THR) {
      if (lastBelow==0) lastBelow=now_ms;
      if (now_ms - lastBelow >= OFF_DEBOUNCE_MS) {
        unsigned long dur = now_ms - onStart;

        // Basic duration sanity
        bool durationOK = (dur >= DOT_MIN && dur <= DASH_MAX);
        // Anti-noise gating: strong enough + clear initial rise
        bool peakOK = (D_peak >= MIN_PEAK_D);
        bool riseOK = ((D_risePeak - D_onset) >= MIN_RISE_DELTA);

        // Enforce a minimum OFF between elements (refractory)
        bool intraSilOK = true;
        if (idleSince!=0) intraSilOK = ((now_ms - idleSince) >= MIN_INTRA_SIL);

        if (durationOK && peakOK && riseOK && intraSilOK) {
          // Optional snap of the gray band near 1.8–2.0 s
          if (dur > 1700 && dur < 2000) dur = (dur < 1850) ? DOT_MAX : DASH_MIN;

          if      (dur >= DOT_MIN  && dur <= DOT_MAX)  letter += '.';
          else if (dur >= DASH_MIN && dur <= DASH_MAX) letter += '-';
          // else outside dot/dash ⇒ ignore
        }
        // Reset to IDLE
        state = IDLE;
        idleSince = now_ms;       // OFF timing starts here
        lastBelow = 0;
      }
    } else { lastBelow = 0; }
  }

#if ENABLE_DEBUG_LOG
  // Serial.printf("D=%.2f st=%d peak=%.2f rise=%.2f dur? %lu\n", D, state, D_peak, D_risePeak-D_onset, (state==ON)?(now_ms-onStart):0);
#endif
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  // Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  for (int i=0;i<40 && WiFi.status()!=WL_CONNECTED;i++){ delay(500); Serial.print("."); }
  Serial.println();
  if (WiFi.status()==WL_CONNECTED) { Serial.print("IP: "); Serial.println(WiFi.localIP()); }
  else { Serial.println("Wi-Fi connect FAILED (continuing offline)."); }

  // I2C + IMU
  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000);
  if (!imu.begin_I2C(0x6A, &Wire) && !imu.begin_I2C(0x6B, &Wire)) {
    Serial.println("ERR: LSM6DSOX not found. Check wiring/address.");
    while(1) delay(10);
  }
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imu.setGyroRange (LSM6DS_GYRO_RANGE_1000_DPS);
  imu.setAccelDataRate(LSM6DS_RATE_416_HZ);
  imu.setGyroDataRate (LSM6DS_RATE_416_HZ);

  if (WiFi.status()==WL_CONNECTED) q_push("🚀 ESP32 6DoF Morse booted");
  nextSampleUS = micros();
}

// -------------------- LOOP --------------------
void loop() {
  unsigned long now_ms = millis();
  pumpSends(); // non-blocking Telegram drain

  // Heartbeat
#if ENABLE_HEARTBEAT
  if (now_ms - lastHeartbeat >= HEARTBEAT_MS) {
    lastHeartbeat = now_ms;
    sensors_event_t a,g,t; imu.getEvent(&a,&g,&t);
    String beat = "📡 ESP32 Status\nIP: " + WiFi.localIP().toString() +
                  "\nRSSI: " + String(WiFi.RSSI()) + " dBm\n" +
                  "Accel: ("+String(a.acceleration.x,2)+","+String(a.acceleration.y,2)+","+String(a.acceleration.z,2)+") m/s^2\n" +
                  "Gyro: (" +String(g.gyro.x,2)+","+String(g.gyro.y,2)+","+String(g.gyro.z,2)+") rad/s\n" +
                  "Temp: " + String(t.temperature,1) + "°C";
    q_push(beat);
  }
#endif

  // Wi-Fi status log
  if (now_ms - lastWifiLog >= WIFI_LOG_MS) {
    lastWifiLog = now_ms;
#if ENABLE_DEBUG_LOG
    if (WiFi.status()==WL_CONNECTED) { Serial.print("RSSI: "); Serial.println(WiFi.RSSI()); }
    else                             { Serial.println("Wi-Fi disconnected."); }
#endif
  }

  // ----------- High-rate sampler (≈400 Hz) -----------
  uint32_t now_us = micros();
  if ((int32_t)(now_us - nextSampleUS) >= 0) {
    nextSampleUS += SAMPLE_US;

    static uint32_t prev_us = 0;
    float dt = (prev_us==0) ? (SAMPLE_US/1e6f) : ((now_us - prev_us)/1e6f);
    prev_us = now_us;
    if (dt < 0.0005f) dt = SAMPLE_US/1e6f;

    sensors_event_t a,g,t; imu.getEvent(&a,&g,&t);
    updateMorse6DoF(a, g, now_ms);

#if USE_IMPULSE
    // ---------- Legacy impulse detector (disabled by default) ----------
    float alpha = dt / (TAU + dt);
    gX=(1.0f-alpha)*gX + alpha*a.acceleration.x;
    gY=(1.0f-alpha)*gY + alpha*a.acceleration.y;
    gZ=(1.0f-alpha)*gZ + alpha*a.acceleration.z;
    float axHP=a.acceleration.x-gX, ayHP=a.acceleration.y-gY, azHP=a.acceleration.z-gZ;
    float a2HP = axHP*axHP + ayHP*ayHP + azHP*azHP;
    float jerk = fabsf(a2HP - prevA2) / fmaxf(dt, 1e-3f);
    prevA2 = a2HP;
    bool tap_def = (fabsf(azHP) > 1.0f) && (fabsf(prevGyY) < 0.15f && fabsf(g.gyro.y) > 0.25f);
    prevGyY = g.gyro.y;

    if (now_ms < merge_until) { }
    else if ((a2HP > A2_THR || jerk > JERK_THR) && tap_def) { if (!peakStart) peakStart = now_ms; }
    else if (peakStart) {
      unsigned long width = now_ms - peakStart;
      if (width <= PEAK_WIN_MS && (now_ms - lastAlert) >= ALERT_GAP_MS) {
        lastAlert = now_ms; // not used by Morse path anymore
        // q_push("✅ (impulse tap)");  // keep silent to avoid confusion
        merge_until = now_ms + 200;
      }
      peakStart = 0;
    }
#endif
  }

  delay(0); // yield to Wi-Fi
}