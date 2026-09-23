#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include <Adafruit_AS5600.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include <WebServer.h>
#include <Update.h>

// ---- Parameter-Tabellen: Structs muessen ganz oben stehen, noch vor allen
// Funktionen. Grund: Die Arduino-IDE generiert automatisch Funktions-Prototypen
// und fuegt sie ganz am Dateianfang ein (noch vor der bisherigen Position dieser
// Structs) - dort waeren FloatParam/ULongParam/LongParam sonst unbekannt
// ("does not name a type"), z.B. fuer paramRowFloat(const FloatParam&) im Web-UI.
struct FloatParam { const char* mqttName; const char* prefKey; float* var; float minV, maxV, stepV; };
struct ULongParam { const char* mqttName; const char* prefKey; unsigned long* var; unsigned long minV, maxV, stepV; };
struct LongParam { const char* mqttName; const char* prefKey; long* var; long minV, maxV, stepV; };
// Eigene, fruehe Prototypen fuer die Web-UI-Helfer, die diese Structs als Parameter
// nehmen (Definition der Funktionen selbst folgt viel spaeter, kurz vor setupOTA()).
// Ohne das wuerde die Arduino-IDE ihre eigenen Auto-Prototypen ganz oben einfuegen,
// noch VOR dieser Struct-Definition -> "does not name a type". Mit einer bereits
// vorhandenen, passenden Deklaration hier ueberspringt die IDE die Auto-Generierung.
String paramRowFloat(const FloatParam& p);
String paramRowULong(const ULongParam& p);
String paramRowLong(const LongParam& p);

// ------------------- Pin-Belegung -------------------
#define EN_PIN       6
#define DIR_PIN      7
#define STEP_PIN     5
#define HALL1_PIN    13
#define HALL2_PIN    12
#define ANALOG_PIN_A  4
#define ANALOG_PIN_B  3
#define I2C_SDA       9
#define I2C_SCL       8

// ------------------- Richtungs-Pegel -------------------
#define DIR_CLOSE  LOW
#define DIR_OPEN   HIGH
const bool DEBUG_ENABLED = true;

// ------------------- Bewegungsparameter -------------------
float MAX_SPEED = 2000.0f;
const float ACCEL = 800.0;
float DECEL = 1800.0f;
long  BLANK_STEPS = 400;
long  LEARN_BLANK_STEPS = 20;

unsigned long LEARN_PAUSE_MS     = 5000;
unsigned long AUTOCLOSE_DELAY_MS = 60000;
const unsigned long DEBUG_INTERVAL_MS = 1000;
const unsigned long SETTLE_TIME_MS    = 1000;

const bool HALL_ACTIVE_LOW = true;

// ------------------- Filter -------------------
#define MEDIAN_WINDOW  7
const float EMA_ALPHA = 0.10f;

// ------------------- Sicherheit -------------------
float WEIGHT_PLAUSIBLE_MIN    = -1200.0f;
float WEIGHT_PLAUSIBLE_MAX    =  1200.0f;
unsigned long SAFETY_ARM_DELAY_MS = 200;
unsigned long LEARN_ARM_DELAY_MS  = 800;
float weightStopDelta         = 80.0f;
float CAL_REFERENCE_WEIGHT    = 1000.0f;

const unsigned long ERROR_RETRY_DELAY_MS = 10000;

// ------------------- Laufzeitueberwachung Sensoren (NEU) -------------------
// Wie lange die Waage keine neue Messung liefern darf, bevor sie als
// ausgefallen markiert wird (z.B. loses I2C-Kabel zur NAU7802).
const unsigned long SCALE_STALE_TIMEOUT_MS  = 2000;
// Wie lange der AS5600 den Magneten nicht erkennen darf, bevor der
// Encoder als ausgefallen markiert wird (z.B. Dejustage/Luftspalt).
const unsigned long ENCODER_LOST_TIMEOUT_MS = 2000;

// ------------------- Einklemmschutz -------------------
const long  REVERSE_STEPS = 230;
const float REVERSE_SPEED = 400.0;

// ------------------- Manuell-Modus -------------------
const unsigned long IDLE_SETTLE_MS = 500;

// ------------------- Anwesenheitserkennung -------------------
long PRESENCE_ENCODER_DELTA   = 15;
long ENCODER_OPEN_THRESHOLD   = 200;
long CLOSE_POSITION_TOLERANCE = 80;

// ------------------- LEDs -------------------
const int           LED_A_IDLE_BRIGHTNESS   = 64;
const unsigned long LED_A_BREATHE_PERIOD_MS = 1000;
const unsigned long LED_B_BLINK_PERIOD_MS   = 500;
const unsigned long LED_UPDATE_INTERVAL_MS  = 20;

// ------------------- Betriebsart -------------------
String opMode = "Normal";
volatile bool doorEnabled    = true;
bool          doorEnabledLast = true;

// ------------------- Lernphase -------------------
long  LEARN_CYCLES       = 4;
const float LEARN_SAFETY_MARGIN = 1.8f;
float LEARN_SPEED_FACTOR = 0.5f;

bool  learnMode      = true;
int   learnCount     = 0;
float learnMaxDev    = 0.0f;
float learnMaxDevRun = 0.0f;
float learnMaxDevOpen    = 0.0f;
float learnMaxDevRunOpen = 0.0f;
float activeMaxSpeed = 0.0f;
float activeAccel    = 0.0f;
float activeDecel    = 0.0f;

// ============================================================
//  Zustandsmaschine
// ============================================================
enum State { ACCELERATING, CRUISING, DECELERATING, STOPPED,
             PAUSING, REVERSING, EMERGENCY, IDLE, OFF_STATE };
volatile State state = ACCELERATING;

float         currentSpeed    = 0.0;
unsigned long lastStepTime    = 0;
unsigned long lastSpeedUpdate = 0;
volatile long stepCounter     = 0;
volatile bool closing         = true;
unsigned long pauseStart      = 0;
unsigned long pauseDuration   = 0;
unsigned long moveStart       = 0;

bool          positionKnown       = false;
bool          weightCaptured      = false;
volatile float validWeight        = 0.0;
volatile float weightReference    = 10000.0;
volatile bool  weightReferenceValid = false;

bool          pauseIsAutocloseWait = false;
long          presenceEncoderLast  = 0;
bool          presenceForceActive  = false;

unsigned long emergencyStart               = 0;
unsigned long EMERGENCY_MOTOR_OFF_DELAY_MS = 1000;
bool          emergencyMotorOff            = false;
long          emergencyEncoderRef          = 0;

// ------------------- MQTT-Befehle -------------------
volatile bool  cmdForceClose    = false;
volatile bool  cmdReleaseError  = false;
volatile bool  cmdReset         = false;
volatile bool  cmdPublishParams = false;
volatile bool  cmdRelearn       = false;

// "Warte"-Schalter: blockiert automatisches Schliessen nach Timeout.
// Core 0 schreibt, Core 1 liest.
volatile bool  warteAktiv     = false;
// Merker: Timeout abgelaufen, aber Schliessen durch Warte blockiert.
// Sobald warteAktiv=false -> Schliessen startet sofort.
volatile bool  warteBlockiert = false;

volatile State     stateForMqtt                = ACCELERATING;
volatile bool      pauseIsAutocloseWaitForMqtt = false;
volatile bool      learnModeForMqtt            = true;
volatile int       learnCountForMqtt           = 0;

enum LastEvent { EVT_NONE, EVT_MANUAL_OPEN };
volatile LastEvent lastEventForMqtt = EVT_NONE;

bool          doorIsClosed = true;
unsigned long idleStart    = 0;
bool          idleRefValid = false;

volatile bool webServerReady = false;  // gesetzt sobald webServer.begin() gelaufen ist
Preferences prefs;

// ============================================================
//  Logging  +  StopReason (Forward-Deklaration vor emergencyStop)
// ============================================================
enum StopReason {
  STOP_NONE = 0,
  STOP_WEIGHT_IMPLAUSIBLE = 1,
  STOP_WEIGHT_DELTA       = 2,
  STOP_SCALE_TIMEOUT      = 3,
};

enum LogLevel { LOG_INFO, LOG_WARN, LOG_ERROR };

// ---- Log-Ringpuffer fuer Web-Anzeige (/log) ----
#define LOG_RING_SIZE  15
#define LOG_LINE_LEN  192

static char   logRing[LOG_RING_SIZE][LOG_LINE_LEN];
static int    logRingHead  = 0;
static int    logRingCount = 0;
static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

void logMsg(LogLevel lvl, const char* tag, const char* fmt, ...) {
  static const char* lvlStr[] = { "INFO", "WARN", "ERROR" };
  char buf[LOG_LINE_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  char line[LOG_LINE_LEN];
  snprintf(line, sizeof(line), "[%10lu] %-5s %-6s %s", millis(), lvlStr[lvl], tag, buf);
  Serial.println(line);
  portENTER_CRITICAL(&logMux);
  strncpy(logRing[logRingHead], line, LOG_LINE_LEN - 1);
  logRing[logRingHead][LOG_LINE_LEN - 1] = '\0';
  logRingHead = (logRingHead + 1) % LOG_RING_SIZE;
  if (logRingCount < LOG_RING_SIZE) logRingCount++;
  portEXIT_CRITICAL(&logMux);
}

// ============================================================
//  WLAN / MQTT
// ============================================================
// WLAN/MQTT sind ueber die Konfigurationsseiten /wifi und /mqtt zur Laufzeit aenderbar
// und werden in Preferences (NVS) persistiert - genau wie die Tuer-Parameter ueber MQTT.
// Die Werte hier sind nur die Werkseinstellung fuer den allerersten Start (noch keine NVS-Daten).
char     wifiSsid[33]     = "MyBox";
char     wifiPassword[65] = "Ak29#00157";
char     mqttHost[65]     = "192.168.178.127";
uint16_t mqttPort         = 1883;
char     mqttUser[33]     = "smarthome";
char     mqttPass[65]     = "Ak29#00157";

#define MQTT_NODE_ID "barn_door"
#define MQTT_BASE    "barn_door"

const char* OTA_HOSTNAME = MQTT_NODE_ID;
const char* OTA_PASSWORD = "Ak29#00157";   // UNBEDINGT aendern!
volatile bool otaInProgress   = false;
volatile bool otaActiveForMqtt = false;

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
TaskHandle_t mqttTaskHandle = NULL;
WebServer    webServer(80);

// ---- Parameter-Tabellen (Structs siehe ganz oben nach den #include) ----
FloatParam floatParams[] = {
  { "Max_speed",            "p_max_speed",  &MAX_SPEED,             200.0f,  6000.0f, 50.0f  },
  { "Decel_Speed",          "p_decel",      &DECEL,                 200.0f,  8000.0f, 50.0f  },
  { "Min_Movement_Weight",  "p_w_min",      &WEIGHT_PLAUSIBLE_MIN, -2000.0f,    0.0f, 10.0f  },
  { "Max_Movement_Weight",  "p_w_max",      &WEIGHT_PLAUSIBLE_MAX,     0.0f, 2000.0f, 10.0f  },
  { "Collision_weight",     "stop_delta",   &weightStopDelta,        10.0f,  500.0f, 10.0f  },
  { "Learn_Speed_Factor",   "p_learn_fac",  &LEARN_SPEED_FACTOR,      0.1f,    1.0f,  0.05f },
  { "Cal_Reference_Weight", "p_cal_ref",    &CAL_REFERENCE_WEIGHT,    1.0f,50000.0f,  1.0f  },
};
const int FLOAT_PARAM_COUNT = sizeof(floatParams)/sizeof(floatParams[0]);

ULongParam ulongParams[] = {
  { "Learn_Pause_Time",          "p_learn_pause", &LEARN_PAUSE_MS,                500, 120000,  500 },
  { "Activation_time",           "p_arm_delay",   &SAFETY_ARM_DELAY_MS,             0,   5000,   50 },
  { "Learn_Activation_Time",     "p_learn_arm",   &LEARN_ARM_DELAY_MS,              0,   5000,   50 },
  { "Autoclose_Delay",           "p_autoclose",   &AUTOCLOSE_DELAY_MS,           1000,3600000, 1000 },
  { "Emergency_Motor_Off_Delay", "p_em_off",      &EMERGENCY_MOTOR_OFF_DELAY_MS,    0,  60000,  100 },
};
const int ULONG_PARAM_COUNT = sizeof(ulongParams)/sizeof(ulongParams[0]);

LongParam longParams[] = {
  { "Blank_Steps",              "p_blank_steps", &BLANK_STEPS,              0, 5000, 10 },
  { "Learn_Blank_Steps",        "p_learn_blank", &LEARN_BLANK_STEPS,        0, 2000,  5 },
  { "Learn_Cycles",             "p_learn_cyc",   &LEARN_CYCLES,             1,   50,  1 },
  { "Presence_Encoder_Delta",   "p_presence_ed", &PRESENCE_ENCODER_DELTA,   1, 1000,  1 },
  { "Encoder_Open_Threshold",   "p_open_thr",    &ENCODER_OPEN_THRESHOLD,   0,  500,  1 },
  { "Close_Position_Tolerance", "p_close_tol",   &CLOSE_POSITION_TOLERANCE, 0,  500,  5 },
};
const int LONG_PARAM_COUNT = sizeof(longParams)/sizeof(longParams[0]);

bool floatParamDirty[FLOAT_PARAM_COUNT] = { false };
bool ulongParamDirty[ULONG_PARAM_COUNT] = { false };
bool longParamDirty[LONG_PARAM_COUNT]   = { false };
bool modeDirty = false;

// ============================================================
//  Waegezelle + Encoder
// ============================================================
NAU7802 scale;
volatile float scaleWeight = 0.0;
volatile bool  scaleReady  = false;
// NEU: Zeitpunkt der letzten erfolgreichen Waagen-Messung -> fuer Staleness-Erkennung
volatile unsigned long lastScaleReadMs = 0;
TaskHandle_t   scaleTaskHandle = NULL;
long   calOffset  = 0;
float  calFactor  = 1000.0f;
volatile bool calOffsetDirty = false;
volatile bool cmdTare    = false;
volatile bool cmdCalGain = false;

Adafruit_AS5600 as5600;
volatile bool encoderReady    = false;
// NEU: Zeitpunkt der letzten erfolgreichen Magnet-Erkennung -> fuer Staleness-Erkennung
volatile unsigned long lastMagnetMs = 0;
volatile long encoderPosition = 0;
const bool ENCODER_INVERT     = true;
const int  AS5600_MAX_INIT_ATTEMPTS = 10;

// ============================================================
//  Motor-Hilfsfunktionen
// ============================================================
bool hallActive(int pin) {
  return HALL_ACTIVE_LOW ? (digitalRead(pin) == LOW) : (digitalRead(pin) == HIGH);
}
int targetHall() { return closing ? HALL1_PIN : HALL2_PIN; }

bool closeAllowed() {
  if (hallActive(HALL1_PIN)) return false;
  if (encoderReady && encoderPosition <= CLOSE_POSITION_TOLERANCE) return false;
  return true;
}

void syncEncoderToClosed() {
  if (encoderReady && encoderPosition != 0) {
    logMsg(LOG_INFO, "ENC", "Encoder (%ld) auf 0 nachgezogen.", encoderPosition);
    encoderPosition = 0;
  }
}

const char* stateName(State s) {
  switch(s) {
    case ACCELERATING: return "BESCHLEUNIGEN";
    case CRUISING:     return "KONSTANT";
    case DECELERATING: return "BREMSEN";
    case STOPPED:      return "GESTOPPT";
    case PAUSING:      return "PAUSE";
    case REVERSING:    return "RUECKZUG";
    case EMERGENCY:    return "NOT-STOPP";
    case IDLE:         return "IDLE/MOTOR-AUS";
    case OFF_STATE:    return "AUS";
  }
  return "?";
}

void updateSpeed() {
  unsigned long now = micros();
  float dt = (now - lastSpeedUpdate) / 1000000.0f;
  if (dt < 0.001f) return;
  lastSpeedUpdate = now;
  switch (state) {
    case ACCELERATING:
      currentSpeed += activeAccel * dt;
      if (currentSpeed >= activeMaxSpeed) { currentSpeed = activeMaxSpeed; state = CRUISING; }
      break;
    case CRUISING:    currentSpeed = activeMaxSpeed; break;
    case DECELERATING:
      currentSpeed -= activeDecel * dt;
      if (currentSpeed <= 0.0f) { currentSpeed = 0.0f; state = STOPPED; }
      break;
    case REVERSING:
      currentSpeed += ACCEL * dt;
      if (currentSpeed >= REVERSE_SPEED) currentSpeed = REVERSE_SPEED;
      break;
    default: currentSpeed = 0.0f; break;
  }
}

void generateStep() {
  if (currentSpeed <= 0.0f) return;
  unsigned long now      = micros();
  unsigned long interval = (unsigned long)(1000000.0f / currentSpeed);
  if (now - lastStepTime >= interval) {
    lastStepTime = now;
    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(3); digitalWrite(STEP_PIN, LOW);
    stepCounter++;
  }
}

bool motorIsMoving() {
  return (state==ACCELERATING||state==CRUISING||state==DECELERATING||state==REVERSING);
}

void startMovement() {
  if (otaInProgress) {
    static unsigned long lw=0;
    if (millis()-lw>2000){lw=millis();logMsg(LOG_WARN,"OTA","Fahrt verweigert - Update laeuft.");}
    return;
  }
  if (closing && !learnMode && !scaleReady) {
    static unsigned long lw=0;
    if (millis()-lw>2000){lw=millis();logMsg(LOG_WARN,"SAFE","Schliessen verweigert - Waage nicht bereit!");}
    return;
  }
  digitalWrite(DIR_PIN, closing ? DIR_CLOSE : DIR_OPEN);
  delayMicroseconds(5);
  digitalWrite(EN_PIN, LOW);
  stepCounter=0; currentSpeed=0.0; state=ACCELERATING;
  lastSpeedUpdate=micros(); lastStepTime=micros(); moveStart=millis();
  learnMaxDevRun=0.0f; learnMaxDevRunOpen=0.0f;
  bool cautious = learnMode || !positionKnown;
  if (cautious) { activeMaxSpeed=MAX_SPEED*LEARN_SPEED_FACTOR; activeAccel=ACCEL*LEARN_SPEED_FACTOR; activeDecel=DECEL*LEARN_SPEED_FACTOR; }
  else          { activeMaxSpeed=MAX_SPEED; activeAccel=ACCEL; activeDecel=DECEL; }
  logMsg(LOG_INFO,"MOVE",">> %s%s%s (Speed=%.0f)",
         closing?"SCHLIESSEN":"OEFFNEN",
         positionKnown?"":" (Pos unbekannt)",
         learnMode?" [LERNEN]":"", activeMaxSpeed);
}

void startReverse() {
  digitalWrite(DIR_PIN, closing ? DIR_OPEN : DIR_CLOSE);
  delayMicroseconds(5);
  stepCounter=0; currentSpeed=0.0; state=REVERSING;
  lastSpeedUpdate=micros(); lastStepTime=micros();
  logMsg(LOG_WARN,"SAFE","Einklemmschutz: %ld Schritte zurueck...", REVERSE_STEPS);
}

void emergencyStop(StopReason reason, const char* detail) {
  currentSpeed = 0.0;
  logMsg(LOG_ERROR,"STOP","code=%d %s", reason, detail);
  if (REVERSE_STEPS > 0) {
    startReverse();
  } else {
    state=EMERGENCY; emergencyStart=millis(); emergencyMotorOff=false; emergencyEncoderRef=encoderPosition;
    logMsg(LOG_INFO,"STOP","Neustart in %lu ms.", ERROR_RETRY_DELAY_MS);
  }
}

void enterIdle(bool doorClosed) {
  doorIsClosed=doorClosed; state=IDLE; currentSpeed=0.0;
  digitalWrite(EN_PIN,HIGH); idleStart=millis(); idleRefValid=false;
  logMsg(LOG_INFO,"IDLE","Tuer %s | Motor AUS", doorClosed?"GESCHLOSSEN":"OFFEN");
}

void enterOff() {
  state=OFF_STATE; currentSpeed=0.0; digitalWrite(EN_PIN,HIGH);
  cmdForceClose=false; cmdReleaseError=false; cmdRelearn=false;
  logMsg(LOG_INFO,"MODE","AUS: Motor stromlos.");
}

void armWeightReference() {
  if (scaleReady) { weightReference=scaleWeight; weightReferenceValid=true; }
  else            { weightReferenceValid=false; }
}

// ============================================================
//  LEDs
// ============================================================
void updateLEDs() {
  static unsigned long lastLedUpdate=0;
  unsigned long nowMs=millis();
  if (nowMs-lastLedUpdate<LED_UPDATE_INTERVAL_MS) return;
  lastLedUpdate=nowMs;
  if (motorIsMoving()) {
    unsigned long half=LED_A_BREATHE_PERIOD_MS/2, t=nowMs%LED_A_BREATHE_PERIOD_MS;
    analogWrite(ANALOG_PIN_A, t<half ? map(t,0,half,0,255) : map(t-half,0,half,255,0));
  } else {
    analogWrite(ANALOG_PIN_A, LED_A_IDLE_BRIGHTNESS);
  }
  bool err=(state==EMERGENCY||state==REVERSING||!encoderReady||!scaleReady||otaInProgress);
  analogWrite(ANALOG_PIN_B, err ? ((nowMs%LED_B_BLINK_PERIOD_MS)<(LED_B_BLINK_PERIOD_MS/2)?255:0) : 0);
}

// ============================================================
//  Filter
// ============================================================
long medianOf(long *buf, int n) {
  long tmp[MEDIAN_WINDOW]; memcpy(tmp,buf,n*sizeof(long));
  for(int i=1;i<n;i++){long key=tmp[i];int j=i-1;while(j>=0&&tmp[j]>key){tmp[j+1]=tmp[j];j--;}tmp[j+1]=key;}
  return tmp[n/2];
}

// ============================================================
//  Kalibrierung
// ============================================================
long readRawAverage(int samples) {
  long sum=0; int got=0; unsigned long start=millis();
  while(got<samples&&(millis()-start)<3000){if(scale.available()){sum+=scale.getReading();got++;}delay(2);}
  return got>0?(sum/got):0;
}

void doTare() {
  logMsg(LOG_INFO,"CAL","Tara...");
  calOffset=readRawAverage(128); calOffsetDirty=true;
  logMsg(LOG_INFO,"CAL","Offset=%ld", calOffset);
}

void doGainCalibration() {
  logMsg(LOG_INFO,"CAL","Gain-Kalibrierung...");
  if (CAL_REFERENCE_WEIGHT<=0.0f){logMsg(LOG_WARN,"CAL","Abbruch - Referenzgewicht<=0.");return;}
  long raw=readRawAverage(128);
  float nf=(float)(raw-calOffset)/CAL_REFERENCE_WEIGHT;
  if (fabs(nf)<0.0001f){logMsg(LOG_WARN,"CAL","Faktor unplausibel.");return;}
  calFactor=nf; prefs.putFloat("cal_factor",calFactor);
  logMsg(LOG_INFO,"CAL","Faktor=%.4f gespeichert.", calFactor);
}

void scanI2CBus() {
  logMsg(LOG_INFO,"I2C","Scanne..."); int found=0;
  for(uint8_t a=1;a<127;a++){Wire.beginTransmission(a);if(Wire.endTransmission()==0){
    logMsg(LOG_INFO,"I2C","0x%02X%s",a,(a==0x36)?" (AS5600)":(a==0x2A)?" (NAU7802)":"");found++;}}
  if(!found) logMsg(LOG_WARN,"I2C","KEIN Geraet gefunden!");
}

// ============================================================
//  Scale Task (Core 0)
// ============================================================
void scaleTask(void *param) {
  esp_task_wdt_add(NULL);
  Wire.begin(I2C_SDA,I2C_SCL); Wire.setClock(100000);
  if(scale.begin(Wire)){scale.setSampleRate(NAU7802_SPS_80);scale.calibrateAFE();scaleReady=true;lastScaleReadMs=millis();logMsg(LOG_INFO,"CORE0","NAU7802 OK.");}
  else{scaleReady=false;logMsg(LOG_ERROR,"CORE0","NAU7802 FEHLT!");}
  scanI2CBus();

  encoderReady=false;
  for(int a=1;a<=AS5600_MAX_INIT_ATTEMPTS&&!encoderReady;a++){
    esp_task_wdt_reset();
    if(as5600.begin()){as5600.enableWatchdog(false);as5600.setPowerMode(AS5600_POWER_MODE_NOM);as5600.setHysteresis(AS5600_HYSTERESIS_OFF);encoderReady=true;lastMagnetMs=millis();logMsg(LOG_INFO,"CORE0","AS5600 OK (Vers.%d).",a);}
    else{logMsg(LOG_WARN,"CORE0","AS5600 nicht gefunden (%d/%d).",a,AS5600_MAX_INIT_ATTEMPTS);delay(250);}
  }
  if(!encoderReady){logMsg(LOG_ERROR,"CORE0","AS5600 endgueltig nicht gefunden -> Neustart.");Serial.flush();delay(200);ESP.restart();}

  long medianBuf[MEDIAN_WINDOW]; int medIdx=0,medFill=0;
  float ema=0.0f; bool emaInit=false;
  int16_t encLastRaw=0; bool encInit=false;
  unsigned long lastEncRetry=0;
  unsigned long lastScaleRetry=0;

  for(;;){
    esp_task_wdt_reset();
    if(cmdTare)   {doTare();          cmdTare   =false;}
    if(cmdCalGain){doGainCalibration();cmdCalGain=false;}

    // ---- Laufzeit-Ueberwachung: Encoder-Wiederverbindung (bereits vorhanden) ----
    if(!encoderReady&&(millis()-lastEncRetry>=5000)){
      lastEncRetry=millis();
      if(as5600.begin()){as5600.enableWatchdog(false);as5600.setPowerMode(AS5600_POWER_MODE_NOM);as5600.setHysteresis(AS5600_HYSTERESIS_OFF);encoderReady=true;encInit=false;lastMagnetMs=millis();logMsg(LOG_INFO,"CORE0","AS5600 nachtraeglich gefunden.");}
    }

    // ---- NEU: Laufzeit-Ueberwachung: Waagen-Wiederverbindung ----
    if(!scaleReady&&(millis()-lastScaleRetry>=5000)){
      lastScaleRetry=millis();
      if(scale.begin(Wire)){
        scale.setSampleRate(NAU7802_SPS_80); scale.calibrateAFE();
        scaleReady=true; emaInit=false; medFill=0; lastScaleReadMs=millis();
        logMsg(LOG_INFO,"CORE0","NAU7802 nachtraeglich gefunden.");
      }
    }

    if(calOffsetDirty){
      State s=stateForMqtt;
      if(!(s==ACCELERATING||s==CRUISING||s==DECELERATING||s==REVERSING)){prefs.putLong("cal_offset",calOffset);calOffsetDirty=false;}
    }

    if(scaleReady&&scale.available()){
      long raw=scale.getReading();
      medianBuf[medIdx]=raw; medIdx=(medIdx+1)%MEDIAN_WINDOW; if(medFill<MEDIAN_WINDOW)medFill++;
      float grams=(float)(medianOf(medianBuf,medFill)-calOffset)/calFactor;
      if(!emaInit){ema=grams;emaInit=true;}else{ema=EMA_ALPHA*grams+(1.0f-EMA_ALPHA)*ema;}
      scaleWeight=ema;
      lastScaleReadMs=millis();
    }
    // NEU: Waage liefert seit SCALE_STALE_TIMEOUT_MS keine neue Messung mehr
    // (z.B. I2C-Fehler, loses Kabel) -> als ausgefallen markieren. Ein evtl.
    // noetiger Nothalt einer laufenden Fahrt erfolgt in safetyCheck() (Core 1).
    if(scaleReady&&(millis()-lastScaleReadMs>SCALE_STALE_TIMEOUT_MS)){
      scaleReady=false;
      logMsg(LOG_ERROR,"CORE0","NAU7802: keine Daten seit %lu ms -> als AUSGEFALLEN markiert.",SCALE_STALE_TIMEOUT_MS);
    }

    if(encoderReady&&as5600.isMagnetDetected()){
      int16_t raw=(int16_t)as5600.getRawAngle();
      if(!encInit){encLastRaw=raw;encInit=true;}
      else{int d=(int)raw-(int)encLastRaw;if(d>2048)d-=4096;if(d<-2048)d+=4096;if(ENCODER_INVERT)d=-d;encoderPosition+=d;encLastRaw=raw;}
      lastMagnetMs=millis();
    }
    // NEU: Magnet seit ENCODER_LOST_TIMEOUT_MS nicht mehr erkannt (Dejustage/Ausfall)
    // -> als ausgefallen markieren. Fehler-LED/MQTT-Fault zeigen das dann an, und
    // der 5s-Reconnect-Versuch oben greift automatisch, sobald der Magnet zurueck ist.
    else if(encoderReady&&(millis()-lastMagnetMs>ENCODER_LOST_TIMEOUT_MS)){
      encoderReady=false; encInit=false;
      logMsg(LOG_ERROR,"CORE0","AS5600: Magnet seit %lu ms nicht erkannt -> als AUSGEFALLEN markiert.",ENCODER_LOST_TIMEOUT_MS);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================
//  Sicherheitsueberwachung
// ============================================================
void safetyCheck() {
  if(!motorIsMoving()) return;

  // NEU: Waage ist waehrend der Fahrt ausgefallen (von scaleTask erkannter Timeout)
  // oder war beim Start bereits nicht bereit -> nicht blind ohne Gewichtsueberwachung
  // weiterfahren, sondern einmalig den Einklemmschutz-Rueckzug ausloesen.
  // Das Latch verhindert ein wiederholtes Retriggern waehrend REVERSING/EMERGENCY
  // (aehnlich wie bei der bestehenden Plausibilitaetspruefung).
  static bool scaleFaultLatched = false;
  if(!scaleReady){
    if(!scaleFaultLatched && state!=REVERSING && state!=EMERGENCY){
      scaleFaultLatched = true;
      emergencyStop(STOP_SCALE_TIMEOUT,"Waage waehrend Fahrt ausgefallen/nicht bereit");
    }
    return;
  }
  scaleFaultLatched = false;

  if(millis()-moveStart<(learnMode?LEARN_ARM_DELAY_MS:SAFETY_ARM_DELAY_MS)) return;
  float w=scaleWeight;
  if(w<WEIGHT_PLAUSIBLE_MIN||w>WEIGHT_PLAUSIBLE_MAX){emergencyStop(STOP_WEIGHT_IMPLAUSIBLE,"Gewicht unplausibel");return;}
  if(state==REVERSING||!weightReferenceValid) return;
  if(!closing){if(learnMode){float d=fabs(w-weightReference);if(d>learnMaxDevRunOpen)learnMaxDevRunOpen=d;}return;}
  float dev=fabs(w-weightReference);
  if(learnMode){if(dev>learnMaxDevRun)learnMaxDevRun=dev;return;}
  if(dev>weightStopDelta){char m[96];snprintf(m,sizeof(m),"Abw. %.0f g (Ref %.0f, Schw. %.0f)",w-weightReference,weightReference,weightStopDelta);emergencyStop(STOP_WEIGHT_DELTA,m);}
}

// ============================================================
//  IDLE-Handler
// ============================================================
void handleIdle() {
  if(!idleRefValid){if(millis()-idleStart>=IDLE_SETTLE_MS){idleRefValid=true;logMsg(LOG_INFO,"IDLE","Bereit. Schw.=%ld%s.",ENCODER_OPEN_THRESHOLD,encoderReady?"":" (Enc FEHLT!)");}return;}
  if(!encoderReady) return;
  long ep=encoderPosition;
  if(doorIsClosed&&ep>ENCODER_OPEN_THRESHOLD){
    logMsg(LOG_INFO,"IDLE","Pos %ld > %ld -> manuell geoeffnet.",ep,ENCODER_OPEN_THRESHOLD);
    doorIsClosed=false;closing=true;state=PAUSING;pauseStart=millis();pauseDuration=AUTOCLOSE_DELAY_MS;
    weightCaptured=false;pauseIsAutocloseWait=true;presenceForceActive=false;presenceEncoderLast=ep;lastEventForMqtt=EVT_MANUAL_OPEN;
  }
}

// ============================================================
//  Betriebsart
// ============================================================
void applyMode(const char* m, bool markDirty) {
  char u[16]; size_t n=0;
  for(;m[n]!='\0'&&n<sizeof(u)-1;n++) u[n]=toupper((unsigned char)m[n]); u[n]='\0';
  if(strcmp(u,"AUS")==0||strcmp(u,"OFF")==0){opMode="Aus";doorEnabled=false;}
  else{opMode="Normal";doorEnabled=true;}
  if(markDirty) modeDirty=true;
  logMsg(LOG_INFO,"MODE","Set_Mode=%s",opMode.c_str());
}

// ============================================================
//  Debug
// ============================================================
void debugOutput() {
  static unsigned long ld=0; if(millis()-ld<DEBUG_INTERVAL_MS) return; ld=millis();
  char buf[192]; int off=snprintf(buf,sizeof(buf),"%-13s|%s|Schr:%6ld|H1:%d H2:%d|",
    stateName(state),closing?"SCHLIESSEN":"OEFFNEN  ",stepCounter,hallActive(HALL1_PIN)?1:0,hallActive(HALL2_PIN)?1:0);
  if(scaleReady) off+=snprintf(buf+off,sizeof(buf)-off,"Live:%7.1f g|Gut:%7.1f g|",scaleWeight,validWeight);
  else           off+=snprintf(buf+off,sizeof(buf)-off,"Live:---      |Gut:%7.1f g|",validWeight);
  if(encoderReady&&encoderPosition!=0) off+=snprintf(buf+off,sizeof(buf)-off,"Enc:%6.2f mm|",((float)encoderPosition)/40.0f);
  else if(encoderReady)                off+=snprintf(buf+off,sizeof(buf)-off,"Enc: 0.00 mm(0)|");
  if(learnMode) snprintf(buf+off,sizeof(buf)-off,"LERNEN %d/%ld max=%.0fg omax=%.0fg",learnCount,LEARN_CYCLES,learnMaxDev,learnMaxDevOpen);
  else          snprintf(buf+off,sizeof(buf)-off,"StopDelta=%.0fg|%s",weightStopDelta,doorEnabled?"NORMAL":"AUS");
  logMsg(LOG_INFO,"DBG","%s",buf);
}

// ============================================================
//  MQTT Hilfsfunktionen
// ============================================================
char mqttTopicBuf[96];
const char* topicSet(const char* n){snprintf(mqttTopicBuf,sizeof(mqttTopicBuf),"%s/set/%s",MQTT_BASE,n);return mqttTopicBuf;}
const char* topicCmd(const char* n){snprintf(mqttTopicBuf,sizeof(mqttTopicBuf),"%s/cmd/%s",MQTT_BASE,n);return mqttTopicBuf;}

void publishParamState(const char* name, const String& val){
  char t[96];snprintf(t,sizeof(t),"%s/%s/state",MQTT_BASE,name);mqttClient.publish(t,val.c_str(),true);
}
void publishAllParamStates(){
  char buf[32];
  for(int i=0;i<FLOAT_PARAM_COUNT;i++){dtostrf(*floatParams[i].var,0,1,buf);publishParamState(floatParams[i].mqttName,String(buf));}
  for(int i=0;i<ULONG_PARAM_COUNT;i++) publishParamState(ulongParams[i].mqttName,String(*ulongParams[i].var));
  for(int i=0;i<LONG_PARAM_COUNT;i++)  publishParamState(longParams[i].mqttName, String(*longParams[i].var));
  publishParamState("Set_Mode",opMode);
}

void mqttCallback(char* topic, byte* payload, unsigned int length){
  char msg[32]; unsigned int n=length<sizeof(msg)-1?length:sizeof(msg)-1; memcpy(msg,payload,n);msg[n]='\0';
  for(int i=0;i<FLOAT_PARAM_COUNT;i++){if(strcmp(topic,topicSet(floatParams[i].mqttName))==0){float v=constrain((float)atof(msg),floatParams[i].minV,floatParams[i].maxV);*floatParams[i].var=v;floatParamDirty[i]=true;publishParamState(floatParams[i].mqttName,String(v,1));return;}}
  for(int i=0;i<ULONG_PARAM_COUNT;i++){if(strcmp(topic,topicSet(ulongParams[i].mqttName))==0){long v=constrain(atol(msg),(long)ulongParams[i].minV,(long)ulongParams[i].maxV);*ulongParams[i].var=(unsigned long)v;ulongParamDirty[i]=true;publishParamState(ulongParams[i].mqttName,String(v));return;}}
  for(int i=0;i<LONG_PARAM_COUNT;i++){ if(strcmp(topic,topicSet(longParams[i].mqttName))==0){long v=constrain(atol(msg),longParams[i].minV,longParams[i].maxV);*longParams[i].var=v;longParamDirty[i]=true;publishParamState(longParams[i].mqttName,String(v));return;}}
  if(strcmp(topic,topicSet("Set_Mode"))==0){applyMode(msg,true);publishParamState("Set_Mode",opMode);return;}

  // Warte-Schalter: Payload "ON" oder "OFF" (HA Switch-Konvention)
  if(strcmp(topic,MQTT_BASE"/set/Warte")==0){
    bool neu = (strcmp(msg,"ON")==0);
    warteAktiv = neu;
    mqttClient.publish(MQTT_BASE"/Warte/state", neu?"ON":"OFF", true);
    logMsg(LOG_INFO,"WARTE","Warte-Schalter: %s%s", neu?"EIN":"AUS",
           (!neu && warteBlockiert)?" -> Schliessen wird jetzt ausgefuehrt":"");
    // warteBlockiert wird in loop() ausgewertet
    return;
  }

  if(strcmp(topic,topicCmd("tare"))==0)          {cmdTare=true;return;}
  if(strcmp(topic,topicCmd("cal_gain"))==0)      {cmdCalGain=true;return;}
  if(strcmp(topic,topicCmd("close"))==0)         {cmdForceClose=true;return;}
  if(strcmp(topic,topicCmd("release_error"))==0) {cmdReleaseError=true;return;}
  if(strcmp(topic,topicCmd("reset"))==0)         {cmdReset=true;return;}
  if(strcmp(topic,topicCmd("relearn"))==0)       {cmdRelearn=true;return;}
}

void publishDiscovery(){
  char topic[128],payload[600];
  const char* obs[]={"Pause_After_Open","Pause_After_Close","Presence_Hold_Time","Trigger_Weight","Presence_Weight_Delta","Direction_Sign"};
  for(int i=0;i<6;i++){snprintf(topic,sizeof(topic),"homeassistant/number/%s/%s/config",MQTT_NODE_ID,obs[i]);mqttClient.publish(topic,"",true);}
  const char* dev="\"device\":{\"identifiers\":[\"" MQTT_NODE_ID "\"],\"name\":\"Barn Door Controller\",\"manufacturer\":\"DIY\",\"model\":\"ESP32 TMC2209\"}";
  const char* avail=MQTT_BASE "/status";

  // IP-Sensor
  snprintf(topic,sizeof(topic),"homeassistant/sensor/%s/ip_address/config",MQTT_NODE_ID);
  snprintf(payload,sizeof(payload),"{\"name\":\"IP-Adresse\",\"uniq_id\":\"%s_ip\",\"stat_t\":\"%s/ip/state\",\"icon\":\"mdi:ip-network\",\"avty_t\":\"%s\",%s}",MQTT_NODE_ID,MQTT_BASE,avail,dev);
  mqttClient.publish(topic,payload,true);

  for(int i=0;i<FLOAT_PARAM_COUNT;i++){
    snprintf(topic,sizeof(topic),"homeassistant/number/%s/%s/config",MQTT_NODE_ID,floatParams[i].mqttName);
    snprintf(payload,sizeof(payload),"{\"name\":\"%s\",\"uniq_id\":\"%s_%s\",\"stat_t\":\"%s/%s/state\",\"cmd_t\":\"%s/set/%s\",\"min\":%.1f,\"max\":%.1f,\"step\":%.1f,\"avty_t\":\"%s\",%s}",
      floatParams[i].mqttName,MQTT_NODE_ID,floatParams[i].mqttName,MQTT_BASE,floatParams[i].mqttName,MQTT_BASE,floatParams[i].mqttName,floatParams[i].minV,floatParams[i].maxV,floatParams[i].stepV,avail,dev);
    mqttClient.publish(topic,payload,true);
  }
  for(int i=0;i<ULONG_PARAM_COUNT;i++){
    snprintf(topic,sizeof(topic),"homeassistant/number/%s/%s/config",MQTT_NODE_ID,ulongParams[i].mqttName);
    snprintf(payload,sizeof(payload),"{\"name\":\"%s\",\"uniq_id\":\"%s_%s\",\"stat_t\":\"%s/%s/state\",\"cmd_t\":\"%s/set/%s\",\"min\":%lu,\"max\":%lu,\"step\":%lu,\"unit_of_meas\":\"ms\",\"avty_t\":\"%s\",%s}",
      ulongParams[i].mqttName,MQTT_NODE_ID,ulongParams[i].mqttName,MQTT_BASE,ulongParams[i].mqttName,MQTT_BASE,ulongParams[i].mqttName,ulongParams[i].minV,ulongParams[i].maxV,ulongParams[i].stepV,avail,dev);
    mqttClient.publish(topic,payload,true);
  }
  for(int i=0;i<LONG_PARAM_COUNT;i++){
    snprintf(topic,sizeof(topic),"homeassistant/number/%s/%s/config",MQTT_NODE_ID,longParams[i].mqttName);
    snprintf(payload,sizeof(payload),"{\"name\":\"%s\",\"uniq_id\":\"%s_%s\",\"stat_t\":\"%s/%s/state\",\"cmd_t\":\"%s/set/%s\",\"min\":%ld,\"max\":%ld,\"step\":%ld,\"avty_t\":\"%s\",%s}",
      longParams[i].mqttName,MQTT_NODE_ID,longParams[i].mqttName,MQTT_BASE,longParams[i].mqttName,MQTT_BASE,longParams[i].mqttName,longParams[i].minV,longParams[i].maxV,longParams[i].stepV,avail,dev);
    mqttClient.publish(topic,payload,true);
  }

  snprintf(topic,sizeof(topic),"homeassistant/select/%s/Set_Mode/config",MQTT_NODE_ID);
  snprintf(payload,sizeof(payload),"{\"name\":\"Set_Mode\",\"uniq_id\":\"%s_set_mode\",\"stat_t\":\"%s/Set_Mode/state\",\"cmd_t\":\"%s/set/Set_Mode\",\"options\":[\"Normal\",\"Aus\"],\"avty_t\":\"%s\",%s}",MQTT_NODE_ID,MQTT_BASE,MQTT_BASE,avail,dev);
  mqttClient.publish(topic,payload,true);

  snprintf(topic,sizeof(topic),"homeassistant/sensor/%s/state/config",MQTT_NODE_ID);
  snprintf(payload,sizeof(payload),"{\"name\":\"Status\",\"uniq_id\":\"%s_status\",\"stat_t\":\"%s/state\",\"icon\":\"mdi:door\",\"avty_t\":\"%s\",%s}",MQTT_NODE_ID,MQTT_BASE,avail,dev);
  mqttClient.publish(topic,payload,true);

  snprintf(topic,sizeof(topic),"homeassistant/binary_sensor/%s/learn_active/config",MQTT_NODE_ID);
  snprintf(payload,sizeof(payload),"{\"name\":\"Lernphase aktiv\",\"uniq_id\":\"%s_learn_active\",\"stat_t\":\"%s/learn_active/state\",\"payload_on\":\"EIN\",\"payload_off\":\"AUS\",\"icon\":\"mdi:school\",\"avty_t\":\"%s\",%s}",MQTT_NODE_ID,MQTT_BASE,avail,dev);
  mqttClient.publish(topic,payload,true);

  snprintf(topic,sizeof(topic),"homeassistant/sensor/%s/learn_progress/config",MQTT_NODE_ID);
  snprintf(payload,sizeof(payload),"{\"name\":\"Lernfortschritt\",\"uniq_id\":\"%s_learn_progress\",\"stat_t\":\"%s/learn_progress/state\",\"icon\":\"mdi:progress-clock\",\"avty_t\":\"%s\",%s}",MQTT_NODE_ID,MQTT_BASE,avail,dev);
  mqttClient.publish(topic,payload,true);

  snprintf(topic,sizeof(topic),"homeassistant/sensor/%s/encoder_pos/config",MQTT_NODE_ID);
  snprintf(payload,sizeof(payload),"{\"name\":\"Encoder-Position\",\"uniq_id\":\"%s_encoder_pos\",\"stat_t\":\"%s/encoder_pos/state\",\"icon\":\"mdi:rotate-3d-variant\",\"avty_t\":\"%s\",%s}",MQTT_NODE_ID,MQTT_BASE,avail,dev);
  mqttClient.publish(topic,payload,true);

  snprintf(topic,sizeof(topic),"homeassistant/sensor/%s/last_event/config",MQTT_NODE_ID);
  snprintf(payload,sizeof(payload),"{\"name\":\"Letztes Ereignis\",\"uniq_id\":\"%s_last_event\",\"stat_t\":\"%s/last_event/state\",\"icon\":\"mdi:hand-back-right\",\"avty_t\":\"%s\",%s}",MQTT_NODE_ID,MQTT_BASE,avail,dev);
  mqttClient.publish(topic,payload,true);

  snprintf(topic,sizeof(topic),"homeassistant/binary_sensor/%s/fault/config",MQTT_NODE_ID);
  snprintf(payload,sizeof(payload),"{\"name\":\"Stoerung\",\"uniq_id\":\"%s_fault\",\"stat_t\":\"%s/fault/state\",\"dev_cla\":\"problem\",\"payload_on\":\"EIN\",\"payload_off\":\"AUS\",\"avty_t\":\"%s\",%s}",MQTT_NODE_ID,MQTT_BASE,avail,dev);
  mqttClient.publish(topic,payload,true);

  snprintf(topic,sizeof(topic),"homeassistant/binary_sensor/%s/ota_active/config",MQTT_NODE_ID);
  snprintf(payload,sizeof(payload),"{\"name\":\"OTA Update laeuft\",\"uniq_id\":\"%s_ota_active\",\"stat_t\":\"%s/ota_active/state\",\"dev_cla\":\"running\",\"payload_on\":\"EIN\",\"payload_off\":\"AUS\",\"icon\":\"mdi:update\",\"avty_t\":\"%s\",%s}",MQTT_NODE_ID,MQTT_BASE,avail,dev);
  mqttClient.publish(topic,payload,true);

  const char* btns[6][2]={{"reset","Reset"},{"tare","Tara"},{"cal_gain","Waage kalibrieren"},{"close","Tuer schliessen"},{"release_error","Fehler quittieren"},{"relearn","Neu lernen"}};
  for(int i=0;i<6;i++){
    snprintf(topic,sizeof(topic),"homeassistant/button/%s/%s/config",MQTT_NODE_ID,btns[i][0]);
    snprintf(payload,sizeof(payload),"{\"name\":\"%s\",\"uniq_id\":\"%s_%s\",\"cmd_t\":\"%s/cmd/%s\",\"payload_press\":\"PRESS\",\"avty_t\":\"%s\",%s}",btns[i][1],MQTT_NODE_ID,btns[i][0],MQTT_BASE,btns[i][0],avail,dev);
    mqttClient.publish(topic,payload,true);
  }

  // Warte-Schalter (Switch-Entity)
  // Wenn EIN: Autoclose-Timeout blockiert das Schliessen bis Schalter AUS.
  snprintf(topic,sizeof(topic),"homeassistant/switch/%s/Warte/config",MQTT_NODE_ID);
  snprintf(payload,sizeof(payload),
    "{\"name\":\"Warte\","
    "\"uniq_id\":\"%s_warte\","
    "\"stat_t\":\"%s/Warte/state\","
    "\"cmd_t\":\"%s/set/Warte\","
    "\"payload_on\":\"ON\","
    "\"payload_off\":\"OFF\","
    "\"icon\":\"mdi:timer-pause\","
    "\"avty_t\":\"%s\",%s}",
    MQTT_NODE_ID,MQTT_BASE,MQTT_BASE,avail,dev);
  mqttClient.publish(topic,payload,true);
}

void mqttSubscribeAll(){
  for(int i=0;i<FLOAT_PARAM_COUNT;i++) mqttClient.subscribe(topicSet(floatParams[i].mqttName));
  for(int i=0;i<ULONG_PARAM_COUNT;i++) mqttClient.subscribe(topicSet(ulongParams[i].mqttName));
  for(int i=0;i<LONG_PARAM_COUNT;i++)  mqttClient.subscribe(topicSet(longParams[i].mqttName));
  mqttClient.subscribe(topicSet("Set_Mode"));
  mqttClient.subscribe(topicCmd("reset"));   mqttClient.subscribe(topicCmd("relearn"));
  mqttClient.subscribe(topicCmd("tare"));    mqttClient.subscribe(topicCmd("cal_gain"));
  mqttClient.subscribe(topicCmd("close"));   mqttClient.subscribe(topicCmd("release_error"));
  mqttClient.subscribe(MQTT_BASE"/set/Warte");
}

void flushDirtyParamsToNVS(){
  for(int i=0;i<FLOAT_PARAM_COUNT;i++) if(floatParamDirty[i]){prefs.putFloat(floatParams[i].prefKey,*floatParams[i].var);floatParamDirty[i]=false;}
  for(int i=0;i<ULONG_PARAM_COUNT;i++) if(ulongParamDirty[i]){prefs.putULong(ulongParams[i].prefKey,*ulongParams[i].var);ulongParamDirty[i]=false;}
  for(int i=0;i<LONG_PARAM_COUNT;i++)  if(longParamDirty[i]) {prefs.putLong(longParams[i].prefKey,  *longParams[i].var);  longParamDirty[i]=false;}
  if(modeDirty){prefs.putString("op_mode",opMode);modeDirty=false;}
}

// ============================================================
//  Web-UI: gemeinsames Design fuer alle Seiten (Status/WLAN/MQTT/
//  Parameter/Log/Firmware-Login) - Top-Navigation, Karten, mobile-first,
//  ohne externe Abhaengigkeiten (kein Internet noetig).
// ============================================================
String htmlEscape(const String& s){
  String o; o.reserve(s.length());
  for(size_t i=0;i<s.length();i++){
    char c=s[i];
    if(c=='&') o+="&amp;";
    else if(c=='<') o+="&lt;";
    else if(c=='>') o+="&gt;";
    else if(c=='"') o+="&quot;";
    else if(c=='\'') o+="&#39;";
    else o+=c;
  }
  return o;
}

static const char* UI_STYLE =
  "<style>"
  ":root{--bg:#f1f5f9;--card:#fff;--text:#1e293b;--muted:#64748b;--primary:#2563eb;--primary-dark:#1d4ed8;--border:#e2e8f0;--ok:#16a34a;--warn:#d97706;--err:#dc2626}"
  "*{box-sizing:border-box}"
  "body{margin:0;background:var(--bg);color:var(--text);font-family:-apple-system,'Segoe UI',Roboto,sans-serif;font-size:15px}"
  ".topbar{background:#111827;color:#fff;padding:12px 16px;display:flex;align-items:center;gap:8px;flex-wrap:wrap}"
  ".topbar .brand{font-weight:700;font-size:16px;margin-right:auto;display:flex;align-items:center;gap:8px;white-space:nowrap}"
  ".topbar nav{display:flex;gap:2px;flex-wrap:wrap}"
  ".topbar nav a{color:#cbd5e1;text-decoration:none;padding:7px 11px;border-radius:6px;font-size:12.5px;white-space:nowrap}"
  ".topbar nav a:hover{background:#1f2937;color:#fff}"
  ".topbar nav a.active{background:var(--primary);color:#fff}"
  ".wrap{max-width:760px;margin:20px auto;padding:0 16px 40px}"
  ".card{background:var(--card);border:1px solid var(--border);border-radius:10px;padding:20px;margin-bottom:16px;box-shadow:0 1px 2px rgba(0,0,0,.04)}"
  ".card h2{margin:0 0 14px;font-size:16px;display:flex;align-items:center;gap:8px;flex-wrap:wrap}"
  ".subcard{border:1px solid var(--border);border-radius:8px;padding:14px;margin:14px 0;background:#fafbfc}"
  ".subcard h3{margin:0 0 10px;font-size:12px;color:var(--muted);text-transform:uppercase;letter-spacing:.04em}"
  ".row{margin-bottom:14px}"
  "label{display:block;font-size:12.5px;color:var(--muted);margin-bottom:4px}"
  "input[type=text],input[type=password],input[type=number],select{width:100%;padding:9px 10px;border:1px solid var(--border);border-radius:6px;font-size:14px;background:#fff;color:var(--text);font-family:inherit}"
  "input:focus,select:focus{outline:2px solid var(--primary);outline-offset:-1px}"
  ".hint{font-size:11.5px;color:var(--muted);margin-top:3px;font-weight:400}"
  ".btn{background:var(--primary);color:#fff;border:0;padding:10px 18px;border-radius:6px;font-size:14px;cursor:pointer;font-weight:600}"
  ".btn:hover{background:var(--primary-dark)}"
  ".btn.secondary{background:#fff;color:var(--text);border:1px solid var(--border)}"
  ".btn.small{padding:7px 12px;font-size:12.5px}"
  ".pill{display:inline-block;padding:3px 10px;border-radius:999px;font-size:12px;font-weight:600}"
  ".pill.ok{background:#dcfce7;color:var(--ok)}.pill.warn{background:#fef3c7;color:var(--warn)}.pill.err{background:#fee2e2;color:var(--err)}"
  ".statgrid{display:grid;grid-template-columns:1fr 1fr;gap:10px}"
  "@media (max-width:420px){.statgrid{grid-template-columns:1fr}}"
  ".stat{background:#f8fafc;border:1px solid var(--border);border-radius:8px;padding:10px 12px}"
  ".stat .k{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.03em}"
  ".stat .v{font-size:15px;font-weight:600;margin-top:2px}"
  ".v.err-text{color:var(--err)}"
  ".switchrow{display:flex;align-items:center;justify-content:space-between;gap:10px}"
  ".switch{position:relative;display:inline-block;width:42px;height:24px;flex:none}"
  ".switch input{opacity:0;width:0;height:0}"
  ".slider2{position:absolute;cursor:pointer;inset:0;background:#cbd5e1;border-radius:24px;transition:.15s}"
  ".slider2:before{content:'';position:absolute;height:18px;width:18px;left:3px;top:3px;background:#fff;border-radius:50%;transition:.15s}"
  "input:checked+.slider2{background:var(--primary)}"
  "input:checked+.slider2:before{transform:translateX(18px)}"
  ".actionbar{display:flex;gap:8px;flex-wrap:wrap;margin-top:4px}"
  ".msg{padding:10px 14px;border-radius:8px;font-size:13.5px;margin-bottom:14px}"
  ".msg.ok{background:#dcfce7;color:#166534}.msg.info{background:#dbeafe;color:#1e40af}"
  ".loginwrap{max-width:320px;margin:60px auto}"
  ".chk{display:flex;align-items:center;gap:8px;font-size:13px;color:var(--muted);margin-top:2px}"
  "#log{background:#f7f7f7;border:1px solid var(--border);border-radius:8px;padding:12px;text-align:left;max-height:60vh;overflow-y:auto}"
  "#log pre{margin:0 0 4px;white-space:pre-wrap;word-break:break-all;font-family:'Courier New',monospace;font-size:12px;color:#555}"
  "#log .warn{color:#d97706}#log .error{color:#dc2626;font-weight:bold}#log .info{color:#555}"
  "</style>";

// Kopf mit Topbar/Navigation, oeffnet <div class='wrap'>. activePage waehlt die
// hervorgehobene Nav-Kachel ("status","wifi","mqtt","params","login","log").
String pageHeader(const char* title, const char* activePage){
  String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Barn Door Controller</title>";
  h += String(UI_STYLE);
  h += "</head><body>";
  h += "<div class='topbar'><div class='brand'>Barn Door Controller</div><nav>";
  struct NavItem{ const char* key; const char* href; const char* label; };
  static const NavItem items[] = {
    {"status","/","Status"},
    {"wifi","/wifi","WLAN"},
    {"mqtt","/mqtt","MQTT"},
    {"params","/params","Parameter"},
    {"login","/login","Firmware"},
    {"log","/log","Log"},
  };
  for(size_t i=0;i<sizeof(items)/sizeof(items[0]);i++){
    h += "<a href='"; h += items[i].href; h += "'";
    if(strcmp(activePage,items[i].key)==0) h += " class='active'";
    h += ">"; h += items[i].label; h += "</a>";
  }
  h += "</nav></div><div class='wrap'>";
  if(title && strlen(title)>0){ h += "<h1 style='font-size:19px;margin:4px 0 16px'>"; h += title; h += "</h1>"; }
  return h;
}

String pageFooter(){
  return "</div></body></html>";
}

// Formularzeilen fuer die Parameter-Seite - nutzen dieselben Tabellen
// (floatParams/ulongParams/longParams) wie der MQTT-Pfad, also identische
// Namen, Grenzwerte und Feldnamen-Praefixe (f_/u_/l_) fuer /params/save.
String paramRowFloat(const FloatParam& p){
  String name = String("f_") + p.mqttName;
  String s = "<div class='row'><label>" + String(p.mqttName) + " <span class='hint'>(" + String(p.minV,1) + " .. " + String(p.maxV,1) + ")</span></label>";
  s += "<input type='number' step='any' min='" + String(p.minV,2) + "' max='" + String(p.maxV,2) + "' name='" + name + "' value='" + String(*p.var,2) + "'></div>";
  return s;
}
String paramRowULong(const ULongParam& p){
  String name = String("u_") + p.mqttName;
  String s = "<div class='row'><label>" + String(p.mqttName) + " <span class='hint'>(" + String(p.minV) + " .. " + String(p.maxV) + " ms)</span></label>";
  s += "<input type='number' step='1' min='" + String(p.minV) + "' max='" + String(p.maxV) + "' name='" + name + "' value='" + String(*p.var) + "'></div>";
  return s;
}
String paramRowLong(const LongParam& p){
  String name = String("l_") + p.mqttName;
  String s = "<div class='row'><label>" + String(p.mqttName) + " <span class='hint'>(" + String(p.minV) + " .. " + String(p.maxV) + ")</span></label>";
  s += "<input type='number' step='1' min='" + String(p.minV) + "' max='" + String(p.maxV) + "' name='" + name + "' value='" + String(*p.var) + "'></div>";
  return s;
}

// ============================================================
//  ArduinoOTA Setup
// ============================================================
void setupOTA(){
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([](){otaInProgress=true;otaActiveForMqtt=true;digitalWrite(EN_PIN,HIGH);mqttClient.publish(MQTT_BASE"/ota_active/state","EIN",true);mqttClient.publish(MQTT_BASE"/state","OTA UPDATE LAEUFT",true);logMsg(LOG_WARN,"OTA","Update gestartet.");});
  ArduinoOTA.onEnd([](){logMsg(LOG_INFO,"OTA","Fertig -> Neustart.");});
  ArduinoOTA.onProgress([](unsigned int p,unsigned int t){static unsigned long lr=0;if(millis()-lr<1000)return;lr=millis();logMsg(LOG_INFO,"OTA","%u%%",(p*100)/t);});
  ArduinoOTA.onError([](ota_error_t e){const char*r="?";switch(e){case OTA_AUTH_ERROR:r="Auth";break;case OTA_BEGIN_ERROR:r="Begin";break;case OTA_CONNECT_ERROR:r="Connect";break;case OTA_RECEIVE_ERROR:r="Receive";break;case OTA_END_ERROR:r="End";break;}logMsg(LOG_ERROR,"OTA","Fehler [%u]: %s",e,r);otaInProgress=false;otaActiveForMqtt=false;mqttClient.publish(MQTT_BASE"/ota_active/state","AUS",true);});
  ArduinoOTA.begin();
  logMsg(LOG_INFO,"OTA","ArduinoOTA bereit (Host:%s).",OTA_HOSTNAME);
}

// ============================================================
//  MQTT Task (Core 0)
// ============================================================
void mqttTask(void *param){
  esp_task_wdt_add(NULL);
  WiFi.mode(WIFI_STA); WiFi.persistent(false); WiFi.setAutoReconnect(true);
  WiFi.begin(wifiSsid,wifiPassword);
  mqttClient.setServer(mqttHost,mqttPort); mqttClient.setBufferSize(3072); mqttClient.setCallback(mqttCallback);

  bool otaStarted=false;
  State lastSentState=(State)255; bool lastSentAC=false;
  LastEvent lastSentEvt=EVT_NONE; bool lastSentLearn=true; int lastSentLC=-1;
  long lastSentEnc=LONG_MIN; bool lastSentFault=false; bool lastSentOta=false;
  String lastSentIp="";
  unsigned long lastEncPub=0,lastWifiAtt=0,lastMqttAtt=0;

  for(;;){
    esp_task_wdt_reset();
    if(WiFi.status()!=WL_CONNECTED){
      if(millis()-lastWifiAtt>5000){lastWifiAtt=millis();logMsg(LOG_WARN,"MQTT","WLAN getrennt, reconnect...");WiFi.disconnect();WiFi.begin(wifiSsid,wifiPassword);}
      vTaskDelay(pdMS_TO_TICKS(200)); continue;
    }

    // OTA + Web-Updater einmalig starten
    if(!otaStarted){
      setupOTA();
      otaStarted=true;

      // ================================================================
      //  Web-UI "Barn Door Controller" - kartenbasiertes Design mit
      //  Top-Navigation (Status / WLAN / MQTT / Parameter / Firmware / Log),
      //  angelehnt an moderne ESP32-Konfigurationsoberflaechen.
      //  Status:      http://<IP>/
      //  WLAN:        http://<IP>/wifi
      //  MQTT:        http://<IP>/mqtt
      //  Parameter:   http://<IP>/params   (dieselben Werte wie ueber MQTT)
      //  Firmware:    http://<IP>/login    (admin / admin, wie bisher rein clientseitig) -> /serverIndex
      //  Upload-Route: POST /update
      //  Log:         http://<IP>/log
      // ================================================================

      // ---- Route: Status/Dashboard ----
      webServer.on("/", HTTP_GET, [](){
        String page = pageHeader("", "status");
        page += "<div class='card'><h2>Status <span id='st-pill' class='pill ok'>...</span></h2>";
        page += "<div class='statgrid'>";
        page += "<div class='stat'><div class='k'>Zustand</div><div class='v' id='st-state'>-</div></div>";
        page += "<div class='stat'><div class='k'>Tuer</div><div class='v' id='st-door'>-</div></div>";
        page += "<div class='stat'><div class='k'>Betriebsart</div><div class='v' id='st-mode'>-</div></div>";
        page += "<div class='stat'><div class='k'>Stoerung</div><div class='v' id='st-fault'>-</div></div>";
        page += "<div class='stat'><div class='k'>Gewicht</div><div class='v' id='st-weight'>-</div></div>";
        page += "<div class='stat'><div class='k'>Encoder</div><div class='v' id='st-enc'>-</div></div>";
        page += "<div class='stat'><div class='k'>IP-Adresse</div><div class='v' id='st-ip'>-</div></div>";
        page += "<div class='stat' id='learn-wrap' style='display:none'><div class='k'>Lernfortschritt</div><div class='v' id='st-learn'>-</div></div>";
        page += "</div></div>";

        page += "<div class='card'><h2>Warte-Schalter</h2>";
        page += "<div class='switchrow'><span class='hint'>Blockiert automatisches Schliessen nach Timeout</span>";
        page += "<label class='switch'><input type='checkbox' id='warteToggle' onchange='setWarte(this.checked)'><span class='slider2'></span></label></div></div>";

        page += "<div class='card'><h2>Aktionen</h2><div class='actionbar'>";
        page += "<button class='btn secondary small' onclick=\"post('/action/tare')\">Tara</button>";
        page += "<button class='btn secondary small' onclick=\"post('/action/calgain')\">Waage kalibrieren</button>";
        page += "<button class='btn secondary small' onclick=\"post('/action/close')\">Tuer schliessen</button>";
        page += "<button class='btn secondary small' onclick=\"post('/action/releaseerror')\">Fehler quittieren</button>";
        page += "<button class='btn secondary small' onclick=\"post('/action/relearn')\">Neu lernen</button>";
        page += "<button class='btn secondary small' onclick=\"if(confirm('Geraet jetzt neu starten?'))post('/action/reset')\">Reset</button>";
        page += "</div></div>";

        page += "<script>"
          "function post(u){fetch(u,{method:'POST'}).then(function(){setTimeout(refresh,300);});}"
          "function setWarte(v){fetch('/action/warte?v='+(v?1:0),{method:'POST'});}"
          "function refresh(){"
            "fetch('/api/status').then(function(r){return r.json();}).then(function(d){"
              "document.getElementById('st-state').textContent=d.state;"
              "var p=document.getElementById('st-pill');"
              "p.className='pill '+(d.fault?'err':(d.moving?'warn':'ok'));"
              "p.textContent=d.fault?'Stoerung':(d.moving?'Aktiv':'OK');"
              "document.getElementById('st-door').textContent=d.doorClosed?'Geschlossen':'Offen';"
              "document.getElementById('st-mode').textContent=d.mode;"
              "var f=document.getElementById('st-fault');f.textContent=d.fault?'JA':'nein';f.className='v'+(d.fault?' err-text':'');"
              "document.getElementById('st-weight').textContent=d.weight.toFixed(1)+' g';"
              "document.getElementById('st-enc').textContent=d.enc.toFixed(2)+' mm';"
              "document.getElementById('st-ip').textContent=d.ip;"
              "document.getElementById('learn-wrap').style.display=d.learn?'block':'none';"
              "if(d.learn)document.getElementById('st-learn').textContent=d.learnCount+' / '+d.learnCycles;"
              "document.getElementById('warteToggle').checked=d.warte;"
            "}).catch(function(){});"
          "}"
          "setInterval(refresh,2000);window.addEventListener('load',refresh);"
          "</script>";
        page += pageFooter();
        webServer.sendHeader("Connection","close");
        webServer.send(200,"text/html",page);
      });

      // ---- Route: JSON-Status fuer die Dashboard-Aktualisierung (kein Full-Reload) ----
      webServer.on("/api/status", HTTP_GET, [](){
        State s = stateForMqtt;
        bool fault  = (s==EMERGENCY||s==REVERSING||!encoderReady||!scaleReady||otaInProgress);
        bool moving = (s==ACCELERATING||s==CRUISING||s==DECELERATING||s==REVERSING);
        char json[480];
        snprintf(json,sizeof(json),
          "{\"state\":\"%s\",\"moving\":%s,\"doorClosed\":%s,\"fault\":%s,"
          "\"learn\":%s,\"learnCount\":%d,\"learnCycles\":%ld,"
          "\"weight\":%.1f,\"enc\":%.2f,\"mode\":\"%s\",\"warte\":%s,\"ip\":\"%s\"}",
          stateName(s), moving?"true":"false", doorIsClosed?"true":"false", fault?"true":"false",
          learnModeForMqtt?"true":"false", learnCountForMqtt, LEARN_CYCLES,
          scaleWeight, ((float)encoderPosition)/40.0f, opMode.c_str(), warteAktiv?"true":"false",
          WiFi.localIP().toString().c_str());
        webServer.sendHeader("Connection","close");
        webServer.send(200,"application/json",json);
      });

      // ---- Routen: Aktions-Buttons (setzen dieselben volatile cmd-Flags wie MQTT) ----
      webServer.on("/action/tare", HTTP_POST, [](){cmdTare=true; webServer.sendHeader("Connection","close"); webServer.send(200,"text/plain","OK");});
      webServer.on("/action/calgain", HTTP_POST, [](){cmdCalGain=true; webServer.sendHeader("Connection","close"); webServer.send(200,"text/plain","OK");});
      webServer.on("/action/close", HTTP_POST, [](){cmdForceClose=true; webServer.sendHeader("Connection","close"); webServer.send(200,"text/plain","OK");});
      webServer.on("/action/releaseerror", HTTP_POST, [](){cmdReleaseError=true; webServer.sendHeader("Connection","close"); webServer.send(200,"text/plain","OK");});
      webServer.on("/action/relearn", HTTP_POST, [](){cmdRelearn=true; webServer.sendHeader("Connection","close"); webServer.send(200,"text/plain","OK");});
      webServer.on("/action/reset", HTTP_POST, [](){cmdReset=true; webServer.sendHeader("Connection","close"); webServer.send(200,"text/plain","OK");});
      webServer.on("/action/warte", HTTP_POST, [](){
        bool v = webServer.arg("v")=="1";
        warteAktiv=v;
        logMsg(LOG_INFO,"WARTE","Warte-Schalter (Web): %s", v?"EIN":"AUS");
        if(mqttClient.connected()) mqttClient.publish(MQTT_BASE"/Warte/state", v?"ON":"OFF", true);
        webServer.sendHeader("Connection","close");
        webServer.send(200,"text/plain","OK");
      });

      // ---- Route: WLAN-Konfiguration ----
      webServer.on("/wifi", HTTP_GET, [](){
        String page = pageHeader("WLAN-Einstellungen", "wifi");
        if(webServer.hasArg("saved")) page += "<div class='msg ok'>Gespeichert.</div>";
        page += "<div class='card'><h2>WLAN</h2><form method='POST' action='/wifi/save'>";
        page += "<div class='row'><label>SSID</label><input type='text' name='ssid' value='" + htmlEscape(String(wifiSsid)) + "' maxlength='32' required></div>";
        page += "<div class='row'><label>Passwort <span class='hint'>(leer lassen = unveraendert)</span></label><input type='password' name='pass' maxlength='64' placeholder='unveraendert'></div>";
        page += "<label class='chk'><input type='checkbox' name='pass_clear' value='1'> Passwort entfernen (offenes WLAN)</label>";
        page += "<div class='actionbar' style='margin-top:14px'><button class='btn' type='submit'>Speichern &amp; Neustart</button></div>";
        page += "</form></div>";
        page += pageFooter();
        webServer.sendHeader("Connection","close");
        webServer.send(200,"text/html",page);
      });

      webServer.on("/wifi/save", HTTP_POST, [](){
        String ssid = webServer.arg("ssid");
        String pass = webServer.arg("pass");
        bool clearPass = webServer.arg("pass_clear")=="1";
        bool changed=false;
        if(ssid.length()>0 && ssid.length()<sizeof(wifiSsid) && ssid!=String(wifiSsid)){
          strncpy(wifiSsid, ssid.c_str(), sizeof(wifiSsid)-1); wifiSsid[sizeof(wifiSsid)-1]='\0';
          prefs.putString("w_ssid", wifiSsid); changed=true;
        }
        if(clearPass){
          wifiPassword[0]='\0'; prefs.putString("w_pass", wifiPassword); changed=true;
        } else if(pass.length()>0 && pass.length()<sizeof(wifiPassword)){
          strncpy(wifiPassword, pass.c_str(), sizeof(wifiPassword)-1); wifiPassword[sizeof(wifiPassword)-1]='\0';
          prefs.putString("w_pass", wifiPassword); changed=true;
        }
        String page = pageHeader("WLAN-Einstellungen", "wifi");
        if(changed){
          logMsg(LOG_INFO,"WEB","WLAN gespeichert (SSID=%s) - Neustart.", wifiSsid);
          page += "<div class='card'><h2>Gespeichert</h2><p>Die WLAN-Daten wurden gespeichert. Das Geraet startet jetzt neu und verbindet sich mit dem neuen Netzwerk.</p></div>";
        } else {
          page += "<div class='card'><h2>Keine Aenderung</h2><p>Es wurde nichts geaendert.</p><p><a class='btn secondary' href='/wifi'>Zurueck</a></p></div>";
        }
        page += pageFooter();
        webServer.sendHeader("Connection","close");
        webServer.send(200,"text/html",page);
        if(changed){ delay(400); ESP.restart(); }
      });

      // ---- Route: MQTT-Konfiguration ----
      webServer.on("/mqtt", HTTP_GET, [](){
        String page = pageHeader("MQTT-Einstellungen", "mqtt");
        if(webServer.hasArg("saved")) page += "<div class='msg ok'>Gespeichert.</div>";
        page += "<div class='card'><h2>MQTT-Broker</h2><form method='POST' action='/mqtt/save'>";
        page += "<div class='row'><label>Host / IP</label><input type='text' name='host' value='" + htmlEscape(String(mqttHost)) + "' maxlength='64' required></div>";
        page += "<div class='row'><label>Port</label><input type='number' name='port' min='1' max='65535' value='" + String(mqttPort) + "'></div>";
        page += "<div class='row'><label>Benutzername <span class='hint'>(leer = anonym)</span></label><input type='text' name='user' value='" + htmlEscape(String(mqttUser)) + "' maxlength='32'></div>";
        page += "<div class='row'><label>Passwort <span class='hint'>(leer lassen = unveraendert)</span></label><input type='password' name='pass' maxlength='64' placeholder='unveraendert'></div>";
        page += "<label class='chk'><input type='checkbox' name='pass_clear' value='1'> Passwort entfernen</label>";
        page += "<div class='actionbar' style='margin-top:14px'><button class='btn' type='submit'>Speichern &amp; Neustart</button></div>";
        page += "</form></div>";
        page += pageFooter();
        webServer.sendHeader("Connection","close");
        webServer.send(200,"text/html",page);
      });

      webServer.on("/mqtt/save", HTTP_POST, [](){
        String host = webServer.arg("host");
        String user = webServer.arg("user");
        String pass = webServer.arg("pass");
        bool clearPass = webServer.arg("pass_clear")=="1";
        bool changed=false;
        if(host.length()>0 && host.length()<sizeof(mqttHost) && host!=String(mqttHost)){
          strncpy(mqttHost, host.c_str(), sizeof(mqttHost)-1); mqttHost[sizeof(mqttHost)-1]='\0';
          prefs.putString("m_host", mqttHost); changed=true;
        }
        if(webServer.hasArg("port")){
          long p = webServer.arg("port").toInt();
          if(p>=1 && p<=65535 && (uint16_t)p!=mqttPort){ mqttPort=(uint16_t)p; prefs.putUShort("m_port", mqttPort); changed=true; }
        }
        if(user.length()<sizeof(mqttUser) && user!=String(mqttUser)){
          strncpy(mqttUser, user.c_str(), sizeof(mqttUser)-1); mqttUser[sizeof(mqttUser)-1]='\0';
          prefs.putString("m_user", mqttUser); changed=true;
        }
        if(clearPass){
          mqttPass[0]='\0'; prefs.putString("m_pass", mqttPass); changed=true;
        } else if(pass.length()>0 && pass.length()<sizeof(mqttPass)){
          strncpy(mqttPass, pass.c_str(), sizeof(mqttPass)-1); mqttPass[sizeof(mqttPass)-1]='\0';
          prefs.putString("m_pass", mqttPass); changed=true;
        }
        String page = pageHeader("MQTT-Einstellungen", "mqtt");
        if(changed){
          logMsg(LOG_INFO,"WEB","MQTT gespeichert (Host=%s:%u) - Neustart.", mqttHost, mqttPort);
          page += "<div class='card'><h2>Gespeichert</h2><p>Die MQTT-Daten wurden gespeichert. Das Geraet startet jetzt neu.</p></div>";
        } else {
          page += "<div class='card'><h2>Keine Aenderung</h2><p>Es wurde nichts geaendert.</p><p><a class='btn secondary' href='/mqtt'>Zurueck</a></p></div>";
        }
        page += pageFooter();
        webServer.sendHeader("Connection","close");
        webServer.send(200,"text/html",page);
        if(changed){ delay(400); ESP.restart(); }
      });

      // ---- Route: Tuer-Parameter (dieselben Werte/Grenzen wie ueber MQTT, siehe floatParams/ulongParams/longParams) ----
      webServer.on("/params", HTTP_GET, [](){
        String page = pageHeader("Tuer-Parameter", "params");
        if(webServer.hasArg("saved")) page += "<div class='msg ok'>Gespeichert.</div>";
        page += "<div class='card'><h2>Parameter</h2><form method='POST' action='/params/save'>";
        page += "<div class='row'><label>Betriebsart</label><select name='mode'>";
        page += String("<option value='Normal'") + (opMode=="Normal"?" selected":"") + ">Normal</option>";
        page += String("<option value='Aus'") + (opMode=="Aus"?" selected":"") + ">Aus</option>";
        page += "</select></div>";
        page += "<div class='subcard'><h3>Geschwindigkeit &amp; Gewicht</h3>";
        for(int i=0;i<FLOAT_PARAM_COUNT;i++) page += paramRowFloat(floatParams[i]);
        page += "</div>";
        page += "<div class='subcard'><h3>Zeiten (ms)</h3>";
        for(int i=0;i<ULONG_PARAM_COUNT;i++) page += paramRowULong(ulongParams[i]);
        page += "</div>";
        page += "<div class='subcard'><h3>Schritte &amp; Encoder</h3>";
        for(int i=0;i<LONG_PARAM_COUNT;i++) page += paramRowLong(longParams[i]);
        page += "</div>";
        page += "<div class='actionbar'><button class='btn' type='submit'>Speichern</button></div>";
        page += "</form></div>";
        page += pageFooter();
        webServer.sendHeader("Connection","close");
        webServer.send(200,"text/html",page);
      });

      webServer.on("/params/save", HTTP_POST, [](){
        for(int i=0;i<FLOAT_PARAM_COUNT;i++){
          String key = String("f_")+floatParams[i].mqttName;
          if(webServer.hasArg(key)){
            float v = constrain(webServer.arg(key).toFloat(), floatParams[i].minV, floatParams[i].maxV);
            *floatParams[i].var = v; floatParamDirty[i]=true;
          }
        }
        for(int i=0;i<ULONG_PARAM_COUNT;i++){
          String key = String("u_")+ulongParams[i].mqttName;
          if(webServer.hasArg(key)){
            long v = constrain((long)webServer.arg(key).toInt(), (long)ulongParams[i].minV, (long)ulongParams[i].maxV);
            *ulongParams[i].var = (unsigned long)v; ulongParamDirty[i]=true;
          }
        }
        for(int i=0;i<LONG_PARAM_COUNT;i++){
          String key = String("l_")+longParams[i].mqttName;
          if(webServer.hasArg(key)){
            long v = constrain((long)webServer.arg(key).toInt(), longParams[i].minV, longParams[i].maxV);
            *longParams[i].var = v; longParamDirty[i]=true;
          }
        }
        if(webServer.hasArg("mode")) applyMode(webServer.arg("mode").c_str(), true);
        cmdPublishParams = true; // MQTT/HA gleich mit den neuen Werten synchronisieren
        logMsg(LOG_INFO,"WEB","Parameter ueber Web gespeichert.");
        webServer.sendHeader("Location","/params?saved=1");
        webServer.sendHeader("Connection","close");
        webServer.send(303);
      });

      // ---- Route: Firmware-Login (weiterhin rein clientseitig, wie im urspruenglichen Original) ----
      webServer.on("/login", HTTP_GET, [](){
        String page = pageHeader("", "login");
        page += "<div class='card loginwrap'><h2>Firmware-Update Login</h2>";
        page += "<form name='loginForm' onsubmit='return false'>";
        page += "<div class='row'><label>Benutzer</label><input type='text' id='userid'></div>";
        page += "<div class='row'><label>Passwort</label><input type='password' id='pwd'></div>";
        page += "<button class='btn' onclick='check()'>Login</button>";
        page += "</form></div>";
        page += "<script>function check(){var u=document.getElementById('userid').value,p=document.getElementById('pwd').value;"
                "if(u=='admin'&&p=='admin'){window.location='/serverIndex';}else{alert('Falscher Benutzername oder Passwort');}}</script>";
        page += pageFooter();
        webServer.sendHeader("Connection","close");
        webServer.send(200,"text/html",page);
      });

      // ---- Route: Update-Seite (Dateiauswahl + Fortschrittsbalken, Funktion unveraendert, nur neu gestylt) ----
      webServer.on("/serverIndex", HTTP_GET, [](){
        String page = pageHeader("Firmware-Update", "login");
        page += "<div class='card'><h2>OTA-Update</h2>";
        page += "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form' onsubmit='startUpload(event)'>";
        page += "<div class='row'><input type='file' name='update' id='file' onchange='sub(this)' style='display:none'>";
        page += "<label id='file-input' for='file' style='display:block;padding:9px 10px;border:1px dashed var(--border);border-radius:6px;cursor:pointer;text-align:center'>Datei waehlen (.bin) ...</label></div>";
        page += "<button class='btn' type='submit'>Update hochladen</button>";
        page += "<div id='prg' class='hint' style='margin-top:10px'></div>";
        page += "<div id='prgbar' style='background:#e2e8f0;border-radius:10px;margin-top:6px'><div id='bar' style='background:var(--primary);width:0%;height:8px;border-radius:10px'></div></div>";
        page += "</form></div>";
        page += "<script>"
          "function sub(obj){var fn=obj.value.split('\\\\');document.getElementById('file-input').textContent=fn[fn.length-1];}"
          "function startUpload(e){e.preventDefault();var file=document.getElementById('file').files[0];"
          "if(!file){alert('Bitte zuerst eine .bin Datei auswaehlen!');return;}"
          "var data=new FormData();data.append('update',file,file.name);"
          "var xhr=new XMLHttpRequest();xhr.open('POST','/update',true);"
          "xhr.upload.addEventListener('progress',function(evt){if(evt.lengthComputable){"
          "var pct=Math.round(evt.loaded/evt.total*100);"
          "document.getElementById('prg').textContent='Fortschritt: '+pct+'%';"
          "document.getElementById('bar').style.width=pct+'%';}});"
          "xhr.onload=function(){if(xhr.status==200){document.getElementById('prg').textContent='Fertig! Neustart...';document.getElementById('bar').style.width='100%';}"
          "else{document.getElementById('prg').textContent='Fehler: '+xhr.responseText;}};"
          "xhr.onerror=function(){document.getElementById('prg').textContent='Upload fehlgeschlagen!';};"
          "xhr.send(data);}"
          "</script>";
        page += pageFooter();
        webServer.sendHeader("Connection","close");
        webServer.send(200,"text/html",page);
      });

      // ---- Route: Upload-Handler (POST /update, unveraendert) ----
      webServer.on("/update", HTTP_POST,
        [](){
          webServer.sendHeader("Connection","close");
          webServer.send(200,"text/plain",Update.hasError()?"FAIL":"OK");
          delay(500); ESP.restart();
        },
        [](){
          HTTPUpload& upload=webServer.upload();
          if(upload.status==UPLOAD_FILE_START){
            logMsg(LOG_INFO,"OTA","Web-Update: %s",upload.filename.c_str());
            if(!Update.begin(UPDATE_SIZE_UNKNOWN)) logMsg(LOG_ERROR,"OTA","begin fehlgeschlagen");
          } else if(upload.status==UPLOAD_FILE_WRITE){
            if(Update.write(upload.buf,upload.currentSize)!=upload.currentSize) logMsg(LOG_ERROR,"OTA","write fehlgeschlagen");
          } else if(upload.status==UPLOAD_FILE_END){
            if(Update.end(true)) logMsg(LOG_INFO,"OTA","fertig: %u Bytes",upload.totalSize);
            else logMsg(LOG_ERROR,"OTA","end fehlgeschlagen");
          }
        }
      );

      // ---- Route: Live-Log (gleiches Kartendesign wie alle anderen Seiten) ----
      webServer.on("/log", HTTP_GET, [](){
        portENTER_CRITICAL(&logMux);
        int count = logRingCount;
        int start = (logRingCount < LOG_RING_SIZE) ? 0 : logRingHead;
        portEXIT_CRITICAL(&logMux);

        String page = pageHeader("Live-Log", "log");
        page += "<div class='card'><h2>Log <span class='hint' style='font-weight:400'>(aktualisiert alle 2s)</span></h2><div id='log'>";
        portENTER_CRITICAL(&logMux);
        for (int i = 0; i < count; i++) {
          int idx = (start + i) % LOG_RING_SIZE;
          String ln = String(logRing[idx]);
          portEXIT_CRITICAL(&logMux);
          String cls = "info";
          if (ln.indexOf("] WARN ") >= 0)  cls = "warn";
          if (ln.indexOf("] ERROR") >= 0)  cls = "error";
          page += "<pre class='" + cls + "'>" + ln + "</pre>";
          portENTER_CRITICAL(&logMux);
        }
        portEXIT_CRITICAL(&logMux);
        page += "</div><p class='hint' style='margin-top:10px'>Letzte " + String(count) + " von max. " + String(LOG_RING_SIZE) + " Zeilen</p></div>";
        page += "<script>setTimeout(function(){location.reload();},2000);</script>";
        page += pageFooter();
        webServer.sendHeader("Connection","close");
        webServer.send(200,"text/html",page);
      });

      webServer.begin();
      webServerReady = true;  // signalisiert webTask dass er loslegen kann
      String ip=WiFi.localIP().toString();
      logMsg(LOG_INFO,"WEB","Barn Door Controller: http://%s/  (Status)  |  http://%s/login (Firmware-Update)",ip.c_str(),ip.c_str());

      // Rollback-Schutz
      const esp_partition_t* running=esp_ota_get_running_partition();
      esp_ota_img_states_t st;
      if(esp_ota_get_state_partition(running,&st)==ESP_OK&&st==ESP_OTA_IMG_PENDING_VERIFY){
        esp_ota_mark_app_valid_cancel_rollback();
        logMsg(LOG_INFO,"OTA","Firmware als gueltig markiert.");
      }
    }

    ArduinoOTA.handle();
    // webServer laeuft in eigenem webTask (siehe unten) - NICHT hier aufrufen

    if(!mqttClient.connected()){
      if(millis()-lastMqttAtt>5000){
        lastMqttAtt=millis();
        logMsg(LOG_INFO,"MQTT","Verbinde %s...",mqttHost);
        String cid=String(MQTT_NODE_ID)+"-"+String((uint32_t)ESP.getEfuseMac(),HEX);
        bool ok;
        if(strlen(mqttUser)>0) ok=mqttClient.connect(cid.c_str(),mqttUser,mqttPass,MQTT_BASE"/status",0,true,"offline");
        else                    ok=mqttClient.connect(cid.c_str(),MQTT_BASE"/status",0,true,"offline");
        if(ok){
          logMsg(LOG_INFO,"MQTT","Verbunden.");
          mqttClient.publish(MQTT_BASE"/status","online",true);
          // Warte-Schalter Zustand wiederherstellen
          mqttClient.publish(MQTT_BASE"/Warte/state", warteAktiv?"ON":"OFF", true);
          // IP sofort publizieren
          String ip=WiFi.localIP().toString();
          mqttClient.publish(MQTT_BASE"/ip/state",ip.c_str(),true);
          logMsg(LOG_INFO,"MQTT","IP=%s -> barn_door/ip/state",ip.c_str());
          lastSentIp=ip;
          mqttSubscribeAll(); publishDiscovery(); publishAllParamStates();
          mqttClient.publish(MQTT_BASE"/ota_active/state",otaInProgress?"EIN":"AUS",true);
          lastSentState=(State)255; lastSentLearn=!learnModeForMqtt; lastSentLC=-1; lastSentEnc=LONG_MIN;
          lastSentFault=!(state==EMERGENCY||state==REVERSING||!encoderReady||!scaleReady||otaInProgress);
          lastSentOta=otaInProgress;
        } else {
          logMsg(LOG_WARN,"MQTT","Fehler rc=%d",mqttClient.state());
        }
      }
      vTaskDelay(pdMS_TO_TICKS(200)); continue;
    }

    mqttClient.loop();
    if(cmdPublishParams){cmdPublishParams=false;publishAllParamStates();}

    // IP bei Aenderung erneut senden
    String curIp=WiFi.localIP().toString();
    if(curIp!=lastSentIp){lastSentIp=curIp;mqttClient.publish(MQTT_BASE"/ip/state",curIp.c_str(),true);logMsg(LOG_INFO,"MQTT","IP geaendert: %s",curIp.c_str());}

    State s=stateForMqtt; bool ac=pauseIsAutocloseWaitForMqtt;
    if(s!=lastSentState||(s==PAUSING&&ac!=lastSentAC)){
      lastSentState=s;lastSentAC=ac;
      const char* txt;
      if(otaInProgress) txt="OTA UPDATE LAEUFT";
      else if(s==PAUSING) txt=ac?"PAUSE (Autoclose laeuft)":"PAUSE (Lernphase)";
      else txt=stateName(s);
      mqttClient.publish(MQTT_BASE"/state",txt,true);
    }

    LastEvent evt=lastEventForMqtt;
    if(evt!=EVT_NONE&&evt!=lastSentEvt){lastSentEvt=evt;mqttClient.publish(MQTT_BASE"/last_event/state",(evt==EVT_MANUAL_OPEN)?"Manuell geoeffnet":"Unbekannt",true);}

    bool la=learnModeForMqtt;
    if(la!=lastSentLearn){lastSentLearn=la;mqttClient.publish(MQTT_BASE"/learn_active/state",la?"EIN":"AUS",true);}
    int lc=learnCountForMqtt;
    if(la&&lc!=lastSentLC){lastSentLC=lc;char b[24];snprintf(b,sizeof(b),"%d/%ld",lc,LEARN_CYCLES);mqttClient.publish(MQTT_BASE"/learn_progress/state",b,true);}

    if(millis()-lastEncPub>=1000){lastEncPub=millis();long en=encoderPosition;if(en!=lastSentEnc){lastSentEnc=en;mqttClient.publish(MQTT_BASE"/encoder_pos/state",String((int)(((float)en)/40.0f)).c_str(),true);}}

    bool fn=(s==EMERGENCY||s==REVERSING||!encoderReady||!scaleReady||otaInProgress);
    if(fn!=lastSentFault){lastSentFault=fn;mqttClient.publish(MQTT_BASE"/fault/state",fn?"EIN":"AUS",true);}
    bool on=otaActiveForMqtt;
    if(on!=lastSentOta){lastSentOta=on;mqttClient.publish(MQTT_BASE"/ota_active/state",on?"EIN":"AUS",true);}

    bool mv=(s==ACCELERATING||s==CRUISING||s==DECELERATING||s==REVERSING);
    if(!mv&&!otaInProgress) flushDirtyParamsToNVS();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ============================================================
//  Web-Task (Core 0) – laeuft getrennt vom MQTT-Task
//  Eigener Task verhindert dass der Upload den Watchdog ausloest.
//  Watchdog wird fuer diesen Task NICHT registriert, da handleClient()
//  beim Upload mehrere Sekunden blockieren kann.
// ============================================================
TaskHandle_t webTaskHandle = NULL;


void webTask(void *param){
  // Warten bis webServer.begin() im mqttTask aufgerufen wurde
  while(!webServerReady) vTaskDelay(pdMS_TO_TICKS(100));

  logMsg(LOG_INFO,"WEB","Web-Task gestartet.");
  for(;;){
    webServer.handleClient();
    // Kurze Pause damit andere Tasks auf Core 0 auch laufen koennen.
    // 2 ms reicht - handleClient() kehrt sofort zurueck wenn nichts anliegt.
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ============================================================
//  Setup
// ============================================================
void setup(){
  Serial.begin(115200); delay(200);
  Serial.println("\n== Tuer-Steuerung TMC2209 + NAU7802 + AS5600 ==");
  Serial.println("[BUILD] v22 - Barn Door Controller: Web-UI mit WLAN-/MQTT-/Parameter-Konfigurationsseiten");

  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL);

  pinMode(EN_PIN,OUTPUT); pinMode(DIR_PIN,OUTPUT); pinMode(STEP_PIN,OUTPUT);
  pinMode(HALL1_PIN, HALL_ACTIVE_LOW?INPUT_PULLUP:INPUT_PULLDOWN);
  pinMode(HALL2_PIN, HALL_ACTIVE_LOW?INPUT_PULLUP:INPUT_PULLDOWN);
  pinMode(ANALOG_PIN_A,OUTPUT); pinMode(ANALOG_PIN_B,OUTPUT);
  digitalWrite(EN_PIN,LOW); digitalWrite(STEP_PIN,LOW);
  analogWrite(ANALOG_PIN_A,0); analogWrite(ANALOG_PIN_B,0);

  prefs.begin("door",false);
  calOffset=prefs.getLong("cal_offset",0); calFactor=prefs.getFloat("cal_factor",1000.0f);
  if(calFactor==0.0f) calFactor=1.0f;
  logMsg(LOG_INFO,"CAL","Offset=%ld Faktor=%.4f",calOffset,calFactor);

  // WLAN/MQTT aus NVS laden (falls über die Konfigurationsseite schon mal gespeichert),
  // sonst bleiben die Werkseinstellungen von oben aktiv.
  prefs.getString("w_ssid", wifiSsid, sizeof(wifiSsid));
  prefs.getString("w_pass", wifiPassword, sizeof(wifiPassword));
  prefs.getString("m_host", mqttHost, sizeof(mqttHost));
  mqttPort = prefs.getUShort("m_port", mqttPort);
  prefs.getString("m_user", mqttUser, sizeof(mqttUser));
  prefs.getString("m_pass", mqttPass, sizeof(mqttPass));
  logMsg(LOG_INFO,"CFG","WLAN=%s  MQTT=%s:%u",wifiSsid,mqttHost,mqttPort);
  for(int i=0;i<FLOAT_PARAM_COUNT;i++) *floatParams[i].var=prefs.getFloat(floatParams[i].prefKey,*floatParams[i].var);
  for(int i=0;i<ULONG_PARAM_COUNT;i++) *ulongParams[i].var=prefs.getULong(ulongParams[i].prefKey,*ulongParams[i].var);
  for(int i=0;i<LONG_PARAM_COUNT;i++)  *longParams[i].var =prefs.getLong(longParams[i].prefKey,  *longParams[i].var);
  opMode=prefs.getString("op_mode","Normal");
  if(opMode!="Normal"&&opMode!="Aus") opMode="Normal";
  doorEnabled=doorEnabledLast=(opMode=="Normal");
  bool hsd=prefs.isKey("stop_delta"); learnMode=!hsd;
  if(learnMode) logMsg(LOG_INFO,"LERN","Keine StopDelta -> Lernphase (%ld Zyklen)",LEARN_CYCLES);
  else          logMsg(LOG_INFO,"LERN","StopDelta=%.0f g",weightStopDelta);
  logMsg(LOG_INFO,"MODE","Set_Mode=%s",opMode.c_str());

  xTaskCreatePinnedToCore(scaleTask,"scaleTask",4096,NULL,1,&scaleTaskHandle,0);
  xTaskCreatePinnedToCore(mqttTask, "mqttTask", 6144,NULL,1,&mqttTaskHandle, 0);
  // Web-Task: hohe Prioritaet, groesserer Stack wegen String-Operationen in /log
  xTaskCreatePinnedToCore(webTask,  "webTask",  8192,NULL,2,&webTaskHandle,  0);
  cmdTare=true;
  if(doorEnabled){closing=true;armWeightReference();startMovement();}
  else{enterOff();}
}

// ============================================================
//  Loop (Core 1)
// ============================================================
void loop(){
  esp_task_wdt_reset();

  if(doorEnabled!=doorEnabledLast){
    doorEnabledLast=doorEnabled;
    if(doorEnabled){logMsg(LOG_INFO,"MODE","Normal -> fahre zu.");positionKnown=false;closing=true;armWeightReference();startMovement();}
    else{logMsg(LOG_INFO,"MODE","AUS -> Motor stromlos.");enterOff();}
  }

  if(!doorEnabled){stateForMqtt=OFF_STATE;updateLEDs();if(cmdReset){cmdReset=false;Serial.flush();delay(50);ESP.restart();}return;}

  updateSpeed(); generateStep();

  bool cautious=learnMode||!positionKnown;
  if((state==ACCELERATING||state==CRUISING)&&stepCounter>=(cautious?LEARN_BLANK_STEPS:BLANK_STEPS)&&hallActive(targetHall())){
    if(cautious){logMsg(LOG_INFO,"MOVE","Hall %d -> Sofortstopp",closing?1:2);currentSpeed=0.0;state=STOPPED;}
    else        {logMsg(LOG_INFO,"MOVE","Hall %d -> Abbremsen",closing?1:2);state=DECELERATING;}
  }

  safetyCheck();
  if(DEBUG_ENABLED) debugOutput();
  updateLEDs();
  stateForMqtt=state; learnModeForMqtt=learnMode; learnCountForMqtt=learnCount; pauseIsAutocloseWaitForMqtt=pauseIsAutocloseWait;

  if(cmdReset){cmdReset=false;Serial.flush();delay(50);ESP.restart();}

  if(cmdReleaseError){
    cmdReleaseError=false;
    if(state==EMERGENCY){
      if(hallActive(targetHall())){currentSpeed=0.0;state=STOPPED;if(closing)syncEncoderToClosed();}
      else if(closing&&!closeAllowed()){currentSpeed=0.0;state=STOPPED;}
      else{armWeightReference();startMovement();}
    }
  }

  if(cmdForceClose){
    cmdForceClose=false;
    if(!closeAllowed()){logMsg(LOG_INFO,"MQTT","Schliessen ignoriert (schon zu, Enc=%ld).",encoderPosition);syncEncoderToClosed();if(state!=IDLE)enterIdle(true);}
    else if(state==IDLE||state==PAUSING||state==STOPPED){logMsg(LOG_INFO,"MQTT","Schliessen.");digitalWrite(EN_PIN,LOW);closing=true;armWeightReference();startMovement();}
    else{logMsg(LOG_INFO,"MQTT","Schliessen ignoriert (Motor aktiv).");}
  }

  if(cmdRelearn){
    cmdRelearn=false;
    logMsg(LOG_INFO,"MQTT","Neu lernen.");prefs.remove("stop_delta");
    learnMode=true;learnCount=0;learnMaxDev=learnMaxDevOpen=learnMaxDevRun=learnMaxDevRunOpen=0.0f;
    if(state==IDLE||state==PAUSING||state==STOPPED){
      if(closeAllowed()){closing=true;}else{syncEncoderToClosed();closing=false;}
      armWeightReference();startMovement();
    }
  }

  if(state==REVERSING){
    if(stepCounter>=REVERSE_STEPS){currentSpeed=0.0;state=EMERGENCY;emergencyStart=millis();emergencyMotorOff=false;emergencyEncoderRef=encoderPosition;logMsg(LOG_WARN,"SAFE","Rueckzug fertig. Retry in %lu ms.",ERROR_RETRY_DELAY_MS);}
    return;
  }

  if(state==EMERGENCY){
    unsigned long el=millis()-emergencyStart;
    if(!emergencyMotorOff&&el>=EMERGENCY_MOTOR_OFF_DELAY_MS){digitalWrite(EN_PIN,HIGH);emergencyMotorOff=true;logMsg(LOG_INFO,"SAFE","Motor stromlos nach %lu ms.",EMERGENCY_MOTOR_OFF_DELAY_MS);}
    if(encoderReady){long d=encoderPosition-emergencyEncoderRef;if(d>ENCODER_OPEN_THRESHOLD){logMsg(LOG_INFO,"SAFE","Oeffnen erkannt (+%ld) -> Autoclose.",d);if(!emergencyMotorOff){digitalWrite(EN_PIN,HIGH);emergencyMotorOff=true;}doorIsClosed=false;closing=true;state=PAUSING;pauseStart=millis();pauseDuration=AUTOCLOSE_DELAY_MS;weightCaptured=false;pauseIsAutocloseWait=true;presenceForceActive=false;presenceEncoderLast=encoderPosition;lastEventForMqtt=EVT_MANUAL_OPEN;return;}}
    if(el>=ERROR_RETRY_DELAY_MS){
      if(hallActive(targetHall())){currentSpeed=0.0;state=STOPPED;if(closing)syncEncoderToClosed();}
      else if(closing&&!closeAllowed()){currentSpeed=0.0;state=STOPPED;}
      else{armWeightReference();startMovement();}
    }
    return;
  }

  if(state==IDLE){handleIdle();return;}

  if(state==STOPPED){
    if(closing){
      logMsg(LOG_INFO,"MOVE","ZU | Schritte=%ld",stepCounter);
      cmdTare=true;positionKnown=true;encoderPosition=0;logMsg(LOG_INFO,"ENC","Encoder=0 (ZU-Ref).");
      if(learnMode&&weightReferenceValid){
        if(learnMaxDevRun>learnMaxDev) learnMaxDev=learnMaxDevRun;
        learnCount++;
        logMsg(LOG_INFO,"LERN","Schliessf. %d/%ld: max=%.1f g (ges=%.1f g)",learnCount,LEARN_CYCLES,learnMaxDevRun,learnMaxDev);
        if(learnCount>=LEARN_CYCLES){
          weightStopDelta=learnMaxDev*LEARN_SAFETY_MARGIN;learnMode=false;prefs.putFloat("stop_delta",weightStopDelta);
          logMsg(LOG_INFO,"LERN","== LERNPHASE FERTIG == StopDelta=%.0f g",weightStopDelta);
          logMsg(LOG_INFO,"LERN","Open-Diagnose max=%.0f g",learnMaxDevOpen);
          cmdPublishParams=true;
        }
        // FIX (Lernzyklen-Anzeige "3/4" statt "4/4"):
        // stateForMqtt/learnModeForMqtt/learnCountForMqtt wurden weiter oben in dieser
        // Iteration BEREITS VOR diesem Block gelesen, also noch mit dem alten Stand
        // (learnCount vor dem Hochzaehlen, learnMode noch true). Ohne dieses Nachziehen
        // wuerde der finale Zyklus (z.B. "4/4") nie an MQTT/HA publiziert, weil im
        // naechsten loop()-Durchlauf learnModeForMqtt bereits false ist und die
        // Publish-Bedingung "if(la && lc!=lastSentLC)" dann nie mehr greift.
        learnModeForMqtt = learnMode;
        learnCountForMqtt = learnCount;
      }
      if(learnMode){closing=false;state=PAUSING;pauseStart=millis();pauseDuration=LEARN_PAUSE_MS;weightCaptured=false;pauseIsAutocloseWait=false;logMsg(LOG_INFO,"LERN","Pause %lu ms...",pauseDuration);}
      else{enterIdle(true);}
    } else {
      logMsg(LOG_INFO,"MOVE","AUF (Lernph.) | Schritte=%ld",stepCounter); cmdTare=true;
      if(learnMode){if(weightReferenceValid&&learnMaxDevRunOpen>learnMaxDevOpen)learnMaxDevOpen=learnMaxDevRunOpen;logMsg(LOG_INFO,"LERN","Open-Diag max=%.1f g",learnMaxDevRunOpen);closing=true;state=PAUSING;pauseStart=millis();pauseDuration=LEARN_PAUSE_MS;weightCaptured=false;pauseIsAutocloseWait=false;logMsg(LOG_INFO,"LERN","Pause %lu ms...",pauseDuration);}
      else{logMsg(LOG_WARN,"MOVE","Motor-Oeffnen ausserhalb Lernph. -> Idle(offen).");enterIdle(false);}
    }
  }

  if(state==PAUSING){
    unsigned long el=millis()-pauseStart;
    if(!weightCaptured&&el>=SETTLE_TIME_MS){
      validWeight=scaleWeight;weightCaptured=true;
      if(validWeight<WEIGHT_PLAUSIBLE_MIN||validWeight>WEIGHT_PLAUSIBLE_MAX) logMsg(LOG_WARN,"MESS","UNPLAUSIBEL: %.1f g",validWeight);
      else logMsg(LOG_INFO,"MESS","Gewicht: %.1f g",validWeight);
      weightReference=validWeight;weightReferenceValid=true;
    }
    if(pauseIsAutocloseWait&&encoderReady){
      long d=encoderPosition-presenceEncoderLast;
      if(labs(d)>PRESENCE_ENCODER_DELTA){
        if(d>0){if(!presenceForceActive){presenceForceActive=true;logMsg(LOG_INFO,"AC","Bewegung +%ld Ticks -> Timer=%lu ms.",d,AUTOCLOSE_DELAY_MS);}pauseStart=millis();el=0;}
        else presenceForceActive=false;
        presenceEncoderLast=encoderPosition;
      }
    }
    if(el>=pauseDuration){
      if(closing&&!closeAllowed()){
        logMsg(LOG_INFO,"SAFE","Autoclose uebersprungen - schon zu.");
        syncEncoderToClosed(); enterIdle(true); warteBlockiert=false;
      }
      else if(pauseIsAutocloseWait && warteAktiv){
        // Timeout abgelaufen, aber Warte-Schalter ist EIN -> blockieren
        if(!warteBlockiert){
          warteBlockiert=true;
          logMsg(LOG_INFO,"WARTE","Autoclose-Timeout abgelaufen, Schliessen BLOCKIERT (Warte=EIN).");
        }
        // pauseStart laufend erneuern damit el nicht ueberlaeuft
        pauseStart=millis();
      }
      else{
        // Normal schliessen (Warte AUS oder kein Autoclose-Wait)
        if(warteBlockiert){
          logMsg(LOG_INFO,"WARTE","Warte aufgehoben -> Schliessen wird ausgefuehrt.");
          warteBlockiert=false;
        }
        startMovement();
      }
    }

    // Warte wurde deaktiviert waehrend Blockade -> sofort schliessen
    if(warteBlockiert && !warteAktiv){
      warteBlockiert=false;
      logMsg(LOG_INFO,"WARTE","Warte=AUS -> Schliessen jetzt.");
      if(closing&&!closeAllowed()){syncEncoderToClosed();enterIdle(true);}
      else startMovement();
    }
  }
}
