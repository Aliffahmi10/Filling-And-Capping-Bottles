#include <Servo.h>

// === PIN KONSTANTA ===
#define DIR_PIN 2
#define STEP_PIN 3
#define ENABLE_PIN 4
#define IR1 8
#define IR2 9
#define RELAY1 10
#define RELAY2 11
#define RELAY3 12
#define START_BTN A0
#define STOP_BTN A1
#define SERVO1_PIN 5
#define SERVO2_PIN 6
#define SERVO3_PIN 7
#define LED_PIN 13 

// === WAKTU & KONSTANTA ===

const unsigned long sensorDelay = 500, simultanWindow = 100;
const unsigned long relay1Dur = 9200, relay2Dur = 4083, relay3Dur = 4083, relay11Dur = 10200;
const int stepDelay = 5000;

unsigned long now, timeStart, r1Start, r2Start, r3Start, s1Time, s2Time, pendingTime, s1Start;
unsigned long servoTimer, servo2Timer, servo3Timer;

bool systemOn = false, r1Active = false, r2Active = false, r3Active = false;
bool motorOn = false, processing = false, ignoreSensor = false;
bool s1Pending = false, s2Pending = false;
bool lastS1 = HIGH, lastS2 = HIGH;
bool servo2Go = false, startawal = false, FLAS = false;

int servoState = 0, servoAfterPump = 0, servo2State = 0, servo3State = 0;

Servo servo1, servo2, servo3;

void setup() {
  pinMode(DIR_PIN, OUTPUT); pinMode(STEP_PIN, OUTPUT); pinMode(ENABLE_PIN, OUTPUT);
  pinMode(IR1, INPUT); pinMode(IR2, INPUT);
  pinMode(RELAY1, OUTPUT); pinMode(RELAY2, OUTPUT); pinMode(RELAY3, OUTPUT);
  pinMode(START_BTN, INPUT_PULLUP); pinMode(STOP_BTN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(RELAY1, HIGH); digitalWrite(RELAY2, HIGH); digitalWrite(RELAY3, HIGH);
  digitalWrite(LED_PIN, LOW);

  servo1.attach(SERVO1_PIN); servo1.write(0);
  servo2.attach(SERVO2_PIN); servo2.write(0);
  servo3.attach(SERVO3_PIN); servo3.write(0);

  Serial.begin(115200);
  Serial.println("Sistem Siap. Tekan START untuk memulai.");
}

void loop() {
  now = millis();

  if (digitalRead(STOP_BTN) == HIGH && debounce(STOP_BTN)) {
    FLAS = true;
    return stopSystem();
  }
  if (!systemOn && digitalRead(START_BTN) == HIGH && debounce(START_BTN)) {
    systemOn = true;
    aktifkanStepper();
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Tombol START ditekan → Sistem Dimulai");
    startawal = true;
  }
  if (!systemOn) return;
  if (FLAS && startawal) { startawal = false; }

  bool s1 = digitalRead(IR1);
  bool s2 = digitalRead(IR2);

  if (lastS1 == HIGH && s1 == LOW && !s1Pending) {
    s1Pending = true; s1Time = now;
    Serial.println("Sensor 1 terdeteksi");
    matikanStepper();
  }
  if (lastS2 == HIGH && s2 == LOW && !s2Pending) {
    s2Pending = true; s2Time = now;
    Serial.println("Sensor 2 terdeteksi");
    matikanStepper();
  }
  lastS1 = s1; lastS2 = s2;

  if (!processing && !(r1Active || r2Active || r3Active) && !ignoreSensor && (s1Pending || s2Pending)) {
    if (pendingTime == 0) {
      pendingTime = now;
      Serial.println("Mulai window deteksi simultan");
      matikanStepper();
    }

    bool s1Act = s1Pending && (now - s1Time <= sensorDelay);
    bool s2Act = s2Pending && (now - s2Time <= sensorDelay);

    if (now - pendingTime <= simultanWindow) {
      if (s1Act && s2Act) {
        startProcess(true, true);
        servoState = servo2State = 1;
        servoTimer = servo2Timer = servo3Timer = now;
        Serial.println("Deteksi SIMULTAN (Sensor1 & Sensor2)");
      }
    } else {
      if (s1Act) {
        startProcess(true, false);
        servo2State =  1;
        servo2Timer = servo3Timer = now;
        Serial.println("Sensor1 saja terdeteksi");
      } else if (s2Act) {
        startProcess(false, true);
        servoState = 1;
        servoTimer = now;
        Serial.println("Sensor2 saja terdeteksi");
      }
      resetSensorStates();
    }
  }

  // === RELAY TIMER ===
  if (!startawal && r1Active && now - r1Start >= relay1Dur) {
    r1Active = false; digitalWrite(RELAY1, HIGH);
    Serial.println("Relay1 OFF");
    servoAfterPump = 1; servoState = 3; s1Start = now;servo3State = 1;
  }
  if (startawal && r1Active && now - r1Start >= relay11Dur) {
    r1Active = false; digitalWrite(RELAY1, HIGH);
    Serial.println("Relay1 OFF");
    servoAfterPump = 1; servoState = 3; s1Start = now;servo3State = 1;
    servo3State = 1;
  }
  if (r2Active && now - r2Start >= relay2Dur) {
    r2Active = false; digitalWrite(RELAY2, HIGH); Serial.println("Relay2 OFF");servo3State = 1;
  }
  if (r3Active && now - r3Start >= relay3Dur) {
    r3Active = false; digitalWrite(RELAY3, HIGH); Serial.println("Relay3 OFF");
  }

  if (!r1Active && !r2Active && !r3Active && processing) {
    processing = false; ignoreSensor = false;
    aktifkanStepper(); resetSensorStates();
    Serial.println("Proses selesai → Sistem siap deteksi lagi");
  }

  // === SERVO2 ===
  if (servo2State == 1 && now - servo2Timer >= 500) {
    servo2.write(45); servo2Timer = now; servo2State = 2;
  }
  if (servo2State == 2 && now - servo2Timer >= 1000) {
    servo2.write(0); servo2State = 0;
  }

  // === SERVO3 ===
  if (servo3State == 1 && now - servo3Timer >= 1) {
    servo3.write(60); servo3Timer = now; servo3State = 2;
  }
  if (servo3State == 2 && now - servo3Timer >= 300) {
    servo3.write(0); servo3State = 0;
  }

  // === SERVO1 ===
  if (servoState == 3 && servoAfterPump == 1 && now - s1Start >= 1800) {
    servo1.write(90); s1Start = now; servoAfterPump = 2; servo2Go = true;
    matikanStepper();
  }
  if (servo2Go && now - s1Start >= 1000) {
    servo1.write(0); servo2Go = false; servoState = 0;
    aktifkanStepper();
  }

  if (systemOn && motorOn) {
    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, LOW); delayMicroseconds(stepDelay);
  }
}

void aktifkanStepper() {
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
  motorOn = true;
}

void matikanStepper() {
  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(DIR_PIN, LOW);
  motorOn = false;
}

void startProcess(bool r1, bool r2) {
  if (processing) return;
  processing = true;
  ignoreSensor = true;
  now = millis();

  if (r1) {
    r1Active = true; r1Start = now;
    digitalWrite(RELAY1, LOW);
    Serial.println("Relay1 ON");
  }
  if (r2) {
    r2Active = r3Active = true;
    r2Start = r3Start = now;
    digitalWrite(RELAY2, LOW);
    digitalWrite(RELAY3, LOW);
    Serial.println("Relay2 & Relay3 ON");
  }

  resetSensorStates();
}

void resetSensorStates() {
  s1Pending = s2Pending = false;
  pendingTime = s1Time = s2Time = 0;
}

void stopSystem() {
  matikanStepper();
  systemOn = processing = ignoreSensor = false;
  r1Active = r2Active = r3Active = false;
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2, HIGH);
  digitalWrite(RELAY3, HIGH);
  digitalWrite(LED_PIN, LOW);
  resetSensorStates();
  Serial.println("Sistem Dihentikan");
  delay(200);
}

bool debounce(int pin) {
  delay(50);
  return digitalRead(pin) == LOW;
}
