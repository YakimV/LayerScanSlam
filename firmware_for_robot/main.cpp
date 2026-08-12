#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ── Configuration ──────────────────────────────────────────────────
const char* ssid      = "ESP32_ROBOT";
const char* password  = "12345678";
IPAddress targetIP(192, 168, 4, 2);
const char* targetIPc = "192.168.4.2";

const int portLidar = 8888;
const int portMotor = 8890;
const int portIMU   = 8891;

WiFiUDP udpTx, udpRx, udpIMU;

// ── Pins ──────────────────────────────────────────────────────────
#define RX_PIN  1
#define TX_PIN  21
#define PIN_A1A 4
#define PIN_A1B 5
#define PIN_B1A 6
#define PIN_B1B 7
#define I2C_SDA 8
#define I2C_SCL 9

// ── PWM ───────────────────────────────────────────────────────────
const int PWM_FREQ = 1000;
const int PWM_RES  = 8;
const int CH_A1A = 0, CH_A1B = 1;
const int CH_B1A = 2, CH_B1B = 3;

// ── Parameters ────────────────────────────────────────────────────
const size_t   SERIAL1_RXBUF     = 4096;
const size_t   LIDAR_PACKET_MAX  = 500;
const size_t   LIDAR_MIN_FRAME   = 36;
const uint32_t MOTOR_WATCHDOG_MS = 300;

const uint32_t STATION_CHECK_MS  = 200;

// ── State ──────────────────────────────────────────────────────────
uint32_t lastCommandTime    = 0;
uint32_t lastImuTime        = 0;
uint32_t lastStationCheck   = 0;
bool     clientConnected    = false;
MPU6050  mpu(Wire);

// ── PWM setup ─────────────────────────────────────────────────────
void setupPWM() {
  pinMode(PIN_A1A, OUTPUT); pinMode(PIN_A1B, OUTPUT);
  pinMode(PIN_B1A, OUTPUT); pinMode(PIN_B1B, OUTPUT);
  digitalWrite(PIN_A1A, LOW); digitalWrite(PIN_A1B, LOW);
  digitalWrite(PIN_B1A, LOW); digitalWrite(PIN_B1B, LOW);
  ledcSetup(CH_A1A, PWM_FREQ, PWM_RES); ledcSetup(CH_A1B, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B1A, PWM_FREQ, PWM_RES); ledcSetup(CH_B1B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_A1A, CH_A1A); ledcAttachPin(PIN_A1B, CH_A1B);
  ledcAttachPin(PIN_B1A, CH_B1A); ledcAttachPin(PIN_B1B, CH_B1B);
}

void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  if (left  > 0 && left  < 60)  left  = 60;
  if (left  < 0 && left  > -60) left  = -60;
  if (right > 0 && right < 60)  right = 60;
  if (right < 0 && right > -60) right = -60;
  if (left  > 0) { ledcWrite(CH_B1B, 0);  ledcWrite(CH_B1A, left);    }
  else if (left  < 0) { ledcWrite(CH_B1A, 0); ledcWrite(CH_B1B, -left); }
  else                { ledcWrite(CH_B1A, 0); ledcWrite(CH_B1B, 0);     }
  if (right > 0) { ledcWrite(CH_A1B, 0);  ledcWrite(CH_A1A, right);   }
  else if (right < 0) { ledcWrite(CH_A1A, 0); ledcWrite(CH_A1B, -right);}
  else                { ledcWrite(CH_A1A, 0); ledcWrite(CH_A1B, 0);    }
}

void safeUdpSend(WiFiUDP &udp, const uint8_t *data, size_t len,
                 const IPAddress &ip, uint16_t port) {
  if (!len || !data) return;
  if (!udp.beginPacket(ip, port)) return;
  udp.write(data, len);
  udp.endPacket();
  yield();  
}

// ── setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(800);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // Fast mode I2C
  if (mpu.begin() == 0) {
    delay(1000);
    mpu.calcOffsets();
  }

  Serial1.setRxBufferSize(SERIAL1_RXBUF);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  setupPWM();
  setMotors(0, 0);

  WiFi.mode(WIFI_MODE_AP);
  WiFi.setSleep(false);                          // Critical for low latency
  //WiFi.setTxPower(WIFI_POWER_19_5dBm);

  IPAddress local_ip(192,168,4,1), gateway(192,168,4,1), subnet(255,255,255,0);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password, 11, 0, 1);         // Channel 11, max 1 client

  udpRx.begin(portMotor);

  lastCommandTime  = millis();
  lastImuTime      = millis();
  lastStationCheck = 0;

  Serial.println("[OK] Ready");
}

// ── loop ──────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // Cache client state every STATION_CHECK_MS ms
  if (now - lastStationCheck > STATION_CHECK_MS) {
    clientConnected  = (WiFi.softAPgetStationNum() > 0);
    lastStationCheck = now;
  }

  if (!clientConnected) {
    setMotors(0, 0);
    // Flush LiDAR buffer to prevent buildup
    static uint8_t flushBuf[256];
    while (Serial1.available()) {
      size_t n = min((size_t)Serial1.available(), sizeof(flushBuf));
      Serial1.readBytes((char*)flushBuf, n);
    }
    delay(10);
    return;
  }

  // ── IMU ~100 Hz ───────────────────────────────────────────────
  if (now - lastImuTime >= 10) {
    mpu.update();
    char imuPacket[128];
    int n = snprintf(imuPacket, sizeof(imuPacket),
                     "IMU:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                     mpu.getAngleZ(),
                     mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(),
                     mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ());
    if (n > 0) {
      udpIMU.beginPacket(targetIPc, portIMU);
      udpIMU.write((uint8_t*)imuPacket, (size_t)n);
      udpIMU.endPacket();
      yield();
    }
    lastImuTime = now;
  }

  // ── Motor commands ───────────────────────────────────────────
  int pktSize = udpRx.parsePacket();
  while (pktSize) {
    char buf[64];
    int len = udpRx.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = 0;
      int L = 0, R = 0;
      if (sscanf(buf, "%d,%d", &L, &R) == 2) {
        setMotors(L, R);
        lastCommandTime = millis();
      }
    }
    pktSize = udpRx.parsePacket();
  }

  // Watchdog
  if (millis() - lastCommandTime > MOTOR_WATCHDOG_MS) {
    setMotors(0, 0);
  }

  // ── LiDAR → UDP ───────────────────────────────────────────────
  size_t avail = Serial1.available();
  if (avail > 0) {
    // If accumulated too much — flush old data
    if (avail > 800) {
      static uint8_t flushBuf[512];
      while (Serial1.available() > 400) {
        size_t n = min((size_t)Serial1.available(), sizeof(flushBuf));
        Serial1.readBytes((char*)flushBuf, n);
      }
      avail = Serial1.available();
    }

    if (avail >= LIDAR_MIN_FRAME) {
      static uint8_t lidarBuf[LIDAR_PACKET_MAX];
      size_t toRead = min(avail, LIDAR_PACKET_MAX);
      size_t readLen = Serial1.readBytes((char*)lidarBuf, toRead);
      if (readLen > 0) {
        safeUdpSend(udpTx, lidarBuf, readLen, targetIP, portLidar);
      }
    }
  }

  delay(1);
}
