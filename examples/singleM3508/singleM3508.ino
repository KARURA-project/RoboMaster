#include <SPI.h>
#include <mcp2515.h>
#include "RPi_Pico_TimerInterrupt.h"
#include "RoboMaster.h"

#define RX_PIN 4
#define CS_PIN 5
#define SCK_PIN 6
#define TX_PIN 7

MCP2515 mcp(CS_PIN, 8000000, &SPI);
struct can_frame sendMsg[2] = {}, readMsg = {};
RPI_PICO_Timer recvTimer(0);
RPI_PICO_Timer sendTimer(1);
RPI_PICO_Timer serialTimer(2);
M3508 m3508;

struct {
  uint32_t recv   = 1000000 / FEEDBACK_500HZ;
  uint32_t send   = 1000000 / FEEDBACK_500HZ;
  uint32_t serial = 1000000 / 10;
} dt;

struct {
  volatile bool recvMotor = false;
  volatile bool sendMotor = false;
  volatile bool serialIO = false;
} flag;

void initSPI();
void initMCP();
void initTimer();
void setMotorParam();
void recvMotor();
void sendMotor();
bool recvMotorFlag(struct repeating_timer *t);
bool sendMotorFlag(struct repeating_timer *t);
bool serialIOFlag(struct repeating_timer *t);

void setup() {
  Serial.begin(115200);
  initSPI();
  initMCP();
  setMotorParam();
  initTimer();
}

void loop() {
  if (flag.recvMotor) recvMotor();
  if (flag.sendMotor) sendMotor();
  if (flag.serialIO) serialIO();
}


void initSPI() {
  SPI.setRX(RX_PIN);
  SPI.setSCK(SCK_PIN);
  SPI.setTX(TX_PIN);
  SPI.begin();
}

void initMCP() {
  mcp.reset();
  mcp.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp.setNormalMode();
}

void initTimer() {
  recvTimer.attachInterruptInterval(dt.recv, recvMotorFlag);
  sendTimer.attachInterruptInterval(dt.send, sendMotorFlag);
  serialTimer.attachInterruptInterval(dt.serial, serialIOFlag);
}

void setMotorParam() {
  m3508.mode             = MODE::SLEEP;
  m3508.gearRatio        = 1.0;
  m3508.direction        = DIRECTION::FWD;
  m3508.pidParam.angle   = { 0.5, 0.3, 0.0, 3600.0, 450.0 };
  m3508.pidParam.speed   = { 0.1, 0.1, 0.0,  450.0,  20.0 };
  m3508.pidParam.current = { 0.0, 0.0, 0.0,   20.0,  20.0 };
  m3508.setPidInterval(8, 4, 2);
  m3508.init(1);
}

void recvMotor() {
  MCP2515::ERROR status = mcp.readMessage(&readMsg);
  if (status == MCP2515::ERROR_OK) m3508.refresh(micros(), sendMsg, readMsg);
  flag.recvMotor = false;
}

void sendMotor() {
  for (uint8_t i = 0; i < 2; i++) {
    if (sendMsg[i].can_id != 0x00) {
      mcp.sendMessage(&sendMsg[i]);
      sendMsg[i].can_id = 0x00;
    }
  }
  flag.sendMotor = false;
}

void serialIO() {
  if (Serial.available()) {
    switch (Serial.read()) {
      case '1': m3508.mode = MODE::ANGLE; break;
      case 'q': m3508.target.angle += 360.0; break;
      case 'a': m3508.target.angle = 0.0; break;
      case 'z': m3508.target.angle -= 360.0; break;
      case '2': m3508.mode = MODE::SPEED; break;
      case 'w': m3508.target.speed += 60.0; break;
      case 's': m3508.target.speed = 0.0; break;
      case 'x': m3508.target.speed -= 60.0; break;
      case '3': m3508.mode = MODE::CURRENT; break;
      case 'e': m3508.target.current += 0.5; break;
      case 'd': m3508.target.current = 0.0; break;
      case 'c': m3508.target.current -= 0.5; break;
    }
  }
  Serial.printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", m3508.target.angle, m3508.target.speed, m3508.target.current, m3508.getAngle(), m3508.getSpeed(), m3508.getCurrent());
  flag.serialIO = false;
}

bool recvMotorFlag(struct repeating_timer *t) {
  flag.recvMotor = true;
  return true;
}

bool sendMotorFlag(struct repeating_timer *t) {
  flag.sendMotor = true;
  return true;
}

bool serialIOFlag(struct repeating_timer *t) {
  flag.serialIO = true;
  return true;
}