#include <Arduino.h>

// XIAO ESP32C3 UART Pin Definitions
#define CRSF_RX_PIN D7  // GPIO20 -> Wired directly to MT12 Nano Pin 1
#define CRSF_TX_PIN D6  // GPIO21 -> Wired through ~652 ohm resistor to Pin 1
#define CRSF_BAUD   400000

// CRSF Protocol Constants
#define CRSF_ADDRESS_CRSF_TRANSMITTER  0xEE
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA
#define CRSF_FRAMETYPE_BATTERY_SENSOR  0x08
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14
#define CRSF_FRAMETYPE_FLIGHT_MODE     0x21
#define CRSF_FRAMETYPE_PARAMETER_WRITE 0x2D

// Packet Parser & Bus Sync
uint8_t rxBuf[64];
uint8_t rxIdx = 0;
uint16_t bytesToIgnore = 0;

unsigned long lastTelemetryMs = 0;
unsigned long lastLinkStatsMs = 0;
unsigned long lastDemoMsgMs   = 0;
unsigned long lastChunkSentMs = 0;
uint32_t packetCounter        = 0;

// Chunked Message Transmitter State (up to 9 chunks x 10 chars = 90 chars)
char pendingMsg[91]    = {0};
uint8_t pendingLen     = 0;
uint8_t totalChunks    = 0;
uint8_t nextChunkIdx   = 0;
char currentMsgId      = 'A';
bool chunkTxActive     = false;

// Simulated Control Inputs
float steerNorm = 0.0f;
float steerStep = 0.04f;
float throtNorm = 0.0f;
float throtStep = 0.02f;

// Live Status Received FROM MT12 Lua Script (0x2D)
uint8_t mt12BatteryPct   = 0;
float   mt12BatteryVolts = 0.0f;
uint8_t mt12SignalPct    = 0;

uint8_t crsfCrc8(const uint8_t *ptr, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= ptr[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void writeHalfDuplexFrame(const uint8_t *frame, uint8_t len) {
  bytesToIgnore += len;
  Serial1.write(frame, len);
  Serial1.flush();
}

void sendCrsfLinkStats(uint8_t linkQualityPct) {
  uint8_t frame[14];
  frame[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
  frame[1] = 12;
  frame[2] = CRSF_FRAMETYPE_LINK_STATISTICS;

  frame[3]  = 50;
  frame[4]  = 50;
  frame[5]  = linkQualityPct;
  frame[6]  = 10;
  frame[7]  = 0;
  frame[8]  = 2;
  frame[9]  = 3;
  frame[10] = 50;
  frame[11] = linkQualityPct;
  frame[12] = 10;

  frame[13] = crsfCrc8(&frame[2], 11);
  writeHalfDuplexFrame(frame, sizeof(frame));
}

void sendCrsfControls(float steering, float throttle, uint8_t brakePct) {
  uint8_t frame[12];

  steering = constrain(steering, -1.0f, 1.0f);
  throttle = constrain(throttle,  0.0f, 1.0f);
  if (brakePct > 100) brakePct = 100;

  uint16_t steerPacked = (uint16_t)((steering + 1.0f) * 1000.0f);
  uint16_t throtPacked = (uint16_t)(throttle * 1000.0f);

  frame[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
  frame[1] = 10;
  frame[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;

  frame[3] = (steerPacked >> 8) & 0xFF;
  frame[4] = steerPacked & 0xFF;
  frame[5] = (throtPacked >> 8) & 0xFF;
  frame[6] = throtPacked & 0xFF;
  frame[7] = 0;
  frame[8] = 0;
  frame[9] = 0;
  frame[10] = brakePct;

  frame[11] = crsfCrc8(&frame[2], 9);
  writeHalfDuplexFrame(frame, sizeof(frame));
}

// Queue a long console message (up to 90 chars) to be sliced into 10-char CRSF chunks
void queueConsoleMessage(const char *msg) {
  if (!msg || msg[0] == '\0') return;

  strncpy(pendingMsg, msg, 90);
  pendingMsg[90] = '\0';
  pendingLen = strlen(pendingMsg);

  totalChunks = (pendingLen + 9) / 10; // Ceiling division by 10 chars per chunk
  if (totalChunks > 9) totalChunks = 9;

  nextChunkIdx = 0;
  currentMsgId++;
  if (currentMsgId > 'Z') currentMsgId = 'A';
  chunkTxActive = true;
}

// Transmit the next 13-byte chunk: [msgId][chunkIdx][totalChunks][up to 10 chars]
void sendNextMessageChunk() {
  if (!chunkTxActive || nextChunkIdx >= totalChunks) {
    chunkTxActive = false;
    return;
  }

  uint8_t offset = nextChunkIdx * 10;
  uint8_t sliceLen = pendingLen - offset;
  if (sliceLen > 10) sliceLen = 10;

  char chunkText[14];
  chunkText[0] = currentMsgId;
  chunkText[1] = '0' + nextChunkIdx;
  chunkText[2] = '0' + totalChunks;
  memcpy(&chunkText[3], &pendingMsg[offset], sliceLen);
  chunkText[3 + sliceLen] = '\0';

  uint8_t textLen = 3 + sliceLen;
  uint8_t payloadLen = textLen + 1;
  uint8_t frame[20];

  frame[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
  frame[1] = payloadLen + 2;
  frame[2] = CRSF_FRAMETYPE_FLIGHT_MODE;
  memcpy(&frame[3], chunkText, textLen);
  frame[3 + textLen] = '\0';
  frame[3 + payloadLen] = crsfCrc8(&frame[2], payloadLen + 1);

  writeHalfDuplexFrame(frame, payloadLen + 4);

  nextChunkIdx++;
  if (nextChunkIdx >= totalChunks) {
    chunkTxActive = false;
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN, true);
}

void loop() {
  unsigned long nowMs = millis();

  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    if (bytesToIgnore > 0) {
      bytesToIgnore--;
      continue;
    }

    if (rxIdx == 0) {
      if (b == CRSF_ADDRESS_CRSF_TRANSMITTER || b == CRSF_ADDRESS_RADIO_TRANSMITTER || b == 0xC8) {
        rxBuf[rxIdx++] = b;
      }
    } else {
      rxBuf[rxIdx++] = b;

      if (rxIdx >= 2) {
        uint8_t expectedTotalLen = rxBuf[1] + 2;

        if (expectedTotalLen > sizeof(rxBuf) || expectedTotalLen < 4) {
          rxIdx = 0;
        } else if (rxIdx >= expectedTotalLen) {
          uint8_t calcCrc = crsfCrc8(&rxBuf[2], rxBuf[1] - 1);
          uint8_t pktCrc  = rxBuf[expectedTotalLen - 1];
          rxIdx = 0;

          if (calcCrc == pktCrc) {
            uint8_t frameType = rxBuf[2];

            if (frameType == CRSF_FRAMETYPE_PARAMETER_WRITE && expectedTotalLen >= 9) {
              mt12BatteryPct   = rxBuf[5];
              mt12SignalPct    = rxBuf[6];
              mt12BatteryVolts = rxBuf[7] / 10.0f;
            }

            delayMicroseconds(50);

            // Priority 1: Link Statistics every 250ms
            if (nowMs - lastLinkStatsMs >= 250) {
              lastLinkStatsMs = nowMs;
              sendCrsfLinkStats(98);
            }
            // Priority 2: Send active message chunks paced at 60ms intervals
            else if (chunkTxActive && (nowMs - lastChunkSentMs >= 60)) {
              lastChunkSentMs = nowMs;
              sendNextMessageChunk();
            }
            // Priority 3: Queue a long test message every 2 seconds
            else if (!chunkTxActive && (nowMs - lastDemoMsgMs >= 2000)) {
              lastDemoMsgMs = nowMs;
              packetCounter++;
              char longMsg[64];
              snprintf(longMsg, sizeof(longMsg), "HELLO MT12 #%lu - ESP-NOW READY", packetCounter);
              queueConsoleMessage(longMsg);
            }
            // Priority 4: 50Hz Steering / Throttle / Brake controls
            else if (nowMs - lastTelemetryMs >= 20) {
              lastTelemetryMs = nowMs;

              steerNorm += steerStep;
              if (steerNorm >= 1.0f || steerNorm <= -1.0f) {
                steerStep = -steerStep;
              }

              throtNorm += throtStep;
              if (throtNorm >= 1.0f || throtNorm <= 0.0f) {
                throtStep = -throtStep;
              }
              uint8_t brakePct = (uint8_t)((1.0f - throtNorm) * 100.0f);

              sendCrsfControls(steerNorm, throtNorm, brakePct);
            }
          }
        }
      }
    }
  }
}