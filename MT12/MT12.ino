#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

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

// Fleet ESP-NOW Configuration (Matches Gateway Channel 2)
#define ESPNOW_CHANNEL 2
uint8_t broadcastMac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

enum FleetColor {
  COLOR_ORANGE = 0,
  COLOR_GREEN  = 1,
  COLOR_TAN    = 2,
  COLOR_YELLOW = 3
};

enum MessageType {
  MSG_TYPE_STEERING = 0x01,
  MSG_TYPE_PEDALS   = 0x02,
  MSG_TYPE_RADIO    = 0x03,
  MSG_TYPE_LOG      = 0x04
};

#pragma pack(push, 1)
struct packet_header {
  uint8_t magic;    // 0xAF
  uint8_t type;     // MessageType enum
  uint8_t color_id; // FleetColor routing
};

struct msg_steering_telemetry {
  packet_header header;
  int8_t steering_angle;   // -100 to +100
  uint8_t battery_percent;
  uint8_t drive_mode;      // 0 = forward, 1 = reverse
};

struct msg_pedals_telemetry {
  packet_header header;
  uint8_t throttle_percent; // 0 to 100
  uint8_t brake_percent;    // 0 to 100
  uint8_t battery_percent;
};

struct msg_radio_telemetry {
  packet_header header;
  uint8_t current_vehicle;
  bool multi_mode;
  uint8_t battery_percent;
};

struct msg_log_packet {
  packet_header header;
  char log_message[64];
};
#pragma pack(pop)

// Active Fleet Color Filter & Handshake State
volatile uint8_t activeColorId   = COLOR_ORANGE;
volatile bool initialColorSynced = false;
uint8_t lastReportedColorId      = 255;

// Thread-safe Control & Telemetry State
volatile float liveSteer           = 0.0f; // -1.0 to +1.0
volatile float liveThrottle        = 0.0f; //  0.0 to 1.0
volatile float liveBrake           = 0.0f; //  0.0 to 1.0
volatile uint8_t liveDriveMode     = 0;    // 0 = fwd, 1 = rev
volatile unsigned long lastSteerRxMs  = 0;
volatile unsigned long lastPedalsRxMs = 0;

volatile bool newIncomingLog       = false;
char incomingLogBuf[64]            = {0};
bool nodesConnected                = false;

// CRSF Half-Duplex Parser & Bus Sync
uint8_t rxBuf[64];
uint8_t rxIdx = 0;
uint16_t bytesToIgnore = 0;

unsigned long lastTelemetryMs   = 0;
unsigned long lastLinkStatsMs   = 0;
unsigned long lastRadioBeaconMs = 0;

// Chunked Message Transmitter State (up to 9 chunks x 10 chars = 90 chars)
char pendingMsg[91]    = {0};
uint8_t pendingLen     = 0;
uint8_t totalChunks    = 0;
uint8_t nextChunkIdx   = 0;
char currentMsgId      = 'A';
bool chunkTxActive     = false;
unsigned long lastChunkSentMs = 0;

// Live Status Received FROM MT12 Lua Script (0x2D)
uint8_t mt12BatteryPct   = 0;
float   mt12BatteryVolts = 0.0f;
uint8_t mt12SignalPct    = 0;

const char* getColorName(uint8_t color_id) {
  switch (color_id) {
    case COLOR_ORANGE: return "ORANGE";
    case COLOR_GREEN:  return "GREEN";
    case COLOR_TAN:    return "TAN";
    case COLOR_YELLOW: return "YELLOW";
    default:           return "UNKNOWN";
  }
}

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

// Sends Steering (RxBt), Throttle (Curr), Confirmed Color ID (Capa), and Brake (Bat%)
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
  frame[9] = initialColorSynced ? activeColorId : 255; // 255 = waiting for MT12 boot handshake
  frame[10] = brakePct;

  frame[11] = crsfCrc8(&frame[2], 9);
  writeHalfDuplexFrame(frame, sizeof(frame));
}

void queueConsoleMessage(const char *msg) {
  if (!msg || msg[0] == '\0') return;

  strncpy(pendingMsg, msg, 90);
  pendingMsg[90] = '\0';
  pendingLen = strlen(pendingMsg);

  totalChunks = (pendingLen + 9) / 10;
  if (totalChunks > 9) totalChunks = 9;

  nextChunkIdx = 0;
  currentMsgId++;
  if (currentMsgId > 'Z') currentMsgId = 'A';
  chunkTxActive = true;
}

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

// ESP-NOW Receive Callback (Filters strictly by Magic 0xAF and activeColorId)
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < (int)sizeof(packet_header)) return;

  packet_header header;
  memcpy(&header, data, sizeof(packet_header));

  if (header.magic != 0xAF || header.color_id != activeColorId) {
    return;
  }

  switch (header.type) {
    case MSG_TYPE_STEERING: {
      if (len == sizeof(msg_steering_telemetry)) {
        msg_steering_telemetry incoming;
        memcpy(&incoming, data, sizeof(incoming));

        int8_t clampedAngle = constrain(incoming.steering_angle, -100, 100);
        liveSteer = clampedAngle / 100.0f;
        liveDriveMode = incoming.drive_mode;
        lastSteerRxMs = millis();
      }
      break;
    }

    case MSG_TYPE_PEDALS: {
      if (len == sizeof(msg_pedals_telemetry)) {
        msg_pedals_telemetry incoming;
        memcpy(&incoming, data, sizeof(incoming));

        uint8_t clampedThr = (incoming.throttle_percent > 100) ? 100 : incoming.throttle_percent;
        uint8_t clampedBrk = (incoming.brake_percent > 100)    ? 100 : incoming.brake_percent;

        liveThrottle = clampedThr / 100.0f;
        liveBrake    = clampedBrk / 100.0f;
        lastPedalsRxMs = millis();
      }
      break;
    }

    case MSG_TYPE_LOG: {
      if (!newIncomingLog) {
        msg_log_packet incoming;
        memset(&incoming, 0, sizeof(incoming));
        memcpy(&incoming, data, min((size_t)len, sizeof(msg_log_packet)));
        incoming.log_message[sizeof(incoming.log_message) - 1] = '\0';

        if (incoming.log_message[0] != '\0') {
          strncpy(incomingLogBuf, incoming.log_message, sizeof(incomingLogBuf) - 1);
          incomingLogBuf[sizeof(incomingLogBuf) - 1] = '\0';
          newIncomingLog = true;
        }
      }
      break;
    }

    default:
      break;
  }
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    queueConsoleMessage("ERR: ESPNOW INIT FAIL");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastMac, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN, true);
  initEspNow();
  queueConsoleMessage("ESP32 READY - SYNCING...");
}

void loop() {
  unsigned long nowMs = millis();

  // 1. Announce after receiving the MT12's 0x2D handshake (and on any subsequent change)
  if (initialColorSynced && (activeColorId != lastReportedColorId) && !chunkTxActive) {
    lastReportedColorId = activeColorId;
    liveSteer    = 0.0f;
    liveThrottle = 0.0f;
    liveBrake    = 0.0f;
    nodesConnected = false;

    char colorMsg[32];
    snprintf(colorMsg, sizeof(colorMsg), "SYNCED: %u-%s (CH%d)",
             activeColorId, getColorName(activeColorId), ESPNOW_CHANNEL);
    queueConsoleMessage(colorMsg);
  }

  // 2. Failsafe & Connection State Tracking (300ms timeout -> neutral controls)
  bool steerOk  = (lastSteerRxMs > 0)  && (nowMs - lastSteerRxMs < 300);
  bool pedalsOk = (lastPedalsRxMs > 0) && (nowMs - lastPedalsRxMs < 300);
  bool linkUp   = initialColorSynced && (steerOk || pedalsOk);

  if (linkUp && !nodesConnected) {
    nodesConnected = true;
    if (!chunkTxActive) {
      char connMsg[32];
      snprintf(connMsg, sizeof(connMsg), "%s NODES ONLINE", getColorName(activeColorId));
      queueConsoleMessage(connMsg);
    }
  } else if (!linkUp && nodesConnected) {
    nodesConnected = false;
    liveSteer    = 0.0f;
    liveThrottle = 0.0f;
    liveBrake    = 1.0f; // Failsafe: center steering, zero throttle, full brake
    if (!chunkTxActive) {
      queueConsoleMessage("FAILSAFE: NODES LOST");
    }
  }

  // 3. Forward any MSG_TYPE_LOG packet received for this color_id
  if (newIncomingLog && !chunkTxActive) {
    queueConsoleMessage(incomingLogBuf);
    newIncomingLog = false;
  }

  // 4. Broadcast MSG_TYPE_RADIO telemetry back to Gateway/WebUI every 500ms once synced
  if (initialColorSynced && (nowMs - lastRadioBeaconMs >= 500)) {
    lastRadioBeaconMs = nowMs;
    msg_radio_telemetry radioPkt;
    radioPkt.header.magic    = 0xAF;
    radioPkt.header.type     = MSG_TYPE_RADIO;
    radioPkt.header.color_id = activeColorId;
    radioPkt.current_vehicle = activeColorId;
    radioPkt.multi_mode      = false;
    radioPkt.battery_percent = mt12BatteryPct;

    esp_now_send(broadcastMac, (uint8_t *)&radioPkt, sizeof(radioPkt));
  }

  // 5. CRSF Half-Duplex UART Loop
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

            // Capture custom status + activeColorId pushed by ESPViz.lua (0x2D)
            // Payload: [3]=0xEE, [4]=0xEA, [5]=Bat%, [6]=Sig%, [7]=Volt*10, [8]=ColorId
            if (frameType == CRSF_FRAMETYPE_PARAMETER_WRITE && expectedTotalLen >= 10) {
              mt12BatteryPct   = rxBuf[5];
              mt12SignalPct    = rxBuf[6];
              mt12BatteryVolts = rxBuf[7] / 10.0f;
              uint8_t incomingColor = rxBuf[8];
              if (incomingColor <= COLOR_YELLOW) {
                activeColorId = incomingColor;
                initialColorSynced = true;
              }
            }

            delayMicroseconds(50);

            if (nowMs - lastLinkStatsMs >= 250) {
              lastLinkStatsMs = nowMs;
              sendCrsfLinkStats(nodesConnected ? 100 : 50);
            } else if (chunkTxActive && (nowMs - lastChunkSentMs >= 60)) {
              lastChunkSentMs = nowMs;
              sendNextMessageChunk();
            } else if (nowMs - lastTelemetryMs >= 20) {
              lastTelemetryMs = nowMs;
              uint8_t brakePct = (uint8_t)(liveBrake * 100.0f + 0.5f);
              sendCrsfControls(liveSteer, liveThrottle, brakePct);
            }
          }
        }
      }
    }
  }
}