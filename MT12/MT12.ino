#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// Color IDs to route data to the right dashboard column
enum FleetColor {
    COLOR_ORANGE = 0,
    COLOR_GREEN = 1,
    COLOR_TAN = 2,
    COLOR_YELLOW = 3
};

// Explicit Message Type Identifiers
enum MessageType {
    MSG_TYPE_STEERING = 0x01,
    MSG_TYPE_PEDALS   = 0x02,
    MSG_TYPE_RADIO    = 0x03,
    MSG_TYPE_LOG      = 0x04
};

#pragma pack(push, 1)
// Standardized header present at the start of all fleet packets
struct packet_header {
    uint8_t magic;    // Unique signature byte (0xAF)
    uint8_t type;     // MessageType enum
    uint8_t color_id; // Fleet color routing
};

struct msg_steering_telemetry {
    packet_header header;
    int8_t steering_angle;
    uint8_t battery_percent;
    uint8_t drive_mode;
};

struct msg_pedals_telemetry {
    packet_header header;
    uint8_t throttle_percent; 
    uint8_t brake_percent;    
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

const char* getColorName(uint8_t color_id) {
    switch(color_id) {
        case COLOR_ORANGE: return "Orange";
        case COLOR_GREEN: return "Green";
        case COLOR_TAN: return "Tan";
        case COLOR_YELLOW: return "Yellow";
        default: return "Unknown";
    }
}

// Helper to convert raw packet bytes into a hex string for the WebUI console log
void logRawPacketHex(const uint8_t *mac_addr, const uint8_t *data, int len, const char* reason) {
    char hexBuffer[128] = {0};
    int offset = 0;
    for (int i = 0; i < len && i < 32; i++) {
        offset += snprintf(hexBuffer + offset, sizeof(hexBuffer) - offset, "%02X ", data[i]);
    }
    
    Serial.printf("{\"log\": \"%s (Len: %d, From: %02X:%02X:%02X:%02X:%02X:%02X) [Hex: %s]\", \"type\": \"WARNING\", \"source\": \"Gateway\"}\n",
                  reason, len, mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], hexBuffer);
    Serial.flush();
}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    const uint8_t *mac_addr = info->src_addr;

    // 1. Minimum length check for header
    if (len < sizeof(packet_header)) {
        logRawPacketHex(mac_addr, data, len, "Packet dropped: Below minimum header size");
        return;
    }

    // Peek at header
    packet_header header;
    memcpy(&header, data, sizeof(packet_header));

    // 2. Magic byte validation
    if (header.magic != 0xAF) {
        logRawPacketHex(mac_addr, data, len, "Packet dropped: Invalid magic byte");
        return;
    }

    // 3. Route based on explicit Message Type ID
    switch (header.type) {
        case MSG_TYPE_STEERING: {
            if (len != sizeof(msg_steering_telemetry)) {
                logRawPacketHex(mac_addr, data, len, "Steering packet length mismatch");
                return;
            }
            msg_steering_telemetry incoming;
            memcpy(&incoming, data, sizeof(incoming));

            const char* driveModeStr = (incoming.drive_mode == 0) ? "forward" : "reverse";
            char jsonBuffer[256];
            int jsonLen = snprintf(jsonBuffer, sizeof(jsonBuffer), 
                "{\"color\": \"%s\", \"node\": \"steering\", \"channel\": %d, \"status\": \"Online\", \"data\": {\"steering\": %d, \"driveMode\": \"%s\", \"battery\": %d}}",
                getColorName(incoming.header.color_id), WiFi.channel(), incoming.steering_angle, driveModeStr, incoming.battery_percent
            );
            Serial.write((uint8_t*)jsonBuffer, jsonLen);
            Serial.write('\n');
            Serial.flush();
            break;
        }

        case MSG_TYPE_PEDALS: {
            if (len != sizeof(msg_pedals_telemetry)) {
                logRawPacketHex(mac_addr, data, len, "Pedals packet length mismatch");
                return;
            }
            msg_pedals_telemetry incoming;
            memcpy(&incoming, data, sizeof(incoming));

            char jsonBuffer[256];
            int jsonLen = snprintf(jsonBuffer, sizeof(jsonBuffer), 
                "{\"color\": \"%s\", \"node\": \"pedals\", \"channel\": %d, \"status\": \"Online\", \"data\": {\"throttle\": %u, \"brake\": %u, \"battery\": %u}}",
                getColorName(incoming.header.color_id), WiFi.channel(), incoming.throttle_percent, incoming.brake_percent, incoming.battery_percent
            );
            Serial.write((uint8_t*)jsonBuffer, jsonLen);
            Serial.write('\n');
            Serial.flush();
            break;
        }

        case MSG_TYPE_LOG: {
            msg_log_packet incoming;
            memcpy(&incoming, data, min((size_t)len, sizeof(msg_log_packet)));
            incoming.log_message[sizeof(incoming.log_message) - 1] = '\0';

            Serial.printf("{\"log\": \"%s\", \"type\": \"INFO\", \"source\": \"%s\"}\n",
                          incoming.log_message, getColorName(incoming.header.color_id));
            Serial.flush();
            break;
        }

        default: {
            logRawPacketHex(mac_addr, data, len, "Packet dropped: Unknown message type ID");
            break;
        }
    }
}

void setup() {
    Serial.begin(921600);
    delay(2000);
    Serial.println("\n\n=== GATEWAY BOOTING (Type-Dispatch Mode) ===");
    
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(2, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[GW_SYS] Standalone Mode - Radio locked to Channel: %d\n", WiFi.channel());

    if (esp_now_init() != ESP_OK) {
        Serial.println("{\"log\": \"ESP-NOW Init Failed\", \"type\": \"ERROR\", \"source\": \"Gateway\"}");
        return;
    }
    
    esp_now_register_recv_cb(onDataRecv);
    Serial.println("[GW_SYS] ESP-NOW Initialized & Receive Callback Registered.");
}

void loop() {
    // Unblocked high-speed packet bridging
}