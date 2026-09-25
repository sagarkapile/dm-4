#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "DriveMatrixProtocol.h"
#include <WiFiUdp.h>
#include "Vehicle.h"
#include "IMU.h"
#include "IMUConfig.h"
#include "TelemetryManager.h"

// ============================================================
// ASCENT LITE OSD
// ============================================================

#define ASCENT_TX_PIN 40
#define ASCENT_RX_PIN 41
#define ASCENT_BAUD   115200

#define BATTERY_ADC_PIN 1

// Battery divider:
// BAT+ -> 118k -> GPIO1 -> 10k -> GND
#define BATTERY_DIVIDER_MULTIPLIER (128.0f / 10.0f)

// MSP commands
#define MSP_API_VERSION       1
#define MSP_FC_VARIANT        2
#define MSP_FC_VERSION        3
#define MSP_STATUS            101
#define MSP_ANALOG            110
#define MSP_DISPLAYPORT       182
#define MSP_SET_OSD_CANVAS    188

// DisplayPort subcommands
#define DP_HEARTBEAT      0
#define DP_RELEASE        1
#define DP_CLEAR_SCREEN   2
#define DP_WRITE_STRING   3
#define DP_DRAW_SCREEN    4
#define DP_OPTIONS        5

#define DP_OPTION_HD      1

HardwareSerial AscentSerial(1);

// MSP RX parser
uint8_t ascentRxPayload[256];
uint8_t ascentRxState = 0;
uint8_t ascentRxLength = 0;
uint8_t ascentRxCommand = 0;
uint8_t ascentRxIndex = 0;
uint8_t ascentRxChecksum = 0;

// OSD state follows the authoritative RX session state.
enum AscentGameState
{
    ASCENT_GAME_WAITING,
    ASCENT_GAME_RUNNING,
    ASCENT_GAME_OVER
};

static AscentGameState ascentGameState = ASCENT_GAME_WAITING;
static uint32_t ascentSessionDurationSeconds = 0;
static uint32_t ascentSessionEndTime = 0;
static uint32_t ascentLastOverlayUpdate = 0;
static uint32_t ascentLastHeartbeat = 0;

static uint16_t ascentLapCount = 0;
static uint32_t ascentLastLapMs = 0;
static uint32_t ascentBestLapMs = 0;


// ============================================================
// SIMPLE WI-FI TELEMETRY CONNECTION
// ============================================================

// Primary Wi-Fi uses the exact connection strategy proven to work.
// For production, these credentials can later be moved back to
// provisioning without changing the connection mechanism.

// Raspberry Pi addresses for the two networks.
const char* PRIMARY_TARGET_IP  = "10.42.0.1";
const char* FALLBACK_TARGET_IP = "10.42.0.1";
const uint16_t UDP_PORT = 5005;

// Pi AP fallback. Password is taken from the existing NVS provisioning.
const char* FALLBACK_SSID = "DriveMatrix-AP";

const char* DRIVE_MATRIX_SSID = "DriveMatrix-AP";
const char* DRIVE_MATRIX_PASSWORD = "drivematrix";

WiFiUDP telemetryUDP;
static uint32_t lastTelemetry = 0;
static bool wifiConnected = false;
static bool usingFallbackWiFi = false;
static IPAddress telemetryTargetIP(0, 0, 0, 0);

// Wi-Fi service state. The RF/control loop never waits for Wi-Fi.
static volatile bool wifiServiceStarted = false;
static volatile uint32_t wifiServiceNextAttempt = 0;

// Persistent storage used by the original vehicle and Wi-Fi provisioning paths.
Preferences prefs;
Preferences wifiPrefs;

// Permanent vehicle identity/name.
char vehicleName[19] = "Jeep Gray 2";

// Provisioned Wi-Fi profile state retained for the existing RF provisioning path.
String primarySSID;
String primaryPassword;
String fallbackSSID;
String fallbackPassword;

// ============================================================
// CONTROL / SESSION STATE
// ============================================================

// These are declared before the Ascent OSD functions because the
// OSD countdown reads the same authoritative RX session timer.
static uint32_t lastControlTime = 0;
static bool sessionActive = false;
static uint32_t sessionEndTime = 0;

// ============================================================
// ASCENT LITE OSD HELPERS
// ============================================================

float Ascent_ReadBatteryVoltage()
{
    const uint8_t samples = 32;
    uint32_t adcMillivolts = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        adcMillivolts += analogReadMilliVolts(BATTERY_ADC_PIN);
        delayMicroseconds(200);
    }

    float adcVoltage =
        (float)adcMillivolts / samples / 1000.0f;

    return adcVoltage * BATTERY_DIVIDER_MULTIPLIER;
}

void Ascent_SendMSP(
    uint8_t command,
    const uint8_t* payload,
    uint8_t length
)
{
    uint8_t checksum = 0;

    AscentSerial.write('$');
    AscentSerial.write('M');
    AscentSerial.write('>');

    AscentSerial.write(length);
    checksum ^= length;

    AscentSerial.write(command);
    checksum ^= command;

    for (uint8_t i = 0; i < length; i++)
    {
        AscentSerial.write(payload[i]);
        checksum ^= payload[i];
    }

    AscentSerial.write(checksum);
    AscentSerial.flush();
}

void Ascent_SendEmptyMSP(uint8_t command)
{
    Ascent_SendMSP(command, nullptr, 0);
}

void Ascent_SendDP(const uint8_t* payload, uint8_t length)
{
    Ascent_SendMSP(
        MSP_DISPLAYPORT,
        payload,
        length
    );
}

void Ascent_SendHeartbeat()
{
    uint8_t payload[] = { DP_HEARTBEAT };
    Ascent_SendDP(payload, sizeof(payload));
}

void Ascent_SendOptions()
{
    uint8_t payload[] =
    {
        DP_OPTIONS,
        DP_OPTION_HD
    };

    Ascent_SendDP(payload, sizeof(payload));
}

void Ascent_SendClear()
{
    uint8_t payload[] = { DP_CLEAR_SCREEN };
    Ascent_SendDP(payload, sizeof(payload));
}

void Ascent_SendDraw()
{
    uint8_t payload[] = { DP_DRAW_SCREEN };
    Ascent_SendDP(payload, sizeof(payload));
}

void Ascent_SendString(
    uint8_t row,
    uint8_t column,
    const char* text
)
{
    uint8_t payload[4 + 30 + 1];

    payload[0] = DP_WRITE_STRING;
    payload[1] = row;
    payload[2] = column;
    payload[3] = 0;

    size_t len = strlen(text);

    if (len > 30)
        len = 30;

    memcpy(
        &payload[4],
        text,
        len
    );

    // DisplayPort string terminator.
    payload[4 + len] = 0;

    Ascent_SendDP(
        payload,
        5 + len
    );
}

void Ascent_DrawWaitingScreen()
{
    Ascent_SendClear();

    delay(10);

    // Top center.
    Ascent_SendString(
        0,
        19,
        "DRIVEMATRIX"
    );

    // Battery is always visible, including before the game starts.
    float batteryVoltage =
        Ascent_ReadBatteryVoltage();

    char batteryText[16];

    snprintf(
        batteryText,
        sizeof(batteryText),
        "%.2fV",
        batteryVoltage
    );

    Ascent_SendString(
        0,
        43,
        batteryText
    );

    // Center of 50 x 18 HD canvas.
    Ascent_SendString(
        8,
        12,
        "WAITING FOR GAME TO START"
    );

    Ascent_SendDraw();
}

void Ascent_DrawGameHUD()
{
    uint32_t remainingSeconds = 0;

    if (sessionActive)
    {
        int32_t remainingMs =
            (int32_t)(sessionEndTime - millis());

        if (remainingMs > 0)
        {
            remainingSeconds =
                ((uint32_t)remainingMs + 999UL) / 1000UL;
        }
    }

    uint8_t minutes =
        remainingSeconds / 60;

    uint8_t seconds =
        remainingSeconds % 60;

    char timerText[16];

    snprintf(
        timerText,
        sizeof(timerText),
        "%02u:%02u",
        minutes,
        seconds
    );

    float batteryVoltage =
        Ascent_ReadBatteryVoltage();

    char batteryText[16];

    snprintf(
        batteryText,
        sizeof(batteryText),
        "%.2fV",
        batteryVoltage
    );

    Ascent_SendClear();

    delay(10);

    // Top left: countdown.
    Ascent_SendString(
        0,
        1,
        timerText
    );

// Lap information below countdown.

char lapText[16];
char lastLapText[16];
char bestLapText[16];

snprintf(
    lapText,
    sizeof(lapText),
    "LAP %u",
    ascentLapCount
);

snprintf(
    lastLapText,
    sizeof(lastLapText),
    "LAST %02lu:%05.2f",
    ascentLastLapMs / 60000UL,
    (ascentLastLapMs % 60000UL) / 1000.0f
);

snprintf(
    bestLapText,
    sizeof(bestLapText),
    "BEST %02lu:%05.2f",
    ascentBestLapMs / 60000UL,
    (ascentBestLapMs % 60000UL) / 1000.0f
);

Ascent_SendString(
    2,
    1,
    lapText
);

Ascent_SendString(
    3,
    1,
    lastLapText
);

Ascent_SendString(
    4,
    1,
    bestLapText
);

    // Top center: DriveMatrix.
    Ascent_SendString(
        0,
        19,
        "DRIVEMATRIX"
    );

    // Top right: battery voltage.
    Ascent_SendString(
        0,
        43,
        batteryText
    );

    Ascent_SendDraw();
}

void Ascent_DrawGameOver()
{
    Ascent_SendClear();

    delay(20);

    // Keep branding.
    Ascent_SendString(
        0,
        19,
        "DRIVEMATRIX"
    );

    // Battery remains visible after the session ends.
    float batteryVoltage =
        Ascent_ReadBatteryVoltage();

    char batteryText[16];

    snprintf(
        batteryText,
        sizeof(batteryText),
        "%.2fV",
        batteryVoltage
    );

    Ascent_SendString(
        0,
        43,
        batteryText
    );

    // One-line, centered GAME OVER.
    Ascent_SendString(
        8,
        20,
        "GAME OVER"
    );

    Ascent_SendDraw();
}

void Ascent_HandleMSP(
    uint8_t command,
    uint8_t* payload,
    uint8_t length
)
{
    if (command == MSP_API_VERSION)
    {
        uint8_t response[] = { 0, 1, 46 };

        Ascent_SendMSP(
            command,
            response,
            sizeof(response)
        );

        return;
    }

    if (command == MSP_FC_VARIANT)
    {
        uint8_t response[] = { 'B', 'T', 'F', 'L' };

        Ascent_SendMSP(
            command,
            response,
            sizeof(response)
        );

        return;
    }

    if (command == MSP_FC_VERSION)
    {
        uint8_t response[] = { 4, 5, 0 };

        Ascent_SendMSP(
            command,
            response,
            sizeof(response)
        );

        return;
    }

    if (command == MSP_STATUS)
    {
        uint8_t response[11];

        memset(
            response,
            0,
            sizeof(response)
        );

        // Cycle time = 1000 us.
        response[0] = 0xE8;
        response[1] = 0x03;

        Ascent_SendMSP(
            command,
            response,
            sizeof(response)
        );

        return;
    }

    if (command == MSP_ANALOG)
    {
        float batteryVoltage =
            Ascent_ReadBatteryVoltage();

        uint16_t voltageCentiVolts =
            (uint16_t)(batteryVoltage * 100.0f);

        uint8_t response[9];

        memset(
            response,
            0,
            sizeof(response)
        );

        // Legacy voltage = 0.1 V.
        response[0] =
            (uint8_t)(batteryVoltage * 10.0f);

        // mAh.
        response[1] = 0;
        response[2] = 0;

        // RSSI.
        response[3] = 0;
        response[4] = 0;

        // Current.
        response[5] = 0;
        response[6] = 0;

        // Voltage = 0.01 V.
        response[7] =
            voltageCentiVolts & 0xFF;

        response[8] =
            (voltageCentiVolts >> 8) & 0xFF;

        Ascent_SendMSP(
            command,
            response,
            sizeof(response)
        );

        return;
    }

    if (command == MSP_SET_OSD_CANVAS)
    {
        Ascent_SendEmptyMSP(command);

        delay(30);

        Ascent_SendOptions();

        delay(30);

        Ascent_SendHeartbeat();

        return;
    }

    Ascent_SendEmptyMSP(command);
}

void Ascent_ProcessByte(uint8_t b)
{
    switch (ascentRxState)
    {
        case 0:
            if (b == '$')
                ascentRxState = 1;
            break;

        case 1:
            if (b == 'M')
                ascentRxState = 2;
            else
                ascentRxState = 0;
            break;

        case 2:
            if (b == '<')
            {
                ascentRxState = 3;
                ascentRxChecksum = 0;
            }
            else
            {
                ascentRxState = 0;
            }
            break;

        case 3:
            ascentRxLength = b;
            ascentRxChecksum = b;
            ascentRxIndex = 0;

            if (ascentRxLength > sizeof(ascentRxPayload))
                ascentRxState = 0;
            else
                ascentRxState = 4;
            break;

        case 4:
            ascentRxCommand = b;
            ascentRxChecksum ^= b;

            if (ascentRxLength == 0)
                ascentRxState = 6;
            else
                ascentRxState = 5;
            break;

        case 5:
            ascentRxPayload[ascentRxIndex++] = b;
            ascentRxChecksum ^= b;

            if (ascentRxIndex >= ascentRxLength)
                ascentRxState = 6;
            break;

        case 6:
            if (b == ascentRxChecksum)
            {
                Ascent_HandleMSP(
                    ascentRxCommand,
                    ascentRxPayload,
                    ascentRxLength
                );
            }

            ascentRxState = 0;
            break;

        default:
            ascentRxState = 0;
            break;
    }
}

void Ascent_ProcessUART()
{
    while (AscentSerial.available())
    {
        Ascent_ProcessByte(
            AscentSerial.read()
        );
    }
}

void Ascent_Init()
{
    Serial.println();
    Serial.println("==============================");
    Serial.println("DriveMatrix Ascent Lite OSD");
    Serial.println("==============================");

    Serial.println("UART TX: GPIO40");
    Serial.println("UART RX: GPIO41");
    Serial.println("UART Baud: 115200");

    Serial.println("Battery ADC: GPIO1");
    Serial.println("Battery divider: 118k / 10k");

    analogReadResolution(12);

    analogSetPinAttenuation(
        BATTERY_ADC_PIN,
        ADC_11db
    );

    AscentSerial.begin(
        ASCENT_BAUD,
        SERIAL_8N1,
        ASCENT_RX_PIN,
        ASCENT_TX_PIN
    );

    delay(500);

    float batteryVoltage =
        Ascent_ReadBatteryVoltage();

    Serial.printf(
        "Ascent initial battery: %.2f V\n",
        batteryVoltage
    );

    Ascent_SendOptions();

    delay(30);

    Ascent_SendHeartbeat();

    delay(30);

    ascentGameState = ASCENT_GAME_WAITING;
    ascentSessionDurationSeconds = 0;
    ascentSessionEndTime = 0;
    ascentLastOverlayUpdate = millis();

    Ascent_DrawWaitingScreen();

    Serial.println(
        "OSD: WAITING FOR GAME TO START"
    );
}

void Ascent_Update()
{
    Ascent_ProcessUART();

    uint32_t now = millis();

    // Keep DisplayPort alive.
    if (now - ascentLastHeartbeat >= 500)
    {
        ascentLastHeartbeat = now;
        Ascent_SendHeartbeat();
    }

    // --------------------------------------------------------
    // Session state follows the authoritative RX timer.
    // --------------------------------------------------------

    if (sessionActive)
    {
        if (ascentGameState != ASCENT_GAME_RUNNING)
        {
            ascentGameState = ASCENT_GAME_RUNNING;
            ascentSessionDurationSeconds = 0;
            ascentSessionEndTime = sessionEndTime;

            Ascent_DrawGameHUD();

            ascentLastOverlayUpdate = now;
        }

        // Keep the overlay countdown synchronized with the RX
        // sessionEndTime rather than maintaining a second timer.
        if (now - ascentLastOverlayUpdate >= 1000)
        {
            ascentLastOverlayUpdate = now;
            Ascent_DrawGameHUD();
        }
    }
    else
    {
        if (ascentGameState == ASCENT_GAME_RUNNING)
        {
            // A manual SESSION_STOP causes the normal waiting
            // screen to return.
            ascentGameState = ASCENT_GAME_WAITING;
            ascentSessionEndTime = 0;

            Ascent_DrawWaitingScreen();

            ascentLastOverlayUpdate = now;
        }
        else if (ascentGameState == ASCENT_GAME_OVER)
        {
            // Keep GAME OVER visible, but refresh the battery
            // voltage once per second.
            if (now - ascentLastOverlayUpdate >= 1000)
            {
                ascentLastOverlayUpdate = now;
                Ascent_DrawGameOver();

            }
        }
        else if (now - ascentLastOverlayUpdate >= 1000)
        {
            // Battery voltage is live on the waiting screen.
            ascentLastOverlayUpdate = now;
            Ascent_DrawWaitingScreen();
        }
    }
}

void SendUDPTelemetry()
{
    if (!wifiConnected || WiFi.status() != WL_CONNECTED)
        return;

    if (telemetryTargetIP == IPAddress(0, 0, 0, 0))
        return;

    const TelemetryPacket& packet = Telemetry_GetPacket();

    telemetryUDP.beginPacket(telemetryTargetIP, UDP_PORT);
    telemetryUDP.write(
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(packet)
    );
    telemetryUDP.endPacket();
}

void loadWiFiFallbackCredentials()
{
    wifiPrefs.begin("wifi", true);

    fallbackSSID = wifiPrefs.getString("fallback_ssid", FALLBACK_SSID);
    fallbackPassword = wifiPrefs.getString("fallback_pass", "");

    wifiPrefs.end();

    // The Pi AP name is fixed for the current system.
    fallbackSSID = FALLBACK_SSID;
}

void loadProvisionedWiFiCredentials()
{
    wifiPrefs.begin("wifi", true);

    primarySSID = wifiPrefs.getString("primary_ssid", "");
    primaryPassword = wifiPrefs.getString("primary_pass", "");

    fallbackSSID = wifiPrefs.getString("fallback_ssid", FALLBACK_SSID);
    fallbackPassword = wifiPrefs.getString("fallback_pass", "");

    wifiPrefs.end();

    // The Pi AP name remains fixed.
    fallbackSSID = FALLBACK_SSID;

    Serial.print("WIFI_PRIMARY_SSID:");
    Serial.println(primarySSID);

    Serial.print("WIFI_PRIMARY_PASSWORD:");
    Serial.println(primaryPassword.length() > 0 ? "SET" : "EMPTY");

    Serial.print("WIFI_FALLBACK_SSID:");
    Serial.println(fallbackSSID);

    Serial.print("WIFI_FALLBACK_PASSWORD:");
    Serial.println(fallbackPassword.length() > 0 ? "SET" : "EMPTY");
}

bool connectWiFiSimple(const char* ssid, const char* password, const char* targetIP, const char* label)
{
    Serial.println();
    Serial.print("WIFI_");
    Serial.print(label);
    Serial.println("_CONNECT_START");

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.disconnect(true, true);
    delay(100);

    Serial.print("WIFI_BEGIN:");
    Serial.println(ssid);

    if (password != nullptr && password[0] != '\0')
        WiFi.begin(ssid, password);
    else
        WiFi.begin(ssid);

    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 20)
    {
        delay(500);
        Serial.print(".");
        timeout++;
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println();
        Serial.print("WIFI_");
        Serial.print(label);
        Serial.print("_FAILED_STATUS:");
        Serial.println((int)WiFi.status());
        return false;
    }

    Serial.println();
    Serial.println("WIFI_CONNECTED");
    Serial.print("WIFI_IP:");
    Serial.println(WiFi.localIP());
    Serial.print("WIFI_GATEWAY:");
    Serial.println(WiFi.gatewayIP());
    Serial.print("WIFI_RSSI:");
    Serial.println(WiFi.RSSI());

    telemetryTargetIP.fromString(targetIP);

    if (telemetryUDP.begin(UDP_PORT))
    {
        Serial.print("WIFI_UDP_READY:");
        Serial.println(UDP_PORT);
    }
    else
    {
        Serial.println("WIFI_UDP_FAILED");
    }

    return true;
}

void connectToWiFi()
{
    Serial.println("WIFI_CONNECT_START");

    wifiConnected = false;
    usingFallbackWiFi = false;
    telemetryTargetIP = IPAddress(0, 0, 0, 0);

    // Primary: exact known-good strategy.
    if (connectWiFiSimple(
            DRIVE_MATRIX_SSID,
            DRIVE_MATRIX_PASSWORD,
            "10.42.0.1",
            "DRIVEMATRIX"))
    {
        wifiConnected = true;
        usingFallbackWiFi = false;

        if (MDNS.begin("drivematrix-rx"))
            Serial.println("RX_MDNS:STARTED");
        else
            Serial.println("RX_MDNS:FAILED");

        return;
    }

    Serial.println("WIFI_PRIMARY_FAILED_TRY_FALLBACK");

    // Fallback: Raspberry Pi AP. The AP password comes from the
    // already-supported NVS provisioning mechanism.
    loadWiFiFallbackCredentials();

    Serial.print("WIFI_FALLBACK_SSID:");
    Serial.println(fallbackSSID);

    if (connectWiFiSimple(
            fallbackSSID.c_str(),
            fallbackPassword.c_str(),
            FALLBACK_TARGET_IP,
            "FALLBACK"))
    {
        wifiConnected = true;
        usingFallbackWiFi = true;

        if (MDNS.begin("drivematrix-rx"))
            Serial.println("RX_MDNS:STARTED");
        else
            Serial.println("RX_MDNS:FAILED");

        return;
    }

    Serial.println("WIFI_CONNECT:FAILED_ALL_PROFILES");
}

// ============================================================
// nRF24 PINS
// ============================================================

#define NRF_CE_PIN    42
#define NRF_CSN_PIN   39
#define NRF_SCK_PIN   12
#define NRF_MOSI_PIN  11
#define NRF_MISO_PIN  13

SPIClass nrfSPI(FSPI);

RF24 radio(
    NRF_CE_PIN,
    NRF_CSN_PIN,
    500000
);

// ============================================================
// nRF24 ADDRESSES
// ============================================================

// Shared discovery address.
const byte DISCOVERY_ADDRESS[6] = "DISC1";

// Reply address used by RX during discovery.
const byte REPLY_ADDRESS[6] = "REPLY";

// ============================================================
// ESP32 FACTORY IDENTITY
// ============================================================

char receiverID[13];


// ============================================================
// DISCOVERY TIMING
// ============================================================

#define DISCOVERY_BASE_DELAY_MS 50
#define DISCOVERY_SLOT_MS       100
#define DISCOVERY_SLOT_COUNT    10

// ============================================================
// READ FACTORY eFuse MAC
// ============================================================

bool readReceiverMAC()
{
    uint64_t chipId =
        ESP.getEfuseMac();

    if (chipId == 0)
    {
        receiverID[0] = '\0';

        return false;
    }

    snprintf(
        receiverID,
        sizeof(receiverID),
        "%04X%08X",
        (uint16_t)(chipId >> 32),
        (uint32_t)chipId
    );

    return true;
}

// ============================================================
// PRINT IDENTITY
// ============================================================

void printReceiverIdentity()
{
    uint64_t chipId =
        ESP.getEfuseMac();

    Serial.println();
    Serial.println(
        "ESP32 FACTORY IDENTITY"
    );

    Serial.println(
        "------------------------------"
    );

    if (chipId != 0)
    {
        uint8_t mac[6];

        mac[0] =
            (uint8_t)(chipId >> 40);

        mac[1] =
            (uint8_t)(chipId >> 32);

        mac[2] =
            (uint8_t)(chipId >> 24);

        mac[3] =
            (uint8_t)(chipId >> 16);

        mac[4] =
            (uint8_t)(chipId >> 8);

        mac[5] =
            (uint8_t)(chipId);

        Serial.printf(
            "MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
            mac[0],
            mac[1],
            mac[2],
            mac[3],
            mac[4],
            mac[5]
        );

        Serial.print(
            "RX ID: "
        );

        Serial.println(
            receiverID
        );
    }
    else
    {
        Serial.println(
            "MAC: READ FAILED"
        );

        Serial.println(
            "RX ID: READ FAILED"
        );
    }

    Serial.print(
        "Vehicle Name: "
    );

    Serial.println(
        vehicleName
    );

    Serial.println(
        "------------------------------"
    );
}

// ============================================================
// PRINT RF ADDRESS
// ============================================================

void printAddress(
    const byte address[6]
)
{
    for (int i = 0; i < 5; i++)
    {
        if (address[i] < 0x10)
        {
            Serial.print("0");
        }

        Serial.print(
            address[i],
            HEX
        );

        if (i < 4)
        {
            Serial.print(":");
        }
    }
}

// ============================================================
// BUILD UNIQUE CONTROL ADDRESS
// ============================================================

bool buildControlAddress(
    byte address[6]
)
{
    // RX RF ID must contain exactly 12 hexadecimal characters.
    if (strlen(receiverID) != 12)
    {
        return false;
    }

    // Validate the complete RX RF ID.
    for (int i = 0; i < 12; i++)
    {
        char c = receiverID[i];

        bool valid =
            (c >= '0' && c <= '9') ||
            (c >= 'A' && c <= 'F') ||
            (c >= 'a' && c <= 'f');

        if (!valid)
        {
            return false;
        }
    }

    // FNV-1a hash of the COMPLETE RX RF ID.
    uint32_t hash = 2166136261UL;

    for (int i = 0; i < 12; i++)
    {
        hash ^= (uint8_t)receiverID[i];
        hash *= 16777619UL;
    }

    // nRF24 uses a 5-byte address.
    // Fixed DriveMatrix prefix + 32-bit hash.
    address[0] = 0xD3;
    address[1] = (byte)(hash >> 24);
    address[2] = (byte)(hash >> 16);
    address[3] = (byte)(hash >> 8);
    address[4] = (byte)(hash);

    return true;
}

// ============================================================
// CALCULATE DISCOVERY DELAY
// ============================================================

unsigned long calculateDiscoveryDelay()
{
    /*
     * Generate a deterministic hash from the permanent
     * receiver ID.
     *
     * Every RX therefore gets a repeatable discovery slot.
     */

    uint32_t hash =
        2166136261UL;

    for (
        int i = 0;
        receiverID[i] != '\0';
        i++
    )
    {
        hash ^=
            (uint8_t)receiverID[i];

        hash *=
            16777619UL;
    }

    uint8_t slot =
        hash % DISCOVERY_SLOT_COUNT;

    unsigned long delayMs =
        DISCOVERY_BASE_DELAY_MS +
        (
            (unsigned long)slot *
            DISCOVERY_SLOT_MS
        );

    return delayMs;
}

// ============================================================
// CONFIGURE LISTENING PIPES
// ============================================================

void configureListening()
{
    byte controlAddress[6];

    // --------------------------------------------------------
    // Pipe 0 = shared discovery
    // --------------------------------------------------------

    radio.openReadingPipe(
        0,
        DISCOVERY_ADDRESS
    );

    // --------------------------------------------------------
    // Pipe 1 = unique control
    // --------------------------------------------------------

    if (
        buildControlAddress(
            controlAddress
        )
    )
    {
        radio.openReadingPipe(
            1,
            controlAddress
        );

        Serial.print(
            "Control address: "
        );

        printAddress(
            controlAddress
        );

        Serial.println();
    }
    else
    {
        Serial.println(
            "Control address: FAILED"
        );
    }

    radio.startListening();
}

// ============================================================
// SEND DISCOVERY RESPONSE
// ============================================================

void sendDiscoveryResponse()
{
    DiscoveryResponse response;

    memset(
        &response,
        0,
        sizeof(response)
    );

    strncpy(
        response.receiverID,
        receiverID,
        sizeof(response.receiverID) - 1
    );

    strncpy(
        response.vehicleName,
        vehicleName,
        sizeof(response.vehicleName) - 1
    );

    // --------------------------------------------------------
    // Calculate deterministic discovery delay
    // --------------------------------------------------------

    unsigned long discoveryDelay =
        calculateDiscoveryDelay();

    Serial.println();
    Serial.println(
        "DISCOVERY REQUEST RECEIVED"
    );

    Serial.print(
        "RX ID: "
    );

    Serial.println(
        response.receiverID
    );

    Serial.print(
        "Vehicle Name: "
    );

    Serial.println(
        response.vehicleName
    );

    Serial.print(
        "Discovery delay: "
    );

    Serial.print(
        discoveryDelay
    );

    Serial.println(
        " ms"
    );

    // --------------------------------------------------------
    // Wait before response.
    // --------------------------------------------------------

    delay(
        discoveryDelay
    );

    // --------------------------------------------------------
    // Temporarily switch to transmit mode.
    // --------------------------------------------------------

    radio.stopListening();

    radio.openWritingPipe(
        REPLY_ADDRESS
    );

    bool success =
        radio.write(
            &response,
            sizeof(response)
        );

    Serial.print(
        "DISCOVERY RESPONSE: "
    );

    Serial.println(
        success
        ? "SENT"
        : "FAILED"
    );

    // --------------------------------------------------------
    // Return to normal listening.
    // --------------------------------------------------------

    configureListening();
}


// ============================================================
// WI-FI PROVISIONING RECEIVE STATE
// ============================================================

String primarySSIDBuffer;
String primaryPasswordBuffer;
String fallbackSSIDBuffer;
String fallbackPasswordBuffer;

uint8_t primarySSIDExpectedChunks = 0;
uint8_t primaryPasswordExpectedChunks = 0;
uint8_t fallbackSSIDExpectedChunks = 0;
uint8_t fallbackPasswordExpectedChunks = 0;

void resetWiFiProvisionBuffers()
{
    primarySSIDBuffer = "";
    primaryPasswordBuffer = "";
    fallbackSSIDBuffer = "";
    fallbackPasswordBuffer = "";

    primarySSIDExpectedChunks = 0;
    primaryPasswordExpectedChunks = 0;
    fallbackSSIDExpectedChunks = 0;
    fallbackPasswordExpectedChunks = 0;
}

bool saveProvisionedWiFiProfiles()
{
    if (primarySSIDBuffer.length() == 0 || primarySSIDBuffer.length() > 32)
    {
        Serial.println("WIFI_SAVE_ERROR:PRIMARY_SSID");
        return false;
    }

    if (primaryPasswordBuffer.length() > 63)
    {
        Serial.println("WIFI_SAVE_ERROR:PRIMARY_PASSWORD");
        return false;
    }

    if (fallbackSSIDBuffer.length() == 0 || fallbackSSIDBuffer.length() > 32)
    {
        Serial.println("WIFI_SAVE_ERROR:FALLBACK_SSID");
        return false;
    }

    if (fallbackPasswordBuffer.length() > 63)
    {
        Serial.println("WIFI_SAVE_ERROR:FALLBACK_PASSWORD");
        return false;
    }

    wifiPrefs.begin("wifi", false);

    wifiPrefs.putString("primary_ssid", primarySSIDBuffer);
    wifiPrefs.putString("primary_pass", primaryPasswordBuffer);
    wifiPrefs.putString("fallback_ssid", fallbackSSIDBuffer);
    wifiPrefs.putString("fallback_pass", fallbackPasswordBuffer);

    // Keep the legacy keys synchronized for backward compatibility.
    wifiPrefs.putString("ssid", primarySSIDBuffer);
    wifiPrefs.putString("pass", primaryPasswordBuffer);

    wifiPrefs.end();

    primarySSID = primarySSIDBuffer;
    primaryPassword = primaryPasswordBuffer;
    fallbackSSID = fallbackSSIDBuffer;
    fallbackPassword = fallbackPasswordBuffer;

    Serial.println("WIFI_PROVISION_SAVED");
    return true;
}

void handleWiFiProvisionPacket()
{
    WiFiProvisionPacket packet;
    memset(&packet, 0, sizeof(packet));
    radio.read(&packet, sizeof(packet));

    if (packet.type != PACKET_WIFI_PROVISION)
    {
        Serial.println("WIFI_RX_INVALID_TYPE");
        return;
    }

    if (packet.chunkTotal == 0 || packet.chunkIndex >= packet.chunkTotal)
    {
        Serial.println("WIFI_RX_INVALID_CHUNK");
        return;
    }

    String *buffer = nullptr;
    uint8_t *expectedChunks = nullptr;

    if (packet.field == 1)
    {
        if (packet.chunkIndex == 0) primarySSIDBuffer = "";
        buffer = &primarySSIDBuffer;
        expectedChunks = &primarySSIDExpectedChunks;
    }
    else if (packet.field == 2)
    {
        if (packet.chunkIndex == 0) primaryPasswordBuffer = "";
        buffer = &primaryPasswordBuffer;
        expectedChunks = &primaryPasswordExpectedChunks;
    }
    else if (packet.field == 4)
    {
        if (packet.chunkIndex == 0) fallbackSSIDBuffer = "";
        buffer = &fallbackSSIDBuffer;
        expectedChunks = &fallbackSSIDExpectedChunks;
    }
    else if (packet.field == 5)
    {
        if (packet.chunkIndex == 0) fallbackPasswordBuffer = "";
        buffer = &fallbackPasswordBuffer;
        expectedChunks = &fallbackPasswordExpectedChunks;
    }
    else if (packet.field == 3)
    {
        Serial.println("WIFI_COMMIT_RECEIVED");

        if (primarySSIDExpectedChunks == 0 ||
            primaryPasswordExpectedChunks == 0 ||
            fallbackSSIDExpectedChunks == 0 ||
            fallbackPasswordExpectedChunks == 0)
        {
            Serial.println("WIFI_SAVE_ERROR:INCOMPLETE");
            return;
        }

        saveProvisionedWiFiProfiles();
        return;
    }
    else
    {
        Serial.println("WIFI_RX_UNKNOWN_FIELD");
        return;
    }

    if (packet.chunkIndex == 0)
    {
        *expectedChunks = packet.chunkTotal;
    }

    if (*expectedChunks != packet.chunkTotal)
    {
        Serial.println("WIFI_RX_CHUNK_MISMATCH");
        return;
    }

    if (packet.chunkIndex != 0)
    {
        uint8_t expectedIndex = (uint8_t)(buffer->length() / 28);
        if (packet.chunkIndex != expectedIndex)
        {
            Serial.println("WIFI_RX_OUT_OF_ORDER");
            return;
        }
    }

    uint8_t dataLength = 0;
    while (dataLength < sizeof(packet.data) && packet.data[dataLength] != '\0')
    {
        dataLength++;
    }

    for (uint8_t i = 0; i < dataLength; i++)
    {
        buffer->concat(packet.data[i]);
    }

    Serial.print("WIFI_RX:");
    Serial.print(packet.field);
    Serial.print(":");
    Serial.print(packet.chunkIndex + 1);
    Serial.print("/");
    Serial.println(packet.chunkTotal);
}

// ============================================================
// HANDLE DISCOVERY PACKET
// ============================================================

void handleDiscoveryPacket()
{
    DiscoveryPacket request;

    radio.read(
        &request,
        sizeof(request)
    );

    // --------------------------------------------------------
    // Ensure safe string termination.
    // --------------------------------------------------------

    char command[11];

    memcpy(
        command,
        request.cmd,
        10
    );

    command[10] = '\0';

    Serial.print(
        "Discovery command: "
    );

    Serial.println(
        command
    );

    if (
        strcmp(
            command,
            "DISCOVER"
        ) == 0
    )
    {
        sendDiscoveryResponse();
    }
    else
    {
        Serial.println(
            "Unknown discovery command"
        );
    }
}

// ============================================================
// HANDLE SESSION PACKET
// ============================================================

// ============================================================
// HANDLE SESSION PACKET
// ============================================================

void handleSessionPacket()
{
    SessionPacket packet;

    memset(
        &packet,
        0,
        sizeof(packet)
    );

    radio.read(
        &packet,
        sizeof(packet)
    );

    if (packet.type != PACKET_SESSION)
    {
        Serial.println(
            "SESSION_INVALID_TYPE"
        );

        return;
    }

    // --------------------------------------------------------
    // SESSION STOP
    // --------------------------------------------------------

    if (packet.active == 0)
    {
        sessionActive = false;

        sessionEndTime = 0;

        Vehicle_Failsafe();

        Serial.println(
            "SESSION_STOPPED"
        );

        return;
    }

    // --------------------------------------------------------
    // SESSION START VALIDATION
    // --------------------------------------------------------

    if (packet.durationSeconds == 0)
    {
        sessionActive = false;

        sessionEndTime = 0;

        Vehicle_Failsafe();

        Serial.println(
            "SESSION_INVALID"
        );

        return;
    }

    // --------------------------------------------------------
    // START / RESET SESSION
    // --------------------------------------------------------

    sessionActive = true;

    sessionEndTime =
        millis() +
        (
            packet.durationSeconds *
            1000UL
        );

    // Reset the control watchdog when a new session starts.
    lastControlTime = millis();

    Serial.print(
        "SESSION_STARTED:"
    );

    Serial.print(
        packet.durationSeconds
    );

    Serial.println(
        "s"
    );
}

// ============================================================
// HANDLE LAP PACKET
// ============================================================

void handleLapPacket()
{
    LapPacket packet;

    memset(
        &packet,
        0,
        sizeof(packet)
    );

    radio.read(
        &packet,
        sizeof(packet)
    );

    if (packet.type != PACKET_LAP)
    {
        Serial.println(
            "LAP_INVALID_TYPE"
        );

        return;
    }

    ascentLapCount = packet.lapCount;
    ascentLastLapMs = packet.lastLapMs;
    ascentBestLapMs = packet.bestLapMs;

    Serial.print("LAP_RECEIVED:");
    Serial.print(ascentLapCount);
    Serial.print(",");
    Serial.print(ascentLastLapMs);
    Serial.print(",");
    Serial.println(ascentBestLapMs);
}

// ============================================================
// HANDLE CONTROL PACKET
// ============================================================

void handleControlPacket()
{
    ControlPacket packet;

    radio.read(
        &packet,
        sizeof(packet)
    );

    // --------------------------------------------------------
    // SESSION GATE
    // --------------------------------------------------------

    if (
        !sessionActive ||
        (
            (int32_t)(
                millis() -
                sessionEndTime
            ) >= 0
        )
    )
    {
        sessionActive = false;
        sessionEndTime = 0;

        Vehicle_Failsafe();

        Serial.println(
            "CONTROL_BLOCKED:NO_ACTIVE_SESSION"
        );

        return;
    }

    Serial.println();
    Serial.println(
        "CONTROL PACKET RECEIVED"
    );

    Serial.print(
        "Type:      "
    );

    Serial.println(
        packet.type
    );

    Serial.print(
        "Sequence:  "
    );

    Serial.println(
        packet.sequence
    );

    Serial.print(
        "Steering:  "
    );

    Serial.println(
        packet.steering
    );

    Serial.print(
        "Throttle:  "
    );

    Serial.println(
        packet.throttle
    );

    // --------------------------------------------------------
    // Original vehicle control path
    // --------------------------------------------------------

    lastControlTime = millis();

    Vehicle_SetSteering(
        packet.steering
    );

    Vehicle_SetThrottle(
        packet.throttle
    );

    // --------------------------------------------------------
    // Prepare ACK
    // --------------------------------------------------------

    RadioAckPacket ack;

    ack.type =
        PACKET_RADIO_ACK;

    ack.sequence =
        packet.sequence;

    ack.status =
        1;

    ack.failsafe =
        0;

    // --------------------------------------------------------
    // Queue ACK payload on control pipe.
    // --------------------------------------------------------

    bool queued =
        radio.writeAckPayload(
            1,
            &ack,
            sizeof(ack)
        );

    Serial.print(
        "ACK queued: "
    );

    Serial.println(
        queued
        ? "YES"
        : "NO"
    );

    Serial.print(
        "ACK sequence: "
    );

    Serial.println(
        ack.sequence
    );
}

// ============================================================
// ORIGINAL PACKET HANDLERS
// ============================================================

void handleSetNamePacket()
{
    SetNamePacket packet;
    memset(&packet, 0, sizeof(packet));
    radio.read(&packet, sizeof(packet));

    if (strcmp(packet.receiverID, receiverID) != 0) return;

    packet.vehicleName[sizeof(packet.vehicleName) - 1] = '\0';
    strncpy(vehicleName, packet.vehicleName, sizeof(vehicleName) - 1);
    vehicleName[sizeof(vehicleName) - 1] = '\0';

    prefs.putString("vehicleName", vehicleName);

    Serial.print("Vehicle Name Saved: ");
    Serial.println(vehicleName);
}

void handleConfigPacket()
{
    ConfigPacket packet;
    memset(&packet, 0, sizeof(packet));
    radio.read(&packet, sizeof(packet));

    switch (packet.item)
    {
        case CFG_STEERING_REVERSE: Vehicle_SetSteeringReverse(packet.value != 0); break;
        case CFG_STEERING_CENTER:  Vehicle_SetSteeringCenter(packet.value); break;
        case CFG_STEERING_LEFT:    Vehicle_SetSteeringLeft(packet.value); break;
        case CFG_STEERING_RIGHT:   Vehicle_SetSteeringRight(packet.value); break;
        case CFG_IMU_FORWARD_AXIS: imuConfig.forwardAxis = static_cast<Axis>(packet.value % 3); break;
        case CFG_IMU_RIGHT_AXIS:   imuConfig.rightAxis = static_cast<Axis>(packet.value % 3); break;
        case CFG_IMU_UP_AXIS:      imuConfig.upAxis = static_cast<Axis>(packet.value % 3); break;
        case CFG_IMU_FORWARD_SIGN: imuConfig.forwardSign = packet.value ? 1 : -1; break;
        case CFG_IMU_RIGHT_SIGN:   imuConfig.rightSign = packet.value ? 1 : -1; break;
        case CFG_IMU_UP_SIGN:      imuConfig.upSign = packet.value ? 1 : -1; break;
        default: return;
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );

    delay(1000);

    Serial.println();
    Serial.println(
        "=============================="
    );

    Serial.println(
        "DriveMatrix RX Protocol Test"
    );

    Serial.println(
        "=============================="
    );

    // --------------------------------------------------------
    // Factory MAC identity
    // --------------------------------------------------------

    if (
        readReceiverMAC()
    )
    {
        Serial.println(
            "ESP32 FACTORY MAC: OK"
        );
    }
    else
    {
        Serial.println(
            "ESP32 FACTORY MAC: FAILED"
        );
    }

    printReceiverIdentity();

    // This firmware image is specifically for Jeep Gray 1.
    const char* expectedRFID = "9C52F7020F3C";

    if (strcmp(receiverID, expectedRFID) != 0)
    {
        Serial.print("RX ID MISMATCH. Expected: ");
        Serial.println(expectedRFID);
        Serial.print("Actual: ");
        Serial.println(receiverID);

        Vehicle_Failsafe();

        while (1)
        {
            delay(1000);
        }
    }

    Serial.println("RX ID VERIFIED: Jeep Gray 2");

    // --------------------------------------------------------
    // Discovery delay
    // --------------------------------------------------------

    Serial.print(
        "Discovery delay: "
    );

    Serial.print(
        calculateDiscoveryDelay()
    );

    Serial.println(
        " ms"
    );

    // --------------------------------------------------------
    // SPI
    // --------------------------------------------------------

    pinMode(
        NRF_CSN_PIN,
        OUTPUT
    );

    digitalWrite(
        NRF_CSN_PIN,
        HIGH
    );

    nrfSPI.begin(
        NRF_SCK_PIN,
        NRF_MISO_PIN,
        NRF_MOSI_PIN,
        NRF_CSN_PIN
    );

    // --------------------------------------------------------
    // nRF24 initialization
    // --------------------------------------------------------

    Serial.println(
        "Initializing nRF24..."
    );

    if (
        !radio.begin(&nrfSPI)
    )
    {
        Serial.println(
            "NRF24: FAILED"
        );

        while (1)
        {
            delay(1000);
        }
    }

    Serial.println(
        "NRF24: OK"
    );

    // --------------------------------------------------------
    // Protocol sizes
    // --------------------------------------------------------

    Serial.print(
        "ControlPacket size: "
    );

    Serial.println(
        sizeof(ControlPacket)
    );

    Serial.print(
        "RadioAckPacket size: "
    );

    Serial.println(
        sizeof(RadioAckPacket)
    );

    Serial.print(
        "WiFiProvisionPacket size: "
    );

    Serial.println(
        sizeof(WiFiProvisionPacket)
    );

    Serial.print(
        "DiscoveryPacket size: "
    );

    Serial.println(
        sizeof(DiscoveryPacket)
    );

    Serial.print(
        "DiscoveryResponse size: "
    );

    Serial.println(
        sizeof(DiscoveryResponse)
    );

    Serial.print("TelemetryPacket size: ");
    Serial.println(sizeof(TelemetryPacket));

    Serial.print("SessionPacket size: ");
    Serial.println(sizeof(SessionPacket));

    // --------------------------------------------------------
    // nRF24 configuration
    // --------------------------------------------------------

    radio.setPALevel(
        RF24_PA_LOW
    );

    radio.setDataRate(
        RF24_250KBPS
    );

    radio.setChannel(
        76
    );

    radio.setAutoAck(
        true
    );

    radio.setRetries(
        5,
        15
    );

    radio.enableAckPayload();

    radio.enableDynamicPayloads();

    // --------------------------------------------------------
    // Open discovery + unique control pipes
    // --------------------------------------------------------

    configureListening();

    // --------------------------------------------------------
    // Ready
    // --------------------------------------------------------

    Serial.println(
        "ACK payload enabled"
    );

    Serial.println(
        "Discovery address: DISC1"
    );

    Serial.println(
        "Control address: UNIQUE"
    );

    Serial.println(
        "Reply address: REPLY"
    );

    Serial.println(
        "Listening for discovery + control..."
    );

    // Original vehicle/IMU/telemetry stack is initialized independently of Wi-Fi.
    Vehicle_Init();

    {
        String storedVehicleName = prefs.getString(
            "vehicleName",
            "Jeep Gray 2"
        );

        strncpy(
            vehicleName,
            storedVehicleName.c_str(),
            sizeof(vehicleName) - 1
        );

        vehicleName[sizeof(vehicleName) - 1] = '\0';
    }

    if (!IMU_Init())
    {
        Serial.println("IMU initialization failed - control remains available");
    }

    Telemetry_Init();
    lastControlTime = millis();

    // Ascent Lite OSD is initialized after the core vehicle,
    // IMU and telemetry stack, without changing the RF path.
    Ascent_Init();

    loadProvisionedWiFiCredentials();

    // Wi-Fi is secondary. Start its first attempt after the RF/vehicle stack is ready.
    wifiServiceNextAttempt = millis();
    wifiServiceStarted = false;
}

// ============================================================
// PIPE 1 PACKET DISPATCH
// ============================================================

// ============================================================
// PIPE 1 PACKET DISPATCH
// ============================================================

void handlePipe1Packet(
    uint8_t payloadSize
)
{

        // --------------------------------------------------------
    // LAP
    // --------------------------------------------------------

    if (
        payloadSize ==
        sizeof(LapPacket)
    )
    {
        handleLapPacket();

        return;
    }



    // --------------------------------------------------------
    // CONTROL
    // --------------------------------------------------------

    if (
        payloadSize ==
        sizeof(ControlPacket)
    )
    {
        handleControlPacket();

        return;
    }

    // --------------------------------------------------------
    // CONFIG
    // --------------------------------------------------------

    if (
        payloadSize ==
        sizeof(ConfigPacket)
    )
    {
        handleConfigPacket();

        return;
    }

    // --------------------------------------------------------
    // SESSION
    // --------------------------------------------------------

    if (
        payloadSize ==
        sizeof(SessionPacket)
    )
    {
        handleSessionPacket();

        return;
    }

    // --------------------------------------------------------
    // 32-BYTE PACKETS
    //
    // SetNamePacket and WiFiProvisionPacket are both
    // 32 bytes, so inspect packet type.
    // --------------------------------------------------------

    if (
        payloadSize != 32
    )
    {
        Serial.print(
            "INVALID_PAYLOAD_SIZE:"
        );

        Serial.println(
            payloadSize
        );

        radio.flush_rx();

        return;
    }

    uint8_t buffer[32];

    radio.read(
        buffer,
        sizeof(buffer)
    );

    uint8_t packetType =
        buffer[0];

    // --------------------------------------------------------
    // WIFI PROVISION
    // --------------------------------------------------------

    if (
        packetType ==
        PACKET_WIFI_PROVISION
    )
    {
        WiFiProvisionPacket packet;

        memcpy(
            &packet,
            buffer,
            sizeof(packet)
        );

        if (
            packet.type !=
            PACKET_WIFI_PROVISION
        )
        {
            return;
        }

        if (
            packet.chunkTotal == 0 ||
            packet.chunkIndex >=
                packet.chunkTotal
        )
        {
            Serial.println(
                "WIFI_RX_INVALID_CHUNK"
            );

            return;
        }

        String *targetBuffer = nullptr;

        uint8_t *expectedChunks =
            nullptr;

        if (
            packet.field == 1
        )
        {
            if (
                packet.chunkIndex == 0
            )
            {
                primarySSIDBuffer = "";
            }

            targetBuffer =
                &primarySSIDBuffer;

            expectedChunks =
                &primarySSIDExpectedChunks;
        }
        else if (
            packet.field == 2
        )
        {
            if (
                packet.chunkIndex == 0
            )
            {
                primaryPasswordBuffer = "";
            }

            targetBuffer =
                &primaryPasswordBuffer;

            expectedChunks =
                &primaryPasswordExpectedChunks;
        }
        else if (
            packet.field == 4
        )
        {
            if (
                packet.chunkIndex == 0
            )
            {
                fallbackSSIDBuffer = "";
            }

            targetBuffer =
                &fallbackSSIDBuffer;

            expectedChunks =
                &fallbackSSIDExpectedChunks;
        }
        else if (
            packet.field == 5
        )
        {
            if (
                packet.chunkIndex == 0
            )
            {
                fallbackPasswordBuffer = "";
            }

            targetBuffer =
                &fallbackPasswordBuffer;

            expectedChunks =
                &fallbackPasswordExpectedChunks;
        }
        else if (
            packet.field == 3
        )
        {
            Serial.println(
                "WIFI_COMMIT_RECEIVED"
            );

            if (
                primarySSIDExpectedChunks == 0 ||
                primaryPasswordExpectedChunks == 0 ||
                fallbackSSIDExpectedChunks == 0 ||
                fallbackPasswordExpectedChunks == 0
            )
            {
                Serial.println(
                    "WIFI_SAVE_ERROR:INCOMPLETE"
                );

                return;
            }

            saveProvisionedWiFiProfiles();

            return;
        }
        else
        {
            Serial.println(
                "WIFI_RX_UNKNOWN_FIELD"
            );

            return;
        }

        if (
            packet.chunkIndex == 0
        )
        {
            *expectedChunks =
                packet.chunkTotal;
        }

        if (
            *expectedChunks !=
            packet.chunkTotal
        )
        {
            Serial.println(
                "WIFI_RX_CHUNK_MISMATCH"
            );

            return;
        }

        if (
            packet.chunkIndex != 0
        )
        {
            uint8_t expectedIndex =
                (
                    uint8_t
                )(
                    targetBuffer->length() /
                    28
                );

            if (
                packet.chunkIndex !=
                expectedIndex
            )
            {
                Serial.println(
                    "WIFI_RX_OUT_OF_ORDER"
                );

                return;
            }
        }

        uint8_t dataLength = 0;

        while (
            dataLength <
                sizeof(packet.data) &&
            packet.data[dataLength] !=
                '\0'
        )
        {
            dataLength++;
        }

        for (
            uint8_t i = 0;
            i < dataLength;
            i++
        )
        {
            targetBuffer->concat(
                packet.data[i]
            );
        }

        Serial.print(
            "WIFI_RX:"
        );

        Serial.print(
            packet.field
        );

        Serial.print(
            ":"
        );

        Serial.print(
            packet.chunkIndex + 1
        );

        Serial.print(
            "/"
        );

        Serial.println(
            packet.chunkTotal
        );

        return;
    }

    // --------------------------------------------------------
    // SET VEHICLE NAME
    // --------------------------------------------------------

    if (
        packetType ==
        PACKET_SET_NAME
    )
    {
        SetNamePacket packet;

        memcpy(
            &packet,
            buffer,
            sizeof(packet)
        );

        if (
            strcmp(
                packet.receiverID,
                receiverID
            ) != 0
        )
        {
            return;
        }

        packet.vehicleName[
            sizeof(packet.vehicleName) - 1
        ] = '\0';

        strncpy(
            vehicleName,
            packet.vehicleName,
            sizeof(vehicleName) - 1
        );

        vehicleName[
            sizeof(vehicleName) - 1
        ] = '\0';

        prefs.putString(
            "vehicleName",
            vehicleName
        );

        Serial.print(
            "Vehicle Name Saved: "
        );

        Serial.println(
            vehicleName
        );

        return;
    }

    // --------------------------------------------------------
    // UNKNOWN
    // --------------------------------------------------------

    Serial.print(
        "UNKNOWN_PACKET_TYPE:"
    );

    Serial.println(
        packetType
    );
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // nRF24 remains the primary control path.
    // Process every available RF packet without depending on Wi-Fi.
    // --------------------------------------------------------
    while (radio.available())
    {
        uint8_t pipeNumber;
        if (!radio.available(&pipeNumber)) break;

        if (pipeNumber == 0)
        {
            handleDiscoveryPacket();
            continue;
        }

        if (pipeNumber == 1)
        {
            uint8_t payloadSize = radio.getDynamicPayloadSize();
            handlePipe1Packet(payloadSize);
        }
        else
        {
            Serial.print("Unknown pipe: ");
            Serial.println(pipeNumber);
        }
    }

    // --------------------------------------------------------
    // RX session timer.
    // --------------------------------------------------------
    if (sessionActive && (int32_t)(millis() - sessionEndTime) >= 0)
    {
        sessionActive = false;
        sessionEndTime = 0;

        Vehicle_Failsafe();

        Serial.println("SESSION_EXPIRED");

        // Session expiry is the authoritative GAME OVER event.
        ascentGameState = ASCENT_GAME_OVER;
        ascentSessionEndTime = 0;
        ascentLastOverlayUpdate = millis();

        Ascent_DrawGameOver();

    

    }

    // --------------------------------------------------------
    // Vehicle failsafe is also independent of Wi-Fi.
    // --------------------------------------------------------
    if (millis() - lastControlTime > 1000)
    {
        Vehicle_Failsafe();
        lastControlTime = millis();
    }

    // --------------------------------------------------------
    // Ascent Lite OSD is local and independent of Wi-Fi.
    // The displayed timer follows the RX session timer.
    // --------------------------------------------------------
    Ascent_Update();

    // --------------------------------------------------------
    // IMU + telemetry are local and continue without Wi-Fi.
    // --------------------------------------------------------
    IMU_Update();
    Telemetry_Update();

    // --------------------------------------------------------
    // UDP telemetry is best-effort only.
    // --------------------------------------------------------
    if (millis() - lastTelemetry >= 20)
    {
        lastTelemetry = millis();
        SendUDPTelemetry();
    }

    // Wi-Fi connection runs in a separate task so RF control never waits for it.
    if (!wifiServiceStarted && millis() >= wifiServiceNextAttempt)
    {
        wifiServiceStarted = true;
        xTaskCreate(
            [](void *parameter)
            {
                connectToWiFi();
                vTaskDelete(NULL);
            },
            "WiFiConnect",
            8192,
            nullptr,
            1,
            nullptr
        );
    }
}
