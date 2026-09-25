#include <EEPROM.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "DriveMatrixProtocol.h"

// ============================================================
// nRF24
// ============================================================

#define NRF_CE_PIN  10
#define NRF_CSN_PIN 9

RF24 radio(
    NRF_CE_PIN,
    NRF_CSN_PIN
);

// ============================================================
// FIXED RX PAIRING
// ============================================================

// This Nano is permanently paired to Jeep Gray 1.
// The RX RF ID is the unique vehicle identity.
const char PAIRED_RX_ID[] = "9C52F7020F3C";
const char VEHICLE_NAME[] = "Jeep Gray 2";

// ============================================================
// EEPROM RADIO ID
// ============================================================

#define EEPROM_MAGIC_ADDR 0
#define EEPROM_ID_ADDR    1
#define EEPROM_MAGIC      0xD7

struct RadioIdentity
{
    char id[10];
};

RadioIdentity identity;

// ============================================================
// TRANSMISSION STATE
// ============================================================

uint16_t txSequence = 0;

// ============================================================
// FIXED RECEIVER PAIRING STATE
// ============================================================

char pairedReceiverID[13] = "9C52F7020F3C";
bool pairingReady = false;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================

void sendControlPacket(
    int16_t steering,
    int16_t throttle
);

void handleCommand();

void printIdentity();

void printStatus();

void printPairing();

bool buildControlAddress(
    
    const char *rxID,
    byte address[6]
);

bool sendWiFiField(
    uint8_t field,
    const String &value
);

bool handleWiFiFieldCommand(
    uint8_t field,
    const String &command,
    const char *prefix
);

bool commitWiFiCredentials();

// ============================================================
// GENERATE RADIO ID
// ============================================================

void generateRadioID()
{
    unsigned long seed = 0;

    seed ^= analogRead(A0);
    seed ^= ((unsigned long)analogRead(A1) << 10);
    seed ^= ((unsigned long)analogRead(A2) << 20);
    seed ^= micros();

    randomSeed(seed);

    const char hex[] =
        "0123456789ABCDEF";

    identity.id[0] = 'R';
    identity.id[1] = 'F';
    identity.id[2] = '-';

    for (int i = 0; i < 6; i++)
    {
        identity.id[3 + i] =
            hex[random(0, 16)];
    }

    identity.id[9] = '\0';
}

// ============================================================
// LOAD RADIO ID
// ============================================================

void loadRadioID()
{
    uint8_t magic =
        EEPROM.read(
            EEPROM_MAGIC_ADDR
        );

    if (magic == EEPROM_MAGIC)
    {
        EEPROM.get(
            EEPROM_ID_ADDR,
            identity
        );

        if (
            identity.id[0] == 'R' &&
            identity.id[1] == 'F' &&
            identity.id[2] == '-'
        )
        {
            return;
        }
    }

    // No valid ID exists.
    generateRadioID();

    EEPROM.update(
        EEPROM_MAGIC_ADDR,
        EEPROM_MAGIC
    );

    EEPROM.put(
        EEPROM_ID_ADDR,
        identity
    );
}

// ============================================================
// PRINT RADIO ID
// ============================================================

void printIdentity()
{
    Serial.print(
        "RADIO_ID:"
    );

    Serial.println(
        identity.id
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
// BUILD UNIQUE CONTROL ADDRESS FROM RX ID
// ============================================================

bool buildControlAddress(
    const char *rxID,
    byte address[6]
)
{
    // RX ID must contain exactly 12 hexadecimal characters.
    if (strlen(rxID) != 12)
    {
        return false;
    }

    // Validate RX ID.
    for (int i = 0; i < 12; i++)
    {
        char c = rxID[i];

        bool valid =
            (c >= '0' && c <= '9') ||
            (c >= 'A' && c <= 'F') ||
            (c >= 'a' && c <= 'f');

        if (!valid)
        {
            return false;
        }
    }

    // --------------------------------------------------------
    // FNV-1a hash of the COMPLETE RX ID.
    // --------------------------------------------------------

    uint32_t hash = 2166136261UL;

    for (int i = 0; i < 12; i++)
    {
        hash ^= (uint8_t)rxID[i];
        hash *= 16777619UL;
    }

    // --------------------------------------------------------
    // nRF24 uses a 5-byte address.
    //
    // Fixed DriveMatrix prefix + 32-bit hash.
    // --------------------------------------------------------

    address[0] = 0xD3;
    address[1] = (byte)(hash >> 24);
    address[2] = (byte)(hash >> 16);
    address[3] = (byte)(hash >> 8);
    address[4] = (byte)(hash);

    return true;
}

// ============================================================
// INITIALIZE nRF24
// ============================================================

bool initializeRadio()
{
    if (!radio.begin())
    {
        return false;
    }

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

    radio.setPayloadSize(
        32
    );

    // --------------------------------------------------------
    // Fixed RX pairing
    // --------------------------------------------------------

    byte controlAddress[6];

    if (!buildControlAddress(PAIRED_RX_ID, controlAddress))
    {
        Serial.println("PAIRING_ERROR:INVALID_RF_ID");
        return false;
    }

    strncpy(
        pairedReceiverID,
        PAIRED_RX_ID,
        sizeof(pairedReceiverID) - 1
    );
    pairedReceiverID[sizeof(pairedReceiverID) - 1] = '\0';

    radio.openWritingPipe(controlAddress);
    radio.stopListening();

    pairingReady = true;

    Serial.print("PAIRED_RF_ID:");
    Serial.println(pairedReceiverID);

    Serial.print("VEHICLE:");
    Serial.println(VEHICLE_NAME);

    Serial.print("CONTROL_ADDRESS:");
    printAddress(controlAddress);
    Serial.println();

    return true;
}

// ============================================================
// STATUS
// ============================================================

void printStatus()
{
    printIdentity();

    Serial.print(
        "NRF24:"
    );

    Serial.println(
        radio.isChipConnected()
        ? "OK"
        : "ERROR"
    );

    printPairing();
}

// ============================================================
// PRINT FIXED PAIRING
// ============================================================

void printPairing()
{
    Serial.print("RF_ID:");
    Serial.println(pairedReceiverID);

    Serial.print("VEHICLE:");
    Serial.println(VEHICLE_NAME);

    byte controlAddress[6];

    if (buildControlAddress(pairedReceiverID, controlAddress))
    {
        Serial.print("CONTROL_ADDRESS:");
        printAddress(controlAddress);
        Serial.println();
    }
}

// ============================================================
// SEND CONTROL PACKET
// ============================================================

void sendControlPacket(
    int16_t steering,
    int16_t throttle
)
{
    // --------------------------------------------------------
    // Safety: require selected target
    // --------------------------------------------------------

    if (!pairingReady)
    {
        Serial.println(
            "TX_BLOCKED:NO_TARGET"
        );

        return;
    }

    ControlPacket packet;

    packet.type =
        PACKET_CONTROL;

    packet.sequence =
        txSequence++;

    packet.steering =
        steering;

    packet.throttle =
        throttle;

    bool success =
        radio.write(
            &packet,
            sizeof(packet)
        );

    if (!success)
    {
        Serial.print(
            "TX_FAIL:"
        );

        Serial.println(
            packet.sequence
        );

        return;
    }

    Serial.print(
        "TX_OK:"
    );

    Serial.println(
        packet.sequence
    );

    // --------------------------------------------------------
    // ACK payload
    // --------------------------------------------------------

    if (
        radio.isAckPayloadAvailable()
    )
    {
        RadioAckPacket ack;

        memset(
            &ack,
            0,
            sizeof(ack)
        );

        radio.read(
            &ack,
            sizeof(ack)
        );

        Serial.print(
            "ACK_TYPE:"
        );

        Serial.println(
            ack.type
        );

        Serial.print(
            "ACK_SEQUENCE:"
        );

        Serial.println(
            ack.sequence
        );

        Serial.print(
            "ACK_STATUS:"
        );

        Serial.println(
            ack.status
        );

        Serial.print(
            "ACK_FAILSAFE:"
        );

        Serial.println(
            ack.failsafe
        );

        if (
            ack.type ==
            PACKET_RADIO_ACK
        )
        {
            Serial.println(
                "ACK_VALID"
            );
        }
        else
        {
            Serial.println(
                "ACK_INVALID"
            );
        }
    }
    else
    {
        Serial.println(
            "ACK_NONE"
        );
    }
}

// ============================================================
// SEND WI-FI PROVISIONING FIELD
// ============================================================

bool sendWiFiField(
    uint8_t field,
    const String &value
)
{
    if (!pairingReady)
    {
        Serial.println("WIFI_BLOCKED:NO_TARGET");
        return false;
    }

    const uint8_t CHUNK_SIZE = 28;
    uint16_t length = (uint16_t)value.length();

    if (length > 255)
    {
        Serial.println("WIFI_ERROR:TOO_LONG");
        return false;
    }

    uint8_t total =
        (uint8_t)((length + CHUNK_SIZE - 1) / CHUNK_SIZE);

    if (total == 0)
    {
        total = 1;
    }

    for (uint8_t index = 0; index < total; index++)
    {
        WiFiProvisionPacket packet;
        memset(&packet, 0, sizeof(packet));

        packet.type = PACKET_WIFI_PROVISION;
        packet.field = field;
        packet.chunkIndex = index;
        packet.chunkTotal = total;

        uint16_t offset =
            (uint16_t)index * CHUNK_SIZE;

        uint16_t remaining =
            length - offset;

        uint8_t count =
            (remaining > CHUNK_SIZE)
            ? CHUNK_SIZE
            : (uint8_t)remaining;

        if (count > 0)
        {
            memcpy(
                packet.data,
                value.c_str() + offset,
                count
            );
        }

        if (!radio.write(&packet, sizeof(packet)))
        {
            Serial.print("WIFI_TX_FAIL:");
            Serial.println(index);
            return false;
        }

        Serial.print("WIFI_TX:");
        Serial.print(field);
        Serial.print(":");
        Serial.print(index + 1);
        Serial.print("/");
        Serial.println(total);

        delay(5);
    }

    return true;
}

// ============================================================
// HANDLE WI-FI FIELD COMMAND
// ============================================================

bool handleWiFiFieldCommand(
    uint8_t field,
    const String &command,
    const char *prefix
)
{
    String prefixText = String(prefix);

    if (!command.startsWith(prefixText))
    {
        Serial.println("WIFI_INVALID");
        return false;
    }

    String value =
        command.substring(prefixText.length());

    value.trim();

    if (field == 1 || field == 4)
    {
        if (value.length() == 0 || value.length() > 32)
        {
            Serial.println("WIFI_INVALID:SSID");
            return false;
        }
    }
    else if (field == 2 || field == 5)
    {
        if (value.length() > 63)
        {
            Serial.println("WIFI_INVALID:PASSWORD");
            return false;
        }
    }
    else
    {
        Serial.println("WIFI_INVALID:FIELD");
        return false;
    }

    return sendWiFiField(field, value);
}

// ============================================================
// SEND WI-FI COMMIT
// ============================================================

bool commitWiFiCredentials()
{
    if (!pairingReady)
    {
        Serial.println("WIFI_BLOCKED:NO_TARGET");
        return false;
    }

    WiFiProvisionPacket packet;
    memset(&packet, 0, sizeof(packet));

    packet.type = PACKET_WIFI_PROVISION;
    packet.field = 3;
    packet.chunkIndex = 0;
    packet.chunkTotal = 1;

    if (!radio.write(&packet, sizeof(packet)))
    {
        Serial.println("WIFI_TX_FAIL:COMMIT");
        return false;
    }

    Serial.println("WIFI_COMMIT_SENT");
    return true;
}

// ============================================================
// SEND SESSION PACKET
// ============================================================

bool sendSessionPacket(bool active, uint32_t durationSeconds)
{
    if (!pairingReady)
    {
        Serial.println("SESSION_BLOCKED:NO_TARGET");
        return false;
    }

    SessionPacket packet;
    memset(&packet, 0, sizeof(packet));

    packet.type = PACKET_SESSION;
    packet.active = active ? 1 : 0;
    packet.durationSeconds = durationSeconds;

    if (!radio.write(&packet, sizeof(packet)))
    {
        Serial.println("SESSION_TX_FAIL");
        return false;
    }

    if (active)
    {
        Serial.print("SESSION_START_SENT:");
        Serial.println(durationSeconds);
    }
    else
    {
        Serial.println("SESSION_STOP_SENT");
    }

    return true;
}

bool sendLapPacket(
    uint16_t lapCount,
    uint32_t lastLapMs,
    uint32_t bestLapMs
)
{
    if (!pairingReady)
    {
        Serial.println("LAP_BLOCKED:NO_TARGET");
        return false;
    }

    LapPacket packet;
    memset(&packet, 0, sizeof(packet));

    packet.type = PACKET_LAP;
    packet.lapCount = lapCount;
    packet.lastLapMs = lastLapMs;
    packet.bestLapMs = bestLapMs;

    if (!radio.write(&packet, sizeof(packet)))
    {
        Serial.println("LAP_TX_FAIL");
        return false;
    }

    Serial.print("LAP_SENT:");
    Serial.print(lapCount);
    Serial.print(",");
    Serial.print(lastLapMs);
    Serial.print(",");
    Serial.println(bestLapMs);

    return true;
}

// ============================================================
// HANDLE SERIAL COMMAND
// ============================================================

void handleCommand()
{
    if (!Serial.available())
    {
        return;
    }

    String command =
        Serial.readStringUntil(
            '\n'
        );

    command.trim();

    // --------------------------------------------------------
    // WHO
    // --------------------------------------------------------

    if (
        command == "WHO"
    )
    {
        printIdentity();

        return;
    }

    // --------------------------------------------------------
    // PING
    // --------------------------------------------------------

    if (
        command == "PING"
    )
    {
        Serial.println(
            "PONG"
        );

        return;
    }

    // --------------------------------------------------------
    // STATUS
    // --------------------------------------------------------

    if (
        command == "STATUS"
    )
    {
        printStatus();

        return;
    }

    // --------------------------------------------------------
    // FIXED PAIRING STATUS
    // --------------------------------------------------------

    if (command == "PAIRING")
    {
        printPairing();
        return;
    }

    // --------------------------------------------------------
    // CONTROL
    //
    // CONTROL,steering,throttle
    //
    // Example:
    // CONTROL,1000,2000
    // --------------------------------------------------------

    if (
        command.startsWith(
            "CONTROL,"
        )
    )
    {
        int firstComma =
            command.indexOf(',');

        int secondComma =
            command.indexOf(
                ',',
                firstComma + 1
            );

        if (
            firstComma < 0 ||
            secondComma < 0
        )
        {
            Serial.println(
                "CONTROL_INVALID"
            );

            return;
        }

        String steeringText =
            command.substring(
                firstComma + 1,
                secondComma
            );

        String throttleText =
            command.substring(
                secondComma + 1
            );

        steeringText.trim();
        throttleText.trim();

        long steeringValue =
            steeringText.toInt();

        long throttleValue =
            throttleText.toInt();

        // ----------------------------------------------------
        // Clamp to signed 16-bit
        // ----------------------------------------------------

        steeringValue =
            constrain(
                steeringValue,
                -32768L,
                32767L
            );

        throttleValue =
            constrain(
                throttleValue,
                -32768L,
                32767L
            );

        sendControlPacket(
            (int16_t)steeringValue,
            (int16_t)throttleValue
        );

        return;
    }





    // --------------------------------------------------------
    // SESSION_START,<seconds>
    // SESSION_STOP
    // --------------------------------------------------------

    if (command.startsWith("SESSION_START,"))
    {
        String durationText = command.substring(14);
        durationText.trim();

        unsigned long durationSeconds = durationText.toInt();

        if (durationSeconds == 0)
        {
            Serial.println("SESSION_INVALID");
            return;
        }

        sendSessionPacket(true, (uint32_t)durationSeconds);
        return;
    }
        // --------------------------------------------------------
        // LAP
        //
        // LAP,<lapCount>,<lastLapMs>,<bestLapMs>
        //
        // Example:
        // LAP,3,18420,17850
        // --------------------------------------------------------

        if (command.startsWith("LAP,"))
        {
            int comma1 = command.indexOf(',');
            int comma2 = command.indexOf(',', comma1 + 1);
            int comma3 = command.indexOf(',', comma2 + 1);

            if (
                comma1 < 0 ||
                comma2 < 0 ||
                comma3 < 0
            )
            {
                Serial.println("LAP_INVALID");
                return;
            }

            String lapCountText =
                command.substring(
                    comma1 + 1,
                    comma2
                );

            String lastLapText =
                command.substring(
                    comma2 + 1,
                    comma3
                );

            String bestLapText =
                command.substring(
                    comma3 + 1
                );

            lapCountText.trim();
            lastLapText.trim();
            bestLapText.trim();

            uint16_t lapCount =
                (uint16_t)lapCountText.toInt();

            uint32_t lastLapMs =
                (uint32_t)lastLapText.toInt();

            uint32_t bestLapMs =
                (uint32_t)bestLapText.toInt();

            sendLapPacket(
                lapCount,
                lastLapMs,
                bestLapMs
            );

            return;
        }


    

    if (command == "SESSION_STOP")
    {
        sendSessionPacket(false, 0);
        return;
    }

    // --------------------------------------------------------
    // WIFI_SSID,<SSID>
    // WIFI_PASSWORD,<PASSWORD>
    // WIFI_FALLBACK_SSID,<SSID>
    // WIFI_FALLBACK_PASSWORD,<PASSWORD>
    // WIFI_COMMIT
    // --------------------------------------------------------

    if (command.startsWith("WIFI_SSID,"))
    {
        handleWiFiFieldCommand(1, command, "WIFI_SSID,");
        return;
    }

    if (command.startsWith("WIFI_PASSWORD,"))
    {
        handleWiFiFieldCommand(2, command, "WIFI_PASSWORD,");
        return;
    }

    if (command.startsWith("WIFI_FALLBACK_SSID,"))
    {
        handleWiFiFieldCommand(4, command, "WIFI_FALLBACK_SSID,");
        return;
    }

    if (command.startsWith("WIFI_FALLBACK_PASSWORD,"))
    {
        handleWiFiFieldCommand(5, command, "WIFI_FALLBACK_PASSWORD,");
        return;
    }

    if (command == "WIFI_COMMIT")
    {
        commitWiFiCredentials();
        return;
    }

    // --------------------------------------------------------
    // TEST
    // --------------------------------------------------------

    if (
        command == "TEST"
    )
    {
        sendControlPacket(
            1234,
            5678
        );

        return;
    }

    // --------------------------------------------------------
    // Unknown
    // --------------------------------------------------------

    Serial.print(
        "UNKNOWN:"
    );

    Serial.println(
        command
    );
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );

    delay(500);

    loadRadioID();

    Serial.println();
    Serial.println(
        "=============================="
    );

    Serial.println(
        "DriveMatrix Nano Radio"
    );

    Serial.println(
        "=============================="
    );

    printIdentity();

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

    Serial.print("Paired RF ID: ");
    Serial.println(PAIRED_RX_ID);

    Serial.print("Vehicle: ");
    Serial.println(VEHICLE_NAME);

    Serial.println(
        "Initializing nRF24..."
    );

    if (
        initializeRadio()
    )
    {
        Serial.println(
            "NRF24:OK"
        );

        Serial.println(
            "READY"
        );
    }
    else
    {
        Serial.println(
            "NRF24:ERROR"
        );

        while (1)
        {
            delay(1000);
        }
    }
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    handleCommand();
}