#include <Arduino.h>
#include "fm17660.h"
#include "rfid.h"

// ── Pin definitions ───────────────────────────────────────────────────────────
#define FM_CS_PIN   5

FM17660 fm(FM_CS_PIN);
RFID    rfid(fm);

// Target block — block 1 is in sector 0.
// NEVER write to block 0 (contains UID + manufacturer data, usually locked).
static const uint8_t TARGET_BLOCK = 1;

// Data to write — must be exactly 16 bytes
static const uint8_t WRITE_DATA[RFID_BLOCK_SIZE] = {
    'H','e','l','l','o',' ','F','M','1','7','6','6','0','!',0x00,0x00
};

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== FM17660 read/write example ===");

    if (!fm.begin())
    {
        Serial.println("[ERROR] FM17660 did not respond. Check wiring.");
        while (true) delay(1000);
    }

    if (!rfid.begin())
    {
        Serial.println("[ERROR] Failed to load ISO 14443A protocol.");
        while (true) delay(1000);
    }

    Serial.println("[OK] Ready. Hold a MIFARE Classic card near the antenna...\n");
}

void loop()
{
    UID uid;

    if (!rfid.readUID(uid))
    {
        delay(200);
        return;
    }

    uid.print(Serial);

    // ── Authenticate with default key A (FF FF FF FF FF FF) ──────────────────
    if (!rfid.authenticateDefault(TARGET_BLOCK, uid))
    {
        Serial.println("[ERROR] Authentication failed. Wrong key or wrong card type.");
        rfid.haltCard();
        delay(1000);
        return;
    }
    Serial.println("[OK] Authenticated");

    // ── Read block BEFORE write ───────────────────────────────────────────────
    uint8_t blockData[RFID_BLOCK_SIZE];

    if (rfid.readBlock(TARGET_BLOCK, blockData))
    {
        Serial.print("Block ");
        Serial.print(TARGET_BLOCK);
        Serial.print(" (before write): ");
        for (uint8_t i = 0; i < RFID_BLOCK_SIZE; i++)
        {
            if (blockData[i] < 0x10) Serial.print('0');
            Serial.print(blockData[i], HEX);
            Serial.print(' ');
        }
        Serial.println();
    }
    else
    {
        Serial.println("[ERROR] Read failed: " + fm.lastError());
    }

    // ── Write new data ────────────────────────────────────────────────────────
    if (rfid.writeBlock(TARGET_BLOCK, WRITE_DATA))
    {
        Serial.println("[OK] Write successful");
    }
    else
    {
        Serial.println("[ERROR] Write failed: " + fm.lastError());
        rfid.haltCard();
        delay(1000);
        return;
    }

    // ── Re-authenticate then read back to verify ──────────────────────────────
    if (rfid.authenticateDefault(TARGET_BLOCK, uid) &&
        rfid.readBlock(TARGET_BLOCK, blockData))
    {
        Serial.print("Block ");
        Serial.print(TARGET_BLOCK);
        Serial.print(" (after write):  ");
        for (uint8_t i = 0; i < RFID_BLOCK_SIZE; i++)
        {
            if (blockData[i] < 0x10) Serial.print('0');
            Serial.print(blockData[i], HEX);
            Serial.print(' ');
        }
        Serial.println();
    }
    else
    {
        Serial.println("[ERROR] Readback failed: " + fm.lastError());
    }

    rfid.haltCard();
    Serial.println();
    delay(2000);
}
