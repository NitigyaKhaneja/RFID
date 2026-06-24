#include <Arduino.h>
#include "fm17660.h"
#include "rfid.h"

// ── Pin definitions (adjust to your wiring) ───────────────────────────────────
//   ESP32 default SPI: SCK=18, MISO=19, MOSI=23, CS=5
//   If you use non-default pins, pass them to FM17660(cs, sck, mosi, miso).
#define FM_CS_PIN   5

FM17660 fm(FM_CS_PIN);
RFID    rfid(fm);

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== FM17660 UID reader ===");

    if (!fm.begin())
    {
        Serial.println("[ERROR] FM17660 did not respond. Check wiring.");
        while (true) delay(1000);
    }
    Serial.println("[OK] FM17660 initialised");

    if (!rfid.begin())
    {
        Serial.println("[ERROR] Failed to load ISO 14443A protocol.");
        while (true) delay(1000);
    }
    Serial.println("[OK] ISO 14443A protocol loaded");
    Serial.println("Hold a MIFARE card near the antenna...\n");
}

void loop()
{
    UID uid;

    if (rfid.readUID(uid))
    {
        uid.print(Serial);

        // Determine card type from SAK byte
        const char *cardType = "unknown";
        if      (uid.sak == 0x08) cardType = "MIFARE Classic 1K";
        else if (uid.sak == 0x18) cardType = "MIFARE Classic 4K";
        else if (uid.sak == 0x00) cardType = "MIFARE Ultralight";

        Serial.print("Type: ");
        Serial.println(cardType);
        Serial.println();

        rfid.haltCard();    // sleep the card — prevents duplicate reads
        delay(1000);
    }
    else
    {
        delay(200);         // poll every 200 ms when no card present
    }
}
