#pragma once

#include "fm17660.h"

// Maximum UID length (ISO 14443A supports 4, 7, or 10 bytes)
#define RFID_UID_MAX_LEN     10

// MIFARE Classic key length
#define RFID_KEY_LEN          6

// MIFARE Classic block size
#define RFID_BLOCK_SIZE      16

// Default factory key (FFFFFFFFFFFF)
static const uint8_t RFID_KEY_DEFAULT[RFID_KEY_LEN] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Key type
enum class KeyType : uint8_t
{
    KEY_A = 0x60,
    KEY_B = 0x61
};

// Result of a UID read
struct UID
{
    uint8_t  bytes[RFID_UID_MAX_LEN];
    uint8_t  length;      // 4, 7, or 10
    uint8_t  sak;         // Select Acknowledge byte (identifies card type)

    bool isValid() const { return length > 0; }
    void print(HardwareSerial &s) const;
};

class RFID
{
public:
    explicit RFID(FM17660 &dev);

    // Call once after FM17660::begin(). Loads ISO 14443A protocol.
    bool begin();

    // ── Card detection ────────────────────────────────────────────────────────

    // Returns true if at least one card is present in the field.
    bool isCardPresent();

    // Full sequence: REQA → anticollision (cascade) → select.
    // Populates uid. Returns false if no card or comms error.
    bool readUID(UID &uid);

    // Send HALT — puts the card to sleep so it won't disturb the next anticollision.
    bool haltCard();

    // ── MIFARE Classic authentication ─────────────────────────────────────────

    /**
     * Authenticate a sector before reading/writing.
     * @param block   any block number within the target sector
     * @param keyType KEY_A or KEY_B
     * @param key     6-byte key (default: all 0xFF)
     * @param uid     UID obtained from readUID()
     */
    bool authenticate(uint8_t block,
                      KeyType keyType,
                      const uint8_t *key,
                      const UID &uid);

    // Shorthand using the default factory key A
    bool authenticateDefault(uint8_t block, const UID &uid);

    // ── MIFARE Classic data operations ────────────────────────────────────────

    // Read one 16-byte block. Must authenticate the sector first.
    bool readBlock(uint8_t block, uint8_t data[RFID_BLOCK_SIZE]);

    // Write one 16-byte block. Must authenticate the sector first.
    bool writeBlock(uint8_t block, const uint8_t data[RFID_BLOCK_SIZE]);

    // ── Low-level ISO 14443A primitives (public for advanced use) ─────────────

    bool requestA(uint8_t atqa[2]);
    bool anticollision(uint8_t cascadeLevel, uint8_t *uid4, uint8_t *sak);
    bool selectCascade(uint8_t cascadeLevel, const uint8_t *uid4, uint8_t *sak);

private:
    FM17660 &_dev;
};
