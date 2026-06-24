#include "rfid.h"

// ─────────────────────────────────────────────────────────────────────────────
// UID helper
// ─────────────────────────────────────────────────────────────────────────────

void UID::print(HardwareSerial &s) const
{
    s.print("UID (");
    s.print(length);
    s.print(" bytes): ");
    for (uint8_t i = 0; i < length; i++)
    {
        if (bytes[i] < 0x10) s.print('0');
        s.print(bytes[i], HEX);
        if (i < length - 1) s.print(':');
    }
    s.print("  SAK: 0x");
    s.println(sak, HEX);
}

// ─────────────────────────────────────────────────────────────────────────────
// Construction / begin
// ─────────────────────────────────────────────────────────────────────────────

RFID::RFID(FM17660 &dev) : _dev(dev) {}

bool RFID::begin()
{
    // Protocol 0/0 = ISO 14443A @ 106 kbit/s
    return _dev.loadProtocol(0, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// ISO 14443A low-level primitives
// ─────────────────────────────────────────────────────────────────────────────

/*
 * REQA is a 7-bit short frame (0x26).
 * The FM17660 must be told to send only 7 bits in the last byte.
 * CRC must be OFF for REQA and anticollision.
 */
bool RFID::requestA(uint8_t atqa[2])
{
    _dev.enableCRC(false, false);
    _dev.setTxLastBits(7);          // FIX: 7-bit short frame

    uint8_t reqa  = 0x26;
    uint8_t rx[4];
    uint16_t rxLen = 0;

    bool ok = _dev.transceive(&reqa, 1, rx, &rxLen, 50);

    _dev.setTxLastBits(0);          // restore full-byte mode

    if (!ok || rxLen < 2)
        return false;

    atqa[0] = rx[0];
    atqa[1] = rx[1];
    return true;
}

/*
 Single cascade-level anticollision loop.
 cascadeLevel: 0x93 (CL1), 0x95 (CL2), 0x97 (CL3)
 On success, uid4 receives 4 bytes and sak receives the Select Acknowledge.
 FIX: CRC is OFF during anticollision, ON during select.
 */
bool RFID::anticollision(uint8_t cascadeLevel, uint8_t *uid4, uint8_t *sak)
{
    // ── Anticollision (no CRC) ────────────────────────────────────────────────
    _dev.enableCRC(false, false);

    uint8_t cmd[2] = { cascadeLevel, 0x20 };
    uint8_t rx[8];
    uint16_t rxLen = 0;

    if (!_dev.transceive(cmd, 2, rx, &rxLen, 100))
        return false;

    if (rxLen < 5)
        return false;

    // rx[0..3] = UID bytes, rx[4] = BCC (XOR check)
    uint8_t bcc = rx[0] ^ rx[1] ^ rx[2] ^ rx[3];
    if (bcc != rx[4])
        return false;

    memcpy(uid4, rx, 4);

    // ── Select (with CRC) ─────────────────────────────────────────────────────
    return selectCascade(cascadeLevel, uid4, sak);
}

bool RFID::selectCascade(uint8_t cascadeLevel, const uint8_t *uid4, uint8_t *sak)
{
    _dev.enableCRC(true, true);     // FIX: CRC must be on for SELECT

    // SELECT frame: SEL | NVB(0x70) | UID0..3 | BCC
    uint8_t frame[7];
    frame[0] = cascadeLevel;
    frame[1] = 0x70;
    memcpy(&frame[2], uid4, 4);
    frame[6] = uid4[0] ^ uid4[1] ^ uid4[2] ^ uid4[3];  // BCC

    uint8_t rx[4];
    uint16_t rxLen = 0;

    if (!_dev.transceive(frame, 7, rx, &rxLen, 100))
        return false;

    if (rxLen < 1)
        return false;

    *sak = rx[0];
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Public card detection
// ─────────────────────────────────────────────────────────────────────────────

bool RFID::isCardPresent()
{
    uint8_t atqa[2];
    return requestA(atqa);
}

/*
 Full UID read with cascade support (ISO 14443A 4/7/10-byte UIDs).
 Cascade tag 0x88 in the first byte of a CL1 UID means the UID is longer
 than 4 bytes — we must do another cascade level.
 */
bool RFID::readUID(UID &uid)
{
    uid.length = 0;

    uint8_t atqa[2];
    if (!requestA(atqa))
        return false;

    uint8_t  cl1uid[4], cl2uid[4], cl3uid[4];
    uint8_t  sak = 0;

    static const uint8_t CASCADE_LEVELS[3] = { 0x93, 0x95, 0x97 };

    // ── Cascade level 1 ───────────────────────────────────────────────────────
    if (!anticollision(CASCADE_LEVELS[0], cl1uid, &sak))
        return false;

    if (cl1uid[0] != 0x88)
    {
        // 4-byte UID
        memcpy(uid.bytes, cl1uid, 4);
        uid.length = 4;
        uid.sak    = sak;
        return true;
    }

    // CT (cascade tag) present — first 3 bytes are the UID prefix
    memcpy(uid.bytes, &cl1uid[1], 3);

    // ── Cascade level 2 ───────────────────────────────────────────────────────
    if (!anticollision(CASCADE_LEVELS[1], cl2uid, &sak))
        return false;

    if (cl2uid[0] != 0x88)
    {
        // 7-byte UID
        memcpy(&uid.bytes[3], cl2uid, 4);
        uid.length = 7;
        uid.sak    = sak;
        return true;
    }

    memcpy(&uid.bytes[3], &cl2uid[1], 3);

    // ── Cascade level 3 ───────────────────────────────────────────────────────
    if (!anticollision(CASCADE_LEVELS[2], cl3uid, &sak))
        return false;

    // 10-byte UID
    memcpy(&uid.bytes[6], cl3uid, 4);
    uid.length = 10;
    uid.sak    = sak;
    return true;
}

bool RFID::haltCard()
{
    _dev.enableCRC(true, true);

    uint8_t halt[2] = { 0x50, 0x00 };
    uint8_t rx[4];
    uint16_t rxLen = 0;

    // Card does not ACK HALT — a timeout here is expected and is fine.
    _dev.transceive(halt, 2, rx, &rxLen, 30);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// MIFARE Classic authentication
// ─────────────────────────────────────────────────────────────────────────────

/*
 The FM17660 AUTHENT command works as follows:
 1. Load the 6-byte key into FIFO, then issue CMD_LOADKEY.
 2. Build the auth frame: [KeyType, BlockNum, UID0, UID1, UID2, UID3]
 Load that into FIFO, then issue CMD_AUTHENT.
 3. Poll for IDLE IRQ. Check STATUS register bit 5 (CRYPTO1ON) for success.
 FIX: original code used magic register addresses, had no error check,
 and always returned true.
 */
bool RFID::authenticate(uint8_t block,
                        KeyType keyType,
                        const uint8_t *key,
                        const UID &uid)
{
    // Step 1: load key
    _dev.flushFIFO();
    _dev.writeFIFO(key, RFID_KEY_LEN);
    _dev.clearIRQ();
    _dev.writeReg(FM_REG_COMMAND, FM_CMD_LOADKEY);
    if (!_dev.waitIRQ(FM_IRQ_IDLE, 100))
        return false;

    // Step 2: authenticate
    uint8_t authFrame[6];
    authFrame[0] = static_cast<uint8_t>(keyType);
    authFrame[1] = block;
    // Use first 4 UID bytes (ISO 14443A auth always uses first 4)
    memcpy(&authFrame[2], uid.bytes, 4);

    _dev.flushFIFO();
    _dev.writeFIFO(authFrame, 6);
    _dev.clearIRQ();
    _dev.writeReg(FM_REG_COMMAND, FM_CMD_AUTHENT);
    if (!_dev.waitIRQ(FM_IRQ_IDLE, 100))
        return false;

    // Step 3: verify CRYPTO1ON in STATUS register
    uint8_t status = _dev.readReg(FM_REG_STATUS);
    return (status & (1 << 5)) != 0;       // bit 5 = CRYPTO1ON
}

bool RFID::authenticateDefault(uint8_t block, const UID &uid)
{
    return authenticate(block, KeyType::KEY_A, RFID_KEY_DEFAULT, uid);
}

// ─────────────────────────────────────────────────────────────────────────────
// MIFARE Classic read / write
// ─────────────────────────────────────────────────────────────────────────────

bool RFID::readBlock(uint8_t block, uint8_t data[RFID_BLOCK_SIZE])
{
    _dev.enableCRC(true, true);

    uint8_t cmd[2] = { 0x30, block };
    uint16_t rxLen = 0;

    if (!_dev.transceive(cmd, 2, data, &rxLen, 150))
        return false;

    return (rxLen == RFID_BLOCK_SIZE);
}

bool RFID::writeBlock(uint8_t block, const uint8_t data[RFID_BLOCK_SIZE])
{
    _dev.enableCRC(true, true);

    // Phase 1: send WRITE command and block number, expect ACK (0x0A nibble)
    uint8_t cmd[2] = { 0xA0, block };
    uint8_t ack[4];
    uint16_t rxLen = 0;

    if (!_dev.transceive(cmd, 2, ack, &rxLen, 100))
        return false;

    if (rxLen < 1 || (ack[0] & 0x0F) != 0x0A)   // 0x0A = MIFARE ACK
        return false;

    // Phase 2: send the 16 data bytes
    if (!_dev.transceive(data, RFID_BLOCK_SIZE, ack, &rxLen, 150))
        return false;

    return (rxLen >= 1 && (ack[0] & 0x0F) == 0x0A);
}
