#include "fm17660.h"

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / begin
// ─────────────────────────────────────────────────────────────────────────────

FM17660::FM17660(uint8_t csPin, int8_t sckPin, int8_t mosiPin, int8_t misoPin)
    : _cs(csPin), _sck(sckPin), _mosi(mosiPin), _miso(misoPin),
      _spiSettings(FM_SPI_CLOCK, FM_SPI_ORDER, FM_SPI_MODE)
{
}

bool FM17660::begin()
{
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);

    // Allow custom pins (useful on ESP32 which has flexible SPI routing)
    if (_sck != -1 && _mosi != -1 && _miso != -1)
        SPI.begin(_sck, _miso, _mosi, _cs);
    else
        SPI.begin();

    reset();

    // Verify the chip responds: after reset REG_COMMAND should read 0x00 (IDLE)
    uint8_t cmd = readReg(FM_REG_COMMAND);
    return (cmd == FM_CMD_IDLE);
}

void FM17660::reset()
{
    writeReg(FM_REG_COMMAND, FM_CMD_SOFTRESET);
    delay(50);
    // Flush any stale IRQ flags left from before the reset
    clearIRQ();
}

// ─────────────────────────────────────────────────────────────────────────────
// SPI primitives
// ─────────────────────────────────────────────────────────────────────────────

/*
 FM17660 SPI address byte format:
 bit 0   : 0 = write, 1 = read
 bits 7:1: register address
 */

void FM17660::_beginSPI()
{
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_cs, LOW);
}

void FM17660::_endSPI()
{
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
}

uint8_t FM17660::readReg(uint8_t reg)
{
    _beginSPI();
    SPI.transfer((reg << 1) | 0x01);   // address with read bit set
    uint8_t val = SPI.transfer(0x00);  // clock out data byte
    _endSPI();
    return val;
}

void FM17660::writeReg(uint8_t reg, uint8_t value)
{
    _beginSPI();
    SPI.transfer(reg << 1);            // address with write bit (0)
    SPI.transfer(value);
    _endSPI();
}

void FM17660::modifyReg(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current = readReg(reg);
    current = (current & ~mask) | (value & mask);
    writeReg(reg, current);
}

// ─────────────────────────────────────────────────────────────────────────────
// FIFO
// ─────────────────────────────────────────────────────────────────────────────

void FM17660::flushFIFO()
{
    modifyReg(FM_REG_FIFOCTRL, FM_FIFO_FLUSH, FM_FIFO_FLUSH);
}

uint16_t FM17660::fifoLength()
{
    return (uint16_t)readReg(FM_REG_FIFOLENGTH);
}

void FM17660::writeFIFO(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
        writeReg(FM_REG_FIFODATA, data[i]);
}

uint16_t FM17660::readFIFO(uint8_t *buffer, uint16_t maxLen)
{
    uint16_t len = fifoLength();
    if (len > maxLen) len = maxLen;
    for (uint16_t i = 0; i < len; i++)
        buffer[i] = readReg(FM_REG_FIFODATA);
    return len;
}

// ─────────────────────────────────────────────────────────────────────────────
// CRC / framing helpers
// ─────────────────────────────────────────────────────────────────────────────

void FM17660::enableCRC(bool txCRC, bool rxCRC)
{
    // REG_TXCRCCON bit 0: TX CRC enable
    // REG_RXCRCCON bit 0: RX CRC enable
    modifyReg(FM_REG_TXCRCCON, 0x01, txCRC ? 0x01 : 0x00);
    modifyReg(FM_REG_RXCRCCON, 0x01, rxCRC ? 0x01 : 0x00);
}

void FM17660::setTxLastBits(uint8_t bits)
{
    // REG_TXDATANUM bits [2:0]: number of valid bits in last TX byte (0 = 8)
    modifyReg(FM_REG_TXDATANUM, 0x07, bits & 0x07);
}

// ─────────────────────────────────────────────────────────────────────────────
// IRQ
// ─────────────────────────────────────────────────────────────────────────────

void FM17660::clearIRQ()
{
    // Write 0x00 to both IRQ status registers to clear all flags
    writeReg(FM_REG_IRQ0, 0x00);
    writeReg(FM_REG_IRQ1, 0x00);
}

bool FM17660::waitIRQ(uint8_t irqMask, uint32_t timeoutMs)
{
    uint32_t start = millis();
    while ((millis() - start) < timeoutMs)
    {
        uint8_t irq = readReg(FM_REG_IRQ0);
        if (irq & irqMask)
            return true;
        // Yield to RTOS on ESP32 — avoids watchdog resets on long waits
        yield();
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// High-level command wrappers
// ─────────────────────────────────────────────────────────────────────────────

bool FM17660::loadProtocol(uint8_t rxProtocol, uint8_t txProtocol)
{
    flushFIFO();
    writeReg(FM_REG_FIFODATA, rxProtocol);
    writeReg(FM_REG_FIFODATA, txProtocol);
    clearIRQ();                             // FIX: clear before issuing command
    writeReg(FM_REG_COMMAND, FM_CMD_LOADPROTOCOL);
    return waitIRQ(FM_IRQ_IDLE);
}

bool FM17660::transceive(const uint8_t *txData, uint16_t txLen,
                         uint8_t *rxData,       uint16_t *rxLen,
                         uint32_t timeoutMs)
{
    flushFIFO();
    writeFIFO(txData, txLen);

    clearIRQ();                              // FIX: always clear before command
    writeReg(FM_REG_COMMAND, FM_CMD_TRANSCEIVE);

    // Wait for RX done OR idle (card went away) OR error
    if (!waitIRQ(FM_IRQ_RX | FM_IRQ_IDLE | FM_IRQ_ERR, timeoutMs))
        return false;

    uint8_t err = readReg(FM_REG_ERROR);
    if (err & ~FM_ERR_NODATA)               // ignore "no data" on REQA/WUPA
        return false;

    *rxLen = readFIFO(rxData, 64);
    return (*rxLen > 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Error decoding
// ─────────────────────────────────────────────────────────────────────────────

String FM17660::lastError()
{
    uint8_t err = readReg(FM_REG_ERROR);
    if (!err)                        return "none";
    if (err & FM_ERR_CMD)            return "command error";
    if (err & FM_ERR_FIFO_WR)        return "FIFO write error";
    if (err & FM_ERR_FIFO_OVF)       return "FIFO overflow";
    if (err & FM_ERR_MINFRAME)       return "min frame error";
    if (err & FM_ERR_NODATA)         return "no data";
    if (err & FM_ERR_COLLISION)      return "collision";
    if (err & FM_ERR_PROTOCOL)       return "protocol error";
    if (err & FM_ERR_CRC_PARITY)     return "CRC / parity error";
    return "unknown (0x" + String(err, HEX) + ")";
}
