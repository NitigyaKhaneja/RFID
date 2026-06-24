#pragma once

#include <Arduino.h>
#include <SPI.h>

// ─── Register map (FM17660 datasheet) ────────────────────────────────────────
#define FM_REG_COMMAND       0x00
#define FM_REG_FIFOCTRL      0x02
#define FM_REG_FIFOLENGTH    0x04
#define FM_REG_FIFODATA      0x05
#define FM_REG_IRQ0          0x06
#define FM_REG_IRQ1          0x07
#define FM_REG_IRQ0EN        0x08
#define FM_REG_IRQ1EN        0x09
#define FM_REG_ERROR         0x0A
#define FM_REG_STATUS        0x0B
#define FM_REG_RXBITCTRL     0x0C
#define FM_REG_RXCOLL        0x0D
#define FM_REG_TXCRCCON      0x2C
#define FM_REG_RXCRCCON      0x2D
#define FM_REG_TXDATANUM     0x2E
#define FM_REG_FRAMECON      0x33
#define FM_REG_RXCTRL        0x35
#define FM_REG_RXTXCON       0x77

// ─── Commands ─────────────────────────────────────────────────────────────────
#define FM_CMD_IDLE          0x00
#define FM_CMD_LOADKEY       0x02
#define FM_CMD_AUTHENT       0x03
#define FM_CMD_RECEIVE       0x05
#define FM_CMD_TRANSMIT      0x06
#define FM_CMD_TRANSCEIVE    0x07
#define FM_CMD_LOADPROTOCOL  0x0D
#define FM_CMD_SOFTRESET     0x1F

// ─── IRQ0 bit masks ───────────────────────────────────────────────────────────
#define FM_IRQ_IDLE          (1 << 4)
#define FM_IRQ_TX            (1 << 3)
#define FM_IRQ_RX            (1 << 2)
#define FM_IRQ_ERR           (1 << 1)

// ─── FIFO control ─────────────────────────────────────────────────────────────
#define FM_FIFO_FLUSH        (1 << 4)

// ─── Error register bit masks ─────────────────────────────────────────────────
#define FM_ERR_CMD           (1 << 7)
#define FM_ERR_FIFO_WR       (1 << 6)
#define FM_ERR_FIFO_OVF      (1 << 5)
#define FM_ERR_MINFRAME      (1 << 4)
#define FM_ERR_NODATA        (1 << 3)
#define FM_ERR_COLLISION     (1 << 2)
#define FM_ERR_PROTOCOL      (1 << 1)
#define FM_ERR_CRC_PARITY    (1 << 0)

// ─── SPI settings (FM17660: max 10 MHz, SPI mode 0) ──────────────────────────
#define FM_SPI_CLOCK         5000000UL  // 5 MHz — safe for most ESP32 boards
#define FM_SPI_MODE          SPI_MODE0
#define FM_SPI_ORDER         MSBFIRST

class FM17660
{
public:
    /**
     * @param csPin   GPIO connected to FM17660 NSS/CS
     * @param sckPin  GPIO for SCK  (-1 = use default SPI pins)
     * @param mosiPin GPIO for MOSI (-1 = use default SPI pins)
     * @param misoPin GPIO for MISO (-1 = use default SPI pins)
     */
    FM17660(uint8_t csPin,
            int8_t  sckPin  = -1,
            int8_t  mosiPin = -1,
            int8_t  misoPin = -1);

    // Initialise SPI and reset the IC. Returns false if the chip does not respond.
    bool begin();

    // Software reset — blocks until the IC is ready (~50 ms).
    void reset();

    // Raw register access
    uint8_t  readReg(uint8_t reg);
    void     writeReg(uint8_t reg, uint8_t value);
    void     modifyReg(uint8_t reg, uint8_t mask, uint8_t value); // clear mask bits, set value bits

    // FIFO helpers
    void     flushFIFO();
    uint16_t fifoLength();
    void     writeFIFO(const uint8_t *data, uint16_t len);
    uint16_t readFIFO(uint8_t *buffer, uint16_t maxLen);

    // CRC control — call before each command type
    void enableCRC(bool txCRC, bool rxCRC);

    // Set how many bits of the last byte to send (0 = whole byte = 8 bits)
    void setTxLastBits(uint8_t bits);

    // IRQ helpers
    void clearIRQ();
    bool waitIRQ(uint8_t irqMask, uint32_t timeoutMs = 150);

    // High-level command wrappers
    bool loadProtocol(uint8_t rxProtocol = 0, uint8_t txProtocol = 0);

    /**
     * Send txData and wait for a response into rxData.
     * @param rxData   buffer to receive into (caller provides, min 64 bytes for safety)
     * @param rxLen    number of bytes actually received
     * @param timeoutMs
     */
    bool transceive(const uint8_t *txData, uint16_t txLen,
                    uint8_t *rxData,       uint16_t *rxLen,
                    uint32_t timeoutMs = 150);

    // Human-readable error description from the last operation
    String lastError();

private:
    uint8_t _cs;
    int8_t  _sck, _mosi, _miso;

    SPISettings _spiSettings;

    void    _beginSPI();
    void    _endSPI();
};
