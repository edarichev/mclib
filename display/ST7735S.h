/**
 * ST7735 display
 */

#ifndef _ST7735S_H_INCLUDED_
#define _ST7735S_H_INCLUDED_

#include <cmath>
#include <cstdlib>
#include "../spiclient.h"
#include "../delay.h"
#include "../xuart.h"

enum class ST7735ColorMode : uint8_t {
    ColorMode12 = 0b011,
    ColorMode16 = 0b101,
    ColorMode18 = 0b110,
};


// 1111'1111'1111
#define ST7735_COLOR_CR4(x) ((uint32_t)(0xFF & (x)) * 0b1111 / 256)
#define ST7735_COLOR_CR5(x) ((uint32_t)(0xFF & (x)) * 0b11111 / 256)
#define ST7735_COLOR_CR6(x) ((uint32_t)(0xFF & (x)) * 0b111111 / 256)
#define ST7735_COLOR_12(r8, g8, b8) (uint32_t) ((ST7735_COLOR_CR4((r8)) << 8) | (ST7735_COLOR_CR4((g8)) << 4) | (ST7735_COLOR_CR4((b8))))
#define ST7735_COLOR_16(r8, g8, b8) (uint32_t) ((ST7735_COLOR_CR5((r8)) << 11) | (ST7735_COLOR_CR6((g8)) << 5) | (ST7735_COLOR_CR5((b8))))
#define ST7735_COLOR_18(r8, g8, b8) (uint32_t) ((ST7735_COLOR_CR6((r8)) << 12) | (ST7735_COLOR_CR6((g8)) << 6) | (ST7735_COLOR_CR6((b8))))

struct SPIConnectionParams
{
    GPIO_TypeDef* CS_PORT;
    uint16_t CS_Pin;
};

// контейнер для параметров соединения
class SPI
{
protected:
    SPI_HandleTypeDef *_hspi = nullptr;
    SPIConnectionParams _params;
protected:
    SPI(SPI_HandleTypeDef *hspi, const SPIConnectionParams &p) :
            _hspi(hspi), _params(p) {}
};

class SPIPollingModeMaster : public SPI
{
public:
    SPIPollingModeMaster(SPI_HandleTypeDef *hInstance, const SPIConnectionParams &p) :
            SPI(hInstance, p)
    {
    }

    inline bool write(const uint8_t *pData, uint16_t size, uint32_t timeout) const
    {
        return HAL_OK == HAL_SPI_Transmit(_hspi, (uint8_t*)pData, size, timeout);
    }
};

class SPIClientBase
{
protected:
    uint32_t Timeout = 1000;
public:
    SPIClientBase() {}
};

template <class TSPI> // TSPI==SPIPollingModeMaster
class SPIClientImpl : public SPIClientBase
{
protected:
    TSPI *_spi = nullptr;
public:
    using InterfaceType = TSPI;
    using InterfacePtrType = TSPI*;

    SPIClientImpl(TSPI *spi) :
            SPIClientBase(), _spi(spi)
    {
    }

    inline bool write(const uint8_t *b, uint16_t size) const
    {
        return _spi->write(b, size, Timeout);
    }
};

using SPIPollingClient = SPIClientImpl<SPIPollingModeMaster>;

extern UARTLogger logger;

struct ST7735InitParams
{
#if defined(PLATFORM_STM32_HAL)
    GPIO_TypeDef* DC_PORT;
    GPIO_TypeDef* RS_PORT;
    GPIO_TypeDef* CS_PORT;
#else
#endif
    uint16_t DC_Pin;
    uint16_t RS_Pin;
    uint16_t CS_Pin;
};

template<class TInterfaceClient = SPIPollingClient>
class ST7735SImpl: public TInterfaceClient
{
    ST7735InitParams _params;
public:
    ST7735SImpl(typename TInterfaceClient::InterfacePtrType spi,
            ST7735InitParams &params)
            : TInterfaceClient(spi), _params(params)
    {

    }

    enum ST7735Command : uint8_t
    {
        FRMCTR1 = 0xB1,
        FRMCTR2 = 0xB2,
        FRMCTR3 = 0xB3,
        INVCTR = 0xB4,
        PWCTR1 = 0xC0,
        PWCTR2 = 0xC1,
        PWCTR3 = 0xC2,
        PWCTR4 = 0xC3,
        PWCTR5 = 0xC4,
        VMCTR1 = 0xC5,
        INVOFF = 0x20,
        INVON = 0x21,
        MADCTL = 0x36,
        COLMOD = 0x3A,
        GMCTRP1 = 0xE0,
        GMCTRN1 = 0xE1,
        NORON = 0x13,
        DISPOFF = 0x28,
        DISPON = 0x29,
        SWRESET = 0x01,
        SLPOUT = 0x11,
        CASET = 0x2A,
        RASET = 0x2B,
        RAMWR = 0x2C,
    };

    ST7735ColorMode _colorMode = ST7735ColorMode::ColorMode16;

    void setColorMode(ST7735ColorMode cm)
    {
        _colorMode = cm;
    }

    void init()
    {
        pinSet(_params.RS_PORT, _params.RS_Pin);
        Delay::wait(5);
        pinReset(_params.RS_PORT, _params.RS_Pin);
        Delay::wait(5);
        pinSet(_params.RS_PORT, _params.RS_Pin);
        Delay::wait(5);

        sendCommand(ST7735Command::SWRESET);
        Delay::wait(150);
        sendCommand(ST7735Command::SLPOUT);
        Delay::wait(500);
        sendCommand(ST7735Command::FRMCTR1);
        uint8_t frmctr[] = {0x01, 0x2C, 0x2D};
        sendData(frmctr, sizeof(frmctr));
        sendCommand(ST7735Command::FRMCTR2);
        sendData(frmctr, sizeof(frmctr));
        sendCommand(ST7735Command::FRMCTR3);
        sendData(frmctr, sizeof(frmctr));
        sendData(frmctr, sizeof(frmctr));
        sendCommand(ST7735Command::INVCTR);
        sendData(0x07);
        sendCommand(ST7735Command::PWCTR1);
        uint8_t pwctr1[] = {0xA2, 0x02, 0x84};
        sendData(pwctr1, sizeof(pwctr1));
        sendCommand(ST7735Command::PWCTR2);
        sendData(0xC5);
        sendCommand(ST7735Command::PWCTR3);
        uint8_t pwctr3[] = {0x0A, 0};
        sendData(pwctr3, sizeof(pwctr3));
        sendCommand(ST7735Command::PWCTR4);
        uint8_t pwctr4[] = {0x8A, 0x2A};
        sendData(pwctr4, sizeof(pwctr4));
        sendCommand(ST7735Command::PWCTR5);
        uint8_t pwctr5[] = {0x8A, 0xEE};
        sendData(pwctr5, sizeof(pwctr5));
        sendCommand(ST7735Command::VMCTR1);
        sendData(0x0E);
        sendCommand(ST7735Command::INVOFF);
        sendCommand(ST7735Command::MADCTL);
        sendData(0xC0);
//        011 12-bit/pixel
//        101 16-bit/pixel
//        110 18-bit/pixel
//        12 битов на пиксель = 4 бита красный (R) + 4 бита зеленый (G) + 4 бита синий (B)
//        16 битов на пиксель = 5 битов красный (R) + 6 битов зеленый (G) + 5 битов синий (B)
//        18 битов на пиксель = 6 битов красный (R) + 6 битов зеленый (G) + 6 битов синий (B)
        sendCommand(ST7735Command::COLMOD);
        sendData((uint8_t)_colorMode);
        sendCommand(ST7735Command::GMCTRP1);
        uint8_t gmctrp1[] = {0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2b, 0x39, 0x00, 0x01, 0x03, 0x10};
        sendData(gmctrp1, sizeof(gmctrp1));
        sendCommand(ST7735Command::GMCTRN1);
        uint8_t gmctrn1[] = {0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2c, 0x29, 0x2d, 0x2e, 0x2e, 0x37, 0x3f, 0, 0, 0x02, 0x10};
        sendData(gmctrn1, sizeof(gmctrn1));
        sendCommand(ST7735Command::NORON);
        Delay::wait(10);
        sendCommand(ST7735Command::DISPON);
        Delay::wait(100);
    }

#if defined(PLATFORM_STM32_HAL)
    void pinSet(GPIO_TypeDef* port, uint16_t pin)
    {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    }

    void pinReset(GPIO_TypeDef* port, uint16_t pin)
    {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    }
#endif

    bool sendCommand(uint8_t data)
    {
        pinReset(_params.DC_PORT, _params.DC_Pin);
        return TInterfaceClient::write(&data, sizeof(data));
    }

    bool sendData(uint8_t data)
    {
        return sendData(&data, 1);
    }

    bool sendData(const uint8_t *data, size_t size)
    {
        pinSet(_params.DC_PORT, _params.DC_Pin);
        return TInterfaceClient::write(data, size);
    }

    // по краям экрана шум
    static const constexpr int COLUMN_OFFSET = 2;
    static const constexpr int ROW_OFFSET = 1;
    static const constexpr int Width = 128;
    static const constexpr int Height = 160;

    void setColAddr(uint16_t cStart, uint16_t cStop)
    {
        uint8_t data[4];
        cStart += COLUMN_OFFSET;
        cStop += COLUMN_OFFSET;

        data[0] = (cStart & 0xFF00) >> 8;
        data[1] = cStart & 0x00FF;
        data[2] = (cStop & 0xFF00) >> 8;
        data[3] = cStop & 0x00FF;

        sendCommand(ST7735Command::CASET);
        sendData(data, 4);
    }

    void setRowAddr(uint16_t rStart, uint16_t rStop)
    {
        uint8_t data[4];
        rStart += ROW_OFFSET;
        rStop += ROW_OFFSET;
        data[0] = (rStart & 0xFF00) >> 8;
        data[1] = rStart & 0x00FF;
        data[2] = (rStop & 0xFF00) >> 8;
        data[3] = rStop & 0x00FF;
        sendCommand(ST7735Command::RASET);
        sendData(data, 4);
    }

    void drawLine(uint32_t color, int x0, int y0, int x1, int y1)
    {
        if (x0 == x1) {
            for (int y = y0; y <= y1; ++y)
                setPixel(color, x0, y);
            return;
        }
        if (y0 == y1) {
            for (int x = x0; x <= x1; ++x)
                setPixel(color, x, y0);
            return;
        }
        int dx = abs(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0);
        int sy = y0 < y1 ? 1 : -1;
        int error = dx + dy;

        while (true) {
            setPixel(color, x0, y0);
            if (x0 == x1 && y0 == y1)
                break;
            int e2 = 2 * error;
            if (e2 >= dy) {
                if (x0 == x1)
                    break;
                error = error + dy;
                x0 = x0 + sx;
            }
            if (e2 <= dx) {
                if (y0 == y1)
                    break;
                error = error + dx;
                y0 = y0 + sy;
            }
        }
    }

    void setPixel(uint32_t color, int x, int y)
    {
        if (x < 0 || x >= Width || y < 0 || y >= Height)
            return;
        setColAddr(x, x+1);
        setRowAddr(y, y+1);

        sendCommand(ST7735Command::RAMWR);
        uint8_t colorBytes[2];
        colorBytes[0] = (color & 0xFF00) >> 8;
        colorBytes[1] = color & 0x00FF;
        sendData(colorBytes, 2);
    }

    void fillRectangle(uint32_t color, uint16_t x, uint16_t y,
            uint16_t width, uint16_t height)
    {
        setColAddr(x, x + width - 1);
        setRowAddr(y, y + height - 1);

        sendCommand(ST7735Command::RAMWR);

        uint32_t size = width * height;
        uint8_t colorBytes[2];
        colorBytes[0] = (color & 0xFF00) >> 8;
        colorBytes[1] = color & 0x00FF;

        for (uint32_t i = 0; i < size; i++) {
            sendData(colorBytes, 2);
        }
    }
};

using ST7735S = ST7735SImpl<SPIPollingClient>;

#endif // _ST7735S_H_INCLUDED_
