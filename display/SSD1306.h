/**
 * SSD1306 / GM009605 OLED Display
 * 12.04.2024 09:49:41
 * Author: Evgeny Darichev
 */

#ifndef _SSD1306_H_INCLUDED_
#define _SSD1306_H_INCLUDED_

#include <cmath>
#include <cstdlib>
#include "../i2cclient.h"

#include "../xuart.h"
extern UART_HandleTypeDef huart2;

#ifndef ABS
#define ABS(x) ((x) > 0 ? (x) : -(x))
#endif

#ifndef SWAP
#define SWAP(x, y) { if ((x) > (y)) { std::swap(x, y); } }
#endif

#ifndef MIN
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#endif

#ifndef MAX
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#endif

enum class RasterOpCode
{
    // Copies the source rectangle directly to the destination rectangle.
    SRCCOPY,
    // Combines the colors of the source and destination rectangles by using the Boolean AND operator.
    SRCAND,
    // Combines the colors of the source and destination rectangles by using the Boolean OR operator.
    SRCOR,
    // Combines the colors of the source and destination rectangles by using the Boolean XOR operator.
    SRCXOR
};


enum SSD1306Command : uint8_t
{
    SET_CONTRAST_CONTROL = 0x81,
    DISPLAY_ON_RESUME_TO_RAM = 0xA4,
    DISPLAY_ON_IGNORE_RAM = 0xA5,
    SET_NORMAL_DISPLAY = 0xA6,
    SET_INVERSE_DISPLAY = 0xA7,
    DISPLAY_OFF = 0xAE,
    DISPLAY_ON = 0xAF,
    SET_MEMORY_ADDRESSING_MODE = 0X20,
    SET_HORIZONTAL_ADDRESSING_MODE = SET_MEMORY_ADDRESSING_MODE | 0x00,
    SET_VERTICAL_ADDRESSING_MODE = SET_MEMORY_ADDRESSING_MODE | 0x01,
    SET_PAGE_ADDRESSING_MODE = SET_MEMORY_ADDRESSING_MODE | 0b10,
};

/**
 * Represents the display with SSD1306
 *
 * В графическом режиме отрисовка делается в буфере ОЗУ,
 * но не отправляется сразу на дисплей,
 * поэтому, закончив рисование, нужно вызвать flush().
 */
template <class TInterfaceClient = I2CPollingClient>
class SSD1306Impl : public TInterfaceClient
{
public:
    SSD1306Impl(typename TInterfaceClient::InterfacePtrType i2c, uint8_t addr = 0x3C)
                : TInterfaceClient(i2c, (uint16_t) (addr << 1))
    {
        
    }

    virtual ~SSD1306Impl()
    {
        delete [] _ram;
    }

    bool init()
    {
        sendCommand(SSD1306Command::DISPLAY_OFF);
        sendCommand(SSD1306Command::SET_MEMORY_ADDRESSING_MODE);
        sendCommand(SSD1306Command::SET_PAGE_ADDRESSING_MODE); // page addressing, 0xb0 - start address
        sendCommand(0xB0);
        sendCommand(0xC8); // Set COM Output Scan Direction
        sendCommand(0x00); // set low column address
        sendCommand(0x10); // set high column address
        sendCommand(0x40); // set start line address
        sendCommand(SSD1306Command::SET_CONTRAST_CONTROL);
        sendCommand(0xFF);
        sendCommand(0xA1); // set segment re-map 0 to 127
        sendCommand(SSD1306Command::SET_NORMAL_DISPLAY); //--set normal display
        sendCommand(0xA8); // set multiplex ratio(1 to 64)
        sendCommand(0x3F); //
        sendCommand(SSD1306Command::DISPLAY_ON_RESUME_TO_RAM);
        sendCommand(0xD3); // set display offset
        sendCommand(0x00); // not offset
        sendCommand(0xD5); // set display clock divide ratio/oscillator frequency
        sendCommand(0xF0); // set divide ratio
        sendCommand(0xD9); // set pre-charge period
        sendCommand(0x22); //
        sendCommand(0xDA); // set com pins hardware configuration
        sendCommand(0x12);
        sendCommand(0xDB); // set vcomh
        sendCommand(0x20); // 0x20,0.77xVcc
        sendCommand(0x8D); // set DC-DC enable
        sendCommand(0x14); //
        sendCommand(SSD1306Command::DISPLAY_ON);
        return true;
    }

    bool setContrast(uint8_t value)
    {
        uint8_t cmd[2] = { SSD1306Command::SET_CONTRAST_CONTROL, value };
        return sendCommand(cmd, sizeof(cmd));
    }

    /**
     * @param useRAMContent true: Resume to RAM content display (RESET) Output follows RAM content;
     *                      false: Entire display ON, Output ignores RAM content

     */
    bool on(bool useRAMContent)
    {
        return TInterfaceClient::write(useRAMContent ?
                SSD1306Command::DISPLAY_ON_RESUME_TO_RAM :
                SSD1306Command::DISPLAY_ON_IGNORE_RAM);
    }

    inline bool sendCommand(uint8_t* data, uint32_t size)
    {
        return TInterfaceClient::memWrite(0, 8, data, size);
    }

    inline bool sendCommand(uint8_t data)
    {
        return TInterfaceClient::memWrite(0, 8, &data, sizeof(data));
    }

    inline bool sendCommand(uint8_t cmd, uint8_t data)
    {
        uint8_t c[2] = {cmd, data};
        return TInterfaceClient::memWrite(0, 8, c, sizeof(data));
    }

    bool sendData(const uint8_t* data, uint32_t size)
    {
        // 0x40 - начало блока RAM в дисплее
        return TInterfaceClient::memWrite(0x40, 8, data, size);// or interrupt
    }

    void fill(uint32_t color)
    {
        if (_ram && _mode == DisplayMode::Graphics) {
            memset(_ram, color ? 0xFF : 0, _ramSize);
            return;
        }
        fillWith(color ? 0xFF : 0x00);
    }

    void fillWith(uint8_t pattern)
    {
        // for page addressing mode
        uint8_t row[_width / 8] {};
        memset(row, pattern, sizeof(row));

        for (uint32_t r = 0; r < _height / 8; ++r) {
            sendCommand(0x00); //---set low column address
            sendCommand(0x10); //---set high column address
            sendCommand(0xB0 | r);
            for (uint32_t c = 0; c < sizeof(row); ++c)
                sendData(row, sizeof(row));
        }
    }

    void fillWith(uint8_t pattern[8])
    {
        // for page addressing mode
        for (uint32_t r = 0; r < 8; ++r) {
            for (uint32_t c = 0; c < 128; c += 8) {
                setPos(r, c);
                sendData(pattern, 8);
            }
        }
    }

    void printString(const char *str, const uint8_t *font8x8)
    {
        const char *t = str;
        while (uint32_t code = *t++) {
            sendData(font8x8 + code * 8, 8);
        }
    }


    void drawLine(uint32_t color, int x0, int y0, int x1, int y1)
    {
        if (!_ram)
            return;
        line(color, x0, y0, x1, y1, _ram, _width, _height);
    }

    void drawRectangle(uint32_t color, int x0, int y0, int width, int height)
    {
        if (!_ram)
            return;
        line(color, x0, y0, x0 + width, y0, _ram, _width, _height);
        line(color, x0 + width, y0, x0 + width, y0 + height, _ram, _width, _height);
        line(color, x0, y0, x0, y0 + height, _ram, _width, _height);
        line(color, x0, y0 + height, x0 + width, y0 + height, _ram, _width, _height);
    }

    void fillRectangle(uint32_t color, int x0, int y0, int width, int height)
    {
        if (!_ram)
            return;
        x0 = MAX(0, x0);
        y0 = MAX(0, y0);
        int right = MIN((int)_width, x0 + width);
        for (int y = y0; y <= y0 + height && y < (int)_height; ++y)
            line(color, x0, y, right, y, _ram, _width, _height);
    }

    void drawEllipse(uint32_t color, int x0, int y0, int width, int height)
    { // https://zingl.github.io/bresenham.html
        if (!_ram)
            return;
        int x1 = x0 + width, y1 = y0 + height;
        int a = ABS(x1-x0), b = ABS(y1-y0), b1 = b&1; /* values of diameter */
        long dx = 4*(1-a)*b*b, dy = 4*(b1+1)*a*a; /* error increment */
        long err = dx+dy+b1*a*a, e2; /* error of 1.step */

        if (x0 > x1) { x0 = x1; x1 += a; } /* if called with swapped points */
        if (y0 > y1) y0 = y1; /* .. exchange them */
        y0 += (b+1)/2; y1 = y0-b1;   /* starting pixel */
        a *= 8*a; b1 = 8*b*b;

        do {
            setPixel(color, x1, y0); /*   I. Quadrant */
            setPixel(color, x0, y0); /*  II. Quadrant */
            setPixel(color, x0, y1); /* III. Quadrant */
            setPixel(color, x1, y1); /*  IV. Quadrant */
            e2 = 2*err;
            if (e2 <= dy) { y0++; y1--; err += dy += a; }  /* y step */
            if (e2 >= dx || 2*err > dy) { x0++; x1--; err += dx += b1; } /* x step */
        } while (x0 <= x1);

        while (y0-y1 < b) {  /* too early stop of flat ellipses a=1 */
            setPixel(color, x0-1, y0); /* -> finish tip of ellipse */
            setPixel(color, x1+1, y0++);
            setPixel(color, x0-1, y1);
            setPixel(color, x1+1, y1--);
        }
    }

    void fillEllipse(uint32_t color, int x0, int y0, int width, int height)
    {
        if (!_ram)
            return;
        int x1 = x0 + width, y1 = y0 + height;
        int a = ABS(x1-x0), b = ABS(y1-y0), b1 = b&1; /* values of diameter */
        long dx = 4*(1-a)*b*b, dy = 4*(b1+1)*a*a; /* error increment */
        long err = dx+dy+b1*a*a, e2; /* error of 1.step */

        if (x0 > x1) { x0 = x1; x1 += a; } /* if called with swapped points */
        if (y0 > y1) y0 = y1; /* .. exchange them */
        y0 += (b+1)/2; y1 = y0-b1;   /* starting pixel */
        a *= 8*a; b1 = 8*b*b;

        do {
            drawLine(color, x0, y0, x1, y0);
            drawLine(color, x0, y1, x1, y1);
            e2 = 2*err;
            if (e2 <= dy) { y0++; y1--; err += dy += a; }  /* y step */
            if (e2 >= dx || 2*err > dy) { x0++; x1--; err += dx += b1; } /* x step */
        } while (x0 <= x1);

        while (y0-y1 < b) {  /* too early stop of flat ellipses a=1 */
            setPixel(color, x0-1, y0); /* -> finish tip of ellipse */
            setPixel(color, x1+1, y0++);
            setPixel(color, x0-1, y1);
            setPixel(color, x1+1, y1--);
        }
    }

    void drawString(uint32_t color, int left, int top, const char *s,
                const uint8_t *fontData, int symbolWidth, int symbolHeight)
    {
        if (!_ram)
            return;
        drawString(color, left, top, s, fontData, symbolWidth, symbolHeight, _ram, _width, _height);
    }

    void drawString(uint32_t color, int left, int top, const char *s,
                    const uint8_t *fontData, const uint8_t *remapFontArray,
                    uint32_t remapCharCount,
                    int symbolWidth, int symbolHeight)
    {
        if (!_ram)
            return;
        drawString(color, left, top, s, fontData, remapFontArray, remapCharCount, symbolWidth, symbolHeight, _ram, _width, _height);
    }

    void drawBitmap(int xdest, int ydest, uint8_t *bmpMonoSource,
                uint32_t bmpWidth, uint32_t bmpHeight, RasterOpCode opCode)
    {
        drawBitmap(xdest, ydest, bmpMonoSource, bmpWidth, bmpHeight, _ram, _width, _height, opCode);
    }

    /**
     * Sets the position in text mode
     *
     * @param row [0..7]
     * @param column [0..127]
     */
    void setPos(uint8_t row, uint8_t column)
    {
        uint8_t cLow = 0xF & column; // младший полубайт номера колонки
        uint8_t cHigh = 0xF & (column >> 4); // старший полубайт номера колонки
        sendCommand(0x00 | cLow); //---set low column address
        sendCommand(0x10 | cHigh); //---set high column address
        sendCommand(0xB0 | row);
    }

    enum class DisplayMode : uint8_t
    {
        Text,
        Graphics
    };

    const uint32_t _width = 128; // px
    const uint32_t _height = 64; // px

    DisplayMode _mode = DisplayMode::Text;

    // Graphics mode
    const uint32_t _ramSize = (_height) * (_width / 8);
    uint8_t *_ram = nullptr;

    void setTextMode()
    {
        _mode = DisplayMode::Text;
        delete [] _ram;
        _ram = nullptr;
        sendCommand(SSD1306Command::SET_PAGE_ADDRESSING_MODE);
        sendCommand(0x00); //---set low column address
        sendCommand(0x10); //---set high column address
        sendCommand(0xB0);
    }

    void setGraphicsMode()
    {
        sendCommand(SSD1306Command::SET_MEMORY_ADDRESSING_MODE);
        if (!_ram)
            _ram = new uint8_t[_ramSize];
        _mode = DisplayMode::Graphics;
        UARTLogger logger(&huart2);
        if (!_ram)
            logger.writeLine("Error allocating memory");
    }

    void flush()
    {
        for (uint32_t r = 0; r < _height / 8; ++r) {
            sendCommand(0x00); //---set low column address
            sendCommand(0x10); //---set high column address
            sendCommand(0xB0 | r);
            sendData(_ram + r * _width, _width);
        }
    }

    void setPixel(uint32_t color, uint16_t x, uint16_t y)
    {
        return setPixel(color, x, y, _ram, _width, _height);
    }


private:
    static void setPixel(uint32_t color, int x, int y, uint8_t *arr, int width, int height)
    {
        if (x < 0 || x >= width || y < 0 || y >= height || width < 0 || height < 0)
            return;
        uint8_t &p = arr[x + (y / 8) * width];
        uint8_t v = (1 << (y % 8));
        if (color)
            p |= v;
        else
            p &= ~v;
    }

    static uint32_t getPixel(int x, int y, uint8_t *arr, int width, int height)
    {
        if (x < 0 || x >= width || y < 0 || y >= height || width < 0 || height < 0)
            return 0;
        return (arr[x + (y / 8) * width] & (1 << (y % 8))) ? 0xFFFFFFFF : 0;
    }

    static void line(uint32_t color, int x0, int y0, int x1, int y1, uint8_t *arr, int width, int height)
    {
        if (x0 == x1) {
            for (int y = y0; y <= y1; ++y)
                setPixel(color, x0, y, arr, width, height);
            return;
        }
        if (y0 == y1) {
            for (int x = x0; x <= x1; ++x)
                setPixel(color, x, y0, arr, width, height);
            return;
        }
        int dx = ABS(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0);
        int sy = y0 < y1 ? 1 : -1;
        int error = dx + dy;

        while (true) {
            setPixel(color, x0, y0, arr, width, height);
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

    static uint8_t getMonoBitmapBitValue(uint32_t pixelNumber, const uint8_t *bmpMonoSource)
    {
        uint32_t byteNumber = pixelNumber / 8;
        uint8_t byte = *(bmpMonoSource + byteNumber);
        uint32_t bitNumber = 7 - (pixelNumber - byteNumber * 8);
        byte >>= bitNumber;
        byte &= 0x01; // выделить значение пиксела
        return byte;
    }

    /**
     * Рисует строку в битмапе.
     *
     * @param fontData Битмапы букв шрифта. Шрифт может быть любого размера.
     *        Кодируется побитово: 1 - есть окраска, 0 - не окрашено.
     *        Буква выравнивается до байта, например, буква в 12 пикселей
     *        по ширине и 18 в высоту займёт 2 байта по ширине и 18 по высоте.
     *        Буквы рисуются как есть, сверху вниз без разворотов,
     *        незанятые биты заполняются нулями.
     */
    static void drawString(uint32_t color, int left, int top, const char *s,
            const uint8_t *fontData, int symbolWidth, int symbolHeight,
            uint8_t *dest, int width, int height)
    {
        int x = left;
        int y = top;
        UARTLogger logger(&huart2);
        while (char ch = *s++) {
            int charIndex = ch;
            int bitmapStartPixel = charIndex * symbolWidth * symbolHeight;
            int i = bitmapStartPixel;
            for (int r = 0; r < symbolHeight; ++r) {
                for (int c = 0; c < symbolWidth; ++c, ++i) {
                    uint32_t byteNumber = i / 8;
                    uint32_t bitNumber = 7 - (i - byteNumber * 8);
                    uint8_t byte = fontData[byteNumber];
                    byte >>= bitNumber;
                    byte &= 1;
                    if (byte)
                        setPixel(color, x + c, y + r, dest, width, height);
                    logger.write("%c", byte ? '1' : '0');
                }
                logger.write("\r\n");
            }
            logger.write("\r\n");
            x += symbolWidth;
        }
    }

    static void drawString(uint32_t color, int left, int top, const char *s,
            const uint8_t *fontData, const uint8_t *remapFontArray,
            uint32_t remapCharCount,
            int symbolWidth, int symbolHeight,
            uint8_t *dest, int width, int height)
    {
        int x = left;
        int y = top;
        UARTLogger logger(&huart2);
        while (char ch = *s++) {
            int charIndex = ch;
            uint8_t *pfind = (uint8_t *)std::bsearch(&charIndex, remapFontArray, remapCharCount, sizeof(uint8_t),
                    [](void const *lhs, void const *rhs) { return *(uint8_t*)lhs - *(uint8_t*)rhs; });
            if (!pfind)
                continue;
            charIndex = pfind - remapFontArray;
            int bitmapStartPixel = charIndex * symbolWidth * symbolHeight;
            int i = bitmapStartPixel;
            for (int r = 0; r < symbolHeight; ++r) {
                for (int c = 0; c < symbolWidth; ++c, ++i) {
                    uint32_t byteNumber = i / 8;
                    uint32_t bitNumber = 7 - (i - byteNumber * 8);
                    uint8_t byte = fontData[byteNumber];
                    byte >>= bitNumber;
                    byte &= 1;
                    if (byte)
                        setPixel(color, x + c, y + r, dest, width, height);
                    logger.write("%c", byte ? '1' : '0');
                }
                logger.write("\r\n");
            }
            logger.write("\r\n");
            x += symbolWidth;
        }
    }
    /**
     * Draws the bitmap
     *
     * @param bmpMonoSource монохромный битмап (1 бит на пиксель) в нормальном человеческом представлении,
     *                      т.е. по строкам и по столбцам, как если бы это
     *                      была обычная картинка (не постранично и не повёрнуто,
     *                      как в этом дисплее).
     *                      Можно нарисовать в Calc или Excel с помощью 0 и 1
     *                      нужный битмап, а затем последовательно соединить всё в одну строку,
     *                      разбить по 8 цифр и дополнить справа до 8.
     */
    static void drawBitmap(int xdest, int ydest, uint8_t *bmpMonoSource,
            int bmpWidth, int bmpHeight,
            uint8_t *dest, int width, int height,
            RasterOpCode opCode)
    {
        if (xdest >= width || ydest >= height)
            return;
        int right = MIN(width, xdest + MIN(bmpWidth, width));
        int bottom = MIN(height, ydest + MIN(bmpHeight, height));
        uint32_t pixelNumber = 0;
        int ySrc = 0, x = 0, y = 0;
        uint32_t color = 0, prevColor = 0;
        switch (opCode) {
        case RasterOpCode::SRCCOPY: {
            for (y = ydest, ySrc = 0; y < bottom; ++y, ++ySrc) {
                pixelNumber = ySrc * bmpWidth;
                for (x = xdest; x < right; ++x, ++pixelNumber) {
                    uint8_t byte = getMonoBitmapBitValue(pixelNumber, bmpMonoSource);
                    color = byte ? 0xFFFFFFFF : 0;
                    setPixel(color, x, y, dest, width, height);
                }
            }
        }
        break;
        case RasterOpCode::SRCAND: {
            for (y = ydest, ySrc = 0; y < bottom; ++y, ++ySrc) {
                pixelNumber = ySrc * bmpWidth;
                for (x = xdest; x < right; ++x, ++pixelNumber) {
                    uint8_t byte = getMonoBitmapBitValue(pixelNumber, bmpMonoSource);
                    color = byte ? 0xFFFFFFFF : 0;
                    prevColor = getPixel(x, y, dest, width, height);
                    setPixel(color & prevColor, x, y, dest, width, height);
                }
            }
        }
        break;
        case RasterOpCode::SRCOR: {
            for (y = ydest, ySrc = 0; y < bottom; ++y, ++ySrc) {
                pixelNumber = ySrc * bmpWidth;
                for (x = xdest; x < right; ++x, ++pixelNumber) {
                    uint8_t byte = getMonoBitmapBitValue(pixelNumber, bmpMonoSource);
                    color = byte ? 0xFFFFFFFF : 0;
                    prevColor = getPixel(x, y, dest, width, height);
                    setPixel(color | prevColor, x, y, dest, width, height);
                }
            }
        }
        break;
        case RasterOpCode::SRCXOR: {
            for (y = ydest, ySrc = 0; y < bottom; ++y, ++ySrc) {
                pixelNumber = ySrc * bmpWidth;
                for (x = xdest; x < right; ++x, ++pixelNumber) {
                    uint8_t byte = getMonoBitmapBitValue(pixelNumber, bmpMonoSource);
                    color = byte ? 0xFFFFFFFF : 0;
                    prevColor = getPixel(x, y, dest, width, height);
                    setPixel(color ^ prevColor, x, y, dest, width, height);
                }
            }
        }
        break;
        } // switch
    }
};

using SSD1306 = SSD1306Impl<I2CPollingClient>;

#endif // _SSD1306_H_INCLUDED_
