#include <Adafruit_GFX.h>
#include <SAAB_ICM2.h>

#ifdef __AVR__
#include <avr/pgmspace.h>
#elif defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#endif

// Many (but maybe not all) non-AVR board installs define macros
// for compatibility with existing PROGMEM-reading AVR code.
// Do our own checks and defines here for good measure...

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
#ifndef pgm_read_word
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif
#ifndef pgm_read_dword
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#endif

/*!
    @brief  Constructor for the I2C-interfaced ICM2 display.
    @param  w
            Display width in pixels (106)
    @param  h
            Display height in pixels (65)
    @param  twi
            Pointer to an existing TwoWire instance (e.g. &Wire, the
            microcontroller's primary I2C bus).
*/
SAAB_ICM2::SAAB_ICM2() : Adafruit_GFX(_width, _height), buffer(NULL), shadowBuffer(NULL) {}

/*!
    @brief  Destructor for Adafruit_SSD1306 object.
*/
SAAB_ICM2::~SAAB_ICM2(void)
{
    if (buffer)
    {
        free(buffer);
        buffer = NULL;
    }

    if (shadowBuffer)
    {
        free(shadowBuffer);
        shadowBuffer = NULL;
    }
}

void SAAB_ICM2::icm2_command1(uint8_t c)
{
    // I2C
    Wire.beginTransmission(i2caddr);
    Wire.write(c);
    Wire.endTransmission();
}

void SAAB_ICM2::icm2_commandList(const uint8_t *c, uint8_t n)
{
    Wire.beginTransmission(i2caddr);
    Wire.write(c, n);
    Wire.endTransmission();
}

// A public version of icm2_command1(), for existing user code that
// might rely on that function. This encapsulates the command transfer
// in a transaction start/end, similar to old library's handling of it.
/*!
    @brief  Issue a single low-level command directly to the ICM2
            display, bypassing the library.
    @param  c
            Command to issue (0x00 to 0xFF, see datasheet).
    @return None (void).
*/
void SAAB_ICM2::icm2_command(uint8_t c)
{
    icm2_command1(c);
}

// Private variable getters
int8_t SAAB_ICM2::width()
{
    return _width;
}

int8_t SAAB_ICM2::height()
{
    return _height;
}

void SAAB_ICM2::sendDisplayRow(const uint8_t *source, uint8_t row)
{
    if (!source)
    {
        return;
    }

    uint8_t lineAddr = 0x40 + row;
    uint8_t dat[3] = {0x00, 0x01, lineAddr};

    icm2_commandList(dat, sizeof(dat));
    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x20}, 3);
    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x8d}, 3);

    Wire.beginTransmission(i2caddr);
    Wire.write((uint8_t)0x40);

    if (row == 8)
    {
        const uint8_t *ptr = source;
        for (int x = 0; x < _width; x++)
        {
            Wire.write(*ptr++ & 0b10000000);
        }
    }
    else
    {
        const uint8_t *current = source + (row * _width);
        const uint8_t *next = current + _width;

        for (int x = 0; x < _width; x++)
        {
            Wire.write(((current[x] & 127) << 1) | ((next[x] & 0b10000000) >> 7));
        }
    }

    Wire.endTransmission();
}

void SAAB_ICM2::flushIfDirty(const uint8_t *source)
{
    if (!source || !shadowBuffer)
    {
        return;
    }

    const size_t frameSize = _width * ((_height + 7) / 8);
    bool dirtyRows[9] = {false, false, false, false, false, false, false, false, false};

    for (uint8_t page = 0; page < 9; page++)
    {
        const uint8_t *sourcePage = source + (page * _width);
        const uint8_t *shadowPage = shadowBuffer + (page * _width);

        if (memcmp(sourcePage, shadowPage, _width) != 0)
        {
            if (page == 0)
            {
                dirtyRows[8] = true;
                dirtyRows[0] = true;
            }
            else if (page < 8)
            {
                dirtyRows[page - 1] = true;
                dirtyRows[page] = true;
            }
            else
            {
                dirtyRows[7] = true;
            }
        }
    }

    if (dirtyRows[8])
    {
        sendDisplayRow(source, 8);
    }

    for (uint8_t row = 0; row < 8; row++)
    {
        if (dirtyRows[row])
        {
            sendDisplayRow(source, row);
        }
    }

    memcpy(shadowBuffer, source, frameSize);
}

void SAAB_ICM2::flushFull(const uint8_t *source)
{
    if (!source)
    {
        return;
    }

    sendDisplayRow(source, 8);
    for (uint8_t row = 0; row < 8; row++)
    {
        sendDisplayRow(source, row);
    }

    if (shadowBuffer)
    {
        const size_t frameSize = _width * ((_height + 7) / 8);
        memcpy(shadowBuffer, source, frameSize);
    }
}

bool SAAB_ICM2::begin(void)
{
    const size_t frameSize = _width * ((_height + 7) / 8);
    bool allocatedBuffer = false;

    if (!buffer)
    {
        buffer = (uint8_t *)malloc(frameSize);
        if (!buffer)
            return false;
        allocatedBuffer = true;
    }

    if (!shadowBuffer)
    {
        shadowBuffer = (uint8_t *)malloc(frameSize);
        if (!shadowBuffer)
        {
            if (allocatedBuffer)
            {
                free(buffer);
                buffer = NULL;
            }
            return false;
        }
    }

    // Zero the framebuffer
    clearDisplay();
    memset(shadowBuffer, 0xFF, frameSize);

    // run all the init commands
    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x10}, 3);
    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x0e, 0x12}, 4);
    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x0e, 0x06}, 4);
    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x0e, 0x0a}, 4);

    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x0e, 0x24}, 4);
    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x0e, 0x84}, 4);
    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x0d, 0x0a}, 4);
    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x0d, 0x13}, 4);

    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x0d, 0xb7}, 4);
    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x0d, 0x07}, 4);
    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x0b, 0x04}, 4);
    icm2_commandList((const uint8_t[]){0x00, 0x01, 0x0b, 0x40}, 4);

    return true; // Success
}

// DRAWING FUNCTIONS

/*!
    @brief  Set/clear/invert a single pixel. This is also invoked by the
            Adafruit_GFX library in generating many higher-level graphics
            primitives.
    @param  x
            Column of display -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @param  color
            Pixel color, one of: ICM2_ON, ICM2_OFF, ICM2_INVERSE
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void SAAB_ICM2::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    if ((x >= 0) && (x < _width) && (y >= 0) && (y < _height))
    {
        // Upgraded to do bytewise y-flipping
        switch (color)
        {
        case ICM2_ON:
            buffer[x + (y / 8) * _width] |= (128 >> (y & 7));
            break;
        case ICM2_OFF:
            buffer[x + (y / 8) * _width] &= ~(128 >> (y & 7));
            break;
        case ICM2_INVERSE:
            buffer[x + (y / 8) * _width] ^= (128 >> (y & 7));
            break;
        }
    }
}

/**************************************************************************/
/*!
    @brief  Fill the framebuffer completely with one color
    @param  color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void SAAB_ICM2::fillScreen(uint16_t color) {
    memset(buffer, color ? 0xFF : 0x00, _width * ((_height + 7) / 8));
}

/*!
    @brief  Get base address of display buffer for direct reading or writing.
    @return Pointer to an unsigned 8-bit array, column-major, columns padded
            to full byte boundary if needed.
*/
uint8_t *SAAB_ICM2::getBuffer(void)
{
    return buffer;
}

/*!
    @brief  Clear contents of display buffer (set all pixels to off).
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void SAAB_ICM2::clearDisplay(void)
{
    if (buffer)
    {
        memset(buffer, 0, _width * ((_height + 7) / 8));
    }
}

void SAAB_ICM2::display(void)
{
    flushFull(buffer);
}

void SAAB_ICM2::displayPartial(void)
{
    flushIfDirty(buffer);
}

void SAAB_ICM2::invalidateShadow(void)
{
    if (!shadowBuffer)
    {
        return;
    }

    memset(shadowBuffer, 0xFF, _width * ((_height + 7) / 8));
}

void SAAB_ICM2::forceClear(void)
{
    for (int line = 0; line < 9; line++)
    {
        uint8_t lineAddr = 0x40 + line;
        uint8_t dat[3] = {0x00, 0x01, lineAddr};
        icm2_commandList(dat, sizeof(dat));
        icm2_commandList((const uint8_t[]){0x00, 0x01, 0x20}, 3);
        icm2_commandList((const uint8_t[]){0x00, 0x01, 0x8d}, 3);

        Wire.beginTransmission(i2caddr);
        Wire.write((uint8_t)0x40);
        for (int i = 0; i < _width; i++)
        {
            Wire.write(0);
        }
        Wire.endTransmission();
    }

    if (shadowBuffer)
    {
        memset(shadowBuffer, 0, _width * ((_height + 7) / 8));
    }
}

void SAAB_ICM2::sendBuffer(uint8_t *extBuffer)
{
    flushFull(extBuffer);
}