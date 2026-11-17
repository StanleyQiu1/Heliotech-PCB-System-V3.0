
/*
 * STM32G030F6Px - HelioTech Backup Board
 *
 * SSD1306 128x32 OLED Dinosaur Game
 * Isolated INA226 Voltage Monitoring
 * PA0 Buzzer Alarm
 *
 * PIN CONNECTIONS:
 *
 * PA12 / Pin 17 = OLED SDA + ISO1540 SDA
 * PA11 / Pin 16 = OLED SCL + ISO1540 SCL
 * PA7  / Pin 14 = Jump Button
 * PB0  / Pin 15 = Duck Button
 * PA0  / Pin  7 = Buzzer Transistor Control
 *
 * I2C SDA/SCL: external 4.7k pull-ups
 * Buttons: external 10k pull-ups
 *
 * BUZZER:
 * Active buzzer using transistor
 * PA0 HIGH = buzzer ON
 *
 * VOLTAGE:
 * Monitoring regulated 12V rail
 * Alarm ON at or below 11.4V
 * Alarm OFF at or above 11.7V
 *
 * This is test firmware, not a certified safety system.
 */

#include "main.h"
#include <stdint.h>
#include <string.h>

/* ========================================= */
/* GENERAL SETTINGS                          */
/* ========================================= */

#define OLED_W                  128
#define OLED_H                  32

#define GROUND_Y                31
#define DINO_X                  12

#define FRAME_MS                60U
#define MAX_OBSTACLES           3
#define FP                      16

/* ========================================= */
/* INA226 SETTINGS                           */
/* ========================================= */

#define INA_REG_BUS_V           0x02U
#define INA_REG_MANUFACTURER    0xFEU
#define INA_REG_DIE_ID          0xFFU

#define INA_POLL_MS             250U
#define INA_RESCAN_MS           2000U

/* ========================================= */
/* BUZZER ALARM SETTINGS                     */
/* ========================================= */

#define ALARM_TRIP_MV           11400U
#define ALARM_CLEAR_MV          11700U

#define ALARM_NO_DATA_MS        1500U

#define ALARM_TRIP_SAMPLES      3U
#define ALARM_CLEAR_SAMPLES     4U

#define BUZZER_ACTIVE_HIGH      1U

/* ========================================= */
/* I2C HANDLE                                */
/* ========================================= */

I2C_HandleTypeDef hi2c2;

/* ========================================= */
/* DEBUG VARIABLES                           */
/* ========================================= */

volatile uint32_t oled_ready = 0;
volatile uint32_t oled_errors = 0;

volatile uint32_t game_score = 0;
volatile uint32_t game_running = 0;

volatile uint32_t button_jump = 0;
volatile uint32_t button_duck = 0;

volatile uint32_t ina226_found = 0;
volatile uint32_t ina226_address7 = 0xFFU;

volatile uint32_t ina226_bus_mV = 0;
volatile uint32_t ina226_raw = 0;

volatile float ina226_voltage_v = 0.0f;

volatile uint32_t ina226_errors = 0;
volatile uint32_t ina226_read_count = 0;

volatile uint32_t ina226_manufacturer_id = 0;
volatile uint32_t ina226_die_id = 0;

/*
 * buzzer_on:
 * 0 = OFF
 * 1 = ON
 *
 * alarm_reason:
 * 0 = Normal
 * 1 = Low voltage
 * 2 = INA226 communication lost
 */

volatile uint32_t buzzer_on = 0;
volatile uint32_t alarm_reason = 0;

volatile uint32_t alarm_low_samples = 0;
volatile uint32_t alarm_good_samples = 0;

/* ========================================= */
/* INTERNAL VARIABLES                        */
/* ========================================= */

static uint32_t ina_last_good_ms = 0;
static uint32_t alarm_processed_reads = 0;

static uint32_t ina_last_read_ms = 0;
static uint32_t ina_last_probe_ms = 0;

static uint16_t oled_address = (0x3C << 1);

static uint8_t screen[512];
static uint8_t tx[513];

static uint32_t rng = 0x12345678UL;

/* ========================================= */
/* OBSTACLE STRUCTURE                        */
/* ========================================= */

typedef struct
{
    int16_t x;
    uint8_t type;
} Obstacle;

/* ========================================= */
/* FUNCTION DECLARATIONS                     */
/* ========================================= */

void SystemClock_Config(void);

static void Board_GPIO_Init(void);
static void Board_I2C2_Init(void);
static void Board_Service(void);

/* ========================================= */
/* BUTTON INPUT                              */
/* ========================================= */

static uint8_t JumpPressed(void)
{
    return (
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)
        == GPIO_PIN_RESET
    );
}

static uint8_t DuckPressed(void)
{
    return (
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)
        == GPIO_PIN_RESET
    );
}

static void WaitReleased(void)
{
    while (JumpPressed() || DuckPressed())
    {
        Board_Service();
        HAL_Delay(5);
    }

    Board_Service();
    HAL_Delay(20);
}

/* ========================================= */
/* RANDOM GENERATOR                          */
/* ========================================= */

static uint32_t RandomNumber(void)
{
    rng = rng * 1664525UL + 1013904223UL;

    return rng;
}

/* ========================================= */
/* OLED I2C TRANSMISSION                     */
/* ========================================= */

static void OLED_Transmit(
    const uint8_t *data,
    uint16_t size
)
{
    if (
        HAL_I2C_Master_Transmit(
            &hi2c2,
            oled_address,
            (uint8_t *)data,
            size,
            150
        ) != HAL_OK
    )
    {
        oled_errors++;
    }
}

/* ========================================= */
/* OLED COMMAND                              */
/* ========================================= */

static void OLED_Command(uint8_t command)
{
    uint8_t data[2];

    data[0] = 0x00;
    data[1] = command;

    OLED_Transmit(data, 2);
}

/* ========================================= */
/* OLED INITIALIZATION                       */
/* ========================================= */

static void OLED_Init(void)
{
    static const uint8_t commands[] =
    {
        0xAE,
        0x2E,

        0xD5, 0x80,
        0xA8, 0x1F,

        0xD3, 0x00,
        0x40,

        0x8D, 0x14,

        0x20, 0x00,

        0xA1,
        0xC8,

        0xDA, 0x02,

        0x81, 0x8F,

        0xD9, 0xF1,

        0xDB, 0x40,

        0xA4,
        0xA6,
        0xAF
    };

    HAL_Delay(100);

    oled_address = (0x3C << 1);

    if (
        HAL_I2C_IsDeviceReady(
            &hi2c2,
            oled_address,
            2,
            100
        ) != HAL_OK
    )
    {
        oled_address = (0x3D << 1);

        if (
            HAL_I2C_IsDeviceReady(
                &hi2c2,
                oled_address,
                2,
                100
            ) != HAL_OK
        )
        {
            oled_ready = 0;
            return;
        }
    }

    oled_ready = 1;

    for (
        uint32_t i = 0;
        i < sizeof(commands);
        i++
    )
    {
        OLED_Command(commands[i]);
    }
}

/* ========================================= */
/* INA226 REGISTER READ                      */
/* ========================================= */

static uint8_t INA226_ReadRegister(
    uint8_t addr7,
    uint8_t reg,
    uint16_t *value
)
{
    uint8_t bytes[2];

    HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read(
            &hi2c2,
            (uint16_t)(addr7 << 1),
            reg,
            I2C_MEMADD_SIZE_8BIT,
            bytes,
            2,
            50
        );

    if (status != HAL_OK)
    {
        return 0;
    }

    *value = (uint16_t)(
        ((uint16_t)bytes[0] << 8)
        | bytes[1]
    );

    return 1;
}

/* ========================================= */
/* INA226 BUS VOLTAGE                        */
/* ========================================= */

static uint8_t INA226_ReadBus(void)
{
    uint16_t raw;

    if (
        ina226_address7 > 0x4FU ||
        !INA226_ReadRegister(
            (uint8_t)ina226_address7,
            INA_REG_BUS_V,
            &raw
        )
    )
    {
        ina226_errors++;

        ina226_found = 0;
        ina226_bus_mV = 0;
        ina226_raw = 0;

        ina226_voltage_v = 0.0f;

        return 0;
    }

    ina226_raw = raw;

    /*
     * INA226:
     * Bus voltage resolution = 1.25mV
     */

    ina226_bus_mV =
        ((uint32_t)raw * 5U) / 4U;

    ina226_voltage_v =
        (float)raw * 0.00125f;

    ina226_read_count++;

    ina_last_good_ms = HAL_GetTick();

    return 1;
}

/* ========================================= */
/* INA226 DETECTION                          */
/* ========================================= */

static uint8_t INA226_Probe(void)
{
    uint16_t manufacturer;
    uint16_t die;

    ina_last_probe_ms = HAL_GetTick();

    ina226_found = 0;
    ina226_address7 = 0xFFU;

    ina226_manufacturer_id = 0;
    ina226_die_id = 0;

    for (
        uint8_t a = 0x40U;
        a <= 0x4FU;
        a++
    )
    {
        if (
            HAL_I2C_IsDeviceReady(
                &hi2c2,
                (uint16_t)(a << 1),
                1,
                4
            ) != HAL_OK
        )
        {
            continue;
        }

        if (
            !INA226_ReadRegister(
                a,
                INA_REG_MANUFACTURER,
                &manufacturer
            )
            ||
            !INA226_ReadRegister(
                a,
                INA_REG_DIE_ID,
                &die
            )
        )
        {
            continue;
        }

        if (
            manufacturer == 0x5449U &&
            (die & 0xFFF0U) == 0x2260U
        )
        {
            ina226_found = 1;
            ina226_address7 = a;

            ina226_manufacturer_id =
                manufacturer;

            ina226_die_id = die;

            ina_last_read_ms = HAL_GetTick();

            INA226_ReadBus();

            return (uint8_t)ina226_found;
        }
    }

    ina226_bus_mV = 0;
    ina226_raw = 0;
    ina226_voltage_v = 0.0f;

    return 0;
}

/* ========================================= */
/* INA226 POLLING                            */
/* ========================================= */

static void INA226_Poll(void)
{
    uint32_t now = HAL_GetTick();

    if (!ina226_found)
    {
        if (
            (uint32_t)(
                now - ina_last_probe_ms
            ) >= INA_RESCAN_MS
        )
        {
            INA226_Probe();
        }
    }
    else if (
        (uint32_t)(
            now - ina_last_read_ms
        ) >= INA_POLL_MS
    )
    {
        ina_last_read_ms = now;

        INA226_ReadBus();
    }
}

/* ========================================= */
/* BUZZER CONTROL                            */
/* ========================================= */

static void Buzzer_Set(
    uint8_t enabled,
    uint32_t reason
)
{
    GPIO_PinState state;

    if (BUZZER_ACTIVE_HIGH)
    {
        state = enabled
            ? GPIO_PIN_SET
            : GPIO_PIN_RESET;
    }
    else
    {
        state = enabled
            ? GPIO_PIN_RESET
            : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_0,
        state
    );

    buzzer_on = enabled ? 1U : 0U;

    alarm_reason =
        enabled ? reason : 0U;
}

/* ========================================= */
/* LOW VOLTAGE ALARM                         */
/* ========================================= */

static void Alarm_Update(void)
{
    uint32_t now = HAL_GetTick();

    /*
     * No valid INA226 measurement
     * for 1500 milliseconds.
     *
     * This can indicate power loss
     * OR communication failure.
     */

    if (
        (uint32_t)(
            now - ina_last_good_ms
        ) >= ALARM_NO_DATA_MS
    )
    {
        alarm_low_samples = 0;
        alarm_good_samples = 0;

        Buzzer_Set(1U, 2U);

        return;
    }

    /*
     * Only process each successful
     * measurement once.
     */

    if (
        ina226_read_count ==
        alarm_processed_reads
    )
    {
        return;
    }

    alarm_processed_reads =
        ina226_read_count;

    /* Voltage too low */

    if (
        ina226_bus_mV <=
        ALARM_TRIP_MV
    )
    {
        alarm_good_samples = 0;

        if (
            alarm_low_samples <
            ALARM_TRIP_SAMPLES
        )
        {
            alarm_low_samples++;
        }

        if (
            alarm_low_samples >=
            ALARM_TRIP_SAMPLES
        )
        {
            Buzzer_Set(1U, 1U);
        }
    }

    /* Voltage recovered */

    else if (
        ina226_bus_mV >=
        ALARM_CLEAR_MV
    )
    {
        alarm_low_samples = 0;

        if (
            alarm_good_samples <
            ALARM_CLEAR_SAMPLES
        )
        {
            alarm_good_samples++;
        }

        if (
            alarm_good_samples >=
            ALARM_CLEAR_SAMPLES
        )
        {
            Buzzer_Set(0U, 0U);
        }
    }

    /* Hysteresis region */

    else
    {
        alarm_low_samples = 0;
        alarm_good_samples = 0;
    }
}

/* ========================================= */
/* BACKUP BOARD SERVICE                      */
/* ========================================= */

static void Board_Service(void)
{
    INA226_Poll();
    Alarm_Update();
}

/* ========================================= */
/* VOLTAGE DISPLAY FORMAT                    */
/* ========================================= */

static void VoltageText(char out[12])
{
    if (!ina226_found)
    {
        strcpy(out, "NO INA");

        return;
    }

    uint32_t cv =
        (ina226_bus_mV + 5U) / 10U;

    uint32_t whole =
        cv / 100U;

    uint32_t fraction =
        cv % 100U;

    out[0] = 'V';
    out[1] = '=';

    out[2] = (char)(
        '0' + (whole / 10U) % 10U
    );

    out[3] = (char)(
        '0' + whole % 10U
    );

    out[4] = '.';

    out[5] = (char)(
        '0' + fraction / 10U
    );

    out[6] = (char)(
        '0' + fraction % 10U
    );

    out[7] = '\0';
}

/* ========================================= */
/* GRAPHICS                                  */
/* ========================================= */

static void ClearScreen(void)
{
    memset(
        screen,
        0,
        sizeof(screen)
    );
}

static void Pixel(
    int16_t x,
    int16_t y,
    uint8_t on
)
{
    if (
        x < 0 ||
        x >= OLED_W ||
        y < 0 ||
        y >= OLED_H
    )
    {
        return;
    }

    uint16_t index =
        (uint16_t)(
            (y / 8) * OLED_W + x
        );

    uint8_t mask =
        (uint8_t)(
            1U << (y & 7)
        );

    if (on)
    {
        screen[index] |= mask;
    }
    else
    {
        screen[index] &=
            (uint8_t)~mask;
    }
}

static void Rectangle(
    int16_t x,
    int16_t y,
    int16_t w,
    int16_t h
)
{
    for (
        int16_t row = 0;
        row < h;
        row++
    )
    {
        for (
            int16_t col = 0;
            col < w;
            col++
        )
        {
            Pixel(
                x + col,
                y + row,
                1
            );
        }
    }
}

/* ========================================= */
/* OLED UPDATE                               */
/* ========================================= */

static void OLED_Show(void)
{
    uint8_t address_window[] =
    {
        0x00,
        0x21, 0, 127,
        0x22, 0, 3
    };

    OLED_Transmit(
        address_window,
        sizeof(address_window)
    );

    tx[0] = 0x40;

    memcpy(
        &tx[1],
        screen,
        sizeof(screen)
    );

    OLED_Transmit(
        tx,
        sizeof(tx)
    );
}

/* ========================================= */
/* FONT - LETTER D FIXED                    */
/* ========================================= */

static void Glyph(
    char c,
    uint8_t out[5]
)
{
    static const uint8_t digit[10][5] =
    {
        {0x3E,0x51,0x49,0x45,0x3E}, // 0
        {0x00,0x42,0x7F,0x40,0x00}, // 1
        {0x62,0x51,0x49,0x49,0x46}, // 2
        {0x22,0x41,0x49,0x49,0x36}, // 3
        {0x18,0x14,0x12,0x7F,0x10}, // 4
        {0x27,0x45,0x45,0x45,0x39}, // 5
        {0x3C,0x4A,0x49,0x49,0x30}, // 6
        {0x01,0x71,0x09,0x05,0x03}, // 7
        {0x36,0x49,0x49,0x49,0x36}, // 8
        {0x06,0x49,0x49,0x29,0x1E}  // 9
    };

    /*
     * Correct font for A-Z.
     *
     * Previously letter D was incorrectly
     * defined using the same shape as E.
     *
     * Correct D:
     * {0x7F,0x41,0x41,0x22,0x1C}
     */

    static const uint8_t letter[26][5] =
    {
        {0x7E,0x11,0x11,0x11,0x7E}, // A
        {0x7F,0x49,0x49,0x49,0x36}, // B
        {0x3E,0x41,0x41,0x41,0x22}, // C

        {0x7F,0x41,0x41,0x22,0x1C}, // D FIXED

        {0x7F,0x49,0x49,0x49,0x41}, // E
        {0x7F,0x09,0x09,0x09,0x01}, // F
        {0x3E,0x41,0x49,0x49,0x7A}, // G
        {0x7F,0x08,0x08,0x08,0x7F}, // H
        {0x00,0x41,0x7F,0x41,0x00}, // I
        {0x20,0x40,0x41,0x3F,0x01}, // J
        {0x7F,0x08,0x14,0x22,0x41}, // K
        {0x7F,0x40,0x40,0x40,0x40}, // L
        {0x7F,0x02,0x04,0x02,0x7F}, // M
        {0x7F,0x04,0x08,0x10,0x7F}, // N
        {0x3E,0x41,0x41,0x41,0x3E}, // O
        {0x7F,0x09,0x09,0x09,0x06}, // P
        {0x3E,0x41,0x51,0x21,0x5E}, // Q
        {0x7F,0x09,0x19,0x29,0x46}, // R
        {0x46,0x49,0x49,0x49,0x31}, // S
        {0x01,0x01,0x7F,0x01,0x01}, // T
        {0x3F,0x40,0x40,0x40,0x3F}, // U
        {0x1F,0x20,0x40,0x20,0x1F}, // V
        {0x3F,0x40,0x38,0x40,0x3F}, // W
        {0x63,0x14,0x08,0x14,0x63}, // X
        {0x03,0x04,0x78,0x04,0x03}, // Y
        {0x61,0x51,0x49,0x45,0x43}  // Z
    };

    memset(out, 0, 5);

    if (c >= '0' && c <= '9')
    {
        memcpy(
            out,
            digit[c - '0'],
            5
        );
    }
    else if (c >= 'A' && c <= 'Z')
    {
        memcpy(
            out,
            letter[c - 'A'],
            5
        );
    }
    else if (c == '-')
    {
        for (
            uint8_t i = 0;
            i < 5;
            i++
        )
        {
            out[i] = 0x08;
        }
    }
    else if (c == '=')
    {
        for (
            uint8_t i = 0;
            i < 5;
            i++
        )
        {
            out[i] = 0x14;
        }
    }
}

/* ========================================= */
/* TEXT DRAWING                              */
/* ========================================= */

static void Text(
    int16_t x,
    int16_t y,
    const char *s
)
{
    while (*s)
    {
        uint8_t f[5];

        Glyph(*s++, f);

        for (
            uint8_t a = 0;
            a < 5;
            a++
        )
        {
            for (
                uint8_t b = 0;
                b < 7;
                b++
            )
            {
                if (
                    (f[a] & (1U << b)) != 0U
                )
                {
                    Pixel(
                        x + a,
                        y + b,
                        1
                    );
                }
            }
        }

        x += 6;
    }
}

static void CenterText(
    int16_t y,
    const char *s
)
{
    int16_t width =
        (int16_t)strlen(s) * 6;

    Text(
        (OLED_W - width) / 2,
        y,
        s
    );
}

/* ========================================= */
/* NUMBER TO TEXT                            */
/* ========================================= */

static void NumberText(
    uint32_t value,
    char out[11]
)
{
    char reverse[10];

    uint8_t n = 0;

    do
    {
        reverse[n++] =
            (char)('0' + value % 10U);

        value /= 10U;

    } while (
        value != 0U &&
        n < 10
    );

    uint8_t length = n;

    for (
        uint8_t i = 0;
        i < length;
        i++
    )
    {
        out[i] = reverse[--n];
    }

    out[length] = '\0';
}

/* ========================================= */
/* DINOSAUR SPRITE                           */
/* ========================================= */

static void Dino(
    int16_t x,
    int16_t y,
    uint8_t duck,
    uint8_t leg
)
{
    if (duck)
    {
        Rectangle(x + 3, y, 5, 1);
        Rectangle(x + 1, y + 1, 8, 2);
        Rectangle(x, y + 3, 6, 1);

        Pixel(
            x + (leg ? 2 : 1),
            y + 4,
            1
        );

        Pixel(
            x + (leg ? 6 : 5),
            y + 4,
            1
        );

        Pixel(x + 7, y + 1, 0);
    }
    else
    {
        Rectangle(x + 2, y, 4, 2);
        Rectangle(x + 1, y + 2, 5, 2);
        Rectangle(x, y + 4, 4, 2);
        Rectangle(x + 1, y + 6, 3, 1);

        Pixel(x, y + 3, 1);
        Pixel(x + 4, y, 0);

        Pixel(
            x + (leg ? 1 : 3),
            y + 7,
            1
        );

        Pixel(
            x + (leg ? 4 : 0),
            y + 6,
            1
        );
    }
}

/* ========================================= */
/* CACTUS SPRITE                             */
/* ========================================= */

static void Cactus(int16_t x)
{
    int16_t y = GROUND_Y - 8;

    Rectangle(x + 1, y, 2, 8);
    Rectangle(x, y + 2, 1, 3);
    Rectangle(x + 3, y + 1, 1, 3);
}

/* ========================================= */
/* BIRD SPRITE                               */
/* ========================================= */

static void Bird(
    int16_t x,
    uint8_t wing
)
{
    const int16_t y = 22;

    Rectangle(x + 2, y + 1, 5, 2);

    Pixel(x + 7, y + 1, 1);

    Rectangle(x, y + 2, 2, 1);

    Rectangle(
        x + (wing ? 2 : 3),
        y + (wing ? 0 : 3),
        2,
        1
    );
}

/* ========================================= */
/* OBSTACLE WIDTH                            */
/* ========================================= */

static int16_t ObstacleWidth(
    uint8_t type
)
{
    return type ? 8 : 4;
}

/* ========================================= */
/* COLLISION DETECTION                       */
/* ========================================= */

static uint8_t Collision(
    int16_t dy,
    uint8_t duck,
    int16_t ox,
    uint8_t type
)
{
    int16_t dl = DINO_X + 1;

    int16_t dr =
        DINO_X + (duck ? 9 : 6) - 2;

    int16_t dt = dy + 1;

    int16_t db =
        dy + (duck ? 5 : 8) - 1;

    int16_t ol = ox + 1;

    int16_t oright =
        ox + ObstacleWidth(type) - 1;

    int16_t ot =
        type ? 22 : GROUND_Y - 8 + 1;

    int16_t ob =
        type ? 25 : GROUND_Y - 1;

    return (
        dl <= oright &&
        dr >= ol &&
        dt <= ob &&
        db >= ot
    );
}

/* ========================================= */
/* TITLE SCREEN                              */
/* ========================================= */

static void TitleScreen(void)
{
    uint32_t last_draw_ms = 0;

    char voltage[12];

    for (;;)
    {
        Board_Service();

        button_jump = JumpPressed();
        button_duck = DuckPressed();

        if (button_jump || button_duck)
        {
            WaitReleased();
            return;
        }

        if (
            (uint32_t)(
                HAL_GetTick() - last_draw_ms
            ) >= 300U ||
            last_draw_ms == 0U
        )
        {
            last_draw_ms = HAL_GetTick();

            VoltageText(voltage);

            ClearScreen();

            /* Corrected DINO RUN text */
            CenterText(0, "DINO RUN");

            CenterText(11, voltage);

            CenterText(22, "JUMP OR DUCK");

            OLED_Show();
        }

        HAL_Delay(10);
    }
}

/* ========================================= */
/* GAME OVER                                 */
/* ========================================= */

static uint8_t GameOver(
    uint32_t score
)
{
    char number[11];
    char voltage[12];

    uint32_t last_draw_ms = 0;

    NumberText(score, number);

    WaitReleased();

    for (;;)
    {
        Board_Service();

        if (
            (uint32_t)(
                HAL_GetTick() - last_draw_ms
            ) >= 300U ||
            last_draw_ms == 0U
        )
        {
            last_draw_ms = HAL_GetTick();

            VoltageText(voltage);

            ClearScreen();

            CenterText(0, "GAME OVER");

            Text(0, 9, voltage);

            Text(84, 9, number);

            CenterText(18, "JUMP RESTART");
            CenterText(25, "DUCK MENU");

            OLED_Show();
        }

        if (JumpPressed())
        {
            HAL_Delay(20);

            if (JumpPressed())
            {
                WaitReleased();
                return 1;
            }
        }

        if (DuckPressed())
        {
            HAL_Delay(20);

            if (DuckPressed())
            {
                WaitReleased();
                return 0;
            }
        }

        HAL_Delay(5);
    }
}

/* ========================================= */
/* DINOSAUR GAME                             */
/* ========================================= */

static void Game(void)
{
    for (;;)
    {
        Obstacle obstacles[MAX_OBSTACLES];

        int16_t start_x = OLED_W;

        int16_t y_fp =
            (GROUND_Y - 8) * FP;

        int16_t velocity = 0;
        int16_t speed = 32;

        uint8_t jumping = 0;
        uint8_t old_jump = 0;

        uint32_t frame = 0;

        game_score = 0;
        game_running = 1;

        rng ^= HAL_GetTick();

        /* Initialize obstacles */

        for (
            uint8_t i = 0;
            i < MAX_OBSTACLES;
            i++
        )
        {
            obstacles[i].type =
                (uint8_t)(
                    (RandomNumber() >> 16)
                    & 1U
                );

            obstacles[i].x =
                start_x * FP;

            start_x +=
                ObstacleWidth(
                    obstacles[i].type
                )
                + 20
                + (int16_t)(
                    (RandomNumber() >> 16)
                    % 20U
                );
        }

        WaitReleased();

        /* Main game loop */

        for (;;)
        {
            uint32_t start_ms =
                HAL_GetTick();

            /*
             * Monitor voltage and update
             * buzzer while game runs.
             */

            Board_Service();

            uint8_t jump =
                JumpPressed();

            uint8_t duck_button =
                DuckPressed();

            button_jump = jump;
            button_duck = duck_button;

            /* Jump physics */

            if (
                jump &&
                !old_jump &&
                !jumping
            )
            {
                jumping = 1;
                velocity = -60;
            }

            old_jump = jump;

            if (jumping)
            {
                velocity += 9;
                y_fp += velocity;

                if (
                    y_fp >=
                    (GROUND_Y - 8) * FP
                )
                {
                    y_fp =
                        (GROUND_Y - 8) * FP;

                    velocity = 0;
                    jumping = 0;
                }
            }

            /* Ducking */

            uint8_t duck =
                (uint8_t)(
                    duck_button && !jumping
                );

            int16_t dy =
                duck
                    ? GROUND_Y - 5
                    : y_fp / FP;

            /* Move obstacles */

            for (
                uint8_t i = 0;
                i < MAX_OBSTACLES;
                i++
            )
            {
                obstacles[i].x -= speed;
            }

            /* Respawn obstacles */

            for (
                uint8_t i = 0;
                i < MAX_OBSTACLES;
                i++
            )
            {
                int16_t x =
                    obstacles[i].x / FP;

                if (
                    x <
                    -ObstacleWidth(
                        obstacles[i].type
                    )
                )
                {
                    int16_t rightmost =
                        OLED_W;

                    for (
                        uint8_t j = 0;
                        j < MAX_OBSTACLES;
                        j++
                    )
                    {
                        if (j == i)
                        {
                            continue;
                        }

                        int16_t edge =
                            obstacles[j].x / FP
                            + ObstacleWidth(
                                obstacles[j].type
                            );

                        if (
                            edge > rightmost
                        )
                        {
                            rightmost = edge;
                        }
                    }

                    obstacles[i].type =
                        (uint8_t)(
                            (RandomNumber() >> 16)
                            & 1U
                        );

                    obstacles[i].x =
                        (int16_t)(
                            rightmost
                            + 20
                            + (int16_t)(
                                (RandomNumber() >> 16)
                                % 20U
                            )
                        ) * FP;

                    if (speed < 56)
                    {
                        speed++;
                    }
                }
            }

            /* Collision detection */

            uint8_t crashed = 0;

            for (
                uint8_t i = 0;
                i < MAX_OBSTACLES;
                i++
            )
            {
                if (
                    Collision(
                        dy,
                        duck,
                        obstacles[i].x / FP,
                        obstacles[i].type
                    )
                )
                {
                    crashed = 1;
                    break;
                }
            }

            if (crashed)
            {
                game_running = 0;

                if (GameOver(game_score))
                {
                    break;
                }

                return;
            }

            /* Score */

            if ((frame % 6U) == 0U)
            {
                game_score++;
            }

            /* Render screen */

            ClearScreen();

            char score_string[11];
            char voltage_string[12];

            NumberText(
                game_score,
                score_string
            );

            VoltageText(voltage_string);

            Text(
                0,
                0,
                voltage_string
            );

            Text(
                127 -
                (int16_t)strlen(
                    score_string
                ) * 6,
                0,
                score_string
            );

            /* Ground */

            for (
                int16_t x = 0;
                x < OLED_W;
                x++
            )
            {
                Pixel(
                    x,
                    GROUND_Y,
                    1
                );
            }

            /* Ground marks */

            for (
                int16_t x =
                    5 - (int16_t)(
                        frame % 12U
                    );
                x < OLED_W;
                x += 20
            )
            {
                Pixel(
                    x,
                    GROUND_Y - 1,
                    1
                );
            }

            /* Dinosaur */

            Dino(
                DINO_X,
                dy,
                duck,
                (uint8_t)(
                    (frame >> 2) & 1U
                )
            );

            /* Obstacles */

            for (
                uint8_t i = 0;
                i < MAX_OBSTACLES;
                i++
            )
            {
                int16_t x =
                    obstacles[i].x / FP;

                if (
                    obstacles[i].type
                )
                {
                    Bird(
                        x,
                        (uint8_t)(
                            (frame >> 2) & 1U
                        )
                    );
                }
                else
                {
                    Cactus(x);
                }
            }

            OLED_Show();

            frame++;

            /* Frame timing */

            uint32_t elapsed =
                HAL_GetTick() - start_ms;

            if (
                elapsed <
                FRAME_MS
            )
            {
                HAL_Delay(
                    FRAME_MS - elapsed
                );
            }
        }
    }
}

/* ========================================= */
/* MAIN                                      */
/* ========================================= */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    Board_GPIO_Init();
    Board_I2C2_Init();

    /*
     * Start INA226 communication
     * loss detection timer.
     */

    ina_last_good_ms =
        HAL_GetTick();

    /* Initialize OLED and INA226 */

    OLED_Init();
    INA226_Probe();

    Board_Service();

    /*
     * If OLED fails, keep the
     * voltage alarm operating.
     */

    if (!oled_ready)
    {
        for (;;)
        {
            Board_Service();
            HAL_Delay(10);
        }
    }

    /* Start dinosaur game */

    for (;;)
    {
        TitleScreen();
        Game();
    }
}

/* ========================================= */
/* SYSTEM CLOCK                              */
/* ========================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    osc.HSIState =
        RCC_HSI_ON;

    osc.HSIDiv =
        RCC_HSI_DIV1;

    osc.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    if (
        HAL_RCC_OscConfig(&osc)
        != HAL_OK
    )
    {
        Error_Handler();
    }

    clk.ClockType =
        RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_HCLK
        | RCC_CLOCKTYPE_PCLK1;

    clk.SYSCLKSource =
        RCC_SYSCLKSOURCE_HSI;

    clk.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    clk.APB1CLKDivider =
        RCC_HCLK_DIV1;

    if (
        HAL_RCC_ClockConfig(
            &clk,
            FLASH_LATENCY_0
        ) != HAL_OK
    )
    {
        Error_Handler();
    }
}

/* ========================================= */
/* GPIO INITIALIZATION                       */
/* ========================================= */

static void Board_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA7 = Jump button */

    gpio.Pin =
        GPIO_PIN_7;

    gpio.Mode =
        GPIO_MODE_INPUT;

    gpio.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        GPIOA,
        &gpio
    );

    /* PB0 = Duck button */

    gpio.Pin =
        GPIO_PIN_0;

    HAL_GPIO_Init(
        GPIOB,
        &gpio
    );

    /*
     * PA0 = Buzzer transistor control.
     *
     * Set the output LOW before
     * enabling output mode.
     */

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_0,
        BUZZER_ACTIVE_HIGH
            ? GPIO_PIN_RESET
            : GPIO_PIN_SET
    );

    gpio.Pin =
        GPIO_PIN_0;

    gpio.Mode =
        GPIO_MODE_OUTPUT_PP;

    gpio.Pull =
        GPIO_NOPULL;

    gpio.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOA,
        &gpio
    );
}

/* ========================================= */
/* I2C2 INITIALIZATION                       */
/* ========================================= */

static void Board_I2C2_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    /*
     * OLED and ISO1540 share I2C2.
     *
     * PA11 = SCL
     * PA12 = SDA
     *
     * AF6
     */

    gpio.Pin =
        GPIO_PIN_11 |
        GPIO_PIN_12;

    gpio.Mode =
        GPIO_MODE_AF_OD;

    gpio.Pull =
        GPIO_NOPULL;

    gpio.Speed =
        GPIO_SPEED_FREQ_LOW;

    gpio.Alternate =
        GPIO_AF6_I2C2;

    HAL_GPIO_Init(
        GPIOA,
        &gpio
    );

    /* Configure I2C2 */

    hi2c2.Instance =
        I2C2;

    hi2c2.Init.Timing =
        0x30420F13;

    hi2c2.Init.OwnAddress1 =
        0;

    hi2c2.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;

    hi2c2.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;

    hi2c2.Init.OwnAddress2 =
        0;

    hi2c2.Init.OwnAddress2Masks =
        I2C_OA2_NOMASK;

    hi2c2.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;

    hi2c2.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;

    if (
        HAL_I2C_Init(&hi2c2)
        != HAL_OK
    )
    {
        Error_Handler();
    }

    if (
        HAL_I2CEx_ConfigAnalogFilter(
            &hi2c2,
            I2C_ANALOGFILTER_ENABLE
        ) != HAL_OK
    )
    {
        Error_Handler();
    }

    if (
        HAL_I2CEx_ConfigDigitalFilter(
            &hi2c2,
            0
        ) != HAL_OK
    )
    {
        Error_Handler();
    }
}

/* ========================================= */
/* ERROR HANDLER                             */
/* ========================================= */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

/* ========================================= */
/* ASSERT                                    */
/* ========================================= */

#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line
)
{
    (void)file;
    (void)line;
}

#endif
