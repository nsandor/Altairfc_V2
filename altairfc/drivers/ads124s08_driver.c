#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>

/* ------------------------------------------------------------------ */
/* ads124s08 command bytes                                              */
/* ------------------------------------------------------------------ */
#define CMD_WAKEUP     0x02
#define CMD_POWERDOWN  0x04
#define CMD_RESET      0x06
#define CMD_START      0x08
#define CMD_STOP       0x0A

#define CMD_RDATA      0x12

#define CMD_RREG0      0x20   /* RREG starting at reg 0, 1 register (0010 00 00) */
#define CMD_WREG0      0x40   /* WREG starting at reg 0, 1 register (0100 00 00) */

/* Input multiplexer register (02h): MUXP[3:0] MUXN [3:0]*/
/* All measurements are differential with reference to the external 2.5V reference on pin AIN0 to give a 0-5V range*/
#define MUX_VGND    0x20    /* Differential read, AIN2 (Virtual Ground)(+) AIN0 (2.5V ref)(-)*/
#define MUX_IVC     0x30    /* Differential read, AIN3(IVC level shift output)(+) AIN0 (2.5V ref)(-)*/
#define MUX_ACF     0x40    /* Differential read, AIN4(ACF level shift output)(+) AIN0 (2.5V ref)(-)*/  
#define MUX_TIA     0x50    /* Differential read, AIN5(TIA output)(+) AIN0 (2.5V ref)(-)*/
#define MUX_BOARD_TMP 0x60    /* Differential read, AIN6(Board Temp Sensor)(+) AIN0 (2.5V ref)(-)*/
#define MUX_PD_TMP 0x70    /* Differential read, AIN7(PD Temp Sensor)(+) AIN0 (2.5V ref)(-)*/

/* Gain Setting Register (03h): DELAY[2:0] PGA_EN[1:0] GAIN[2:0] */
/* We bypass the PGA. Not much else to do here.*/
#define BYPASS_PGA 0x00

/* Data Rate Register (04h): G_CHOP CLK MODE FILTER DR[3:0]*/
/* We always use the external clock source. 
    Highest accuracy measurement with global chop enabled, sinc3 filter and 2.5 SPS datarate. */
#define G_CHOP_BIT 0x80
#define CLK_EXTERNAL_BIT 0x40
#define SINGLE_SHOT_BIT 0x20
#define FILTER_SINC3_BIT 0x10

#define DR2_5SPS 0x00
#define DR5SPS 0x01
#define DR10SPS 0x02
#define DR16_6SPS 0x03
#define DR20SPS 0x04
#define DR50SPS 0x05
#define DR60SPS 0x06
#define DR100SPS 0x07
#define DR200SPS 0x08
#define DR400SPS 0x09
#define DR800SPS 0x0A
#define DR1000SPS 0x0B
#define DR2000SPS 0x0C
#define DR4000SPS 0x0D

/*  Reference Control Register (05h): FL_REF_EN[1:0] REFP_BUF REFN_BUF REFSEL[1:0] REFCON[1:0]  */
/* We do not use the internally generated reference and leave it off, so leaving it as all defaults in this register is fine, using REFP0 (2.5V external) and REFN0 (GND) */

/* Skipping over the excitation, bias, system, and calibration registers. We don't use them for now.*/

/* GPIO Data Register (10h): DIR[3:0] DAT[3:0] */
/* GPIO pin control (default all low). These are tied to the reed relays for signal routing. Direction stays default (output)*/
#define GPIO_ACF 0x01
#define GPIO_IVC 0x02
#define GPIO_TIA 0x04
#define GPIO_TIA_LOWGAIN 0x08 // note that the tia low gain path requires both the TIA relay and the gain relay to be on.

/* GPIO config register (11h): 0000 Con[3:0]*/
/* We turn all of the GPIOs on.*/
#define GPIO_ON_ALL 0x0F

#define VREF_V            2.5f
#define FULL_SCALE_CODE   8388608.0f  /* 2^23 */

/* Board and diode thermistor divider info */
#define BOARD_THERMISTOR_RNOM 10000
#define BOARD_THERMISTOR_BETA 3380
#define DIODE_THERMISTOR_RNOM 10000
#define DIODE_THERMISTOR_BETA 3380 
#define T0_KELVIN 298.15


/* Data rate settle time: ~1.5x period for 20 SPS default, generous margin */
#define SETTLE_NS   (long)((1.0 / 20.0) * 1.5 * 1e9)

typedef struct {
    int fd_spi;
    struct gpiod_chip *chip;
    struct gpiod_line *cs_line;
} ads124s08;

/* ------------------------------------------------------------------ */
/* Low-level SPI + CS helpers                                          */
/* ------------------------------------------------------------------ */

static void cs_low(ads124s08 *dev)  { gpiod_line_set_value(dev->cs_line, 0); }
static void cs_high(ads124s08 *dev) { gpiod_line_set_value(dev->cs_line, 1); }

static int spi_xfer(ads124s08 *dev, uint8_t *buf, int len)
{
    struct spi_ioc_transfer xfer;
    memset(&xfer, 0, sizeof(xfer));
    xfer.tx_buf = (unsigned long)buf;
    xfer.rx_buf = (unsigned long)buf;
    xfer.len    = (uint32_t)len;
    xfer.speed_hz = 1000000;
    xfer.bits_per_word = 8;

    cs_low(dev);
    int ret = ioctl(dev->fd_spi, SPI_IOC_MESSAGE(1), &xfer);
    usleep(50);
    cs_high(dev);
    usleep(50);
    return ret < 0 ? -1 : 0;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

/*
 * Open an ads124s08 on the given spidev node, with CS manually driven on
 * gpiochip_name/cs_offset (e.g. "gpiochip0", 17 for BCM GPIO17).
 * Returns a heap-allocated handle, or NULL on failure.
 */
ads124s08 *ads124s08_open(const char *spi_dev, const char *gpiochip_name, unsigned int cs_offset)
{
    ads124s08 *dev = (ads124s08 *)malloc(sizeof(ads124s08));
    if (!dev) return NULL;
    dev->fd_spi = -1;
    dev->chip = NULL;
    dev->cs_line = NULL;

    dev->fd_spi = open(spi_dev, O_RDWR);
    if (dev->fd_spi < 0) {
        free(dev);
        return NULL;
    }

    uint8_t mode = SPI_MODE_1 | SPI_NO_CS;   /* ads124s08: CPOL=0, CPHA=1 */
    uint8_t bits = 8;
    uint32_t speed = 100000;
    if (ioctl(dev->fd_spi, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(dev->fd_spi, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(dev->fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        close(dev->fd_spi);
        free(dev);
        return NULL;
    }

    dev->chip = gpiod_chip_open_by_name(gpiochip_name);
    if (!dev->chip) {
        close(dev->fd_spi);
        free(dev);
        return NULL;
    }

    dev->cs_line = gpiod_chip_get_line(dev->chip, cs_offset);
    if (!dev->cs_line) {
        gpiod_chip_close(dev->chip);
        close(dev->fd_spi);
        free(dev);
        return NULL;
    }

    /* Request as output, idle high (CS is active-low) */
    if (gpiod_line_request_output(dev->cs_line, "ads124s08", 1) < 0) {
        gpiod_chip_close(dev->chip);
        close(dev->fd_spi);
        free(dev);
        return NULL;
    }

    return dev;
}

int ads124s08_reset(ads124s08 *dev)
{
    uint8_t buf[1] = { CMD_RESET };
    int ret = spi_xfer(dev, buf, 1);
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 5000000L }; /* 5ms tosc startup margin */
    nanosleep(&ts, NULL);
    return ret;
}

static int write_reg(ads124s08 *dev, uint8_t addr, uint8_t value)
{
    uint8_t buf[3] = { (uint8_t)(CMD_WREG0 | (addr & 0x1F)), 0x00, value };
    return spi_xfer(dev, buf, 3);
}

static int read_reg(ads124s08 *dev, uint8_t addr, uint8_t *out)
{
    uint8_t buf[3] = { (uint8_t)(CMD_RREG0 | (addr & 0x1F)), 0x00, 0x00 };
    if (spi_xfer(dev, buf, 3) < 0) return -1;
    *out = buf[2];
    return 0;
}

int ads124s08_read_register(ads124s08 *dev, uint8_t addr, uint8_t *out)
{
    return read_reg(dev, addr, out);
}

int ads124s08_write_register(ads124s08 *dev, uint8_t addr, uint8_t value)
{
    return write_reg(dev, addr, value);
}
/*
 Configure all registers to baseline:
 reg02h: Input mux register configured to mux setting
 reg03h: gain register set to bypass PGA
 reg04h: Datarate register set to global chop, external clock,single shot, sinc3 filter, input datarate
 reg11h: GPIO config register set to all GPIOs enabled
 
 * Returns  0: success, out_regs populated with the 4 bytes written
 *         -1: SPI error
 */
int ads124s08_configure(ads124s08 *dev, uint8_t mux, uint8_t dr,  uint8_t out_regs[5])
{
    uint8_t reg02h = (uint8_t)mux;
    uint8_t reg03h = (uint8_t)BYPASS_PGA;
    uint8_t reg04h = (uint8_t)(G_CHOP_BIT|CLK_EXTERNAL_BIT|SINGLE_SHOT_BIT|FILTER_SINC3_BIT|dr);
    uint8_t reg10h = (uint8_t)0x00;
    uint8_t reg11h = (uint8_t)GPIO_ON_ALL;

    if (write_reg(dev, 0x02, reg02h) < 0) return -1;
    if (write_reg(dev, 0x03, reg03h) < 0) return -1;
    if (write_reg(dev, 0x04, reg04h) < 0) return -1;
    //if (write_reg(dev, 0x10, reg10h) < 0) return -1;
    if (write_reg(dev, 0x11, reg11h) < 0) return -1;

    out_regs[0] = reg02h;
    out_regs[1] = reg03h;
    out_regs[2] = reg04h;
    out_regs[3] = reg10h;
    out_regs[4] = reg11h;
    return 0;
}

/*
 * Read back all relevant config registers into out_regs.
 * Returns  0: success
 *         -1: SPI error
 */
int ads124s08_read_config(ads124s08 *dev, uint8_t out_regs[5])
{
    if (read_reg(dev, 0x02, &out_regs[0]) < 0) return -1;

    if (read_reg(dev, 0x03, &out_regs[1]) < 0) return -1;
    if (read_reg(dev, 0x04, &out_regs[2]) < 0) return -1;
    if (read_reg(dev, 0x10, &out_regs[3]) < 0) return -1;
    if (read_reg(dev, 0x11, &out_regs[4]) < 0) return -1;
    return 0;
}

/*
 * Trigger a single-shot conversion and read back the signed 24-bit result.
 *
 * NOTE: not DRDY-driven — issues START, sleeps a fixed margin sized for
 * the 20 SPS config above, then issues RDATA once. Simple and matches
 * this driver's fixed data rate; wire DRDY to a GPIO and poll it instead
 * if more robust timing is ever needed.
 *
 * Returns  0: success, *out_code populated (sign-extended)
 *         -1: SPI error
 */

int ads124s08_read_single_shot(ads124s08 *dev, int32_t *out_code)
{
    uint8_t start_buf[1] = { CMD_START };
    if (spi_xfer(dev, start_buf, 1) < 0) return -1;

    struct timespec ts = { .tv_sec = 0, .tv_nsec = SETTLE_NS };
    nanosleep(&ts, NULL);

    uint8_t buf[4] = { CMD_RDATA, 0x00, 0x00, 0x00 };
    if (spi_xfer(dev, buf, 4) < 0) return -1;

    int32_t code = ((int32_t)buf[1] << 16) | ((int32_t)buf[2] << 8) | (int32_t)buf[3];
    if (code & 0x800000) code -= (1 << 24);

    *out_code = code;
    return 0;
}

/*
 * Switch relays to measure current from different sources
 * 
 * Returns  0: success
 *         -1: SPI error
 */
int ads124s08_switch_relays(ads124s08 *dev, uint8_t relay_mask)
{
    uint8_t reg10h = (uint8_t)relay_mask;
    if (write_reg(dev, 0x10, reg10h) < 0) return -1;
    return 0;
}

/* Differential measurement, so to get the actual 0-5V level we offset by the reference*/
float ads124s08_code_to_volts(int32_t code)
{
    return (((float)code / FULL_SCALE_CODE) * VREF_V)+VREF_V;
}

/* Thermistors are the bottom half of a 10k/10k voltage divider excited by a 5V reference */
float ads124s08_thermistor_volts_to_resistance(float Vtherm)
{
    if (Vtherm == 0.0f) return INFINITY;
    return BOARD_THERMISTOR_RNOM * Vtherm / (5 - Vtherm);
}

/* Beta-parameter NTC equation, inverted for temperature */
float ads124s08_resistance_to_celsius(float r)
{
    if (r <= 0.0f) return NAN;
    float t_kelvin = 1.0f / (1.0f / T0_KELVIN + (1.0f / BOARD_THERMISTOR_BETA) * logf(r / BOARD_THERMISTOR_RNOM));
    return t_kelvin - 273.15f;
}

void ads124s08_close(ads124s08 *dev)
{
    if (!dev) return;
    if (dev->cs_line) {
        cs_high(dev);
        gpiod_line_release(dev->cs_line);
    }
    if (dev->chip) gpiod_chip_close(dev->chip);
    if (dev->fd_spi >= 0) close(dev->fd_spi);
    free(dev);
}
