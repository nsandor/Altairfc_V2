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
/* ADS1220 command bytes                                              */
/* ------------------------------------------------------------------ */
#define CMD_RESET      0x06
#define CMD_START      0x08
#define CMD_POWERDOWN  0x02
#define CMD_RDATA      0x10
#define CMD_RREG0      0x20   /* RREG starting at reg 0, 1 register (0010 00 00) */
#define CMD_WREG0      0x40   /* WREG starting at reg 0, 1 register (0100 00 00) */

/* Config Register 0: MUX[7:4] GAIN[3:1] PGA_BYPASS[0] */
#define MUX_AIN0_AVSS       0x8   /* single-ended AIN0 vs AVSS (requires PGA_BYPASS=1) */
#define MUX_AIN2_AIN3_DIFF  0x5   /* differential AIN2(+) - AIN3(-) */
#define GAIN_1X             0x0
#define PGA_BYPASS_ON       0x1
#define PGA_BYPASS_OFF      0x0

/* Config Register 1: DR[7:5] MODE[4:3] CM[2] TS[1] BCS[0] */
#define DR_20SPS_NORMAL  0x0
#define MODE_NORMAL      0x0
#define CM_SINGLE_SHOT   0x0

/* Config Register 2: VREF[7:6] FIR[5:4] PSW[3] IDAC[2:0] */
#define VREF_INTERNAL  0x0  /* internal 2.048V reference */

#define VREF_V            2.048f
#define FULL_SCALE_CODE   8388608.0f  /* 2^23 */

/* Bridge / thermistor constants */
#define BRIDGE_R    10000.0f   /* R3=R4=R5, ohms */
#define VEXC        5.0f       /* bridge excitation voltage */
#define THERM_R25   10000.0f
#define THERM_B     3380.0f
#define T0_KELVIN   298.15f

/* Data rate settle time: ~1.5x period for 20 SPS default, generous margin */
#define SETTLE_NS   (long)((1.0 / 20.0) * 1.5 * 1e9)

typedef struct {
    int fd_spi;
    struct gpiod_chip *chip;
    struct gpiod_line *cs_line;
} Ads1220;

/* ------------------------------------------------------------------ */
/* Low-level SPI + CS helpers                                          */
/* ------------------------------------------------------------------ */

static void cs_low(Ads1220 *dev)  { gpiod_line_set_value(dev->cs_line, 0); }
static void cs_high(Ads1220 *dev) { gpiod_line_set_value(dev->cs_line, 1); }

static int spi_xfer(Ads1220 *dev, uint8_t *buf, int len)
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
    cs_high(dev);

    usleep(2);
    return ret < 0 ? -1 : 0;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

/*
 * Open an ADS1220 on the given spidev node, with CS manually driven on
 * gpiochip_name/cs_offset (e.g. "gpiochip0", 17 for BCM GPIO17).
 * Returns a heap-allocated handle, or NULL on failure.
 */
Ads1220 *ads1220_open(const char *spi_dev, const char *gpiochip_name, unsigned int cs_offset)
{
    Ads1220 *dev = (Ads1220 *)malloc(sizeof(Ads1220));
    if (!dev) return NULL;
    dev->fd_spi = -1;
    dev->chip = NULL;
    dev->cs_line = NULL;

    dev->fd_spi = open(spi_dev, O_RDWR);
    if (dev->fd_spi < 0) {
        free(dev);
        return NULL;
    }

    uint8_t mode = SPI_MODE_1;   /* ADS1220: CPOL=0, CPHA=1 */
    uint8_t bits = 8;
    uint32_t speed = 1000000;
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
    if (gpiod_line_request_output(dev->cs_line, "ads1220", 1) < 0) {
        gpiod_chip_close(dev->chip);
        close(dev->fd_spi);
        free(dev);
        return NULL;
    }

    return dev;
}

int ads1220_reset(Ads1220 *dev)
{
    uint8_t buf[1] = { CMD_RESET };
    int ret = spi_xfer(dev, buf, 1);
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 1000000L }; /* 1ms tosc startup margin */
    nanosleep(&ts, NULL);
    return ret;
}

static int write_reg(Ads1220 *dev, uint8_t addr, uint8_t value)
{
    uint8_t buf[2] = { (uint8_t)(CMD_WREG0 | (addr << 2)), value };
    return spi_xfer(dev, buf, 2);
}

static int read_reg(Ads1220 *dev, uint8_t addr, uint8_t *out)
{
    uint8_t buf[2] = { (uint8_t)(CMD_RREG0 | (addr << 2)), 0x00 };
    if (spi_xfer(dev, buf, 2) < 0) return -1;
    *out = buf[1];
    return 0;
}

/*
 * Configure MUX/GAIN/PGA_BYPASS (reg0), fixed 20 SPS normal single-shot
 * (reg1), and VREF=internal 2.048V (reg2). reg3 left at 0x00.
 * Returns  0: success, out_regs populated with the 4 bytes written
 *         -1: SPI error
 */
int ads1220_configure(Ads1220 *dev, uint8_t mux, uint8_t pga_bypass, uint8_t out_regs[4])
{
    uint8_t reg0 = (uint8_t)((mux << 4) | (GAIN_1X << 1) | pga_bypass);
    uint8_t reg1 = (uint8_t)((DR_20SPS_NORMAL << 5) | (MODE_NORMAL << 3) | (CM_SINGLE_SHOT << 2));
    uint8_t reg2 = (uint8_t)(VREF_INTERNAL << 6);
    uint8_t reg3 = 0x00;

    if (write_reg(dev, 0, reg0) < 0) return -1;
    if (write_reg(dev, 1, reg1) < 0) return -1;
    if (write_reg(dev, 2, reg2) < 0) return -1;
    if (write_reg(dev, 3, reg3) < 0) return -1;

    out_regs[0] = reg0;
    out_regs[1] = reg1;
    out_regs[2] = reg2;
    out_regs[3] = reg3;
    return 0;
}

/*
 * Read back all 4 config registers into out_regs.
 * Returns  0: success
 *         -1: SPI error
 */
int ads1220_read_config(Ads1220 *dev, uint8_t out_regs[4])
{
    for (int i = 0; i < 4; i++) {
        if (read_reg(dev, (uint8_t)i, &out_regs[i]) < 0) return -1;
    }
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
int ads1220_read_single_shot(Ads1220 *dev, int32_t *out_code)
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

float ads1220_code_to_volts(int32_t code)
{
    return ((float)code / FULL_SCALE_CODE) * VREF_V;
}

/* TH1 = R * (Vexc/2 + Vdiff) / (Vexc/2 - Vdiff) */
float ads1220_bridge_volts_to_resistance(float vdiff)
{
    float half_vexc = VEXC / 2.0f;
    float denom = half_vexc - vdiff;
    if (denom == 0.0f) return INFINITY;
    return BRIDGE_R * (half_vexc + vdiff) / denom;
}

/* Beta-parameter NTC equation, inverted for temperature */
float ads1220_resistance_to_celsius(float r)
{
    if (r <= 0.0f) return NAN;
    float t_kelvin = 1.0f / (1.0f / T0_KELVIN + (1.0f / THERM_B) * logf(r / THERM_R25));
    return t_kelvin - 273.15f;
}

void ads1220_close(Ads1220 *dev)
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
