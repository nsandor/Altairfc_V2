#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>

typedef struct {
    int fd_spi;
    struct gpiod_chip *chip;
    struct gpiod_line *cs_line;
} Dac5311;

/* ------------------------------------------------------------------ */
/* Low-level SPI + CS helpers                                         */
/* ------------------------------------------------------------------ */

static void cs_low(Dac5311 *dev)  { gpiod_line_set_value(dev->cs_line, 0); }
static void cs_high(Dac5311 *dev) { gpiod_line_set_value(dev->cs_line, 1); }

static int spi_xfer(Dac5311 *dev, uint8_t *buf, int len)
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
    return ret < 0 ? -1 : 0;
}

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */

Dac5311 *dac5311_open(const char *spi_dev, const char *gpiochip_name, unsigned int cs_offset)
{
    Dac5311 *dev = (Dac5311 *)malloc(sizeof(Dac5311));
    if (!dev) return NULL;
    dev->fd_spi = -1;
    dev->chip = NULL;
    dev->cs_line = NULL;

    dev->fd_spi = open(spi_dev, O_RDWR);
    if (dev->fd_spi < 0) {
        free(dev);
        return NULL;
    }

    uint8_t mode = SPI_MODE_1;   /* DAC5311: CPOL=0, CPHA=1 */
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
    if (gpiod_line_request_output(dev->cs_line, "dac5311", 1) < 0) {
        gpiod_chip_close(dev->chip);
        close(dev->fd_spi);
        free(dev);
        return NULL;
    }

    return dev;
}

int dac5311_write_value(Dac5311 *dev, uint8_t value)
{
    /* Normal Operation mode: PD1=0, PD0=0
       Data Format: [PD1] [PD0] [D7] [D6] [D5] [D4] [D3] [D2] [D1] [D0] [X] [X] [X] [X] [X] [X]
       This translates to: value << 6 */
    uint16_t payload = (uint16_t)value << 6;
    uint8_t buf[2];
    buf[0] = (payload >> 8) & 0xFF;
    buf[1] = payload & 0xFF;
    return spi_xfer(dev, buf, 2);
}

int dac5311_power_down(Dac5311 *dev, uint8_t mode)
{
    /* Power-down mode configuration:
       PD1, PD0 located at bits 15 and 14 respectively.
       00: Normal Operation
       01: Output 1 kOhm to GND
       10: Output 100 kOhm to GND
       11: High-Z */
    uint16_t payload = (uint16_t)(mode & 0x03) << 14;
    uint8_t buf[2];
    buf[0] = (payload >> 8) & 0xFF;
    buf[1] = payload & 0xFF;
    return spi_xfer(dev, buf, 2);
}

void dac5311_close(Dac5311 *dev)
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
