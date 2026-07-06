#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#define MCP4728_ADDR        0x60
#define MCP4728_NUM_CH      4
#define MCP4728_MAX_CODE    4095

/* Multi-Write command byte per channel: 0b0100_0_AAA_0
 * where AAA selects the channel (0=A..3=D); LSB is UDAC (0 = update now). */
#define MULTIWRITE_CMD      0x40

/* EEPROM write takes up to ~50 ms per the datasheet. */
#define EEPROM_WRITE_MS     50

typedef struct {
    uint16_t code[MCP4728_NUM_CH];  /* 0-4095 */
    uint8_t  vref_vdd[MCP4728_NUM_CH]; /* 1 = Vdd, 0 = internal 2.048V ref */
    uint8_t  gain2x[MCP4728_NUM_CH];   /* 1 = 2x gain (only applies to internal ref) */
    uint8_t  powered_down[MCP4728_NUM_CH];
} MCP4728State;

/* ------------------------------------------------------------------ */
/* Low-level I2C helpers                                              */
/* ------------------------------------------------------------------ */

static int i2c_write_block(int fd, const uint8_t *buf, int len)
{
    struct i2c_msg msg = {
        .addr  = MCP4728_ADDR,
        .flags = 0,
        .len   = (uint16_t)len,
        .buf   = (uint8_t *)buf,
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = &msg, .nmsgs = 1 };
    return ioctl(fd, I2C_RDWR, &xfer);
}

static int i2c_read_block(int fd, uint8_t *out, int len)
{
    struct i2c_msg msg = {
        .addr  = MCP4728_ADDR,
        .flags = I2C_M_RD,
        .len   = (uint16_t)len,
        .buf   = out,
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = &msg, .nmsgs = 1 };
    return ioctl(fd, I2C_RDWR, &xfer);
}

static uint16_t clamp_code(int32_t code)
{
    if (code < 0) return 0;
    if (code > MCP4728_MAX_CODE) return MCP4728_MAX_CODE;
    return (uint16_t)code;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int mcp4728_open(const char *i2c_dev)
{
    int fd = open(i2c_dev, O_RDWR);
    if (fd < 0) return -1;

    if (ioctl(fd, I2C_SLAVE, MCP4728_ADDR) < 0) {
        close(fd);
        return -1;
    }

    /* Confirm device responds: read the 24-byte register block. */
    uint8_t probe[24];
    if (i2c_read_block(fd, probe, sizeof(probe)) < 0) {
        close(fd);
        return -1;
    }

    return fd;
}

/*
 * Write all 4 channels via Multi-Write, explicitly setting Vref/PD/gain
 * per channel. Use this whenever the reference or power-down mode may
 * need to change; it is slower than mcp4728_fast_write (one transaction
 * per channel instead of one for all four).
 *
 * Returns  0: success
 *         -1: I2C error
 */
int mcp4728_multi_write(int fd, const MCP4728State *state)
{
    for (int ch = 0; ch < MCP4728_NUM_CH; ch++) {
        uint16_t code = clamp_code(state->code[ch]);
        uint8_t vref  = state->vref_vdd[ch]     ? 0 : 1; /* bit=0 means Vdd */
        uint8_t gain  = state->gain2x[ch]        ? 1 : 0;
        uint8_t pd    = state->powered_down[ch]  ? 1 : 0; /* PD1:PD0, 01 = 1kOhm pulldown */

        uint8_t cmd   = (uint8_t)(MULTIWRITE_CMD | (ch << 1));
        uint8_t upper = (uint8_t)((vref << 7) | (pd << 5) | (gain << 4) | ((code >> 8) & 0x0F));
        uint8_t lower = (uint8_t)(code & 0xFF);

        uint8_t buf[3] = { cmd, upper, lower };
        if (i2c_write_block(fd, buf, sizeof(buf)) < 0)
            return -1;
    }
    return 0;
}

/*
 * Write all 4 channels' DAC codes in a single transaction via Fast Write.
 * Fast Write cannot change Vref/PD/gain — those retain whatever was last
 * set via mcp4728_multi_write (or EEPROM/power-on default). Call
 * mcp4728_multi_write at least once after power-up if the reference
 * needs to be Vdd rather than the internal 2.048V reference.
 *
 * Returns  0: success
 *         -1: I2C error
 */
int mcp4728_fast_write(int fd, const uint16_t codes[MCP4728_NUM_CH])
{
    uint8_t buf[MCP4728_NUM_CH * 2];
    for (int ch = 0; ch < MCP4728_NUM_CH; ch++) {
        uint16_t code = clamp_code(codes[ch]);
        buf[ch * 2]     = (uint8_t)((code >> 8) & 0x0F); /* 00 D11 D10 D9 D8 */
        buf[ch * 2 + 1] = (uint8_t)(code & 0xFF);
    }
    return i2c_write_block(fd, buf, sizeof(buf));
}

/*
 * Read back the DAC input registers (not EEPROM) for all 4 channels,
 * populating codes and the Vref/gain/power-down flags actually latched
 * on the chip.
 *
 * Returns  0: success
 *         -1: I2C error
 */
int mcp4728_read(int fd, MCP4728State *state)
{
    uint8_t raw[24];
    if (i2c_read_block(fd, raw, sizeof(raw)) < 0)
        return -1;

    /* Each channel occupies 6 bytes: 3 for the DAC (input) register,
     * 3 for the EEPROM register. DAC register layout:
     *   byte0: [RDY/BSY][POR][CH1][CH0][x][x][x][x]  (channel/status byte)
     *   byte1: [x][x][VREF][PD1][PD0][GAIN][D11][D10]
     *   byte2: [D9][D8][D7][D6][D5][D4][D3][D2][D1][D0] -- see note below
     *
     * NOTE: byte1 bit layout on read differs slightly from the write-form
     * byte used by Multi-Write; VREF/PD/GAIN sit at bits [3:0] alongside
     * the top 4 data bits, per the MCP4728 datasheet's "Read Command"
     * timing diagram for the DAC register.
     */
    for (int ch = 0; ch < MCP4728_NUM_CH; ch++) {
        int base = ch * 6;
        uint8_t upper = raw[base + 1];
        uint8_t lower = raw[base + 2];

        state->vref_vdd[ch]      = ((upper >> 3) & 0x01) ? 0 : 1;
        state->powered_down[ch]  = ((upper >> 1) & 0x03) != 0;
        state->gain2x[ch]        = upper & 0x01;
        state->code[ch]          = (uint16_t)(((upper & 0x0F) << 8) | lower);
    }
    return 0;
}

/*
 * Write all 4 channels to EEPROM (persists across power cycles) using
 * Sequential Write starting at channel A. Blocks for ~50 ms per the
 * datasheet while the EEPROM write completes.
 *
 * Returns  0: success
 *         -1: I2C error
 */
int mcp4728_write_eeprom(int fd, const MCP4728State *state)
{
    /* Sequential Write command byte: 0b0101_0_AAA_0, starting at channel A. */
    uint8_t buf[1 + MCP4728_NUM_CH * 2];
    buf[0] = 0x50;

    for (int ch = 0; ch < MCP4728_NUM_CH; ch++) {
        uint16_t code = clamp_code(state->code[ch]);
        uint8_t vref  = state->vref_vdd[ch]     ? 0 : 1;
        uint8_t gain  = state->gain2x[ch]        ? 1 : 0;
        uint8_t pd    = state->powered_down[ch]  ? 1 : 0;

        buf[1 + ch * 2]     = (uint8_t)((vref << 7) | (pd << 5) | (gain << 4) | ((code >> 8) & 0x0F));
        buf[1 + ch * 2 + 1] = (uint8_t)(code & 0xFF);
    }

    if (i2c_write_block(fd, buf, sizeof(buf)) < 0)
        return -1;

    struct timespec ts = { .tv_sec = 0, .tv_nsec = EEPROM_WRITE_MS * 1000000L };
    nanosleep(&ts, NULL);
    return 0;
}

/*
 * Probe the device by reading its register block.
 * Returns  0: device responding
 *         -1: I2C error (device absent or bus hung)
 */
int mcp4728_ping(int fd)
{
    uint8_t probe[24];
    return i2c_read_block(fd, probe, sizeof(probe)) < 0 ? -1 : 0;
}

void mcp4728_close(int fd)
{
    if (fd >= 0) close(fd);
}
