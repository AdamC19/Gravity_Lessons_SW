#ifndef _AMC7812_H_
#define _AMC7812_H_

#include <linux/interrupt.h>
#include <linux/spi/spi.h>


#define AMC_RESET_GPIO  22
#define AMC_DAV_GPIO    23
#define AMC_CNVT_GPIO   24

#define AMC_SPI_DESCRIPTION "amc7812-spi-driver"
#define AMC_SPI_SPEED_HZ    8000000
#define AMC_SPI_MODE        SPI_MODE_1

/* AMC7812 REGISTER MAP */
#define AMC_REG_ADC_DATA_0  (0x23)
#define AMC_REG_ADC_DATA_1  (AMC_REG_ADC_DATA_0 + 1)
#define AMC_REG_ADC_DATA_2  (AMC_REG_ADC_DATA_0 + 2)
#define AMC_REG_ADC_DATA_3  (AMC_REG_ADC_DATA_0 + 3)
#define AMC_REG_ADC_DATA_4  (AMC_REG_ADC_DATA_0 + 4)
#define AMC_REG_ADC_DATA_5  (AMC_REG_ADC_DATA_0 + 5)
#define AMC_REG_ADC_DATA_6  (AMC_REG_ADC_DATA_0 + 6)
#define AMC_REG_ADC_DATA_7  (AMC_REG_ADC_DATA_0 + 7)
#define AMC_REG_ADC_DATA_8  (AMC_REG_ADC_DATA_0 + 8)
#define AMC_REG_ADC_DATA_9  (AMC_REG_ADC_DATA_0 + 9)
#define AMC_REG_ADC_DATA_10 (AMC_REG_ADC_DATA_0 + 10)
#define AMC_REG_ADC_DATA_11 (AMC_REG_ADC_DATA_0 + 11)
#define AMC_REG_ADC_DATA_12 (AMC_REG_ADC_DATA_0 + 12)
#define AMC_REG_ADC_DATA_13 (AMC_REG_ADC_DATA_0 + 13)
#define AMC_REG_ADC_DATA_14 (AMC_REG_ADC_DATA_0 + 14)
#define AMC_REG_ADC_DATA_15 (AMC_REG_ADC_DATA_0 + 15)

#define AMC_REG_CONFIG_0    (0x4C)
#define AMC_REG_CONFIG_1    (0x4D)
#define AMC_REG_STATUS      (0x4F)
#define AMC_REG_ADC_CH_0    (0x50)
#define AMC_REG_ADC_CH_1    (0x51)

#define AMC_REG_POWER_DOWN  (0x6B)
#define AMC_REG_DEVICE_ID   (0x6C)
#define AMC_REG_SOFT_RESET  (0x7C)
#define AMC_SOFT_RESET_VALUE (0x6600)

/* AMC Configuration 0 bit fields */
#define AMC_CONFIG_0_CMODE       13
#define AMC_CONFIG_0_ICONV       12
#define AMC_CONFIG_0_ILDAC       11
#define AMC_CONFIG_0_ADC_REF_INT 10
#define AMC_CONFIG_0_EN_ALARM    9
#define AMC_CONFIG_0_GALR        8
#define AMC_CONFIG_0_SET_MASK    0x3E00

#define AMC_SET_CMODE_AUTO(word)            (word |=  (1 << AMC_CONFIG_0_CMODE))
#define AMC_SET_CMODE_DIRECT(word)          (word &= ~(1 << AMC_CONFIG_0_CMODE))
#define AMC_SET_INTERNAL_CONV_TRIG(word)    (word |= (1 << AMC_CONFIG_0_ICONV))
#define AMC_SET_EXTERNAL_CONV_TRIG(word)    (word &= ~(1 << AMC_CONFIG_0_ICONV))

/* AMC Configuration 1 bit fields */
#define AMC_CONFIG_1_CONV_RATE      8

#define AMC_CONV_RATE_500K          (0 << AMC_CONFIG_1_CONV_RATE)
#define AMC_CONV_RATE_250K          (1 << AMC_CONFIG_1_CONV_RATE)
#define AMC_CONV_RATE_125K          (2 << AMC_CONFIG_1_CONV_RATE)
#define AMC_CONV_RATE_62P5K         (3 << AMC_CONFIG_1_CONV_RATE)

#define AMC_SET_CONV_RATE(word, rate)       (word = (word & ~(0x3 << AMC_CONFIG_1_CONV_RATE)) | rate)

typedef union adc_sample_union {
    struct {
        unsigned char index;
        unsigned long long timestamp;
        unsigned short value;
    };
    unsigned char data[11];
}adc_sample_t;


typedef struct amc7812_struct {
    unsigned int dav_irq_number;
    struct spi_controller * spi_ctrl;
    struct spi_device * spi_dev;
    unsigned char tx_buffer[3];
    unsigned char rx_buffer[3];
    unsigned long long int_timestamp;
    adc_sample_t samples[16];
}Amc7812_t;

struct circular_buffer {
    unsigned char * data;
    size_t size;
    size_t length;
    int position;
};

static void circ_buffer_read(struct circular_buffer * circ, unsigned char * dst, size_t len);
static void circ_buffer_write(struct circular_buffer * circ, unsigned char * src, size_t len);
static void circ_buffer_read_all(struct circular_buffer * circ, unsigned char * dst);

// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

// prototypes for AMC7812 specific things
static void amc7812_init_gpio(void);
static void amc7812_cleanup_gpio(void);
static void amc7812_init_spi();
static void amc7812_spi_xfer(Amc7812_t* daq);
static void amc7812_soft_reset(Amc7812_t* daq);
static void amc7812_start_conv(Amc7812_t* daq);
static void amc7812_read_all_channels(Amc7812_t* daq, unsigned long long timestamp, unsigned long long timestep);
static unsigned int amc7812_read_reg(Amc7812_t* daq, unsigned char reg_addr);
static void amc7812_write_reg(Amc7812_t* daq, unsigned char reg_addr, unsigned int reg_value);
static irq_handler_t amc7812_dav_handler(unsigned int irq, void* dev_id, struct pt_regs* regs);

#endif