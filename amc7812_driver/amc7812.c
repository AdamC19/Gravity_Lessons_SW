
#include "amc7812.h"
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/uaccess.h>

// device will appear at /dev/amc7812
#define DEVICE_NAME     "amc7812" 

// the device class -- this is a character device driver
#define CLASS_NAME      "amc"

#define CHUNK_DELIM         0xA5
#define CHUNK_DELIM_COUNT   4
#define MAX_CHUNKS          8
#define DATA_BUF_MAX        (16*sizeof(adc_sample_t) + CHUNK_DELIM_COUNT)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Adam Cordingley");
MODULE_DESCRIPTION("A driver for the AMC7812");
MODULE_VERSION("0.1");

static DEFINE_MUTEX(circ_buf_mutex);

static int chunk_lens[MAX_CHUNKS] = {0};
static int major_num = 0;
static int number_opens = 0;
static unsigned char copy_to_user_buffer[DATA_BUF_MAX] = {0};
static unsigned char data_buffer[DATA_BUF_MAX] = {0};
static unsigned char data_buf_a[DATA_BUF_MAX] = {0};
static unsigned char data_buf_b[DATA_BUF_MAX] = {0};
static unsigned char* data_bufs[] = {data_buf_a, data_buf_b};
static int bytes_available[] = {0, 0};
static int start_read_index[] = {0, 0}; // index in data buffer at which to begin reading from
static int active_data_buf  = 0; // indicates which data buffer we're stuffing now

static struct class* amc7812_class = NULL;
static struct device* amc7812_device = NULL;
static Amc7812_t amc_daq;

///////////////////////////////////////////////////////////////////////////////
//
// CIRCULAR BUFFER THINGS
//
///////////////////////////////////////////////////////////////////////////////

// static void circ_buffer_read(struct circular_buffer * circ, unsigned char * dst, size_t len){
//     // FIFO behavior is needed. Find the start first
//     int start_pos = 0;
//     if (circ->length > circ->position){
//         start_pos = circ->size - (circ->length - circ->position);
//     }else{
//         start_pos = circ->position - circ->length;
//     }

//     // Clamp len to the size of the buffer
//     if (len > circ->size){
//         len = circ->size;
//     }
    
//     size_t first_chunk_len = len;
//     size_t last_chunk_len = 0;
//     if(first_chunk_len + start_pos > circ->size){
//         first_chunk_len = circ->size - start_pos;
//         last_chunk_len = len - first_chunk_len;
//     }

//     memcpy(dst, circ->data + start_pos, first_chunk_len);
//     circ->length -= first_chunk_len;
//     if(last_chunk_len > 0){
//         start_pos += first_chunk_len;
//         if (start_pos >= circ->size){
//             start_pos = 0;
//         }
//         memcpy(dst + first_chunk_len, circ->data + start_pos, last_chunk_len);
//         circ->length -= last_chunk_len;
//     }
// }

// static void circ_buffer_write(struct circular_buffer * circ, unsigned char * src, size_t len){
//     size_t first_chunk_len = len;
//     if (first_chunk_len > circ->size){
//         first_chunk_len = circ->size;
//     }
//     size_t second_chunk_len = 0;
//     if(first_chunk_len > circ->len - circ->position){
//         first_chunk_len = circ->len - circ->position;
//         second_chunk_len = len - first_chunk_len;
//     }
//     memcpy(circ->data + circ->position, src, first_chunk_len);
//     circ->length += first_chunk_len;
//     circ->position += first_chunk_len;

//     if(circ->position == circ->size){
//         circ->position = 0;
//     }
//     if (second_chunk_len > 0){
//         memcpy(circ->data, src + first_chunk_len, second_chunk_len);
//         circ->position += second_chunk_len;
//         circ->length += second_chunk_len;
//     }
//     if (circ->length >= circ->size){
//         circ->length = circ->size;
//     }
// }


// static void circ_buffer_read_all(struct circular_buffer * circ, unsigned char * dst){
//     circ_buffer_read(circ, dst, circ->length);
// }

///////////////////////////////////////////////////////////////////////////////
//
// FILE OPERATION THINGS
//
///////////////////////////////////////////////////////////////////////////////

static struct file_operations fops =
{
    .open = dev_open,
    .read = dev_read,
    .release = dev_release,
};


static int dev_open(struct inode *inodep, struct file *filep){
   number_opens++;
   printk(KERN_INFO "AMC7812: Device has been opened %d time(s)\n", number_opens);
   return 0;
}

static int dev_release(struct inode *inodep, struct file *filep){
   printk(KERN_INFO "AMC7812: Device successfully closed\n");
   number_opens--;
   return 0;
}

/**
 * Reads out to the user from the last batch of data gathered.
 * 
 */
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){
    
    // read from whichever is NOT the active buffer
    int select = (active_data_buf + 1) % 2; 
    uint8_t* data = data_bufs[select];


    // first, check if we've read the entirety of the current, latent buffer
    if(start_read_index[select] >= bytes_available[select]){
        return 0;
    }

    int size_of_data = 0;
    if (bytes_available[select] <= len){
        // send everything we got
        size_of_data = bytes_available[select];
    }else{
        // send only len-many bytes
        size_of_data = len;
    }

    int error_count = copy_to_user(buffer, data + start_read_index[select], size_of_data);
    start_read_index[select] += size_of_data;
    bytes_available[select] -= size_of_data;


    if (error_count == 0){     // if true then we have success
        // printk(KERN_INFO "AMC7812: Sent %d characters to the user\n", size_of_data);
        return size_of_data;
    }
    else {
        printk(KERN_INFO "AMC7812: Failed to send %d characters to the user\n", error_count);
        return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
    }
}



///////////////////////////////////////////////////////////////////////////////
//
// DRIVER THINGS
//
///////////////////////////////////////////////////////////////////////////////
/**
 * driver initialization function
*/
static int __init amc7812_driver_init(void){
    printk(KERN_INFO "AMC7812: Initializing the AMC7812 driver...\n");

    // Try to dynamically allocate a major number for the device -- more difficult but worth it
    major_num = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_num < 0){
        printk(KERN_ALERT "AMC7812: failed to register a major number\n");
        return major_num;
    }
    printk(KERN_INFO "AMC7812: registered correctly with major number %d\n", major_num);

    // Register the device class
    amc7812_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(amc7812_class)){                // Check for error and clean up if there is
        unregister_chrdev(major_num, DEVICE_NAME);
        printk(KERN_ALERT "AMC7812: Failed to register device class\n");
        return PTR_ERR(amc7812_class);         // Correct way to return an error on a pointer
    }
    printk(KERN_INFO "AMC7812: device class registered correctly\n");
    
    // Register the device driver
    amc7812_device = device_create(amc7812_class, NULL, MKDEV(major_num, 0), NULL, DEVICE_NAME);
    if (IS_ERR(amc7812_device)){               // Clean up if there is an error
        class_destroy(amc7812_class);           // Repeated code but the alternative is goto statements
        unregister_chrdev(major_num, DEVICE_NAME);
        printk(KERN_ALERT "AMC7812: Failed to create the device\n");
        return PTR_ERR(amc7812_device);
    }
    printk(KERN_INFO "AMC7812: device class created correctly\n"); // Made it! device was initialized

    if(amc7812_init() < 0){
        class_destroy(amc7812_class);           // Repeated code but the alternative is goto statements
        unregister_chrdev(major_num, DEVICE_NAME);
        printk(KERN_ALERT "AMC7812: amc7812_init() failed.\n");
        return -1;
    }

    uint32_t device_id = amc7812_get_device_id(&amc_daq);
    if(device_id != AMC_DFLT_DEVICE_ID){
        class_destroy(amc7812_class);           // Repeated code but the alternative is goto statements
        unregister_chrdev(major_num, DEVICE_NAME);
        printk(KERN_ALERT "AMC7812: did not read correct device ID. Read 0x%04X\n", device_id);
        return -1;
    }
    printk(KERN_INFO "AMC7812: Got the correct device ID.\n"); // Comms with the target are working

    printk(KERN_INFO "AMC7812: Driver initialization complete!\n"); // Comms with the target are working
    return 0;
};

/**
 * driver exit function
 */
static void __exit amc7812_driver_exit(void){
    device_destroy(amc7812_class, MKDEV(major_num, 0));     // remove the device
    class_unregister(amc7812_class);                          // unregister the device class
    class_destroy(amc7812_class);                             // remove the device class
    unregister_chrdev(major_num, DEVICE_NAME);             // unregister the major number
    spi_unregister_device(amc_daq.spi_dev);
    amc7812_cleanup_gpio();
    printk(KERN_INFO "AMC7812: Exiting...\n");
}

///////////////////////////////////////////////////////////////////////////////
//
// AMC7812 SPECIFIC THINGS
//
///////////////////////////////////////////////////////////////////////////////

/**
 * Sets up the gpio for the AMC7812, including the reset, convert trigger,
 * and DAV interrupt pins. Also registers the DAV interrupt and the 
 * amc7812_dav_handler() function. Also performs a reset of the AMC7812.
*/
static void amc7812_init_gpio(void){
    int result = 0;

    /* INIT RESET OUTPUT */
    if(gpio_request(AMC_RESET_GPIO, "sysfs") != 0){
        printk(KERN_ALERT "AMC7812: failed to get gpio number %d for the RESET pin.\n", AMC_RESET_GPIO);
        return;
    }
    gpio_direction_output(AMC_RESET_GPIO, 0); // START IN RESET
    gpio_export(AMC_RESET_GPIO, false);
    msleep(1); // pause to ensure device actually resets

    /* INIT DAV INPUT */
    if(gpio_request(AMC_DAV_GPIO, "sysfs") != 0){
        printk(KERN_ALERT "AMC7812: failed to get gpio number %d for the DAV pin.\n", AMC_DAV_GPIO);
        gpio_set_value(AMC_RESET_GPIO, 1); // Release from reset before returning
        return;
    }
    gpio_direction_input(AMC_DAV_GPIO);
    gpio_export(AMC_DAV_GPIO, false);   // Causes gpio to appear in /sys/class/gpio, 
                                        // the bool argument prevents the direction from being changed
    amc_daq.dav_irq_number = gpio_to_irq(AMC_DAV_GPIO);
    result = request_irq(
                    amc_daq.dav_irq_number,                 // The interrupt number requested
                    (irq_handler_t) amc7812_dav_handler,    // The pointer to the handler function below
                    IRQF_TRIGGER_FALLING,                   // Interrupt on falling edge
                    "amc7812_dav_handler",                  // Used in /proc/interrupts to identify the owner
                    NULL);                                  // The *dev_id for shared interrupt lines, NULL is okay

    /*INIT CONVERT TRIGGER OUTPUT */
    if(gpio_request(AMC_CNVT_GPIO, "sysfs") != 0){
        printk(KERN_ALERT "AMC7812: failed to get gpio number %d for the CNVT pin.\n", AMC_CNVT_GPIO);
        gpio_set_value(AMC_RESET_GPIO, 1); // Release from reset before returning
        return;
    }
    gpio_direction_output(AMC_CNVT_GPIO, 1); // No trigger
    gpio_export(AMC_CNVT_GPIO, false);

    gpio_set_value(AMC_RESET_GPIO, 1); // Finally, release from reset
}


/**
 * Unexport GPIOs and the rest
*/
static void amc7812_cleanup_gpio(void){
    printk(KERN_INFO "AMC7812: Cleaning up GPIO...\n");
    /* RESET OUTPUT */
    gpio_set_value(AMC_RESET_GPIO, 1); // ensure not in reset
    gpio_unexport(AMC_RESET_GPIO);

    /* DAV INPUT */
    free_irq(amc_daq.dav_irq_number, NULL);
    gpio_unexport(AMC_DAV_GPIO);

    /* CONVERT OUTPUT */
    gpio_set_value(AMC_CNVT_GPIO, 1); // ensure not triggering a conversion
    gpio_unexport(AMC_CNVT_GPIO);
}

/**
 * 1. Initializes the spi bus.
 * 2. Performs a soft reset of the AMC7812.
 */
static int amc7812_init_spi(){
    amc_daq.spi_ctrl = spi_busnum_to_master(0); // we're using SPI0
    if(amc_daq.spi_ctrl == NULL){
        printk(KERN_ALERT "AMC7812: failed to get the SPI-%d controller\n", 0);
        return -1;
    }
    // Setup the spi_board_info struct
    struct spi_board_info spi_info = {
        .modalias       = AMC_SPI_DESCRIPTION,
        .max_speed_hz   = AMC_SPI_SPEED_HZ,
        .bus_num        = 0,
        .chip_select    = 0,
        .mode           = AMC_SPI_MODE
    };

    amc_daq.spi_dev = spi_new_device(amc_daq.spi_ctrl, &spi_info);
    if(amc_daq.spi_dev == NULL){
        printk(KERN_ALERT "AMC7812: failed to create a SPI slave device\n");
        return -2;
    }
    if(spi_setup(amc_daq.spi_dev) != 0){
        printk(KERN_ALERT "AMC7812: failed to configure the SPI device\n");
        return -3;
    }
    return 0;
}

/**
 * Calls amc7812_init_gpio and amc7812_init_spi, then performs initial configuration
 * of the AMC7812. 
*/
static int amc7812_init(){
    // Initialize GPIO
    amc7812_init_gpio(); // this also hard resets the AMC7812
    msleep(10); // allow DAQ to come out of reset
    if (amc7812_init_spi() < 0){
        printk(KERN_ALERT "AMC7812: amc7812_init_spi() failed.\n");
        return -1;
    }
    uint32_t pwrdn_reg = (1 << 14); // enable the ADC
    pwrdn_reg |= (1 << 13); // enable the internal reference
    amc7812_write_reg(&amc_daq, AMC_REG_POWER_DOWN, pwrdn_reg);

    // amc7812_soft_reset(&amc_daq); // just for good measure
    
    msleep(10);

    unsigned int config_0 = amc7812_read_reg(&amc_daq, AMC_REG_CONFIG_0);
    config_0 = amc7812_read_reg(&amc_daq, AMC_REG_CONFIG_0);
    printk(KERN_INFO "AMC7812: config_0 = %04X.\n", config_0);
    AMC_SET_ADC_REF_INT(config_0);
    // config_0 |= (1 << AMC_CONFIG_0_ADC_REF_INT);
    AMC_SET_CMODE_AUTO(config_0);
    AMC_SET_EXTERNAL_CONV_TRIG(config_0);
    printk(KERN_INFO "AMC7812: Updating config_0 to %04X.\n", config_0);
    amc7812_write_reg(&amc_daq, AMC_REG_CONFIG_0, config_0);

    unsigned int config_1 = amc7812_read_reg(&amc_daq, AMC_REG_CONFIG_1);
    AMC_SET_CONV_RATE(config_1, AMC_CONV_RATE_250K);
    amc7812_write_reg(&amc_daq, AMC_REG_CONFIG_1, config_1);

    /* Set up all 16 channels to be converted */
    unsigned int adc_ch_0 = 0;
    adc_ch_0 |= (0x6 << 12); // CH0 and CH1 are accessed as single-ended inputs
    adc_ch_0 |= (0x6 << 12); // CH2 and CH3 are accessed as single-ended inputs
    adc_ch_0 |= (0x1F);      // CH4 thru CH12 are accessed as single-ended
    amc7812_write_reg(&amc_daq, AMC_REG_ADC_CH_0, adc_ch_0);

    unsigned int adc_ch_1 = (0x7 << 12); // CH13 thru CH15
    amc7812_write_reg(&amc_daq, AMC_REG_ADC_CH_1, adc_ch_1);

    amc7812_start_conv(&amc_daq); // kicks off the continuous auto conversion
    return 0;
}


/**
 * Reads and returns the value of the device ID register. Userful for determining
 * if everything is working and the chip is present, etc.
*/
static unsigned int amc7812_get_device_id(Amc7812_t* daq){
    return amc7812_read_reg(daq, AMC_REG_DEVICE_ID);
}


/**
 * Transfers the contents of the daq struct's tx_buffer, and reads into
 * the struct's rx_buffer. Each transfer is 3 bytes long.
 */
static void amc7812_spi_xfer(Amc7812_t* daq){
    struct spi_transfer xfers[1];
    xfers[0].tx_buf = daq->tx_buffer;
    xfers[0].rx_buf = daq->rx_buffer;
    xfers[0].len = 3;
    spi_sync_transfer(daq->spi_dev, xfers, 1);
}

/**
 *
 */
static void amc7812_read_all_channels(Amc7812_t* daq, unsigned long long timestamp, unsigned long long timestep){
    struct spi_transfer xfers[1]; // we need a dummy transfer
    int total_len = 3*17;
    uint8_t xfer_tx_buf[3*17];
    uint8_t xfer_rx_buf[3*17];

    int xfer_ind = 0;
    int i;
    for(i = 0; i < 17; i++){
        xfer_tx_buf[3*i] = (AMC_REG_ADC_DATA_0 + i) | (1 << 7);
        xfer_tx_buf[3*i + 1] = 0;
        xfer_tx_buf[3*i + 2] = 0;
        xfers[0].tx_buf = xfer_tx_buf + 3*i;
        xfers[0].rx_buf = xfer_rx_buf + 3*i;
        xfers[0].len = 3;
        // xfers[0].cs_change = 1; // NECEESSARY: change CS line between transfers
        spi_sync_transfer(daq->spi_dev, xfers, 1);
    }
    
    // skip the first rx result (it's unknown what it is)
    for(i = 1; i < 17; i++){
        int j = i - 1; // the sample index
        daq->samples[j].timestamp = timestamp + (timestep * j); // approx sample time
        daq->samples[j].index = (uint8_t)j;
        daq->samples[j].value = (xfer_rx_buf[3*i + 1] << 8) | (xfer_rx_buf[3*i + 2] & 0xFF);
    }
}

/**
 *
 */
static unsigned int amc7812_read_reg(Amc7812_t* daq, unsigned char reg_addr){
    daq->tx_buffer[0] = reg_addr | (1 << 7);
    daq->tx_buffer[1] = 0;
    daq->tx_buffer[2] = 0;

    amc7812_spi_xfer(daq); // dummy read, sends over the target read register
    amc7812_spi_xfer(daq); // acutally reads the target register

    return ((daq->rx_buffer[1] << 8) | daq->rx_buffer[2]);
}

/**
 * 
*/
static void amc7812_write_reg(Amc7812_t* daq, unsigned char reg_addr, unsigned int reg_value){
    daq->tx_buffer[0] = reg_addr;
    daq->tx_buffer[1] = (reg_value >> 8) & 0xFF;
    daq->tx_buffer[2] = reg_value & 0xFF;

    amc7812_spi_xfer(daq);
}

/**
 * Performs a soft reset of the DAQ
 */
static void amc7812_soft_reset(Amc7812_t* daq){
    amc7812_write_reg(daq, AMC_REG_SOFT_RESET, AMC_SOFT_RESET_VALUE);
}


/**
 * Pulses the CNVT line low for 1us to initiate a conversion
 */
static void amc7812_start_conv(Amc7812_t* daq){
    gpio_set_value(AMC_CNVT_GPIO, 0);
    udelay(1);
    gpio_set_value(AMC_CNVT_GPIO, 1);
}

/**
 * What to do when DAV pin signals data available. First it calls
 * amc7812_read_all_channels(), then stuffs the data into the selected
 * active buffer. 
 */
static irq_handler_t amc7812_dav_handler(unsigned int irq, void* dev_id, struct pt_regs* regs){
    unsigned long long timestamp = ktime_get_ns();
    unsigned long long delta = timestamp - amc_daq.int_timestamp;
    unsigned long long step_per_chan = delta >> 4; // divide by 16

    amc7812_read_all_channels(&amc_daq, amc_daq.int_timestamp, step_per_chan);
    
    uint8_t tmp_buf[CHUNK_DELIM_COUNT] = {0};
    int i;
    for(i = 0; i < CHUNK_DELIM_COUNT; i++){
        tmp_buf[i] = CHUNK_DELIM;
    }

    /* put all data all at once into active buffer */

    bytes_available[active_data_buf] = 0; // reset count of bytes available

    memcpy(data_bufs[active_data_buf] + bytes_available[active_data_buf], tmp_buf, CHUNK_DELIM_COUNT);
    bytes_available[active_data_buf] += CHUNK_DELIM_COUNT;

    for(i = 0; i < 16; i++){
        if (bytes_available[active_data_buf] + sizeof(adc_sample_t) >= DATA_BUF_MAX){
            break;
        }
        memcpy(data_bufs[active_data_buf] + bytes_available[active_data_buf], amc_daq.samples[i].data, sizeof(adc_sample_t));
        bytes_available[active_data_buf] += sizeof(adc_sample_t);
    }

    // we also need to signal to the dev_read function that it can actually read
    // from the just populated buffer starting at index 0
    start_read_index[active_data_buf] = 0; 

    /* switch which buffer is active */
    // switch to write to other buffer on next interrupt
    // this also means the read function will extract data from the buffer
    // that we just populated
    active_data_buf = (active_data_buf + 1) % 2; 

    

    amc_daq.int_timestamp = timestamp;

    return (irq_handler_t) IRQ_HANDLED;
}


///////////////////////////////////////////////////////////////////////////////
//
// INIT AND EXIT MACROS
//
///////////////////////////////////////////////////////////////////////////////

module_init(amc7812_driver_init);
module_exit(amc7812_driver_exit);