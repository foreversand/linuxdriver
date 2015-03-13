/**************************************************************
*
* Copyright (C) 2008 VIA Technologies, Inc. All Rights Reserved.
*
* Information in this file is the intellectual property of
* VIA Technologies, Inc., and may contains trade secrets that must be stored
* and viewed confidentially.
*
* Filename:  via_spi.h
* Version:   2.0.0
* Date:      2008-11-04
* Author:    AnyongXia
*
* Functionality: This defines all registers in VIA SPI controller
*
**************************************************************/
#ifndef __VIA_SPI_H__
#define __VIA_SPI_H__

#include <linux/spi/spi.h>

/* control whether print informations */
/*#define VIA_SPI_DBG*/

#ifdef VIA_SPI_DBG
#define via_spi_printk printk
#else
#define via_spi_printk(...)
#endif

#define M25P80_DEVICE_NAME    "m25p80"

/* support four transfer types:
  * type0: only read data from spi bus. max_rx_len decides the max
  *           length for one transfer, max_tx_len has no sense
  * type1: only write data from spi bus. max_tx_len decides the max
  *           length for one transfer, max_rx_len has no sense.
  * type2: write data firstly, then read data from device. max_rx_len
  *           and max_tx_len decides the max receive data length and
  *           max transmit data length in one transfer. max_rx_tx_len
  *           decides the total max transfer length in one transfer.
  * type3: write two discontious data block to device. max_tx_len
  *           decides the max transfer length. max_rx_len and
  *           max_tx_rx_len have no sense.
  */
struct via_spi_trans_type {
    u32 max_tx_len;
    u32 max_rx_len;
    u32 max_rx_tx_len;
};



struct via_spi_message{

    /* spi_command is the spi_message structure passed from SPI device
	 * driver
	 */
   struct spi_message *spi_command;

    /* max data length in one transfer(receive or transmit) the SPI controller
	 *supporting
	 */

    struct via_spi_trans_type trans_type[4];

    /* the parse result of device module.
	 * 0 means that this command is not parsed;
	 * 1 means that this command is parsed successfully;
	 * 2 means that this command can't be parsed successfully
	 */
    unsigned    parse_state;

    /* this buffer is supported by controller module to save some special
	 * data. For example, for m25p80 flash device command, we can use this
	 * buffer to save the command and address for every via_spi_transfer
	 */
    void        *via_backup_buffer;
    unsigned    via_backup_buf_len;

    /* this buffer is supported by SPI controller module to allocate sub
	 * transfer structure
	 */
    void        *via_spi_transfer_buffer;
    unsigned    via_spi_transfer_buffer_len;

	struct list_head via_transfers;
};

struct via_spi_transfer{

    /* transfer_type define the type of this transfer:
	 * 0 means that this transfer only reads data from SPI device;
	 * 1 means that this transfer only writes data from SPI device;
	 * 2 means that this transfer writes data to device firstly, then
	 *   reads data from SPI device;
	 * 3 means that this transfer will write two block data to SPI device
	 *   and the buffers of the two block is not continous
	 */

    u8    transfer_type;

	/* if this transfer is a fast read type transfer, then there will be a
	 * dummy byte following command type and target address
	 */

     u8  is_fast_read_transfer;

    /* tx_buf and tx_len defines the start address and length of the data
	 * writen to SPI device. rx_buf and rx_len define the start address
	 * and length of the data read from the SPI device. when the
	 * transfer_type is 3, rx_buf and rx_len define the first data block's
	 * start address and length
	 */

    unsigned char  *tx_buf;
    unsigned char  *rx_buf;
    unsigned    tx_len;
    unsigned    rx_len;

    /* actual_tx_len defines the length of the data which is requsted to
	 * transfer to device by SPI device driver. actual_rx_len defines the
	 * length of the data which is requested to read from device by SPI
	 * device driver. they are may less than tx_len and rx_len because
	 * controller module may need transfer extra data as command code
	 * and address to the device
	 */
    unsigned    actual_tx_len;
    unsigned    actual_rx_len;

    /* the byte width of this transfer's data */
    u8          bits_per_word;

    /* the clock speed this transfer requested */
    u32         speed_hz;

    /* delay how much usens after this transfer */
    u16         delay_usecs;

    void (*transfer_callback)(struct spi_device *spi_dev,
			void *context);
    void *transfer_callback_context;

    struct list_head via_transfer_list;
};

struct via_spi_device{
    char    modalias[20];

    /* slower signaling on noisy or low voltage boards */
    u32     max_speed_hz;


    /* bus_num is board specific and matches the bus_num of some
	 * spi_master that will probably be registered later.
	 *
	 * chip_select reflects how this chip is wired to that master;
	 * it's less than num_chipselect.
	 */
    u16     bus_num;
    u16     chip_select;

    /* mode becomes spi_device.mode, and is essential for chips
	 * where the default of SPI_CS_HIGH = 0 is wrong.
	 */
    u8      mode;

    /* bits_per_word defines the byte width this SPI device supporting */
    u8      bits_per_word;

    /* platform_data define the device's private information which will
	 * passed to the spi_dev->dev.platform_data
	 */
    void *platform_data;

    /* via_parse_spi_cmd is the routine registered to controller module to
	 * parse the SPI command based on the device's feature
	 */
    int (*via_parse_spi_cmd)(struct spi_device *spi,
			struct via_spi_message *spi_cmd);
	/* bSST25vf080bType decides whether this device is SST25vf080b type device
	  * for this type device, the write command has different definition in
	  *command code
	  */
	u8	bSST25vf080bType;

    /* spi_dev is the linux SPI subsystem object of this device */
    struct spi_device *spi_dev;
};

int via_spi_register_device(struct via_spi_device *spi_dev);

int via_spi_unregister_device(struct via_spi_device *spi_dev);

int m25p80_parse_spi_cmd(struct spi_device *spi_dev,
		struct via_spi_message *spi_cmd);
int via_spi_transfer(struct spi_device *spi_dev, struct spi_message *m);

#endif /* __VIA_SPI_H__ */

