/**************************************************************
*
* Copyright (C) 2008 VIA Technologies, Inc. All Rights Reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version. 
*
* Filename:  via_spi.c
* Version:   2.5.1
* Date:      2011-5-26
* Author:    AnyongXia/CobeChen
* Signed-off-by: CobeChen<CobeChen@viatech.com.cn>
*
* Functionality: As the VIA SPI controller's driver, it supply the interfaces
* for customer who develops their own spi slave device driver for the device
* connnected to our controller, which only works master mode.
*
**************************************************************/

#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
#include <linux/workqueue.h>
#endif


#include "via_spi.h"

#define PCI_VENDOR_ID_VIA    0x1106
#define PCI_DEVICE_ID_VIA_SB_VT3353    0x8353
#define PCI_DEVICE_ID_VIA_SB_VT3409    0x8409
#define PCI_DEVICE_ID_VIA_SB_VT3410    0x8410
#define PCI_DEVICE_ID_VIA_SB_VT3402    0x3402
#define PCI_DEVICE_ID_VIA_SB_VT3456    0x345B

#define VIA_SPI_NAME    "via_spi"
#define VIA_SPI_DRV_VERSION "v2.5.1"

#define MAX_MASTER_NUMBER 4
#define MAX_DEVICE_NUMBER 4

#define VIA_SPI_TRANSFER_TYPE0  0  /* only read data from device */
#define VIA_SPI_TRANSFER_TYPE1  1  /* only write data to device */
/* write data to device, then read data from device*/
#define VIA_SPI_TRANSFER_TYPE2  2
/* write two discoutinous block data to device */
#define VIA_SPI_TRANSFER_TYPE3  3

#define PARSE_STATE_NOT_PARSING          0
#define PARSE_STATE_SUCCESS              1
#define PARSE_STATE_FAIL                 2

#define VIA_SPI_CONTROLLER_MODE_MASTER 1
#define VIA_SPI_CONTROLLER_MODE_SLAVE  2

#define VIA_SPI_TRANSFER_MODE_DMA      1
#define VIA_SPI_TRANSFER_MODE_PIO      2

#define VIA_SPI_DEVICE_MODE_FLASH      1
#define VIA_SPI_DEVICE_MODE_DEVICE     2

#define TRANSFER_TIME_OUT_S            1000  /* 5 second at most */

#define DATA_LEN_THRESHOLD_FOR_INT     16

#define VT3409_SPI_INTERRUPT_LINE       11

struct via_proc_file {
    char *name;
    mode_t mode;
    int (*show)(char *buffer, char **start, off_t offset, int length,
			int *eof, void *data);
    int (*store)(struct file *file, const char __user *buf,
			unsigned long count, void *data);
};

struct via_spi_master_extension {
    struct spi_master *master;

    unsigned char bus_number;

    struct workqueue_struct *workqueue;
    struct work_struct  work;

    struct list_head    queue;
    spinlock_t queue_lock;
    u8          busy;

    struct resource *ioarea;
    void __iomem    *regs;

    struct completion  transfer_done;
    u32     support_min_speed_hz;
    u32     current_speed_hz;
    u8      bits_per_word;

    u8      support_controller_mode;
    u8      current_controller_mode;

    u8      support_transfer_mode;
    u8      current_transfer_mode;

    u8      support_device_mode;
    u8      current_device_mode;

    u8      support_int;

    struct via_spi_trans_type trans_type[4];
    u32     type2_max_tx_len;

    void    *via_spi_message_buffer;

    void    *via_spi_transfer_buffer;
    u32     via_spi_trans_buf_len;

    unsigned char    *via_dma_w_buffer;
    u32     via_dma_w_buf_len;

    unsigned char    *via_dma_r_buffer;
    u32     via_dma_r_buf_len;

    void    *via_backup_buffer;
    u32     via_backup_buf_len;

    int     (*cmd_dispatch)(struct spi_device *spi_dev,
				struct via_spi_transfer *via_trans);
    int     (*master_isr)(void *master_ext);

    struct proc_dir_entry *master_proc;

    int fake_dev_chip_sel;
    u32 slave_mode_data_len;
    u8  slave_test_port;
    struct proc_dir_entry *bus_data_proc;
    struct proc_dir_entry *bus_data_len_proc;
    struct proc_dir_entry *slave_test_port_proc;

    struct via_spi_device *dev[MAX_DEVICE_NUMBER];
};

struct via_spi_controller{
    struct platform_device *platform_dev;
    struct pci_dev *via_busctl_pci_dev;
    u32  busctl_pci_dev_id;
    int irq;
    int num_master;
    struct resource *mmio_ioarea;
    void __iomem    *mmio_regs;
    struct spi_master *master[MAX_MASTER_NUMBER];
};

struct via_spi_master {
    u8      master_num;
    u8      num_chipselect;
    u32     support_min_speed_hz;
    u8      bits_per_word;
    u8      support_controller_mode;
    u8      support_transfer_mode;
    u8      support_device_mode;
    u8      support_int;
    int     (*master_init)(struct via_spi_controller *chip_info,
				struct via_spi_master_extension *master_ext);
    int     (*master_resume)(struct via_spi_controller *chip_info,
				struct via_spi_master_extension *master_ext);
    int     (*master_exit)(struct via_spi_controller *chip_info,
				struct via_spi_master_extension *master_ext);
    int     (*cmd_dispatch)(struct spi_device *spi_dev,
				struct via_spi_transfer *via_trans);
    int     (*master_isr)(void *master_ext);
};

u32 busctl_dev_ven_id;
static struct platform_device *via_spi_adapter;
struct via_spi_controller *via_spi_chip_info;
static struct proc_dir_entry *via_proc_spi;

static void via_spi_complete(void *arg)
{
	complete(arg);
}

/***********************************************************
*
* Function: read_bus_data
* Precondition: user cat the information of the read bus_data proc file to read
*               data from bus
* Input:  @buffer: store the returned data
*         @start: start offset of the buffer
*         @offset: the offset in the proc file
*         @length: the data length of the buffer
*         @eof: whether reaches to the end of the proc file.
*         @data: point to the via_spi_master_extension structure
* Output:the returned char number.
* Purpose: user can read data from the spi bus by cat this proc file
* Reference: none
*
***********************************************************/

static int read_bus_data(char *buffer, char **start,
				off_t offset, int length, int *eof, void *data)
{
    struct via_spi_master_extension *master_ext;
    struct spi_transfer t;
    struct spi_message m;
    int actual_data_len;
    int status = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
	DECLARE_COMPLETION(done);
#else
	DECLARE_COMPLETION_ONSTACK(done);
#endif
	via_spi_printk(KERN_INFO "read_bus_data, offset %x, len %x\n",
		(int)offset, length);
	if (!length)
		return 0;
	if (offset != 0) {
		*start = buffer + offset;
		*eof = 1;
		return 0;
	}

	master_ext = (struct via_spi_master_extension *)data;
	actual_data_len = min(length, (int) master_ext->slave_mode_data_len);
	via_spi_printk(KERN_INFO "read_bus_data: want %d, actual %d data\n",
		length, actual_data_len);

	spi_message_init(&m);
    memset(&t, 0, (sizeof t));
    t.rx_buf = buffer;
    t.len = actual_data_len;
    spi_message_add_tail(&t, &m);

	m.complete = via_spi_complete;
	m.context = &done;
	m.spi = master_ext->dev[master_ext->fake_dev_chip_sel]->spi_dev;
	status = via_spi_transfer(m.spi, &m);
	if (status == 0)
		wait_for_completion(&done);
	m.context = NULL;

    via_spi_printk(KERN_INFO "read_data_bus complete, actual read %d\n",
			m.actual_length);

    actual_data_len = m.actual_length - offset;
    if (actual_data_len < 0)
		actual_data_len = 0;
	else {
		if (actual_data_len < length)
			buffer[actual_data_len] = '\n';
		actual_data_len++;
	}

    *start = buffer + offset;
    *eof = 1;

    return actual_data_len;
}

/***********************************************************
*
* Function: write_bus_data
* Precondition: user echo data to the spi_bus_data file to write data to the
*               spi bus
* Input:  @file: the handle of the proc file
*         @buf: user buffer storing the echo infrormation
*         @count: the data length
*         @data: point to the via_spi_master_extension structure
* Output:the char number writen successfully.
* Purpose: user can write data to the spi bus by echo dat to the spi_bus_data
*          proc file
* Reference: none
*
***********************************************************/

static int write_bus_data(struct file *file,
		const char __user *buf, unsigned long length, void *data)
{
    struct via_spi_master_extension *master_ext;
    struct spi_transfer t;
    struct spi_message m;
    int actual_data_len;
    int status = 0;
    char *kernel_buf = NULL;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
	DECLARE_COMPLETION(done);
#else
	DECLARE_COMPLETION_ONSTACK(done);
#endif

    if (!length)
		return 0;

    master_ext = (struct via_spi_master_extension *)data;
    actual_data_len = min(length,
			(unsigned long)master_ext->slave_mode_data_len);
    via_spi_printk(KERN_INFO "write_bus_data: want %d, actual %d length data\n",
			(int)length, actual_data_len);

    kernel_buf = (char *)__get_free_page(GFP_KERNEL);
    if (kernel_buf == NULL)
		return 0;
    if (copy_from_user(kernel_buf, buf, actual_data_len)) {
		free_page((unsigned long)kernel_buf);
		return -EFAULT;
	}

	spi_message_init(&m);
    memset(&t, 0, (sizeof t));

    t.tx_buf = kernel_buf;
    t.len = actual_data_len;
    spi_message_add_tail(&t, &m);

	m.complete = via_spi_complete;
	m.context = &done;
	m.spi = master_ext->dev[master_ext->fake_dev_chip_sel]->spi_dev;
	status = via_spi_transfer(m.spi, &m);
	if (status == 0)
		wait_for_completion(&done);
	m.context = NULL;

    via_spi_printk(KERN_INFO "write_data_bus complete, actual write %d\n",
			m.actual_length);

    free_page((unsigned long)kernel_buf);
    return length;
}

/* when the master is in slave mode, we doesn't introduce a device representing
 * this slave master to the system to avoid writing a device driver. we just
 * introduce a proc file to the system, by which user can read/write data
 * from/to the spi bus. we just want to verify the slave mode hardware design,
 * we need not introduce a device to the system.
 */

struct via_proc_file spi_bus_data = {
	.name = "spi_bus_data",
	.mode = 0644,
	.show = read_bus_data,
	.store = write_bus_data,
};

/***********************************************************
*
* Function: show_bus_data_len
* Precondition: user cat the information of the bus_data_len proc file to get
*               the data length which will be read/write from/to bus
* Input:  @buffer: store the returned data
*         @start: start offset of the buffer
*         @offset: the offset in the proc file
*         @length: the data length of the buffer
*         @eof: whether reaches to the end of the proc file.
*         @data: point to the via_spi_master_extension structure
* Output:the returned char number.
* Purpose: user can get the information of current data length by cat this proc
*          file
* Reference: none
*
***********************************************************/

static int show_bus_data_len(char *buffer, char **start,
				off_t offset, int length, int *eof, void *data)
{
    struct via_spi_master_extension *master_ext;
    int data_len = 0;
	via_spi_printk(KERN_INFO "show_bus_data_len, offset %x, length %x\n",
			(int)offset, length);
	master_ext = (struct via_spi_master_extension *)data;
    data_len = snprintf(buffer, 6, "%d\n", master_ext->slave_mode_data_len);

    data_len -= offset;
    if (data_len < 0)
		data_len = 0;

    *eof = (data_len <= length) ? 1 : 0;
    *start = buffer + offset;

	return data_len;
}

/***********************************************************
*
* Function: store_bus_data_len
* Precondition: user echo data lenght to the spi_bus_data_len file to setting
*               the length of the data which will be read/write by spi_bus_data
*               proc file
* Input:  @file: the handle of the proc file
*         @buf: user buffer storing the echo infrormation
*         @count: the data length
*         @data: point to the via_spi_master_extension structure
* Output:the char number writen successfully.
* Purpose: user can setting the data len which will be read/write by
*          spi_bus_data proc file by echo data to the spi_bus_data_len proc file
* Reference: none
*
***********************************************************/

static int store_bus_data_len(struct file *file,
		const char __user *buf, unsigned long count, void *data)
{
    struct via_spi_master_extension *master_ext;
    char *kernel_buf;
    int i;
    int input_value;

    master_ext = (struct via_spi_master_extension *)data;

    kernel_buf = kmalloc(count, GFP_KERNEL);
    if (kernel_buf == NULL)
		return -ENOMEM;

    if (copy_from_user(kernel_buf, buf, count))
		return -EFAULT;

    for (i = 0; i < count - 1; i++) {
		if ((kernel_buf[i] > '9') || (kernel_buf[i] < '0'))
			return -EINVAL;
    }



    input_value = (int)simple_strtol(kernel_buf, NULL, 10);
    via_spi_printk(KERN_INFO "via_spi:store %x to data_len proc file\n",
			(u32)input_value);
    if ((input_value <= 0) || (input_value > 4096) ||
		((input_value >= 125) && (input_value <= 131)) ||
		((input_value >= 253) && (input_value <= 259)) ||
		((input_value >= 381) && (input_value <= 387)) ||
		((input_value >= 509) && (input_value <= 512))) {
		 via_spi_printk(KERN_INFO "via_spi:invalid bus data len\n");
		 return -EINVAL;
	}

    master_ext->slave_mode_data_len = (u32)input_value;

	kfree(kernel_buf);

	return count;
}

struct via_proc_file spi_bus_data_len = {
	.name = "spi_bus_data_len",
	.mode = 0644,
	.show = show_bus_data_len,
	.store = store_bus_data_len,
};


/***********************************************************
*
* Function: m25p80_write_cmd_callback
* Precondition: the target device has the same operation feature as m25p80
*               flash memory
* Input:
*     @spi_dev: pointer to the spi_device structure. the target device
*     @context:    pointer to the via_spi_master_extension structure
* Output: none
* Purpose:
*     the callback routine for a m25p80 write transfer. after issue the write
*     command to m25p80, need issue read status command to m25p80 and wait
*     until the m25p80 is ready.
* Reference: none
*
***********************************************************/

void m25p80_write_cmd_callback(struct spi_device *spi_dev, void *context)
{
    struct via_spi_master_extension *master_ext;
    u8 status = 0;
    struct via_spi_transfer via_trans, *vspt;
    unsigned char cmd[4];
    u8 int_backup;

	via_spi_printk(KERN_INFO "enter m25p80_write_cmd_callback routine\n");

    master_ext = (struct via_spi_master_extension *)context;
    if (master_ext->bus_number != 0)
		return;
    //if (spi_dev->chip_select != 0)
//		return;

    int_backup = master_ext->support_int;
    master_ext->support_int = 0;

    do {
		vspt = &via_trans;
		memset(vspt, 0, sizeof(struct via_spi_transfer));
		INIT_LIST_HEAD(&vspt->via_transfer_list);
		vspt->transfer_type = VIA_SPI_TRANSFER_TYPE2;
		vspt->rx_buf = &status;
		vspt->rx_len = 1;
		vspt->tx_buf = cmd;
		cmd[0] = 0x05; /* read status command */
		cmd[1] = cmd[2] = cmd[3] = 0;
		vspt->tx_len = 1;
		vspt->actual_rx_len = vspt->actual_tx_len = 0;
		master_ext->cmd_dispatch(spi_dev, vspt);
    } while ((status & 0x01) == 1);

    master_ext->support_int = int_backup;

	via_spi_printk(KERN_INFO "out m25p80_write_cmd_callback routine\n");
}

/***********************************************************
*
* Function: m25p80_read_jedec_cmd_callback
* Precondition: the target device has the same operation feature as m25p80
*               flash memory
* Input:
*     @spi_dev: pointer to the spi_device structure. the target device
*     @context:    pointer to the via_spi_master_extension structure
* Output: none
* Purpose:
*     the callback routine for a m25p80 read JEDEC transfer. The SST25VF080B
*     chip has specific write command code, need identify this chip
* Reference: none
*
***********************************************************/

void m25p80_read_jedec_cmd_callback(struct spi_device *spi_dev,
	void *context)
{
    unsigned char *jedec_buf;
	u32 		jedec;
	struct via_spi_master_extension *master_ext = NULL;
	struct via_spi_device *via_spi_dev = NULL;
	u8 status = 0;
	struct via_spi_transfer via_trans, *vspt;
	unsigned char cmd[4];
	u8 int_backup;

	master_ext = (struct via_spi_master_extension *)spi_dev->controller_data;
	if (master_ext == NULL) {
		via_spi_printk(KERN_ALERT "master_ext is null, error\n");
		return;
	}
	via_spi_dev = master_ext->dev[spi_dev->chip_select];
	if (via_spi_dev == NULL) {
		via_spi_printk(KERN_ALERT "the target via_spi_device is NULL\n");
		return;
	}
	
		
	via_spi_printk(KERN_INFO "enter m25p80_read_jedec_cmd_callback routine\n");
	jedec_buf = (unsigned char *)context;
	jedec = jedec_buf[0];
	jedec = jedec << 8;
	jedec |= jedec_buf[1];
	jedec = jedec << 8;
	jedec |= jedec_buf[2];
	if (jedec == 0xbf258e) {
		via_spi_printk(KERN_INFO "the flash device is sst25vf080b\n");
		via_spi_dev->bSST25vf080bType = 1;
	} else {
		via_spi_printk(KERN_INFO "the flash device is not sst25vf080b\n");
		via_spi_dev->bSST25vf080bType = 0;
	}

	if (via_spi_dev->bSST25vf080bType == 0) {
		via_spi_printk(KERN_INFO "out m25p80_write_cmd_callback routine\n");
		return;
	}    

	via_spi_printk(KERN_INFO "clear write protection for SST25VF080\n");
	int_backup = master_ext->support_int;
	master_ext->support_int = 0;

	vspt = &via_trans;
	memset(vspt, 0, sizeof(struct via_spi_transfer));
	INIT_LIST_HEAD(&vspt->via_transfer_list);
	vspt->transfer_type = VIA_SPI_TRANSFER_TYPE2;
	vspt->rx_buf = &status;
	vspt->rx_len = 1;
	vspt->tx_buf = cmd;
	cmd[0] = 0x05; /* read status command */
	cmd[1] = cmd[2] = cmd[3] = 0;
	vspt->tx_len = 1;
	vspt->actual_rx_len = vspt->actual_tx_len = 0;
	master_ext->cmd_dispatch(spi_dev, vspt);
	via_spi_printk(KERN_INFO "the chip's status is %x\n", status);

	vspt = &via_trans;
	memset(vspt, 0, sizeof(struct via_spi_transfer));
	INIT_LIST_HEAD(&vspt->via_transfer_list);
	vspt->transfer_type = VIA_SPI_TRANSFER_TYPE1;
	vspt->rx_buf = NULL;
	vspt->rx_len = 0;
	vspt->tx_buf = cmd;
	cmd[0] = 0x06; /* write enable command */
	cmd[1] = cmd[2] = cmd[3] = 0;
	vspt->tx_len = 1;
	vspt->actual_rx_len = vspt->actual_tx_len = 0;
	master_ext->cmd_dispatch(spi_dev, vspt);
	via_spi_printk(KERN_INFO "enable write\n");
	
	vspt = &via_trans;
	memset(vspt, 0, sizeof(struct via_spi_transfer));
	INIT_LIST_HEAD(&vspt->via_transfer_list);
	vspt->transfer_type = VIA_SPI_TRANSFER_TYPE1;
	vspt->rx_buf = NULL;
	vspt->rx_len = 0;
	vspt->tx_buf = cmd;
	cmd[0] = 0x50; /* enable write status command */
	cmd[1] = cmd[2] = cmd[3] = 0;
	vspt->tx_len = 1;
	vspt->actual_rx_len = vspt->actual_tx_len = 0;
	master_ext->cmd_dispatch(spi_dev, vspt);
	via_spi_printk(KERN_INFO "enable write status register\n");

	vspt = &via_trans;
	memset(vspt, 0, sizeof(struct via_spi_transfer));
	INIT_LIST_HEAD(&vspt->via_transfer_list);
	vspt->transfer_type = VIA_SPI_TRANSFER_TYPE1;
	vspt->rx_buf = NULL;
	vspt->rx_len = 0;
	vspt->tx_buf = cmd;
	cmd[0] = 0x01; /* write status register command */
	cmd[1] = cmd[2] = cmd[3] = 0;
	vspt->tx_len = 2;
	vspt->actual_rx_len = vspt->actual_tx_len = 0;
	master_ext->cmd_dispatch(spi_dev, vspt);
	via_spi_printk(KERN_INFO "write %x to the flash's status register\n",
		cmd[1]);

	vspt = &via_trans;
	memset(vspt, 0, sizeof(struct via_spi_transfer));
	INIT_LIST_HEAD(&vspt->via_transfer_list);
	vspt->transfer_type = VIA_SPI_TRANSFER_TYPE2;
	vspt->rx_buf = &status;
	vspt->rx_len = 1;
	vspt->tx_buf = cmd;
	cmd[0] = 0x05; /* read status command */
	cmd[1] = cmd[2] = cmd[3] = 0;
	vspt->tx_len = 1;
	vspt->actual_rx_len = vspt->actual_tx_len = 0;
	master_ext->cmd_dispatch(spi_dev, vspt);
	via_spi_printk(KERN_INFO "the chip's status is %x\n", status);
	
	master_ext->support_int = int_backup;
}

/***********************************************************
*
* Function: m25p80_parse_write_cmd
* Precondition: the target device has the same operation feature as m25p80 flash
*               memory
* Input:
*     @spi_dev:    the target spi device
*     @spi_cmd:   the command from the spi device driver
* Output: int, whether the spi command is parsed successfully based on the via
*         spi protocol
* Purpose:
*     parse the passed spi write command based on the m25p80 flash device
*     feature and the via spi protocol interface
* Reference: none
*
***********************************************************/

int m25p80_parse_write_cmd(struct spi_device *spi_dev,
		struct via_spi_message *spi_cmd)
{
    struct spi_message *m;
    struct spi_transfer *first_t;
    struct spi_transfer *sec_t;
    struct via_spi_transfer *vspt;
    void   *vspt_base_addr = NULL;
    void   *vspt_cmd_base_addr = NULL;
    int vspt_buf_len = 0;
    int vspt_max_num = 0;
    int vspt_backbuf_len = 0;
    int vspt_max_trans_num = 0;
    int vspt_num = 0;
    int data_offset_num = 0;
    unsigned int data_len = 0;
    unsigned int actual_data_len = 0;
    unsigned int flash_addr;
    u8 transfer_type;
    u32 max_transfer_len;
	unsigned char *tx_buffer;
	int i;
	struct spi_master *master = spi_dev->master;
    struct via_spi_master_extension *master_ext = spi_dev->controller_data;
	struct via_spi_device *via_spi_dev = NULL;
	//u8 bSpecialWriteDataCmd = 0;
   
    if (master_ext != spi_master_get_devdata(master)) {
		via_spi_printk(KERN_ALERT "the master and ext don't match\n");
		return -EINVAL;
    }
	via_spi_dev = master_ext->dev[spi_dev->chip_select];
	if (via_spi_dev == NULL) {
		via_spi_printk(KERN_ALERT "the via_spi_dev is NULL\n");
		return -EINVAL;
    }

    vspt_buf_len = spi_cmd->via_spi_transfer_buffer_len;
    vspt_base_addr = spi_cmd->via_spi_transfer_buffer;
    vspt_backbuf_len = spi_cmd->via_backup_buf_len;
    vspt_cmd_base_addr = spi_cmd->via_backup_buffer;
    if ((vspt_cmd_base_addr == NULL) || (vspt_backbuf_len == 0)) {
		via_spi_printk(KERN_ALERT "via_spi: via_spi_trans buffer" \
				"is invalid\n");
		return -ENOMEM;
    }

    vspt_max_num = vspt_buf_len / (sizeof(struct via_spi_transfer));
    m = spi_cmd->spi_command;
    first_t = container_of(m->transfers.next,
				struct spi_transfer, transfer_list);
    sec_t = container_of(first_t->transfer_list.next,
				struct spi_transfer, transfer_list);

    if (first_t->transfer_list.next->next == &first_t->transfer_list) {
		via_spi_printk(KERN_ALERT "via_spi:must have two trans\n");
		return -EINVAL;
    }

    via_spi_printk(KERN_INFO "the command has two via_transfer\n");
	tx_buffer = (unsigned char *)(first_t->tx_buf);
    via_spi_printk(KERN_INFO "command transfer:(len:%x),(tx_buf:%p),"\
		"(tx_data: %x, %x, %x, %x)\n",
		first_t->len, first_t->tx_buf, tx_buffer[0], tx_buffer[1],
		tx_buffer[2], tx_buffer[3]);
    via_spi_printk(KERN_INFO "data: rx/tx_len %x, rx_buf %p, tx_buf %p\n",
			sec_t->len, sec_t->rx_buf, sec_t->tx_buf);

	/*
	if ((tx_buffer[0] == 0x02)&& (via_spi_dev->bSST25vf080bType == 1)) {
		via_spi_printk(KERN_INFO "this is a write command for SST25vf080b\n");
		bSpecialWriteDataCmd = 1;
	}
	*/

	tx_buffer = (unsigned char *)(sec_t->tx_buf);
	via_spi_printk(KERN_INFO "the data is:");
	for (i = 0; i < 16; i++)
		via_spi_printk("%x,", tx_buffer[i]);
	via_spi_printk("\n");

	via_spi_printk(KERN_INFO "via_m25p80_parse: write data to device\n");

 	transfer_type = VIA_SPI_TRANSFER_TYPE3;
	max_transfer_len = spi_cmd->trans_type[transfer_type].max_tx_len;
	tx_buffer = (unsigned char *)(first_t->tx_buf);

	if (((via_spi_chip_info->busctl_pci_dev_id == 0x84101106) ||
	     (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)) &&
		(via_spi_dev->bSST25vf080bType == 1) &&
		(tx_buffer[0] == 0x02)) {
		via_spi_printk(KERN_INFO "this command is a special write command\n");
		max_transfer_len = 0x05;
	}

    if ((first_t->len + sec_t->len) <= max_transfer_len) {
		via_spi_printk(KERN_INFO "cmd need one via_spi_transfer\n");
		vspt = (struct via_spi_transfer *)(vspt_base_addr +
				vspt_num * sizeof(struct via_spi_transfer));
		memset(vspt, 0, sizeof(struct via_spi_transfer));
		INIT_LIST_HEAD(&vspt->via_transfer_list);
		vspt->transfer_type = transfer_type;
		vspt->rx_buf = (unsigned char *)first_t->tx_buf;
		vspt->rx_len = vspt->actual_rx_len = first_t->len;
		vspt->tx_buf = (unsigned char *)sec_t->tx_buf;
		vspt->tx_len = vspt->actual_tx_len = sec_t->len;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
		vspt->bits_per_word = 8;
		vspt->speed_hz = 0;
#else
		vspt->bits_per_word = sec_t->bits_per_word;
		vspt->speed_hz = sec_t->speed_hz;
#endif
		vspt->delay_usecs = sec_t->delay_usecs;
		list_add_tail(&vspt->via_transfer_list,
			&spi_cmd->via_transfers);
		spi_cmd->parse_state = PARSE_STATE_SUCCESS;

		/*
		if ((bSpecialWriteDataCmd == 1) && (sec_t->len > 1)) {
			via_spi_printk(KERN_INFO "redefine the command code\n");
			tx_buffer = (unsigned char *)(first_t->tx_buf);
			tx_buffer[0] = 0xAD;
		}
		*/
		return 0;
    }

    via_spi_printk(KERN_INFO "cmd need more than one via_spi_transfer\n");
    vspt_max_trans_num = vspt_backbuf_len / first_t->len;
    vspt_max_num = min(vspt_max_num, vspt_max_trans_num);
	via_spi_printk(KERN_INFO "the max via_spi_transfer num is %d\n",
		vspt_max_num);

    tx_buffer = (unsigned char *)(first_t->tx_buf);
	flash_addr = (((tx_buffer[1] << 16) & 0x00FF0000) |
				((tx_buffer[2] << 8) & 0x0000FF00) |
				(tx_buffer[3] & 0x000000FF));
    data_len = sec_t->len;
    via_spi_printk(KERN_INFO "addr:%x, data len: %d, max len: %d\n",
			flash_addr, data_len, max_transfer_len - first_t->len);
    while ((data_len > 0) && (vspt_num < vspt_max_num)) {
		actual_data_len = min(max_transfer_len - first_t->len,
				data_len);

	/* for m25p80 write command, must issue a write enable command to the
	 * device before each write data command. if we divide one spi_transfer
	 * into some via_spi_transfers, we have to issue write enable command
	 * to the m25p80 before each via_spi_transfer except the first one. for
	 * the first one, m25p80 device driver have issued a write enable cmd
	 * to it
	 */

		if (vspt_num != 0) {
			vspt = (struct via_spi_transfer *)(vspt_base_addr +
				vspt_num * sizeof(struct via_spi_transfer));
			memset(vspt, 0, sizeof(struct via_spi_transfer));
			INIT_LIST_HEAD(&vspt->via_transfer_list);
			vspt->transfer_type = VIA_SPI_TRANSFER_TYPE1;
			vspt->rx_buf = NULL;
			vspt->rx_len = 0;
			vspt->tx_buf = vspt_cmd_base_addr +
				(vspt_num - 1) * first_t->len;
			vspt->tx_buf[0] = 0x06; /* write enable command */
			vspt->tx_buf[1] = vspt->tx_buf[2] = vspt->tx_buf[3] = 0;
			vspt->tx_len = 1;
			vspt->actual_rx_len = vspt->actual_tx_len = 0;
			list_add_tail(&vspt->via_transfer_list,
					&spi_cmd->via_transfers);
			vspt_num += 1;
		}
		vspt = (struct via_spi_transfer *)(vspt_base_addr +
				vspt_num * sizeof(struct via_spi_transfer));
		memset(vspt, 0, sizeof(struct via_spi_transfer));
		INIT_LIST_HEAD(&vspt->via_transfer_list);
		vspt->transfer_type = transfer_type;

		if (data_offset_num == 0) {
			vspt->rx_buf = (unsigned char *)first_t->tx_buf;
		} else {
			vspt->rx_buf = vspt_cmd_base_addr +
				(vspt_num - 1) * first_t->len;
			tx_buffer = (unsigned char *)(first_t->tx_buf);
			/*
			if ((bSpecialWriteDataCmd == 1) && (actual_data_len> 1)) {
				via_spi_printk(KERN_INFO "redefine the command code\n");
				vspt->rx_buf[0] = 0xAD;
			} else {
				vspt->rx_buf[0] = tx_buffer[0];
			}
			*/
			vspt->rx_buf[0] = tx_buffer[0];
			vspt->rx_buf[1] = (u8)((flash_addr >> 16) & 0x000000FF);
			vspt->rx_buf[2] = (u8)((flash_addr >> 8) & 0x000000FF);
			vspt->rx_buf[3] = (u8)(flash_addr & 0x000000FF);
		}
		vspt->rx_len = first_t->len;
		if (data_offset_num == 0)
			vspt->actual_rx_len = first_t->len;
		else
			vspt->actual_rx_len = 0;

		vspt->tx_buf = (unsigned char *)(sec_t->tx_buf +
			data_offset_num * (max_transfer_len - first_t->len));
		vspt->tx_len = vspt->actual_tx_len = actual_data_len;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
		vspt->bits_per_word = 8;
		vspt->speed_hz = 0;
#else
		vspt->bits_per_word = sec_t->bits_per_word;
		vspt->speed_hz = sec_t->speed_hz;
#endif
		vspt->delay_usecs = sec_t->delay_usecs;
		vspt->transfer_callback = m25p80_write_cmd_callback;
		vspt->transfer_callback_context = (void *)master_ext;

		via_spi_printk(KERN_INFO "%dth tran:addr:%x,len:%x, buf:%p\n",
			vspt_num, flash_addr, vspt->tx_len, vspt->tx_buf);

		list_add_tail(&vspt->via_transfer_list,
				&spi_cmd->via_transfers);
		data_len -= actual_data_len;
		vspt_num += 1;
		data_offset_num += 1;
		flash_addr += actual_data_len;
	}

	/*
	if (bSpecialWriteDataCmd == 1) {
		via_spi_printk(KERN_INFO "redefine the command code for first tran\n");
		tx_buffer = (unsigned char *)(first_t->tx_buf);
		tx_buffer[0] = 0xAD;
	}
	*/
	
    via_spi_printk(KERN_INFO "final, need %d trans, remain data len: %x\n",
			vspt_num, data_len);
	spi_cmd->parse_state = PARSE_STATE_SUCCESS;
	return 0;
}

/***********************************************************
*
* Function: m25p80_parse_read_cmd
* Precondition: the target device has the same operation feature as m25p80
*               flash memory
* Input:
*     @spi_dev:    the target spi device
*     @spi_cmd:   the command from the spi device driver
* Output: int, whether the spi command is parsed successfully based on the via
*         spi protocol
* Purpose:
*     parse the passed spi read command based on the m25p80 flash device
*     feature and the via spi protocol interface
* Reference: none
*
***********************************************************/

int m25p80_parse_read_cmd(struct spi_device *spi_dev,
		struct via_spi_message *spi_cmd)
{
    struct spi_message *m;
    struct spi_transfer *first_t;
    struct spi_transfer *sec_t;
    struct via_spi_transfer *vspt;
    void   *vspt_base_addr = NULL;
    void   *vspt_cmd_base_addr = NULL;
    int vspt_buf_len = 0;
    int vspt_max_num = 0;
    int vspt_backbuf_len = 0;
    int vspt_max_trans_num = 0;
    int vspt_num = 0;
    unsigned int data_len = 0;
    unsigned int actual_data_len = 0;
    unsigned int actual_max_data_len = 0;
    unsigned int flash_addr;
    u8 transfer_type;
    u32 max_rx_len;
    u32 max_tx_len;
    u32 max_rx_tx_len;
	unsigned char *tx_buffer;
	u8 is_fast_read_cmd = 0;
	u8 is_jedec_read_cmd = 0;

    vspt_buf_len = spi_cmd->via_spi_transfer_buffer_len;
    vspt_base_addr = spi_cmd->via_spi_transfer_buffer;
    vspt_backbuf_len = spi_cmd->via_backup_buf_len;
    vspt_cmd_base_addr = spi_cmd->via_backup_buffer;
    if ((vspt_cmd_base_addr == NULL) || (vspt_backbuf_len == 0))
		return -ENOMEM;

    vspt_max_num = vspt_buf_len / (sizeof(struct via_spi_transfer));
    m = spi_cmd->spi_command;
    first_t = container_of(m->transfers.next,
				struct spi_transfer, transfer_list);
    sec_t = container_of(first_t->transfer_list.next,
				struct spi_transfer, transfer_list);

    if (first_t->transfer_list.next->next == &first_t->transfer_list) {
		via_spi_printk(KERN_ALERT "via_spi:must have two trans\n");
		return -EINVAL;
	}

    via_spi_printk(KERN_INFO "the command has two via_transfer\n");
	tx_buffer = (unsigned char *)(first_t->tx_buf);
    via_spi_printk(KERN_INFO "command transfer:(len:%x),(tx_buf:%p),"\
		"(tx_data: %x, %x, %x, %x)\n",
		first_t->len, first_t->tx_buf, tx_buffer[0], tx_buffer[1],
		tx_buffer[2], tx_buffer[3]);
	via_spi_printk(KERN_INFO "data: rx/tx_len %x, rx_buf %p, tx_buf %p\n",
			sec_t->len, sec_t->rx_buf, sec_t->tx_buf);
    via_spi_printk(KERN_INFO "via_m25p80_parse:read data from device\n");

    transfer_type = VIA_SPI_TRANSFER_TYPE2;
    max_rx_len = spi_cmd->trans_type[transfer_type].max_rx_len;
    max_tx_len = spi_cmd->trans_type[transfer_type].max_tx_len;
    max_rx_tx_len = spi_cmd->trans_type[transfer_type].max_rx_tx_len;

    if (first_t->len > max_tx_len) {
		via_spi_printk(KERN_INFO "m25p80_read_cmd: " \
			"this transfer can't be supported\n");
		return -EINVAL;
    }
	tx_buffer = (unsigned char *)(first_t->tx_buf);
	if ((first_t->len == 0x05) && (tx_buffer[0] == 0x0b)) {
		via_spi_printk(KERN_INFO "m25p80_read_cmd: fast read cmd\n");
		is_fast_read_cmd = 1;
	}

	if ((first_t->len == 0x01) && (tx_buffer[0] == 0x9F)) {
		via_spi_printk(KERN_INFO "m25p80_read_cmd: read JEDEC cmd\n");
		is_jedec_read_cmd = 1;
	}

    if (((first_t->len + sec_t->len) <= max_rx_tx_len) &&
		(sec_t->len <= max_rx_len)) {
		via_spi_printk(KERN_INFO "cmd need one via_spi_transfer\n");
		vspt = (struct via_spi_transfer *)(vspt_base_addr +
				vspt_num * sizeof(struct via_spi_transfer));
		memset(vspt, 0, sizeof(struct via_spi_transfer));
		INIT_LIST_HEAD(&vspt->via_transfer_list);
		vspt->transfer_type = transfer_type;
		vspt->tx_buf = (unsigned char *)first_t->tx_buf;
		vspt->tx_len = vspt->actual_tx_len = first_t->len;
		vspt->rx_buf = (unsigned char *)sec_t->rx_buf;
		vspt->rx_len = vspt->actual_rx_len = sec_t->len;
		vspt->delay_usecs = sec_t->delay_usecs;
		vspt->is_fast_read_transfer = is_fast_read_cmd;
		if (is_jedec_read_cmd == 1) {
			vspt->transfer_callback = m25p80_read_jedec_cmd_callback;
			vspt->transfer_callback_context = (void *)(vspt->rx_buf);
		}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
		vspt->bits_per_word = 8;
		vspt->speed_hz = 0;
#else
		vspt->bits_per_word = sec_t->bits_per_word;
		vspt->speed_hz = sec_t->speed_hz;
#endif
		list_add_tail(&vspt->via_transfer_list,
			&spi_cmd->via_transfers);
		spi_cmd->parse_state = PARSE_STATE_SUCCESS;
		return 0;
	}

	via_spi_printk(KERN_INFO "cmd need more than one via_spi_transfer\n");
    if (max_rx_len == max_rx_tx_len)
		actual_max_data_len = max_rx_len - first_t->len;
	else
		actual_max_data_len = max_rx_len;
	vspt_max_trans_num = vspt_backbuf_len / first_t->len;
    vspt_max_num = min(vspt_max_num, vspt_max_trans_num);

	tx_buffer = (unsigned char *)(first_t->tx_buf);
    flash_addr = (((tx_buffer[1] << 16) & 0x00FF0000) |
				((tx_buffer[2] << 8) & 0x0000FF00) |
				(tx_buffer[3] & 0x000000FF));
    data_len = sec_t->len;

    via_spi_printk(KERN_INFO "data transfer len: %d, max len: %d\n",
			data_len, actual_max_data_len);

    while ((data_len > 0) && (vspt_num < vspt_max_num)) {
		actual_data_len = min(actual_max_data_len, data_len);
		vspt = (struct via_spi_transfer *)(vspt_base_addr +
				vspt_num * sizeof(struct via_spi_transfer));
		memset(vspt, 0, sizeof(struct via_spi_transfer));
		INIT_LIST_HEAD(&vspt->via_transfer_list);
		vspt->transfer_type = transfer_type;
		if (vspt_num == 0) {
			vspt->tx_buf = (unsigned char *)first_t->tx_buf;
		} else {
			vspt->tx_buf = vspt_cmd_base_addr +
				(vspt_num - 1) * first_t->len;
			tx_buffer = (unsigned char *)(first_t->tx_buf);
			vspt->tx_buf[0] = tx_buffer[0];
			vspt->tx_buf[1] = (u8)((flash_addr >> 16) & 0x000000FF);
			vspt->tx_buf[2] = (u8)((flash_addr >> 8) & 0x000000FF);
			vspt->tx_buf[3] = (u8)(flash_addr & 0x000000FF);
		}
		vspt->tx_len = first_t->len;
		if (vspt_num == 0)
			vspt->actual_tx_len = first_t->len;
		else
			vspt->actual_tx_len = 0;

		vspt->rx_buf = sec_t->rx_buf + vspt_num * actual_max_data_len;
		vspt->rx_len = vspt->actual_rx_len = actual_data_len;
		vspt->delay_usecs = sec_t->delay_usecs;
		vspt->is_fast_read_transfer = is_fast_read_cmd;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
		vspt->bits_per_word = 8;
		vspt->speed_hz = 0;
#else
		vspt->bits_per_word = sec_t->bits_per_word;
		vspt->speed_hz = sec_t->speed_hz;
#endif

		via_spi_printk(KERN_INFO "%dtran: addr:%x,len:%x,buf:%p\n",
			vspt_num, flash_addr, vspt->rx_len, vspt->rx_buf);

		list_add_tail(&vspt->via_transfer_list,
			&spi_cmd->via_transfers);
		data_len -= actual_data_len;
		vspt_num += 1;
		flash_addr += actual_data_len;
	}
	via_spi_printk(KERN_INFO "total, cmd need %d trans, remain len: %x\n",
			vspt_num, data_len);

	spi_cmd->parse_state = PARSE_STATE_SUCCESS;

	return 0;
}


/***********************************************************
*
* Function: m25p80_parse_spi_cmd
* Precondition: the target device has the same operation feature as m25p80 flash
*               memory
* Input:
*     @spi_dev:    the target spi device
*     @spi_cmd:   the command from the spi device driver
* Output: int, whether the spi command is parsed successfully based on the via
*         spi protocol
* Purpose:
*     parse the passed spi command based on the m25p80 flash device feature and
*     the via spi protocol interface
* Reference: none
*
***********************************************************/

int m25p80_parse_spi_cmd(struct spi_device *spi_dev,
		struct via_spi_message *spi_cmd)
{
    struct spi_message *m;
    struct spi_transfer *first_t;
    struct spi_transfer *sec_t;
    struct via_spi_transfer *vspt;
    void   *vspt_base_addr = NULL;
    int vspt_buf_len = 0;
    int vspt_max_num = 0;
    int vspt_num = 0;
    u8 transfer_type;
    u32 max_trans_len;
	unsigned char *tx_buffer;
	unsigned char is_jedec_read_cmd = 0;
	struct spi_master *master = spi_dev->master;
	struct via_spi_master_extension *master_ext = spi_dev->controller_data;
	struct via_spi_device *via_spi_dev = NULL;
	//u8 bSpecialWriteDataCmd = 0;    

    via_spi_printk(KERN_INFO "enter the m25p80_parse_spi_cmd routine\n");

	if (master_ext != spi_master_get_devdata(master)) {
		via_spi_printk(KERN_ALERT "the master and ext don't match\n");
		return -EINVAL;
	}
	via_spi_dev = master_ext->dev[spi_dev->chip_select];
	if (via_spi_dev == NULL) {
		via_spi_printk(KERN_ALERT "the via_spi_dev is NULL\n");
		return -EINVAL;
	}

    vspt_buf_len = spi_cmd->via_spi_transfer_buffer_len;
    vspt_base_addr = spi_cmd->via_spi_transfer_buffer;
    if ((vspt_base_addr == NULL) || (vspt_buf_len == 0)) {
		via_spi_printk(KERN_ALERT "via_spi: trans buffer invalid\n");
		return -ENOMEM;
	}

    vspt_max_num = vspt_buf_len / (sizeof(struct via_spi_transfer));
    if (vspt_max_num == 0) {
		via_spi_printk(KERN_ALERT "via_spi: buffer num zero\n");
		return -ENOMEM;
    }

    m = spi_cmd->spi_command;
    first_t = container_of(m->transfers.next,
			struct spi_transfer, transfer_list);

    if (first_t->transfer_list.next->next == &first_t->transfer_list) {
		via_spi_printk(KERN_INFO "cmd has only one via_transfer\n");
		tx_buffer = (unsigned char *)(first_t->tx_buf);
		via_spi_printk(KERN_INFO "len: %x, rx_buf %p, tx_buf %p\n",
				first_t->len, first_t->rx_buf, first_t->tx_buf);
		if (first_t->tx_buf != NULL) {
			via_spi_printk(KERN_INFO "tx_data: %x, %x, %x, %x\n",
				tx_buffer[0], tx_buffer[1],
				tx_buffer[2], tx_buffer[3]);
			if (tx_buffer[0] == 0x9F) {
				via_spi_printk(KERN_INFO "read JEDEC cmd\n");
				is_jedec_read_cmd = 1;
			}
		}
		
		if ((first_t->tx_buf != NULL) && (first_t->len > 1) &&
			(first_t->rx_buf != NULL)) {
			transfer_type = VIA_SPI_TRANSFER_TYPE2;
			max_trans_len = 
				spi_cmd->trans_type[transfer_type].max_rx_tx_len;
		} else if (first_t->tx_buf != NULL) {
			transfer_type = VIA_SPI_TRANSFER_TYPE1;
			max_trans_len =
				spi_cmd->trans_type[transfer_type].max_tx_len;
		} else if (first_t->rx_buf != NULL) {
			transfer_type = VIA_SPI_TRANSFER_TYPE0;
			max_trans_len =
				spi_cmd->trans_type[transfer_type].max_rx_len;
		} else
			return -EINVAL;

		vspt = (struct via_spi_transfer *)(vspt_base_addr +
			vspt_num * sizeof(struct via_spi_transfer));
		memset(vspt, 0, sizeof(struct via_spi_transfer));
		INIT_LIST_HEAD(&vspt->via_transfer_list);
		vspt->transfer_type = transfer_type;
		if (transfer_type == VIA_SPI_TRANSFER_TYPE0) {
			vspt->rx_buf = first_t->rx_buf;
			vspt->rx_len = vspt->actual_rx_len =
				min(max_trans_len, first_t->len);
		} else if (transfer_type == VIA_SPI_TRANSFER_TYPE1) {
			vspt->tx_buf = (unsigned char *)first_t->tx_buf;
			vspt->tx_len = vspt->actual_tx_len =
				min(max_trans_len, first_t->len);
			if ((vspt->tx_buf[0] == 0x02)&& (via_spi_dev->bSST25vf080bType == 1) &&
				(vspt->tx_len > 5)) {
				via_spi_printk(KERN_INFO "this is a write command for SST25vf080b\n");
				vspt->tx_buf[0] = 0xAD;
			}		
		} else {
			vspt->tx_buf = (unsigned char *)first_t->tx_buf;
			/* the tx length is assumed to be 1, the command code */
			vspt->tx_len = vspt->actual_tx_len = 1;
			vspt->rx_buf = vspt->tx_buf + 1;
			vspt->rx_len = vspt->actual_rx_len = first_t->len - 1;

			if (is_jedec_read_cmd == 1) {
				vspt->transfer_callback = m25p80_read_jedec_cmd_callback;
				vspt->transfer_callback_context = (void *)(vspt->rx_buf);
			}
		}
		vspt->delay_usecs = first_t->delay_usecs;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
		vspt->bits_per_word = 8;
		vspt->speed_hz = 0;
#else
		vspt->bits_per_word = first_t->bits_per_word;
		vspt->speed_hz = first_t->speed_hz;
#endif
		list_add_tail(&vspt->via_transfer_list,
			&spi_cmd->via_transfers);
		spi_cmd->parse_state = PARSE_STATE_SUCCESS;
		return 0;
	}

    sec_t = container_of(first_t->transfer_list.next,
				struct spi_transfer, transfer_list);

    if (sec_t->rx_buf == NULL) {
		return m25p80_parse_write_cmd(spi_dev, spi_cmd);
    } else if (sec_t->tx_buf == NULL) {
		return m25p80_parse_read_cmd(spi_dev, spi_cmd);
    } else {
		via_spi_printk(KERN_ALERT "via_spi: rx/tx buf all valid\n");
		return -EINVAL;
    }
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 22)
struct flash_platform_data m25p40_platform_data = {
    .name = "m25p40",
    .type = "m25p40",
};
#endif


struct via_spi_device m25p80_spi_dev = {
    .modalias = "m25p80",
    .max_speed_hz = 9000000,   /* 0 means any speed is acceptable */
    .bus_num = 0,        /* first master */
    // for VT3456 verify: mount flash on port1 to verify device mode
    //.chip_select = 1,
    .chip_select = 0,    /* first port */
    .mode = 0,
    .bits_per_word = 8,
    .via_parse_spi_cmd = m25p80_parse_spi_cmd,
    .bSST25vf080bType = 0,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 22)
    .platform_data = &m25p40_platform_data,
#endif
};

/***********************************************************
*
* Function: show_test_port
* Precondition: user cat the information of the slave_test_port proc file to
*               get the test port of spi bus0.
* Input:  @buffer: store the returned data
*         @start: start offset of the buffer
*         @offset: the offset in the proc file
*         @length: the data length of the buffer
*         @eof: whether reaches to the end of the proc file.
*         @data: point to the via_spi_master_extension structure
* Output:the returned char number.
* Purpose: user can get the information of current current test port of spi bus0
* Reference: none
*
***********************************************************/

static int show_test_port(char *buffer, char **start,
		off_t offset, int length, int *eof, void *data)
{
    struct via_spi_master_extension *master_ext;
    int data_len = 0;

	via_spi_printk(KERN_INFO "show_bus_data_len, offset %x, length %x\n",
			(int)offset, length);
    master_ext = (struct via_spi_master_extension *)data;

    data_len = snprintf(buffer, 3, "%d\n", master_ext->slave_test_port);

    data_len -= offset;
    if (data_len < 0)
		data_len = 0;

    *eof = (data_len <= length) ? 1 : 0;
    *start = buffer + offset;

	return data_len;

}

/***********************************************************
*
* Function: store_test_port
* Precondition: user echo test port to the slave_test_port file to setting the
*               test port of bus0 to test the bus1 slave mode
* Input:  @file: the handle of the proc file
*         @buf: user buffer storing the echo infrormation
*         @count: the data length
*         @data: point to the via_spi_master_extension structure
* Output:the char number writen successfully.
* Purpose: user can setting the test port which will be connected to the bus1
*          to test the bus1 slave mode function
* Reference: none
*
***********************************************************/

static int store_test_port(struct file *file,
		const char __user *buf, unsigned long count, void *data)
{
    struct via_spi_master_extension *master_ext;
    char *kernel_buf;
    u8 input_value;
    struct proc_dir_entry *pde;
    struct spi_device *spi_dev;

    master_ext = (struct via_spi_master_extension *)data;

    kernel_buf = kmalloc(count, GFP_KERNEL);
    if (kernel_buf == NULL)
		return -ENOMEM;

    if (copy_from_user(kernel_buf, buf, count)) {
		kfree(kernel_buf);
		return -EFAULT;
	}

    if (kernel_buf[0] == '0') {
		via_spi_printk(KERN_INFO "via_spi: set the test port to 0\n");
		input_value = 0;
    } else if (kernel_buf[0] == '1') {
		via_spi_printk(KERN_INFO "via_spi: set the test port to 1\n");
		input_value = 1;
    } else if (kernel_buf[0] == '2') {
		via_spi_printk(KERN_INFO "via_spi: set the test port to 2\n");
		input_value = 2;
    } else {
		via_spi_printk(KERN_INFO "via_spi: has invalid param\n");
		kfree(kernel_buf);
		return -EINVAL;
    }

    if (input_value == master_ext->slave_test_port) {
		via_spi_printk(KERN_INFO "via_spi: need not change the port\n");
		kfree(kernel_buf);
		return count;
    }

    if ((master_ext->slave_test_port > 0) && (input_value > 0)) {
		via_spi_printk(KERN_INFO "via_spi: don't support the method\n");
		kfree(kernel_buf);
		return -EINVAL;
    }

    if (master_ext->slave_test_port == 0) {

		pde = create_proc_entry(spi_bus_data_len.name,
			spi_bus_data_len.mode, master_ext->master_proc);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 30)
		pde->owner      = THIS_MODULE;
#endif
		pde->data       = master_ext;
		pde->read_proc  = spi_bus_data_len.show;
		pde->write_proc = spi_bus_data_len.store;

		master_ext->bus_data_len_proc = pde;

		pde = create_proc_entry(spi_bus_data.name,
			spi_bus_data.mode, master_ext->master_proc);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 30)
		pde->owner      = THIS_MODULE;
#endif
		pde->data       = master_ext;
		pde->read_proc  = spi_bus_data.show;
		pde->write_proc = spi_bus_data.store;
		master_ext->bus_data_proc = pde;

		/* create a fake spi device for uniform processing */
		master_ext->dev[input_value] =
			kzalloc(sizeof(struct via_spi_device), GFP_KERNEL);
		memcpy(master_ext->dev[input_value], &m25p80_spi_dev,
				sizeof(struct via_spi_device));
		master_ext->dev[input_value]->chip_select = input_value;
		master_ext->dev[input_value]->spi_dev =
			kzalloc(sizeof(struct spi_device), GFP_KERNEL);
		spi_dev = master_ext->dev[input_value]->spi_dev;
		spi_dev->bits_per_word = 8;
		spi_dev->chip_select = input_value;
		spi_dev->controller_data = master_ext;
		spi_dev->master = master_ext->master;
		spi_dev->max_speed_hz = m25p80_spi_dev.max_speed_hz;
		spi_dev->mode = 0x00;
		master_ext->fake_dev_chip_sel = spi_dev->chip_select;
    } else {
		u8 chip_sel = master_ext->fake_dev_chip_sel;
		kfree(master_ext->dev[chip_sel]->spi_dev);
		kfree(master_ext->dev[chip_sel]);
		master_ext->dev[chip_sel] = NULL;
		remove_proc_entry(spi_bus_data_len.name,
			master_ext->master_proc);
		remove_proc_entry(spi_bus_data.name, master_ext->master_proc);
		master_ext->bus_data_proc = NULL;
		master_ext->fake_dev_chip_sel = -1;
    }

    master_ext->slave_test_port = input_value;

	kfree(kernel_buf);

	return count;
}

struct via_proc_file slave_test_port = {
	.name = "slave_test_port",
	.mode = 0644,
	.show = show_test_port,
	.store = store_test_port,
};


/***********************************************************
*
* Function: show_transfer_mode
* Precondition: user cat the information of the trans_mode proc file
* Input:  @buffer: store the returned data
*         @start: start offset of the buffer
*         @offset: the offset in the proc file
*         @length: the data length of the buffer
*         @eof: whether reaches to the end of the proc file.
*         @data: point to the via_spi_master_extension structure
* Output:the returned char number.
* Purpose: introduce the supported transfer mode to user
* Reference: none
*
***********************************************************/

static int show_transfer_mode(char *buffer, char **start,
		off_t offset, int length, int *eof, void *data)
{
    struct via_spi_master_extension *master_ext;
    int data_len = 0;

	via_spi_printk(KERN_INFO "show_transfer__mode, offset %x, length %x\n",
			(int)offset, length);
	master_ext = (struct via_spi_master_extension *)data;

    if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_DMA) {
		if (master_ext->support_transfer_mode &
			VIA_SPI_TRANSFER_MODE_PIO)
			data_len = snprintf(buffer, 11, "[DMA] PIO\n");
		else
			data_len = snprintf(buffer, 7, "[DMA]\n");
    } else if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_PIO) {
		if (master_ext->support_transfer_mode &
			VIA_SPI_TRANSFER_MODE_DMA)
			data_len = snprintf(buffer, 11, "DMA [PIO]\n");
		else
			data_len = snprintf(buffer, 7, "[PIO]\n");
    }

    data_len -= offset;
    if (data_len < 0)
		data_len = 0;

    *eof = (data_len <= length) ? 1 : 0;
    *start = buffer + offset;

	return data_len;
}

/***********************************************************
*
* Function: store_transfer_mode
* Precondition: user echo "PIO" or "DMA" to the trans_mode proc file
* Input:  @file: the handle of the proc file
*         @buf: user buffer storing the echo infrormation
*         @count: the data length
*         @data: point to the via_spi_master_extension structure
* Output:the char number writen successfully.
* Purpose: set the transfer mode of the master
* Reference: none
*
***********************************************************/

static int store_transfer_mode(struct file *file,
		const char __user *buf, unsigned long count, void *data)
{
    u8 target_mode;
    struct via_spi_master_extension *master_ext;
    char *kernel_buf;

    master_ext = (struct via_spi_master_extension *)data;

    if (count != sizeof("PIO"))
		return -EINVAL;

    kernel_buf = kmalloc(count, GFP_KERNEL);
    if (kernel_buf == NULL)
		return -ENOMEM;

    if (copy_from_user(kernel_buf, buf, count)) {
		kfree(kernel_buf);
		return -EFAULT;
	}

	via_spi_printk(KERN_INFO "the target transfer mode is %4.4s\n",
			kernel_buf);

    if (memcmp(kernel_buf, "PIO", count - 1) == 0) {
		via_spi_printk(KERN_INFO "via_spi: set PIO trans mode\n");
		target_mode = VIA_SPI_TRANSFER_MODE_PIO;
    } else if (memcmp(kernel_buf, "DMA", count - 1) == 0) {
		via_spi_printk(KERN_INFO "via_spi: set DMA trans mode\n");
		target_mode = VIA_SPI_TRANSFER_MODE_DMA;
    } else {
		via_spi_printk(KERN_INFO "via_spi: set unknown trans mode\n");
		kfree(kernel_buf);
		return -EINVAL;
    }

    if (!(master_ext->support_transfer_mode & target_mode)) {
		via_spi_printk(KERN_ALERT "via_spi:not support %s mode\n",
			buf);
		kfree(kernel_buf);
		return -EINVAL;
    }

    if (master_ext->current_transfer_mode == target_mode) {
		via_spi_printk(KERN_INFO "via_spi:trans mode %s need not" \
				"change\n", buf);
		kfree(kernel_buf);
		return count;
    }

    master_ext->current_transfer_mode = target_mode;

    if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_DMA) {
		master_ext->trans_type[0].max_rx_len = PAGE_SIZE;
		master_ext->trans_type[1].max_tx_len = PAGE_SIZE;
		master_ext->trans_type[2].max_rx_len = 512;
		master_ext->trans_type[2].max_tx_len = 16;
		master_ext->trans_type[2].max_rx_tx_len = 512 + 16;
		master_ext->trans_type[3].max_tx_len = 16;
    } else {
		if (master_ext->current_device_mode ==
				VIA_SPI_DEVICE_MODE_DEVICE) {
			master_ext->trans_type[0].max_rx_len = 16;
			master_ext->trans_type[1].max_tx_len = 16;
			master_ext->trans_type[2].max_rx_len = 16;
			master_ext->trans_type[2].max_tx_len = 16;
			master_ext->trans_type[2].max_rx_tx_len = 16;
			master_ext->trans_type[3].max_tx_len = 16;
		} else {
			master_ext->trans_type[0].max_rx_len = 16;
			master_ext->trans_type[1].max_tx_len = 20;
			master_ext->trans_type[2].max_rx_len = 16;
			master_ext->trans_type[2].max_tx_len = 5;
			master_ext->trans_type[2].max_rx_tx_len = 21;
			master_ext->trans_type[3].max_tx_len = 20;
		}
	}

	kfree(kernel_buf);
	return count;
}



/***********************************************************
*
* Function: show_controller_mode
* Precondition: user cat the information of the ctler_mode proc file
* Input:  @buffer: store the returned data
*         @start: start offset of the buffer
*         @offset: the offset in the proc file
*         @length: the data length of the buffer
*         @eof: whether reaches to the end of the proc file.
*         @data: point to the via_spi_master_extension structure
* Output:the returned char number.
* Purpose: introduce the supported controller mode to user
* Reference: none
*
***********************************************************/

static int show_controller_mode(char *buffer, char **start,
		off_t offset, int length, int *eof, void *data)
{
    struct via_spi_master_extension *master_ext;
    int data_len = 0;

	via_spi_printk(KERN_INFO "show_ctler__mode, offset %x, length %x\n",
		(int)offset, length);

    master_ext = (struct via_spi_master_extension *)data;

	if (master_ext->current_controller_mode ==
			VIA_SPI_CONTROLLER_MODE_MASTER) {
		if (master_ext->support_controller_mode &
				VIA_SPI_CONTROLLER_MODE_SLAVE)
			data_len = snprintf(buffer, 16, "[master] slave\n");
		else
			data_len = snprintf(buffer, 10, "[master]\n");

    } else if (master_ext->current_controller_mode ==
			VIA_SPI_CONTROLLER_MODE_SLAVE) {
		if (master_ext->support_controller_mode &
				VIA_SPI_CONTROLLER_MODE_MASTER)
			data_len = snprintf(buffer, 16, "master [slave]\n");
		else
			data_len = snprintf(buffer, 9, "[slave]\n");
    }
    data_len -= offset;
    if (data_len < 0)
		data_len = 0;

    *eof = (data_len <= length) ? 1 : 0;
    *start = buffer + offset;

	return data_len;
}

/***********************************************************
*
* Function: store_controller_mode
* Precondition: user echo "master" or "slave" to the ctler_mode proc file
* Input:  @file: the handle of the proc file
*         @buf: user buffer storing the echo infrormation
*         @count: the data length
*         @data: point to the via_spi_master_extension structure
* Output:the char number writen successfully.
* Purpose: set the controller mode of the master
* Reference: none
*
***********************************************************/

static int store_controller_mode(struct file *file,
		const char __user *buf, unsigned long count, void *data)
{
    u8 target_mode;
    struct via_spi_master_extension *master_ext;
    int i;
    u8 reg_value_8;
    char *kernel_buf;
    struct proc_dir_entry *pde;
    struct spi_device *spi_dev;

    master_ext = (struct via_spi_master_extension *)data;

    if ((count != sizeof("master")) && (count != sizeof("slave"))) {
		via_spi_printk(KERN_INFO "store_ctler_mode invalid param\n");
		return -EINVAL;
    }

    kernel_buf = kmalloc(count, GFP_KERNEL);
    if (kernel_buf == NULL)
		return -ENOMEM;

	if (copy_from_user(kernel_buf, buf, count)) {
		kfree(kernel_buf);
		return -EFAULT;
	}

    if (memcmp(buf, "master", count - 1) == 0) {
		via_spi_printk(KERN_INFO "via_spi: set ctler mode to master\n");
		target_mode = VIA_SPI_CONTROLLER_MODE_MASTER;
    } else if (memcmp(buf, "slave", count - 1) == 0) {
		via_spi_printk(KERN_INFO "via_spi: set ctler mode to slave\n");
		target_mode = VIA_SPI_CONTROLLER_MODE_SLAVE;
    } else {
		via_spi_printk(KERN_INFO "via_spi: set unknown ctler mode\n");
		kfree(kernel_buf);
		return -EINVAL;
    }

    if (!(master_ext->support_controller_mode & target_mode)) {
		via_spi_printk(KERN_ALERT "via_spi: this spi master doesn't " \
				"support %s controller mode\n", buf);
		kfree(kernel_buf);
		return -EINVAL;
    }

    if (master_ext->current_controller_mode == target_mode) {
		via_spi_printk(KERN_INFO "via_spi: mode %s, no need change\n",
			buf);
		kfree(kernel_buf);
		return count;
    }
    if (master_ext->current_controller_mode ==
			VIA_SPI_CONTROLLER_MODE_MASTER) {
		for (i = 0; i < MAX_DEVICE_NUMBER; i++) {
			if (master_ext->dev[i] != NULL) {
				/* todo, unregister this device */
			}
		}

		pde = create_proc_entry(spi_bus_data_len.name,
			spi_bus_data_len.mode,
			master_ext->master_proc);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 30)
		pde->owner      = THIS_MODULE;
#endif
		pde->data       = master_ext;
		pde->read_proc  = spi_bus_data_len.show;
		pde->write_proc = spi_bus_data_len.store;

		master_ext->bus_data_len_proc = pde;

		pde = create_proc_entry(spi_bus_data.name,
			spi_bus_data.mode,
			master_ext->master_proc);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 30)
		pde->owner      = THIS_MODULE;
#endif
		pde->data       = master_ext;
		pde->read_proc  = spi_bus_data.show;
		pde->write_proc = spi_bus_data.store;

		master_ext->bus_data_proc = pde;

		/* create a fake spi device for uniform processing */
		master_ext->dev[0] =
			kzalloc(sizeof(struct via_spi_device), GFP_KERNEL);
		memcpy(master_ext->dev[0], &m25p80_spi_dev,
				sizeof(struct via_spi_device));
		master_ext->dev[0]->spi_dev =
			kzalloc(sizeof(struct spi_device), GFP_KERNEL);
		spi_dev = master_ext->dev[0]->spi_dev;
		spi_dev->bits_per_word = 8;
		spi_dev->chip_select = 0;
		spi_dev->controller_data = master_ext;
		spi_dev->master = master_ext->master;
		spi_dev->max_speed_hz = m25p80_spi_dev.max_speed_hz;
		spi_dev->mode = 0x00;
		master_ext->fake_dev_chip_sel = spi_dev->chip_select;

		/* set the master's work mode */
		reg_value_8 = ioread8(master_ext->regs + 0x01);
		reg_value_8 |= 0x40;
		iowrite8(reg_value_8, master_ext->regs + 0x01);

		master_ext->current_controller_mode = target_mode;
		kfree(kernel_buf);

		return count;
    }

    if (master_ext->current_controller_mode ==
			VIA_SPI_CONTROLLER_MODE_SLAVE) {
		remove_proc_entry(spi_bus_data.name, master_ext->master_proc);
		remove_proc_entry(spi_bus_data_len.name,
			master_ext->master_proc);
		master_ext->bus_data_len_proc = NULL;
		master_ext->bus_data_proc = NULL;
		kfree(master_ext->dev[0]->spi_dev);
		kfree(master_ext->dev[0]);
		master_ext->dev[0] = NULL;
		master_ext->fake_dev_chip_sel = -1;
		master_ext->slave_mode_data_len = 0;

		reg_value_8 = ioread8(master_ext->regs + 0x01);
		reg_value_8 &= 0xBF;
		iowrite8(reg_value_8, master_ext->regs + 0x01);
		master_ext->current_controller_mode = target_mode;
		kfree(kernel_buf);
		return count;
	}

    kfree(kernel_buf);

    return count;
}


/***********************************************************
*
* Function: show_device_mode
* Precondition: user cat the information of the dev_mode sys file
* Input:  @buffer: store the returned data
*         @start: start offset of the buffer
*         @offset: the offset in the proc file
*         @length: the data length of the buffer
*         @eof: whether reaches to the end of the proc file.
*         @data: point to the via_spi_master_extension structure
* Output:the returned char number.
* Purpose: introduce the supported device mode to user
* Reference: none
*
***********************************************************/

static int show_device_mode(char *buffer, char **start,
		off_t offset, int length, int *eof, void *data)
{
    struct via_spi_master_extension *master_ext;
    int data_len = 0;

	via_spi_printk(KERN_INFO "show_device_mode, offset %x, length %x\n",
			(int)offset, length);

    master_ext = (struct via_spi_master_extension *)data;
	if (master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_DEVICE) {
		if (master_ext->support_device_mode & VIA_SPI_DEVICE_MODE_FLASH)
			data_len = snprintf(buffer, 16, "[device] flash\n");
		else
			data_len = snprintf(buffer, 10, "[device]\n");

    } else if (master_ext->current_device_mode ==
			VIA_SPI_DEVICE_MODE_FLASH) {
		if (master_ext->support_device_mode &
				VIA_SPI_DEVICE_MODE_DEVICE)
			data_len = snprintf(buffer, 16, "device [flash]\n");
		else
			data_len = snprintf(buffer, 9, "[flash]\n");
    }
    data_len -= offset;
    if (data_len < 0)
		data_len = 0;

    *eof = (data_len <= length) ? 1 : 0;
    *start = buffer + offset;

	return data_len;
}

/***********************************************************
*
* Function: store_device_mode
* Precondition: user echo "device" or "flash" to the dev_mode proc file
* Input:  @file: the handle of the proc file
*         @buf: user buffer storing the echo infrormation
*         @count: the data length
*         @data: point to the via_spi_master_extension structure
* Output:the char number writen successfully.
* Purpose: set the device mode of the master
* Reference: none
*
***********************************************************/

static int store_device_mode(struct file *file,
		const char __user *buf, unsigned long count, void *data)
{
    u8 target_mode;
    struct via_spi_master_extension *master_ext;
    char *kernel_buf;

    master_ext = (struct via_spi_master_extension *)data;

    if ((count != sizeof("device")) && (count != sizeof("flash"))) {
		via_spi_printk(KERN_INFO "store_dev_mode invalid param\n");
		return -EINVAL;
    }


    kernel_buf = kmalloc(count, GFP_KERNEL);
    if (kernel_buf == NULL)
		return -ENOMEM;

	if (copy_from_user(kernel_buf, buf, count)) {
		kfree(kernel_buf);
		return -EFAULT;
	}

    if (memcmp(buf, "device", count - 1) == 0) {
		via_spi_printk(KERN_INFO "via_spi: set dev mode to device\n");
		target_mode = VIA_SPI_DEVICE_MODE_DEVICE;
    } else if (memcmp(buf, "flash", count - 1) == 0) {
		via_spi_printk(KERN_INFO "via_spi: set dev mode to flash\n");
		target_mode = VIA_SPI_DEVICE_MODE_FLASH;
    } else {
		via_spi_printk(KERN_INFO "via_spi: set unknown dev mode\n");
		kfree(kernel_buf);
		return -EINVAL;
    }

    if (!(master_ext->support_device_mode & target_mode)) {
		via_spi_printk(KERN_INFO "via_spi:not support %s mode\n",
			buf);
		kfree(kernel_buf);
		return -EINVAL;
    }

    if (master_ext->current_device_mode == target_mode) {
		via_spi_printk(KERN_INFO "via_spi: mode %s no need to change\n",
			buf);
		kfree(kernel_buf);
		return count;
    }

	/* change the device mode, need to reinit the max transfer length and
	 * supported transfer mode. but now, our controller doesn't support
	 * change the device mode online, you need change the device mode by
	 * hardware jumper, so just do nothing here and the following code is
	 * just for reference
	 */
	kfree(kernel_buf);
	return count;

    master_ext->current_device_mode = target_mode;

	/* adjust the device mode based on the register value */
    if (master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_FLASH) {
		master_ext->current_transfer_mode = VIA_SPI_TRANSFER_MODE_PIO;
		master_ext->support_transfer_mode = VIA_SPI_TRANSFER_MODE_PIO;
    } else if (master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_FLASH) {
		switch (via_spi_chip_info->busctl_pci_dev_id) {
		case 0x83531106:
		case 0x34021106:
			master_ext->current_transfer_mode =
				VIA_SPI_TRANSFER_MODE_PIO;
			master_ext->support_transfer_mode =
				VIA_SPI_TRANSFER_MODE_PIO;
			break;
		case 0x84091106:
		case 0x84101106:
		case 0x345B1106:
			master_ext->current_transfer_mode =
				VIA_SPI_TRANSFER_MODE_DMA;
			master_ext->support_transfer_mode =
				(VIA_SPI_TRANSFER_MODE_DMA |
				 VIA_SPI_TRANSFER_MODE_PIO);
			break;
		default:
			break;
		}
	}
    if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_DMA) {
		master_ext->trans_type[0].max_rx_len = PAGE_SIZE;
		master_ext->trans_type[1].max_tx_len = PAGE_SIZE;
		master_ext->trans_type[2].max_rx_len = 512;
		master_ext->trans_type[2].max_tx_len = 16;
		master_ext->trans_type[2].max_rx_tx_len = 512 + 16;
		master_ext->trans_type[3].max_tx_len = 16;
    } else {
		if (master_ext->current_device_mode ==
				VIA_SPI_DEVICE_MODE_DEVICE) {
			master_ext->trans_type[0].max_rx_len = 16;
			master_ext->trans_type[1].max_tx_len = 16;
			master_ext->trans_type[2].max_rx_len = 16;
			master_ext->trans_type[2].max_tx_len = 16;
			master_ext->trans_type[2].max_rx_tx_len = 16;
			master_ext->trans_type[3].max_tx_len = 16;
		} else {
			master_ext->trans_type[0].max_rx_len = 16;
			master_ext->trans_type[1].max_tx_len = 20;
			master_ext->trans_type[2].max_rx_len = 16;
			master_ext->trans_type[2].max_tx_len = 5;
			master_ext->trans_type[2].max_rx_tx_len = 21;
			master_ext->trans_type[3].max_tx_len = 20;
		}
    }

	kfree(kernel_buf);

	return count;
}

struct via_proc_file via_proc_file_tbl[] = {
    {"trans_mode", 0644, show_transfer_mode, store_transfer_mode},
    {"ctler_mode", 0644, show_controller_mode, store_controller_mode},
    {"dev_mode", 0644, show_device_mode, store_device_mode}
};

/***********************************************************
*
* Function: via_spi_calculate_speed
* Precondition: the target is not zero
* Input:  @orig: the original clock speed
*         @target: the target clock speed
*         @div: save the div
* Output: None.
* Purpose: cacluate the div which can meet orig <= target * (2^div)
* Reference: none
*
***********************************************************/

void via_spi_calculate_speed(u32 orig, u32 target, u8 *div)
{
    u8 i = 0;
    while (target * (1 << i) < orig)
		i++;
    *div = i;
}

/***********************************************************
*
* Function: preload_data_for_pio_write
* Precondition: none
* Input:
*     @master_ext:   the target master's extension
*     @spi_trans: the transfer to be executed
* Output: none
* Purpose: write the data to the data register when transfer data to device.
* Reference: none
*
***********************************************************/

void preload_data_for_pio_write(struct via_spi_master_extension *master_ext,
    struct via_spi_transfer *spi_trans)
{
    int i;
    u32 total_len;
	u8 value_byte;

    if (master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_DEVICE) {
		switch (spi_trans->transfer_type) {
		case VIA_SPI_TRANSFER_TYPE1:
		case VIA_SPI_TRANSFER_TYPE2:
			for (i = 0; i < spi_trans->tx_len; i++)
				iowrite8(spi_trans->tx_buf[i],
					master_ext->regs + 0x08 + i);
			break;
		case VIA_SPI_TRANSFER_TYPE3:
			for (i = 0; i < spi_trans->rx_len; i++)
				iowrite8(spi_trans->rx_buf[i],
					master_ext->regs + 0x08 + i);
			for (i = 0; i < spi_trans->tx_len; i++)
				iowrite8(spi_trans->tx_buf[i],
					master_ext->regs + 0x08 +
					spi_trans->rx_len + i);

			via_spi_printk(KERN_INFO "data to be loaded is :\n");
			for (i = 0; i < spi_trans->rx_len; i++)
				via_spi_printk(KERN_INFO "%x,",
						spi_trans->rx_buf[i]);
			for (i = 0; i < spi_trans->tx_len; i++)
				via_spi_printk(KERN_INFO "%x,",
						spi_trans->tx_buf[i]);
			via_spi_printk(KERN_INFO "\n");
			break;
		default:
			break;
		}
		return;
	}

	switch (spi_trans->transfer_type) {
	case VIA_SPI_TRANSFER_TYPE1:
	case VIA_SPI_TRANSFER_TYPE2:
		iowrite8(spi_trans->tx_buf[0], master_ext->regs + 0x58);
		value_byte = ioread8(master_ext->regs + 0x02);
        value_byte &= 0x0F;
		iowrite8(value_byte, master_ext->regs + 0x02);

		via_spi_printk(KERN_INFO "this transfer's command data is: ");
		for (i = 0; i < spi_trans->tx_len; i++)
			via_spi_printk(KERN_INFO "%x ", spi_trans->tx_buf[i]);
		via_spi_printk(KERN_INFO "\n");

		if (spi_trans->tx_len < 4) {
			for (i = 1; i < spi_trans->tx_len; i++) {
				iowrite8(spi_trans->tx_buf[i],
					master_ext->regs + 0x08 + i - 1);
			}
			break;
		}

		for (i = 1; ((i < spi_trans->tx_len) && (i < 4)); i++)
			iowrite8(spi_trans->tx_buf[i],
				master_ext->regs + 0x07 - i);
		if (i < 4)
			break;

		for (i = 4; i < spi_trans->tx_len; i++)
			iowrite8(spi_trans->tx_buf[i],
				master_ext->regs + 0x08 + i - 4);
		break;
	case VIA_SPI_TRANSFER_TYPE3:
		memset(master_ext->via_dma_w_buffer, 0,
			master_ext->via_dma_w_buf_len);
		for (i = 0; i < spi_trans->rx_len; i++)
			master_ext->via_dma_w_buffer[i] = spi_trans->rx_buf[i];
		for (i = spi_trans->rx_len;
				i < (spi_trans->tx_len+spi_trans->rx_len); i++)
			master_ext->via_dma_w_buffer[i] =
				spi_trans->tx_buf[i - spi_trans->rx_len];

		iowrite8(master_ext->via_dma_w_buffer[0],
			master_ext->regs + 0x58);

		value_byte = ioread8(master_ext->regs + 0x02);
        value_byte &= 0x0F;
		iowrite8(value_byte, master_ext->regs + 0x02);

		total_len = spi_trans->tx_len + spi_trans->rx_len;
		if (total_len == 1)
			break;

		if (total_len < 4) {
			for (i = 1; i < total_len; i++)
				iowrite8(master_ext->via_dma_w_buffer[i],
					master_ext->regs + 0x08 + i - 1);
			break;
		}

		for (i = 1; ((i < total_len) && (i < 4)); i++)
			iowrite8(master_ext->via_dma_w_buffer[i],
					master_ext->regs+0x07-i);
		if (i < 4)
			break;

		for (i = 4; i < total_len; i++)
			iowrite8(master_ext->via_dma_w_buffer[i],
					master_ext->regs + 0x08 + i - 4);
		break;
	default:
		break;
	}

}

/***********************************************************
*
* Function: wait_cmd_complete
* Precondition: none
* Input:
*     @regs:   the spi status register address
*     @bit_offset: the offset of the cycle done bit in the status register
* Output: int, whether this command has been completed without timeout
* Purpose: wait the command to be completed in polling mode.
* Reference: none
*
***********************************************************/

int wait_cmd_complete(void __iomem *regs, u8 bit_offset)
{
    int i = 0;
    u8 reg_value_8;
    u8 cycle_done_value;

    cycle_done_value = bit_offset;

    for (i = 0; i < TRANSFER_TIME_OUT_S; i++) {
		reg_value_8 = ioread8(regs);
		if (reg_value_8 & cycle_done_value) {
			via_spi_printk(KERN_INFO "cmd finish on %dth wait\n",
					i);
			break;
		}
		msleep(5);
    }
    reg_value_8 = ioread8(regs);
    if ((i == TRANSFER_TIME_OUT_S) && ((reg_value_8 & cycle_done_value) == 0)) {
		via_spi_printk(KERN_INFO "via_spi_dispatch:trans time out\n");
		return -ETIMEDOUT;
    } else
		return 0;
}



/***********************************************************
*
* Function: get_flash_mode_cmd_type
* Precondition: none
* Input:  @spi_trans: the transfer to be executed
* Output: u8, the command type of this flash command
* Purpose: get the command type of this spi transfer.
* Reference: none
*
***********************************************************/

u8 get_flash_mode_cmd_type(struct via_spi_transfer *spi_trans)
{
    u8 cmd_type = 0;

    switch (spi_trans->transfer_type) {
    case VIA_SPI_TRANSFER_TYPE0:
		cmd_type = 0;
		break;
    case VIA_SPI_TRANSFER_TYPE1:
		if (spi_trans->tx_len >= 4)
			cmd_type = 0x03;
		else
			cmd_type = 0x01;
		break;
    case VIA_SPI_TRANSFER_TYPE2:
		if (spi_trans->tx_len > 1)
			cmd_type = 0x02;
		else
			cmd_type = 0x00;
		break;
    case VIA_SPI_TRANSFER_TYPE3:
		if (spi_trans->tx_len + spi_trans->rx_len >= 4)
			cmd_type = 0x03;
		else
			cmd_type = 0x01;
    break;
    }

    return cmd_type;
}


/***********************************************************
*
* Function: vt3353_master0_init
* Precondition: the platform is vt3353 and has a spi controller
* Input:  @chip_info: infromation of the spi controller
*         @master_ext: information of the spi master
* Output: int, whether the initialization of the spi master is successful.
* Purpose:
*     init the given spi master based on the hardware feature
* Reference: none
*
***********************************************************/

int vt3353_master0_init(struct via_spi_controller *chip_info,
		struct via_spi_master_extension *master_ext)
{
    u32 spi_register_bar;
    u32 status = 0;
    u8 config_value;
    u16 spis = 0;
    u8 clk_div = 0;
    u8 mis_ctrl = 0;
	u32 config_value_32;
	unsigned char value_byte;

    if (chip_info->busctl_pci_dev_id == 0x83531106) {
        chip_info->via_busctl_pci_dev =
		pci_get_device(PCI_VENDOR_ID_VIA,
			PCI_DEVICE_ID_VIA_SB_VT3353, NULL);

	pci_read_config_byte(chip_info->via_busctl_pci_dev,
		0x47, &config_value);
    	chip_info->irq = config_value >> 4;
    } else if (chip_info->busctl_pci_dev_id == 0x34021106) {
        chip_info->via_busctl_pci_dev =
		pci_get_device(PCI_VENDOR_ID_VIA,
			PCI_DEVICE_ID_VIA_SB_VT3402, NULL);

	pci_read_config_byte(chip_info->via_busctl_pci_dev,
		0xBF, &config_value);
    	chip_info->irq = ((config_value >> 1) & 0x0F);
    }

	pci_read_config_dword(chip_info->via_busctl_pci_dev, 0xBC,
			&spi_register_bar);
    spi_register_bar = (spi_register_bar << 8) & 0x0FFFFFF00;
	via_spi_printk(KERN_INFO "the base address %x\n", spi_register_bar);

    pci_dev_put(chip_info->via_busctl_pci_dev);

	master_ext->ioarea = request_mem_region(spi_register_bar, 256,
							"via_spi_master0");
    if (master_ext->ioarea == NULL) {
		status = -ENXIO;
		via_spi_printk(KERN_INFO "fail request io mem for 353\n");
		return status;
    }

    master_ext->regs = ioremap(spi_register_bar, 256);
    if (master_ext->regs == NULL) {
		status = -ENOMEM;
		via_spi_printk(KERN_INFO "fail to map io region for 353\n");
		goto free_mem_region;
    }
    
	/* adjust the device mode based on the register value */
    if (master_ext->support_device_mode & VIA_SPI_DEVICE_MODE_FLASH) {
		config_value = ioread8(master_ext->regs + 0x00);
		if (config_value & 0x10) {
			master_ext->current_device_mode =
				VIA_SPI_DEVICE_MODE_FLASH;
			master_ext->current_transfer_mode =
				VIA_SPI_TRANSFER_MODE_PIO;
		}
    }

    /* init the max transfer length in one transfer */
    if (master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_DEVICE) {
		master_ext->trans_type[0].max_rx_len = 16;
		master_ext->trans_type[1].max_tx_len = 16;
		master_ext->trans_type[2].max_rx_len = 16;
		master_ext->trans_type[2].max_tx_len = 16;
		master_ext->trans_type[2].max_rx_tx_len = 16;
		master_ext->trans_type[3].max_tx_len = 16;
    } else {
		master_ext->trans_type[0].max_rx_len = 16;
		master_ext->trans_type[1].max_tx_len = 20;
		master_ext->trans_type[2].max_rx_len = 16;
		master_ext->trans_type[2].max_tx_len = 5;
		master_ext->trans_type[2].max_rx_tx_len = 21;
		master_ext->trans_type[3].max_tx_len = 20;
    }

	/* clear SPI cycle go and block access status(write 1 ) */
    spis = ioread16(master_ext->regs + 0x00);
    via_spi_printk(KERN_INFO "spi status reg =0x%x\n", spis);
    spis |= 0x000C;
    iowrite16(spis, master_ext->regs + 0x00);

    /* set the clock speed */
    clk_div = ioread8(master_ext->regs + 0x6C);
    via_spi_printk(KERN_INFO "clock divider reg =0x%x\n", clk_div);
    clk_div = 0x01;
    iowrite8(clk_div, master_ext->regs + 0x6C);
    via_spi_printk(KERN_INFO "clk_div =0x%x\n",
			ioread8(master_ext->regs + 0x6C));

    /*disable interrupt becase this controller doesn't support interrupt mode*/
    mis_ctrl = ioread8(master_ext->regs + 0x6D);
    via_spi_printk(KERN_INFO "spi miscellaneous control reg =0x%x\n", mis_ctrl);
    mis_ctrl = 0x00;
    iowrite8(mis_ctrl, master_ext->regs + 0x6D);

	config_value_32 = ioread32(master_ext->regs + 0x50);
    via_spi_printk(KERN_INFO "the BIOS base address register is %x\n", config_value_32);
	/* disable system flash protect */
	iowrite32(0, master_ext->regs + 0x50);	

	config_value_32 = ioread32(master_ext->regs + 0x60);
    via_spi_printk(KERN_INFO "the BIOS protect register is %x\n", config_value_32);
	value_byte = ioread8(master_ext->regs + 0x63);
	value_byte &= 0x7F;
	iowrite8(value_byte, master_ext->regs + 0x63);
    
	config_value_32 = ioread32(master_ext->regs + 0x64);
    via_spi_printk(KERN_INFO "the BIOS protect register is %x\n", config_value_32);
	value_byte = ioread8(master_ext->regs + 0x67);
	value_byte &= 0x7F;
	iowrite8(value_byte, master_ext->regs + 0x67);

	config_value_32 = ioread32(master_ext->regs + 0x68);
    via_spi_printk(KERN_INFO "the BIOS protect register is %x\n", config_value_32);
	value_byte = ioread8(master_ext->regs + 0x6B);
	value_byte &= 0x7F;
	iowrite8(value_byte, master_ext->regs + 0x6B);
    return status;

free_mem_region:
    release_mem_region(master_ext->ioarea->start, 256);
    return status;
}



/***********************************************************
*
* Function: vt3353_master0_resume
* Precondition: the platform is vt3353 and has a spi controller
* Input:  @chip_info: infromation of the spi controller
*         @master_ext: information of the spi master
* Output: int, whether the initialization of the spi master is successful.
* Purpose:
*     resume the given spi master based on the hardware feature
* Reference: none
*
***********************************************************/

int vt3353_master0_resume(struct via_spi_controller *chip_info,
		struct via_spi_master_extension *master_ext)
{
    u32 status = 0;    
    u16 spis = 0;
    u8 clk_div = 0;
    u8 mis_ctrl = 0;
	u32 config_value_32;
	u8 value_byte;

    

	/* clear SPI cycle go and block access status(write 1 ) */
    spis = ioread16(master_ext->regs + 0x00);
    via_spi_printk(KERN_INFO "spi status reg =0x%x\n", spis);
    spis |= 0x000C;
    iowrite16(spis, master_ext->regs + 0x00);

    /* set the clock speed */
    clk_div = ioread8(master_ext->regs + 0x6C);
    via_spi_printk(KERN_INFO "clock divider reg =0x%x\n", clk_div);
    clk_div = 0x01;
    iowrite8(clk_div, master_ext->regs + 0x6C);
    via_spi_printk(KERN_INFO "clk_div =0x%x\n",
			ioread8(master_ext->regs + 0x6C));

    /*disable interrupt becase this controller doesn't support interrupt mode*/
    mis_ctrl = ioread8(master_ext->regs + 0x6D);
    via_spi_printk(KERN_INFO "spi miscellaneous control reg =0x%x\n", mis_ctrl);
    mis_ctrl = 0x00;
    iowrite8(mis_ctrl, master_ext->regs + 0x6D);

	config_value_32 = ioread32(master_ext->regs + 0x50);
    via_spi_printk(KERN_INFO "the BIOS base address register is %x\n", config_value_32);
	/* disable system flash protect */
	iowrite32(0, master_ext->regs + 0x50);	

	config_value_32 = ioread32(master_ext->regs + 0x60);
    via_spi_printk(KERN_INFO "the BIOS protect register is %x\n", config_value_32);
	value_byte = ioread8(master_ext->regs + 0x63);
	value_byte &= 0x7F;
	iowrite8(value_byte, master_ext->regs + 0x63);
    
	config_value_32 = ioread32(master_ext->regs + 0x64);
    via_spi_printk(KERN_INFO "the BIOS protect register is %x\n", config_value_32);
	value_byte = ioread8(master_ext->regs + 0x67);
	value_byte &= 0x7F;
	iowrite8(value_byte, master_ext->regs + 0x67);

	config_value_32 = ioread32(master_ext->regs + 0x68);
    via_spi_printk(KERN_INFO "the BIOS protect register is %x\n", config_value_32);
	value_byte = ioread8(master_ext->regs + 0x6B);
	value_byte &= 0x7F;
	iowrite8(value_byte, master_ext->regs + 0x6B);
    return status;

}


/***********************************************************
*
* Function: vt3353_master0_exit
* Precondition: the platform is vt3353 and has a spi controller
* Input:  @chip_info: infromation of the spi controller
*         @master_ext: information of the spi master
* Output: int, whether the exit of the spi master is successful.
* Purpose:
*     destroy the given spi master based on the hardware feature
* Reference: none
*
***********************************************************/

int vt3353_master0_exit(struct via_spi_controller *chip_info,
	struct via_spi_master_extension *master_ext)
{
    u32 status = 0;

    if (master_ext->ioarea != NULL)
		release_mem_region(master_ext->ioarea->start, 256);

    if (master_ext->regs != NULL)
		iounmap(master_ext->regs);

	return status;
}

/***********************************************************
*
* Function: vt3353_master0_dispatch
* Precondition: the platform is vt3353 and spi bus0 is implemented
* Input:
*     @spi_dev:   the target device of this transfer
*     @spi_trans: the transfer to be executed
* Output: int, whether the transfer is dispatched to the spi master successfully
* Purpose:
*     program the spi controller to execute the requested transfer.
*     Note:
*         1, VT3353 SPI controller doesn't support interrupt, need wait for the
*            completion of the transfer
*         2, must program the controller based on the transfer type of this
*            transfer
* Reference: none
*
***********************************************************/

int vt3353_master0_dispatch(struct spi_device *spi_dev,
		struct via_spi_transfer *spi_trans)
{
    struct spi_master *master;
    struct via_spi_master_extension *master_ext;
    u8 reg_value_8;
    u16 reg_value_16;
    char data_len;
    int i;
    u8 cmd_type;
    u8 chip_select;

    via_spi_printk(KERN_INFO "enter vt3353_master0_dispatch routine\n");

    master = spi_dev->master;
    master_ext = spi_master_get_devdata(master);

    chip_select = spi_dev->chip_select;
    chip_select &= 0x01;  /* only have two port in this master */

    /* clear the cycle done status and blocked access status bit */
    reg_value_16 = 0x0C;
    iowrite16(reg_value_16, master_ext->regs + 0x00);

    /* enable the dynamic clock, disable command post write and disable the
	 * interrupt because this controller doesn't support interrupt when the
	 * transfer is completed
	 */
    reg_value_8 = 0x08;
    iowrite8(reg_value_8, master_ext->regs + 0x6D);

    /* disable the atomic cycle sequence */
    reg_value_8 = 0x00;
    iowrite8(reg_value_8, master_ext->regs + 0x02);

    /* set up the clock speed for this transfer */
    if (spi_trans->speed_hz != 0) {
		via_spi_printk(KERN_INFO "need reset the clock speed\n");
		via_spi_calculate_speed(33*1000*1000, spi_trans->speed_hz,
			&reg_value_8);
		iowrite8(reg_value_8, master_ext->regs + 0x6C);
    }

    /* set the data transaction bit */
    reg_value_8 = ioread8(master_ext->regs + 0x03);
    reg_value_8 &= 0x1F;
    /*
    if ((master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_DEVICE) ||
		(spi_trans->transfer_type != VIA_SPI_TRANSFER_TYPE1) ||
		(spi_trans->tx_len > 4) || (chip_select > 0)) {
		reg_value_8 |= 0x40;
    }
    */
    iowrite8(reg_value_8, master_ext->regs + 0x03);

	via_spi_printk(KERN_ALERT "the transfer type is %d\n",
		spi_trans->transfer_type);
    /* set the data length register */
	data_len = 0;
    if ((master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_DEVICE) ||
		(chip_select > 0)) {
		data_len = spi_trans->tx_len + spi_trans->rx_len;
		//data_len -= 1;
    } else if (master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_FLASH) {
		if (spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE0)
			data_len = spi_trans->rx_len;
		else if (spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE1) {
			if (spi_trans->tx_len > 4)
				data_len = spi_trans->tx_len - 4;
			else if (spi_trans->tx_len < 4)
				data_len = spi_trans->tx_len - 1;
			else
				data_len = 0;
		}
		else if (spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE2)
			data_len = spi_trans->rx_len;
		else if (spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE3) {
			if ((spi_trans->tx_len + spi_trans->rx_len) > 4)
				data_len = spi_trans->tx_len + spi_trans->rx_len - 4;
			else if ((spi_trans->tx_len + spi_trans->rx_len) < 4)
				data_len = spi_trans->tx_len + spi_trans->rx_len - 1;
			else
				data_len = 0;
		}
		/*
		if (data_len < 0)
			data_len = 0;
		*/

    }
    data_len &= 0x1F;
    reg_value_8 = ioread8(master_ext->regs + 0x03);
    reg_value_8 &= 0xB0;
    if (data_len != 0) {
        reg_value_8 |= (data_len - 1);
	 reg_value_8 |= 0x40;
    }
	via_spi_printk(KERN_INFO "data byte count register is %x\n", reg_value_8);
    iowrite8(reg_value_8, master_ext->regs + 0x03);

    preload_data_for_pio_write(master_ext, spi_trans);

	reg_value_8 = ioread8(master_ext->regs + 0x03);
	reg_value_8 &= 0xDF;
	if ((master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_FLASH) &&
	    (spi_trans->is_fast_read_transfer > 0))
		reg_value_8 |= 0x20;
	iowrite8(reg_value_8, master_ext->regs + 0x03);

    cmd_type = 0;
    cmd_type = get_flash_mode_cmd_type(spi_trans);
	via_spi_printk(KERN_INFO "this transfer's cmd type is %x\n", cmd_type);
    reg_value_8 = ioread8(master_ext->regs + 0x56);
    reg_value_8 &= 0xFC;
    reg_value_8 |= cmd_type;
    iowrite8(reg_value_8, master_ext->regs + 0x56);

    /* now, walk to the last step, we need select the spi port */
    reg_value_8 = ioread8(master_ext->regs + 0x03);
    reg_value_8 &= 0xEF;
    reg_value_8 |= (chip_select << 4);
    iowrite8(reg_value_8, master_ext->regs + 0x03);

    /* now, we can set the cycle go bit to start this transfer */
    reg_value_8 = ioread8(master_ext->regs + 0x02);
    reg_value_8 |= 0x02;
    iowrite8(reg_value_8, master_ext->regs + 0x02);

    /* because this controller doesn't support interrupt, so need wait the
	 * completion of this transfer. we can polling the status of cycle done
	 * status to judge whether this transfer is completed
	 */
    for (i = 0; i < TRANSFER_TIME_OUT_S; i++) {
		reg_value_8 = ioread8(master_ext->regs + 0x00);
		if (reg_value_8 & 0x04)
			break;
		msleep(5);
    }
    reg_value_8 = ioread8(master_ext->regs + 0x00);
    if ((i == TRANSFER_TIME_OUT_S) && ((reg_value_8 & 0x04) == 0)) {
		via_spi_printk(KERN_INFO "via_spi_dispatch: time out\n");
		return -ETIMEDOUT;
    }

    /* clear the cycle done status bit to clear the interrupt */
    reg_value_8 = ioread8(master_ext->regs + 0x00);
    reg_value_8 |= 0x04;
    iowrite8(reg_value_8, master_ext->regs + 0x00);

    /* if this transfer reads data from device, need copy the data from the
	 * data register to the buffer
	 */
    if ((spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE0) ||
		(spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE2)) {

		u8 base_addr = 0;
		if (master_ext->current_device_mode ==
				VIA_SPI_DEVICE_MODE_FLASH)
			base_addr = 0x00;
		else
			base_addr = spi_trans->tx_len;
		via_spi_printk(KERN_INFO "after read, the data is :");
		for (i = 0; i < spi_trans->rx_len; i++) {
			spi_trans->rx_buf[i] =
				ioread8(master_ext->regs+0x08+i+base_addr);
			via_spi_printk(KERN_INFO "%x, ", spi_trans->rx_buf[i]);
		}
		via_spi_printk(KERN_INFO "\n");
    }

    return 0;

}


/***********************************************************
*
* Function: vt3409_master0_init
* Precondition: the platform is vt3409 and spi bus0 is implemented
* Input:  @chip_info: infromation of the spi controller
*         @master_ext: information of the spi master
* Output: int, whether the initialization of the spi master is successful.
* Purpose:
*     init the given spi master based on the hardware feature
* Reference: none
*
***********************************************************/

int vt3409_master0_init(struct via_spi_controller *chip_info,
    struct via_spi_master_extension *master_ext)
{
	u32 spi_register_bar;
	u32 spi_register_len;
	u32 status = 0;
	u8 config_value;
	u8 value_byte;
	u16 value_word;
	struct proc_dir_entry *pde;

	config_value = ioread8(chip_info->mmio_regs + 0x09);
	chip_info->irq = config_value & 0x0F;
	via_spi_printk(KERN_INFO "the VT3409 interrupt line is %d, config is %x\n",
		chip_info->irq, config_value);

	iowrite8(0x01, chip_info->mmio_regs + 0x00); /*enable the master0 */

	spi_register_bar = ioread32(chip_info->mmio_regs + 0x00);
	spi_register_len = 0x88;
	spi_register_bar &= 0xFFFFFF00;

	master_ext->ioarea = request_mem_region(
			spi_register_bar, spi_register_len, "via_spi_master0");
	if (master_ext->ioarea == NULL) {
		status = -ENXIO;
		via_spi_printk(KERN_INFO "fail request io mem for master%d\n",
			master_ext->bus_number);
		return status;
	}

	master_ext->regs = ioremap(spi_register_bar, spi_register_len);
	if (master_ext->regs == NULL) {
		status = -ENOMEM;
		via_spi_printk(KERN_INFO "fail map io reg for master%d\n",
			master_ext->bus_number);
		goto free_mem_region;
	}

	/* adjust the device mode based on the register value */
	if (master_ext->support_device_mode & VIA_SPI_DEVICE_MODE_FLASH) {
		config_value = ioread8(master_ext->regs + 0x00);
		if (config_value & 0x10) {
			// VT3456 verify : for port1, mount another flash and port1 is on device PIO mode
			//master_ext->current_device_mode =
			//	VIA_SPI_DEVICE_MODE_DEVICE;
			master_ext->current_device_mode =
				VIA_SPI_DEVICE_MODE_FLASH;
			//VT3456 verify: for port1, mount another flash and port1 in on device DMA mode
			master_ext->current_transfer_mode =
				VIA_SPI_TRANSFER_MODE_PIO;
			master_ext->support_transfer_mode =
				VIA_SPI_TRANSFER_MODE_PIO;
		} else
			via_spi_printk(KERN_INFO "cur dev mode is dev\n");
	}
	

	if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_DMA) {
		master_ext->trans_type[0].max_rx_len = PAGE_SIZE;
		master_ext->trans_type[1].max_tx_len = PAGE_SIZE;
		master_ext->trans_type[2].max_rx_len = 512;
		master_ext->trans_type[2].max_tx_len = 16;
		master_ext->trans_type[2].max_rx_tx_len = 512 + 16;
		master_ext->trans_type[3].max_tx_len = 16;
	} else {
		if (master_ext->current_device_mode ==
				VIA_SPI_DEVICE_MODE_DEVICE) {
			master_ext->trans_type[0].max_rx_len = 16;
			master_ext->trans_type[1].max_tx_len = 16;
			master_ext->trans_type[2].max_rx_len = 16;
			master_ext->trans_type[2].max_tx_len = 16;
			master_ext->trans_type[2].max_rx_tx_len = 16;
			master_ext->trans_type[3].max_tx_len = 16;
		} else {
			master_ext->trans_type[0].max_rx_len = 16;
			master_ext->trans_type[1].max_tx_len = 20;
			master_ext->trans_type[2].max_rx_len = 16;
			master_ext->trans_type[2].max_tx_len = 5;
			master_ext->trans_type[2].max_rx_tx_len = 21;
			master_ext->trans_type[3].max_tx_len = 20;
		}
	}


	/* clear SPI cycle go and block access status(write 1 ) */
	value_word = ioread16(master_ext->regs + 0x00);
	value_word |= 0x000C;
	iowrite16(value_word, master_ext->regs + 0x00);	

	/* disable interrupt, dynamic clock on, command post write when
	 * initialization procedure. also set the transfer mode
	 */
	value_byte = ioread8(master_ext->regs + 0x6D);
	if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_DMA)
		value_byte = 0x48;
	else
		value_byte = 0x08;
	iowrite8(value_byte, master_ext->regs + 0x6D);

	/* set the clock speed */
	
	if ((chip_info->busctl_pci_dev_id == 0x84101106) ||
	    (chip_info->busctl_pci_dev_id == 0x345B1106)) {
		/* select the clock to 33M*/
		value_byte = ioread8(master_ext->regs + 0x6D);
		value_byte &= 0x7F;
		iowrite8(value_byte, master_ext->regs + 0x6D);
	}
	value_byte = ioread8(master_ext->regs + 0x6C);
	value_byte = 0x02;
	iowrite8(value_byte, master_ext->regs + 0x6C);
	

	/* disable all dma buffer interrupt */
	value_byte = 0x00;
	iowrite8(value_byte, master_ext->regs + 0x70);

	/* clear all dma buffer interrupt */
	value_byte = 0xFF;
	iowrite8(value_byte, master_ext->regs + 0x73);

	/* disable system flash protect */
	iowrite32(0, master_ext->regs + 0x50);

	/* disable write protection */
	value_byte = ioread8(master_ext->regs + 0x63);
	value_byte &= 0x7F;
	iowrite8(value_byte, master_ext->regs + 0x63);
	value_byte = ioread8(master_ext->regs + 0x67);
	value_byte &= 0x7F;
	iowrite8(value_byte, master_ext->regs + 0x67);
	value_byte = ioread8(master_ext->regs + 0x6B);
	value_byte &= 0x7F;
	iowrite8(value_byte, master_ext->regs + 0x6B);

	/* following is just for spi bus1 slave function test */
	if (via_spi_chip_info->busctl_pci_dev_id == 0x84091106) {
		pde = create_proc_entry(slave_test_port.name, slave_test_port.mode,
			master_ext->master_proc);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 30)
		pde->owner      = THIS_MODULE;
#endif
		pde->data       = master_ext;
		pde->read_proc  = slave_test_port.show;
		pde->write_proc = slave_test_port.store;
		master_ext->slave_test_port_proc = pde;
		master_ext->slave_mode_data_len =  master_ext->trans_type[0].max_rx_len;
	}
	return status;
		
free_mem_region:
	release_mem_region(master_ext->ioarea->start, 0x88);
	return status;
}
/***********************************************************
*
* Function: vt3409_master0_resume
* Precondition: the platform is vt3409 and spi bus0 is implemented
* Input:  @chip_info: infromation of the spi controller
*         @master_ext: information of the spi master
* Output: int, whether the initialization of the spi master is successful.
* Purpose:
*     resume the given spi master based on the hardware feature
* Reference: none
*
***********************************************************/

int vt3409_master0_resume(struct via_spi_controller *chip_info,
    struct via_spi_master_extension *master_ext)
{
	u32 status = 0;	
	u8 value_byte;
	u16 value_word;	

	struct via_spi_device *via_spi_dev = NULL;
	struct via_spi_transfer via_trans, *vspt;
	unsigned char cmd[4];
	u8 int_backup;

	struct spi_device *spi_dev;

	iowrite8(0x01, chip_info->mmio_regs + 0x00); /*enable the master0 */

	/* clear SPI cycle go and block access status(write 1 ) */
	value_word = ioread16(master_ext->regs + 0x00);
	value_word |= 0x000C;
	iowrite16(value_word, master_ext->regs + 0x00);	

	/* disable interrupt, dynamic clock on, command post write when
	 * initialization procedure. also set the transfer mode
	 */
	value_byte = ioread8(master_ext->regs + 0x6D);
	if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_DMA)
		value_byte = 0x48;
	else
		value_byte = 0x08;
	iowrite8(value_byte, master_ext->regs + 0x6D);

	/* set the clock speed */
	
	if ((chip_info->busctl_pci_dev_id == 0x84101106) ||
	    (chip_info->busctl_pci_dev_id == 0x345B1106)) {
		/* select the clock to 33M*/
		value_byte = ioread8(master_ext->regs + 0x6D);
		value_byte &= 0x7F;
		iowrite8(value_byte, master_ext->regs + 0x6D);
	}
	value_byte = ioread8(master_ext->regs + 0x6C);
	value_byte = 0x02;
	iowrite8(value_byte, master_ext->regs + 0x6C);
	

	/* disable all dma buffer interrupt */
	value_byte = 0x00;
	iowrite8(value_byte, master_ext->regs + 0x70);

	/* clear all dma buffer interrupt */
	value_byte = 0xFF;
	iowrite8(value_byte, master_ext->regs + 0x73);

	/* disable system flash protect */
	iowrite32(0, master_ext->regs + 0x50);

	/* disable write protection */
	value_byte = ioread8(master_ext->regs + 0x63);
	value_byte &= 0x7F;
	iowrite8(value_byte, master_ext->regs + 0x63);
	value_byte = ioread8(master_ext->regs + 0x67);
	value_byte &= 0x7F;
	iowrite8(value_byte, master_ext->regs + 0x67);
	value_byte = ioread8(master_ext->regs + 0x6B);
	value_byte &= 0x7F;
	iowrite8(value_byte, master_ext->regs + 0x6B);

	via_spi_dev = master_ext->dev[0];
	if (via_spi_dev == NULL) {
		via_spi_printk(KERN_ALERT "the target via_spi_device is NULL\n");
		return -ENODEV;
	}

	spi_dev = via_spi_dev->spi_dev;
	if (spi_dev == NULL) {
		via_spi_printk(KERN_ALERT "the target spi_device is NULL\n");
		return -ENODEV;
	}

	if (via_spi_dev->bSST25vf080bType == 0) {		
		return 0;
	}    

	via_spi_printk(KERN_INFO "clear write protection for SST25VF080\n");
	int_backup = master_ext->support_int;
	master_ext->support_int = 0;

	vspt = &via_trans;
	memset(vspt, 0, sizeof(struct via_spi_transfer));
	INIT_LIST_HEAD(&vspt->via_transfer_list);
	vspt->transfer_type = VIA_SPI_TRANSFER_TYPE2;
	vspt->rx_buf = (unsigned char *)(&status);
	vspt->rx_len = 1;
	vspt->tx_buf = cmd;
	cmd[0] = 0x05; /* read status command */
	cmd[1] = cmd[2] = cmd[3] = 0;
	vspt->tx_len = 1;
	vspt->actual_rx_len = vspt->actual_tx_len = 0;
	master_ext->cmd_dispatch(spi_dev, vspt);
	via_spi_printk(KERN_INFO "the chip's status is %x\n", status);

	vspt = &via_trans;
	memset(vspt, 0, sizeof(struct via_spi_transfer));
	INIT_LIST_HEAD(&vspt->via_transfer_list);
	vspt->transfer_type = VIA_SPI_TRANSFER_TYPE1;
	vspt->rx_buf = NULL;
	vspt->rx_len = 0;
	vspt->tx_buf = cmd;
	cmd[0] = 0x06; /* write enable command */
	cmd[1] = cmd[2] = cmd[3] = 0;
	vspt->tx_len = 1;
	vspt->actual_rx_len = vspt->actual_tx_len = 0;
	master_ext->cmd_dispatch(spi_dev, vspt);
	via_spi_printk(KERN_INFO "enable write\n");
	
	vspt = &via_trans;
	memset(vspt, 0, sizeof(struct via_spi_transfer));
	INIT_LIST_HEAD(&vspt->via_transfer_list);
	vspt->transfer_type = VIA_SPI_TRANSFER_TYPE1;
	vspt->rx_buf = NULL;
	vspt->rx_len = 0;
	vspt->tx_buf = cmd;
	cmd[0] = 0x50; /* enable write status command */
	cmd[1] = cmd[2] = cmd[3] = 0;
	vspt->tx_len = 1;
	vspt->actual_rx_len = vspt->actual_tx_len = 0;
	master_ext->cmd_dispatch(spi_dev, vspt);
	via_spi_printk(KERN_INFO "enable write status register\n");

	vspt = &via_trans;
	memset(vspt, 0, sizeof(struct via_spi_transfer));
	INIT_LIST_HEAD(&vspt->via_transfer_list);
	vspt->transfer_type = VIA_SPI_TRANSFER_TYPE1;
	vspt->rx_buf = NULL;
	vspt->rx_len = 0;
	vspt->tx_buf = cmd;
	cmd[0] = 0x01; /* write status register command */
	cmd[1] = cmd[2] = cmd[3] = 0;
	vspt->tx_len = 2;
	vspt->actual_rx_len = vspt->actual_tx_len = 0;
	master_ext->cmd_dispatch(spi_dev, vspt);
	via_spi_printk(KERN_INFO "write %x to the flash's status register\n",
		cmd[1]);

	vspt = &via_trans;
	memset(vspt, 0, sizeof(struct via_spi_transfer));
	INIT_LIST_HEAD(&vspt->via_transfer_list);
	vspt->transfer_type = VIA_SPI_TRANSFER_TYPE2;
	vspt->rx_buf = (unsigned char *)(&status);
	vspt->rx_len = 1;
	vspt->tx_buf = cmd;
	cmd[0] = 0x05; /* read status command */
	cmd[1] = cmd[2] = cmd[3] = 0;
	vspt->tx_len = 1;
	vspt->actual_rx_len = vspt->actual_tx_len = 0;
	master_ext->cmd_dispatch(spi_dev, vspt);
	via_spi_printk(KERN_INFO "the chip's status is %x\n", status);
	
	master_ext->support_int = int_backup;

	return status;
}
/***********************************************************
*
* Function: vt3409_master0_exit
* Precondition: the platform is vt3409 and has a spi controller
* Input:  @chip_info: infromation of the spi controller
*           @master_ext: information of the spi master
* Output: int, whether the exit of the spi master is successful.
* Purpose:
*     destroy the given spi master based on the hardware feature
* Reference: none
*
***********************************************************/

int vt3409_master0_exit(struct via_spi_controller *chip_info,
    struct via_spi_master_extension *master_ext)
{
    u32 status = 0;

    if (master_ext->ioarea != NULL)
		release_mem_region(master_ext->ioarea->start, 0x88);

    if (master_ext->regs != NULL)
		iounmap(master_ext->regs);

	return status;
}


/***********************************************************
*
* Function: vt3409_master0_dispatch
* Precondition: the platform is vt3409 and spi bus0 is implemented
* Input:
*     @spi_dev:   the target device of this transfer
*     @spi_trans: the transfer to be executed
* Output: int, whether the transfer is dispatched to the spi master successfully
* Purpose:
*     program the spi controller to execute the requested transfer.
*     Note: must program the controller based on the transfer type of this
*           transfer
* Reference: none
*
***********************************************************/

int vt3409_master0_dispatch(struct spi_device *spi_dev,
		struct via_spi_transfer *spi_trans)
{
    struct spi_master *master;
    struct via_spi_master_extension *master_ext;
    u8 reg_value_8;
    u16 reg_value_16;
    u32 reg_value_32;
    char data_len;
    int i;
    u8 cmd_type;
    u8 chip_select;
    u32 phy_addr;
    u32 max_tx_len;
    u32 max_rx_len;
	int isr_need = 1;
	u8 wait_reg_offset = 0;
	u8 wait_reg_value = 0;
    u8 wait_status = 0;
	int is_undefined_len_trans = 0;
	u32 actual_trans_len = 0;
	u32 speed = 33 * 1000 * 1000;
	u8 bNeed48MSpeed = 0;

    via_spi_printk(KERN_INFO "enter vt3409_master0_dispatch routine\n");

    master = spi_dev->master;
    master_ext = spi_master_get_devdata(master);

	switch (spi_trans->transfer_type) {
	case VIA_SPI_TRANSFER_TYPE0:
	case VIA_SPI_TRANSFER_TYPE2:
		if (spi_trans->rx_len < 16)
			isr_need = 0;
		break;
	case VIA_SPI_TRANSFER_TYPE1:
	case VIA_SPI_TRANSFER_TYPE3:
		if ((spi_trans->tx_len + spi_trans->rx_len) < 16)
			isr_need = 0;
		break;
	}
	if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_PIO)
		isr_need = 1;
	if (master_ext->support_int == 0)
		isr_need = 0;

	via_spi_printk(KERN_INFO "this trans dispatch with int mode %d\n",
		isr_need);
    chip_select = spi_dev->chip_select;
    chip_select &= 0x03;  /* only have three port in this master */

    /* clear the cycle done status and blocked access status bit */
    reg_value_16 = 0x0C;
    iowrite16(reg_value_16, master_ext->regs + 0x00);

    /* enable the dynamic clock, disable command post write and set the
	 * interrupt enable
	 */
    reg_value_8 = ioread8(master_ext->regs + 0x6D);
    reg_value_8 &= 0xE0;
    reg_value_8 |= 0x08;
    if (isr_need)
		reg_value_8 |= 0x10;
    iowrite8(reg_value_8, master_ext->regs + 0x6D);

    /* disable the atomic cycle sequence */
    reg_value_8 = 0x00;
    iowrite8(reg_value_8, master_ext->regs + 0x02);

	if ((via_spi_chip_info->busctl_pci_dev_id == 0x84101106) ||
	    (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)) {
		if ((spi_trans->is_fast_read_transfer) &&
			(master_ext->current_transfer_mode == 
			VIA_SPI_TRANSFER_MODE_PIO))
			bNeed48MSpeed = 1;
		else
			bNeed48MSpeed = 0;
		if (bNeed48MSpeed == 1) {
			reg_value_8 = ioread8(master_ext->regs + 0x6D);
			reg_value_8 |= 0x80;
			iowrite8(reg_value_8, master_ext->regs + 0x6D);
			iowrite8(0x00, master_ext->regs + 0x6C);
			speed = 48 * 1000 * 1000;
		} else {
			reg_value_8 = ioread8(master_ext->regs + 0x6D);
			reg_value_8 &= 0x7F;
			iowrite8(reg_value_8, master_ext->regs + 0x6D);
			iowrite8(0x02, master_ext->regs + 0x6C);
			speed = 33 * 1000 * 1000;
		}
	}

    /* set up the clock speed for this transfer */
    if (spi_trans->speed_hz != 0) {
		via_spi_calculate_speed(speed, spi_trans->speed_hz,
				&reg_value_8);
		if ((reg_value_8 == 0) &&
			(master_ext->current_transfer_mode ==
			 VIA_SPI_TRANSFER_MODE_DMA))
			reg_value_8 = 1;
		iowrite8(reg_value_8, master_ext->regs + 0x6C);
    }

    /* set the current transfer mode */
    reg_value_8 = ioread8(master_ext->regs + 0x6D);
    reg_value_8 &= 0xBF;
    if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_DMA)
		reg_value_8 |= 0x40;
    iowrite8(reg_value_8, master_ext->regs + 0x6D);

    /* set the clock latency */
    if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_DMA) {
		reg_value_8 = ioread8(master_ext->regs + 0x6C);
		if (reg_value_8 == 0x01)
			reg_value_8 = 0x0B;
		else
			reg_value_8 = 0x08;
		iowrite8(reg_value_8, master_ext->regs + 0x6E);
    }
    if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_PIO) {
		reg_value_8 = ioread8(master_ext->regs + 0x6C);
		if (reg_value_8 == 0x00) {
			reg_value_8 = ioread8(master_ext->regs + 0x6E);
			reg_value_8 |= 0x0C;
		}
		else {
			reg_value_8 = ioread8(master_ext->regs + 0x6E);
			reg_value_8 &= 0xFB;
			reg_value_8 |= 0x08;
		}
		iowrite8(reg_value_8, master_ext->regs + 0x6E);
    }

    /* process the dma transfer mode firstly */
    if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_DMA) {

		u8 data_offset = 0;

		via_spi_printk(KERN_INFO "the current transfer mode is DMA\n");

		memset(master_ext->via_dma_w_buffer, 0,
			master_ext->via_dma_w_buf_len);
		memset(master_ext->via_dma_r_buffer, 0,
			master_ext->via_dma_r_buf_len);

		iowrite8(0x40, master_ext->regs + 0x03);
		iowrite8(0x00, master_ext->regs + 0x56);

		/* set the DMA read buffer address and write buffer address.
		 * for write data need copy it to a physical continous buffer
		 */
		if (spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE3) {
			memcpy(master_ext->via_dma_w_buffer,
				spi_trans->rx_buf, spi_trans->rx_len);
			data_offset = spi_trans->rx_len;
		}
		if (spi_trans->tx_len != 0) {
			memcpy(master_ext->via_dma_w_buffer + data_offset,
				spi_trans->tx_buf, spi_trans->tx_len);
		}

		iowrite8(0x00, master_ext->regs + 0x70);
		iowrite8(0xFF, master_ext->regs + 0x73);

		max_tx_len = master_ext->trans_type[2].max_tx_len;
		max_rx_len = master_ext->trans_type[2].max_rx_len;

		switch (spi_trans->transfer_type) {
		case VIA_SPI_TRANSFER_TYPE0:
			phy_addr = virt_to_phys(master_ext->via_dma_r_buffer);
			iowrite32(phy_addr, master_ext->regs + 0x78);
			if (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)
				iowrite16(max_rx_len, master_ext->regs + 0x7E);
			else
				iowrite16(spi_trans->rx_len, master_ext->regs + 0x7E);
			if (spi_trans->rx_len > max_rx_len) {
				reg_value_16 = 0x00;
				reg_value_32 = 0x00;
				iowrite8(0x80, master_ext->regs + 0x70);
				wait_reg_offset = 0x73;
				wait_reg_value = 0x80;
			} else {
				reg_value_16 = 0x8000;
				reg_value_16 |=
					(((spi_trans->rx_len - 1) << 4) &
					 0x1FF0);
				/*special for VT3456 */
				reg_value_32 = 0x80000000;
				reg_value_32 |=
					(((spi_trans->rx_len - 1) << 16) &
					 0x1FF0000);
				wait_reg_offset = 0x00;
				wait_reg_value = 0x04;
			}
			if (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)
				iowrite32(reg_value_32, master_ext->regs + 0x4C);
			else
				iowrite16(reg_value_16, master_ext->regs + 0x71);
			break;
		case VIA_SPI_TRANSFER_TYPE1:
			phy_addr = virt_to_phys(master_ext->via_dma_w_buffer);
			iowrite32(phy_addr, master_ext->regs + 0x74);
			if (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)
				iowrite16(max_tx_len, master_ext->regs + 0x7C);
			else
				iowrite16(spi_trans->tx_len, master_ext->regs + 0x7C);
			if (spi_trans->tx_len > max_tx_len) {
				reg_value_16 = 0x2000;
				reg_value_32 = 0x20000000;
				iowrite8(0x20, master_ext->regs + 0x70);
				wait_reg_offset = 0x73;
				wait_reg_value = 0x20;
				is_undefined_len_trans = 1;
				actual_trans_len = spi_trans->tx_len;
			} else {
				reg_value_16 = 0xA000;
				reg_value_16 |=
					((spi_trans->tx_len - 1) & 0x0F);
				reg_value_32 = 0xA0000000;
				reg_value_32 |=
					((spi_trans->tx_len - 1) & 0x0FF);
				wait_reg_offset = 0x00;
				wait_reg_value = 0x04;
			}
			if (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)
				iowrite32(reg_value_32, master_ext->regs + 0x4C);
			else
				iowrite16(reg_value_16, master_ext->regs + 0x71);
			break;
		case VIA_SPI_TRANSFER_TYPE2:
			phy_addr = virt_to_phys(master_ext->via_dma_r_buffer);
			iowrite32(phy_addr, master_ext->regs + 0x78);			
			if (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)
				iowrite16(max_rx_len, master_ext->regs + 0x7E);
			else
				iowrite16(spi_trans->rx_len, master_ext->regs + 0x7E);

			phy_addr = virt_to_phys(master_ext->via_dma_w_buffer);
			iowrite32(phy_addr, master_ext->regs + 0x74);			
			if (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)
				iowrite16(max_tx_len, master_ext->regs + 0x7C);
			else
				iowrite16(spi_trans->tx_len, master_ext->regs + 0x7C);

			reg_value_16 = 0xC000;
			reg_value_16 |=
				(((spi_trans->rx_len - 1) << 4) & 0x1FF0);
			reg_value_16 |= ((spi_trans->tx_len - 1) & 0x0F);

			reg_value_32 = 0xC0000000;
			reg_value_32 |=
				(((spi_trans->rx_len - 1) << 16) & 0x1FF0000);
			reg_value_32 |= ((spi_trans->tx_len - 1) & 0x0FF);
			printk(KERN_ALERT "the bus control register is 0x%x\n", reg_value_32);
			if (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)
				iowrite32(reg_value_32, master_ext->regs + 0x4C);
			else
				iowrite16(reg_value_16, master_ext->regs + 0x71);
			wait_reg_offset = 0x00;
			wait_reg_value = 0x04;
			break;
		case VIA_SPI_TRANSFER_TYPE3:
			phy_addr = virt_to_phys(master_ext->via_dma_w_buffer);
			iowrite32(phy_addr, master_ext->regs + 0x74);
			if (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)
				iowrite16(max_tx_len, master_ext->regs + 0x7C);
			else
				iowrite16(spi_trans->tx_len + spi_trans->rx_len,
					 master_ext->regs + 0x7C);
			if ((spi_trans->tx_len + spi_trans->rx_len) >
				max_tx_len) {
				reg_value_16 = 0x2000;
				reg_value_32 = 0x20000000;
				iowrite8(0x20, master_ext->regs + 0x70);
				wait_reg_offset = 0x73;
				wait_reg_value = 0x20;
				is_undefined_len_trans = 1;
				actual_trans_len =
					spi_trans->tx_len + spi_trans->rx_len;
			} else {
				reg_value_16 = 0xA000;
				reg_value_16 |= ((spi_trans->tx_len +
					spi_trans->rx_len-1)&0x0F);
				
				reg_value_32 = 0xA0000000;
				reg_value_32 |= ((spi_trans->tx_len +
					spi_trans->rx_len-1)&0x0FF);

				wait_reg_offset = 0x00;
				wait_reg_value = 0x04;
			}
			if (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)
				iowrite32(reg_value_32, master_ext->regs + 0x4C);
			else
				iowrite16(reg_value_16, master_ext->regs + 0x71);
			break;
		}

		goto start_transfer;


	}

	via_spi_printk(KERN_INFO "the current transfer mode is PIO\n");

    /* set the data transaction bit */
    reg_value_8 = ioread8(master_ext->regs + 0x03);
    reg_value_8 &= 0x1F;
    /*
    if ((master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_DEVICE) ||
		(spi_trans->transfer_type != VIA_SPI_TRANSFER_TYPE1) ||
		(spi_trans->tx_len > 4) || (chip_select > 0)) {
		reg_value_8 |= 0x40;
    }
    */
    iowrite8(reg_value_8, master_ext->regs + 0x03);

    /* set the data length register */
	data_len = 0;
    if ((master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_DEVICE) ||
		(chip_select > 0)) {
		data_len = spi_trans->tx_len + spi_trans->rx_len;
		//data_len -= 1;
    } else if (master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_FLASH) {
		if (spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE0)
			data_len = spi_trans->rx_len;
		else if (spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE1) {
			if (spi_trans->tx_len > 4)
				data_len = spi_trans->tx_len - 4;
			else if (spi_trans->tx_len < 4)
				data_len = spi_trans->tx_len - 1;
			else
				data_len = 0;
		}
		else if (spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE2)
			data_len = spi_trans->rx_len;
		else if (spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE3) {
			if ((spi_trans->tx_len + spi_trans->rx_len) > 4)
				data_len = spi_trans->tx_len + spi_trans->rx_len - 4;
			else if ((spi_trans->tx_len + spi_trans->rx_len) < 4)
				data_len = spi_trans->tx_len + spi_trans->rx_len - 1;
			else
				data_len = 0;
		}
		/*
		if (data_len < 0)
			data_len = 0;
		*/

    }
    data_len &= 0x1F;
    reg_value_8 = ioread8(master_ext->regs + 0x03);
    reg_value_8 &= 0xB0;
    if (data_len != 0) {
        reg_value_8 |= (data_len - 1);
	 reg_value_8 |= 0x40;
    }
	via_spi_printk(KERN_INFO "data byte count register is %x\n", reg_value_8);
    iowrite8(reg_value_8, master_ext->regs + 0x03);

	preload_data_for_pio_write(master_ext, spi_trans);

	reg_value_8 = ioread8(master_ext->regs + 0x02);
	reg_value_8 &= 0xFE;
	if ((master_ext->current_device_mode == VIA_SPI_DEVICE_MODE_FLASH) &&
	    (spi_trans->is_fast_read_transfer > 0))
		reg_value_8 |= 0x01;
	iowrite8(reg_value_8, master_ext->regs + 0x02);    

    /* for flash mode, also need set the command type */
    cmd_type = 0;
    cmd_type = get_flash_mode_cmd_type(spi_trans);
    reg_value_8 = ioread8(master_ext->regs + 0x56);
	reg_value_8 &= 0xFC;
    reg_value_8 |= cmd_type;
    iowrite8(reg_value_8, master_ext->regs + 0x56);

	wait_reg_offset = 0x00;
	wait_reg_value = 0x04;

start_transfer:
    /* now, walk to the last step, we need select the spi port */
    reg_value_8 = ioread8(master_ext->regs + 0x03);
    reg_value_8 &= 0xCF;
    reg_value_8 |= (chip_select << 4);
    iowrite8(reg_value_8, master_ext->regs + 0x03);

    /* if this master works at interrupt mode, we need init the completion
	 * structure and wait for the command to be completed
	 */
    if (isr_need)
		init_completion(&master_ext->transfer_done);

	/* dump the all registers for debug */
	/*
	via_spi_printk(KERN_INFO "Dump the all registers\n");
	for (i = 0; i < 0x88; i++) {
		via_spi_printk(KERN_INFO "%x, ", ioread8(master_ext->regs+i));
		if (((i+1) % 8) == 0)
			via_spi_printk(KERN_INFO "\n");
	}
	via_spi_printk(KERN_INFO "\n");
	*/

    /* now, we can set the cycle go bit to start this transfer */
    reg_value_8 = ioread8(master_ext->regs + 0x02);
    reg_value_8 |= 0x02;
    iowrite8(reg_value_8, master_ext->regs + 0x02);


    if (isr_need)
		wait_for_completion(&master_ext->transfer_done);
	else
		wait_status = wait_cmd_complete(
			master_ext->regs + wait_reg_offset,
			wait_reg_value);


	if (is_undefined_len_trans == 1)
		ssleep(actual_trans_len / 50);

    reg_value_8 = ioread8(master_ext->regs + 0x02);
    reg_value_8 &= 0xFD;
    iowrite8(reg_value_8, master_ext->regs + 0x02);

    /* clear the cycle done status bit to clear the interrupt */
    reg_value_8 = ioread8(master_ext->regs + 0x00);
    reg_value_8 |= 0x04;
    iowrite8(reg_value_8, master_ext->regs + 0x00);

    iowrite8(0x00, master_ext->regs + 0x70);
    iowrite8(0xFF, master_ext->regs + 0x73);

    if (wait_status != 0)
		return wait_status;

    /* if this transfer reads data from device, need copy the data from the
	 * data register to the buffer
	 */

    if ((spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE1) ||
		(spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE3))
		return 0;

    if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_PIO) {
		u8 base_addr = 0;
		if (master_ext->current_device_mode ==
				VIA_SPI_DEVICE_MODE_FLASH)
			base_addr = 0x00;
		else
			base_addr = spi_trans->tx_len;
		via_spi_printk(KERN_INFO "the data after read is :");
		for (i = 0; i < spi_trans->rx_len; i++) {
			spi_trans->rx_buf[i] = ioread8(master_ext->regs +
					0x08 + i + base_addr);
			via_spi_printk(KERN_INFO "%x,", spi_trans->rx_buf[i]);
		}
		via_spi_printk(KERN_INFO "\n");
    } else {
		memcpy(spi_trans->rx_buf, master_ext->via_dma_r_buffer,
			spi_trans->rx_len);
		via_spi_printk(KERN_INFO "the data after read is:");
		for (i = 0; i < spi_trans->rx_len; i++)
			via_spi_printk(KERN_INFO "%x, ", spi_trans->rx_buf[i]);
		via_spi_printk(KERN_INFO "\n");
    }
    return 0;

}

/***********************************************************
*
* Function: vt3409_master0_isr
* Precondition: interrupt enabled in SPI master controller
* Input:  @dev: point to struct via_spi_master_extension
* Output: int, whether this interrupt is happen on this master
* Purpose: process the interrupt which happens on the vt3409 master0
* Reference: none
*
***********************************************************/
int vt3409_master0_isr(void *dev)
{
    struct via_spi_master_extension *master_ext = NULL;
    u8 reg_value_8;
    int status = 1;

    master_ext = (struct via_spi_master_extension *)dev;
    reg_value_8 = ioread8(master_ext->regs + 0x00);
    if (reg_value_8 & 0x04) {
		via_spi_printk(KERN_INFO "master0 isr,cycle done is 1\n");
		status = 0;
    }
    reg_value_8 = ioread8(master_ext->regs + 0x73);
    if (reg_value_8 & 0xAC) {
		via_spi_printk(KERN_INFO "master0 isr, buf int, value is %x\n",
			reg_value_8);
		status = 0;
    }

    if (status == 1)
		return status;

    reg_value_8 = ioread8(master_ext->regs + 0x02);
    reg_value_8 &= 0xFD;
    iowrite8(reg_value_8, master_ext->regs + 0x02);

    reg_value_8 = ioread8(master_ext->regs + 0x00);
    reg_value_8 |= 0x04;
    iowrite8(reg_value_8, master_ext->regs + 0x00);

    iowrite8(0x00, master_ext->regs + 0x70);
    iowrite8(0xFF, master_ext->regs + 0x73);

	return status;
}


/***********************************************************
*
* Function: vt3409_master1_init
* Precondition: the platform is vt3409 and spi bus1 is implemented
* Input:  @chip_info: infromation of the spi controller
*           @master_ext: information of the spi master
* Output: int, whether the initialization of the spi master is successful.
* Purpose:
*     init the given spi master based on the hardware feature
* Reference: none
*
***********************************************************/

int vt3409_master1_init(struct via_spi_controller *chip_info,
		struct via_spi_master_extension *master_ext)
{
    u32 spi_register_bar;
    u32 spi_register_len;
    u32 status = 0;
    u8 config_value;
    u8 value_byte;
	u8 bus_num;


	config_value = ioread8(chip_info->mmio_regs + 0x09);
    chip_info->irq = config_value & 0x0F;

	iowrite8(0x01, chip_info->mmio_regs + 0x04); /*enable master1 */

    spi_register_bar = ioread32(chip_info->mmio_regs + 0x04);
    spi_register_len = 0x1C;
    spi_register_bar &= 0xFFFFFF00;

	bus_num = master_ext->bus_number;

    master_ext->ioarea = request_mem_region(
			spi_register_bar, spi_register_len, "via_spi_master1");
    if (master_ext->ioarea == NULL) {
		status = -ENXIO;
		via_spi_printk(KERN_INFO "fail req io mem for master%d\n",
			bus_num);
		return status;
    }

    master_ext->regs = ioremap(spi_register_bar, spi_register_len);
    if (master_ext->regs == NULL) {
		status = -ENOMEM;
		via_spi_printk(KERN_INFO "fail map io reg for master%d\n",
			bus_num);
		goto free_mem_region;
    }
    master_ext->trans_type[0].max_rx_len = PAGE_SIZE;
    master_ext->trans_type[1].max_tx_len = PAGE_SIZE;
    master_ext->trans_type[2].max_rx_len = 512;
    master_ext->trans_type[2].max_tx_len = 16;
    master_ext->trans_type[2].max_rx_tx_len = 512 + 16;
    master_ext->trans_type[3].max_tx_len = 16;

    master_ext->slave_mode_data_len = 64;

    /* clear SPI cycle go and block access status(write 1 ) */
    value_byte = ioread8(master_ext->regs + 0x06);
    value_byte = 0x02;
    iowrite8(value_byte, master_ext->regs + 0x06);

    /* set the clock speed */
    value_byte = ioread8(master_ext->regs + 0x00);
    value_byte = 0x01;
    iowrite8(value_byte, master_ext->regs + 0x00);

    /* disable interrupt, dynamic clock when initialization procedure
     *  also init the controller work mode
     */
    value_byte = ioread8(master_ext->regs + 0x01);
    if (master_ext->current_controller_mode == VIA_SPI_CONTROLLER_MODE_MASTER)
		value_byte = 0x08;
	else
		value_byte = 0x48;

    iowrite8(value_byte, master_ext->regs + 0x01);

    /* disable all dma buffer interrupt */
    value_byte = 0x00;
    iowrite8(value_byte, master_ext->regs + 0x03);

    /* clear all dma buffer interrupt */
    value_byte = 0xFF;
    iowrite8(value_byte, master_ext->regs + 0x07);

    return status;

free_mem_region:
	release_mem_region(master_ext->ioarea->start, 0x1C);
	return status;
}



/***********************************************************
*
* Function: vt3409_master1_resume
* Precondition: the platform is vt3409 and spi bus1 is implemented
* Input:  @chip_info: infromation of the spi controller
*           @master_ext: information of the spi master
* Output: int, whether the initialization of the spi master is successful.
* Purpose:
*     resume the given spi master based on the hardware feature
* Reference: none
*
***********************************************************/

int vt3409_master1_resume(struct via_spi_controller *chip_info,
		struct via_spi_master_extension *master_ext)
{
    u32 status = 0;
    u8 value_byte;
	

	iowrite8(0x01, chip_info->mmio_regs + 0x04); /*enable master1 */

    /* clear SPI cycle go and block access status(write 1 ) */
    value_byte = ioread8(master_ext->regs + 0x06);
    value_byte = 0x02;
    iowrite8(value_byte, master_ext->regs + 0x06);

    /* set the clock speed */
    value_byte = ioread8(master_ext->regs + 0x00);
    value_byte = 0x01;
    iowrite8(value_byte, master_ext->regs + 0x00);

    /* disable interrupt, dynamic clock when initialization procedure
     *  also init the controller work mode
     */
    value_byte = ioread8(master_ext->regs + 0x01);
    if (master_ext->current_controller_mode == VIA_SPI_CONTROLLER_MODE_MASTER)
		value_byte = 0x08;
	else
		value_byte = 0x48;

    iowrite8(value_byte, master_ext->regs + 0x01);

    /* disable all dma buffer interrupt */
    value_byte = 0x00;
    iowrite8(value_byte, master_ext->regs + 0x03);

    /* clear all dma buffer interrupt */
    value_byte = 0xFF;
    iowrite8(value_byte, master_ext->regs + 0x07);

    return status;
}


/***********************************************************
*
* Function: vt3409_master1_exit
* Precondition: the platform is vt3409 and has a spi controller
* Input:  @chip_info: infromation of the spi controller
*           @master_ext: information of the spi master
* Output: int, whether the exit of the spi master is successful.
* Purpose:
*     destroy the given spi master based on the hardware feature
* Reference: none
*
***********************************************************/

int vt3409_master1_exit(struct via_spi_controller *chip_info,
		struct via_spi_master_extension *master_ext)
{
    u32 status = 0;

    if (master_ext->ioarea != NULL)
		release_mem_region(master_ext->ioarea->start, 0x1C);

    if (master_ext->regs != NULL)
		iounmap(master_ext->regs);

	return status;
}


/***********************************************************
*
* Function: vt3409_master1_dispatch
* Precondition: the platform is vt3409 and spi bus1 is implemented
* Input:
*     @spi_dev:   the target device of this transfer
*     @spi_trans: the transfer to be executed
* Output: int, whether the transfer is dispatched to the spi master successfully
* Purpose:
*     program the spi controller to execute the requested transfer.
*     Note: must program the controller based on the transfer type of this
*           transfer
* Reference: none
*
***********************************************************/

int vt3409_master1_dispatch(struct spi_device *spi_dev,
		struct via_spi_transfer *spi_trans)
{
    struct spi_master *master;
    struct via_spi_master_extension *master_ext;
    u8 reg_value_8;
    u16 reg_value_16;
	u32 phy_addr;
    u8 data_offset = 0;
    u32 max_tx_len;
    u32 max_rx_len;
    int isr_need = 1;
	u8 wait_reg_offset = 0;
	u8 wait_reg_value = 0;
    u8 wait_status = 0;
	int is_undefined_len_trans = 0;
	u32 actual_trans_len = 0;

    via_spi_printk(KERN_INFO "enter vt3409_master1_dispatch routine\n");

    master = spi_dev->master;
    master_ext = spi_master_get_devdata(master);

	switch (spi_trans->transfer_type) {
	case VIA_SPI_TRANSFER_TYPE0:
	case VIA_SPI_TRANSFER_TYPE2:
		if (spi_trans->rx_len < 16)
			isr_need = 0;
		break;
	case VIA_SPI_TRANSFER_TYPE1:
	case VIA_SPI_TRANSFER_TYPE3:
		if ((spi_trans->tx_len + spi_trans->rx_len) < 16)
			isr_need = 0;
		break;
	}
	if (master_ext->current_transfer_mode == VIA_SPI_TRANSFER_MODE_PIO)
		isr_need = 1;
	if (master_ext->support_int == 0)
		isr_need = 0;
	via_spi_printk(KERN_INFO "tran dispatch with int mode %d\n", isr_need);

    /* clear the cycle done status bit */
    reg_value_8 = 0x02;
    iowrite8(reg_value_8, master_ext->regs + 0x06);

    /* enable the dynamic clock, disable command post write and set the
	 * interrupt enable
	 */
    reg_value_8 = ioread8(master_ext->regs + 0x01);
	via_spi_printk(KERN_INFO "in spi1 dispatch, the reg 0x01 value is %x\n",
		(u32)reg_value_8);
    reg_value_8 &= 0xE0;
    reg_value_8 |= 0x08;
    if (isr_need)
		reg_value_8 |= 0x10;
    iowrite8(reg_value_8, master_ext->regs + 0x01);

    /* set up the clock speed for this transfer */
    if (spi_trans->speed_hz != 0) {
		via_spi_calculate_speed(33*1000*1000, spi_trans->speed_hz,
				&reg_value_8);
		if ((reg_value_8 == 0) &&
			(master_ext->current_transfer_mode ==
			 VIA_SPI_TRANSFER_MODE_DMA))
			reg_value_8 = 1;
		iowrite8(reg_value_8, master_ext->regs + 0x00);
    }

    /* set the current work mode */
    reg_value_8 = ioread8(master_ext->regs + 0x01);
    reg_value_8 &= 0xBF;
    if (master_ext->current_controller_mode == VIA_SPI_CONTROLLER_MODE_SLAVE)
		reg_value_8 |= 0x40;
    iowrite8(reg_value_8, master_ext->regs + 0x01);

    /* set the clock latency */
    reg_value_8 = ioread8(master_ext->regs + 0x00);
	if (reg_value_8 == 0x01)
		reg_value_8 = 0x03;
	else
		reg_value_8 = 0x00;
    iowrite8(reg_value_8, master_ext->regs + 0x02);

    memset(master_ext->via_dma_w_buffer, 0, master_ext->via_dma_w_buf_len);
    memset(master_ext->via_dma_r_buffer, 0, master_ext->via_dma_r_buf_len);

    /* firstly, set the DMA read buffer address and write buffer address. for
	 * write data need copy it to a physical continous buffer
	 */
    if (spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE3) {
		memcpy(master_ext->via_dma_w_buffer,
			spi_trans->rx_buf, spi_trans->rx_len);
		data_offset = spi_trans->rx_len;
    }
    if (spi_trans->tx_len != 0) {
		memcpy(master_ext->via_dma_w_buffer + data_offset,
			spi_trans->tx_buf, spi_trans->tx_len);
    }

    /* set the transfer buffer address and length. for read and write command,
	 * use internal buffer because we should make sure that the buffer is
	 * continous in physical
	 */

    iowrite8(0x00, master_ext->regs + 0x03); /* disable buffer interrupt */
    iowrite8(0xFF, master_ext->regs + 0x07); /* clear buffer interrupt */
    max_tx_len = master_ext->trans_type[2].max_tx_len;
    max_rx_len = master_ext->trans_type[2].max_rx_len;

	switch (spi_trans->transfer_type) {
	case VIA_SPI_TRANSFER_TYPE0:
		phy_addr = virt_to_phys(master_ext->via_dma_r_buffer);
		iowrite32(phy_addr, master_ext->regs + 0x0C);
		iowrite16(spi_trans->rx_len, master_ext->regs + 0x12);
		if (spi_trans->rx_len > max_rx_len) {
			reg_value_16 = 0x00;
			iowrite8(0x80, master_ext->regs + 0x03);
			wait_reg_offset = 0x07;
			wait_reg_value = 0x80;
		} else {
			reg_value_16 = 0x8000;
			reg_value_16 |=
				(((spi_trans->rx_len - 1) << 4) & 0x1FF0);
			wait_reg_offset = 0x06;
			wait_reg_value = 0x02;
		}
		iowrite16(reg_value_16, master_ext->regs + 0x04);
		break;
	case VIA_SPI_TRANSFER_TYPE1:
		phy_addr = virt_to_phys(master_ext->via_dma_w_buffer);
		iowrite32(phy_addr, master_ext->regs + 0x08);
		iowrite16(spi_trans->tx_len, master_ext->regs + 0x10);
		if (spi_trans->tx_len > max_tx_len) {
			reg_value_16 = 0x02000;
			iowrite8(0x20, master_ext->regs + 0x03);
			wait_reg_offset = 0x07;
			wait_reg_value = 0x20;
			is_undefined_len_trans = 1;
			actual_trans_len = spi_trans->tx_len;
		} else {
			reg_value_16 = 0xA000;
			reg_value_16 |= ((spi_trans->tx_len - 1) & 0x0F);
			wait_reg_offset = 0x06;
			wait_reg_value = 0x02;
		}
		iowrite16(reg_value_16, master_ext->regs + 0x04);
		break;
	case VIA_SPI_TRANSFER_TYPE2:
		phy_addr = virt_to_phys(master_ext->via_dma_r_buffer);
		iowrite32(phy_addr, master_ext->regs + 0x0C);
		iowrite16(spi_trans->rx_len, master_ext->regs + 0x12);

		phy_addr = virt_to_phys(master_ext->via_dma_w_buffer);
		iowrite32(phy_addr, master_ext->regs + 0x08);
		iowrite16(spi_trans->tx_len, master_ext->regs + 0x10);

		reg_value_16 = 0xC000;
		reg_value_16 |= (((spi_trans->rx_len - 1) << 4) & 0x1FF0);
		reg_value_16 |= ((spi_trans->tx_len - 1) & 0x0F);
		iowrite16(reg_value_16, master_ext->regs + 0x04);

		wait_reg_offset = 0x06;
		wait_reg_value = 0x02;
		break;
	case VIA_SPI_TRANSFER_TYPE3:
		phy_addr = virt_to_phys(master_ext->via_dma_w_buffer);
		iowrite32(phy_addr, master_ext->regs + 0x08);
		iowrite16(spi_trans->tx_len + spi_trans->rx_len,
			master_ext->regs + 0x10);
		if ((spi_trans->tx_len + spi_trans->rx_len) > max_tx_len) {
			reg_value_16 = 0x2000;
			iowrite8(0x20, master_ext->regs + 0x03);
			wait_reg_offset = 0x07;
			wait_reg_value = 0x20;
			is_undefined_len_trans = 1;
			actual_trans_len =
				spi_trans->tx_len + spi_trans->rx_len;
		} else {
			reg_value_16 = 0xA000;
			reg_value_16 |= ((spi_trans->tx_len +
				spi_trans->rx_len - 1)&0x0F);
			wait_reg_offset = 0x06;
			wait_reg_value = 0x02;
		}
		iowrite16(reg_value_16, master_ext->regs + 0x04);
		break;
	}

    /* if this master works at interrupt mode, we need init the completion
	 * structure and wait for the command to be completed
	 */
    if (isr_need)
		init_completion(&master_ext->transfer_done);

    /* now, we can set the cycle go bit to start this transfer */
    reg_value_8 = ioread8(master_ext->regs + 0x06);
    reg_value_8 |= 0x01;
    iowrite8(reg_value_8, master_ext->regs + 0x06);


    if (isr_need)
		wait_for_completion(&master_ext->transfer_done);
	else
		wait_status =
			wait_cmd_complete(master_ext->regs + wait_reg_offset,
				wait_reg_value);

	if ((is_undefined_len_trans) &&
		(master_ext->current_controller_mode ==
		 VIA_SPI_CONTROLLER_MODE_SLAVE))
		ssleep(6);
	if ((is_undefined_len_trans) &&
		(master_ext->current_controller_mode ==
		 VIA_SPI_CONTROLLER_MODE_MASTER))
		ssleep(actual_trans_len / 50);

    /* clear the cycle done status bit to clear the interrupt */
    iowrite8(0x02, master_ext->regs + 0x06);

    iowrite8(0x00, master_ext->regs + 0x03);
    iowrite8(0xFF, master_ext->regs + 0x07);

    if (wait_status != 0)
		return wait_status;

    /* if this transfer reads data from device, need copy the data from the
	 * data register to the buffer
	 */
    if ((spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE0) ||
		(spi_trans->transfer_type == VIA_SPI_TRANSFER_TYPE2)) {
		memcpy(spi_trans->rx_buf, master_ext->via_dma_r_buffer,
			spi_trans->rx_len);
    }


    return 0;

}

/***********************************************************
*
* Function: vt3409_master1_isr
* Precondition: interrupt enabled in SPI master controller
* Input:  @dev: point to struct via_spi_master_extension
* Output: int, whether this interrupt is happen on this master
* Purpose: process the interrupt which happens on the vt3409 master1
* Reference: none
*
***********************************************************/
int vt3409_master1_isr(void *dev)
{
    struct via_spi_master_extension *master_ext = NULL;
    u8 reg_value_8;
    int status = 1;

    master_ext = (struct via_spi_master_extension *)dev;
    reg_value_8 = ioread8(master_ext->regs + 0x06);
    if (reg_value_8 & 0x02) {
		via_spi_printk(KERN_INFO " master1 isr,cycle done is 1\n");
		status = 0;
    }
    reg_value_8 = ioread8(master_ext->regs + 0x07);
    if (reg_value_8 & 0xAC)
		status = 0;

    if (status == 1)
		return status;

    iowrite8(0x02, master_ext->regs + 0x02);

    iowrite8(0x00, master_ext->regs + 0x03);
    iowrite8(0xFF, master_ext->regs + 0x07);

    return status;
}

struct via_spi_master vt3353_master_0 = {
    .master_num = 0,
    .num_chipselect = 2,
    .support_min_speed_hz = 0,
    .bits_per_word = 8,
    .support_controller_mode = VIA_SPI_CONTROLLER_MODE_MASTER,
    .support_transfer_mode = VIA_SPI_TRANSFER_MODE_PIO,
    .support_device_mode =
		VIA_SPI_DEVICE_MODE_DEVICE|VIA_SPI_DEVICE_MODE_FLASH,
    .support_int = 0,
    .master_init = vt3353_master0_init,
    .master_resume = vt3353_master0_resume,
    .master_exit = vt3353_master0_exit,
    .cmd_dispatch = vt3353_master0_dispatch,
};

struct via_spi_master vt3409_master_0 = {
    .master_num = 0,
    .num_chipselect = 3,
    .support_min_speed_hz = 0,
    .bits_per_word = 8,
    .support_controller_mode = VIA_SPI_CONTROLLER_MODE_MASTER,
    .support_transfer_mode =
		VIA_SPI_TRANSFER_MODE_PIO|VIA_SPI_TRANSFER_MODE_DMA,
    .support_device_mode =
		VIA_SPI_DEVICE_MODE_DEVICE|VIA_SPI_DEVICE_MODE_FLASH,
    .support_int = 0,
    .master_init = vt3409_master0_init,
    .master_resume = vt3409_master0_resume,
    .master_exit = vt3409_master0_exit,
    .cmd_dispatch = vt3409_master0_dispatch,
    .master_isr   = vt3409_master0_isr,
};

struct via_spi_master vt3409_master_1 = {
    .master_num = 1,
    .num_chipselect = 1,
    .support_min_speed_hz = 0,
    .bits_per_word = 8,
    .support_controller_mode =
		VIA_SPI_CONTROLLER_MODE_MASTER|VIA_SPI_CONTROLLER_MODE_SLAVE,
    .support_transfer_mode = VIA_SPI_TRANSFER_MODE_DMA,
    .support_device_mode = VIA_SPI_DEVICE_MODE_DEVICE,
    .support_int = 0,
    .master_init = vt3409_master1_init,
    .master_resume = vt3409_master1_resume,
    .master_exit = vt3409_master1_exit,
    .cmd_dispatch = vt3409_master1_dispatch,
    .master_isr   = vt3409_master1_isr,
};


/***********************************************************
*
* Function: via_spi_register_device
* Precondition: the master has been register to the system
* Input:
*     @spi_dev: the information of the spi device which is needed to register
*               to system
* Output: int, whether this spi device is registerd to the system successfully
* Purpose: register a spi device to the system and controller module
* Reference: none
*
***********************************************************/

int via_spi_register_device(struct via_spi_device *spi_dev)
{
	int status = 0;
	u8 bus_num;
	u8 chip_select;
	struct spi_master *master = NULL;
	struct via_spi_master_extension *master_ext = NULL;
	struct via_spi_device *dev;
	struct spi_board_info dev_info;
	struct spi_board_info *pdev_info;
	struct spi_device *pdev = NULL;

	/* firstly, should check whether the arguments are valid */
	bus_num = spi_dev->bus_num;
	chip_select = spi_dev->chip_select;

	via_spi_printk(KERN_INFO "register %s to master %d port %d\n",
		spi_dev->modalias, bus_num, chip_select);

	if (bus_num >= MAX_MASTER_NUMBER) {
		status = -EINVAL;
		goto error1;
	}

	master = via_spi_chip_info->master[bus_num];
	if (master == NULL) {
		status = -EINVAL;
		goto error1;
	}

	if (chip_select >= master->num_chipselect) {
		status = -EINVAL;
		goto error1;
	}

	master_ext = spi_master_get_devdata(master);
	if (master_ext == NULL) {
		status = -EINVAL;
		goto error1;
	}

	if ((spi_dev->max_speed_hz != 0) &&
		(spi_dev->max_speed_hz < master_ext->support_min_speed_hz)) {
		status = -EINVAL;
		goto error1;
	}

	if (master_ext->dev[chip_select] != NULL) {
		status = -EINVAL;
		goto error1;
	}

	via_spi_printk(KERN_INFO "device %s has valid args\n", spi_dev->modalias);

	/* allocate a copy of the via_spi_device for internal saving */
	dev = kzalloc(sizeof(struct via_spi_device), GFP_KERNEL);
	if (dev == NULL) {
		status = -EINVAL;
		goto error1;
	}
	memcpy(dev, spi_dev, sizeof(struct via_spi_device));
	master_ext->dev[chip_select] = dev;

	/* set the command parsing rourtine for this spi device */
	if (spi_dev->via_parse_spi_cmd == NULL)
		dev->via_parse_spi_cmd = m25p80_parse_spi_cmd;

	/* register the device to system */
	memset(&dev_info, 0, sizeof(struct spi_board_info));
	pdev_info = &dev_info;
	pdev_info->controller_data = master_ext;
	pdev_info->max_speed_hz = spi_dev->max_speed_hz;
	pdev_info->platform_data = spi_dev->platform_data;
	pdev_info->chip_select = spi_dev->chip_select;
	pdev_info->bus_num = spi_dev->bus_num;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 17)
	pdev_info->mode = spi_dev->mode;
#endif
	strcpy(pdev_info->modalias, spi_dev->modalias);

	pdev = spi_new_device(master, pdev_info);
	if (pdev == NULL) {
		via_spi_printk(KERN_INFO "fail to register spi device %s\n",
			spi_dev->modalias);
		status = -ENOMEM;
		master_ext->dev[chip_select] = NULL;
		kfree(dev);
	}

	master_ext->dev[chip_select]->spi_dev = spi_dev_get(pdev);

	via_spi_printk(KERN_INFO "via_spi:device %s register successfully\n",
		spi_dev->modalias);

error1:
	return status;
}

/***********************************************************
*
* Function: via_spi_unregister_device
* Precondition: the master has been register to the system
* Input: @spi_dev:the information of the spi device which is needed to
*                 unregister from system
* Output: int, whether this spi device is unregisterd to the system successfully
* Purpose: unregister a spi device from the system and controller module
* Reference: none
*
***********************************************************/

int via_spi_unregister_device(struct via_spi_device *spi_dev)
{
    int status = 0;
    u8 bus_num;
    u8 chip_select;
    struct spi_master *master = NULL;
    struct via_spi_master_extension *master_ext = NULL;
    struct spi_device *pdev = NULL;

    /* firstly, should check whether the arguments are valid */
    bus_num = spi_dev->bus_num;
    chip_select = spi_dev->chip_select;

    via_spi_printk(KERN_INFO "unregister %s to master %d port %d\n",
		spi_dev->modalias, bus_num, chip_select);

    if (bus_num >= MAX_MASTER_NUMBER) {
		status = -EINVAL;
		goto error1;
    }

    master = via_spi_chip_info->master[bus_num];
    if (master == NULL) {
		status = -EINVAL;
		goto error1;
    }

    if (chip_select >= master->num_chipselect) {
		status = -EINVAL;
		goto error1;
    }

    master_ext = spi_master_get_devdata(master);
    if (master_ext == NULL) {
		status = -EINVAL;
		goto error1;
    }

    if ((master_ext->dev[chip_select] == NULL) ||
		(master_ext->dev[chip_select]->spi_dev == NULL)) {
		status = -EINVAL;
		goto error1;
    }

    pdev = master_ext->dev[chip_select]->spi_dev;

    /* unregister the spi device from the system */
    spi_unregister_device(pdev);
    spi_dev_put(pdev);

    kfree(master_ext->dev[chip_select]);

    master_ext->dev[chip_select] = NULL;

    via_spi_printk(KERN_INFO "via_spi: the device unregistered successfully\n");

error1:
    return status;
}

/***********************************************************
*
* Function: via_spi_check_msg_arg
* Precondition: a SPI slave device is registered into OS
* Input:  @sdev: master side proxy for an SPI slave device
*         @m:     the spi command to be checked
* Output: int, 0--the arg is valid, otherwise --the arg is invalid
* Purpose: check the command parameter, such as bits_per_word, speed, etc. .
* Reference: none
*
***********************************************************/
static int via_spi_check_msg_arg(struct spi_device *sdev,
	struct spi_message *m)
{
    struct via_spi_master_extension *master_ext;
    struct spi_transfer *t;

    master_ext = spi_get_ctldata(sdev);

    list_for_each_entry(t, &m->transfers, transfer_list) {

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 17)
		if ((t->speed_hz) &&
			(t->speed_hz < master_ext->support_min_speed_hz))
			 return -EINVAL;
		if ((t->bits_per_word) &&
			(t->bits_per_word != master_ext->bits_per_word))
			return -EINVAL;
#endif

		if (!t->tx_buf && !t->rx_buf && t->len)
			return -EINVAL;
	}

    return 0;
}

/***********************************************************
*
* Function: via_spi_check_trans_arg
* Precondition: a SPI slave device is registered into OS
* Input:  @sdev: master side proxy for an SPI slave device
*         @m:     the spi command to be checked
* Output: int, 0--the arg is valid, otherwise --the arg is invalid
* Purpose: check the command parameter, such as bits_per_word, speed, etc. .
* Reference: none
*
***********************************************************/
static int via_spi_check_trans_arg(struct spi_device *sdev,
	struct via_spi_transfer *m)
{
    struct via_spi_master_extension *master_ext;
    int status = 0;

    master_ext = spi_master_get_devdata(sdev->master);

    if ((m->speed_hz) && (m->speed_hz < master_ext->support_min_speed_hz))
		return -EINVAL;

    if ((m->bits_per_word) &&
		(m->bits_per_word != master_ext->bits_per_word))
		return -EINVAL;

	switch (m->transfer_type) {
	case 0: /* only read data from spi device */
		if ((m->rx_buf == NULL) || (m->rx_len == 0)) {
			via_spi_printk(KERN_INFO "via_spi:type0," \
					"rx_buf/rx_len NULL\n");
			status = -EINVAL;
		}
		if ((m->tx_buf != NULL) || (m->tx_len != 0))
			status = -EINVAL;
		if (m->actual_rx_len > m->rx_len)
			status = -EINVAL;
		if (m->rx_len > master_ext->trans_type[0].max_rx_len)
			status = -EINVAL;
		break;
	case 1: /* only write data to spi device */
		if ((m->tx_buf == NULL) || (m->tx_len == 0))
			status = -EINVAL;
		if ((m->rx_buf != NULL) || (m->rx_len != 0))
			status = -EINVAL;
		if (m->actual_tx_len > m->tx_len)
			status = -EINVAL;
		if (m->tx_len > master_ext->trans_type[1].max_tx_len)
			status = -EINVAL;
		break;
	case 2:
		if (!m->rx_buf || !m->rx_len || !m->tx_buf || !m->tx_len)
			status = -EINVAL;
		if ((m->actual_tx_len > m->tx_len) ||
			(m->actual_rx_len > m->rx_len))
			status = -EINVAL;
		if ((m->rx_len > master_ext->trans_type[2].max_rx_len) ||
			(m->tx_len > master_ext->trans_type[2].max_tx_len) ||
			(m->rx_len + m->tx_len >
			 master_ext->trans_type[2].max_rx_tx_len))
			status = -EINVAL;
		break;
	case 3: /* write two discontious data block to device */
		if (!m->rx_buf || !m->rx_len || !m->tx_buf || !m->tx_len)
			status = -EINVAL;
		if ((m->actual_tx_len > m->tx_len) ||
			(m->actual_rx_len > m->rx_len))
			status = -EINVAL;
		if (m->rx_len + m->tx_len >
				master_ext->trans_type[3].max_tx_len)
			status = -EINVAL;
		break;
	}

	return status;
}

/***********************************************************
*
* Function: via_spi_setup
* Precondition: a SPI slave device is registered into OS
* Input:  @sdev: master side proxy for an SPI slave device
* Output: int, 0--SUCCESS
* Purpose: set up transfer for the slave device attached to controller,
*                such as bits_per_word, mode and so on
*
* Reference: none
*
***********************************************************/
static int via_spi_setup(struct spi_device *sdev)
{
    struct spi_master *master;
    struct via_spi_master_extension *master_ext;

    via_spi_printk(KERN_INFO "enter via_spi_setup routine\n");

    master = sdev->master;
    master_ext = sdev->controller_data;

    /* check whether sdev is valid */
    if (master_ext != spi_master_get_devdata(master))
		return -EINVAL;

    if (!sdev->bits_per_word)
		sdev->bits_per_word = 8;
    else if (sdev->bits_per_word != master_ext->bits_per_word)
		return -EINVAL;

    if (sdev->mode & ~(0x03)) {
		via_spi_printk(KERN_INFO "device %s unsupport mode bits %x\n",
			sdev->modalias, sdev->mode);
		return -EINVAL;
    }

    if ((sdev->max_speed_hz != 0) &&
		(sdev->max_speed_hz < master_ext->support_min_speed_hz))
		return -EINVAL;

    if (sdev->max_speed_hz == 0)
		sdev->max_speed_hz = master_ext->current_speed_hz;

    return 0;
}

/***********************************************************
*
* Function: via_spi_transfer_work
* Precondition: a work is submited to the workqueue
* Input:  @work: the work structure which is being processed now
* Output: none
* Purpose:
*     process the command in the command queue. it must process all the command
*     in the queue because all the command use the same work.
* Reference: none
*
***********************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
static void via_spi_transfer_work(void *work_temp)
#else
static void via_spi_transfer_work(struct work_struct *work)
#endif
{
    struct via_spi_master_extension *master_ext;
    unsigned long       flags;
    struct spi_message  *m;
	//struct spi_transfer *first_t;
	//unsigned char *tx_buffer;
	//unsigned char cmd_code_bakeup;
	//unsigned char bCmdCodeBakeup;
    struct spi_device   *spi_dev;
    int         status;
    unsigned char chip_select;
    struct via_spi_message *vmsg;
    struct via_spi_device *vspd = NULL;
    struct via_spi_transfer *vst = NULL;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
    struct work_struct *work = (struct work_struct *)work_temp;
#endif	

    master_ext = container_of(work,
			struct via_spi_master_extension, work);

    spin_lock_irqsave(&master_ext->queue_lock, flags);

    master_ext->busy = 1;

    while (!list_empty(&master_ext->queue)) {

		m = container_of(master_ext->queue.next,
				struct spi_message, queue);
		list_del_init(&m->queue);
		spin_unlock_irqrestore(&master_ext->queue_lock, flags);

		//first_t = container_of(m->transfers.next,
		//		struct spi_transfer, transfer_list);
		spi_dev = m->spi;
		chip_select = spi_dev->chip_select;
		status = 0;

		/* construct a via_spi_message to describe this command */
		vmsg = (struct via_spi_message *)
			(master_ext->via_spi_message_buffer);
		memset(vmsg, 0, sizeof(struct via_spi_message));
		vmsg->spi_command = m;
		memcpy(vmsg->trans_type, master_ext->trans_type,
			sizeof(struct via_spi_trans_type)*4);
		vmsg->via_spi_transfer_buffer =
			master_ext->via_spi_transfer_buffer;
		vmsg->via_spi_transfer_buffer_len =
			master_ext->via_spi_trans_buf_len;
		vmsg->via_backup_buffer =
			master_ext->via_backup_buffer;
		vmsg->via_backup_buf_len =
			master_ext->via_backup_buf_len;
		INIT_LIST_HEAD(&vmsg->via_transfers);

		/* call the device provided routine to parse the command */

		vspd = master_ext->dev[chip_select];
		if (vspd == NULL) {
			m->status = -EINVAL;
			m->complete(m->context);
			spin_lock_irqsave(&master_ext->queue_lock, flags);
			continue;
		}

		if (vspd->via_parse_spi_cmd == NULL)
			vspd->via_parse_spi_cmd = m25p80_parse_spi_cmd;

		/*
		tx_buffer = (unsigned char *)(first_t->tx_buf);
		if ((tx_buffer[0] == 0x02) && (vspd->bSST25vf080bType == 1)) {
			via_spi_printk(KERN_INFO "back up the command code\n");
			bCmdCodeBakeup = 1;
			cmd_code_bakeup = tx_buffer[0];
		} else {
			bCmdCodeBakeup = 0;
		}
		*/

		status = vspd->via_parse_spi_cmd(spi_dev, vmsg);
		if (status != 0) {
			m->status = -EINVAL;
			m->complete(m->context);
			spin_lock_irqsave(&master_ext->queue_lock, flags);
			continue;
		}

		list_for_each_entry(vst, &vmsg->via_transfers,
				via_transfer_list) {
			/* check the para of this transfer firstly */
			status = via_spi_check_trans_arg(spi_dev, vst);
			if (status != 0) {
				via_spi_printk(KERN_INFO "via_spi: has" \
						"invalid transfer\n");
				status = -EMSGSIZE;
				break;
			}

			/* dispatch the command to controller hardware */
			status = master_ext->cmd_dispatch(spi_dev, vst);
			if (status != 0) {
				via_spi_printk(KERN_INFO "via_spi: has" \
						"invalid transfer\n");
				status = -EMSGSIZE;
				break;
			}

			if (vst->transfer_callback != NULL)
				vst->transfer_callback(spi_dev,
					vst->transfer_callback_context);

			/* delay as the transfer need */
			if (vst->delay_usecs > 0)
				udelay(vst->delay_usecs);
			m->actual_length +=
				(vst->actual_rx_len + vst->actual_tx_len);

			status = 0;
		}

		/*
		if (bCmdCodeBakeup) {
			via_spi_printk(KERN_INFO "restore the command code\n");
			tx_buffer = (unsigned char *)(first_t->tx_buf);
			tx_buffer[0] = cmd_code_bakeup;
			bCmdCodeBakeup = 0;
		}
		*/

		/*now, the message has been transfered totally */
		m->status = status;
		m->complete(m->context);

		spin_lock_irqsave(&master_ext->queue_lock, flags);
	}
	master_ext->busy = 0;
	spin_unlock_irqrestore(&master_ext->queue_lock, flags);
}

/***********************************************************
*
* Function: via_spi_transfer
* Precondition: spi device driver issue a spi command to the controller driver
*               by this routine
* Input:  @spi: the target device for this command
*         @m:  describe the command
* Output: int, whether the command is accepted by the controller driver
* Purpose:
*     the linux spi subsystem routine, spi device issue command to spi
*     controller driver by this routine. check the argument and submit the
*     command to the command queue
* Reference: none
*
***********************************************************/

int via_spi_transfer(struct spi_device *spi, struct spi_message *m)
{
    struct spi_master *master;
    struct via_spi_master_extension *master_ext;
    unsigned long       flags;
    int         status = 0;

    m->actual_length = 0;
    m->status = -EINPROGRESS;

    master = spi->master;
    master_ext = spi->controller_data;

    if (master_ext != spi_master_get_devdata(master)) {
		m->status = EINVAL;
		m->complete(m->context);
		return -EINVAL;
    }

    /* check whether the para in every spi_transfer is valid */
    status = via_spi_check_msg_arg(spi, m);
    if (status != 0) {
		via_spi_printk(KERN_INFO "via_spi:spi_msg has" \
				"invalid spi_trans\n");
		m->status = EINVAL;
		m->complete(m->context);
		return -EINVAL;
    }

    spin_lock_irqsave(&master_ext->queue_lock, flags);
    if (!spi->max_speed_hz)
		status = -ENETDOWN;
    else {
		list_add_tail(&m->queue, &master_ext->queue);
		queue_work(master_ext->workqueue, &master_ext->work);
    }
    spin_unlock_irqrestore(&master_ext->queue_lock, flags);
    return status;
}

/***********************************************************
*
* Function: via_spi_irq
* Precondition: interrupt enabled in SPI master controller
* Input:  @irq: IRQ number for SPI master controller
*            @dev: point to struct via_spi_chip_info
* Output: irqreturn_t
* Purpose: process the interrupt which means that one transfer has been
*          completed
* Reference: none
*
***********************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
static irqreturn_t via_spi_irq(int irq, void *dev, struct pt_regs *reg_ptr)
#else
static irqreturn_t via_spi_irq(int irq, void *dev)
#endif
{
    struct via_spi_controller *spi_chip;
    struct spi_master *master = NULL;
    struct via_spi_master_extension *master_ext = NULL;
    int i;
    u8 is_our_int = 0;
    int status = 0;

    via_spi_printk(KERN_INFO "into irq handle routine!\n");

    spi_chip = (struct via_spi_controller *)dev;
    if ((spi_chip->busctl_pci_dev_id == 0x83531106) ||
        (spi_chip->busctl_pci_dev_id == 0x34021106)) {
		via_spi_printk(KERN_INFO "SPI doesn't support interrupt\n");
		return IRQ_NONE;
    }
    if ((spi_chip->busctl_pci_dev_id != 0x84091106) &&
		(spi_chip->busctl_pci_dev_id != 0x84101106) &&
		(spi_chip->busctl_pci_dev_id != 0x345B1106)) {
		via_spi_printk(KERN_INFO "not our int\n");
		return IRQ_NONE;
    }

    for (i = 0; i < spi_chip->num_master; i++) {
		master = spi_chip->master[i];
		if (master == NULL)
			continue;
		master_ext = spi_master_get_devdata(master);
		if (master_ext == NULL)
			continue;
		if (!master_ext->master_isr)
			continue;
		status = master_ext->master_isr(master_ext);
		if (status != 0)
			continue;

		via_spi_printk(KERN_INFO "via_spi:int occur on master %d\n",
				i);

		is_our_int = 1;
		complete(&master_ext->transfer_done);
	}

    if (is_our_int == 0)
		via_spi_printk(KERN_INFO "via_spi: int not our interrupt\n");
    return is_our_int?IRQ_HANDLED:IRQ_NONE;
}


/***********************************************************
*
* Function: via_spi_master_proc_exit
* Precondition: a spi_master structrue has been allocated for the spi master
* Input:  @master_ext: the master extension
* Output: none.
* Purpose: remove associated proc file for this master
* Reference: none
*
***********************************************************/

void via_spi_master_proc_exit(struct via_spi_master_extension *master_ext)
{
    int    i;
    unsigned char master_name[8];

	if (!master_ext->master_proc)
		return;
    if (!via_proc_spi)
		return;

	for (i = 0; i < ARRAY_SIZE(via_proc_file_tbl); i++)
		remove_proc_entry(via_proc_file_tbl[i].name,
				master_ext->master_proc);

    snprintf(master_name, 8, "%s%d", "master", master_ext->bus_number);
    remove_proc_entry(master_name, via_proc_spi);

   return;
}

/***********************************************************
*
* Function: via_spi_master_proc_init
* Precondition: a spi_master structrue has been allocated for the spi master
* Input:  @master_ext: the master extension
* Output: int, whether the initialization of proc file is successful.
* Purpose: create associated proc file for this master
* Reference: none
*
***********************************************************/

int via_spi_master_proc_init(struct via_spi_master_extension *master_ext)
{
    struct proc_dir_entry *pde;
    int    i;
    unsigned char master_name[8];

	if (!via_proc_spi)
		return -1;

    snprintf(master_name, 8, "%s%d", "master", master_ext->bus_number);
    master_ext->master_proc = proc_mkdir(master_name, via_proc_spi);
    if (master_ext->master_proc == NULL)
		return -1;

	for (i = 0; i < ARRAY_SIZE(via_proc_file_tbl); i++) {
		pde = create_proc_entry(via_proc_file_tbl[i].name,
				via_proc_file_tbl[i].mode | S_IFREG,
				master_ext->master_proc);

		if (!pde) {
			via_spi_printk(KERN_INFO "fail create %s proc file\n",
				via_proc_file_tbl[i].name);
			goto out;
		} else
			via_spi_printk(KERN_INFO "success create %s file\n",
				via_proc_file_tbl[i].name);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 30)
		pde->owner      = THIS_MODULE;
#endif
		pde->data       = master_ext;
		pde->read_proc  = via_proc_file_tbl[i].show;
		pde->write_proc = via_proc_file_tbl[i].store;
	}

	return 0;

out:

    via_spi_master_proc_exit(master_ext);

    return -1;
}

/***********************************************************
*
* Function: via_spi_master_start
* Precondition: a spi_master structrue has been allocated for the spi master
* Input:  @chip_info: infromation of the spi controller
*          @master_info: information of the spi master
* Output: int, whether the initialization of the spi master is successful.
* Purpose:
*     init the given spi master, including the via_spi_master_extension
*     structure and the hardware.
* Reference: none
*
***********************************************************/

int via_spi_master_start(struct via_spi_controller *chip_info,
		struct via_spi_master *master_info)
{
    int status = 0;
    unsigned char bus_num;
    struct spi_master *master = NULL;
    struct via_spi_master_extension *master_ext;

    unsigned char workqueue_name[20];

	unsigned int order = 0;

    bus_num = master_info->master_num;
    master = chip_info->master[bus_num];
    master->bus_num = bus_num;

    master->num_chipselect = master_info->num_chipselect;

    /* install the linux SPI subsystem routines which will be called by spi
	 * core and spi device driver
	 */
    master->setup = via_spi_setup;
    master->transfer = via_spi_transfer;

    master_ext = spi_master_get_devdata(master);
    memset(master_ext, 0, sizeof(struct via_spi_master_extension));

    master_ext->master = master;
    master_ext->bus_number = bus_num;
    master_ext->fake_dev_chip_sel = -1;

    snprintf(workqueue_name, 20, "%s.%u", "via_spi", bus_num);
    master_ext->workqueue = create_singlethread_workqueue(workqueue_name);
    if (master_ext->workqueue == NULL)
		return -EBUSY;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
	INIT_WORK(&master_ext->work, via_spi_transfer_work,
			(void *)&master_ext->work);
#else
	INIT_WORK(&master_ext->work, via_spi_transfer_work);
#endif
    spin_lock_init(&master_ext->queue_lock);
    INIT_LIST_HEAD(&master_ext->queue);

    master_ext->busy = 0;

    master_ext->support_min_speed_hz = master_info->support_min_speed_hz;
    master_ext->current_speed_hz = 33*1000*1000/4;

    master_ext->bits_per_word = master_info->bits_per_word;

    /* init the controller mode, device mode and transfer mode */
    master_ext->support_controller_mode = master_info->support_controller_mode;
    master_ext->support_device_mode = master_info->support_device_mode;
    master_ext->support_transfer_mode = master_info->support_transfer_mode;
    master_ext->current_controller_mode = VIA_SPI_CONTROLLER_MODE_MASTER;
    master_ext->current_device_mode = VIA_SPI_DEVICE_MODE_DEVICE;
    if (master_ext->support_transfer_mode & VIA_SPI_TRANSFER_MODE_DMA)
		master_ext->current_transfer_mode = VIA_SPI_TRANSFER_MODE_DMA;
	else
		master_ext->current_transfer_mode = VIA_SPI_TRANSFER_MODE_PIO;

    master_ext->support_int = master_info->support_int;

    /* allocate the buffer for via_spi_message which replace the spi command
	 * from spi device driver
	 */
    master_ext->via_spi_message_buffer =
		kmalloc(sizeof(struct via_spi_message), GFP_KERNEL);
    if (master_ext->via_spi_message_buffer == NULL) {
		status = -ENOMEM;
		goto free_workqueue;
    }

    /* allocate the buffer for via_spi_transfer which will be used by device
	 * module to create sub transfer
	 */
	 if ((via_spi_chip_info->busctl_pci_dev_id == 0x84101106) ||
	     (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106))
	 	order = 2;
	 else
	 	order = 0;
    master_ext->via_spi_transfer_buffer = (void *)__get_free_pages(GFP_KERNEL, order);
    if (master_ext->via_spi_transfer_buffer == NULL) {
		status = -ENOMEM;
		goto free_message_buffer;
    } else {
		master_ext->via_spi_trans_buf_len = PAGE_SIZE;
    }

	via_spi_master_proc_init(master_ext);

    if (master_info->master_init)
		status = master_info->master_init(chip_info, master_ext);
	if (status != 0)
		goto free_transfer_buffer;

    /* allocate dma write buffer if needed */
	master_ext->via_dma_w_buffer =
		(unsigned char *)__get_free_page(GFP_KERNEL|GFP_DMA);
	if (master_ext->via_dma_w_buffer == NULL) {
		status = -ENOMEM;
		goto free_transfer_buffer;
	} else {
		master_ext->via_dma_w_buf_len = PAGE_SIZE;
	}

    /* allocate dma write buffer if needed */
    if (master_ext->support_transfer_mode & VIA_SPI_TRANSFER_MODE_DMA) {
		master_ext->via_dma_r_buffer =
			(unsigned char *)__get_free_page(GFP_KERNEL|GFP_DMA);
		if (master_ext->via_dma_r_buffer == NULL) {
			status = -ENOMEM;
			goto free_dma_w_buffer;
		} else {
			 master_ext->via_dma_r_buf_len = PAGE_SIZE;
		}
	}


    /* allocate backup buffer, this buffer can be used to save command code
	 * and address
	 */
    master_ext->via_backup_buffer =
		(void *)__get_free_page(GFP_KERNEL | GFP_DMA);
	if (master_ext->via_backup_buffer == NULL) {
		status = -ENOMEM;
		goto free_dma_r_buffer;
    } else {
		master_ext->via_backup_buf_len = PAGE_SIZE;
    }

    master_ext->cmd_dispatch = master_info->cmd_dispatch;

    return status;

free_dma_r_buffer:
    if (master_ext->via_dma_r_buffer)
		free_page((unsigned long)master_ext->via_dma_r_buffer);
free_dma_w_buffer:
    if (master_ext->via_dma_w_buffer)
		free_page((unsigned long)master_ext->via_dma_w_buffer);
free_transfer_buffer:
    free_pages((unsigned long)master_ext->via_spi_transfer_buffer, order);
	via_spi_master_proc_exit(master_ext);
free_message_buffer:
    kfree(master_ext->via_spi_message_buffer);
free_workqueue:
    destroy_workqueue(master_ext->workqueue);
    return status;
}


/***********************************************************
*
* Function: via_spi_master_resume
* Precondition: a spi_master structrue has been allocated for the spi master
* Input:  @chip_info: infromation of the spi controller
*          @master_info: information of the spi master
* Output: int, whether the resume of the spi master is successful.
* Purpose:
*  resume the given master, mainly remove the protection
* Reference: none
*
***********************************************************/

int via_spi_master_resume(struct via_spi_controller *chip_info,
		struct via_spi_master *master_info)
{
    int status = 0;
    unsigned char bus_num;
    struct spi_master *master = NULL;
    struct via_spi_master_extension *master_ext;   

    bus_num = master_info->master_num;
    master = chip_info->master[bus_num];
    master_ext = spi_master_get_devdata(master);    

    if (master_info->master_resume)
		status = master_info->master_resume(chip_info, master_ext);    

    return status;
}


/***********************************************************
*
* Function: via_spi_master_stop
* Precondition: a spi master need to be unregister from the system
* Input:  @chip_info: infromation of the spi controller
*           @master_info: information of the spi master
* Output: None.
* Purpose:
*     stop the spi master, mainly destroy the resouce of this master.
* Reference: none
*
***********************************************************/

void via_spi_master_stop(struct via_spi_controller *chip_info,
		struct via_spi_master *master_info)
{
    unsigned char bus_num;
    struct spi_master *master = NULL;
    struct via_spi_master_extension *master_ext;
	unsigned int order = 0;

	via_spi_printk(KERN_INFO "enter the via_spi_master_stop routine\n");

    bus_num = master_info->master_num;
    master = chip_info->master[bus_num];

    master_ext = spi_master_get_devdata(master);

    if (master_ext->fake_dev_chip_sel >= 0) {
		u8 chip_sel = master_ext->fake_dev_chip_sel;
		via_spi_printk(KERN_INFO "the fake dev exist,delete it\n");
		kfree(master_ext->dev[chip_sel]->spi_dev);
		kfree(master_ext->dev[chip_sel]);
		master_ext->dev[chip_sel] = NULL;
    }
    if (master_ext->bus_data_proc) {
		remove_proc_entry(spi_bus_data.name,
				master_ext->master_proc);
		master_ext->bus_data_proc = NULL;
    }
    if (master_ext->bus_data_len_proc) {
		remove_proc_entry(spi_bus_data_len.name,
				master_ext->master_proc);
		master_ext->bus_data_len_proc = NULL;
    }
    if (master_ext->slave_test_port_proc) {
		remove_proc_entry(slave_test_port.name,
				master_ext->master_proc);
		master_ext->slave_test_port_proc = NULL;
    }

	if ((via_spi_chip_info->busctl_pci_dev_id == 0x84101106) ||
	    (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106))
		order = 2;
	else
		order = 0;

    via_spi_master_proc_exit(master_ext);

    if (master_ext->via_dma_w_buffer)
		free_page((unsigned long)master_ext->via_dma_w_buffer);
    if (master_ext->via_dma_r_buffer)
		free_page((unsigned long)master_ext->via_dma_r_buffer);
    if (master_ext->via_spi_transfer_buffer)
		free_pages((unsigned long)master_ext->via_spi_transfer_buffer, order);
    if (master_ext->via_backup_buffer)
		free_page((unsigned long)master_ext->via_backup_buffer);
    if (master_ext->via_spi_message_buffer)
		kfree(master_ext->via_spi_message_buffer);
    if (master_ext->workqueue)
		destroy_workqueue(master_ext->workqueue);

    if (master_info->master_exit)
		master_info->master_exit(chip_info, master_ext);
	via_spi_printk(KERN_INFO "out via_spi_stop_master routine\n");
}

/***********************************************************
*
* Function: via_spi_probe
* Precondition: paltform device matches the platform driver
* Input: @pdev:  point to a struct platform_device
* Output: int
* Purpose: get a struct spi_master master to point to SPI master controller,
*          register SPI subsystem routines, such as via_spi_transfer and
*          via_spi_setup,request mem resource and irq routine, register SPI
*    	   master controller and SPI slave device to OS
* Reference: none
*
***********************************************************/
static int __init via_spi_probe(struct platform_device *pdev)
{
	int ret = 0;
 	int i;
 	struct spi_master *master = NULL;
	struct via_spi_master_extension *master_ext = NULL;

	u32 VT3409_SPI_base_reg;

	via_spi_printk(KERN_INFO "into:via_spi_probe\n");

 	/* allocate a via_spi_controller structure to save the chip information */
	via_spi_chip_info = kzalloc(sizeof *via_spi_chip_info, GFP_KERNEL);
	if (via_spi_chip_info == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, via_spi_chip_info);

	/* init the via_spi_chip_info */
	via_spi_chip_info->busctl_pci_dev_id = busctl_dev_ven_id;
	via_spi_chip_info->platform_dev = via_spi_adapter;

	if ((via_spi_chip_info->busctl_pci_dev_id != 0x83531106) &&
		(via_spi_chip_info->busctl_pci_dev_id != 0x34021106) &&
		(via_spi_chip_info->busctl_pci_dev_id != 0x84091106) &&
		(via_spi_chip_info->busctl_pci_dev_id != 0x84101106) &&
		(via_spi_chip_info->busctl_pci_dev_id != 0x345B1106)) {
		via_spi_printk(KERN_INFO "via_spi:platform not supported\n");
		ret = -ENODEV;
		goto free_mem;
	}

	if ((via_spi_chip_info->busctl_pci_dev_id == 0x83531106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x34021106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x84101106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x345B1106))
		via_spi_chip_info->num_master = 1;
	else
		via_spi_chip_info->num_master = 2;

	/* allocate spi_master for every SPI bus */
	for (i = 0; i < via_spi_chip_info->num_master; i++) {
		master = spi_alloc_master(&pdev->dev,
			sizeof(struct via_spi_master_extension));
		if (master == NULL) {
			via_spi_printk(KERN_INFO "No memory for spi_master!\n");
			ret = -ENOMEM;
			goto put_master;
		}
		via_spi_chip_info->master[i] = spi_master_get(master);
	}

	/* special init for VT3409, map mmio registers */
	if ((via_spi_chip_info->busctl_pci_dev_id == 0x84091106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x84101106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)) {
		if (via_spi_chip_info->busctl_pci_dev_id == 0x84091106)
			via_spi_chip_info->via_busctl_pci_dev =
				pci_get_device(PCI_VENDOR_ID_VIA,
					PCI_DEVICE_ID_VIA_SB_VT3409, NULL);
		else if (via_spi_chip_info->busctl_pci_dev_id == 0x84101106)
			via_spi_chip_info->via_busctl_pci_dev =
				pci_get_device(PCI_VENDOR_ID_VIA,
					PCI_DEVICE_ID_VIA_SB_VT3410, NULL);
		else
			via_spi_chip_info->via_busctl_pci_dev =
				pci_get_device(PCI_VENDOR_ID_VIA,
					PCI_DEVICE_ID_VIA_SB_VT3456, NULL);

		pci_read_config_dword(
			via_spi_chip_info->via_busctl_pci_dev, 0xBC,
			&VT3409_SPI_base_reg);
		VT3409_SPI_base_reg = (VT3409_SPI_base_reg << 8) & 0xFFFFF000;
		via_spi_printk(KERN_INFO "the VT3409 base reg is %x\n",
			VT3409_SPI_base_reg);
		pci_dev_put(via_spi_chip_info->via_busctl_pci_dev);

		via_spi_chip_info->mmio_ioarea = request_mem_region(
				VT3409_SPI_base_reg, 10, "via_spi_ctrl");
		if (via_spi_chip_info->mmio_ioarea == NULL) {
			ret = -ENXIO;
			via_spi_printk(KERN_INFO "via_spi:fail req iomem\n");
			goto put_master;
		}

		via_spi_chip_info->mmio_regs = ioremap(VT3409_SPI_base_reg, 10);
		if (via_spi_chip_info->mmio_regs == NULL) {
			ret = -ENOMEM;
			via_spi_printk(KERN_INFO "via_spi:fail map io reg\n");
			goto free_mem_region;
		}
	}

	if ((via_spi_chip_info->busctl_pci_dev_id == 0x83531106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x34021106)) {
		ret = via_spi_master_start(via_spi_chip_info, &vt3353_master_0);
		if (ret != 0) {
			via_spi_printk(KERN_INFO "via_spi:fail start" \
					"master0\n");
			goto unmap_io;
		}
	} else if (via_spi_chip_info->busctl_pci_dev_id == 0x84091106) {
		ret = via_spi_master_start(via_spi_chip_info, &vt3409_master_0);
		if (ret != 0) {
			via_spi_printk(KERN_INFO "via_spi: fail start" \
					"master0\n");
			goto unmap_io;
		}

		via_spi_printk(KERN_INFO "successfully start VT3409 master0\n");

		ret = via_spi_master_start(via_spi_chip_info, &vt3409_master_1);
		if (ret != 0) {
			via_spi_printk(KERN_INFO "via_spi: fail start " \
					"master1\n");
			via_spi_master_stop(via_spi_chip_info,
					&vt3409_master_0);
			goto unmap_io;
		}
		via_spi_printk(KERN_INFO "successfully start VT3409 master1\n");
	} else if ((via_spi_chip_info->busctl_pci_dev_id == 0x84101106) ||
		   (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)) {
		ret = via_spi_master_start(via_spi_chip_info, &vt3409_master_0);
		if (ret != 0) {
			via_spi_printk(KERN_INFO "via_spi: fail start" \
					"master0\n");
			goto unmap_io;
		}

		via_spi_printk(KERN_INFO "successfully start VT3410/VT3456 master0\n");		
	}

	/*Register for SPI Interrupt*/
	if (!((via_spi_chip_info->busctl_pci_dev_id == 0x83531106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x34021106) ||
		(via_spi_chip_info->irq == 0))) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
		ret = request_irq(via_spi_chip_info->irq, &via_spi_irq, 0,
				pdev->name, via_spi_chip_info);
#else
		ret = request_irq(via_spi_chip_info->irq, via_spi_irq, 0,
				pdev->name, via_spi_chip_info);
#endif
		if (ret != 0) {
			via_spi_printk(KERN_INFO "via_spi: Cannot claim IRQ\n");
			goto destroy_master;
		 }
	} else {
		for (i = 0; i < via_spi_chip_info->num_master; i++) {
			master_ext = spi_master_get_devdata(
					via_spi_chip_info->master[i]);
			if (master_ext == NULL)
				continue;
			master_ext->support_int = 0;
		}
	}

	/* register the spi master to system */
	if ((via_spi_chip_info->busctl_pci_dev_id == 0x83531106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x34021106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x84101106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)) {
		ret = spi_register_master(via_spi_chip_info->master[0]);
		if (ret != 0) {
			via_spi_printk(KERN_INFO "via_spi:fail to register" \
					"master0\n");
			goto destroy_master;
		}
	} else if (via_spi_chip_info->busctl_pci_dev_id == 0x84091106) {
		ret = spi_register_master(via_spi_chip_info->master[0]);
		if (ret != 0) {
			via_spi_printk(KERN_INFO "via_spi:fail to register" \
					"master0\n");
			goto free_irq;
		}

		ret = spi_register_master(via_spi_chip_info->master[1]);
		if (ret != 0) {
			via_spi_printk(KERN_INFO "via_spi:fail to register" \
					"master1\n");
			/* should unregister the spi master 0 */
			spi_unregister_master(via_spi_chip_info->master[0]);
			goto free_irq;
		}
	}

	via_spi_printk(KERN_INFO "via_spi: via_spi_probe success, add dev\n");

	via_spi_register_device(&m25p80_spi_dev);

	return ret;

free_irq:
	free_irq(via_spi_chip_info->irq, via_spi_chip_info);
destroy_master:
	if ((via_spi_chip_info->busctl_pci_dev_id == 0x83531106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x34021106)) {
		via_spi_master_stop(via_spi_chip_info, &vt3353_master_0);
	} else if (via_spi_chip_info->busctl_pci_dev_id == 0x84091106) {
		via_spi_master_stop(via_spi_chip_info, &vt3409_master_0);
		via_spi_master_stop(via_spi_chip_info, &vt3409_master_1);
	} else if ((via_spi_chip_info->busctl_pci_dev_id == 0x84101106) ||
		   (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)) {
		via_spi_master_stop(via_spi_chip_info, &vt3409_master_0);
	}
unmap_io:
    if (via_spi_chip_info->mmio_regs != NULL)
		iounmap(via_spi_chip_info->mmio_regs);
free_mem_region:
    if (via_spi_chip_info->mmio_ioarea != NULL)
		release_mem_region(via_spi_chip_info->mmio_ioarea->start, 10);
put_master:
    for (i = 0; i < via_spi_chip_info->num_master; i++)
		spi_master_put(via_spi_chip_info->master[i]);
free_mem:
    kfree(via_spi_chip_info);
    return ret;
}


/***********************************************************
*
* Function: via_spi_suspend
* Precondition: paltform device matches the platform driver
* Input: @pdev:  point to a struct platform_device
* Output: int
* Purpose: need do nothing
* Reference: none
*
***********************************************************/
static int via_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
	via_spi_printk(KERN_INFO "into:via_spi_probe\n");
	return 0;
	
}



/***********************************************************
*
* Function: via_spi_resume
* Precondition: paltform device matches the platform driver
* Input: @pdev:  point to a struct platform_device
* Output: int
* Purpose: restore the setting, mainly remove the memory protection
* Reference: none
*
***********************************************************/
static int via_spi_resume(struct platform_device *pdev)
{
	int ret = 0;

	via_spi_printk(KERN_INFO "into:via_spi_resume\n");

	if ((via_spi_chip_info->busctl_pci_dev_id == 0x83531106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x34021106)) {
		ret = via_spi_master_resume(via_spi_chip_info, &vt3353_master_0);
		if (ret != 0) {
			via_spi_printk(KERN_INFO "via_spi:fail resume" \
					"master0\n");
			return ret;
		}
	} else if (via_spi_chip_info->busctl_pci_dev_id == 0x84091106) {
		ret = via_spi_master_resume(via_spi_chip_info, &vt3409_master_0);
		if (ret != 0) {
			via_spi_printk(KERN_INFO "via_spi: fail resume" \
					"master0\n");
			return ret;
		}

		via_spi_printk(KERN_INFO "successfully resume VT3409 master0\n");

		ret = via_spi_master_resume(via_spi_chip_info, &vt3409_master_1);
		if (ret != 0) {
			via_spi_printk(KERN_INFO "via_spi: fail resume " \
					"master1\n");
			return ret;
		}
		via_spi_printk(KERN_INFO "successfully resume VT3409 master1\n");
	} else if ((via_spi_chip_info->busctl_pci_dev_id == 0x84101106) ||
		   (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)) {
		ret = via_spi_master_resume(via_spi_chip_info, &vt3409_master_0);
		if (ret != 0) {
			via_spi_printk(KERN_INFO "via_spi: fail resume" \
					"master0\n");
			return ret;
		}

		via_spi_printk(KERN_INFO "successfully resume VT3410/VT3456 master0\n");		
	}

 	

	via_spi_printk(KERN_INFO "via_spi: via_spi_resume success\n");
	return 0;	
}



/***********************************************************
*
* Function: via_spi_remove
* Precondition: unregister driver
* Input: @pdev: point to a struct platform_device
* Output: int, 0--SUCCESS
* Purpose: unregister SPI master controller, free resource (irq and memory)
*          for SPI controller
* Reference: none
*
***********************************************************/
static int __devexit via_spi_remove(struct platform_device *pdev)
{
	int i;

	via_spi_printk(KERN_INFO "via_spi: enter via_spi_remove routine\n");

	/* free the resouce allocated to each master */
	if ((via_spi_chip_info->busctl_pci_dev_id == 0x83531106) ||
		(via_spi_chip_info->busctl_pci_dev_id == 0x34021106)) {
		via_spi_master_stop(via_spi_chip_info, &vt3353_master_0);
	} else if (via_spi_chip_info->busctl_pci_dev_id == 0x84091106) {
		via_spi_master_stop(via_spi_chip_info, &vt3409_master_0);
		via_spi_master_stop(via_spi_chip_info, &vt3409_master_1);
	} else if ((via_spi_chip_info->busctl_pci_dev_id == 0x84101106) ||
		   (via_spi_chip_info->busctl_pci_dev_id == 0x345B1106)) {
		via_spi_master_stop(via_spi_chip_info, &vt3409_master_0);
	}

	/* unregister the m25p80 device */
	via_spi_unregister_device(&m25p80_spi_dev);

	/* unregister the spi master. for SPI subsystem, a platform device only
	 * represent one spi master, this is not perfect. so when unregister one
	 * spi master by spi_unregister_master, the another master will be also
	 * unregistered by spi_unregister_master. so unregistering one master is
	 * enough
	 */

	spi_unregister_master(via_spi_chip_info->master[0]);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 22)
	if (via_spi_chip_info->busctl_pci_dev_id == 0x84091106)
		spi_unregister_master(via_spi_chip_info->master[1]);
#endif

	/* free the irq */
	if ((via_spi_chip_info->busctl_pci_dev_id != 0x83531106) &&
	    (via_spi_chip_info->busctl_pci_dev_id != 0x34021106) &&
		(via_spi_chip_info->irq != 0))
		free_irq(via_spi_chip_info->irq, via_spi_chip_info);

	/* free the region requested by the controller if needed */
	if (via_spi_chip_info->mmio_regs != NULL)
		iounmap(via_spi_chip_info->mmio_regs);
	if (via_spi_chip_info->mmio_ioarea != NULL)
		release_mem_region(via_spi_chip_info->mmio_ioarea->start, 10);

	platform_set_drvdata(pdev, 0);
	for (i = 0; i < via_spi_chip_info->num_master; i++)
		spi_master_put(via_spi_chip_info->master[i]);

	via_spi_printk(KERN_INFO "begin to free the via_spi_chi_info\n");

	/* free the via_spi_chip_info */
	kfree(via_spi_chip_info);

	if (via_proc_spi) {
		remove_proc_entry(VIA_SPI_NAME, NULL);
		via_proc_spi = NULL;
	}
	return 0;
}


static struct platform_driver via_spi_driver = {
	.probe  = via_spi_probe,
	.remove = __devexit_p(via_spi_remove),
	.suspend	= via_spi_suspend,
	.resume		= via_spi_resume,

	.driver = {
		.name = VIA_SPI_NAME,
		.owner = THIS_MODULE,
	},
};


/***********************************************************
*
* Function: via_spi_init
* Precondition: install driver
* Input: none
* Output: int, 0--SUCCESS
* Purpose: register a platform device and driver to OS,
*                read PCI configuration to get SPI memory map base address,
*                read PCI configuration to get SPI IRQ number
* Reference: none
*
***********************************************************/
static int __init via_spi_init(void)
{
	int ret = 0;
	struct pci_dev *via_busctl_dev = NULL;

	via_spi_printk(KERN_INFO "Welcome to via_spi\n");

	printk(KERN_ERR "the VIA SPI Host Controller Driver %s loading....\n", VIA_SPI_DRV_VERSION);


	busctl_dev_ven_id = 0;
	via_spi_adapter = NULL;
	via_spi_chip_info = NULL;
	via_proc_spi = NULL;

	via_busctl_dev = pci_get_device(PCI_VENDOR_ID_VIA,
		PCI_DEVICE_ID_VIA_SB_VT3353, NULL);
	if (via_busctl_dev != NULL) {
		via_spi_printk(KERN_INFO "via_spi: platform is 353," \
			"supported\n");
		busctl_dev_ven_id = 0x83531106;
	}
	if (via_busctl_dev == NULL) {
		via_busctl_dev = pci_get_device(PCI_VENDOR_ID_VIA,
			PCI_DEVICE_ID_VIA_SB_VT3409, NULL);
		if (via_busctl_dev != NULL) {
			via_spi_printk(KERN_INFO "via_spi:platform is 409," \
				"support\n");
			busctl_dev_ven_id = 0x84091106;
		}
	}
	if (via_busctl_dev == NULL) {
		via_busctl_dev = pci_get_device(PCI_VENDOR_ID_VIA,
			PCI_DEVICE_ID_VIA_SB_VT3410, NULL);
		if (via_busctl_dev != NULL) {
			via_spi_printk(KERN_INFO "platform is 410," 
				"support\n");
			busctl_dev_ven_id = 0x84101106;
		}
	}
	if (via_busctl_dev == NULL) {
		via_busctl_dev = pci_get_device(PCI_VENDOR_ID_VIA,
			PCI_DEVICE_ID_VIA_SB_VT3402, NULL);
		if (via_busctl_dev != NULL) {
			printk(KERN_INFO "platform is 402,support\n");
			busctl_dev_ven_id = 0x34021106;
		}
	}
	if (via_busctl_dev == NULL) {
		via_busctl_dev = pci_get_device(PCI_VENDOR_ID_VIA,
			PCI_DEVICE_ID_VIA_SB_VT3456, NULL);
		if (via_busctl_dev != NULL) {
			printk(KERN_INFO "platform is 456,support\n");
			busctl_dev_ven_id = 0x345B1106;
		}
	}

	if (via_busctl_dev == NULL) {
		via_spi_printk(KERN_INFO "via_spi:platform not supported\n");
		return -ENODEV;
	} else {
		pci_dev_put(via_busctl_dev);
	}

	/* create a platform device object for the SPI controller */
	via_spi_adapter = platform_device_alloc(VIA_SPI_NAME, 0);
	if (!via_spi_adapter) {
		ret = -ENOMEM;
		via_spi_printk(KERN_INFO "via_spi:platform_device_alloc" \
			"failed!\n");
		goto out;
	}

	/* add the platform device to system */
	ret = platform_device_add(via_spi_adapter);
	via_spi_printk(KERN_INFO "via_spi:platform_device_add, ret=%d\n", ret);
	if (ret != 0)
		goto put_dev;

	via_proc_spi = proc_mkdir(VIA_SPI_NAME, NULL);
	if (via_proc_spi == NULL)
		via_spi_printk(KERN_INFO "via_spi:fail to create" \
			"the proc dir\n");


	/* register via_spi_driver to drive the platform device */
	ret = platform_driver_register(&via_spi_driver);
	via_spi_printk(KERN_INFO "via_spi:platform_driver_register, ret=%d\n", ret);
	if (ret == 0) {
		printk(KERN_ERR "VIA SPI Host Controller Driver %s is " \
			"loaded successfully\n", VIA_SPI_DRV_VERSION);
		goto out;
	}

	printk(KERN_ERR "fail to load VIA SPI Host Controller Driver %s\n", VIA_SPI_DRV_VERSION);

	if (via_proc_spi != NULL) {
		remove_proc_entry(VIA_SPI_NAME, NULL);
		via_proc_spi = NULL;
    }

    platform_device_del(via_spi_adapter);
 put_dev:
    platform_device_put(via_spi_adapter);
 out:
    return ret;
}
module_init(via_spi_init);

/***********************************************************
*
* Function: via_spi_exit
* Precondition: unistall driver
* Input: none
* Output: none
* Purpose: unregister a platform device and driver from OS
* Reference: none
*
***********************************************************/
static void __exit via_spi_exit(void)
{
    via_spi_printk(KERN_INFO "Goodbye, via_spi\n");
	printk(KERN_ERR "VIA SPI Host Controller Driver %s is unloaded\n", VIA_SPI_DRV_VERSION);
    platform_driver_unregister(&via_spi_driver);
	platform_device_unregister(via_spi_adapter);
	via_spi_adapter = NULL;
}
module_exit(via_spi_exit);

MODULE_AUTHOR("VIA TECH");
MODULE_DESCRIPTION("VIA SPI driver");
MODULE_VERSION("2.5.1");
MODULE_LICENSE("GPL");
