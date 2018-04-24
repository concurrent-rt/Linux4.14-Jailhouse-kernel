/*
 *  linux/drivers/char/ttyprintk.c
 *
 *  Copyright (C) 2010  Samo Pogacnik
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the smems of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

/*
 * This pseudo device allows user to make printk messages. It is possible
 * to store "console" messages inline with kernel messages for better analyses
 * of the boot process, for example.
 */

#include <linux/device.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/tty_flip.h>

struct ttyprintk_port {
	struct tty_port port;
	struct mutex port_write_mutex;
	int open_count;
};

static struct ttyprintk_port tpk_port;

#define MAPPED_SIZE  0x100000 //place the size here
#define DDR_RAM_PHYS 0x3f200000
#define MAPPED_SIZE_TX   0xFFE00 //place the size here


#define MAPPED_SIZE_RX  0x100//(DDR_RAM_PHYS+MAPPED_SIZE_TX)

#define SYNC_POINTER_OFFSET (MAPPED_SIZE_TX+MAPPED_SIZE_RX) //0xFFF00
#define WRITE_PTR_OFFSET SYNC_POINTER_OFFSET 
#define READ_PTR_OFFSET (SYNC_POINTER_OFFSET+4)

/*
 * Our simple preformatting supports transparent output of (time-stamped)
 * printk messages (also suitable for logging service):
 * - any cr is replaced by nl
 * - adds a ttyprintk source tag in front of each line
 * - too long message is fragmented, with '\'nl between fragments
 * - TPK_STR_SIZE isn't really the write_room limiting factor, because
 *   it is emptied on the fly during preformatting.
 */
#define TPK_STR_SIZE 508 /* should be bigger then max expected line length */
#define TPK_MAX_ROOM 4096 /* we could assume 4K for instance */
static int tpk_curr;

static char tpk_buffer[TPK_STR_SIZE + 4];

static void tpk_flush(void)
{
	if (tpk_curr > 0) {
		tpk_buffer[tpk_curr] = '\0';
		pr_info("[U] %s\n", tpk_buffer);
		tpk_curr = 0;
	}
}

static int tpk_printk(const unsigned char *buf, int count)
{
	int i = tpk_curr;

	if (buf == NULL) {
		tpk_flush();
		return i;
	}

	for (i = 0; i < count; i++) {
		if (tpk_curr >= TPK_STR_SIZE) {
			/* end of tmp buffer reached: cut the message in two */
			tpk_buffer[tpk_curr++] = '\\';
			tpk_flush();
		}

		switch (buf[i]) {
		case '\r':
			tpk_flush();
			if ((i + 1) < count && buf[i + 1] == '\n')
				i++;
			break;
		case '\n':
			tpk_flush();
			break;
		default:
			tpk_buffer[tpk_curr++] = buf[i];
			break;
		}
	}

	return count;
}

/*
 * TTY operations open function.
 */
static int tpk_open(struct tty_struct *tty, struct file *filp)
{
	tty->driver_data = &tpk_port;
	
	if(tpk_port.open_count == 0)
		tpk_port.open_count++;

	return tty_port_open(&tpk_port.port, tty, filp);
}

/*
 * TTY operations close function.
 */
static void tpk_close(struct tty_struct *tty, struct file *filp)
{
	struct ttyprintk_port *tpkp = tty->driver_data;

	mutex_lock(&tpkp->port_write_mutex);
	/* flush tpk_printk buffer */
	tpk_printk(NULL, 0);
	mutex_unlock(&tpkp->port_write_mutex);

	tty_port_close(&tpkp->port, tty, filp);

	if(tpkp->open_count != 0)
		tpkp->open_count =0;
		
}

/*
 * TTY operations write function.
 */
static unsigned int write_ptr =0;
static unsigned int uart_read_ptr =0;
static volatile unsigned int *wr_ptr,*read;

static int tpk_write(struct tty_struct *tty,
		const unsigned char *buf, int count)
{
	struct ttyprintk_port *tpkp = tty->driver_data;
	int ret = -EINVAL;
	unsigned int i;
	static volatile unsigned char *sh_ptr;	
	unsigned int uart_write_ptr;
	if(!tpkp)
		return -ENODEV;	
	
	if(!tpkp->open_count)
		goto exit;

	/* exclusive use of tpk_printk within this tty */
	mutex_lock(&tpkp->port_write_mutex);

	if(write_ptr == 0)
        {
                void *shared_virt = ioremap(DDR_RAM_PHYS, MAPPED_SIZE);
                sh_ptr = (volatile unsigned char *)shared_virt;
		
		void __iomem *shm_ptr = ioremap((DDR_RAM_PHYS+WRITE_PTR_OFFSET),						MAPPED_SIZE_RX);
		wr_ptr =(int *)shm_ptr;
		
		void __iomem *rd_ptr = ioremap((DDR_RAM_PHYS + READ_PTR_OFFSET),								MAPPED_SIZE_RX);
		read = (int *)rd_ptr;
	}
	uart_write_ptr = *wr_ptr;
	printk("\nuart_write_ptr :%d\t local_write_ptr:%d\n",
						uart_write_ptr,write_ptr);
	if(uart_write_ptr > write_ptr)
	{

	  for(i=0;i<((uart_write_ptr-write_ptr));i++)
	  {	
		tty_insert_flip_string(&tpkp->port, 
					   (char*)(sh_ptr + write_ptr), count);	
		tty_flip_buffer_push(&tpkp->port);
		write_ptr += 1;
	  }
	}
	else
	{
		//printk("%c",*buf);
		*(sh_ptr + write_ptr) = *buf;
		tty_insert_flip_string(&tpkp->port, buf, count);
                tty_flip_buffer_push(&tpkp->port);
		write_ptr += 1;
		*read = write_ptr; // nothing but: DDR_RAM_PHYS+READ_PTR_OFFSET
	}


	mutex_unlock(&tpkp->port_write_mutex);

	return count;

exit:
	mutex_unlock(&tpkp->port_write_mutex);
	return ret;
}

/*
 * TTY operations write_room function.
 */
static int tpk_write_room(struct tty_struct *tty)
{
	return TPK_MAX_ROOM;
}

/*
 * TTY operations ioctl function.
 */
static int tpk_ioctl(struct tty_struct *tty,
			unsigned int cmd, unsigned long arg)
{
	struct ttyprintk_port *tpkp = tty->driver_data;

	if (!tpkp)
		return -EINVAL;

	switch (cmd) {
	/* Stop TIOCCONS */
	case TIOCCONS:
		return -EOPNOTSUPP;
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

static const struct tty_operations ttyprintk_ops = {
	.open = tpk_open,
	.close = tpk_close,
	.write = tpk_write,
	.write_room = tpk_write_room,
	.ioctl = tpk_ioctl,
};

static const struct tty_port_operations null_ops = { };

static struct tty_driver *ttyprintk_driver;

static int __init ttyprintk_init(void)
{
	int ret = -ENOMEM;

	mutex_init(&tpk_port.port_write_mutex);

	ttyprintk_driver = tty_alloc_driver(1,
			TTY_DRIVER_RESET_TERMIOS |
			TTY_DRIVER_REAL_RAW |
			TTY_DRIVER_UNNUMBERED_NODE);
	if (IS_ERR(ttyprintk_driver))
		return PTR_ERR(ttyprintk_driver);

	tty_port_init(&tpk_port.port);
	tpk_port.port.ops = &null_ops;

	ttyprintk_driver->driver_name = "ttyprint";
	ttyprintk_driver->name = "arun";
	ttyprintk_driver->major = 240;//TTYAUX_MAJOR;
	ttyprintk_driver->minor_start = 3;
	ttyprintk_driver->type = TTY_DRIVER_TYPE_CONSOLE;
	ttyprintk_driver->init_termios = tty_std_termios;
	ttyprintk_driver->init_termios.c_oflag = OPOST | OCRNL | ONOCR | ONLRET;
	tty_set_operations(ttyprintk_driver, &ttyprintk_ops);
	tty_port_link_device(&tpk_port.port, ttyprintk_driver, 0);

	ret = tty_register_driver(ttyprintk_driver);
	if (ret < 0) {
		printk(KERN_ERR "Couldn't register ttyprintk driver\n");
		goto error;
	}

	return 0;

error:
	put_tty_driver(ttyprintk_driver);
	tty_port_destroy(&tpk_port.port);
	return ret;
}

static void __exit ttyprintk_exit(void)
{
	tty_unregister_driver(ttyprintk_driver);
	put_tty_driver(ttyprintk_driver);
	tty_port_destroy(&tpk_port.port);
}

device_initcall(ttyprintk_init);
module_exit(ttyprintk_exit);

MODULE_LICENSE("GPL");
