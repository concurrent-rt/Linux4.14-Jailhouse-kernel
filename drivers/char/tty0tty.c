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


#define DRAM_SHARED_MEM         0x3f200000
#define DRAM_SHARED_MEM_SIZE    0x100000 //place the size here

#define DRAM_SHARED_MEM_SIZE_TX  0xFFE00 // place the size here
#define DRAM_SHARED_MEM_SIZE_RX  0x100   //(DRAM_SHARED_MEM+DRAM_SHARED_MEM_SIZE_TX)
#define DRAM_SHARED_MEM_OFFSET_RX  DRAM_SHARED_MEM_SIZE_TX


#define SYNC_POINTER_OFFSET (DRAM_SHARED_MEM_SIZE_TX+DRAM_SHARED_MEM_SIZE_RX) //0x6000
#define WRITE_PTR_OFFSET SYNC_POINTER_OFFSET 
#define READ_PTR_OFFSET (SYNC_POINTER_OFFSET+4)

static volatile unsigned char *shared_mem;
static volatile unsigned int *shared_mem_tx_ptr =0 ;
static volatile unsigned int *shared_mem_rx_ptr =0 ;
static unsigned int shared_mem_init_done = 0;
static unsigned int local_write_ptr = 0;
static unsigned int local_read_ptr = 0;
static volatile unsigned char *shared_mem_tx;
static volatile unsigned char *shared_mem_rx;


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
/* This function will get invoked when user gives the input to 
   the kernel on the curretn terminal
   
   @tty   - Current terminal port 
   @buf   - Character to push
   @count - Number of character count

*/

static int tpk_write(struct tty_struct *tty,
		const unsigned char *buf, int count)
{
	struct ttyprintk_port *tpkp = tty->driver_data;
	int ret = -EINVAL;
	int i;
	
	if(!tpkp)
		return -ENODEV;	
	
	if(!tpkp->open_count)
		goto exit;

	/* exclusive use of tpk_printk within this tty */
	mutex_lock(&tpkp->port_write_mutex);

	/*For the first time accessing the Shared memory region*/
	if(shared_mem_init_done == 0)
        {
		
           void *shared_virt = ioremap(DRAM_SHARED_MEM, DRAM_SHARED_MEM_SIZE);
           shared_mem = (volatile unsigned char *)shared_virt;
	
	   shared_mem_tx = (char *)shared_virt;
           shared_mem_rx = (char *)(shared_virt+ DRAM_SHARED_MEM_OFFSET_RX);
	
           shared_mem_tx_ptr = (unsigned int *)(shared_virt + WRITE_PTR_OFFSET);

	   shared_mem_rx_ptr = (unsigned int *)(shared_virt+ READ_PTR_OFFSET);		   shared_mem_init_done = 1;	
	}

// Non-Root Cell to Root  Cell print
	{

	int num_char_to_read = 0;
        int shared_mem_current_loc =*(shared_mem_tx_ptr);

	 if (local_read_ptr > shared_mem_current_loc)
        {

                num_char_to_read = (DRAM_SHARED_MEM_SIZE_TX - local_read_ptr)+ shared_mem_current_loc;
        }
        else
        	num_char_to_read = shared_mem_current_loc - local_read_ptr;

	  for(i=0; i<num_char_to_read; i++)
	  {

	    tty_insert_flip_string(&tpkp->port,(char*)(shared_mem_tx + 
						      local_read_ptr), 1);	
	    tty_flip_buffer_push(&tpkp->port);
	    local_read_ptr += 1;
	  
  	   if(local_read_ptr == DRAM_SHARED_MEM_SIZE_TX)
                        local_read_ptr=0;
	  }
	}
	
	/// Root ==> Non-Root Cell

	for (i=0;i<count;i++)
	{

	*(shared_mem_rx + local_write_ptr) = *(buf+i);	
	tty_insert_flip_string(&tpkp->port, (buf+i), 1);
        tty_flip_buffer_push(&tpkp->port);
	local_write_ptr += 1;
	if(local_write_ptr == DRAM_SHARED_MEM_SIZE_RX)
               local_write_ptr = 0;

	*(shared_mem_rx_ptr) = local_write_ptr;
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
	ttyprintk_driver->name = "hyp";
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
