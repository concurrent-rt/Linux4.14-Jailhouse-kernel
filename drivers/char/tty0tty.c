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
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

/*
This is used to handle current port to send and receive data
*/
struct ttyprintk_port {
	struct tty_port *port;   /* port */
	struct mutex port_write_mutex;  
};

static struct ttyprintk_port tpk_port;  


#define DRIVER_DESC "Jailhouse virtual console driver"
#define DRIVER_VERSION "v1.0"

#define DRIVER_NAME "ttyprint"
#define DEVICE_NAME  "hyp"
#define MAJOR_NUM    0
#define MINOR_NUM    0
#define NUM_OF_DRIV  8   /*This is to handle number of virual consoles */

#define DRAM_SHARED_MEM         0x3f200000  /* start address of shared memory region */
#define DRAM_SHARED_MEM_SIZE    0x1000 // Maximum memory of shared memory region (4k)

#define DRAM_SHARED_MEM_SIZE_TX  0xF00 //TX buffer size
#define DRAM_SHARED_MEM_SIZE_RX  0xF8  //RX_buffer_sixe
#define DRAM_SHARED_MEM_OFFSET_RX  DRAM_SHARED_MEM_SIZE_TX


#define SYNC_POINTER_OFFSET (DRAM_SHARED_MEM_SIZE_TX+DRAM_SHARED_MEM_SIZE_RX) 
#define WRITE_PTR_OFFSET SYNC_POINTER_OFFSET 
#define READ_PTR_OFFSET (SYNC_POINTER_OFFSET+4)


static volatile unsigned int *shared_mem_tx_ptr[NUM_OF_DRIV] ={0, };
static volatile unsigned int *shared_mem_rx_ptr[NUM_OF_DRIV] ={0, };
static unsigned int shared_mem_init_done[NUM_OF_DRIV] = {0, };
static unsigned int local_write_ptr[NUM_OF_DRIV] = {0, };
static unsigned int local_read_ptr[NUM_OF_DRIV] = {0, };
static volatile unsigned char *shared_mem_tx[NUM_OF_DRIV];
static volatile unsigned char *shared_mem_rx[NUM_OF_DRIV];
static void *shared_virt[NUM_OF_DRIV];
static int device_index[NUM_OF_DRIV];

void periodic_poll_tx(unsigned long data);
int g_time_interval = 1000;
struct timer_list g_timer_shared_mem;

DEFINE_TIMER(g_timer_shared_mem, periodic_poll_tx,0,0);



static struct tty_port **shared_mem_port = 0;

#define TPK_STR_SIZE 508 /* should be bigger then max expected line length */
#define TPK_MAX_ROOM 4096 /* we could assume 4K for instance */


/*
 * TTY operations open function.
 * This function will be invoked with opening device node from minicom.


 */
static int tpk_open(struct tty_struct *tty, struct file *filp)
{
	int index = tty->index;
	int i;
	printk("\ntpk open..\n");
	tty->driver_data = &tpk_port;
	
	for(i=0; i<NUM_OF_DRIV; i++)
	{
		if(i == index)
		  device_index[i] = 1; 
	}
        /*This condition is to initialize shared memory region when we open port for the first time */
	if(shared_mem_init_done[index] == 0)
        {
		
           shared_virt[index] = ioremap((DRAM_SHARED_MEM + 
			(index*DRAM_SHARED_MEM_SIZE)), DRAM_SHARED_MEM_SIZE);

           //shared_mem[index] = (volatile unsigned char *)shared_virt;

           shared_mem_tx[index] = (volatile unsigned char *)shared_virt[index];
           
	   shared_mem_rx[index] = (volatile unsigned char *)(shared_virt[index]+DRAM_SHARED_MEM_OFFSET_RX);
           shared_mem_tx_ptr[index] = (unsigned int *)(shared_virt[index]+WRITE_PTR_OFFSET);

           shared_mem_rx_ptr[index] = (unsigned int *)(shared_virt[index]+ 
							READ_PTR_OFFSET);                  
	   shared_mem_init_done[index] = 1;
        }

	mod_timer(&g_timer_shared_mem, jiffies + msecs_to_jiffies(g_time_interval));
	

	return tty_port_open(&tpk_port.port[index], tty, filp);
}

/*
 * TTY operations close function.
 */
static void tpk_close(struct tty_struct *tty, struct file *filp)
{
	struct ttyprintk_port *tpkp = tty->driver_data;
	int index = tty->index;
	del_timer(&g_timer_shared_mem);
	
	mutex_lock(&tpkp->port_write_mutex);
	
	shared_mem_init_done[index] = 0;
	mutex_unlock(&tpkp->port_write_mutex);

	tty_port_close(&tpkp->port[index], tty, filp);
		
}



/*
 * TTY operations write function.
 */
/* This function will get invoked when user gives the input to 
   the kernel on the current terminal
   
   @tty   - Current terminal port 
   @buf   - Character to push
   @count - Number of character count

*/

static int tpk_write(struct tty_struct *tty,
		const unsigned char *buf, int count)
{
	struct ttyprintk_port *tpkp = tty->driver_data;
	int i;
	int index = tty->index;
	
	
	if(!tpkp)
		return -ENODEV;	
	
	/* exclusive use of tpk_printk within this tty */
	mutex_lock(&tpkp->port_write_mutex);

	/// Root ==> Non-Root Cell

	for (i=0;i<count;i++)
	{

	    *(shared_mem_rx[index] + local_write_ptr[index]) = *(buf+i);	
	    tty_insert_flip_string(&tpkp->port[index], (buf+i), 1);
            tty_flip_buffer_push(&tpkp->port[index]);
	    local_write_ptr[index] += 1;
            /*check if buffer is reached maximum*/    
	    if(local_write_ptr[index] == DRAM_SHARED_MEM_SIZE_RX)
                  local_write_ptr[index] = 0;
              /* This is store number of characters written into shared memory from root cell to non rootcell*/ 
	    *(shared_mem_rx_ptr[index]) = local_write_ptr[index];
	}
	mutex_unlock(&tpkp->port_write_mutex);

	return count;
}

/*
*  This is to read shared memory region from non rootcell to rootcell if there is any data available in shared memory
* region. Its call back function which we will be invoked every 10 seconds to read data.
*/
void periodic_poll_tx(unsigned long data)
{
	int i,index;	
	int num_char_to_read = 0;
        unsigned int shared_mem_current_loc;

	for(index=0; index<NUM_OF_DRIV; index++)
	{
	    if(device_index[index] == 1)
	        shared_mem_current_loc = *(shared_mem_tx_ptr[index]);
	    else
	        continue;
/* Non Root cell ===> Root cell  */

            if (local_read_ptr[index] > shared_mem_current_loc)
            {
                num_char_to_read = (DRAM_SHARED_MEM_SIZE_TX - local_read_ptr[index])+ shared_mem_current_loc;
            }
            else
            {
                num_char_to_read = shared_mem_current_loc-local_read_ptr[index];
            } 
	    for(i=0; i<num_char_to_read; i++)
            {
            	  tty_insert_flip_string(shared_mem_port[index],
		    (char*)((shared_mem_tx[index]) + local_read_ptr[index]), 1);
               	  
		  tty_flip_buffer_push(shared_mem_port[index]);
            		local_read_ptr[index] += 1;
	    	    if(local_read_ptr[index] == DRAM_SHARED_MEM_SIZE_TX)
               	        	 local_read_ptr[index]=0;
            }
	}
	mod_timer(&g_timer_shared_mem, jiffies + msecs_to_jiffies(g_time_interval));
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
	int i;

	mutex_init(&tpk_port.port_write_mutex);

	/*ttyprintk_driver = tty_alloc_driver(NUM_OF_DRIV,
			TTY_DRIVER_RESET_TERMIOS |
			TTY_DRIVER_REAL_RAW );*/

	ttyprintk_driver = alloc_tty_driver(NUM_OF_DRIV);	
	
	if (!ttyprintk_driver)
		return -ENOMEM;

	tpk_port.port = kmalloc(NUM_OF_DRIV*sizeof(struct tty_port), GFP_KERNEL);

	shared_mem_port = (struct tty_port **)kmalloc(NUM_OF_DRIV*
				         sizeof(struct tty_port*), GFP_KERNEL);

	for(i=0; i<NUM_OF_DRIV; i++)
	{
		shared_mem_port[i] = kmalloc(sizeof(struct tty_port),
						GFP_KERNEL);
	}
	ttyprintk_driver->driver_name = DRIVER_NAME;
	ttyprintk_driver->name = DEVICE_NAME;
	ttyprintk_driver->major = MAJOR_NUM;
	ttyprintk_driver->minor_start = MINOR_NUM;
	ttyprintk_driver->type = TTY_DRIVER_TYPE_CONSOLE;
	ttyprintk_driver->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW ;
	ttyprintk_driver->init_termios = tty_std_termios;
	ttyprintk_driver->init_termios.c_oflag = OPOST | OCRNL | ONOCR | ONLRET;
	tty_set_operations(ttyprintk_driver, &ttyprintk_ops);
	
	for(i=0 ;i<NUM_OF_DRIV; i++)
	{
		tty_port_init(&tpk_port.port[i]);
		shared_mem_port[i] = &tpk_port.port[i];

		tpk_port.port[i].ops = &null_ops;
		tty_port_link_device(&tpk_port.port[i], ttyprintk_driver, i);
	
	}
	ret = tty_register_driver(ttyprintk_driver);
	if (ret < 0) {
		printk(KERN_ERR "Couldn't register ttyprintk driver\n");
		goto error;
	}

	printk(KERN_INFO DRIVER_DESC " " DRIVER_VERSION "\n");
	return 0;

error:
	put_tty_driver(ttyprintk_driver);
	for(i=0; i<NUM_OF_DRIV; i++)
	tty_port_destroy(&tpk_port.port[i]);
	return ret;
}

static void __exit ttyprintk_exit(void)
{
	int i;

	for(i=0; i<NUM_OF_DRIV; i++)
	{
		tty_port_destroy(&tpk_port.port[i]);
	}
	kfree(tpk_port.port);
	put_tty_driver(ttyprintk_driver);
	tty_unregister_driver(ttyprintk_driver);
}

device_initcall(ttyprintk_init);
module_exit(ttyprintk_exit);

MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");
