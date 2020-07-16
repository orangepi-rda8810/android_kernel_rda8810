/*
 * Copyright @2014 Buddy Zhangrenze
 */
/*
 * Description
 * The AT24C01A/02/04/08A/16A provides 1024/2048/4096/8196/16384 bits of
 * serial electrically erasable and programmable read-only memory(EEPROM)
 * organized as 128/256/512/1024/2048 words of 8 bit each.The device is
 * optimized for use in many industrial and commercial applications where
 * low-power and low-voltage operation are essential.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/module.h>

#define DEV_NAME "at24c-i2c2"
#define DEBUG            1
#define TEST             0

#define WRITE_PAGE       1
#define WRITE_BYTE       0

#define READ_CURRENT     0
#define READ_RANDOM      0
#define READ_SEQUEN      1

static struct class *i2c_dev_class;
static struct i2c_client *my_client;
static int i2c_major;

#if DEBUG
static void buddy_debug(struct i2c_client *client,char *s)
{
	printk(KERN_INFO "======clinet %s======\n=>client->addr:%08x\n=>client->name:%s\n=>client->adapter:%s\n=>client->flags:%08x\n========================\n",s,client->addr,client->name,client->adapter->name,client->flags);
}
#endif
/*
 * open - file operations
 */
static int buddy_open(struct inode *inode,struct file *filp)
{
#if DEBUG
	printk(KERN_INFO "Buddy open file\n");
#endif
	filp->private_data = my_client;
	return 0;
}
/*
 * close - file operations
 */
static int buddy_release(struct inode *inode,struct file *filp)
{
#if DEBUG
	printk(KERN_INFO "close file\n");
#endif
	filp->private_data = NULL;
	return 0;
}
#if READ_CURRENT
/*
 * CURRENT ADDRESS READ
 * The internal data word address counter maintains the last address 
 * accessed during the last read or write operation,incremented by one.
 * The address stays valid between operations as long as the chip power 
 * maintained.The address "roll over" during read is from the last byte of
 * the last memory page to the first byte of the first page.The address 
 * "roll over" during write is from the last byte if the current page to 
 * the first byte of the same page.
 * Once the deviec address with the read/write select bit set to one is 
 * clocked in and acknowledged by the EEPROM,the current address data 
 * word is serially clocked out.The microntroller does not respond with an
 * input zero but does generate a following stop condition.
 */
static int buddy_read_current(struct i2c_client *client,char *buf)
{
	int ret;
	struct i2c_msg msg[2];

	/* Initial write operation */
	msg[0].addr   = client->addr;
	msg[0].flags  = client->flags | I2C_M_TEN;
	msg[0].buf    = NULL;
	msg[0].len    = 0;
	/* Initial read operation */
	msg[1].addr   = client->addr;
	msg[1].flags  = client->flags | I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].buf    = buf;
	msg[1].len    = 1;
#if TEST
	printk(">>read byte:buf[%s]\n",buf);
#endif 
	ret = i2c_transfer(client->adapter,msg,2);
	if(ret != 2)
	{
#if DEBUG
		printk(KERN_INFO "Unable to Byte Read\n");
#endif
		return 1;
	} else
	{
#if TEST
		printk("Succeed to read byte\n");
#endif
		return 0;
	}
}
#endif
#if READ_RANDOM
/*
 * RANDOM READ
 * A random read requires a "dummy" byte write sequence to load in the data
 * word address.Once the device word and word address are clocked in and
 * acknowledge by the EEPROM,the microcontroller must generate another start
 * condition.The microcontroller now initiates a current address read by 
 * sending a device address with the read/write select bit high.The EEPROM
 * acknowlegdes the device address and serially clocks out the data word.
 * The microcontroller does not respond with a zero but does generate a fol-
 * lowing stop condition.
 */
static int buddy_read_random(struct i2c_client *client,char address,
				char *buf)
{
	int ret;
	struct i2c_msg msg[2];
	/* write dummy */
	msg[0].addr   = client->addr;
	msg[0].flags  = client->flags | I2C_M_TEN;
	msg[0].buf    = &address;
	msg[0].len    = 1;
	/* read initial */
	msg[1].addr   = client->addr;
	msg[1].flags  = client->flags | I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].buf    = buf;
	msg[1].len    = 1;

	ret = i2c_transfer(client->adapter,msg,2);
	if(ret != 2)
	{
#if DEBUG
		printk(KERN_INFO "Unable write data to i2c bus\n");
#endif
		return 1;
	} else
		return 0;
}
#endif
#if READ_SEQUEN
/*
 * SEQUENTIAL READ
 * Sequential reads are initiated by either a current address read or random
 * address read.After the microcontroller recevices a data word,it responds
 * with an acknowledge.As long as the EEPROM receives an acknowledge,it will
 * continue to increment the data word address and serially clock out seque-
 * ntial data words.When the memory address limit is reached,the data word
 * address will "roll over" and the sequential read will continue.The seque-
 * ntial read operation is terminated when the microcontroller does not res-
 * pond with a zero but does generate a following stop condition.
 */
static int buddy_read_sequen(struct i2c_client *client,char address,
				char *buf,int len)
{
	int ret;
	struct i2c_msg msg[2];

	/* dummy write */
	msg[0].addr    = client->addr;
	msg[0].flags   = client->flags | I2C_M_TEN;
	msg[0].buf     = &address;
	msg[0].len     = 1;
	/* read initial */
	msg[1].addr    = client->addr;
	msg[1].flags   = client->flags | I2C_M_TEN;
	msg[1].flags  |= I2C_M_RD;
	msg[1].buf     = buf;
	msg[1].len     = len;

	ret = i2c_transfer(client->adapter,msg,2);
	if(ret != 2)
	{
#if DEBUG
		printk(KERN_INFO "Unable to read data from i2c bus\n");
#endif
		return 1;
	} else
	{
#if TEST
		printk(KERN_INFO "Success to read data from i2c bus\n");
		return 0;
#endif
	}
}
#endif
/*
 * read - file operations
 */
static ssize_t buddy_read(struct file *filp,char __user *buf,size_t count,loff_t *offset)
{
	int ret;
	char *tmp;
	struct i2c_client *client = filp->private_data;

	tmp = kmalloc(count,GFP_KERNEL);
	if(tmp == NULL)
	{
		printk(KERN_INFO "Unable to get more memory\n");
		ret = -ENOMEM;
		goto out;
	}
#if TEST
	printk("Succeed to get memory\n");
#endif 
	/*
	 * Read data from i2c
	 */
#if READ_CURRENT
	ret = buddy_read_current(client,tmp);
#endif
#if READ_RANDOM
	ret = buddy_read_random(client,*offset,tmp);
#endif
#if READ_SEQUEN
	ret = buddy_read_sequen(client,*offset,tmp,count);
#endif
	if(ret == 1)
	{
		ret = -EINVAL;
		goto out_free;
	}
#if TEST
	printk("Succeed to read byte in read\n");
#endif
	/*
	 * Push data to Userspace
	 */
	ret = copy_to_user(buf,tmp,count);
	if(ret != 0)
	{
		ret = -EFAULT;
		goto out_free;
	}
#if TEST
	printk("Succee to push data to userspace\n");
#endif
	kfree(tmp);
	return 0;
out_free:
	kfree(tmp);
out:
	return ret;

}
#if WIRTE_BYTE
/*
 * BYTE WRITE
 * A write operation requires an 8-bit data word address following the 
 * device addree word and acknowledgment.Upon receiot of this address,the
 * EEPROM will again respond with a zero and then clock in the first 8-bit
 * data word.Following receipt of the 8-bit data word.the EEPROM will outp
 * -ut a zero and the addressing device,such as a microcontroller,must 
 * terminate the write sequence with a stop condition.At this time the EEP
 * -ROM enters an internally timed write cycle,t(WR),to the nonvolatile 
 * memory.All inputs are disable during this write cycle and the EEPROM 
 * will not respond until the write is complete.
 */
static int buddy_write_byte(struct i2c_client *client,char address,
		char *buf)
{
	struct i2c_msg msg;
	int ret;
	char data[2];
	data[0] = address;
	data[1] = *buf;
	/* initial the i2c_msg */
	msg.addr   = client->addr;
	msg.flags  = client->flags | I2C_M_TEN;
	msg.buf    = data;
	msg.len    = 2;

	ret = i2c_transfer(client->adapter,&msg,1);
	if(ret != 1)
	{
		printk("Unable byte write to at24c\n");
		return 1;
	}
	else
	{
#if TEST
		printk("Succeed to write to at24c\n");
#endif
		return 0;
	}
}
#endif
#if WRITE_PAGE
/*
 * PAGE WRITE
 * The 1K/2K EEPROM is capable of an 8-byte write,and the 4K,8K and 16K 
 * devices are capable of 16-byte page writes.
 * A page write is initiated the same as a byte write,but the microcontr
 * -oller does not send a stop condition after the first data word is cl
 * -ock in.Instead,after the EEPROM acknowledges receipt of the first data
 * word,the microcontroller can transmit up to seven(1K/2K) or fifteen(4K,
 * 8K,16K) more data words.The EEPROM will respond with a zero after each
 * data word received.The microcontroller must terminate the page write 
 * sequence with stop condition.
 * The data word address lower three(1K/2K) or four(4K,8K,16K)bits are
 * internally incremented following the receipt of each data word.The high
 * -er data word address bits are not incremented ,retaining the memory 
 * page row location.When the word address,internally generated,reaches the
 * page boundary,the following byte is placed at the beginning of same page
 * If more than eight(1K/2K) or sixteen(4K,8K,16K) data words are transmi
 * -tted to the EEPROM,the data word address will "roll over" and previous
 * data will be overwriten.
 */
static int buddy_write_page(struct i2c_client *client,char address,
				char *buf,int len)
{
	int ret;
	struct i2c_msg msg;
	char *tmp = kmalloc((len + 1),GFP_KERNEL);
	tmp[0] = address;
	ret = 1;
	while(ret <= len)
	{
		tmp[ret] = buf[ret-1];
		ret++;
	}
	/* initial write page */
	msg.addr   = client->addr;
	msg.flags  = client->flags | I2C_M_TEN;
	msg.buf    = tmp;
	msg.len    = len+1;

	ret = i2c_transfer(client->adapter,&msg,1);
	if(ret != 1)
	{
		printk(KERN_INFO "Unable to page write\n");
		kfree(tmp);
		return 1;
	} else
	{
		kfree(tmp);
		return 0;
	}
}
#endif
/*
 * write - file operations
 */
static ssize_t buddy_write(struct file *filp,const char __user *buf,size_t count,loff_t *offset)
{
	int ret;
	char *tmp;
	struct i2c_client *client = filp->private_data;
	char addr = (char)*offset;

	tmp = kmalloc(count,GFP_KERNEL);
	if(tmp == NULL)
	{
		ret = -ENOMEM;
		goto out;
	}
#if TEST
	printk("Succeed to get more memory in write\n");
#endif
	/*
	 * Pull data from Userspace
	 */
	ret = copy_from_user(tmp,buf,count);
	if(ret != 0)
	{
		ret = -EINVAL;
		goto out_free;
	}
#if TEST
	printk("Succeed to pull data from Userspace\n");
#endif
	/*
	 * Write to i2c
	 */
#if WRITE_BYTE
	ret = buddy_write_byte(client,addr,tmp);
#endif
#if WRITE_PAGE
	ret = buddy_write_page(client,addr,tmp,count);
#endif
	if(ret == 1)
	{
		ret = -EFAULT;
		goto out_free;
	}
#if TEST
	printk("Succeed to write byte to i2c at write\n");
#endif
	kfree(tmp);
	return 0;
out_free:
	kfree(tmp);
out:
	return ret;

}
/*
 * llseek() - locate new posion.
 */
static loff_t buddy_llseek(struct file *filp,loff_t offset,int when)
{
	struct i2c_client *client = filp->private_data;
	loff_t ret;
	printk(KERN_INFO "=====buddy llseek=====\n");
	switch(when)
	{
		case 0: /* start in head */
			if(offset < 0)
			{
				printk(KERN_INFO "Bad address\n");
				ret = -EINVAL;
				break;
			}
			/* Don't know at24c size */

			filp->f_pos = (unsigned int)offset;
			ret = filp->f_pos;
			break;
		case 1: /* current position */
			if((offset + filp->f_pos) < 0 )
			{
				printk(KERN_INFO "Bad address\n");
				ret = -EINVAL;
				break;
			}
			filp->f_pos += offset;
			ret = filp->f_pos;
			break;
		default:
			ret = -EINVAL;
	}
	return ret;
}
/*
 * struct for file_operations
 */
static const struct file_operations buddy_ops = 
{
	.owner    = THIS_MODULE,
	.llseek   = buddy_llseek,
	.read     = buddy_read,
	.write    = buddy_write,
	.open     = buddy_open,  
	.release  = buddy_release,
};
/*
 * Memory Organization
 * AT24C01A,1K SERIAL EEPROM:
 *		Internally organized with 16 pages of 8 bytes,the 1K requires a 
 *		7-bit data word address for random word addressing.
 * AT24C02,2K SERIAL EEPROM:
 *		Internally organized with 32 pages of 8 bytes each,the 2K requires
 *      an 8-bit data word address for random word addressing.
 * AT24C04,4K SERIAL EEPROM:
 *		Internally organized with 32 pages of 16 bytes each,the 4K requries
 *		a 9-bit data word address for random word addressing.
 * AT24C08A,8K SERIAL EEPROM:
 *		Internally organized with 64 pages of 16 bytes each,the 8K requires
 *		a 10-bit data word address for random word addressing.
 * AT24C16A,16K SERIAL EEPROM:
 *		Internally organized with 128 pages of 16 bytes each,the 16K requir
 *      -es an 11-bit data word address for random word addressing.
 */
/*
 * Device Addressing
 * The 1K,2K,4K,8K and 16K EEPROM devices all require an 8-bit device 
 * address word following a start condition to enable to enable to enable
 * the chip for a read or write operation.
 * The device address word consists of a mandatory one,zero sequence for 
 * the first four most significant bit.This is common to all the EEPROM 
 * device.
 * The next 3 bits are the A2,A1 and A0 device address bits for the 1K/2K
 * EEPROM.These 3 bits must compare to their corresponding hard-writed 
 * input pins.
 * The 4K EEPROM only uses the A2 and A1 device address bits with the third
 * bits being a memory page address bit.The two device address bits must
 * compare to their corresponding hard-writed input pins.The A0 pin is no 
 * connect.
 * The 8K EEPROM only uses the A2 device address bit with the next 2 bits 
 * being for memory page addressing.The A2 bit must compare to its corres
 * ponding hard-writed input pin.The A1 and A0 pins are no connect.
 * The 16K does not use any device address bit but instead the 3 bits are
 * use for memory page addressing.These page addressing bits on the 4K,8K
 * and 16K devices should be considered the most significant bits of the
 * data word address which follows.
 * The eighth bit of the device address is the read/write operation select
 * bit.A read operation is initiated if this bit is high and a write opera
 * -tion is initiated if this bit is low.
 * Upon a compare of the device address,the EEPROM will output a zero.If a
 * compare is not made,the chip will return to a standby state.
 */
/*
 *	i2c id table
 */
static const struct i2c_device_id id_table[] = {
	{"at24c01",0x50},
	{"at24c02",0x50},
	{},
};
/*
 * probe client
 */
static int buddy_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int res;
	struct device *dev;
#if DEBUG
	buddy_debug(my_client,"probe0");
#endif
	my_client = client;
#if DEBUG
	buddy_debug(my_client,"probe1");
#endif
	/*
	 * register this i2c driver into driver core
	 */
	dev = device_create(i2c_dev_class,&my_client->dev,MKDEV(i2c_major,0),NULL,DEV_NAME);
	if(IS_ERR(dev))
	{
		res = PTR_ERR(dev);
		goto error;
	}
	printk(KERN_INFO "Succeed to probe device\n");
	return 0;
error:
	return res;

}
/*
 * remove i2c-driver
 */
static int buddy_remove(struct i2c_client *client)
{
#if DEBUG
	buddy_debug(client,"remove");
#endif
	printk(KERN_INFO "at24c_remove\n");
	device_destroy(i2c_dev_class,MKDEV(i2c_major,0));
	return 0;
}
/*
 * i2c driver
 */
static struct i2c_driver buddy_i2c_driver = {
	.driver = {
		.name = "at24c01",
	},
	.probe   = buddy_probe,
	.remove  = buddy_remove,
	.id_table= id_table,
};
/*
 * init module
 */
static __init  int buddy_init(void)
{
	int ret = 0;
	/*
	 * create i2c board information
	 */
	struct i2c_adapter *adap;
	struct i2c_board_info i2c_info;

	adap = i2c_get_adapter(2);
	if(adap == NULL)
	{
		printk(KERN_INFO "Unable to get i2c bus adapter\n");
		return -EFAULT;
	}
	printk(KERN_INFO "Succeed to get i2c adapter\n");
	memset(&i2c_info,0,sizeof(struct i2c_board_info));
	strlcpy(i2c_info.type,"at24c02",I2C_NAME_SIZE);
	i2c_info.addr = 0x50;

	my_client = i2c_new_device(adap,&i2c_info);
	if(my_client == NULL)
	{
		printk(KERN_INFO "Unable to get at24c client\n");
		i2c_put_adapter(adap);
		return -EFAULT;
	}
	printk(KERN_INFO "Succeed to get client\n");
	i2c_put_adapter(adap);
#if DEBUG
	buddy_debug(my_client,"init");
#endif
	/*
	 * register chardev
	 */
	i2c_major = register_chrdev(0,DEV_NAME,&buddy_ops);
	if(i2c_major == 0)
	{
		printk(KERN_INFO "Unable register chrdev into kernel\n");
		goto out;
	}
	printk(KERN_INFO "Succeed to register chrdev into kernel\n");
#if DEBUG
	printk(KERN_INFO "The char major is %d\n",i2c_major);
#endif
	/*
	 * create class
	 */
	i2c_dev_class = class_create(THIS_MODULE,DEV_NAME);
	if(i2c_dev_class == NULL)
	{
		printk(KERN_INFO "Unable to create class\n");
		goto out_register;
	}
	printk(KERN_INFO "Succeed to create class\n");
	/*
	 * add i2c-driver into i2c core
	 */
	ret = i2c_add_driver(&buddy_i2c_driver);
	if(ret)
	{
		printk(KERN_INFO "Unable to add i2c driver to i2c core\n");
		goto out_class;
	}
	printk(KERN_INFO "Complete to init module\n");
	return 0;
	/*
	 * deal with error
	 */
out_class:
	class_destroy(i2c_dev_class);
out_register:
	unregister_chrdev(i2c_major,DEV_NAME);
out:
	return -EFAULT;
}
/*
 * exit module
 */
static __exit void buddy_exit(void)
{
	printk(KERN_INFO "at24c01 exit\n");
	i2c_del_driver(&buddy_i2c_driver);
	class_destroy(i2c_dev_class);
	unregister_chrdev(i2c_major,DEV_NAME);
}
module_init(buddy_init);
module_exit(buddy_exit);

MODULE_AUTHOR("Buddy Zhang<51498122@qq.com>");
MODULE_DESCRIPTION(DEV_NAME);
MODULE_LICENSE("GPL");
