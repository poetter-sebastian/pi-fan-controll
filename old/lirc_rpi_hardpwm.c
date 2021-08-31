Skip to content
Why GitHub? 
Enterprise
Explore 
Marketplace
Pricing 
Search

Sign in
Sign up
1 0 0 Al-/lirc_rpi_hardpwm
 Code  Issues 0  Pull requests 0  Projects 0  Security  Insights
Join GitHub today
GitHub is home to over 40 million developers working together to host and review code, manage projects, and build software together.

lirc_rpi_hardpwm/lirc_rpi.c
@Al- Al- Add files via upload
06fcbe9 on 14 Jan 2017
1035 lines (921 sloc)  29.4 KB
  
/*
 * lirc_rpi.c
 *
 * lirc_rpi - Device driver that records pulse- and pause-lengths
 *	      (space-lengths) (just like the lirc_serial driver does)
 *	      between GPIO interrupt events on the Raspberry Pi.
 *	      Lots of code has been taken from the lirc_serial module,
 *	      so I would like say thanks to the authors.
 *
 * Copyright (C) 2012 Aron Robert Szabo <aron@reon.hu>,
 *		      Michael Bishop <cleverca22@gmail.com>
 * 2017 Andreas Christ <software@quantentunnel.de>
 *      Added hardware pulse-wave modulation
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/timekeeping.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_data/bcm2708.h>

#define LIRC_DRIVER_NAME "lirc_rpi"
#define RBUF_LEN 256
#define LIRC_TRANSMITTER_LATENCY 50

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

#define dprintk(fmt, args...)					\
	do {							\
		if (debug)					\
			printk(KERN_DEBUG LIRC_DRIVER_NAME ": "	\
			       fmt, ## args);			\
	} while (0)

/* for pulse-width modulation */
#define pwm_clock_freq     19200000
/* for low level pin access that the gpio system does not seem to handle;
   copied and modified from
   http://blogsmayan.blogspot.ch/p/programming-interrupts-in-raspberry-pi.html
   and from wiringPi.com */
/* Define Raspberry Pi board */
#ifndef RPI
   #define RPI 3
#endif
/* set base address of RPi peripherals */
#ifndef RPI
   #error "RPI must be #defined as the model number of the raspberry pi"
#elif RPI==0
   #define RASPBERRY_PI_PERI_BASE  0x20000000
#elif RPI==1
   #define RASPBERRY_PI_PERI_BASE  0x20000000
#elif RPI==2
   #define RASPBERRY_PI_PERI_BASE  0x3F000000
#elif RPI==3
   #define RASPBERRY_PI_PERI_BASE  0x3F000000
#else
   #error "RPI must be #defined as integer of the used RPi version (0 - 3)"
#endif
// offsets into RASPBERRY_PI_PERI_BASE
#define CLOCK_OFFSET            0x101000
#define GPIO_OFFSET             0x200000
#define PWM_OFFSET              0x20C000
//#define BLOCK_SIZE 		(4*1024)
/* from  wiringPi/wiringPi.c and .h */
// BCM Magic
#define	BCM_PASSWORD	0x5A000000
/* codes for alternate functions of pins */
#define	FSEL_ALT0		0b100
#define	FSEL_ALT1		0b101
#define	FSEL_ALT2		0b110
#define	FSEL_ALT3		0b111
#define	FSEL_ALT4		0b011
#define	FSEL_ALT5		0b010
//	Word offsets into the PWM control region
#define	PWM_CONTROL 0
#define	PWM_STATUS  1
#define	PWM0_RANGE  4
#define	PWM0_DATA   5
#define	PWM1_RANGE  8
#define	PWM1_DATA   9
//	Clock regsiter offsets
#define	PWMCLK_CNTL	40
#define	PWMCLK_DIV	41
/* values */
#define	PWM_MODE_MS		 0
#define	PWM_MODE_BAL	 1
#define	PWM0_MS_MODE    0x0080  // Run in MS mode
#define	PWM0_USEFIFO    0x0020  // Data from FIFO
#define	PWM0_REVPOLAR   0x0010  // Reverse polarity
#define	PWM0_OFFSTATE   0x0008  // Ouput Off state
#define	PWM0_REPEATFF   0x0004  // Repeat last value if FIFO empty
#define	PWM0_SERIAL     0x0002  // Run in serial mode
#define	PWM0_ENABLE     0x0001  // Channel Enable
#define	PWM1_MS_MODE    0x8000  // Run in MS mode
#define	PWM1_USEFIFO    0x2000  // Data from FIFO
#define	PWM1_REVPOLAR   0x1000  // Reverse polarity
#define	PWM1_OFFSTATE   0x0800  // Ouput Off state
#define	PWM1_REPEATFF   0x0400  // Repeat last value if FIFO empty
#define	PWM1_SERIAL     0x0200  // Run in serial mode
#define	PWM1_ENABLE     0x0100  // Channel Enable
// Locals to hold pointers to the hardware
// gpioToGPFSEL:	Map a BCM_GPIO pin to it's Function Selection control port.
// (GPFSEL 0-5) Groups of 10 - 3 bits per Function - 30 bits per port
static uint8_t gpioToGPFSEL [] = {
  0,0,0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,1,1,
  2,2,2,2,2,2,2,2,2,2,
  3,3,3,3,3,3,3,3,3,3,
  4,4,4,4,4,4,4,4,4,4,
  5,5,5,5,5,5,5,5,5,5,
} ;
// gpioToShift: Define the shift up for the 3 bits per pin in each GPFSEL port
static uint8_t gpioToShift [] = {
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
} ;
// gpioToPwmALT: the ALT value to put a GPIO pin into PWM mode
static uint8_t gpioToPwmALT [] = {
          0,         0,         0,         0,         0,         0, 0, 0,
          0,         0,         0,         0, FSEL_ALT0, FSEL_ALT0, 0, 0,
          0,         0, FSEL_ALT5, FSEL_ALT5,         0,         0, 0, 0,
          0,         0,         0,         0,         0,         0, 0, 0,
          0,         0,         0,         0,         0,         0, 0, 0,
  FSEL_ALT0, FSEL_ALT0,         0,         0,         0, FSEL_ALT0, 0, 0,
          0,         0,         0,         0,         0,         0, 0, 0,
          0,         0,         0,         0,         0,         0, 0, 0,
} ;
// gpioToPwmPort: the port value to put a GPIO pin into PWM mode
static uint8_t gpioToPwmPort [] = {
          0,         0,         0,         0,         0,         0, 0, 0,
          0,         0,         0,         0, PWM0_DATA, PWM1_DATA, 0, 0,
          0,         0, PWM0_DATA, PWM1_DATA,         0,         0, 0, 0,
          0,         0,         0,         0,         0,         0, 0, 0,
          0,         0,         0,         0,         0,         0, 0, 0,
  PWM0_DATA, PWM1_DATA,         0,         0,         0, PWM1_DATA, 0, 0,
          0,         0,         0,         0,         0,         0, 0, 0,
          0,         0,         0,         0,         0,         0, 0, 0,
} ;
/* for ioremap, as per http://blogsmayan.blogspot.ch/p/
   programming-interrupts-in-raspberry-pi.html  */
struct bcm2835_peripheral {
    unsigned long addr_p;
    int mem_fd;
    void *map;
    volatile unsigned int *addr;
};
struct bcm2835_peripheral p_gpio, p_pwm, p_clk;

/* module parameters */

/* set the default GPIO input pin */
static int gpio_in_pin = 18;
/* set the default GPIO output pin */
static int gpio_out_pin = 17;
/* enable debugging messages */
static bool debug;
/* -1 = auto, 0 = active high, 1 = active low */
static int sense = -1;
/* use softcarrier by default */
static bool softcarrier = 1;
/* do not use hardpwm by default */
static bool hardpwm = 0;
/* 0 = do not invert output, 1 = invert output */
static bool invert = 0;

struct gpio_chip *gpiochip;
static int irq_num;
static int auto_sense = 1;

/* forward declarations */
static long send_pulse(unsigned long length);
static void send_space(long length);
static void lirc_rpi_exit(void);

static struct platform_device *lirc_rpi_dev;
static struct timeval lasttv = { 0, 0 };
static struct lirc_buffer rbuf;
static spinlock_t lock;

/* initialized/set in init_timing_params() */
static unsigned int freq = 38000;
static unsigned int duty_cycle = 50;
static unsigned long period;
static unsigned long pulse_width = 0;
static unsigned long space_width;

static void safe_udelay(unsigned long usecs)
{
	while (usecs > MAX_UDELAY_US) {
		udelay(MAX_UDELAY_US);
		usecs -= MAX_UDELAY_US;
	}
	udelay(usecs);
}

static unsigned long read_current_us(void)
{
	struct timespec now;
	getnstimeofday(&now);
	return (now.tv_sec * 1000000) + (now.tv_nsec/1000);
}

static unsigned int best_pulse_width( unsigned int range,
                                      unsigned int new_duty_cycle )
{
   unsigned int too_low, too_high;
   too_low = range * new_duty_cycle / 100;
   if ( too_low == 0 ) return 1;
   too_high = too_low + 1;
   return new_duty_cycle - too_low * 100 / new_duty_cycle
        < too_high * 100 / new_duty_cycle - new_duty_cycle? too_low : too_high;
}

static int init_timing_params(unsigned int new_duty_cycle,
	unsigned int new_freq)
{
   // printk(KERN_INFO LIRC_DRIVER_NAME ": init_timing_params entered\n");
   // printk(KERN_INFO LIRC_DRIVER_NAME ": softcarrier %d\n", softcarrier);
   // printk(KERN_INFO LIRC_DRIVER_NAME ": hardpwm %d\n", hardpwm);
   // printk(KERN_INFO LIRC_DRIVER_NAME ": out_pin %d\n", gpio_out_pin);
   // printk(KERN_INFO LIRC_DRIVER_NAME ": in_pin %d\n", gpio_in_pin);
	if (duty_cycle == new_duty_cycle && freq == new_freq && pulse_width > 0)
	   return 0;	// no changes; pulse_width > 0, thus function run already
	if (hardpwm) {
      unsigned int range;
      uint32_t pwm_control;
      unsigned int divisor;
      divisor = (unsigned int) -1;
      if (new_freq == 0 || new_duty_cycle == 100) { /* unmodulated signal */
         divisor = 500;    /* any divisor and range will do */
         range = 2;
         pulse_width = range;
      } else {
         /* Optimal divisor and range: minimal deviation from new_duty_cycle
            and new_freq; high divisor to keep the stress on the chip low.
            Arbitrary decision: 2.9% deviation from new_freq and 5.9% from
            new_duty_cycle are ok; else minimize sum of the two deviations.*/
         unsigned int error_freq, error_duty, error_sum;
         unsigned int d;
		   printk(KERN_INFO LIRC_DRIVER_NAME
		       ": determine optimal pwm settings for freq %u and duty_cycle %u\n",
		       new_freq, new_duty_cycle);
         error_sum = (unsigned int) -1; // a high number
         for (d = pwm_clock_freq / new_freq / 2 + 1; d >= 1; d--) {
            //dprintk("try d=%u\n", d);
            range = DIV_ROUND_CLOSEST( pwm_clock_freq, new_freq );
            range = DIV_ROUND_CLOSEST( range, d );
            //dprintk("thus range=%u\n", range);
            if (range > 1) {
               error_freq = pwm_clock_freq / d / range; // achieved frequency
               error_freq = error_freq > new_freq ? error_freq - new_freq
                                                  : new_freq - error_freq;
               error_freq = error_freq * 100 / new_freq;       // relative error
               //dprintk("error_freq=%u\n", error_freq);
               pulse_width = best_pulse_width( range, new_duty_cycle );
               error_duty = pulse_width * 100 / range; // achieved duty_cycle
               error_duty = error_duty > new_duty_cycle ? error_duty - new_duty_cycle
                                                        : new_duty_cycle - error_duty;
               error_duty = error_duty * 100 / new_duty_cycle; // relative error
               //dprintk("error_duty=%u\n", error_duty);
               if (error_freq <= 2 && error_duty <= 5 ) { // small errors
                  divisor = d;
                  break;                                  // --> accept
               }
               if ( (error_freq + error_duty) < error_sum ) { // best so far
                  error_sum = error_freq + error_duty;
                  divisor = d;
               }
            }
         }
         range = DIV_ROUND_CLOSEST( pwm_clock_freq, new_freq );
         range = DIV_ROUND_CLOSEST( range, divisor );
         pulse_width = best_pulse_width( range, new_duty_cycle );
         if (pulse_width == 0) pulse_width = 1;
			space_width = invert ? range : 0;
      }
      /* following code in analogy to wiringPi.c */
      pwm_control = *(p_pwm.addr + PWM_CONTROL);		// preserve PWM_CONTROL
      // stop PWM prior to stopping PWM clock in MS mode otherwise BUSY
      // stays high.
      *(p_pwm.addr + PWM_CONTROL) = 0 ;				// Stop PWM
      // Stop PWM clock before changing divisor
      *(p_clk.addr + PWMCLK_CNTL) = BCM_PASSWORD | 0x01 ;	// Stop PWM Clock
      udelay(110) ;			// prevents clock going sloooow
      while ((*(p_clk.addr + PWMCLK_CNTL) & 0x80) != 0)	// Wait to be !BUSY
             udelay(1) ;
      *(p_clk.addr + PWMCLK_DIV)  = BCM_PASSWORD | (divisor << 12);
      *(p_clk.addr + PWMCLK_CNTL) = BCM_PASSWORD | 0x11;	// Start PWM clock
      *(p_pwm.addr + PWM_CONTROL) = pwm_control;		// restore PWM_CONTROL
      *(p_pwm.addr + PWM0_RANGE) = range; udelay(10);
      *(p_pwm.addr + PWM1_RANGE) = range; udelay(10);
		printk(KERN_INFO LIRC_DRIVER_NAME
		       ": set pwm divisor to %u, range to %u, pulse_width to %lu; "
		       "space_width is %lu\n",
              divisor, range, pulse_width, space_width);
		duty_cycle = new_duty_cycle;
		freq = new_freq;
   } else { // i.e., not hardpwm
	if (1000 * 1000000L / new_freq * new_duty_cycle / 100 <=
	    LIRC_TRANSMITTER_LATENCY)
		return -EINVAL;
	if (1000 * 1000000L / new_freq * (100 - new_duty_cycle) / 100 <=
	    LIRC_TRANSMITTER_LATENCY)
		return -EINVAL;
	duty_cycle = new_duty_cycle;
	freq = new_freq;
	period = 1000 * 1000000L / freq;
	pulse_width = period * duty_cycle / 100;
	space_width = period - pulse_width;
	dprintk("in init_timing_params, freq=%d pulse=%ld, "
		"space=%ld\n", freq, pulse_width, space_width);
   }
	return 0;
}

static long send_pulse_softcarrier(unsigned long length)
{
	int flag;
	unsigned long actual, target;
	unsigned long actual_us, initial_us, target_us;

	length *= 1000;

	actual = 0; target = 0; flag = 0;
	actual_us = read_current_us();

	while (actual < length) {
		if (flag) {
			gpiochip->set(gpiochip, gpio_out_pin, invert);
			target += space_width;
		} else {
			gpiochip->set(gpiochip, gpio_out_pin, !invert);
			target += pulse_width;
		}
		initial_us = actual_us;
		target_us = actual_us + (target - actual) / 1000;
		/*
		 * Note - we've checked in ioctl that the pulse/space
		 * widths are big enough so that d is > 0
		 */
		if  ((int)(target_us - actual_us) > 0)
			udelay(target_us - actual_us);
		actual_us = read_current_us();
		actual += (actual_us - initial_us) * 1000;
		flag = !flag;
	}
	return (actual-length) / 1000;
}

static long send_pulse(unsigned long length)
{
	if (length <= 0)
		return 0;

	if (hardpwm) {
      // printk(KERN_INFO LIRC_DRIVER_NAME ": send hardpwm pulse of %lu ms\n", length);
		*(p_pwm.addr + gpioToPwmPort[gpio_out_pin]) = pulse_width;
		safe_udelay(length);
		// *(p_pwm.addr + gpioToPwmPort[gpio_out_pin]) = 0;
		return 0;
   } else
	if (softcarrier) {
		return send_pulse_softcarrier(length);
	} else {
		gpiochip->set(gpiochip, gpio_out_pin, !invert);
		safe_udelay(length);
		return 0;
	}
}

static void send_space(long length)
{
   if (hardpwm) *(p_pwm.addr + gpioToPwmPort[gpio_out_pin]) = space_width;
	else
	gpiochip->set(gpiochip, gpio_out_pin, invert);
	if (length <= 0)
		return;
	safe_udelay(length);
}

static void rbwrite(int l)
{
	if (lirc_buffer_full(&rbuf)) {
		/* no new signals will be accepted */
		dprintk("Buffer overrun\n");
		return;
	}
	lirc_buffer_write(&rbuf, (void *)&l);
}

static void frbwrite(int l)
{
	/* simple noise filter */
	static int pulse, space;
	static unsigned int ptr;

	if (ptr > 0 && (l & PULSE_BIT)) {
		pulse += l & PULSE_MASK;
		if (pulse > 250) {
			rbwrite(space);
			rbwrite(pulse | PULSE_BIT);
			ptr = 0;
			pulse = 0;
		}
		return;
	}
	if (!(l & PULSE_BIT)) {
		if (ptr == 0) {
			if (l > 20000) {
				space = l;
				ptr++;
				return;
			}
		} else {
			if (l > 20000) {
				space += pulse;
				if (space > PULSE_MASK)
					space = PULSE_MASK;
				space += l;
				if (space > PULSE_MASK)
					space = PULSE_MASK;
				pulse = 0;
				return;
			}
			rbwrite(space);
			rbwrite(pulse | PULSE_BIT);
			ptr = 0;
			pulse = 0;
		}
	}
	rbwrite(l);
}

static irqreturn_t irq_handler(int i, void *blah, struct pt_regs *regs)
{
	struct timeval tv;
	long deltv;
	int data;
	int signal;

	/* use the GPIO signal level */
	signal = gpiochip->get(gpiochip, gpio_in_pin);

	if (sense != -1) {
		/* get current time */
		do_gettimeofday(&tv);

		/* calc time since last interrupt in microseconds */
		deltv = tv.tv_sec-lasttv.tv_sec;
		if (tv.tv_sec < lasttv.tv_sec ||
		    (tv.tv_sec == lasttv.tv_sec &&
		     tv.tv_usec < lasttv.tv_usec)) {
			printk(KERN_WARNING LIRC_DRIVER_NAME
			       ": AIEEEE: your clock just jumped backwards\n");
			printk(KERN_WARNING LIRC_DRIVER_NAME
			       ": %d %d %lx %lx %lx %lx\n", signal, sense,
			       tv.tv_sec, lasttv.tv_sec,
			       tv.tv_usec, lasttv.tv_usec);
			data = PULSE_MASK;
		} else if (deltv > 15) {
			data = PULSE_MASK; /* really long time */
			if (!(signal^sense)) {
				/* sanity check */
				printk(KERN_DEBUG LIRC_DRIVER_NAME
				       ": AIEEEE: %d %d %lx %lx %lx %lx\n",
				       signal, sense, tv.tv_sec, lasttv.tv_sec,
				       tv.tv_usec, lasttv.tv_usec);
				/*
				 * detecting pulse while this
				 * MUST be a space!
				 */
				if (auto_sense) {
					sense = sense ? 0 : 1;
				}
			}
		} else {
			data = (int) (deltv*1000000 +
				      (tv.tv_usec - lasttv.tv_usec));
		}
		frbwrite(signal^sense ? data : (data|PULSE_BIT));
		lasttv = tv;
		wake_up_interruptible(&rbuf.wait_poll);
	}

	return IRQ_HANDLED;
}

static int is_right_chip(struct gpio_chip *chip, void *data)
{
	dprintk("is_right_chip %s %d\n", chip->label, strcmp(data, chip->label));

	if (strcmp(data, chip->label) == 0)
		return 1;
	return 0;
}

static inline int read_bool_property(const struct device_node *np,
				     const char *propname,
				     bool *out_value)
{
	u32 value = 0;
	int err = of_property_read_u32(np, propname, &value);
	if (err == 0)
		*out_value = (value != 0);
	return err;
}

static void read_pin_settings(struct device_node *node)
{
	u32 pin;
	int index;

	for (index = 0;
	     of_property_read_u32_index(
		     node,
		     "brcm,pins",
		     index,
		     &pin) == 0;
	     index++) {
		u32 function;
		int err;
		err = of_property_read_u32_index(
			node,
			"brcm,function",
			index,
			&function);
		if (err == 0) {
			if (function == 1) /* Output */
				gpio_out_pin = pin;
			else if (function == 0) /* Input */
				gpio_in_pin = pin;
		}
	}
}

static int init_port(void)
{
	int i, nlow, nhigh;
	struct device_node *node;

   dprintk("init_port entered\n");
   dprintk("softcarrier %d\n", softcarrier);
   dprintk("hardpwm %d\n", hardpwm);
   dprintk("out_pin %d\n", gpio_out_pin);
   dprintk("in_pin %d\n", gpio_in_pin);
   dprintk("debug %d\n", debug);

	node = lirc_rpi_dev->dev.of_node;

	gpiochip = gpiochip_find("bcm2708_gpio", is_right_chip);

	/*
	 * Because of the lack of a setpull function, only support
	 * pinctrl-bcm2835 if using device tree.
	*/
	if (!gpiochip && node)
		gpiochip = gpiochip_find("pinctrl-bcm2835", is_right_chip);

	if (!gpiochip) {
		pr_err(LIRC_DRIVER_NAME ": gpio chip not found!\n");
		return -ENODEV;
	}

	if (node) {
		struct device_node *pins_node;

		pins_node = of_parse_phandle(node, "pinctrl-0", 0);
		if (!pins_node) {
			printk(KERN_ERR LIRC_DRIVER_NAME
			       ": pinctrl settings not found!\n");
			return -EINVAL;
		}

		read_pin_settings(pins_node);

		of_property_read_u32(node, "rpi,sense", &sense);

		read_bool_property(node, "rpi,softcarrier", &softcarrier);

		read_bool_property(node, "rpi,invert", &invert);

		read_bool_property(node, "rpi,debug", &debug);

	} else {
		return -EINVAL;
	}

   printk ( KERN_INFO LIRC_DRIVER_NAME ": setting carrier up, if needed" );
	if (hardpwm == 0)
	gpiochip->set(gpiochip, gpio_out_pin, invert);
   else {  /* hardpwm is requested */
      int fSel, shift, alt;
      /* Map the individual hardware components, as per wiringPi.c, but using
         structure of blog (as this is a kernel module) */
      //	GPIO:
      p_gpio.map = ioremap(RASPBERRY_PI_PERI_BASE + GPIO_OFFSET, BLOCK_SIZE);
      if ( !p_gpio.map ) {
			printk(KERN_ALERT LIRC_DRIVER_NAME ": cant ioremap gpio\n");
			return -ENODEV;
		}
      p_gpio.addr = (volatile unsigned int*)p_gpio.map;
      //	PWM
      p_pwm.map = ioremap(RASPBERRY_PI_PERI_BASE + PWM_OFFSET, BLOCK_SIZE);
      if ( !p_pwm.map ) {
			printk(KERN_ALERT LIRC_DRIVER_NAME ": cant ioremap pwm\n");
			return -ENODEV;
		}
      p_pwm.addr = (volatile unsigned int*)p_pwm.map;
      //	Clock control
      p_clk.map = ioremap(RASPBERRY_PI_PERI_BASE + CLOCK_OFFSET, BLOCK_SIZE);
      if ( !p_clk.map ) {
			printk(KERN_ALERT LIRC_DRIVER_NAME ": cant ioremap clk\n");
			return -ENODEV;
		}
      p_clk.addr = (volatile unsigned int*)p_clk.map;
  		printk(KERN_INFO LIRC_DRIVER_NAME ": ioremap gpio %p, pwm %p, clk %p\n",
             p_gpio.map, p_pwm.map, p_clk.map);
      alt = gpioToPwmALT [gpio_out_pin];
      fSel = gpioToGPFSEL [gpio_out_pin];
      shift = gpioToShift [gpio_out_pin];
      *(p_gpio.addr + fSel) =
                     (*(p_gpio.addr + fSel) & ~(7 << shift)) | (alt << shift);
      udelay(110);
      *(p_pwm.addr + PWM_CONTROL) =
                PWM0_ENABLE | PWM1_ENABLE | PWM0_MS_MODE | PWM1_MS_MODE;
   }

	irq_num = gpiochip->to_irq(gpiochip, gpio_in_pin);
	dprintk("to_irq %d\n", irq_num);

	/* if pin is high, then this must be an active low receiver. */
	if (sense == -1) {
		/* wait 1/2 sec for the power supply */
		msleep(500);

		/*
		 * probe 9 times every 0.04s, collect "votes" for
		 * active high/low
		 */
		nlow = 0;
		nhigh = 0;
		for (i = 0; i < 9; i++) {
			if (gpiochip->get(gpiochip, gpio_in_pin))
				nlow++;
			else
				nhigh++;
			msleep(40);
		}
		sense = (nlow >= nhigh ? 1 : 0);
		printk(KERN_INFO LIRC_DRIVER_NAME
		       ": auto-detected active %s receiver on GPIO pin %d\n",
		       sense ? "low" : "high", gpio_in_pin);
	} else {
		printk(KERN_INFO LIRC_DRIVER_NAME
		       ": manually using active %s receiver on GPIO pin %d\n",
		       sense ? "low" : "high", gpio_in_pin);
		auto_sense = 0;
	}

	return 0;
}

// called when the character device is opened
static int set_use_inc(void *data)
{
	int result;

	/* initialize timestamp */
	do_gettimeofday(&lasttv);

	result = request_irq(irq_num,
			     (irq_handler_t) irq_handler,
			     IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING,
			     LIRC_DRIVER_NAME, (void*) 0);

	switch (result) {
	case -EBUSY:
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": IRQ %d is busy\n",
		       irq_num);
		return -EBUSY;
	case -EINVAL:
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": Bad irq number or handler\n");
		return -EINVAL;
	default:
		dprintk("Interrupt %d obtained\n",
			irq_num);
		break;
	};

	/* initialize pulse/space widths */
	init_timing_params(duty_cycle, freq);

	return 0;
}

static void set_use_dec(void *data)
{
	/* GPIO Pin Falling/Rising Edge Detect Disable */
	irq_set_irq_type(irq_num, 0);
	disable_irq(irq_num);

	free_irq(irq_num, (void *) 0);

	dprintk(KERN_INFO LIRC_DRIVER_NAME
		": freed IRQ %d\n", irq_num);
}

static ssize_t lirc_write(struct file *file, const char *buf,
	size_t n, loff_t *ppos)
{
	int i, count;
	unsigned long flags;
	long delta = 0;
	int *wbuf;

	count = n / sizeof(int);
	if (n % sizeof(int) || count % 2 == 0)
		return -EINVAL;
	wbuf = memdup_user(buf, n);
	if (IS_ERR(wbuf))
		return PTR_ERR(wbuf);
	printk(KERN_INFO LIRC_DRIVER_NAME ": start %d pulses and spaces at %lu",
                                     count, read_current_us());
	spin_lock_irqsave(&lock, flags);

	for (i = 0; i < count; i++) {
		if (i%2)
			send_space(wbuf[i] - delta);
		else
			delta = send_pulse(wbuf[i]);
	}
	if (hardpwm) *(p_pwm.addr + gpioToPwmPort[gpio_out_pin]) = space_width;
	else
	gpiochip->set(gpiochip, gpio_out_pin, invert);

	spin_unlock_irqrestore(&lock, flags);
	printk(KERN_INFO LIRC_DRIVER_NAME ": sequence sent completed at %lu",
	                                  read_current_us() );
	kfree(wbuf);
	return n;
}

static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int result;
	__u32 value;

	switch (cmd) {
	case LIRC_GET_SEND_MODE:
		return -ENOIOCTLCMD;
		break;

	case LIRC_SET_SEND_MODE:
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		/* only LIRC_MODE_PULSE supported */
		if (value != LIRC_MODE_PULSE)
			return -ENOSYS;
		break;

	case LIRC_GET_LENGTH:
		return -ENOSYS;
		break;

	case LIRC_SET_SEND_DUTY_CYCLE:
		dprintk("SET_SEND_DUTY_CYCLE\n");
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		if (value <= 0 || value > 100)
			return -EINVAL;
		return init_timing_params(value, freq);
		break;

	case LIRC_SET_SEND_CARRIER:
		dprintk("SET_SEND_CARRIER\n");
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		if (value > 500000 || value < 20000)
			return -EINVAL;
		return init_timing_params(duty_cycle, value);
		break;

	default:
		return lirc_dev_fop_ioctl(filep, cmd, arg);
	}
	return 0;
}

static const struct file_operations lirc_fops = {
	.owner		= THIS_MODULE,
	.write		= lirc_write,
	.unlocked_ioctl	= lirc_ioctl,
	.read		= lirc_dev_fop_read,
	.poll		= lirc_dev_fop_poll,
	.open		= lirc_dev_fop_open,
	.release	= lirc_dev_fop_close,
	.llseek		= no_llseek,
};

static struct lirc_driver driver = {
	.name		= LIRC_DRIVER_NAME,
	.minor		= -1,
	.code_length	= 1,
	.sample_rate	= 0,
	.data		= NULL,
	.add_to_buf	= NULL,
	.rbuf		= &rbuf,
	.set_use_inc	= set_use_inc,
	.set_use_dec	= set_use_dec,
	.fops		= &lirc_fops,
	.dev		= NULL,
	.owner		= THIS_MODULE,
};

static const struct of_device_id lirc_rpi_of_match[] = {
	{ .compatible = "rpi,lirc-rpi", },
	{},
};
MODULE_DEVICE_TABLE(of, lirc_rpi_of_match);

static struct platform_driver lirc_rpi_driver = {
	.driver = {
		.name   = LIRC_DRIVER_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(lirc_rpi_of_match),
	},
};

static int __init lirc_rpi_init(void)
{
	struct device_node *node;
	int result;

	/* Init read buffer. */
	result = lirc_buffer_init(&rbuf, sizeof(int), RBUF_LEN);
	if (result < 0)
		return -ENOMEM;

	result = platform_driver_register(&lirc_rpi_driver);
	if (result) {
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": lirc register returned %d\n", result);
		goto exit_buffer_free;
	}

	node = of_find_compatible_node(NULL, NULL,
				       lirc_rpi_of_match[0].compatible);

	if (node) {
		/* DT-enabled */
		lirc_rpi_dev = of_find_device_by_node(node);
		WARN_ON(lirc_rpi_dev->dev.of_node != node);
		of_node_put(node);
	}
	else {
		lirc_rpi_dev = platform_device_alloc(LIRC_DRIVER_NAME, 0);
		if (!lirc_rpi_dev) {
			result = -ENOMEM;
			goto exit_driver_unregister;
		}

		result = platform_device_add(lirc_rpi_dev);
		if (result)
			goto exit_device_put;
	}

	return 0;

	exit_device_put:
	platform_device_put(lirc_rpi_dev);

	exit_driver_unregister:
	platform_driver_unregister(&lirc_rpi_driver);

	exit_buffer_free:
	lirc_buffer_free(&rbuf);

	return result;
}

static void lirc_rpi_exit(void)
{
	if (!lirc_rpi_dev->dev.of_node)
		platform_device_unregister(lirc_rpi_dev);
	platform_driver_unregister(&lirc_rpi_driver);
	lirc_buffer_free(&rbuf);

   /* release the mappings for pwm */
   if (p_gpio.addr) iounmap(p_gpio.addr);
   if (p_pwm.addr) iounmap(p_pwm.addr);
   if (p_clk.addr) iounmap(p_clk.addr);
}

static int __init lirc_rpi_init_module(void)
{
	int result;
   dprintk("lirc_rpi_init_module entered\n");
   dprintk("softcarrier %d\n", softcarrier);
   dprintk("hardpwm %d\n", hardpwm);
   dprintk("out_pin %d\n", gpio_out_pin);
   dprintk("in_pin %d\n", gpio_in_pin);
   dprintk("debug %d\n", debug);
	result = lirc_rpi_init();
	if (result)
		return result;

   // check parameters related to hardpwm
   if (hardpwm) {
      if (softcarrier) { 
         result = -EINVAL;
	   	printk(KERN_ERR LIRC_DRIVER_NAME
			       ": softcarrier and hardpwm are mutually exclusive.\n");
		   goto exit_rpi;
      }
      if (gpioToPwmALT[gpio_out_pin] == 0) {
			result = -EINVAL;
			printk(KERN_ERR LIRC_DRIVER_NAME
				       ": GPIO out-pin %d is no pwm pin!\n", gpio_out_pin);
			goto exit_rpi;
      }
   }

	result = init_port();
	if (result < 0)
		goto exit_rpi;

	driver.features = LIRC_CAN_SET_SEND_DUTY_CYCLE |
			  LIRC_CAN_SET_SEND_CARRIER |
			  LIRC_CAN_SEND_PULSE |
			  LIRC_CAN_REC_MODE2;

	driver.dev = &lirc_rpi_dev->dev;
	driver.minor = lirc_register_driver(&driver);

	if (driver.minor < 0) {
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": device registration failed with %d\n", result);
		result = -EIO;
		goto exit_rpi;
	}

	printk(KERN_INFO LIRC_DRIVER_NAME ": driver registered!\n");

	return 0;

	exit_rpi:
	lirc_rpi_exit();

	return result;
}

static void __exit lirc_rpi_exit_module(void)
{
	lirc_unregister_driver(driver.minor);

	gpio_free(gpio_out_pin);
	gpio_free(gpio_in_pin);

	lirc_rpi_exit();

	printk(KERN_INFO LIRC_DRIVER_NAME ": cleaned up module\n");
}

module_init(lirc_rpi_init_module);
module_exit(lirc_rpi_exit_module);

MODULE_DESCRIPTION("Infra-red receiver and blaster driver for Raspberry Pi GPIO.");
MODULE_AUTHOR("Aron Robert Szabo <aron@reon.hu>");
MODULE_AUTHOR("Michael Bishop <cleverca22@gmail.com>");
MODULE_AUTHOR("Andreas Christ <software@quantentunnel.de>");
MODULE_LICENSE("GPL");

module_param(gpio_out_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_out_pin, "GPIO output/transmitter pin number of the BCM"
		 " processor. (default 17");

module_param(gpio_in_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_in_pin, "GPIO input pin number of the BCM processor."
		 " (default 18");

module_param(sense, int, S_IRUGO);
MODULE_PARM_DESC(sense, "Override autodetection of IR receiver circuit"
		 " (0 = active high, 1 = active low )");

module_param(softcarrier, bool, S_IRUGO);
MODULE_PARM_DESC(softcarrier, "Software carrier (0 = off, 1 = on, default on)");

module_param(hardpwm, bool, S_IRUGO);
MODULE_PARM_DESC(hardpwm, "Hardware pulse-width modulation as carrier (0 = off, 1 = on, default off)");

module_param(invert, bool, S_IRUGO);
MODULE_PARM_DESC(invert, "Invert output (0 = off, 1 = on, default off");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");
© 2019 GitHub, Inc.
Terms
Privacy
Security
Status
Help
Contact GitHub
Pricing
API
Training
Blog
About
