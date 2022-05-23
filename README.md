# Fan-controll-for-raspberry-pi
---
![Pi with fan]([https://raw.githubusercontent.com/poetter-sebastian/concurrent-priority-list/main/doc/work.jpg](https://github.com/poetter-sebastian/pi-fan-controll/blob/main/picture.png) "Pi3 with PrismWraith")
## Requirements:

### Hardware
* Raspberry Pi with Raspbian (Kernelversion 4.x)
* BME280 Module
* 4 Pin Fan (12v-VCC, GND, RPM, PWM)

### Software 

* Access to the kernel (load and unload kernel modules)

## Setup:

### Install required software
To get access to the kernel you have to do the folowing:
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install git bc bison flex libssl-devsudo apt-get install libncurses5-dev
sudo wget https://raw.githubusercontent.com/notro/rpi-source/master/rpi-source -O /usr/local/bin/rpi-source && sudo chmod +x /usr/local/bin/rpi-source && /usr/local/bin/rpi-source -q –tag-update
sudo rpi-source
```

Now you can view your kernel version (uname -r) and load/unload kernel modules.
```
load kernel module:
sudo insmod {kernel module}*.ko 

unload kernel module:
sudo rmmod {kernel module}*.ko

show all loaded kernel modules:
lsmod

show detailed information about a module:
modeinfo {kernel module}

show kernel log:
dmesg
```
