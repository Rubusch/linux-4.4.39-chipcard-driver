# linux-4.4.39-chipcard-driver

TODO list of fotos




# INTRO - What is this?

The code snippet illustrates how generally a chipcard driver via UART can be
implemented.

It...
 + ...is NOT a full fledged driver
 + ...is NOT finalized
 + ...is NOT tested
 + ...is NOT a patch (as it would be common among Linux)
 + ...is just copy/paste source (readability & independence of linux version)

Generally this is a hobbyist approach, which should be able to compile by the
provided instructions.

The source shows the bring up of the chipcard until the interpretation of ATR.
After this step, the communication becomes specific to the chipcards, or in
case of ISO/IEC 7816 will be continued with setting up a PPC message to
configure the communication to the higher baud rate of operation.

I did this with synchronous chipcards and asynchronous chipcards.





# BUILDING THE SOURCE

Download the specific kernel source, e.g. for this setup a Toradex kernel
patched for the VF50 board, via yocto layer.

Setup environment variables for toolchain
(the path where e.g. arm-poky-linux-gnueabi-gcc can be found)
```bash
export PATH=$PATH:/opt/repos/rubusclo/YOCTO/gc__sdk/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi
export KERNEL_DIRECTORY="/opt/repos/rubusclo/YOCTO/gc__kernel/linux-4.4.patched__develop"
```

Force rebuild, by cleaning
```bash
ARCH=arm CROSS_COMPILE=arm-poky-linux-gnueabi- make clean
```

Prepare kernel local version (identification!! )
```bash
cp arch/arm/configs/iso7816_chipcard_defconfig ./.config
sed -i '/CONFIG_LOCALVERSION=/s/.*/CONFIG_LOCALVERSION="-20170607-chipcard05"/' ./.config
```

Just open "menuconfig", then close and save it
[in case make sure the following is enabled
	drivers -> character devices -> serial -> fsl lpuart and iso7816 drivers
when using pwm, then check also the corresponding driver
	drivers -> fsl pwm]
```bash
ARCH=arm CROSS_COMPILE=arm-poky-linux-gnueabi- make menuconfig
```

Compile zImage and dts manually
(or integrate it into your yocto layer generating the target images right away)
```bash
time ARCH=arm CROSS_COMPILE=arm-poky-linux-gnueabi- make zImage vf500-colibri-scmain.dtb
```




# SETTING UP TFTP

I'm running the kernel+dtb via tftp. Alternatively build the regular kernel+dtb
image (fit) and flash it to the target. For the tftp setup run the folling:
```bash
export TFTP_DIRECTORY="/opt/tftpboot"
cp ./arch/arm/boot/zImage ${TFTP_DIRECTORY}/
cp ./arch/arm/boot/dts/*.dtb ${TFTP_DIRECTORY}/dts/
```




# RUNNING THE TARGET
(zImage+dtb via ftp)

Reboot target, stop at uboot, set up the environment.
On the tftp server set up to something, e.g.
```bash
sudo vi /etc/default/tftpd-hpa
#    (...)
    TFTP_DIRECTORY="/opt/repos/rubusclo/YOCTO/gc__kernel/linux-4.4.patched__develop/arch/arm/boot"
#    (...)
```

the kernel was built for ARM, with LOADADDR 0x81000000
```shell
# network:
=> setenv ipaddr 10.12.34.56
=> setenv serverip 10.12.34.55
=> setenv netmask 255.255.252.0
=> setenv gatewayip 10.12.34.1
=> setenv hostname mytftpserver
=> setenv ethaddr 00:11:1a:4b:c3:eb
=> setenv netdev eth0
=> setenv addip 'setenv bootargs ${bootargs} ip=${ipaddr}:${serverip}:${gatewayip}:${netmask}:${hostname}:${netdev}:off panic=1'

# kernel: set address and name of the kernel uImage file
# the kernel here is a FIT image (uImage + FDT)
=> setenv kernel_addr 0x81000000
=> setenv kernel_file zImage
# test with:
# => tftp ${kernel_addr} ${kernel_file}

# dtb:
=> setenv fdt_addr 0x82000000
=> setenv fdt_file dts/vf500-colibri-scmain.dtb
# test
# => tftp ${fdt_addr} ${fdt_file}

## console:
=> setenv console ttyLP0
=> setenv baudrate 115200
=> setenv mtdparts 'mtdparts=vf610_nfc:128k(vf-bcb)ro,1408k(u-boot)ro,512k(u-boot-env),-(ubi)'
=> setenv setup 'setenv setupargs console=tty1 console=${console},${baudrate}n8 consoleblank=0 ${mtdparts}'

## bootstrap environment
=> setenv ubiargs 'ubi.mtd=ubi root=ubi0:rootfs rootfstype=ubifs ubi.fm_autoconvert=1'
=> setenv boot_tftp 'run setup; setenv bootargs ${addip} ${ubiargs} ${setupargs}; ubi part ubi && tftp ${kernel_addr} ${kernel_file} && tftp ${fdt_addr} ${fdt_file}; bootz ${kernel_addr} - ${fdt_addr}'

## store the above setup executing 'saveenv' on the uboot shell

## in future, reboot, stop at uboot, execute
=> run boot_tftp
```




# THEORY

Several types of cards are on the market, ranging from pure memory cards up to
featurefull crypto something, depending on the price. Generally there are
 - Contactless Cards (RFID)
 - Contact Cards - ISO/IEC 7816
 - Multicompound Cards
This project deals with Contact Chipcards.

### Signal Contacts Chipcard:

| left      | right      |
| --------- | ---------- |
| VCC       | GND        |
| RST       | VPP (opt.) |
| CLK       | IO         |
| NC (opt.) | NC (opt.)  |

### 1. Synchronous Contact Cards
The signals are aligned to CLK ticks, one bit corresponds to one tick. Hence no
additional start-, stop- or parity-bits are needed. Synchronous communication
should be denser and thus faster than asynchronous.

+ ISO/IEC 7816-10 Electronic signals and answer to reset for synchronous cards.


### 2. Asynchronous Contact Cards
Most memory cards are asynchronous cards (still 2017). One bit corresponds to
372 CLK ticks. The ticks are rather driving the chipcard processor. Since the
ticks are not clocking communication, the communication works by asynchronous
means, i.e. start bit, parity bit and two wait bits (similar to serial
communication). Communication thus is more fault tolerant than synchronous.

+ ISO/IEC 7816-3 Electronic signals and transmission protocols
+ ISO/IEC 7816-4 Industry commands for interchange


### UARTs with ISO7816 Mode
Some UARTs, e.g. NXP aka Freescale's UART offer a special "ISO7816 Mode". To my
investigation this will mean that the UART is going to detect ATR, and setup
its registers by the information received via the ATR, noteably the baud rate,
signal inversation or LSB/MSB settings. It further should be able to operate
NACK or overflow error handlings and sort of timeouts and corresponding
resends.





# DEVICE TREE SETUP

### Driver used by default:

+ drivers/tty/serial/fsl_lpuart.c


### Which UART is used?

The interface naming varies depending on the documentation:

| [Colibri/Toradex][1] | [VF50/NXP][2] | Linux (Target Setup) |
| -------------------- | ------------- | -------------------- |
| UART_C               | SCI1          | /dev/ttyLP1          |

[1]: http://docs.toradex.com/101355-colibri-vf50-datasheet.pdf
[2]: http://www.nxp.com/docs/en/reference-manual/VFXXXRM.pdf


### GPIOs

In a summary the list of GPIOs boils down to the following table

| SODIMM PIN | Toradex Name | VF50 GPIO | VF50 Signal | Colibri GPIO            | Product IO Name |
| ---------- | ------------ | --------- | ----------- | ----------------------- | --------------- |
| 19         | UART_C_RX    | PTB5      | SCI1_RX     | VF610_PAD_PTB5__GPIO_27 | Chip_EN         |
| 21         | UART_C_TX    | PTB4      | SCI1_TX     | VF610_PAD_PTB4__GPIO_26 | Chip_IO         |
| 81         | UART_C_CTS   | PTB7      | SCI1_CTS    | VF610_PAD_PTB7__GPIO_29 | Chip_CLK        |
|            |              |           |             | alternatively PWM set   |                 |
|            |              |           |             | to this GPIO            |                 |
| 94         | UART_C_RTS   | PTB6      | SCI1_RTS    | VF610_PAD_PTB6__GPIO_28 | Chip_RST        |





# SOURCE

The source is ment to be a code snippet how things can be implemented. It is
NOT a full fledged driver or driver at all. Unlock, Read, Write operations are
missing and highly depending on the particular chipcard. Once achieved an ATR
and being able to send via IO, it becomes easy, though, to implement the
specific read/write operations.





# ABBREVIATIONS

### ATR (Answer to Reset)
Typically a chipcard answers to a single Reset impulse combined with some clock
ticks at very low speed. The ATR then is the answer of the chipcard which
baudrate and communication preferences shall be used for further communication.

### PPC (Protocol and Parameter Change)
Configuration command for asynch chipcards, how to carry on communication.
Initially the baudrate is on a very low level (around 9600). The chipcard then
communicates what it is able to deal with. Typically then a PPC will set up the
communication to the highest possible baudrate.

### T0 and T1 Protocol
In ISO/IEC7816 a serial send of bits (T0) or a blockwise sending are
configurable.

### TPDU (Transmission Protocol Data Unit)
The data frame to be sent over the IO line, the commands are described in the
chipcard's datasheet.

### APDU (Application Protocl Data Unit)
A wrapper over a TPDU frame, e.g. ALPAR is such a protocol when communicating
with NXP's TDA8029 card reader IC, which then talks to the chipcard. APDU's
format are described in e.g. some ALPAR standard or the cardreader's datasheet.





# LINKS

### Theory
[Smartcard basics] (http://www.smartcardbasics.com/smart-card-types.html "Theory about smartcards")
[Wikipedia on Chipcards](https://en.wikipedia.org/wiki/Smart_card "General information on smartcards")
[Wikipedia on ATR](https://en.wikipedia.org/wiki/Answer_to_reset "ATR explained, find more details in the ISO/IEC 7816")
[List of ATR sequences for chipcards](https://eftlab.co.uk/index.php/site-map/knowledge-base/171-atr-list-full "Identify a card type by ATR sequence")

### Tools
[Saleae Logic](https://www.saleae.com/ "A simple logic analyzer")

### Datasheets
[Toradex VF50 Colibri Datasheet](http://docs.toradex.com/101355-colibri-vf50-datasheet.pdf "Toradex VF50 Colibri")
[NXP VFxxx Controller Reference Manual](http://www.nxp.com/docs/en/reference-manual/VFXXXRM.pdf "NXP VF50 Reference Manual")
[chipcard FM4442](http://www.datasheetlib.com/datasheet/1270957/fm4442_shanghai-fudan-microelectronics.html "Fudan 8k bit chipcard")
[chipcard SLE5542](http://www.datasheetlib.com/datasheet/788448/sle5542_infineon-technologies.html "Infenion 8k bit chipcard")
[chipcard FM4428](http://www.datasheetlib.com/datasheet/1270958/fm4428_shanghai-fudan-microelectronics.html "Fudan 1k bit chipcard")
[chipcard SLE5528](http://www.datasheetlib.com/datasheet/788451/sle5528_infineon-technologies.html "Infenion 1k bit chipcard")
[chipcard AT88SC25616C](http://www.datasheetlib.com/datasheet/212456/at88sc25616c_atmel-corporation.html "Atmel various size asynchronous ISO/IEC 7816 chipcard")





# HARDWARE

 - evalboard module: NXP®/Freescale Vybrid VF5xx Computer on Module - Colibri VF50 (ARM Cortex-A5)
 - evalboard: Carrier Board / Single Board Computer (SBC)
or
 - Colibri Evaluation Board v3.2A (soldered card slot to UART_C)
 - Saleae Logic + Software
 - Fudan FM4442 resp. Infenion SLE5542 (synchronous 8kbit chipcard)
 - Fudan FM5528 resp. Infenion SLE5528 (synchronous 1kbit chipcard)
 - Atmel AT88SC25616C (used as asynchronous ISO/IEC 7816 chipcard), tested with several sizes of the AT88SC type





# SOFTWARE

 - Linux Kernel: 4.4.39 modified Toradex (started),
   compilation also tested with vanilla 4.4.80 and vanilla 4.9.41 kernel

