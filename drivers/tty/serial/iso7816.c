/*
 * Driver for iso7816 contact chipcards based on gpios and freescale/NXP uarts;
 * core driver
 *
 * Based on tty/serial/fsl_lpuart.c by Freescale Semiconductor, Inc. (NXP)
 * Based on tty/serial/cpm_uart by Kumar Gala, Pantelis Antoniou and Vitaly Bordug
 *
 * Author: Lothar Rubusch <l.rubusch@gmx.ch>,
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

//#if defined(CONFIG_SERIAL_ISO7816_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#if defined(CONFIG_SERIAL_FSL_LPUART_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/delay.h>

#include "iso7816.h"

/* linux constants */
//#define ISO7816_MAJOR         ??? // not specified
//#define ISO7816_MINOR	        1 // start numbering from first defined uart in fdt, here only one
#define DRIVER_NAME             "iso7816"
#define DEV_NAME                "ttyLP"
#define ISO7816_UART_NR         6

/* gpios */
/* - clk by bitbanging */
/* - io is left to uart */
/* - en can be enabled by default (not done) */

#define ISO7816_GPIO_EN         0
#define ISO7816_GPIO_CLK	1
#define ISO7816_GPIO_RST	2
//#define ISO7816_GPIO_IO         3

#define ISO7816_NUM_GPIOS       3



/* debug hacks */

#define DBG_FUNCNAME            if (1 == ccport->port.minor) { printk(KERN_ERR "YYY UART1::%s()\n", __func__); }
#define DBG_FUNCNAME2           if (1 == port->minor) { printk(KERN_ERR "YYY UART1::%s()\n", __func__); }
#define DBG_FUNCNAME3           if (1 == dev->port.minor) { printk(KERN_ERR "YYY UART1::%s()\n", __func__); }
static void printb( const char *funcname, unsigned char input, char *message)
{
	char buf[9];
	unsigned char mask = 1 << (8 - 1);
	char* pbuf = buf;
	memset(buf, '\0', 9);
	while (mask != 0) {
		*pbuf++ = (mask & input ? '1' : '0');
		mask >>= 1;
	}
	printk(KERN_ERR "YYY %s: '%s' = %s", funcname, buf, message);
}




/* structs - initial*/

static bool nodma = false; // currently no dma support

struct iso7816_port {
// TODO 	
	struct uart_port	port;
	struct clk		*clk;
	unsigned int		txfifo_size;
	unsigned int		rxfifo_size;
//	bool			iso781632;
//	bool			iso7816_dma_tx_use;
//	bool			iso7816_dma_rx_use;
//	struct dma_chan		*dma_tx_chan;
//	struct dma_chan		*dma_rx_chan;
//	struct dma_async_tx_descriptor  *dma_tx_desc;
//	struct dma_async_tx_descriptor  *dma_rx_desc;
//	dma_cookie_t		dma_tx_cookie;
//	dma_cookie_t		dma_rx_cookie;
//	unsigned int		dma_tx_bytes;
//	unsigned int		dma_rx_bytes;
//	bool			dma_tx_in_progress;
//	unsigned int		dma_rx_timeout;
	struct timer_list	iso7816_timer;
	struct scatterlist	rx_sgl, tx_sgl[2];
	struct circ_buf		rx_ring;
//	int			rx_dma_rng_buf_len;
//	unsigned int		dma_tx_nents;
//	wait_queue_head_t	dma_wait;
 	unsigned		gpio[ISO7816_NUM_GPIOS];
};

static const struct of_device_id iso7816_dt_ids[] = {
	{
		.compatible = "nxp,iso7816",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, iso7816_dt_ids);




/* functions - initial */

static void iso7816_transmit_buffer(struct iso7816_port *ccport);
static irqreturn_t iso7816_int(int irq, void *dev_id);

static void iso7816_console_putchar(struct uart_port *port, int ch); 
//static void iso7816_cc_command(struct iso7816_port *ccport, const unsigned char *str, unsigned int count); // TODO not used  
static void iso7816_start_tx(struct uart_port *port); 

/**
 * setup watermark
 *
 * reset the uart, i.e. transmitter and receiver, as also flush the FIFOS
 */
static void iso7816_setup_watermark(struct iso7816_port *ccport)
{
	unsigned char val, cr2;
	unsigned char cr2_saved;

	/* unset interrupts and RE, TE */
	cr2 = readb(ccport->port.membase + UARTCR2);
	cr2_saved = cr2;
	cr2 &= ~(UARTCR2_TIE | UARTCR2_TCIE | UARTCR2_TE |
			UARTCR2_RIE | UARTCR2_RE);
	writeb(cr2, ccport->port.membase + UARTCR2);

	/* enable RX and TX FIFO */
	val = readb(ccport->port.membase + UARTPFIFO);
	writeb(val | UARTPFIFO_TXFE | UARTPFIFO_RXFE,
			ccport->port.membase + UARTPFIFO);

	/* flush Tx and Rx FIFO (a MUST when FIFO was enabled) */
	writeb(UARTCFIFO_TXFLUSH | UARTCFIFO_RXFLUSH,
			ccport->port.membase + UARTCFIFO);

	/* explicitly clear RDRF */
	if (readb(ccport->port.membase + UARTSR1) & UARTSR1_RDRF) {
		readb(ccport->port.membase + UARTDR);
		writeb(UARTSFIFO_RXUF, ccport->port.membase + UARTSFIFO);
	}
	writeb(0, ccport->port.membase + UARTTWFIFO);
// TODO needed? (receiver is disabled)	
//	writeb(1, ccport->port.membase + UARTRWFIFO);

	/* Restore cr2 */
	writeb(cr2_saved, ccport->port.membase + UARTCR2);
}


/* cc functions */

static void iso7816_cc_report(struct iso7816_port *ccport)
{
	unsigned int tmp, val, /*baud,*/ sbr/*, brfa*/;
	unsigned char bdh, bdl, sr1, sr2;
	unsigned char cr1, cr2, cr3, cr4/*, cr5*/;
	unsigned char pfifo, cfifo, sfifo, twfifo, tcfifo;
	unsigned char cr7816, ie7816, et7816, is7816;
//	int uartc7816_ttype, uartsr1_or, uartc7816_anack;
	char* marker = "YYY";

        printk(KERN_ERR "%s %s(): Reporting UART Registers\n", __func__, marker);


	/* bdh */
	bdh = readb(ccport->port.membase + UARTBDH);
	printk(KERN_ERR "%s %s(): RXEDGIE is %sset [RxD Input Active Edge Interrupt Enable]\n", __func__, marker, (bdh & 0x40)?"":"not ");
	printb(__func__, (bdh & 0x1f), "sbr (bdh) [0x00......]\n");


	/* bdl */
	bdl = readb(ccport->port.membase + UARTBDL);
	printb(__func__, bdl, "sbr (bdl) [0x........]\n");


	/* sbr */
	sbr = 0;
	sbr |= (bdh&0x1f) <<8;
	sbr |= bdl;
	printk(KERN_ERR "%s %s(): '%d' = sbr\n", __func__, marker, sbr);


	/* cr1 */
	cr1 = readb(ccport->port.membase + UARTCR1);
	printk(KERN_ERR "%s %s(): C1[LOOPS] is set to %s [Loop Mode Select]\n", __func__, marker, (cr1 & 0x80)?"loop mode, transmitter output internally connected to receiver input":"normal operation");
//	printk(KERN_ERR "%s %s(): C1[Reserved] is %sset [0x40]\n", __func__, marker, (cr1 & 0x40)?"":"not ");
	printk(KERN_ERR "%s %s(): C1[RSRC]] is set to %s [Receiver Source Select]\n", __func__, marker, (cr1 & 0x20)?"single wire UART mode":"internal loop back mode");
	printk(KERN_ERR "%s %s(): C1[M] is set to 'start + %s data bits + stop' [9-bit or 8-bit Mode Select]\n", __func__, marker, (cr1 & 0x10)?"9":"8 (normal)");
	printk(KERN_ERR "%s %s(): C1[WAKE] is set to '%s wakeup' [Receiver Wakeup Method Select]\n", __func__, marker, (cr1 & 0x08)?"address mark":"idle line");
	printk(KERN_ERR "%s %s(): C1[ILT] idle character bit count starts after %s bit [Idle Line Type Select]\n", __func__, marker, (cr1 & 0x04)?"stop":"start");
	printk(KERN_ERR "%s %s(): C1[PE] is %sset [Parity Enable]\n", __func__, marker, (cr1 & 0x02)?"":"not ");
	printk(KERN_ERR "%s %s(): C1[PT] is set '%s' [Parity Type]\n", __func__, marker, (cr1 & 0x01)?"odd":"even");


	/* cr2 */
	cr2 = readb(ccport->port.membase + UARTCR2);
	printk(KERN_ERR "%s %s(): c2[TIE] TDRE interrupt %sabled [Transmitter Interrupt or DMA Transfer Enable]\n", __func__, marker, (cr2 & 0x80)?"or DMA transfer requests en":"and DMA transfer requests dis");
	printk(KERN_ERR "%s %s(): c2[TCIE] is %sset [0x40]\n", __func__, marker, (cr2 & 0x40)?"":"not ");
	printk(KERN_ERR "%s %s(): c2[RIE] RDRF interrupt %sabled [Receiver Full Interrupt or DMA Transfer Enable]\n", __func__, marker, (cr2 & 0x20)?"or DMAtransfer requests en":"and DMA transfer requests dis");
	printk(KERN_ERR "%s %s(): c2[ILIE] IDLE interrupt requests %s [Idle Line Interrupt Enable]\n", __func__, marker, (cr2 & 0x10)?"enabled":"disabled");
	printk(KERN_ERR "%s %s(): c2[TE] transmitter is %s [Transmitter Enable]\n", __func__, marker, (cr2 & 0x08)?"enabled":"disabled");
	printk(KERN_ERR "%s %s(): c2[RE] receiver is %s [Receiver Enable]\n", __func__, marker, (cr2 & 0x04)?"enabled":"disabled");
	printk(KERN_ERR "%s %s(): c2[RWU] is set to %s [Receiver Wakeup Control]\n", __func__, marker, (cr2 & 0x02)?"wakeup function enabled and further receiver interrupt requests inhibited":"normal operation");
	printk(KERN_ERR "%s %s(): c2[SBK] is set to %s [Send Break]\n", __func__, marker, (cr2 & 0x01)?"queue break characters to be sent":"normal transmitter operation");


	/* sr1 */
	sr1 = readb(ccport->port.membase + UARTSR1);
	printk(KERN_ERR "%s %s(): S1[TDRE] amount of data in the transmit buffer is %s value indicated by TXFIFO[TXWATER] [Transmit Data Register Empty Flag]\n", __func__, marker, (sr1 & 0x80)?"=<":">");
	printk(KERN_ERR "%s %s(): S1[TC] transmitter is %s [Transmit Complete Flag]\n", __func__, marker, (sr1 & 0x40)?"idle (transmission activity complete)":"active (sending data, a preamble or a break)");
	printk(KERN_ERR "%s %s(): S1[RDRF] number of datawords in the recieve buffer is %s number indicated by RXWATER [Receiver Data REgister Full Flag]\n", __func__, marker, (sr1 & 0x20)?"=>":"<");
	printk(KERN_ERR "%s %s(): S1[IDLE] reciver input %s [Idle Line Flag]\n", __func__, marker, (sr1 & 0x10)?"has become idle or the flag ahs not been clered since it last asserted":"is either active now or has never become active since the IDLE flag was last cleared");
	printk(KERN_ERR "%s %s(): S1[OR] %sreceive overrun detected [Receiver Overrun Flag]\n", __func__, marker, (sr1 & 0x08)?"":"no ");
	printk(KERN_ERR "%s %s(): S1[NF] %snoise detected (for greater receive buffer read manual) [Noise Flag]\n", __func__, marker, (sr1 & 0x04)?"":"no ");
	printk(KERN_ERR "%s %s(): S1[FE] %sframing error detected [Framing Error]\n", __func__, marker, (sr1 & 0x02)?"":"no ");
	printk(KERN_ERR "%s %s(): S1[PF] %sparity error detected [Parity Error Flag]\n", __func__, marker, (sr1 & 0x01)?"":"no ");


        /* sr2 */
	sr2 = readb(ccport->port.membase + UARTSR2);
//	printk(KERN_ERR "%s %s(): S2[Reserved] is %sset [0x80]\n", __func__, marker, (sr2 & 0x80)?"":"not ");
	printk(KERN_ERR "%s %s(): S2[RXEDGIF] %sactive edge on the receive pin has occurred [RxD Pin Active Edge Interrupt Flag]\n", __func__, marker, (sr2 & 0x40)?"":"no ");
	printk(KERN_ERR "%s %s(): S2[MSBF] %s is the first bit that is transmitted after the start bit [Most Significant Bit First]\n", __func__, marker, (sr2 & 0x20)?"MSB (bit8, bit7 or bit6)":"LSB (bit0)");
	printk(KERN_ERR "%s %s(): S2[RXINV] receive data is %sinverted [Receive Data Inversion]\n", __func__, marker, (sr2 & 0x10)?"":"not ");
	printk(KERN_ERR "%s %s(): S2[RWUID] S1[IDLE] is %sset upon detection of an idle character [Receive Wakeup Idle Detect]\n", __func__, marker, (sr2 & 0x08)?"":"not ");
	printk(KERN_ERR "%s %s(): S2[BRK13] break character is %s bits long [Break Transmit Character Length]\n", __func__, marker, (sr2 & 0x04)?"10, 11 or 12":"13 or 14");
//	printk(KERN_ERR "%s %s(): S2[Reserved] is %sset [0x02]\n", __func__, marker, (sr2 & 0x02)?"":"not ");
	printk(KERN_ERR "%s %s(): S2[RAF] UART receiver %s [Receiver Active Flag]\n", __func__, marker, (sr2 & 0x01)?"active, RxD input not idle":"idle/inactive waiting for a start bit");


	/* cr3 */
	cr3 = readb(ccport->port.membase + UARTCR3);
	printk(KERN_ERR "%s %s(): C3[R8] R8 was %sreceived - R8 is the ninth bit [Received Bit 8]\n", __func__, marker, (cr3 & 0x80)?"":"not ");
	printk(KERN_ERR "%s %s(): C3[T8] T8 was %sreceived - T8 is the ninth bit [Transmit Bit 8]\n", __func__, marker, (cr3 & 0x40)?"":"not ");
	printk(KERN_ERR "%s %s(): C3[TXDIR] TXD pin is an %s in single wire mode [Transmitter Pin Data Direction in Single-Wire mode]\n", __func__, marker, (cr3 & 0x20)?"output":"input");
	printk(KERN_ERR "%s %s(): C3[TXINV] Transmit data is %sinverted [Transmit Data Inversion]\n", __func__, marker, (cr3 & 0x10)?"":"not ");
	printk(KERN_ERR "%s %s(): C3[ORIE] OR interrupts are %s [Overrun Error Interrupt Enable]\n", __func__, marker, (cr3 & 0x08)?"enabled":"disabled");
	printk(KERN_ERR "%s %s(): C3[NEIE] NF interrupts are %s [Noise Error Interrupt Enable]\n", __func__, marker, (cr3 & 0x04)?"enabled":"disabled");
	printk(KERN_ERR "%s %s(): C3[FEIE] FE interrupts are %s [Framing Error Interrupt Enable]\n", __func__, marker, (cr3 & 0x02)?"enabled":"disabled");
	printk(KERN_ERR "%s %s(): C3[PEIE] PF interrupts are %s [Parity Error Interrupt Enable]\n", __func__, marker, (cr3 & 0x01)?"enabled":"disabled");


	/* pfifo */
	pfifo = readb(ccport->port.membase + UARTPFIFO);
	printk(KERN_ERR "%s %s(): PFIFO[TXFE] Transmit FIFO is %s [Transmit FIFO Enable]\n", __func__, marker, (pfifo & 0x80)?"enabled, buffer is depth indicated by TXFIFOSIZE":"not enabled, buffer is depth 1");
	tmp = ((pfifo & 0x70) >>4) & 0x07;
	val = ( tmp == 0x0 ? 1 :
		( tmp == 0x1 ? 4 :
		  ( tmp == 0x2 ? 8 :
		    ( tmp == 0x3 ? 16 :
		      ( tmp == 0x4 ? 32 :
			( tmp == 0x5 ? 64 :
			  ( tmp == 0x6 ? 128 : 0 )
				)
			      )
			    )
			  )
			)
		);
	printk(KERN_ERR "%s %s(): PFIFO[TXFIFOSIZE] transmit buffer depth is %d dataword(s) [Transmit FIFO Buffer Depth]\n", __func__, marker, val );
	printk(KERN_ERR "%s %s(): PFIFO[RXFE] Receive FIFO is %senabled [Receive FIFO Enable]\n", __func__, marker, (pfifo & 0x08)?"":"not ");
	tmp = (pfifo & 0x07) & 0x07;
	val = ( tmp == 0x0 ? 1 :
		( tmp == 0x1 ? 4 :
		  ( tmp == 0x2 ? 8 :
		    ( tmp == 0x3 ? 16 :
		      ( tmp == 0x4 ? 32 :
			( tmp == 0x5 ? 64 :
			  ( tmp == 0x6 ? 128 : 0 )
				)
			      )
			    )
			  )
			)
		);
	printk(KERN_ERR "%s %s(): PFIFO[RXFIFOSIZE] receive buffer depth is %d [Receive FIFO Buffer Depth]\n", __func__, marker, val);


	/* cfifo */
	cfifo = readb(ccport->port.membase + UARTCFIFO);
	printk(KERN_ERR "%s %s(): CFIFO[TXFLUSH] %sflush operation occurs in the transmit FIFO/Buffer [Transmit FIFO/Buffer Flush]\n", __func__, marker, (cfifo & 0x80)?"":"no ");
	printk(KERN_ERR "%s %s(): CFIFO[RXFLUSH] %sflush operation occurs in the receive FIFO/Buffer [Receive FIFO/Buffer Flush]\n", __func__, marker, (cfifo & 0x40)?"":"no ");
//	printk(KERN_ERR "%s %s(): CFIFO[Reserved] is %sset [0x38]\n", __func__, marker, (cfifo & 0x38)?"":"not ");
	printk(KERN_ERR "%s %s(): CFIFO[RXOFE] RXOF flag %s an interrupt to the host [Receive FIFO Overrun Interrupt Enable]\n", __func__, marker, (cfifo & 0x04)?"generates":"does not generate");
	printk(KERN_ERR "%s %s(): CFIFO[TXOFE] TXOF flag %s an interrupt to the host [Transmit FIFO Overrun Interrupt Enable]\n", __func__, marker, (cfifo & 0x02)?"generates":"does not generate");
	printk(KERN_ERR "%s %s(): CFIFO[RXUFE] RXUF flag %s an interrupt to the host [Receive FIFO Underflow Interrupt Enable]\n", __func__, marker, (cfifo & 0x01)?"generates":"does not generate");


	/* sfifo */
	sfifo = readb(ccport->port.membase + UARTSFIFO);
	printk(KERN_ERR "%s %s(): SFIFO[TXEMPT] Transmit buffer is %sempty [Transmit Buffer/FIFO Empty]\n", __func__, marker, (sfifo & 0x80)?"":"not ");
	printk(KERN_ERR "%s %s(): SFIFO[RXEMPT] Receive buffer is %sempty [Receive Buffer/FIFO Empty]\n", __func__, marker, (sfifo & 0x40)?"":"not ");
//	printk(KERN_ERR "%s %s(): SFIFO[Reserved] is %sset [0x38]\n", __func__, marker, (sfifo & 0x38)?"":"not ");
	printk(KERN_ERR "%s %s(): SFIFO[RXOF] %sreceive buffer overflow has occurred [Receiver Buffer Overflow Flag]\n", __func__, marker, (sfifo & 0x04)?"":"no ");
	printk(KERN_ERR "%s %s(): SFIFO[TXOF] %stransmit buffer overflow has occurred [Transmitter Buffer Overflow Flag]\n", __func__, marker, (sfifo & 0x02)?"":"no ");
	printk(KERN_ERR "%s %s(): SFIFO[RXUF] %sreceive buffer underflow has occurred [Receiver Buffer Underflow Flag]\n", __func__, marker, (sfifo & 0x01)?"":"no ");


	/* twfifo */
	twfifo = readb(ccport->port.membase + UARTTWFIFO);
	printb(__func__, twfifo, "TWFIFO[TXWATER] [0x........]\n");


	/* tcfifo */
	tcfifo = readb(ccport->port.membase + UARTTCFIFO);
	printb(__func__, tcfifo, "TCFIFO[TXCOUNT] [0x........]\n");


	/* cr7816 */
	cr7816 = readb(ccport->port.membase + UARTCR7816);
//	printk(KERN_ERR "%s %s(): C7816[Reserved] is 0x%02x [0xE0]\n", __func__, marker, (cr7816 & 0xE0));
	printk(KERN_ERR "%s %s(): C7816[ONACK] %sNACK is generated, when the receipt of the data results in an overflow [Generate NACK on Overflow]\n", __func__, marker, (cr7816 & 0x10)?"a ":"no ");
	printk(KERN_ERR "%s %s(): C7816[ANACK] %sNACK is generated, if parity error is detected or an invalid initial character is detected [Generate NACK on Error]\n", __func__, marker, (cr7816 & 0x08)?"":"no ");
	printk(KERN_ERR "%s %s(): C7816[INIT] %sfor initial character [Detect Initial Character]\n", __func__, marker, (cr7816 & 0x04)?"receiver searches ":"normal operation mode, receiver does not seek ");
	printk(KERN_ERR "%s %s(): C7816[TTYPE] %s transfer [Transfer Type]\n", __func__, marker, (cr7816 & 0x02)?"block":"serial");
	printk(KERN_ERR "%s %s(): C7816[ISO_7816E] is %senabled [ISO-7816 Functionality Enabled]\n", __func__, marker, (cr7816 & 0x01)?"":"not ");


	/* ie7816 */
	ie7816 = readb(ccport->port.membase + UARTIE7816);
	printk(KERN_ERR "%s %s(): IE7816[WTE] %s interrupt is generated at the assertion of IS7816[WT] [Wait Timer Interrupt Enable]\n", __func__, marker, (ie7816 & 0x80)?"an":"no");
	printk(KERN_ERR "%s %s(): IE7816[CWTE] %s interrupt is generated at the assertion of IS7816[CWT] [Character Wait Timer Interrupt Enable]\n", __func__, marker, (ie7816 & 0x40)?"an":"no");
	printk(KERN_ERR "%s %s(): IE7816[BWTE] %s interrupt is generated at the assertion of IS7816[BWT] [Block Wait Timer Interrupt Enable]\n", __func__, marker, (ie7816 & 0x20)?"an":"no");
	printk(KERN_ERR "%s %s(): IE7816[INITDE] %s interrupt is generated at the assertion of IS7816[INITD] [Initial Character Detected Interrupt Enable]\n", __func__, marker, (ie7816 & 0x10)?"an":"no");
//	printk(KERN_ERR "%s %s(): IE7816[Reserved] is %sset []\n", __func__, marker, (ie7816 & 0x08)?"":"not ");
	printk(KERN_ERR "%s %s(): IE7816[GTVE] %s interrupt is generated at the assertion of IS7816[GTV] [Guard Timer Violated Interrupt Enable]\n", __func__, marker, (ie7816 & 0x04)?"an":"no");
	printk(KERN_ERR "%s %s(): IE7816[TXTE] %s interrupt is generated at the assertion of IS7816[TXT] [Transmit Threshold Exceeded Interrupt Enable]\n", __func__, marker, (ie7816 & 0x02)?"an":"no");
	printk(KERN_ERR "%s %s(): IE7816[RXTE] %s interrupt is generated at the assertion of IS7816[RXT] [Receive Threshold Exceeded Interrupt Enable]\n", __func__, marker, (ie7816 & 0x01)?"an":"no");


	/* is7816 */
	is7816 = readb(ccport->port.membase + UARTIS7816);
	printk(KERN_ERR "%s %s(): IS7816[WT] Wait time (WT) has %sbeen violated [Wait Timer Interrupt]\n", __func__, marker, (is7816 & 0x80)?"":"not ");
	printk(KERN_ERR "%s %s(): IS7816[CWT] Character wait time (CWT) has %sbeen violated [Character Wait Timer Interrupt]\n", __func__, marker, (is7816 & 0x40)?"":"not ");
	printk(KERN_ERR "%s %s(): IS7816[BWT] Block wait time (BWT) has %sbeen violated [Block Wait Timer Interrupt]\n", __func__, marker, (is7816 & 0x20)?"":"not ");
	printk(KERN_ERR "%s %s(): IS7816[INITD] A valid initial character has %sbeen received [Initial Character Detected Interrupt]\n", __func__, marker, (is7816 & 0x10)?"":"not ");
//	printk(KERN_ERR "%s %s(): IS7816[Reserved] is %sset []\n", __func__, marker, (is7816 & 0x08)?"":"not ");
	printk(KERN_ERR "%s %s(): IS7816[GTV] A guard time (Gt, CGT, or BGT) has %sbeen violated [Guard Timer Violated Interrupt]\n", __func__, marker, (is7816 & 0x04)?"":"not ");
	printk(KERN_ERR "%s %s(): IS7816[TXT] The number of retries and corresponding NACKS %s the value in ET7816[TXTHRESHOLD] [Transmit Threshold Exceeded Interrupt]\n", __func__, marker, (is7816 & 0x02)?"exceeds":"does not exceed");
	printk(KERN_ERR "%s %s(): IS7816[RXT] The number of consecutive NACKS generated as a result of parity errors and buffer overruns is %s the value in ET7816[RXTHRESHOLD] [Receive Threshold Exceeded Interrupt]\n", __func__, marker, (is7816 & 0x01)?"greater than":"less than or equal to");


	/* et7816 */
	et7816 = readb(ccport->port.membase + UARTET7816);
//	printk(KERN_ERR "%s %s(): PT is %sset []\n", __func__, marker, (et7816 & 0x10)?"":"not ");
// TODO  	


	/* cr4 */
	cr4 = readb(ccport->port.membase + UARTCR4);
	printk(KERN_ERR "%s %s(): C4[MAEN1] is %s [Match Address Mode Enable 1]\n", __func__, marker, (cr4 & 0x80)?"set - MUST BE CLEARED WHEN c7816[ISO7816E] IS SET!!!":"not set (normal when ISO7816 enabled)");
	printk(KERN_ERR "%s %s(): C4[MAEN2] is %s [Match Address Mode Enable 2]\n", __func__, marker, (cr4 & 0x40)?"set - MUST BE CLEARED WHEN c7816[ISO7816E] IS SET!!!":"not set (normal when ISO7816 enabled)");
	printk(KERN_ERR "%s %s(): C4[M10] is %s [10-bit Mode select]\n", __func__, marker, (cr4 & 0x20)?"set - MUST BE CLEARED WHEN c7816[ISO7816E] IS SET!!!":"not set (normal when ISO7816 enabled)");
	printk(KERN_ERR "%s %s(): C4[BRFA] is 0x%02x [Baud Rate Fine Adjust]\n", __func__, marker, (cr4 & 0x1F));

//	printb(__func__, cr4, "cr4 [0x000xxxxx]\n");

	printk(KERN_ERR "%s %s(): Reporting UART Registers ~~~ done\n", __func__, marker);
}

/**
 * init gpios as cc function
 *
 * request gpios from the kernel
 *
 * currently defined:
 * en - enable (power on)
 * rst - the reset line
 */
// TODO former function name was iso7816_init_port()    
static int iso7816_cc_init_gpios(struct device_node *np,
				 struct iso7816_port *ccport)
{
	int ret, idx;
	unsigned gpio;
	char* gpionames[] = {"gpios-en", "gpios-clk", "gpios-rst"};
	
	DBG_FUNCNAME;
	

/* driver clock */
// TODO check for "clock" property, and how to set it up (taken from cpm uart), alternatively write a probe() and run ccport->clk = devm_clk_get(&pdev->dev, "ipg"); there      	
//	data = of_get_property(np, "clock", NULL);
//	if (data) {
//		struct clk *clk = clk_get(NULL, (const char*)data);
//		if (!IS_ERR(clk))
//			ccport->clk = clk;
//	}
// TODO further needed at all? or rm    	
//	if (!ccport->clk) {
//		data = of_get_property(np, "fsl,cpm-brg", &len);
//		if (!data || len != 4) {
//			printk(KERN_ERR "ISO7816 %s has no/invalid "
//			                "fsl,??? property.\n", np->name);
//			return -EINVAL;
//		}
//		ccport->brg = *data;
//	}

	/* fifo setup */
// TODO check to do this here or in a separate function iso7816_startup()
//	pinfo->tx_nrfifos = TX_NUM_FIFO;
// 	pinfo->tx_fifosize = TX_BUF_SIZE;
// 	pinfo->rx_nrfifos = RX_NUM_FIFO;
// 	pinfo->rx_fifosize = RX_BUF_SIZE;
// TODO      

	/* ccport->port setup */
// TODO check if this is matter of iso7816_probe() like function  	
// 	pinfo->port.uartclk = ppc_proc_freq;
// 	pinfo->port.mapbase = (unsigned long)mem;
// 	pinfo->port.type = PORT_CPM;
// 	pinfo->port.ops = &cpm_uart_pops,
// 	pinfo->port.iotype = UPIO_MEM;
// 	pinfo->port.fifosize = pinfo->tx_nrfifos * pinfo->tx_fifosize;
// 	spin_lock_init(&pinfo->port.lock);
// 	pinfo->port.irq = irq_of_parse_and_map(np, 0);
// 	if (pinfo->port.irq == NO_IRQ) {
// 		ret = -EINVAL;
// 		goto out_pram;
// 	}
// TODO incl. uartclk = ppc_proc_freq	  

	// init gpio values
	// TODO is this needed, or is there a better solution?
	ccport->gpio[ISO7816_GPIO_EN] = 27; // enable
	ccport->gpio[ISO7816_GPIO_CLK] = 29; // gpio clock
	ccport->gpio[ISO7816_GPIO_RST] = 28; // reset
//	ccport->gpio[ISO7816_GPIO_IO] = 26; // io   

	printk(KERN_ERR "YYY %s(): starting loop for gpios... \n", __func__);   
//*
        for (idx = 0; idx < ISO7816_NUM_GPIOS; ++idx) {
printk(KERN_ERR "YYY %s(): %d. gpio\n", __func__, idx);   		
//		ccport->gpio[idx] = -1; // TODO rm
//		gpio = of_get_gpio( np, idx); // TODO check or rm
		gpio = ccport->gpio[idx];

		if (gpio_is_valid(gpio)) {
			ret = gpio_request( gpio, gpionames[idx]); // TODO check if this is valid                                                 
			if (ret) {
				pr_err( "ISO7816: can't request gpio #%d: %d\n", idx, ret);
				continue; // TODO: alternatively cancel here...
			}

			// all gpios are outgoing, initial to L
			ret = gpio_direction_output( gpio, 0);
			if (ret) {
				pr_err( "ISO7816: can't set direction for gpio #%d: %d\n",
					idx, ret);
				gpio_free(gpio);
				continue; // TODO: alternatively cancel here...
			}

			if (0 > gpio) {
				printk(KERN_ERR "ISO7816: gpio FAILED +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
				continue;
			}
			gpio_export( gpio, true );
			ccport->gpio[idx] = gpio;
		}
	}

	printk(KERN_ERR "YYY %s(): gpio - done\n", __func__);

	ret = 0;
	return ret;
}

/**
 * cc function: initialize
 */
static int iso7816_cc_initialize(struct iso7816_port *ccport){
	unsigned int baud, sbr, brfa;
	unsigned char cr1, cr2, cr3, cr4, cr5, cr7816, ie7816, et7816, bdh, bdl, sr2, temp;
//	unsigned char pfifo, cfifo;
	int uartc7816_ttype, uartsr1_or, uartc7816_anack;

	if (1 != ccport->port.minor) {
		printk(KERN_ERR "YYY %s(): return - not called by UART1 (UART%d)\n", __func__, ccport->port.minor);
		return 0;
	}
	printk(KERN_ERR "YYY %s() YYY\n", __func__);

	
//printk(KERN_ERR "YYY %s(): return - DISABLED!\n", __func__);
//return 0;
	

// TODO iso7816.c has no iso781632	
//	if (ccport->iso781632) {
//		/* NOTE: this is not valid for 32-bit versions! */
//		printk(KERN_ERR "ISO7816: %s() does not work for 32-bit architecture\n", __func__);
//		return -1;
//	}

	/* preset TTYPE behaviour - according to ISO-7816 init in datasheet */
	uartc7816_ttype = 0;

	/* preset OR behaviour - read out overrun register */
	temp = readb(ccport->port.membase + UARTSR1);
	uartsr1_or = temp & UARTSR1_OR ? 1 : 0;

	/* preset anack behaviour */
	uartc7816_anack = uartsr1_or;

	/* turn off dma */
	nodma = true; // YYY TODO experimental               

	/* 1. */

	/* start initialization sequence */
	printk(KERN_ERR "YYY %s: 1. select a baud rate\n", __func__);
	/* note: ccport->port.uartclk == 66000000 */
	printk(KERN_ERR "YYY %s: '%d' = port->uartclk\n", __func__, ccport->port.uartclk);
	baud = ccport->port.uartclk / 372;
	printk(KERN_ERR "YYY %s: '%d' = baud\n", __func__, baud);
	if (5000000 <= baud) {
		printk(KERN_ERR "YYY %s: ERROR! baud rate >= 5MHz\n", __func__);
		return -1;
	}

	/* the following is copy-paste-ed from uart_set_termios(), */
	/* since the processing of the selected baud rate should be */
	/* similar (assumption?) */
	bdh = readb(ccport->port.membase + UARTBDH);
//	bdh |= UARTBDH_RXEDGIE; /* set rx input active edge interrupt enable */ //FIXME: NOISE, too many interrupts issue   
	bdh &= ~UARTBDH_SBR; /* reset baud bits */


	
//	sbr = ccport->port.uartclk / (16 * baud);
//	sbr = baud; FIXME


//	sbr = 8191;
//	sbr = 5795;

//	sbr = 66;
	sbr = 133; //YYY    

//	sbr = 17;
	


	printk(KERN_ERR "YYY %s: '%d' = sbr\n", __func__, sbr);
	printb(__func__, ((sbr >> 8) & UARTBDH_SBR),   "\tsbr (high) [0x000xxxxx]\n");
	printb(__func__, (sbr & UARTBDL_SBR), "\tsbr (low)  [0xxxxxxxxx]\n");
	bdh |= ((sbr >> 8) & UARTBDH_SBR);

	/* CR4 - clear MAEN1, MAEN2 and 10-bit mode */
	cr4 &= 0x00;


	
	/* baud rate fine adjust */
//	brfa |= ((ccport->port.uartclk - (16 * sbr * baud)) * 2) / baud;
	


	brfa = 0x0;
	cr4 |= (brfa & UARTCR4_BRFA); /* MAEN1, MAEN2 and 10-bit Mode select (M10) must be cleared for ISO7816, thus mask 0x1F */
	writeb(cr4, ccport->port.membase + UARTCR4);

	cr4 = readb(ccport->port.membase + UARTCR4);
	printb(__func__, cr4, "cr4 [0x000xxxxx]\n");


	/* BDH - first set baud rate (high bits) */
	writeb(bdh, ccport->port.membase + UARTBDH);

	bdh = readb(ccport->port.membase + UARTBDH);
	printb(__func__, bdh, "bdh [0x000xxxxx]\n");


	/* BDL - then set baud rate (low bits) */
	bdl = 0x00;
	bdl = (sbr & UARTBDL_SBR);
	writeb(bdl, ccport->port.membase + UARTBDL);

	bdl = readb(ccport->port.membase + UARTBDL);
	printb(__func__, bdl, "bdl [0xxxxxxxxx], not 0, now bdh and bdl are set\n");



	/* 2. */

	printk(KERN_ERR "YYY %s: 2. configure word length, parity and other configuration fields (LOOPS, RSRC)\n", __func__);
	/* set C1[M] = 1, C1[PE] = 1 and C1[PT] = 0 */
	cr1 = readb(ccport->port.membase + UARTCR1);
	cr1 &= ~UARTCR1_LOOPS;         /* no loop back, and */
	cr1 &= ~UARTCR1_RSRC;          /* set receiver source select (enable rsrc needs LOOPS to be enabled) */
	cr1 |= UARTCR1_M;              /* set 9-bit mode */
	cr1 |= UARTCR1_PE;             /* set parity */
	cr1 &= ~UARTCR1_PT;            /* set parity to even */
	writeb(cr1, ccport->port.membase + UARTCR1);

	cr1 = readb(ccport->port.membase + UARTCR1);
	printb(__func__, cr1, "cr1, [0x0R01xx10]\n");


	/* 3. */

	printk(KERN_ERR "YYY %s: 3. set S2[RWUID] = 0 (ISO7816 requirement)\n", __func__);
	sr2 = readb(ccport->port.membase + UARTSR2);
	sr2 &= ~UARTSR2_RWUID;
	writeb(sr2, ccport->port.membase + UARTSR2);

	sr2 = readb(ccport->port.membase + UARTSR2);
	printb(__func__, sr2, "sr2, [0xRxxx0xRx]\n");


	/* 4. */

	printk(KERN_ERR "YYY %s: 4. set C3 - interrupt enable fields (values as desired)\n", __func__);
	cr3 = readb(ccport->port.membase + UARTCR3);
	cr3 &= ~UARTCR3_ORIE;          /* no overrun error interrupt enable */
	cr3 &= ~UARTCR3_NEIE;          /* no noise error interrupt enable */
	cr3 &= ~UARTCR3_FEIE;          /* no framing error interrupt enable */
	cr3 &= ~UARTCR3_PEIE;          /* no parity error interrupt enable */
	writeb(cr3, ccport->port.membase + UARTCR3);

	cr3 = readb(ccport->port.membase + UARTCR3);
	printb(__func__, cr3, "cr3, [0x00xxxxxx], as desired\n");


	/* 5. */

	printk(KERN_ERR "YYY %s: 5. set C4 - unset match address enable's (ISO7816 requirement)\n", __func__);
	cr4 = readb(ccport->port.membase + UARTCR4);
	cr4 &= ~UARTCR4_MAEN1; /* match address mode enable 1, must be cleared when C7816[ISO7816E] is set */
	cr4 &= ~UARTCR4_MAEN2; /* match address mode enable 2, must be cleared when C7816[ISO7816E] is set */
	cr4 &= ~UARTCR4_M10;   /* 10-bit mode select, must be cleared when C7816[ISO7816E] is set */
	writeb(cr4, ccport->port.membase + UARTCR4);

	cr4 = readb(ccport->port.membase + UARTCR4);
	printb(__func__, cr4, "cr4, [0x000xxxxx], brfa as above\n");


	/* 6. */

	printk(KERN_ERR "YYY %s: 6. set C5 - dma control depends on entry in port structure of driver\n", __func__);
	cr5 = readb( ccport->port.membase + UARTCR5);
	if (!nodma) {
		cr5 |= UARTCR5_TDMAS;  /* turn on DMA for transmit */
		cr5 |= UARTCR5_RDMAS;  /* turn on DMA for receive */
	} else {
		cr5 &= ~UARTCR5_TDMAS; /* turn off DMA for transmit (?) */
		cr5 &= ~UARTCR5_RDMAS; /* turn off DMA for receive (?) */
	}
	writeb(cr5, ccport->port.membase + UARTCR5);

	cr5 = readb( ccport->port.membase + UARTCR5);
	printb(__func__, cr5, "cr5, [0x10100000], dma - undocumented...\n");
//	printk(KERN_ERR "YYY %s: \t'%s' = ccport->iso7816_dma_tx_use\n", __func__, ccport->iso7816_dma_tx_use ? "true" : "false");
//	printk(KERN_ERR "YYY %s: \t'%s' = ccport->iso7816_dma_rx_use\n", __func__, ccport->iso7816_dma_rx_use ? "true" : "false");


	/* 7. */

	printk(KERN_ERR "YYY %s: 7. set C7816[ISO_7816E]=1\n", __func__);
	cr7816 = readb( ccport->port.membase + UARTCR7816);
	if (uartsr1_or) {
		cr7816 |= UARTCR7816_ONACK;     /* generate NACK on overflow (?) */
		cr7816 |= UARTCR7816_ANACK;     /* generate NACK on error (?) */
	} else {
		cr7816 &= ~UARTCR7816_ONACK;     /* generate NACK on overflow (?) */
		cr7816 &= ~UARTCR7816_ANACK;     /* generate NACK on error (?) */
	}
	cr7816 |= UARTCR7816_INIT;      /* detect initial character - normal operation mode */
	if (uartc7816_ttype) {
		/* transfer type (blocks i.e. 1) */
		cr7816 |=  UARTCR7816_TTYPE;
	} else {
		/* or as stream (i.e. 0) */
		cr7816 &= ~UARTCR7816_TTYPE;
	}
	                                                                                              
	
// XXX
// TODO for DEBUGGING turned OFF
	cr7816 |= UARTC7816_ISO_7816E; /* ISO-7816 functionality enabled */
	writeb(cr7816, ccport->port.membase + UARTCR7816);
// XXX
	
                                                                                                      
	cr7816 = readb( ccport->port.membase + UARTCR7816);
	printb(__func__, cr7816, "cr7816, [0xRRR11101]\n");


	/* 8. */

	printk(KERN_ERR "YYY %s: 8. set IE7816 (interrupt enable)\n", __func__);
	ie7816 = readb( ccport->port.membase + UARTIE7816);
	ie7816 |= UARTIE7816_WTE;      /* wait timer interrupt enable */
	ie7816 |= UARTIE7816_CWTE;     /* character wait timer interrupt enable */
	ie7816 &= ~UARTIE7816_BWTE;    /* block wait timer interrupt enable, for ISO7816 it must be turned off */
	ie7816 |= UARTIE7816_INITDE;   /* initial character detected interrupt enable */
	ie7816 |= UARTIE7816_GTVE;     /* guard timer violated interrupt enable */
	ie7816 &= ~UARTIE7816_TXTE;     /* transmit threshold exceeded interrupt enable, noisy */
	ie7816 |= UARTIE7816_RXTE;     /* receive threshold exceeded interrupt enable */
	writeb(ie7816, ccport->port.membase + UARTIE7816);

	ie7816 = readb( ccport->port.membase + UARTIE7816);
	printb(__func__, ie7816, "ie7816, [0xxxxxRxxx]\n");


	/* 9. */

	printk(KERN_ERR "YYY %s: 9. set ET7816 (error threshold) as desired\n", __func__);
	et7816 = readb(ccport->port.membase + UARTET7816);
	et7816 |= 0x00; // TODO:  0xff does not make difference
	if (0 == uartc7816_ttype) {
		printk(KERN_ERR "YYY %s: \tTTYPE == 0\n", __func__);
		if (1 == uartc7816_anack) {
			/* TXT asserts on the first NACK that is received */
			/* only meaningfull if UARTC7816[UARTCR7816_TTYPE]=0 */
			/* and UARTC7816[UARTCR7816_ANACK] == 1 */
			printk(KERN_ERR "YYY %s: \tANACK == 1, thus ET[TXTHRESHOLD] cleared\n", __func__);
			et7816 &= ~UART1ET7816_TXTHRESHOLD;
		}
		/* TXT asserts on the first NACK that is received */
		/* only meaningfull if UARTC7816[UARTCR7816_TTYPE] == 0 */
		printk(KERN_ERR "YYY %s: \tthus ET[RXTHRESHOLD] cleared\n", __func__);
		et7816 &= ~UART1ET7816_RXTHRESHOLD;
	}
	writeb(et7816, ccport->port.membase + UARTET7816); // problematic, when set too low for e.g. receiver issue

	et7816 = readb(ccport->port.membase + UARTET7816);
	printb(__func__, et7816, "et7816, [0xxxxxxxxx]\n");


	/* 10. */

	printk(KERN_ERR "YYY %s: 10. set C2[ILIE]=0, C2[RE]=1, C2[TE]=1, C2[RWU]=0, C2[SBK]=0 - C2[TIE]\n", __func__);
	cr2 = readb(ccport->port.membase + UARTCR2);
	if (!nodma) {
		printk(KERN_ERR "YYY %s: \tDMA settings\n", __func__);
		cr2 |= UARTCR2_TIE;    /* turn on DMA for transmit */
		cr2 |= UARTCR2_RIE;    /* turn on DMA for receiver */
	} else {
		printk(KERN_ERR "YYY %s: \tDMA turned off\n", __func__);
		cr2 &= ~UARTCR2_TIE;   /* turn off DMA for transmit */
		cr2 &= ~UARTCR2_RIE;   /* turn off DMA for receiver */
	}
	cr2 &= ~UARTCR2_TCIE;          /* reserved to 0 */
	cr2 &= ~UARTCR2_ILIE;          /* idle interrupt requests disabled */
	if (uartc7816_ttype) {
		cr2 |= UARTCR2_TE;     /* type 1: transmitter on */
	} else {
		cr2 |= UARTCR2_TE;     /* type 0: transmitter and */
      	        cr2 |= UARTCR2_RE;     /* receiver on */
	}
	cr2 &= ~UARTCR2_RWU;           /* normal operation: 0 */
	cr2 &= ~UARTCR2_SBK;           /* normal transmitter operation: 0 */
	writeb(cr2, ccport->port.membase + UARTCR2);

	cr2 = readb(ccport->port.membase + UARTCR2);
	printb(__func__, cr2, "cr2, [0xxRx01100]\n");

	/* addtional steps */
// DEBUGGING


	printk(KERN_ERR "YYY %s: ~~~ ISO7816 INITIALIZATION DONE ~~~\n", __func__);

	return 0;
}

/**
 * cc function: atr
 */
static int iso7816_cc_atr(struct iso7816_port *ccport)
{
//	unsigned char /* cr2, */ cr3, sr2, cr7816;
	int /*idx,*/ tictacs;
	unsigned long flags;
	unsigned char is7816;

	printk(KERN_ERR "YYY %s(): started\n", __func__);

	local_irq_save(flags);

	/* init */
	gpio_set_value(ccport->gpio[ISO7816_GPIO_RST], 0);
	tictacs=100;

	/* turn on clock */
	printk(KERN_ERR "YYY %s(): set reset to high\n", __func__);
	gpio_set_value(ccport->gpio[ISO7816_GPIO_RST], 1); // RST to H after 400 clock cycles

	// tic tic tic
/*	
  	for (idx=0; idx<196 * 2*372; ++idx) {
 		if (gpio_get_value(ccport->gpio[ISO7816_GPIO_CLK])) {
 			gpio_set_value(ccport->gpio[ISO7816_GPIO_CLK], 0);
 		} else {
 			gpio_set_value(ccport->gpio[ISO7816_GPIO_CLK], 1);
 		}
 		udelay(tictacs);
 	}
/*/
	mdelay(10);

	local_irq_restore(flags);           

	/* additional */
//	mdelay(10);

//	is7816 = readb(ccport->port.membase + UARTIS7816);
//	writeb(is7816 | UARTIS7816_INIT, ccport->port.membase + UARTIS7816);

	printk(KERN_ERR "YYY %s: ~~~ ISO7816 ATR DONE ~~~\n", __func__);
	return 0;
}


/**
 * Stops and starts te and re registers
 */
static void iso7816_cc_restart_re_te(struct iso7816_port *ccport)
{
	unsigned char cr2;

	/* stop */
	cr2 = readb(ccport->port.membase + UARTCR2);
	cr2 &= ~(UARTCR2_TE | UARTCR2_RE | UARTCR2_TIE | UARTCR2_TCIE
		 | UARTCR2_RIE);
	writeb(cr2, ccport->port.membase + UARTCR2);

	/* start again */
	cr2 = readb(ccport->port.membase + UARTCR2);
	cr2 |= (UARTCR2_TE | UARTCR2_RE);

// for DMA, also enable the corresponding interrupts
//	cr2 &= (UARTCR2_TE | UARTCR2_RE | UARTCR2_TIE | UARTCR2_TCIE
//		 | UARTCR2_RIE);

	writeb(cr2, ccport->port.membase + UARTCR2);

}

// /**
//  * read fabrication code for testing and debugging
//  * AT88SC256C specific command
//  */
// static void iso7816_cc_read_fab(struct iso7816_port *ccport)
// {
// 	unsigned char message[] = {0x00, 0xb6, 0x00, 0x08, 0x00}; // read address 0x08, FAB code + etc
// 	int message_siz;
// 
// 	printk(KERN_ERR "YYY %s(): started\n", __func__);
// 	message_siz = sizeof(message);
// 
// 	/* put message to buffer */
// // TODO implementation for "early write" in fsl_lpuart.c		
// //	iso7816_cc_command(ccport, message, message_siz);
// 
// 	/* transmit */
// // TODO init xmit.buf	
// //	iso7816_start_tx(&ccport->port); 
// // or (direct)
// 	if (readb(ccport->port.membase + UARTSR1) & UARTSR1_TDRE)
// 		iso7816_transmit_buffer(ccport);
// 
// 	
// 	/* read answer */
// // TODO 	
// 
// 	printk( KERN_ERR "YYY %s(): done\n", __func__);
// }


/* pops functions */

/**
 * pops function: tx_empty()
 *
 * return TIOCSER_TEMT when transmitter is not busy
 */
static unsigned int iso7816_tx_empty(struct uart_port *port)
{
//	struct iso7816_port *ccport = container_of(port,
//			struct iso7816_port, port);
	unsigned char sr1 = readb(port->membase + UARTSR1);
	unsigned char sfifo = readb(port->membase + UARTSFIFO);

// TODO no dma
//	if (ccport->dma_tx_in_progress)
//		return 0;

	if (sr1 & UARTSR1_TC && sfifo & UARTSFIFO_TXEMPT)
		return TIOCSER_TEMT;
//*/
	return 0;
}

/**
 * pops function: set_mctrl()
 */
static void iso7816_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
//	unsigned char temp;
	struct iso7816_port *ccport = container_of(port,
				struct iso7816_port, port);

	if (1 == ccport->port.minor) {
		printk( KERN_ERR "YYY %s(): no modem control implemented\n", __func__);
		return;
	}

// TODO no RS485 for chipcard
        /* Make sure RXRTSE bit is not set when RS485 is enabled */
//	if (!(ccport->port.rs485.flags & SER_RS485_ENABLED)) {

// TODO no RTS/CTS for chipcard
//		temp = readb(ccport->port.membase + UARTMODEM) &
//			~(UARTMODEM_RXRTSE | UARTMODEM_TXCTSE);
//
//		if (mctrl & TIOCM_RTS)
//			temp |= UARTMODEM_RXRTSE;
//
//		if (mctrl & TIOCM_CTS)
//			temp |= UARTMODEM_TXCTSE;
//
//		writeb(temp, port->membase + UARTMODEM);

//	}
}

/**
 * pops function: get_mctrl()
 */
static unsigned int iso7816_get_mctrl(struct uart_port *port)
{
	unsigned int temp = 0;
//	unsigned char reg;

	if (1 == port->minor) {
		printk( KERN_ERR "YYY %s(): no modem control implemented\n", __func__);
		return 0;
	}

// TODO no RTS/CTS implemented
//	reg = readb(port->membase + UARTMODEM);
//	if (reg & UARTMODEM_TXCTSE)
//		temp |= TIOCM_CTS;
//
//	if (reg & UARTMODEM_RXRTSE)
//		temp |= TIOCM_RTS;
//
	return temp;
}

/**
 * pops function: stop_tx()
 */
static void iso7816_stop_tx(struct uart_port *port)
{
	unsigned char cr2;

	printk( KERN_ERR "YYY %s(): started\n", __func__);    

	/* get C2 register,
	   clear transmit interrupts (TIE) and
	   undocumented TCIE field,
	   and set it again */

	cr2 = readb(port->membase + UARTCR2);
	cr2 &= ~(UARTCR2_TIE | UARTCR2_TCIE);
	writeb(cr2, port->membase + UARTCR2);
}

/**
 * pops function: start_tx()
 *
 * set transmit interrups or dma transfer enable (TIE),
 * then check TWFIFO[TXWATER] to send buffer
 */
static void iso7816_start_tx(struct uart_port *port)
{
	struct iso7816_port *ccport = container_of(port,
			struct iso7816_port, port);
//	struct circ_buf *xmit = &ccport->port.state->xmit;
	unsigned char temp;

	printk(KERN_ERR "YYY %s(): started\n", __func__);

	/* debugging */
	iso7816_cc_report(ccport);

	/* set transmit interrupt or dma transfer enable (TIE) */
	temp = readb(port->membase + UARTCR2);
	writeb(temp | UARTCR2_TIE, port->membase + UARTCR2);

// TODO no dma usage    	
//	if (ccport->iso7816_dma_tx_use) {
//		if (!uart_circ_empty(xmit) && !uart_tx_stopped(port))
//			iso7816_dma_tx(ccport);
//	} else {
  	        /* transmit buffer, when the amount of data in the transmit
		   buffer is less than or equal to the value indicated by
		   TWFIFO[TXWATER] at some point in time since the flag has been
		   cleared. */
		if (readb(port->membase + UARTSR1) & UARTSR1_TDRE)
			iso7816_transmit_buffer(ccport);
//	}
}

/**
 * pops function: stop_rx()
 */
static void iso7816_stop_rx(struct uart_port *port)
{
	unsigned char temp;
	
DBG_FUNCNAME2
	

	/* turn off receiver */
	temp = readb(port->membase + UARTCR2);
	writeb(temp & ~UARTCR2_RE, port->membase + UARTCR2);
}

/**
 * pops function: break_ctl()
 *
 * @port: the struct uart_port instance
 * @break_state: 1, set send break flag (SBK)
 */
static void iso7816_break_ctl(struct uart_port *port, int break_state)
{
	unsigned char temp;
	
//DBG_FUNCNAME2
	

	temp = readb(port->membase + UARTCR2) & ~UARTCR2_SBK;

	/* set send break (SBK), if break_state was set */
	if (break_state != 0)
		temp |= UARTCR2_SBK;

	writeb(temp, port->membase + UARTCR2);
}

/**
 * pops function: startup()
 *
 * @port: instance of the struct uart_port
 */
static int iso7816_startup(struct uart_port *port)
{
	struct iso7816_port *ccport = container_of(port, struct iso7816_port, port);
	int ret;

	unsigned long flags;
	unsigned char temp;
	
DBG_FUNCNAME
	


	/* determine FIFO size and enable FIFO mode */
	temp = readb(ccport->port.membase + UARTPFIFO);

	ccport->txfifo_size = 0x1 << (((temp >> UARTPFIFO_TXSIZE_OFF) &
		UARTPFIFO_FIFOSIZE_MASK) + 1);

	ccport->port.fifosize = ccport->txfifo_size;

	ccport->rxfifo_size = 0x1 << (((temp >> UARTPFIFO_RXSIZE_OFF) &
		UARTPFIFO_FIFOSIZE_MASK) + 1);

	ret = devm_request_irq(port->dev, port->irq, iso7816_int, 0,
				DRIVER_NAME, ccport);
	if (ret)
		return ret;


	spin_lock_irqsave(&ccport->port.lock, flags);
	iso7816_setup_watermark(ccport);

// TODO not performed for ISO7816, TE and RE will be turned on later             
//	if (1 != ccport->port.minor) {
//		temp = readb(ccport->port.membase + UARTCR2);
//		temp |= (UARTCR2_RIE | UARTCR2_TIE | UARTCR2_RE | UARTCR2_TE);
//		writeb(temp, ccport->port.membase + UARTCR2);
//	}

	spin_unlock_irqrestore(&ccport->port.lock, flags);

// TODO currently no dma                    
//	if (ccport->dma_rx_chan && !iso7816_start_rx_dma(ccport)) {
//if (1 == ccport->port.minor) { printk(KERN_ERR "YYY %s: ccport->dma_rx_chan enabled\n", __func__); }
//		// set Rx DMA timeout
//		ccport->dma_rx_timeout = msecs_to_jiffies(DMA_RX_TIMEOUT);
//		if (!ccport->dma_rx_timeout)
//		     ccport->dma_rx_timeout = 1;
//
//		ccport->iso7816_dma_rx_use = true;
//		setup_timer(&ccport->iso7816_timer, iso7816_timer_func,
//				(unsigned long)ccport);
//		ccport->iso7816_timer.expires = jiffies + ccport->dma_rx_timeout;
//		add_timer(&ccport->iso7816_timer);
//	} else {
//if (1 == ccport->port.minor) { printk(KERN_ERR "YYY %s: ccport->dma_rx_chan DISABLED!\n", __func__); }	        
//		ccport->iso7816_dma_rx_use = false;
//	}

// TODO currently no dma		
//	if (ccport->dma_tx_chan && !iso7816_dma_tx_request(port)) {
//if (1 == ccport->port.minor) { printk(KERN_ERR "YYY %s: ccport->dma_tx_chan enabled\n", __func__); }		
//		init_waitqueue_head(&ccport->dma_wait);
//		ccport->iso7816_dma_tx_use = true;
//		temp = readb(port->membase + UARTCR5);
//		writeb(temp | UARTCR5_TDMAS, port->membase + UARTCR5);
//	} else {
//if (1 == ccport->port.minor) { printk(KERN_ERR "YYY %s: ccport->dma_tx_chan DISABLED!\n", __func__); }		
//		ccport->iso7816_dma_tx_use = false;
//	}

	return 0;
}


/**
 * pops function: shutdown()
 *
 * @port: instance of struct uart_port
 */
static void iso7816_shutdown(struct uart_port *port)
{
	struct iso7816_port *ccport = container_of(port, struct iso7816_port, port);
	unsigned char temp;
	unsigned long flags;
	
DBG_FUNCNAME

	spin_lock_irqsave(&port->lock, flags);

	/* disable Rx/Tx and interrupts */
	temp = readb(port->membase + UARTCR2);
	temp &= ~(UARTCR2_TE | UARTCR2_RE |
			UARTCR2_TIE | UARTCR2_TCIE | UARTCR2_RIE);
	writeb(temp, port->membase + UARTCR2);

	spin_unlock_irqrestore(&port->lock, flags);

	devm_free_irq(port->dev, port->irq, ccport);
}


/**
 * pops function: set_termios()
 *
 * settings / initialization according to NXP documented chipcard settings
 * legacy driver settings were handling terminal settings
 *
 * @port: instance of struct uart_port
 * @termios: the terminal settings
 * @old: backup of the terminal settings
 */
static void
iso7816_set_termios(struct uart_port *port, struct ktermios *termios,
		   struct ktermios *old)
{
	struct iso7816_port *ccport = container_of(port, struct iso7816_port, port);
	unsigned long flags;
	unsigned char cr1, old_cr1, old_cr2, cr3, cr4, bdh, modem;
	unsigned int  baud;
//	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;
	unsigned int sbr, brfa;
//	int ret;

		
        /* implementation for serial uart */

	// read out control, baud rate and modem registers
	cr1 = old_cr1 = readb(ccport->port.membase + UARTCR1);
	old_cr2 = readb(ccport->port.membase + UARTCR2);
	cr3 = readb(ccport->port.membase + UARTCR3);
	cr4 = readb(ccport->port.membase + UARTCR4);
	bdh = readb(ccport->port.membase + UARTBDH);
	modem = readb(ccport->port.membase + UARTMODEM);

// TODO cc init takes care of that      	
//	/*
//	 * only support CS8 and CS7, and for CS7 must enable PE.
//	 * supported mode:
//	 *  - (7,e/o,1)
//	 *  - (8,n,1)
//	 *  - (8,m/s,1)
//	 *  - (8,e/o,1)
//	 */
//	while ((termios->c_cflag & CSIZE) != CS8 &&
//		(termios->c_cflag & CSIZE) != CS7) {
//		termios->c_cflag &= ~CSIZE;
//		termios->c_cflag |= old_csize;
//		old_csize = CS8;
//	}
//
//	if ((termios->c_cflag & CSIZE) == CS8 ||
//		(termios->c_cflag & CSIZE) == CS7)
//		cr1 = old_cr1 & ~UARTCR1_M;
//
//	if (termios->c_cflag & CMSPAR) {
//		if ((termios->c_cflag & CSIZE) != CS8) {
//			termios->c_cflag &= ~CSIZE;
//			termios->c_cflag |= CS8;
//		}
//		cr1 |= UARTCR1_M;
//	}

// TODO no RS485 for cc                             
//	/*
//	 * When auto RS-485 RTS mode is enabled,
//	 * hardware flow control need to be disabled.
//	 */
//	if (ccport->port.rs485.flags & SER_RS485_ENABLED)
//		termios->c_cflag &= ~CRTSCTS;

// TODO no RTS/CTS support for cc                      
//	/* RTS/CTS */
//	if (termios->c_cflag & CRTSCTS) {
//		modem |= (UARTMODEM_RXRTSE | UARTMODEM_TXCTSE);
//	} else {
//		termios->c_cflag &= ~CRTSCTS;
//		modem &= ~(UARTMODEM_RXRTSE | UARTMODEM_TXCTSE);
//	}

// TODO cc init takes care of that      		
//	/* stop bit */
//	if (termios->c_cflag & CSTOPB)
//		termios->c_cflag &= ~CSTOPB;

// TODO cc init takes care of that      		
//	/* parity must be enabled when CS7 to match 8-bits format */
//	if ((termios->c_cflag & CSIZE) == CS7)
//		termios->c_cflag |= PARENB;
//
//	/* parenty bit */
//	if ((termios->c_cflag & PARENB)) {
//		if (termios->c_cflag & CMSPAR) {
//			cr1 &= ~UARTCR1_PE;
//			if (termios->c_cflag & PARODD)
//				cr3 |= UARTCR3_T8;
//			else
//				cr3 &= ~UARTCR3_T8;
//		} else {
//			cr1 |= UARTCR1_PE;
//			if ((termios->c_cflag & CSIZE) == CS8)
//				cr1 |= UARTCR1_M;
//			if (termios->c_cflag & PARODD)
//				cr1 |= UARTCR1_PT;
//			else
//				cr1 &= ~UARTCR1_PT;
//		}
//	}

	/* ask the core to calculate the divisor */
	baud = uart_get_baud_rate(port, termios, old, 50, port->uartclk / 16);

	spin_lock_irqsave(&ccport->port.lock, flags);

	ccport->port.read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		ccport->port.read_status_mask |= (UARTSR1_FE | UARTSR1_PE);
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		ccport->port.read_status_mask |= UARTSR1_FE;

// TODO cc init takes care of that      		
//	/* characters to ignore */
//	ccport->port.ignore_status_mask = 0;
//	if (termios->c_iflag & IGNPAR)
//		ccport->port.ignore_status_mask |= UARTSR1_PE;
//	if (termios->c_iflag & IGNBRK) {
//		ccport->port.ignore_status_mask |= UARTSR1_FE;
//		/*
//		 * if we're ignoring parity and break indicators,
//		 * ignore overruns too (for real raw support).
//		 */
//		if (termios->c_iflag & IGNPAR)
//			ccport->port.ignore_status_mask |= UARTSR1_OR;
//	}

// TODO cc init takes care of that      		
//	/* update the per-port timeout */
//	uart_update_timeout(port, termios->c_cflag, baud);

// TODO check if this is needed 	
	/* wait transmit engin complete */
	while (!(readb(ccport->port.membase + UARTSR1) & UARTSR1_TC))
		barrier();

	/* disable transmit and receive, for baud rate adjustment */
	writeb(old_cr2 & ~(UARTCR2_TE | UARTCR2_RE),
			ccport->port.membase + UARTCR2);

	/* compute baud rate and baud rate fine adjustment (brfa) */
	sbr = ccport->port.uartclk / (16 * baud);
	brfa = ((ccport->port.uartclk - (16 * sbr * baud)) * 2) / baud;
	bdh &= ~UARTBDH_SBR;
	bdh |= (sbr >> 8) & 0x1F;
	cr4 &= ~UARTCR4_BRFA;
	brfa &= UARTCR4_BRFA;
	writeb(cr4 | brfa, ccport->port.membase + UARTCR4);
	writeb(bdh, ccport->port.membase + UARTBDH);
	writeb(sbr & 0xFF, ccport->port.membase + UARTBDL);
	writeb(cr3, ccport->port.membase + UARTCR3);
	writeb(cr1, ccport->port.membase + UARTCR1);
	writeb(modem, ccport->port.membase + UARTMODEM);

	/* restore control register 2, i.e. turn TE and RE on */
	writeb(old_cr2, ccport->port.membase + UARTCR2);

// TODO no dma      			
//	/*
//	 * If new baud rate is set, we will also need to update the Ring buffer
//	 * length according to the selected baud rate and restart Rx DMA path.
//	 */
//	if (old) {
//		if (ccport->iso7816_dma_rx_use) {
//			del_timer_sync(&ccport->iso7816_timer);
//			iso7816_dma_rx_free(&ccport->port);
//		}
//
//		if (ccport->dma_rx_chan && !iso7816_start_rx_dma(ccport)) {
//			ccport->iso7816_dma_rx_use = true;
//			setup_timer(&ccport->iso7816_timer, iso7816_timer_func,
//					(unsigned long)ccport);
//			ccport->iso7816_timer.expires =
//					jiffies + ccport->dma_rx_timeout;
//			add_timer(&ccport->iso7816_timer);
//		} else {
//			ccport->iso7816_dma_rx_use = false;
//		}
//	}
	spin_unlock_irqrestore(&ccport->port.lock, flags);

	// don't allow further settings for iso7816 port
	return;
}


/**
 * pops function: type()
 */
static const char *iso7816_type(struct uart_port *port)
{
	return "NXP_ISO7816";
}


/**
 * pops function: request_port()
 */
static int iso7816_request_port(struct uart_port *port)
{
	return  0;
}


/**
 * pops function: release_port()
 */
static void iso7816_release_port(struct uart_port *port)
{
	/* nothing to do */
}


/**
 * pops function: config_port()
 * configure/autoconfigure the port
 */
static void iso7816_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_ISO7816;
}


/**
 * pops function: verify_port()
 */
static int iso7816_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_ISO7816)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != UPIO_MEM)
		ret = -EINVAL;
	if (port->uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}



/**
 * pops function: flush_buffer()
 */
static void iso7816_flush_buffer(struct uart_port *port)
{
// TODO no dma used so far	
//	struct iso7816_port *ccport = container_of(port, struct iso7816_port, port);
//	if (ccport->iso7816_dma_tx_use) {
//		if (ccport->dma_tx_in_progress) {
//			dma_unmap_sg(ccport->port.dev, &ccport->tx_sgl[0],
//				ccport->dma_tx_nents, DMA_TO_DEVICE);
//			ccport->dma_tx_in_progress = false;
//		}
//		dmaengine_terminate_all(ccport->dma_tx_chan);
//	}
}

/**
 * helper function: rxint()
 *
 * isr fo rx interrupts
 */
static irqreturn_t iso7816_rxint(int irq, void *dev_id)
{
	struct iso7816_port *ccport = dev_id;
//	unsigned int flg, ignored = 0;
	struct tty_port *port = &ccport->port.state->port;
	unsigned long flags;
//	unsigned char rx, sr;
/*	
DBG_FUNCNAME
/*/	
//	if (1 == ccport->port.minor) { // YYY
		printk( KERN_ERR "YYY %s(): stubbed\n", __func__);
		goto out;
//	}
//*/

// TODO can we handl parity, overrun and framing error in cc reading???		
//	spin_lock_irqsave(&ccport->port.lock, flags);
//
//	while (!(readb(ccport->port.membase + UARTSFIFO) & UARTSFIFO_RXEMPT)) {
//		flg = TTY_NORMAL;
//		ccport->port.icount.rx++;
//		/*
//		 * to clear the FE, OR, NF, FE, PE flags,
//		 * read SR1 then read DR
//		 */
//		sr = readb(ccport->port.membase + UARTSR1);
//		rx = readb(ccport->port.membase + UARTDR);
//
//		if (uart_handle_sysrq_char(&ccport->port, (unsigned char)rx))
//			continue;
//
//		/* error handling */
//		if (sr & (UARTSR1_PE | UARTSR1_OR | UARTSR1_FE)) {
//			/* parity */
//			if (sr & UARTSR1_PE)
//				ccport->port.icount.parity++;
//
//			/* framing error */
//			else if (sr & UARTSR1_FE)
//				ccport->port.icount.frame++;
//
//			/* overrun */
//			if (sr & UARTSR1_OR)
//				ccport->port.icount.overrun++;
//
//			/* status mask */
//			if (sr & ccport->port.ignore_status_mask) {
//				if (++ignored > 100)
//					goto out;
//				continue;
//			}
//			sr &= ccport->port.read_status_mask;
//
//			/* flag parity error */
//			if (sr & UARTSR1_PE)
//				flg = TTY_PARITY;
//			else if (sr & UARTSR1_FE)
//				flg = TTY_FRAME;
//
//			/* flag overrun error */
//			if (sr & UARTSR1_OR)
//				flg = TTY_OVERRUN;
//
//#ifdef SUPPORT_SYSRQ
//			ccport->port.sysrq = 0;
//#endif
//		}
//
//		/* set flag */
//		tty_insert_flip_char(port, rx, flg);
//	}
//
out:
	spin_unlock_irqrestore(&ccport->port.lock, flags);

	tty_flip_buffer_push(port);
	return IRQ_HANDLED;
}

/**
 * helper function: txint()
 */
static irqreturn_t iso7816_txint(int irq, void *dev_id)
{
	struct iso7816_port *ccport = dev_id;
	struct circ_buf *xmit = &ccport->port.state->xmit;
	unsigned long flags;

	printk( KERN_ERR "YYY %s(): started\n", __func__);

	spin_lock_irqsave(&ccport->port.lock, flags);

	
	if (1 == ccport->port.minor) { // TODO YYY
		printk( KERN_ERR "YYY %s(): stubbed\n", __func__);
		goto out;
	}
	

	/* need to write x_char from buffer into D register */
	if (ccport->port.x_char) {
// TODO no 32 bit implementation		
//		if (ccport->iso781632)
//			iso781632_write(ccport->port.x_char, ccport->port.membase + UARTDATA);
//		else
			writeb(ccport->port.x_char, ccport->port.membase + UARTDR);
		goto out;
	}

	/* need to stop tx */
	if (uart_circ_empty(xmit) || uart_tx_stopped(&ccport->port)) {
// TODO no 32 bit implementation		
//		if (ccport->iso781632)
//			iso781632_stop_tx(&ccport->port);
//		else
			iso7816_stop_tx(&ccport->port);
		goto out;
	}

	/* need to transmit entire buffer */
// TODO no 32 bit implementation		
//	if (ccport->iso781632)
//		iso781632_transmit_buffer(ccport);
//	else
		iso7816_transmit_buffer(ccport);

	/* wake characters */
// TODO do we have WAKEUP_CHARs for chipcards?		
//	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
//		uart_write_wakeup(&ccport->port);

out:
	spin_unlock_irqrestore(&ccport->port.lock, flags);
	return IRQ_HANDLED;
}


/**
 * helper function: int()
 */
static irqreturn_t iso7816_int(int irq, void *dev_id)
{

	struct iso7816_port *ccport = dev_id;
	unsigned char sts;

	sts = readb(ccport->port.membase + UARTSR1);

	if (sts & UARTSR1_RDRF)
		iso7816_rxint(irq, dev_id);

	if (sts & UARTSR1_TDRE)
		iso7816_txint(irq, dev_id);

	return IRQ_HANDLED;
}


/**
 * helper function iso7816_copy_rx_to_tty()
 */
// TODO currently not used
//static void iso7816_copy_rx_to_tty(struct iso7816_port *ccport)
//{
//*
// TODO needs implementation		
//		printk( KERN_ERR "YYY %s(): stubbed\n", __func__);
//		return;
/*/
	struct tty_port *port = &ccport->port.state->port;
	struct dma_tx_state state;
	enum dma_status dmastat;
	struct circ_buf *ring = &ccport->rx_ring;
	unsigned long flags;
	int count = 0;
	unsigned char sr;

//	if (1 == ccport->port.minor) {
		printk( KERN_ERR "YYY %s(): TEST copy rx to out\n", __func__);

		sr = readb(ccport->port.membase + UARTSR1);

		if (sr & (UARTSR1_PE | UARTSR1_FE)) {
//			/ * Read DR to clear the error flags * /
			readb(ccport->port.membase + UARTDR);

			if (sr & UARTSR1_PE)
				ccport->port.icount.parity++;
			else if (sr & UARTSR1_FE)
				ccport->port.icount.frame++;
		}


//		async_tx_ack(ccport->dma_rx_desc); // TODO rm, no active ack in iso7816

		spin_lock_irqsave(&ccport->port.lock, flags);

// TODO check, no ack sent, do we need to check for dmaengine_TX_status() ?
		dmastat = dmaengine_tx_status(ccport->dma_rx_chan,
					      ccport->dma_rx_cookie,
					      &state);

		if (dmastat == DMA_ERROR) {
//			dev_err(ccport->port.dev, "Rx DMA transfer failed!\n");  
			pr_err( "Rx DMA transfer failed!\n");  
			spin_unlock_irqrestore(&ccport->port.lock, flags);
			return;
		}

//		/ * CPU claims ownership of RX DMA buffer * /
		dma_sync_sg_for_cpu(ccport->port.dev, &ccport->rx_sgl, 1, DMA_FROM_DEVICE);

//		/ *
//		 * ring->head points to the end of data already written by the DMA.
//		 * ring->tail points to the beginning of data to be read by the
//		 * framework.
//		 * The current transfer size should not be larger than the dma buffer
//		 * length.
//		 * /
		ring->head = ccport->rx_sgl.length - state.residue;
		BUG_ON(ring->head > ccport->rx_sgl.length);
//		/ *
//		 * At this point ring->head may point to the first byte right after the
//		 * last byte of the dma buffer:
//		 * 0 <= ring->head <= ccport->rx_sgl.length
//		 *
//		 * However ring->tail must always points inside the dma buffer:
//		 * 0 <= ring->tail <= ccport->rx_sgl.length - 1
//		 *
//		 * Since we use a ring buffer, we have to handle the case
//		 * where head is lower than tail. In such a case, we first read from
//		 * tail to the end of the buffer then reset tail.
//		 * /
		if (ring->head < ring->tail) {
			count = ccport->rx_sgl.length - ring->tail;

			tty_insert_flip_string(port, ring->buf + ring->tail, count);
			ring->tail = 0;
			ccport->port.icount.rx += count;
		}

//		/ * Finally we read data from tail to head * /
		if (ring->tail < ring->head) {
			count = ring->head - ring->tail;
			tty_insert_flip_string(port, ring->buf + ring->tail, count);
//			/ * Wrap ring->head if needed * /
			if (ring->head >= ccport->rx_sgl.length)
				ring->head = 0;
			ring->tail = ring->head;
			ccport->port.icount.rx += count;
		}

		dma_sync_sg_for_device(ccport->port.dev, &ccport->rx_sgl, 1,
				       DMA_FROM_DEVICE);

		spin_unlock_irqrestore(&ccport->port.lock, flags);

		tty_flip_buffer_push(port);
		mod_timer(&ccport->iso7816_timer, jiffies + ccport->dma_rx_timeout);

                return;
	}


	sr = readb(ccport->port.membase + UARTSR1);

	if (sr & (UARTSR1_PE | UARTSR1_FE)) {
//		/ * Read DR to clear the error flags * /
		readb(ccport->port.membase + UARTDR);

		if (sr & UARTSR1_PE)
		    ccport->port.icount.parity++;
		else if (sr & UARTSR1_FE)
		    ccport->port.icount.frame++;
	}

	async_tx_ack(ccport->dma_rx_desc);

	spin_lock_irqsave(&ccport->port.lock, flags);

	dmastat = dmaengine_tx_status(ccport->dma_rx_chan,
				ccport->dma_rx_cookie,
				&state);

	if (dmastat == DMA_ERROR) {
//		dev_err(ccport->port.dev, "Rx DMA transfer failed!\n");  
                pr_err( "Rx DMA transfer failed!\n");
		spin_unlock_irqrestore(&ccport->port.lock, flags);
		return;
	}

//	/ * CPU claims ownership of RX DMA buffer * /
	dma_sync_sg_for_cpu(ccport->port.dev, &ccport->rx_sgl, 1, DMA_FROM_DEVICE);

//	/ *
//	 * ring->head points to the end of data already written by the DMA.
//	 * ring->tail points to the beginning of data to be read by the
//	 * framework.
//	 * The current transfer size should not be larger than the dma buffer
//	 * length.
//	 * /
	ring->head = ccport->rx_sgl.length - state.residue;
	BUG_ON(ring->head > ccport->rx_sgl.length);
//	/ *
//	 * At this point ring->head may point to the first byte right after the
//	 * last byte of the dma buffer:
//	 * 0 <= ring->head <= ccport->rx_sgl.length
//	 *
//	 * However ring->tail must always points inside the dma buffer:
//	 * 0 <= ring->tail <= ccport->rx_sgl.length - 1
//	 *
//	 * Since we use a ring buffer, we have to handle the case
//	 * where head is lower than tail. In such a case, we first read from
//	 * tail to the end of the buffer then reset tail.
//	 * /
	if (ring->head < ring->tail) {
		count = ccport->rx_sgl.length - ring->tail;

		tty_insert_flip_string(port, ring->buf + ring->tail, count);
		ring->tail = 0;
		ccport->port.icount.rx += count;
	}

//	/ * Finally we read data from tail to head * /
	if (ring->tail < ring->head) {
		count = ring->head - ring->tail;
		tty_insert_flip_string(port, ring->buf + ring->tail, count);
//		/ * Wrap ring->head if needed * /
		if (ring->head >= ccport->rx_sgl.length)
			ring->head = 0;
		ring->tail = ring->head;
		ccport->port.icount.rx += count;
	}

	dma_sync_sg_for_device(ccport->port.dev, &ccport->rx_sgl, 1,
			       DMA_FROM_DEVICE);

	spin_unlock_irqrestore(&ccport->port.lock, flags);

	tty_flip_buffer_push(port);
	mod_timer(&ccport->iso7816_timer, jiffies + ccport->dma_rx_timeout);
//*/
//}


/**
 * helper function timer_func()
 *
 * to be set as:
 *     setup_timer(&ccport->iso7816_timer, iso7816_timer_func,
 *             (unsigned long)ccport);
 *
 */
//static void iso7816_timer_func(unsigned long data)
//{
//	struct iso7816_port *ccport = (struct iso7816_port *)data;
//
//	iso7816_copy_rx_to_tty(ccport);
//}

/**
 * helper function transmit_buffer()
 *
 * function for sending input received directly from /dev/ttyLP1,
 * and on running system
 */
static inline void iso7816_transmit_buffer(struct iso7816_port *ccport)
{
	struct circ_buf *xmit = &ccport->port.state->xmit;

	printk(KERN_ERR "YYY %s(): started\n", __func__);

	while (!uart_circ_empty(xmit) &&
		(readb(ccport->port.membase + UARTTCFIFO) < ccport->txfifo_size)) {
		writeb(xmit->buf[xmit->tail], ccport->port.membase + UARTDR);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		ccport->port.icount.tx++;
	}

	/* wake up characters */
// TODO needed?
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		uart_write_wakeup(&ccport->port);
  	}

	if (uart_circ_empty(xmit))
		iso7816_stop_tx(&ccport->port);

	printk(KERN_ERR "YYY %s(): done\n", __func__);
}


/* structs - initial, below function definitions */

static struct uart_ops iso7816_pops = {
	.tx_empty	= iso7816_tx_empty,
	.set_mctrl	= iso7816_set_mctrl,
	.get_mctrl	= iso7816_get_mctrl,
	.stop_tx	= iso7816_stop_tx,
	.start_tx	= iso7816_start_tx,
	.stop_rx	= iso7816_stop_rx,
	.break_ctl	= iso7816_break_ctl,
	.startup	= iso7816_startup,
	.shutdown	= iso7816_shutdown,
	.set_termios	= iso7816_set_termios,
	.type		= iso7816_type,
	.request_port	= iso7816_request_port,
	.release_port	= iso7816_release_port,
	.config_port	= iso7816_config_port,
	.verify_port	= iso7816_verify_port,
	.flush_buffer	= iso7816_flush_buffer,
};

struct iso7816_port *iso7816_ports[ISO7816_UART_NR];


                                                                      
// TODO set up 'CONFIG_SERIAL_ISO7816_CONSOLE'                     
//#ifdef CONFIG_SERIAL_ISO7816_CONSOLE
#ifdef CONFIG_SERIAL_FSL_LPUART_CONSOLE
// TODO --> init__, register with iso7816_console instead of iso7816_reg
                                                                      





/* functions - CONFIG_SERIAL_ISO7816_CONSOLE */
/* aka CONFIG_SERIAL_FSL_LPUART_CONSOLE */

/**
 * putchar for console
 *
 * currently called by iso7816_console_write()
 */
static void iso7816_console_putchar(struct uart_port *port, int ch)
{
	printk( KERN_ERR "YYY %s(): started\n", __func__);
	printk( KERN_ERR "YYY %s(): ch = '0x%02x'\n", __func__, ch);

	/* check if transmit data register is empty (TDRE)
	 * 0    the amount of data in the transmit buffer is greater than the
	 *      value indicated by TWFIFO[TXWATER]
	 *
	 * 1    the amount of data in the transmit buffer is less than or equal
	 *      to the value indicated by TWFIFO[TXWATER] at some point in time
	 *      since the flag has been cleared
	 */
	while (!(readb(port->membase + UARTSR1) & UARTSR1_TDRE)) {
		printk( KERN_ERR "YYY %s(): TDRE set!!!\n", __func__);
		barrier();
	}

	writeb(ch, port->membase + UARTDR);
}


/**
 * write a console message to the chipcard
 *
 * implementation based on serial_core's uart_console_write()
 * additionally, disable interrupts, write, wait for transmitter finish, and
 * restore controll register settings
 */
static void
iso7816_console_write(struct console *co, const char *str, unsigned int count)
{
//DBG_FUNCNAME - noisy
	struct iso7816_port *ccport = iso7816_ports[co->index];
	unsigned char  old_cr2, cr2;
	
DBG_FUNCNAME
	
// TODO check this implementation - usefull for chipcard communication?            
	/* enable RX/TX, disable TX/RX interrupts */
	cr2 = old_cr2 = readb(ccport->port.membase + UARTCR2);
	cr2 |= (UARTCR2_TE |  UARTCR2_RE);
	cr2 &= ~(UARTCR2_TIE | UARTCR2_TCIE | UARTCR2_RIE);
	writeb(cr2, ccport->port.membase + UARTCR2);

	/* write character to transmit buffer */
	uart_console_write(&ccport->port, str, count, iso7816_console_putchar);

	/* wait for transmitter finish complete */
	while (!(readb(ccport->port.membase + UARTSR1) & UARTSR1_TC))
		barrier();

	/* and restore CR2 */
	writeb(old_cr2, ccport->port.membase + UARTCR2);
}


// static void
// iso7816_cc_command(struct iso7816_port *ccport, const unsigned char *str, unsigned int count)
// {
// //	unsigned char  old_cr2, cr2;  
// 
// 	printk( KERN_ERR "YYY %s(): started\n", __func__);
// 	printk( KERN_ERR "YYY %s(): str = '%s'\n", __func__, str);
// 	printk( KERN_ERR "YYY %s(): count = '%d'\n", __func__, count);
// 
// 	/* enable RX/TX, disable TX/RX interrupts */
// // TODO cosmetics - is this needed?     	
// //	cr2 = old_cr2 = readb(ccport->port.membase + UARTCR2);
// //	cr2 |= (UARTCR2_TE |  UARTCR2_RE);
// //	cr2 &= ~(UARTCR2_TIE | UARTCR2_TCIE | UARTCR2_RIE);
// //	writeb(cr2, ccport->port.membase + UARTCR2);
// 
// 	/* write character to transmit buffer */
// 	uart_console_write(&ccport->port, str, count, iso7816_console_putchar);
// 
// 	/* wait for transmitter finish complete */
// // TODO problematic: FIXME	
// // "when C7816[ISO_7816E] is enabled this field is set after any NACK signal has been received, but prior to any corresponding guard times expiring"
// //	while (!(readb(ccport->port.membase + UARTSR1) & UARTSR1_TC))
// //		barrier();
// 
// 
// 	/* and restore CR2 */
// //	writeb(old_cr2, ccport->port.membase + UARTCR2);  
// }


/**
 * serves the console setup
 *
 * if the port was already initialised (eg, by a boot loader),
 * try to determine the current setup
 *
 * checks if TE and RE are enabled
 * checks for parity
 * checks for M value (8 or 9 bit shifter)
 * sets up bdh and bdl as needed for baud rate clock
 */
static void __init
iso7816_console_get_options(struct iso7816_port *ccport, int *baud,
			   int *parity, int *bits)
{

	unsigned char cr, bdh, bdl, brfa;
	unsigned int sbr, uartclk, baud_raw;

//	if (1 != ccport->port.minor) { // TODO check if needed              

	/* check if transmitter and receiver are enabled */
	cr = readb(ccport->port.membase + UARTCR2);
	cr &= UARTCR2_TE | UARTCR2_RE;
	if (!cr)
		return;

	/* ok, the port was enabled */
	cr = readb(ccport->port.membase + UARTCR1);

	/* parity setting */
	*parity = 'n';
	if (cr & UARTCR1_PE) {
		if (cr & UARTCR1_PT)
			*parity = 'o';
		else
			*parity = 'e';
	}

	/* 8 or 9 bit mode */
	if (cr & UARTCR1_M)
		*bits = 9;
	else
		*bits = 8;

//	}    

	/* bdh and bdl: setup as is in fsl_lpuart */
	bdh = readb(ccport->port.membase + UARTBDH);
	bdh &= UARTBDH_SBR;
	bdl = readb(ccport->port.membase + UARTBDL);
	sbr = bdh;
	sbr <<= 8;
	sbr |= bdl;
	brfa = readb(ccport->port.membase + UARTCR4);
	brfa &= UARTCR4_BRFA;
	uartclk = clk_get_rate(ccport->clk);
	/*
	 * baud = mod_clk/(16*(sbr[13]+(brfa)/32)
	 */
	baud_raw = uartclk / (16 * (sbr + brfa / 32));

	if (*baud != baud_raw)
		printk(KERN_INFO "ISO7816: Console iso7816 rounded baud rate"
				"from %d to %d\n", baud_raw, *baud);
}


/**
 * setup function placed in the struct console
 *
 * when the console option is configured, this __init becomes active
 */
static int __init iso7816_console_setup(struct console *co, char *options)
{
	struct iso7816_port *ccport;
// TODO check console configuration 	
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';


	printk(KERN_ERR "YYY iso7816::%s()\n", __func__); // TODO 1. test: module comes up            

	/* console index */
	if (co->index == -1 || co->index >= ARRAY_SIZE(iso7816_ports)) {
		pr_err( "ISO7816: console index '%d' for ccport is invalid\n", co->index);
		return -ENODEV;
	}

	/* ccport by console index */
	ccport = iso7816_ports[co->index];
	if (NULL == ccport) {
		pr_err( "ISO7816: ccport from array iso7816_ports was NULL\n");
		return -ENODEV;
	}

//	/* device */
//	do {
//		np = of_find_node_by_type( np, "serial"); // TODO check type            
//		if (!np)
//			return -ENODEV;
//
//
//		if (!of_device_is_compatible(np, "nxp,iso7816")) {
//			--idx;
//		}
//	} while (++idx != co->index);
//
//	/* gpios */
//	ret = iso7816_cc_init_gpios(np, ccport);
//	of_node_put(np);
//	if (ret)
//		return ret;

// TODO cpm: init struct tserial_core::port 	    

	/* evaluate options, or read out initialized uart e.g. by bootloader */
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		iso7816_console_get_options(ccport, &baud, &parity, &bits);

	/* reset uart */
	iso7816_setup_watermark(ccport);

// TODO why is the following at this location?  	
//	ret = platform_driver_register(&iso7816_driver); 
//	if (ret) 
//		uart_unregister_driver(&iso7816_reg); 


	/* finally, set options in serial core driver */
	return uart_set_options(&ccport->port, co, baud, parity, bits, flow);
}

// TODO          


/* structs - CONFIG_SERIAL_ISO7816_CONSOLE (after coresponding funcs) */
/* aka CONFIG_SERIAL_FSL_LPUART_CONSOLE */

static struct uart_driver iso7816_reg;
static struct console iso7816_console = {
	.name		= DEV_NAME,
	.write		= iso7816_console_write,
	.device		= uart_console_device, /* serial_core: returns struct tty_driver and inits index */
	.setup		= iso7816_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &iso7816_reg,
};


// TODO check if this is really needed
#define ISO7816_CONSOLE        &iso7816_console
#else
#define ISO7816_CONSOLE        NULL
#endif /* CONFIG_SERIAL_ISO7816_CONSOLE aka CONFIG_SERIAL_FSL_LPUART_CONSOLE */


/* structs - trailer */

static struct uart_driver iso7816_reg = {
	.owner          = THIS_MODULE,
	.driver_name    = DRIVER_NAME,
	.dev_name       = DEV_NAME,
/*	.major          = ISO7816_MAJOR, */
/*	.minor          = ISO7816_MINOR, */
	.cons           = ISO7816_CONSOLE,
	.nr		= ISO7816_UART_NR, // should be only one, but for bringup as a quickfix set '6' // */
};




/* functions - trailer */

/**
 * probe
 *
 * - inits the ccport struct
 * - inits teh ccport->port struct
 * - device node
 * - gpios
 * - pops
 * - ccport->clk and ccport->port.uartclk
 * - registers drv data
 */
static int iso7816_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct iso7816_port *ccport;
	struct resource *res;
	int ret;

        /* ccport */
	ccport = devm_kzalloc(&pdev->dev, sizeof(*ccport), GFP_KERNEL);

	
	DBG_FUNCNAME
	
	if (!ccport){
		pr_err( "YYY - ISO7816: failed to get port, %d\n", ret);
		return -ENOMEM;
	}

	/* device */
	ret = of_alias_get_id(np, "serial");
	if (ret < 0) {
		pr_err( "YYY - ISO7816: failed to get alias id, errno %d\n", ret);
		return ret;
	}
	ccport->port.line = ret;

	/* gpios */
	printk(KERN_ERR "YYY %s(): setting gpios...started\n", __func__);
	ret = iso7816_cc_init_gpios(np, ccport);
	of_node_put(np);
	if (ret) {
		printk(KERN_ERR "YYY %s(): setting gpios...FAILED\n", __func__);
                pr_err( "YYY - ISO7816: initialization of gpios failed, %d\n", ret);
		return ret;
	}
	printk(KERN_ERR "YYY %s(): probing gpios...done\n", __func__);

	/* power on */
	printk( KERN_ERR "YYY %s(): enabling card reader...", __func__);
       	gpio_set_value(ccport->gpio[ISO7816_GPIO_EN], 1);
	printk( KERN_ERR "YYY %s(): done\n", __func__);

	/* ccport->port - membase, device, pops,... */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ccport->port.membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ccport->port.membase))
		return PTR_ERR(ccport->port.membase);

	ccport->port.mapbase = res->start;
	ccport->port.dev = &pdev->dev;
	ccport->port.type = PORT_ISO7816;
	ccport->port.iotype = UPIO_MEM;
	ccport->port.irq = platform_get_irq(pdev, 0);
	ccport->port.ops = &iso7816_pops;
	ccport->port.flags = UPF_BOOT_AUTOCONF;

	/* ccport->clk */
	ccport->clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(ccport->clk)) {
		ret = PTR_ERR(ccport->clk);
		pr_err( "ISO7816: failed to get uart clk: %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(ccport->clk);
	if (ret) {
		pr_err( "ISO7816: failed to enable uart clk: %d\n", ret);
		return ret;
	}

	/* ccport->port.uartclk */
	ccport->port.uartclk = clk_get_rate(ccport->clk);

	/* ptr to ccport */
	iso7816_ports[ccport->port.line] = ccport;

	/* register uart port */
	platform_set_drvdata(pdev, &ccport->port);

	/* console, should be already set */
	iso7816_reg.cons = ISO7816_CONSOLE;

	ret = uart_add_one_port(&iso7816_reg, &ccport->port);
	if (ret) {
		clk_disable_unprepare(ccport->clk);
		return ret;
	}

// TODO rm - we don't need dma (currently)  	
//	if (!nodma) {
//		ccport->dma_tx_chan = iso7816_request_dma_chan(ccport, "tx");
//		ccport->dma_rx_chan = iso7816_request_dma_chan(ccport, "rx");
//	}

// TODO no rs485 for chipcard needed	
//	if (of_property_read_bool(np, "linux,rs485-enabled-at-boot-time")) {
//		ccport->port.rs485.flags |= SER_RS485_ENABLED;
//		ccport->port.rs485.flags |= SER_RS485_RTS_ON_SEND;
//		writeb(UARTMODEM_TXRTSE, ccport->port.membase + UARTMODEM);
//	}

        if (of_property_read_bool(np, "linux,iso7816-enabled-at-boot-time")) {
		printk(KERN_ERR "YYY DTB entry 'linux,iso7816-enabled-at-boot-time' - FOUND!!! YYY\n"); // YYY

	} else {
		if (1 == ccport->port.minor) {
			printk(KERN_ERR "YYY\n");
			printk(KERN_ERR "YYY DTB ERROR! Make sure to use the correct device tree.\n");
			printk(KERN_ERR "YYY\n");
		}
	}

	/* debugging */
//	iso7816_cc_report(ccport);

	/* initialization sequence */
	ret = iso7816_cc_initialize(ccport);
	if (ret) {
		printk(KERN_ERR "YYY %s: ISO7816 initialization failed!\n", __func__);
		clk_disable_unprepare(ccport->clk); // taken from iso7816_startup()
		return -1;
	}

	/* ATR sequence */
	ret = iso7816_cc_atr(ccport);
	if (ret) {
		printk(KERN_ERR "YYY %s: ISO7816 ATR failed!\n", __func__);
		clk_disable_unprepare(ccport->clk);
		return -1;
	}

        /* turn off TE & RE and on again */
	iso7816_cc_restart_re_te(ccport);

	/* debugging: read out FAB code */
//	iso7816_cc_read_fab(ccport); 
	
	/* debugging */
//	iso7816_cc_report(ccport);

	return 0;
}

/**
 * remove driver
 *
 * removes the port and disables the clock
 */
static int iso7816_remove(struct platform_device *pdev)
{
	struct iso7816_port *ccport = platform_get_drvdata(pdev);

	uart_remove_one_port(&iso7816_reg, &ccport->port);

	clk_disable_unprepare(ccport->clk);

// TODO currently no dma	
//	if (ccport->dma_tx_chan)
//		dma_release_channel(ccport->dma_tx_chan);
//	if (ccport->dma_rx_chan)
//		dma_release_channel(ccport->dma_rx_chan);

// TODO gpio_free();

	return 0;
}




/*   // TODO in case set up suspend and hibernation capability, use the following

#ifdef CONFIG_PM_SLEEP
static int iso7816_suspend(struct device *dev)
{
	struct iso7816_port *ccport = dev_get_drvdata(dev);
	unsigned long temp;
	
DBG_FUNCNAME

//	if (ccport->iso781632) {
////		/ * disable Rx/Tx and interrupts * /
//		temp = iso781632_read(ccport->port.membase + UARTCTRL);
//		temp &= ~(UARTCTRL_TE | UARTCTRL_TIE | UARTCTRL_TCIE);
//		iso781632_write(temp, ccport->port.membase + UARTCTRL);
//	} else {
//		/ * disable Rx/Tx and interrupts * /
		temp = readb(ccport->port.membase + UARTCR2);
		temp &= ~(UARTCR2_TE | UARTCR2_TIE | UARTCR2_TCIE);
		writeb(temp, ccport->port.membase + UARTCR2);
//	}

	uart_suspend_port(&iso7816_reg, &ccport->port);

	if (ccport->iso7816_dma_rx_use) {
//		/ *
//		 * EDMA driver during suspend will forcefully release any
//		 * non-idle DMA channels. If port wakeup is enabled or if port
//		 * is console port or 'no_console_suspend' is set the Rx DMA
//		 * cannot resume as as expected, hence gracefully release the
//		 * Rx DMA path before suspend and start Rx DMA path on resume.
//		 * /
		if (ccport->port.irq_wake) {
			del_timer_sync(&ccport->iso7816_timer);
			iso7816_dma_rx_free(&ccport->port);
		}

		// * Disable Rx DMA to use UART port as wakeup source * /
		writeb(readb(ccport->port.membase + UARTCR5) & ~UARTCR5_RDMAS,
					ccport->port.membase + UARTCR5);
	}

	if (ccport->iso7816_dma_tx_use) {
		ccport->dma_tx_in_progress = false;
		dmaengine_terminate_all(ccport->dma_tx_chan);
	}

	if (ccport->port.suspended && !ccport->port.irq_wake)
		clk_disable_unprepare(ccport->clk);

	return 0;
}

static int iso7816_resume(struct device *dev)
{
	struct iso7816_port *ccport = dev_get_drvdata(dev);
	unsigned long temp;
	
DBG_FUNCNAME

	if (ccport->port.suspended && !ccport->port.irq_wake)
		clk_prepare_enable(ccport->clk);

//	if (ccport->iso781632) {
//		iso781632_setup_watermark(ccport);
//		temp = iso781632_read(ccport->port.membase + UARTCTRL);
//		temp |= (UARTCTRL_RIE | UARTCTRL_TIE | UARTCTRL_RE |
//			 UARTCTRL_TE | UARTCTRL_ILIE);
//		iso781632_write(temp, ccport->port.membase + UARTCTRL);
//	} else {
		iso7816_setup_watermark(ccport);
		temp = readb(ccport->port.membase + UARTCR2);
		temp |= (UARTCR2_RIE | UARTCR2_TIE | UARTCR2_RE | UARTCR2_TE);
		writeb(temp, ccport->port.membase + UARTCR2);
//	}

	if (ccport->iso7816_dma_rx_use) {
		if (ccport->port.irq_wake) {
			if (!iso7816_start_rx_dma(ccport)) {
				ccport->iso7816_dma_rx_use = true;
				setup_timer(&ccport->iso7816_timer,
						iso7816_timer_func,
						(unsigned long)ccport);
				ccport->iso7816_timer.expires = jiffies +
						ccport->dma_rx_timeout;
				add_timer(&ccport->iso7816_timer);
			} else {
				ccport->iso7816_dma_rx_use = false;
			}
		}
	}

	if (ccport->dma_tx_chan && !iso7816_dma_tx_request(&ccport->port)) {
			init_waitqueue_head(&ccport->dma_wait);
			ccport->iso7816_dma_tx_use = true;
			writeb(readb(ccport->port.membase + UARTCR5) |
				UARTCR5_TDMAS, ccport->port.membase + UARTCR5);
	} else {
		ccport->iso7816_dma_tx_use = false;
	}

	uart_resume_port(&iso7816_reg, &ccport->port);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(iso7816_pm_ops, iso7816_suspend, iso7816_resume);
//*/

static struct platform_driver iso7816_driver = {
	.probe		= iso7816_probe,
	.remove		= iso7816_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = iso7816_dt_ids,
/*		.pm	= &iso7816_pm_ops,  // TODO is pm needed? */       
	},
};


static int __init iso7816_init(void)
{
	int ret;

	ret = uart_register_driver(&iso7816_reg);
	if (ret)
		return ret;

	ret = platform_driver_register(&iso7816_driver);
	if (ret)
		uart_unregister_driver(&iso7816_reg);

	return ret;
}


static void __exit iso7816_exit(void)
{
	platform_driver_unregister(&iso7816_driver);
	uart_unregister_driver(&iso7816_reg);
}

module_init(iso7816_init);
module_exit(iso7816_exit);

MODULE_AUTHOR("Lothar Rubusch");
MODULE_DESCRIPTION("ISO7816 chipcard driver for gpios, based on an nxp/freescale UART, rev: 0.00001 $");
MODULE_LICENSE("GPL v2");







