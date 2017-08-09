/*
 *  Driver for ISO7816 chipcards by gpio and NXP UART
 *
 *  Copyright (C) 2017 Lothar Rubusch <lothar.rubusch@bbv.ch>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#ifndef ISO7816_H
#define ISO7816_H


/* adresses */
/* */
/* All registers are 8-bit width */
#define UARTBDH			0x00 /* UART_BDH */
#define UARTBDL			0x01 /* UART_BDL */
#define UARTCR1			0x02 /* UART_C1 */
#define UARTCR2			0x03 /* UART_C2 */
#define UARTSR1			0x04 /* UART_S1 */
#define UARTSR2                 0x05 /* UART_S2 missing? */
#define UARTCR3			0x06 /* UART_C3 */
#define UARTDR			0x07 /* UART_D, actually 2 regs; 'reads' return  */
                                     /* the contents of the read-only receive  */
                                     /* data register and 'writes' go to the     */
                                     /* write-only transmit data register      */
/* UART_MA1 and UART_MA2 missing? */
#define UARTCR4			0x0a /* UART_C4 */
#define UARTCR5			0x0b /* C5 undocumented? */
/* UART_ED missing? */
#define UARTMODEM		0x0d /* MODEM undocumented? */
/* UART_IR missing? */
#define UARTPFIFO		0x10 /* UART_PFIFO, FIFO parameters */
#define UARTCFIFO		0x11 /* UART_CFIFO, FIFO control register */
#define UARTSFIFO		0x12 /* UART_SFIFO, FIFO status register */
#define UARTTWFIFO		0x13 /* UART_TWFIFO, trasmit watermark */
#define UARTTCFIFO		0x14 /* UART_TCFIFO, transmit count - read     */
                                     /* only reg that indicates how many       */
                                     /* datawords are currently in the         */
                                     /* transmit buffer/FIFO; may be read at   */
                                     /* any time */
#define UARTRWFIFO       	0x15 /* UART_RWFIFO, receive watermark */
/* UART_RCFIFO missing? */
#define UARTCR7816              0x18 /* UART 7816 Control Register */
#define UARTIE7816              0x19 /* UART 7816 Interrupt Enable Register */
#define UARTIS7816              0x1a /* UART 7816 Interrupt Status Register */
#define UARTWP7816T1            0x1b /* UART 7816 Wait Parameter Register */
#define UARTWN7816              0x1c /* UART 7816 Wait N Register */
#define UARTWF7816              0x1d /* UART 7816 Wait FD Register */
#define UARTET7816              0x1e /* UART 7816 Error Threshold Register */
#define UARTTL7816              0x1f /* UART 7816 Transmit Length Register */


/* register flags */

/* UART_BDH */
#define UARTBDH_LBKDIE		0x80
#define UARTBDH_RXEDGIE		0x40
#define UARTBDH_SBR	        0x1f

/* UART_BDL */
#define UARTBDL_SBR             0xff

/* UART_C1 flags */
#define UARTCR1_LOOPS		0x80 /* Loop Mode Select */
#define UARTCR1_RSRC		0x20 /* Receiver Source Select, 1 = single wire UART mode where the receiver input is connected to the transmit pin input signal */
#define UARTCR1_M		0x10 /* 9-bit or 8-bit Mode Select, 1 = use start + 9 data bits (MSB/LSB first as determined by MSBF) + stop */
#define UARTCR1_WAKE		0x08 /* Receiver Wakeup Method Select, 1 = Address mark wakeup */
#define UARTCR1_ILT		0x04 /* Idle Line Type Select, 1 = Idle character bit count starts after stop bit */
#define UARTCR1_PE		0x02 /* Parity Enable, 1 = Parity function enabled */
#define UARTCR1_PT		0x01 /* Parity Type, 1 = Odd parity */

/* UART_C2 flags */
#define UARTCR2_TIE		0x80 /* transmit interrupt enable */
                                     /* or DMA transfer enable, */
                                     /* needs C5[TDMAS]=1 and C2[TCIE]=0 */
#define UARTCR2_TCIE		0x40 /* in documentation reserved to 0?! */
#define UARTCR2_RIE		0x20 /* receiver full interrupt */
                                     /* or DMA transfer enable */
#define UARTCR2_ILIE		0x10 /* idle line interrupt enable */
#define UARTCR2_TE		0x08 /* transmitter enable */
                                     /* when C7816[ISO_7816E] is enabled */
                                     /* and C7816[TTYPE]=1, this field is */
                                     /* automatically cleared after the */
                                     /* requested block has been transmitted. */
                                     /* This condition is detected when */
                                     /* TL7816[TLEN]=0 and four additional */
                                     /* characters are transmitted */
#define UARTCR2_RE		0x04 /* receiver enable */
#define UARTCR2_RWU		0x02 /* receiver wakeup control, */
                                     /* normal operation = 0 */
#define UARTCR2_SBK		0x01 /* send break */

/* UART_S1 flags */
#define UARTSR1_TDRE		0x80
#define UARTSR1_TC		0x40
#define UARTSR1_RDRF		0x20
#define UARTSR1_IDLE		0x10
#define UARTSR1_OR		0x08
#define UARTSR1_NF		0x04
#define UARTSR1_FE		0x02
#define UARTSR1_PE		0x01

/* UART_S2 flags */
#define UARTSR2_RXEDGIF         0x40
#define UARTSR2_MSBF            0x20
#define UARTSR2_RXINV           0x10
#define UARTSR2_RWUID           0x08 /* receive wakeup idle detect */
#define UARTSR2_BRK13           0x04
#define UARTSR2_RAF             0x01

/* UART_C3 flags */
#define UARTCR3_R8		0x80
#define UARTCR3_T8		0x40
#define UARTCR3_TXDIR		0x20
#define UARTCR3_TXINV		0x10
#define UARTCR3_ORIE		0x08
#define UARTCR3_NEIE		0x04
#define UARTCR3_FEIE		0x02
#define UARTCR3_PEIE		0x01

/* UART_C4 flags */
#define UARTCR4_MAEN1		0x80 /* match address enable 1 */
#define UARTCR4_MAEN2		0x40 /* match address enable 2 */
#define UARTCR4_M10		0x20
#define UARTCR4_BRFA    	0x1f

/* UART_C5 flags (not documented) */
#define UARTCR5_TDMAS		0x80 /* seems to be transfer DMA enable */
#define UARTCR5_RDMAS		0x20 /* seems to be receive DMA enable */

/* UART_MODEM flags (not documented) */
#define UARTMODEM_RXRTSE	0x08
#define UARTMODEM_TXRTSPOL	0x04
#define UARTMODEM_TXRTSE	0x02
#define UARTMODEM_TXCTSE	0x01

/* UART_PFIFO flags */
#define UARTPFIFO_TXFE		0x80
#define UARTPFIFO_FIFOSIZE_MASK	0x7
#define UARTPFIFO_TXSIZE_OFF	4
#define UARTPFIFO_RXFE		0x08
#define UARTPFIFO_RXSIZE_OFF	0

/* UART_CFIFO flags */
#define UARTCFIFO_TXFLUSH	0x80
#define UARTCFIFO_RXFLUSH	0x40
#define UARTCFIFO_RXOFE		0x04
#define UARTCFIFO_TXOFE		0x02
#define UARTCFIFO_RXUFE		0x01

/* UART_SFIFO flags */
#define UARTSFIFO_TXEMPT	0x80
#define UARTSFIFO_RXEMPT	0x40
#define UARTSFIFO_RXOF		0x04
#define UARTSFIFO_TXOF		0x02
#define UARTSFIFO_RXUF		0x01

/* UARTCR7816 flags */
#define UARTCR7816_ONACK         0x10 /* enable NACK on overflow */
#define UARTCR7816_ANACK         0x08 /* enalbe NACK on error */
#define UARTCR7816_INIT          0x04 /* detect initial character */
                                     /* (normal mode) */
#define UARTCR7816_TTYPE         0x02 /* ISO7816 TTYPE, */
                                     /* C7816[TTYPE]=0 implies */
                                     /* C2[TE]=1 and TXDIR used */
                                     /* i.e. serial transmission */
                                     /* C7816[TTYPE]=1 implies */
                                     /* C2[TE]=1 and C2[RE]=1 */
                                     /* i.e. data is transferred in blocks of */
                                     /* size TL7816[TLEN] and in case a CRC */
#define UARTC7816_ISO_7816E     0x01 /* enable ISO7816 mode */

/* UARTIE7816 flags */
#define UARTIE7816_WTE          0x80 /* wait timer interrupt enable */
#define UARTIE7816_CWTE         0x40 /* character wait timer interrupt enable */
#define UARTIE7816_BWTE         0x20 /* block wait timer interrupt enable */
#define UARTIE7816_INITDE       0x10 /* initial character detected interrupt */
                                     /* enable */
#define UARTIE7816_GTVE         0x04 /* guard timer violated interrupt enable */
#define UARTIE7816_TXTE         0x02 /* transmit threshold exceeded interrupt */
                                     /* enable */
#define UARTIE7816_RXTE         0x01 /* receive threshold exceeded interrupt */
                                     /* enable */

#define UARTIS7816_INIT         0x10 /* initial character was detected */

/* UARTET7816 flags */
#define UART1ET7816_TXTHRESHOLD 0xf0 /* transmit NACK threshold */
                                     /* only if C7816[TTYPE] == 0 */
                                     /* and C7816[ANACK] == 1 */
#define UART1ET7816_RXTHRESHOLD 0x0f /* receive NACK threshold */
                                     /* only if C7816[TTYPE] == 0 */



#endif  /* ISO7816_H */
