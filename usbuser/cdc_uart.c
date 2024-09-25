

#include "main.h"
#include "cdc_uart.h"

/******************************************************************************/

#if 0
static int logbuf[512];
static int logp = 0;

void cdclog(int pos, int rp, int wp)
{
	if(logp<512){
		logbuf[logp] = (pos<<12) | (rp) | (wp<<16);
		logp += 1;
	}
}

int printd(char *fmt, ...);
void cdclog_show(void)
{
	int i;
	for(i=0; i<logp; i++){
		int d = logbuf[i];
		int t = (d>>12)&0x0f;
		if(t<=3){
			printd("log%d:  rp=%3d  wp=%3d\n", t, d&0x0fff, d>>16);
			if(t==3){
				printk("\n");
			}
		}else{
			printd("log%d:  hp=%3d  sz=%3d\n", t, d&0x0fff, d>>16);
		}
	}
	logp = 0;
}
#endif

/******************************************************************************/


static void uart_txbuf_put(CDC_UART *cu, uint8_t *data, int length);
static void uart_config(CDC_UART *cu);


static void cdc_send_start(CDC_UART *cu)
{
	int length;

	if(cu->config_req || cu->ctx_en)
		return;

	//cdclog(1, cu->ctx_rp, cu->ctx_wp);
	if(cu->ctx_rp==0 && cu->ctx_wp==2){
		// txbuf为空
		return;
	}else if(cu->ctx_rp < cu->ctx_wp){
		// 数据在txbuf中间. [        r.........w        ]
		length = cu->ctx_wp - cu->ctx_rp;
		if(length>USB_PACKET_SIZE){
			// 本次数据不能全部传输。
			// 只传510个数据。rp指向508，留出FTDI的数据头。
			length = (USB_PACKET_SIZE-2);
			cu->ctx_nextrp = cu->ctx_rp + USB_PACKET_SIZE-4;
		}else{
			// 数据将全部传输完。
			cu->ctx_nextrp = cu->ctx_rp + (length+3)&~3;
			cu->ctx_wp = cu->ctx_nextrp + 2;
		}
		uint8_t *tbuf = cu->usb_txbuf+cu->ctx_rp;
		tbuf[0] = 0x02;
		tbuf[1] = 0x61;
		usbd_ep_start_write(cu->in_ep.ep_addr, tbuf, length);
		cu->ctx_en = 1;
	}else{
		// 数据在txbuf两端. [........w         r........]
		length = TXBUF_SIZE - cu->ctx_rp;
		if(length>USB_PACKET_SIZE){
			// 右端数据不能全部传输。
			// 只传510个数据。rp指向508，留出FTDI的数据头。
			length = USB_PACKET_SIZE-2;
			cu->ctx_nextrp = cu->ctx_rp + USB_PACKET_SIZE-4;
		}else{
			// 右端数据将全部传输完。
			cu->ctx_nextrp = 0;
		}
		uint8_t *tbuf = cu->usb_txbuf+cu->ctx_rp;
		tbuf[0] = 0x02;
		tbuf[1] = 0x61;
		usbd_ep_start_write(cu->in_ep.ep_addr, tbuf, length);
		cu->ctx_en = 1;
	}
}


static void cdc_txbuf_put(CDC_UART *cu, uint8_t *data, int length)
{
	int remain;

	//printk("cdc_txbuf_put: length=%d  rp=%d wp=%d\n", length, cu->ctx_rp, cu->ctx_wp);

	//cdclog(2, cu->ctx_rp, cu->ctx_wp);
	if(cu->ctx_wp < cu->ctx_rp){
		// 数据在txbuf两端. [........w         r........]
		remain = cu->ctx_rp - cu->ctx_wp -1;
		if(length>remain)
			length = remain;
		memcpy(cu->usb_txbuf + cu->ctx_wp, data, length);
		cu->ctx_wp += length;
	}else{
		// 数据在txbuf中间. [        r.........w        ]
		remain = TXBUF_SIZE - cu->ctx_wp;
		if(remain<0)
			remain = 0;
		if(remain>length)
			remain = length;
		if(remain){
			// 填充右端空间
			memcpy(cu->usb_txbuf + cu->ctx_wp, data, remain);
			length -= remain;
			data += remain;
			cu->ctx_wp += remain;
		}

		if(length>0){
			// 填充左端空间. 要留出FTDI头部数据空间.
			remain = cu->ctx_rp-2-1;
			if(length > remain)
				length = remain;
			memcpy(cu->usb_txbuf+2, data, length);
			cu->ctx_wp = length+2;
		}
	}

	cdc_send_start(cu);
}


void cdc_recv_start(CDC_UART *cu)
{
	usbd_ep_start_read(cu->out_ep.ep_addr, cu->usb_rxbuf, USB_PACKET_SIZE);
}

// UART_RXBUF -> USB_TXD
static void cdc_bulk_in_cb(uint8_t ep, uint32_t nbytes)
{
	CDC_UART *cu = get_cdcuart(ep, 0);

	if(nbytes==USB_PACKET_SIZE){
		// Send ZLP
		usbd_ep_start_write(cu->in_ep.ep_addr, NULL, 0);
		return;
	}

	cu->ctx_en = 0;
	cu->ctx_rp = cu->ctx_nextrp;
	//cdclog(3, cu->ctx_rp, cu->ctx_wp);
	if(cu->ctx_rp+2 == cu->ctx_wp){
		cu->ctx_rp = 0;
		cu->ctx_nextrp = 0;
		cu->ctx_wp = 2;

		// 发送空数据包.
		uint8_t *tbuf = cu->usb_txbuf+cu->ctx_rp;
		tbuf[0] = 0x02;
		tbuf[1] = 0x60;
		usbd_ep_start_write(cu->in_ep.ep_addr, tbuf, 2);
		cu->ctx_en = 1;
	}else{
		//cdclog(3, cu->ctx_rp, cu->ctx_wp);
		cdc_send_start(cu);
	}
}


void cdc_send_startup(CDC_UART *cu)
{
	uint8_t *tbuf = cu->usb_txbuf+cu->ctx_rp;
	tbuf[0] = 0x02;
	tbuf[1] = 0x60;
	usbd_ep_start_write(cu->in_ep.ep_addr, tbuf, 2);
}


// USB_RXD -> UART_TXBUF
static void cdc_bulk_out_cb(uint8_t ep, uint32_t nbytes)
{
	CDC_UART *cu = get_cdcuart(ep, 0);

	cu->crx_size = nbytes;
	uart_txbuf_put(cu, cu->usb_rxbuf, nbytes);
	if(cu->crx_size==0){
		usbd_ep_start_read(cu->out_ep.ep_addr, cu->usb_rxbuf, USB_PACKET_SIZE);
	}
}


/******************************************************************************/

static void uart_recv_start(CDC_UART *cu)
{
	cu->urx_hp = 0;

	cu->rxdma->CFGR  = 0;
	cu->rxdma->CNTR  = RXBUF_SIZE;
	cu->rxdma->PADDR = (uint32_t)&(cu->uart->DATAR);
	cu->rxdma->MADDR = (uint32_t)cu->uart_rxbuf;

	cu->uart->STATR &= ~0x0010; // clear IDLE
	cu->uart->CTLR3 |=  0x0040; // Enable DMAR
	cu->uart->CTLR1 |=  0x0014; // Enable IDLE|RXE

	cu->rxdma->CFGR  = 0x10a7;  // byte, P2M, MINC, Cycle, HTIE, TCIE
}


static void uart_dma_send(CDC_UART *cu, void *data, int length)
{
	cu->txdma->CFGR  = 0;
	cu->txdma->CNTR  = length;
	cu->txdma->PADDR = (uint32_t)&(cu->uart->DATAR);
	cu->txdma->MADDR = (uint32_t)data;

	cu->uart->STATR &= ~0x0040; // clear TC
	cu->uart->CTLR3 |=  0x0080; // Enable DMAT
	cu->uart->CTLR1 |=  0x0040; // Enable TCIE
	cu->utx_en = 1;

	cu->txdma->CFGR  = 0x0091;  // byte, M2P, MINC
}

static void uart_send_start(CDC_UART *cu)
{
	int size = 0;

	if(cu->utx_en)
		return;
	if(cu->utx_rp == cu->utx_wp)
		return;

	if(cu->utx_rp > cu->utx_wp){
		// 数据在txbuf两端. [........w         r........]
		size = TXBUF_SIZE-cu->utx_rp;
	}else if(cu->utx_rp < cu->utx_wp){
		// 数据在txbuf中间. [        r.........w        ]
		size = cu->utx_wp-cu->utx_rp;
	}
	if(size>512){
		size = 512;
	}

	cu->utx_nextrp = cu->utx_rp + size;
	if(cu->utx_nextrp == TXBUF_SIZE){
		cu->utx_nextrp = 0;
	}
	uart_dma_send(cu, cu->uart_txbuf+cu->utx_rp, size);
}


void uart_irq_handle(CDC_UART *cu)
{
	if(cu->uart->STATR & 0x0010){
		// RX:IDLE
		(void)cu->uart->DATAR;
		int size = RXBUF_SIZE - cu->rxdma->CNTR - cu->urx_hp;
		//cdclog(4, cu->urx_hp, size);
		cdc_txbuf_put(cu, cu->uart_rxbuf+cu->urx_hp, size);
		cu->urx_hp += size;
	}
	if(cu->uart->STATR & 0x0040){
		// TX:TC
		cu->dma->INTFCR = 0x0f<<(4*cu->txch);
		cu->txdma->CFGR = 0;

		cu->uart->CTLR1 &= ~0x0040; // Disable TCIE
		cu->uart->STATR &= ~0x0040; // clear TC
		cu->uart->CTLR3 &= ~0x0080; // DIsable DMAT

		cu->utx_en = 0;
		cu->utx_rp = cu->utx_nextrp;
		if(cu->utx_rp == cu->utx_wp){
			cu->utx_rp = 0;
			cu->utx_wp = 0;
		}

		if(cu->config_req){
			cdcuart_reset(cu);
			uart_recv_start(cu);
			cu->config_req = 0;
		}

		if(cu->crx_size){
			uart_txbuf_put(cu, cu->usb_rxbuf, cu->crx_size);
			if(cu->crx_size==0){
				usbd_ep_start_read(cu->out_ep.ep_addr, cu->usb_rxbuf, USB_PACKET_SIZE);
			}else{
				uart_send_start(cu);
			}
		}else{
			uart_send_start(cu);
		}
	}
}


void dma_irq_handle(CDC_UART *cu)
{
	int intsr = cu->dma->INTFR;
	int ht_mask = 4<<(4*cu->rxch);
	int tc_mask = 2<<(4*cu->rxch);

	if(intsr&ht_mask){
		// Half Trans
		int size = RXBUF_SIZE - cu->rxdma->CNTR - cu->urx_hp;
		//cdclog(5, cu->urx_hp, size);
		cdc_txbuf_put(cu, cu->uart_rxbuf+cu->urx_hp, size);
		cu->urx_hp += size;
		cu->dma->INTFCR = ht_mask;
	}else if(intsr&tc_mask){
		// Trans Complete
		int size = RXBUF_SIZE - cu->urx_hp;
		//cdclog(6, cu->urx_hp, size);
		cdc_txbuf_put(cu, cu->uart_rxbuf+cu->urx_hp, size);
		cu->urx_hp = 0;
		cu->dma->INTFCR = tc_mask;
	}
}


static void uart_txbuf_put(CDC_UART *cu, uint8_t *data, int length)
{
	int remain;

	if(cu->utx_wp < cu->utx_rp){
		// 数据在txbuf两端. [........w         r........]
		remain = cu->utx_rp - cu->utx_wp -1;
		if(length>remain){
			return;
		}
		cu->crx_size = 0;
		memcpy(cu->uart_txbuf + cu->utx_wp, data, length);
		cu->utx_wp += length;
	}else{
		// 数据在txbuf中间. [        r.........w        ]
		remain = TXBUF_SIZE - cu->utx_wp + cu->utx_rp -1;
		if(length>remain){
			return;
		}
		cu->crx_size = 0;

		remain = TXBUF_SIZE - cu->utx_wp;
		if(remain>length)
			remain = length;
		if(remain){
			memcpy(cu->uart_txbuf + cu->utx_wp, data, remain);
			length -= remain;
			data += remain;
			cu->utx_wp += remain;
		}

		if(length>0){
			memcpy(cu->uart_txbuf, data, length);
			cu->utx_wp = length;
		}
	}

	uart_send_start(cu);
}


/******************************************************************************/


static void uart_config(CDC_UART *cu)
{
	printk("uart_config ...\n");
	int baudrate = cu->linecoding.dwDTERate;
	int databits = cu->linecoding.bDataBits;
	int stopbits = cu->linecoding.bCharFormat;
	int parity   = cu->linecoding.bParityType;

	if(baudrate==0)
		return;

	int div = (cu->pclk + baudrate/2)/baudrate;
	int ctrl1 = 0x2008;
	int ctrl2 = 0x0000;

	if(databits==9) ctrl1 |= 0x1000;
	if(stopbits==2) ctrl2 |= 0x2000;
	if(stopbits==1) ctrl2 |= 0x3000;
	if(parity!=0)   ctrl1 |= 0x0400;
	if(parity==1)   ctrl1 |= 0x0200;

	cu->uart->CTLR1 = 0;
	cu->uart->STATR = 0;
	(void)cu->uart->STATR;
	(void)cu->uart->DATAR;
	cu->uart->CTLR2 = ctrl2;
	cu->uart->CTLR3 = 0;
	cu->uart->BRR = div;
	cu->uart->CTLR1 = ctrl1;
}


void usbd_cdc_acm_set_line_coding(uint8_t intf, struct cdc_line_coding *line_coding)
{
	CDC_UART *cu = get_cdcuart(0, intf);

	printk("CDC set line_coding: %d %d-%d-%d\n",
		line_coding->dwDTERate, line_coding->bDataBits, line_coding->bParityType, line_coding->bCharFormat);

	if(line_coding->dwDTERate){
		cu->linecoding.dwDTERate = line_coding->dwDTERate;
	}
	if(line_coding->bDataBits){
		cu->linecoding.bDataBits = line_coding->bDataBits;
		cu->linecoding.bParityType = line_coding->bParityType;
		cu->linecoding.bCharFormat = line_coding->bCharFormat;
	}

	if(cu->utx_en==0){
		cdcuart_reset(cu);
		uart_recv_start(cu);
	}else{
		cu->config_req = 1;
	}

}


void usbd_cdc_acm_get_line_coding(uint8_t intf, struct cdc_line_coding *line_coding)
{
	CDC_UART *cu = get_cdcuart(0, intf);
	memcpy(line_coding, (uint8_t *)&cu->linecoding, sizeof(struct cdc_line_coding));
}


/******************************************************************************/

void cdc_pause(int id)
{
	CDC_UART *cu = &cdc_uarts[id];

	if(cu->pause==0){
		cu->pause = 1;
		cdcuart_reset(cu);
	}
}

void cdc_resume(int id)
{
	CDC_UART *cu = &cdc_uarts[id];

	if(cu->pause==1){
		cu->pause = 0;
		cdc_recv_start(cu);
		uart_recv_start(cu);
	}
}


void cdcuart_reset(CDC_UART *cu)
{
	printk("cdcuart_reset ...\n");

	cu->utx_en = 0;
	cu->utx_rp = 0;
	cu->utx_nextrp = 0;
	cu->utx_wp = 0;

	cu->ctx_en = 0;
	cu->ctx_rp = 0;
	cu->ctx_nextrp = 0;
	cu->ctx_wp = 2;
	cu->urx_hp = 0;

	cu->rxdma->CFGR = 0;
	cu->txdma->CFGR = 0;
	cu->dma->INTFCR = 0x0f<<(4*cu->rxch);
	cu->dma->INTFCR = 0x0f<<(4*cu->txch);

	uart_config(cu);
}


void cdcuart_init(CDC_UART *cu)
{
	printk("\ncdcuart_init ...\n");

	usbd_add_interface(usbd_cdc_acm_init_intf(&cu->intf0));
//	usbd_add_interface(usbd_cdc_acm_init_intf(&cu->intf1));

	cu->out_ep.ep_cb = cdc_bulk_out_cb;
	usbd_add_endpoint(&cu->out_ep);
	cu->in_ep.ep_cb = cdc_bulk_in_cb;
	usbd_add_endpoint(&cu->in_ep);

	cu->linecoding.dwDTERate = 115200;
	cu->linecoding.bDataBits = 8;
	cu->linecoding.bParityType = 0;
	cu->linecoding.bCharFormat = 0;

	cdcuart_reset(cu);
}

