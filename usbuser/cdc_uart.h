

#include "main.h"

#include "usbd_core.h"
#include "usb_dc.h"
#include "usbd_cdc.h"

/******************************************************************************/

#define TXBUF_SIZE  2048
#define RXBUF_SIZE  256

typedef struct _CDC_UART{
	volatile UART_REGS *uart;
	int pclk;
	struct cdc_line_coding linecoding;
	int config_req;
	int pause;

	int intf_num;
	struct usbd_interface intf0;
	struct usbd_interface intf1;
	struct usbd_endpoint out_ep;
	struct usbd_endpoint in_ep;

	volatile DMA_REGS *dma;

	// USB_RXD -> UART_TXD
	uint8_t *usb_rxbuf;
	uint8_t *uart_txbuf;
	volatile DMACH_REGS *txdma;
	int txch;

	int crx_size;
	int utx_en;
	int utx_rp;
	int utx_nextrp;
	int utx_wp;

	// UART_RXD -> USB_TXD
	uint8_t *usb_txbuf;
	uint8_t *uart_rxbuf;
	volatile DMACH_REGS *rxdma;
	int rxch;

	int urx_hp;
	int ctx_nextrp;
	int ctx_rp;
	int ctx_wp;
	int ctx_en;
	int ctx_lock;

}CDC_UART;

extern CDC_UART cdc_uarts[];


/******************************************************************************/


void uart_irq_handle(CDC_UART *cu);
void dma_irq_handle(CDC_UART *cu);

CDC_UART *get_cdcuart(int ep, int intf);
void cdcuart_reset(CDC_UART *cu);
void cdcuart_init(CDC_UART *cu);

void cdc_recv_start(CDC_UART *cu);
void cdc_send_startup(CDC_UART *cu);
