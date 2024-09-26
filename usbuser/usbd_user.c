

#include "main.h"

#include "usbd_core.h"
#include "cdc_uart.h"


/******************************************************************************/


#define USBD_VID           0x0403
#define USBD_PID           0x6010
#define USBD_MAX_POWER     100


/* 注意: FTDI的驱动要求in与out端点号不能是同一个。 */

#define JTAG_IN_EP  0x81
#define JTAG_OUT_EP 0x02
#define JTAG_INTF   0
#define JTAG_INTERFACE_SIZE (9 + 7 + 7)


#define UART_IN_EP  0x83
#define UART_OUT_EP 0x04
#define UART_INT_EP 0x85
#define UART_INTF   1
#define UART_INTERFACE_SIZE (9 + 7 + 7)


#define USB_CONFIG_SIZE (9 + JTAG_INTERFACE_SIZE + UART_INTERFACE_SIZE)
#define INTF_NUM        2

/*!< global descriptor */
static uint8_t ftlink_descriptor[] = {
	USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0500, 0x01),
	USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, INTF_NUM, 1, 0x80, USBD_MAX_POWER),

 	/* Interface 0 : JTAG */
	USB_INTERFACE_DESCRIPTOR_INIT(JTAG_INTF, 0, 2, 0xFF, 0xFF, 0xFF, 2),
	USB_ENDPOINT_DESCRIPTOR_INIT(JTAG_IN_EP,  USB_ENDPOINT_TYPE_BULK, USB_PACKET_SIZE, 0x01),
	USB_ENDPOINT_DESCRIPTOR_INIT(JTAG_OUT_EP, USB_ENDPOINT_TYPE_BULK, USB_PACKET_SIZE, 0x01),

	/* Interface 1 : UART */
	USB_INTERFACE_DESCRIPTOR_INIT(UART_INTF, 0, 2, 0xFF, 0xFF, 0xFF, 4),
	USB_ENDPOINT_DESCRIPTOR_INIT(UART_IN_EP,  USB_ENDPOINT_TYPE_BULK, USB_PACKET_SIZE, 0x01),
	USB_ENDPOINT_DESCRIPTOR_INIT(UART_OUT_EP, USB_ENDPOINT_TYPE_BULK, USB_PACKET_SIZE, 0x01),

#ifdef CONFIG_USB_HS
    /* device qualifier descriptor */
    0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00,
#endif
    0x00
};


static struct usb_string_descriptor str0 = USB_ARRAY_DESC(0x0409);
static struct usb_string_descriptor str1 = USB_STRING_DESC("CherryUSB");
static struct usb_string_descriptor str2 = USB_STRING_DESC("FTLINK JTAG");
static struct usb_string_descriptor str3 = USB_STRING_DESC("2024092001");
static struct usb_string_descriptor str4 = USB_STRING_DESC("FTLINK UART");

static uint8_t *string_desc[] = {
	(uint8_t*)&str0,
	(uint8_t*)&str1,
	(uint8_t*)&str2,
	(uint8_t*)&str3,
	(uint8_t*)&str4,
};
static int string_cnt = sizeof(string_desc)/sizeof(uint8_t*);


/******************************************************************************/


static void jtag_out_callback(uint8_t ep, uint32_t nbytes);
static void jtag_in_callback(uint8_t ep, uint32_t nbytes);

static struct usbd_endpoint jtag_out_ep = {
	.ep_addr = JTAG_OUT_EP,
	.ep_cb = jtag_out_callback
};

static struct usbd_endpoint jtag_in_ep = {
	.ep_addr = JTAG_IN_EP,
	.ep_cb = jtag_in_callback
};


struct usbd_interface jtag_intf;


typedef struct {
	int bitmode;
	int latency_timer;
	int timeout;
}FTDEV;

FTDEV ftdevs[2];



/******************************************************************************/


#define JTAG_PACKET_SIZE  USB_PACKET_SIZE

static uint8_t jtag_req_buf[JTAG_PACKET_SIZE];
static uint8_t jtag_resp_buf[JTAG_PACKET_SIZE];
static volatile int jtag_req_size;
volatile int jtag_resp_size;


static void jtag_out_start(void)
{
	usbd_ep_start_read(JTAG_OUT_EP, jtag_req_buf, JTAG_PACKET_SIZE);
}

static void jtag_out_callback(uint8_t ep, uint32_t nbytes)
{
	jtag_req_size = nbytes;
	//printk("out: %d\n", nbytes);
}

static void jtag_in_start(void)
{
	jtag_resp_size = 2;
	jtag_resp_buf[0] = 0x02;
	jtag_resp_buf[1] = 0x60;
	usbd_ep_start_write(JTAG_IN_EP, jtag_resp_buf, jtag_resp_size);
}

static void jtag_in_callback(uint8_t ep, uint32_t nbytes)
{
	jtag_resp_size = 0;
	ftdevs[0].timeout = time_ms + ftdevs[0].latency_timer;
}


static void jtag_init(void)
{
	jtag_req_size = 0;
	jtag_resp_size = 0;
}


int jtag_execute(uint8_t *req, int req_size, uint8_t *resp);

void jtag_handle(void)
{
	if(jtag_req_size==0)
		return;


	while(jtag_resp_size);

	int resp_size = jtag_execute(jtag_req_buf, jtag_req_size, jtag_resp_buf+2);
	if(resp_size){
		jtag_resp_size = resp_size+2;
		jtag_resp_buf[0] = 0x02;
		jtag_resp_buf[1] = 0x61;
		usbd_ep_start_write(JTAG_IN_EP, jtag_resp_buf, jtag_resp_size);
	}

	jtag_req_size = 0;
	jtag_out_start();
}


/******************************************************************************/


static uint8_t uart_txbuf[TXBUF_SIZE];
static uint8_t usb_rxbuf[USB_PACKET_SIZE];
static uint8_t uart_rxbuf[RXBUF_SIZE];
static uint8_t usb_txbuf[TXBUF_SIZE];


CDC_UART cdc_uarts[] = {
	{
#if 0
		.uart = UART3,
		.pclk = APB1CLK_FREQ,
		.dma  = DMA1,
		.txch = 1,
		.txdma = &DMA1->CH[1],
		.rxch = 2,
		.rxdma = &DMA1->CH[2],
		.intf_num = UART_INTF,
		.out_ep = {
			.ep_addr = UART_OUT_EP,
		},
		.in_ep = {
			.ep_addr = UART_IN_EP,
		},
		.usb_rxbuf  = usb_rxbuf,
		.uart_txbuf = uart_txbuf,
		.usb_txbuf  = usb_txbuf,
		.uart_rxbuf = uart_rxbuf,
#endif
	},

	{
		.uart = UART1,
		.pclk = APB2CLK_FREQ,
		.dma  = DMA1,
		.txch = 3,
		.txdma = &DMA1->CH[3],
		.rxch = 4,
		.rxdma = &DMA1->CH[4],
		.intf_num = UART_INTF,
		.out_ep = {
			.ep_addr = UART_OUT_EP,
		},
		.in_ep = {
			.ep_addr = UART_IN_EP,
		},
		.usb_rxbuf  = usb_rxbuf,
		.uart_txbuf = uart_txbuf,
		.usb_txbuf  = usb_txbuf,
		.uart_rxbuf = uart_rxbuf,
	},

};


CDC_UART *get_cdcuart(int ep, int intf)
{
	return &cdc_uarts[1];
}


void dma1_channel3_irqhandler(void)
{
	// UART3.RXDMA
	dma_irq_handle(&cdc_uarts[0]);
}


void dma1_channel5_irqhandler(void)
{
	// UART1.RXDMA
	dma_irq_handle(&cdc_uarts[1]);
}


void usart3_irqhandler(void)
{
	uart_irq_handle(&cdc_uarts[0]);
}


void usart1_irqhandler(void)
{
	uart_irq_handle(&cdc_uarts[1]);
}


int cdc_in_lock(int *val)
{
	return __sync_val_compare_and_swap(val, 0, 1);
}

void cdc_in_unlock(int *val)
{
	*val = 0;
}


/******************************************************************************/


/* Requests */
#define SIO_RESET_REQUEST             0x00 /* Reset the port */
#define SIO_SET_MODEM_CTRL_REQUEST    0x01 /* Set the modem control register */
#define SIO_SET_FLOW_CTRL_REQUEST     0x02 /* Set flow control register */
#define SIO_SET_BAUDRATE_REQUEST      0x03 /* Set baud rate */
#define SIO_SET_DATA_REQUEST          0x04 /* Set the data characteristics of the port */
#define SIO_POLL_MODEM_STATUS_REQUEST 0x05
#define SIO_SET_EVENT_CHAR_REQUEST    0x06
#define SIO_SET_ERROR_CHAR_REQUEST    0x07
#define SIO_SET_LATENCY_TIMER_REQUEST 0x09
#define SIO_GET_LATENCY_TIMER_REQUEST 0x0A
#define SIO_SET_BITMODE_REQUEST       0x0B
#define SIO_READ_PINS_REQUEST         0x0C
#define SIO_READ_EEPROM_REQUEST       0x90
#define SIO_WRITE_EEPROM_REQUEST      0x91
#define SIO_ERASE_EEPROM_REQUEST      0x92


static uint16_t ftdi_eeprom_info[] =
{
	0x0800, 0x0403, 0x6010, 0x0500, 0x3280, 0x0000, 0x0200, 0x1096,
	0x1aa6, 0x0000, 0x0046, 0x0310, 0x004f, 0x0070, 0x0065, 0x006e,
	0x002d, 0x0045, 0x0043, 0x031a, 0x0055, 0x0053, 0x0042, 0x0020,
	0x0044, 0x0065, 0x0062, 0x0075, 0x0067, 0x0067, 0x0065, 0x0072,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1027 
};


void cdc_soft_timer(void)
{
	if(ftdevs[0].timeout){
		if(time_ms>=ftdevs[0].timeout){
			ftdevs[0].timeout = 0;
			jtag_in_start();
		}
	}
	if(ftdevs[1].timeout){
		if(time_ms>=ftdevs[1].timeout){
			ftdevs[1].timeout = 0;
			cdc_send_startup(&cdc_uarts[1]);
		}
	}
}

static int eeprom_buf;

int usbd_vendor_handler(struct usb_setup_packet *setup, uint8_t **data, uint32_t *len)
{
	int port = setup->wIndex&0xff;

	USB_LOG_INFO("FTDI: type %02x request %02x, value %04x, index %04x, length %04x\n",
			setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex, setup->wLength);

	if(port==0)
		port = 1;
	*len = 0;

	switch (setup->bRequest) {
	case SIO_READ_EEPROM_REQUEST:
		if(setup->wIndex<0x40){
			eeprom_buf = ftdi_eeprom_info[setup->wIndex];
		}else{
			eeprom_buf = 0;
		}
		*data = (uint8_t*)&eeprom_buf;
		*len = 2;
		break;
	case SIO_RESET_REQUEST:
		if(setup->wValue==0){
			// RESET_SIO命令后，FTDI驱动会重置ep的状态。ep将要发送的数据也被清除了。
			// 但FTDI驱动接下来会读ep，如果读不到就卡死了。这里设置一个16ms的定时器，
			// 到期后会发送数据给驱动。
			ftdevs[port-1].timeout = time_ms + ftdevs[port-1].latency_timer;
		}
		break;
	case SIO_SET_MODEM_CTRL_REQUEST:
		break;
	case SIO_SET_FLOW_CTRL_REQUEST:
		break;
	case SIO_SET_BAUDRATE_REQUEST://wValue，2个字节波特率
		if(port==2){
			struct cdc_line_coding lc;
			lc.dwDTERate = 3000000/setup->wValue;
			lc.bDataBits = 0;
			usbd_cdc_acm_set_line_coding(UART_INTF, &lc);
		}
		break;
	case SIO_SET_DATA_REQUEST:
		if(port==2){
			/**
			 * D0-D7 databits  BITS_7=7, BITS_8=8
			 * D8-D10 parity  NONE=0, ODD=1, EVEN=2, MARK=3, SPACE=4
			 * D11-D12 		STOP_BIT_1=0, STOP_BIT_15=1, STOP_BIT_2=2 
			 * D14  		BREAK_OFF=0, BREAK_ON=1
			 **/
			struct cdc_line_coding lc;
			lc.dwDTERate = 0;
			lc.bDataBits = setup->wValue&0xff;
			lc.bParityType = (setup->wValue>>8 )&0x07;
			lc.bCharFormat = (setup->wValue>>11)&0x03;
			usbd_cdc_acm_set_line_coding(UART_INTF, &lc);
		}
		break;
	case SIO_POLL_MODEM_STATUS_REQUEST:
		eeprom_buf = 0x6002;
		*data = (uint8_t*)&eeprom_buf;
		*len = 2;
		break;
	case SIO_SET_EVENT_CHAR_REQUEST:
		break;
	case SIO_SET_ERROR_CHAR_REQUEST:
		break;
	case SIO_SET_LATENCY_TIMER_REQUEST:
		ftdevs[port-1].latency_timer = setup->wValue;
		break;
	case SIO_GET_LATENCY_TIMER_REQUEST:
		*data = (uint8_t*)&ftdevs[port-1].latency_timer;
		*len = 1;
		break;
	case SIO_SET_BITMODE_REQUEST:
		if(port==1){
			int mode = setup->wValue>>8;
			if(mode==0x02){
				jtag_setup();
			}else{
				jtag_exit();
			}
		}
		break;
	default:
		USB_LOG_DBG("CDC ACM request 0x%x, value 0x%x\r\n", setup->bRequest, setup->wValue);
		return -1;
	}

	return 0;
}


void usbd_event_handler(uint8_t event)
{
	USB_LOG_RAW("USBD Event: %d\n", event);
	switch (event) {
	case USBD_EVENT_RESET:
		ftdevs[0].timeout = 0;
		ftdevs[0].latency_timer = 16;
		ftdevs[1].timeout = 0;
		ftdevs[1].latency_timer = 16;
		cdcuart_reset(&cdc_uarts[1]);
		break;
	case USBD_EVENT_CONNECTED:
		break;
	case USBD_EVENT_DISCONNECTED:
		break;
	case USBD_EVENT_RESUME:
		break;
	case USBD_EVENT_SUSPEND:
		break;
	case USBD_EVENT_CONFIGURED:
		cdc_recv_start(&cdc_uarts[1]);
		cdc_send_startup(&cdc_uarts[1]);
		jtag_out_start();
		jtag_in_start();
		break;
	case USBD_EVENT_SET_REMOTE_WAKEUP:
		break;
	case USBD_EVENT_CLR_REMOTE_WAKEUP:
		break;
	default:
		break;
	}
}


void usb_dc_user_init(void)
{
	ftdevs[0].timeout = 0;
	ftdevs[0].latency_timer = 16;
	ftdevs[1].timeout = 0;
	ftdevs[1].latency_timer = 16;

	jtag_init();

	usbd_desc_register(ftlink_descriptor, string_desc, string_cnt);


#if 1
	// 这里只使用JTAG功能，不使用串口。
	usbd_add_interface(&jtag_intf);
	usbd_add_endpoint(&jtag_out_ep);
	usbd_add_endpoint(&jtag_in_ep);
#else
	cdcuart_init(&cdc_uarts[0]);
	int_enable(DMA1_Channel3_IRQn);
	int_enable(USART3_IRQn);
#endif

	cdcuart_init(&cdc_uarts[1]);
	int_enable(DMA1_Channel5_IRQn);
	int_enable(USART1_IRQn);

	usbd_initialize();
}

