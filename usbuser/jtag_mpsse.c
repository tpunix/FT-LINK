

#include "main.h"
#include "usbd_core.h"
#include "ftdi.h"

/******************************************************************************/


static void set_tdi(int val)
{
	if(val){
		GPIOB->BSHR = TDI_MASK;
	}else{
		GPIOB->BCR  = TDI_MASK;
	}
}


static void set_tck(int val)
{
	if(val){
		GPIOB->BSHR = TCK_MASK;
	}else{
		GPIOB->BCR  = TCK_MASK;
	}
}

#if 0
static void set_tms(int val)
{
	if(val){
		GPIOB->BSHR = TMS_MASK;
	}else{
		GPIOB->BCR  = TMS_MASK;
	}
}

static int get_tdo(void)
{
	return (GPIOB->INDR >> GPIO_TDO) & 1;
}
#endif

static void set_gpio(int id, int val, int dir)
{
	int pd, pv;

	if(id==1)
		return;

	printk("jtag_set_gpio: val=%02x dir=%02x\n", val, dir);

	pd = (dir&1)? DIR_OUT : DIR_IN;
	pv = (val&1)? 1: 0;
	gpio_mode(1, GPIO_TCK, pd, pv); //  TCK

	pd = (dir&2)? DIR_OUT : DIR_IN;
	pv = (val&2)? 1: 0;
	gpio_mode(1, GPIO_TDI, pd, pv); //  TDI

	pd = (dir&4)? DIR_OUT : DIR_IN;
	pv = (val&4)? 1: 0;
	gpio_mode(1, GPIO_TDO, pd, pv); //  TDO

	pd = (dir&8)? DIR_OUT : DIR_IN;
	pv = (val&8)? 1: 0;
	gpio_mode(1, GPIO_TMS, pd, pv); //  TMS
}

static int get_gpio(int id)
{
	int val = 0;

	if(id==1)
		return 0xff;

	if((GPIOB->INDR & (1<<GPIO_TCK))) val |= 0x01;
	if((GPIOB->INDR & (1<<GPIO_TDI))) val |= 0x02;
	if((GPIOB->INDR & (1<<GPIO_TDO))) val |= 0x04;
	if((GPIOB->INDR & (1<<GPIO_TMS))) val |= 0x08;

	return val;
}

/******************************************************************************/


int jtag_setup(void)
{
	//cdc_pause(0);
	gpio_mode(1, GPIO_TCK, DIR_OUT, 1); //  TCK: out
	gpio_mode(1, GPIO_TDI, DIR_OUT, 1); //  TDI: out
	gpio_mode(1, GPIO_TDO, DIR_IN , 1); //  TDO: in
	gpio_mode(1, GPIO_TMS, DIR_OUT, 1); //  TMS: out
	gpio_mode(2,  6, DIR_OUT, 1); // TRST: out

	return 0;
}

int jtag_exit(void)
{
	//cdc_resume(0);
	gpio_mode(1, 13, DIR_OUT, 1); //  TCK: out
	gpio_mode(1, 10,       9, 1); //  TDI: out TXD
	gpio_mode(1, 11,       8, 1); //  TDO: in  RXD
	gpio_mode(1, 14, DIR_OUT, 1); //  TMS: out
	gpio_mode(2,  6, DIR_OUT, 1); // TRST: out

	return 0;
}


/******************************************************************************/


#define MPSSE_IDLE              0
#define MPSSE_RCV_LENGTH_L      1
#define MPSSE_RCV_LENGTH_H      2
#define MPSSE_TRANSMIT          3
#define MPSSE_TMS_OUT           4
#define MPSSE_TCK_OUT           5
#define MPSSE_RCV0              6
#define MPSSE_RCV1              7
#define MPSSE_ERROR             8


#define CF_WCLK_F    0x01
#define CF_BIT       0x02
#define CF_RCLK_F    0x04
#define CF_LSB       0x08
#define CF_WTDI      0x10
#define CF_RTDO      0x20
#define CF_WTMS      0x40

#define CTRL_LOOPBACK  0x01
#define CTRL_DIV5      0x02
#define CTRL_PHASE3    0x04
#define CTRL_RTCK      0x08

static int m_state;
static int m_cmd;
static int m_len;
static int m_div;
static int m_ctrl = CTRL_DIV5;
static uint8_t m_buf[4];


int (*jtag_trans_func)(int data, int bcnt, int delay);

int jtag_delay_value = 1;

// freq = 6M/(1+div);
// div:  0: 6M
//       1: 3M
//       2: 2M
//       3: 1.5M
//       4: 1.2M
//       5: 1M
//  >5:  period = 43*delay + 140
//       1000*(div+1)/6 = 43*delay + 140;
//       delay = (160+1000*div)/258;
static uint8_t delay_6M[6] = {1, 5, 8, 12, 16, 20};

// freq = 30M/(1+div);
//       1000*(div+1)/30 = 43*delay + 140;
//       delay = (100*div-320)/129;
static uint8_t delay_30M[16] = {1, 1, 1, 1, 1, 2, 3, 4, 4, 5, 6, 7, 8, 8, 8, 9};

static void jtag_set_delay(int div)
{
	if(m_ctrl&CTRL_DIV5){
		if(div<6){
			jtag_delay_value = delay_6M[div];
		}else{
			jtag_delay_value = (160 + 1000*div + 130)/258;
		}
	}else{
		if(div<16){
			jtag_delay_value = delay_30M[div];
		}else{
			jtag_delay_value = (100*div - 320 + 65)/129;
		}
	}
	printk("jtag_set_delay: div=%d  delay=%d\n", div, jtag_delay_value);
}


// +ve: 在clk上升沿将数据发出。对方将在下降沿看到数据。clk空闲时为高。---- --|__|-
// -ve: 在clk下降沿将数据发出。对方将在上升沿看到数据。clk空闲时为低。____ __|--|_

int jtag_execute(uint8_t *req, int req_size)
{
	int rp = 0;

	//printk("\nJTAG_EXEC: %d bytes\n", req_size);

	while(rp<req_size){
		switch(m_state){
		case MPSSE_IDLE:
			m_cmd = req[rp++];
			//printk("CMD: %02x\n", m_cmd);
			if(m_cmd&0x80){
				switch(m_cmd){
				case 0x80:
				case 0x82:
					m_state = MPSSE_RCV0;
					break;
				case 0x81:
				case 0x83:
					jtag_write(get_gpio((m_cmd>>1)&1));
					break;
				case 0x84:
					m_ctrl |=  CTRL_LOOPBACK;
					break;
				case 0x85:
					m_ctrl &= ~CTRL_LOOPBACK;
					break;
				case 0x86: // Set TCK div
					m_state = MPSSE_RCV0;
					break;
				case 0x87: // Flush buffer to pc
					jtag_flush_resp();
					break;
				case 0x88: // Wait GPIOL1 high
					break;
				case 0x89: // Wait GPIOL1 low
					break;
				case 0x8a:
					m_ctrl &= ~CTRL_DIV5;
					break;
				case 0x8b:
					m_ctrl |=  CTRL_DIV5;
					break;
				case 0x8c:
					break;
				case 0x8d:
					break;
				case 0x8e:
					m_state = MPSSE_RCV0;
					break;
				case 0x8f:
					m_state = MPSSE_RCV0;
					break;
				case 0x96:
					break;
				case 0x97:
					break;
				default:
					jtag_write(0xfa);
					jtag_write(m_cmd);
					break;
				}
			}else{
				int type = m_cmd&0x09;

				jtag_trans_func = NULL;
				if(m_cmd&CF_WTMS){
					if(type==0x08){
						jtag_trans_func = jtag_trans_tms_1;
					}else if(type==0x09){
						jtag_trans_func = jtag_trans_tms_0;
					}
				}else if(type==0x00){
					jtag_trans_func = jtag_trans_msb_1;
				}else if(type==0x01){
					jtag_trans_func = jtag_trans_msb_0;
				}else if(type==0x08){
					jtag_trans_func = jtag_trans_lsb_1;
				}else if(type==0x09){
					jtag_trans_func = jtag_trans_lsb_0;
				}
				if(jtag_trans_func==NULL){
					jtag_write(0xfa);
					jtag_write(m_cmd);
				}else{
					m_state = MPSSE_RCV_LENGTH_L;
				}
			}
			break;
		case MPSSE_RCV0:
			m_buf[0] = req[rp++];
			m_buf[1] = 0;
			if(m_cmd==0x8e){
				m_state = MPSSE_TCK_OUT;
			}else{
				m_state = MPSSE_RCV1;
			}
			break;
		case MPSSE_RCV1:
			m_buf[1] = req[rp++];
			if(m_cmd==0x86){
				m_div = (m_buf[1]<<8) | m_buf[0];
				jtag_set_delay(m_div);
				m_state = MPSSE_IDLE;
			}else if(m_cmd==0x8f){
				m_state = MPSSE_TCK_OUT;
			}else{
				// 0x80 or 0x82
				set_gpio((m_cmd>>1)&1, m_buf[0], m_buf[1]);
				m_state = MPSSE_IDLE;
			}
			break;
		case MPSSE_RCV_LENGTH_L:
			m_len = req[rp++];
			if(m_cmd&CF_BIT){
				if(m_cmd&CF_WTMS){
					m_state = MPSSE_TMS_OUT;
				}else{
					m_state = MPSSE_TRANSMIT;
				}
			}else{
				m_state = MPSSE_RCV_LENGTH_H;
			}
			break;
		case MPSSE_RCV_LENGTH_H:
			m_len |= (req[rp++]<<8);
			if(m_cmd&CF_WTMS){
				m_state = MPSSE_TMS_OUT;
			}else{
				m_state = MPSSE_TRANSMIT;
			}
			break;
		case MPSSE_TRANSMIT:
		{
			int bcnt = (m_cmd&CF_BIT)? m_len+1 : 8;
			int wdata = 0xff, rdata;

			if(m_cmd&CF_WTDI){
				wdata = req[rp++];
			}

			rdata = jtag_trans_func(wdata, bcnt, jtag_delay_value);

			if(m_cmd&CF_RTDO){
				jtag_write(rdata);
			}
			if((m_cmd&CF_BIT) || (m_len==0)){
				m_state = MPSSE_IDLE;
			}
			m_len -= 1;
			break;
		}
		case MPSSE_TMS_OUT:
		{
			int bcnt = m_len+1;
			int wdata = req[rp++];
			int rdata;

			set_tdi(wdata&0x80);

			rdata = jtag_trans_func(wdata, bcnt, jtag_delay_value);

			if(m_cmd&CF_RTDO){
				jtag_write(rdata);
			}
			m_state = MPSSE_IDLE;
			break;
		}
		case MPSSE_TCK_OUT:
		{
			m_len = (m_buf[1]<<8) | m_buf[0];
			m_len += 1;
			int bcnt = (m_cmd==0x8e)? m_len : m_len*8;

			for(int i=0; i<bcnt; i++){
				set_tck(0);
				for(int j=0; j<jtag_delay_value; j++){
					asm volatile ("nop");
					asm volatile ("nop");
				}
				set_tck(1);
				for(int j=0; j<jtag_delay_value; j++){
					asm volatile ("nop");
					asm volatile ("nop");
				}
			}
			set_tck(0);
			m_state = MPSSE_IDLE;
			break;
		}
		default:
			break;
		}

		if(m_state==MPSSE_ERROR){
			m_state = MPSSE_IDLE;
			break;
		}
	}

	return rp;
}


/******************************************************************************/


