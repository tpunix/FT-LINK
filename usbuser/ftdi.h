
#ifndef _FTDI_H_
#define _FTDI_H_

/******************************************************************************/

#define GPIO_BASE      0x40010c00  // GPIOB
                                   // 如果4个IO不在同一个bank，则需要另外一个寄存器保存另一个BASE。
#define GPIO_IN        0x08
#define GPIO_SET       0x10
#define GPIO_CLR       0x14


#define GPIO_TCK  13
#define GPIO_TDI  10
#define GPIO_TDO  11
#define GPIO_TMS  14

#define TCK_MASK       (1<<GPIO_TCK)
#define TMS_MASK       (1<<GPIO_TMS)
#define TDI_MASK       (1<<GPIO_TDI)

#define TDO_SHIFT_MSB  GPIO_TDO
#define TDO_SHIFT_LSB  (GPIO_TDO-7)   // 如果GPIO编号小于7，对应的移位指令要改为左移。


#define DIR_OUT 1  // push-poll output
#define DIR_IN  4  // float input


#ifndef __ASSEMBLY__ 

void usb_dc_user_init(void);

int jtag_setup(void);
int jtag_exit(void);
int jtag_execute(uint8_t *req, int req_size);
void jtag_flush_resp(void);
void jtag_write(int byte);
void jtag_handle(void);

int jtag_trans_msb_0(int data, int bcnt, int delay);
int jtag_trans_msb_1(int data, int bcnt, int delay);
int jtag_trans_lsb_0(int data, int bcnt, int delay);
int jtag_trans_lsb_1(int data, int bcnt, int delay);
int jtag_trans_tms_0(int data, int bcnt, int delay);
int jtag_trans_tms_1(int data, int bcnt, int delay);

void ftdi_timer_handle(void);

#endif

/******************************************************************************/


#endif

