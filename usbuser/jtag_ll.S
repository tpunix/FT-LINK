//
// 用C实现的jtag_trans_func大概能达到2-3M的速度。
// 这里手写一下，能有6M的速度，并可以方便的调整延时参数
//

#include "ftdi.h"


// int jtag_trans_func(int data, int bcnt, int delay);
//                         a0        a1        a2
//   a3: TDI mask
//   a4: GPIOB
//   a5: TCK mask
//   a6: tmp
//   a7: rdata


///////////////////////////////////////////////////////////////////////////////

	.global jtag_trans_msb_0
	.func   jtag_trans_msb_0
jtag_trans_msb_0:
	li		a3, TDI_MASK
	li		a4, GPIO_BASE
	lui		a5, 2
	li		a7, 0

99:
	sw		a5, GPIO_CLR(a4)  // set_tck(0)

	andi	a6, a0, 0x80  // set_tdi(data&0x80)
	bnez	a6, 1f
	sw		a3, GPIO_CLR(a4)
	j		2f
1:
	sw		a3, GPIO_SET(a4)
2:

	li		a6, 0         // jtag_delay
3:
	addi	a6, a6, 1
	bltu	a6, a2, 3b

	lw		a6, GPIO_IN(a4)  // get_tdo()

	sw		a5, GPIO_SET(a4) // set_tck(1);

	srli	a6, a6, TDO_SHIFT_MSB
	slli	a7, a7, 1     // rdata <<= 1;
	andi	a6, a6, 0x01
	or		a7, a7, a6    // rdata |= tdo;

	slli	a0, a0, 1     // data <<= 1;

	li		a6, 0         // jtag_delay
3:
	addi	a6, a6, 1
	blt		a6, a2, 3b

	addi	a1, a1, -1
	bnez	a1, 99b

	sw		a5, GPIO_CLR(a4)  // set_tck(0)
	mv		a0, a7
	ret
	.endfunc

///////////////////////////////////////////////////////////////////////////////


	.global jtag_trans_msb_1
	.func   jtag_trans_msb_1
jtag_trans_msb_1:
	li		a3, TDI_MASK
	li		a4, GPIO_BASE
	lui		a5, 2
	li		a7, 0

99:
	sw		a5, GPIO_SET(a4)  // set_tck(1)

	andi	a6, a0, 0x80  // set_tdi(data&0x80)
	bnez	a6, 1f
	sw		a3, GPIO_CLR(a4)
	j		2f
1:
	sw		a3, GPIO_SET(a4)
2:

	li		a6, 0         // jtag_delay
3:
	addi	a6, a6, 1
	bltu	a6, a2, 3b

	lw		a6, GPIO_IN(a4)  // get_tdo()

	sw		a5, GPIO_CLR(a4) // set_tck(0);

	srli	a6, a6, TDO_SHIFT_MSB
	slli	a7, a7, 1     // rdata <<= 1;
	andi	a6, a6, 0x01
	or		a7, a7, a6    // rdata |= tdo;

	slli	a0, a0, 1     // data <<= 1;

	li		a6, 0         // jtag_delay
3:
	addi	a6, a6, 1
	blt		a6, a2, 3b

	addi	a1, a1, -1
	bnez	a1, 99b

	sw		a5, GPIO_SET(a4)  // set_tck(1)
	mv		a0, a7
	ret
	.endfunc

///////////////////////////////////////////////////////////////////////////////

	.global jtag_trans_lsb_0
	.func   jtag_trans_lsb_0
jtag_trans_lsb_0:
	li		a3, TDI_MASK
	li		a4, GPIO_BASE
	lui		a5, 2
	li		a7, 0

99:
	sw		a5, GPIO_CLR(a4)  // set_tck(0)

	andi	a6, a0, 0x01  // set_tdi(data&0x01)
	bnez	a6, 1f
	sw		a3, GPIO_CLR(a4)
	j		2f
1:
	sw		a3, GPIO_SET(a4)
2:

	li		a6, 0         // jtag_delay
3:
	addi	a6, a6, 1
	bltu	a6, a2, 3b

	lw		a6, GPIO_IN(a4)  // get_tdo()

	sw		a5, GPIO_SET(a4) // set_tck(1);

	srli	a6, a6, TDO_SHIFT_LSB
	srli	a7, a7, 1     // rdata >>= 1;
	andi	a6, a6, 0x80
	or		a7, a7, a6    // rdata |= tdo;

	srli	a0, a0, 1     // data >>= 1;

	li		a6, 0         // jtag_delay
3:
	addi	a6, a6, 1
	blt		a6, a2, 3b

	addi	a1, a1, -1
	bnez	a1, 99b

	sw		a5, GPIO_CLR(a4)  // set_tck(0)
	mv		a0, a7
	ret
	.endfunc

///////////////////////////////////////////////////////////////////////////////

	.global jtag_trans_lsb_1
	.func   jtag_trans_lsb_1
jtag_trans_lsb_1:
	li		a3, TDI_MASK
	li		a4, GPIO_BASE
	lui		a5, 2
	li		a7, 0

99:
	sw		a5, GPIO_SET(a4)  // set_tck(1)

	andi	a6, a0, 0x01  // set_tdi(data&0x01)
	bnez	a6, 1f
	sw		a3, GPIO_CLR(a4)
	j		2f
1:
	sw		a3, GPIO_SET(a4)
2:

	li		a6, 0         // jtag_delay
3:
	addi	a6, a6, 1
	bltu	a6, a2, 3b

	lw		a6, GPIO_IN(a4)  // get_tdo()

	sw		a5, GPIO_CLR(a4) // set_tck(0);

	srli	a6, a6, TDO_SHIFT_LSB
	srli	a7, a7, 1     // rdata >>= 1;
	andi	a6, a6, 0x80
	or		a7, a7, a6    // rdata |= tdo;

	srli	a0, a0, 1     // data >>= 1;

	li		a6, 0         // jtag_delay
3:
	addi	a6, a6, 1
	blt		a6, a2, 3b

	addi	a1, a1, -1
	bnez	a1, 99b

	sw		a5, GPIO_SET(a4)  // set_tck(1)
	mv		a0, a7
	ret
	.endfunc


///////////////////////////////////////////////////////////////////////////////

	.global jtag_trans_tms_0
	.func   jtag_trans_tms_0
jtag_trans_tms_0:
	li		a3, TMS_MASK
	li		a4, GPIO_BASE
	lui		a5, 2
	li		a7, 0

99:
	sw		a5, GPIO_CLR(a4)  // set_tck(0)

	andi	a6, a0, 0x01  // set_tms(data&0x01)
	bnez	a6, 1f
	sw		a3, GPIO_CLR(a4)
	j		2f
1:
	sw		a3, GPIO_SET(a4)
2:

	li		a6, 0         // jtag_delay
3:
	addi	a6, a6, 1
	bltu	a6, a2, 3b

	lw		a6, GPIO_IN(a4)  // get_tdo()

	sw		a5, GPIO_SET(a4) // set_tck(1);

	srli	a6, a6, TDO_SHIFT_LSB
	srli	a7, a7, 1     // rdata >>= 1;
	andi	a6, a6, 0x80
	or		a7, a7, a6    // rdata |= tdo;

	srli	a0, a0, 1     // data >>= 1;

	li		a6, 0         // jtag_delay
3:
	addi	a6, a6, 1
	blt		a6, a2, 3b

	addi	a1, a1, -1
	bnez	a1, 99b

	sw		a5, GPIO_CLR(a4)  // set_tck(0)
	mv		a0, a7
	ret
	.endfunc


///////////////////////////////////////////////////////////////////////////////

	.global jtag_trans_tms_1
	.func   jtag_trans_tms_1
jtag_trans_tms_1:
	li		a3, TMS_MASK
	li		a4, GPIO_BASE
	lui		a5, 2
	li		a7, 0

99:
	sw		a5, GPIO_SET(a4)  // set_tck(1)

	andi	a6, a0, 0x01  // set_tms(data&0x01)
	bnez	a6, 1f
	sw		a3, GPIO_CLR(a4)
	j		2f
1:
	sw		a3, GPIO_SET(a4)
2:

	li		a6, 0         // jtag_delay
3:
	addi	a6, a6, 1
	bltu	a6, a2, 3b

	lw		a6, GPIO_IN(a4)  // get_tdo()

	sw		a5, GPIO_CLR(a4) // set_tck(0);

	srli	a6, a6, TDO_SHIFT_LSB
	srli	a7, a7, 1     // rdata >>= 1;
	andi	a6, a6, 0x80
	or		a7, a7, a6    // rdata |= tdo;

	srli	a0, a0, 1     // data >>= 1;

	li		a6, 0         // jtag_delay
3:
	addi	a6, a6, 1
	blt		a6, a2, 3b

	addi	a1, a1, -1
	bnez	a1, 99b

	sw		a5, GPIO_SET(a4)  // set_tck(1)
	mv		a0, a7
	ret
	.endfunc


