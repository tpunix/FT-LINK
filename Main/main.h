
#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch32v.h"
#include "riscv.h"

typedef unsigned  char u8;
typedef unsigned short u16;
typedef unsigned   int u32;
typedef unsigned long long u64;

#ifndef NULL
#define NULL ((void*)0)
#endif

#define REG(x) (*(volatile unsigned int*)(x))

/******************************************************************************/

#define OSC_16M

#define SYSCLK_FREQ  144000000
#define AHBCLK_FREQ  SYSCLK_FREQ
#define APB1CLK_FREQ AHBCLK_FREQ
#define APB2CLK_FREQ AHBCLK_FREQ


/******************************************************************************/


void system_init(void);

#define TIMER_HZ (SYSCLK_FREQ/8)
extern u32 time_ms;

u64 get_mcycle(void);
void timer_init(void);
void reset_timer(void);
u32 get_timer(void);
void udelay(int us);
void mdelay(int ms);


void int_priorit(int id, int priorit);
void int_enable(int id);
void int_disable(int id);

void uart1_init(int baudrate);
int  _getc(int tmout);
void _putc(int ch);
void _puts(char *str);


void gpio_mode(int group, int bit, int mode, int pullup);
void gpio_set(int group, int bit, int val);
int  gpio_get(int group, int bit);


/******************************************************************************/



int printk(char *fmt, ...);
int sprintk(char *sbuf, const char *fmt, ...);
void hex_dump(char *str, void *addr, int size);


unsigned int strlen(const char *s);
int strcmp(const char *s1, const char *s2);
int strncmp(const char *s1, const char *s2, unsigned int n);
int strcasecmp(const char *s1, const char *s2);
char *strcpy(char *dst, const char *src);
char *strncpy(char *dst, const char *src, unsigned int n);
char *strchr(const char *s1, int ch);
unsigned long strtoul(const char *str, char **endptr, int requestedbase);

void *memset(void *s, int v, unsigned int n);
void *memcpy(void *to, const void *from, unsigned int n);
int memcmp(const void *dst, const void *src, unsigned int n);


void simple_shell(void);


/******************************************************************************/


#endif

