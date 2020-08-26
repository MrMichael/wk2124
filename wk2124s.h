#ifndef __WK2124S_H_
#define __WK2124S_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <rthw.h>

#include <string.h>
#include <stdlib.h>

//全局寄存器列表 共4个
#define GENA	0x00   													/*全局控制寄存器 											R/W*/
#define GRST  0x01   													/*全局子串口复位寄存器 								R/W*/
#define GIER  0x10   													/*全局中断寄存器 											R/W*/
#define GIFR  0x11   													/*全局中断标志寄存器 									R*/

//子串口寄存器   共18个 (x:00 --- 11)
#define	SPAGE(x)     					((x << 4)|0x03)	/*子串口页控制寄存器 									R/W*/

#define	SPAGE0_SCR(x)     		((x << 4)|0x04) /*子串口使能寄存器 										R/W*/
#define	SPAGE0_LCR(x)     		((x << 4)|0x05) /*子串口配置寄存器 										R/W*/
#define	SPAGE0_FCR(x)     		((x << 4)|0x06) /*子串口FIFO控制寄存器 								R/W*/
#define	SPAGE0_SIER(x)     		((x << 4)|0x07) /*子串口中断使能寄存器 								R/W*/
#define	SPAGE0_SIFR(x)     		((x << 4)|0x08) /*子串口中断标志寄存器 								R/W*/
#define	SPAGE0_TFCNT(x)     	((x << 4)|0x09) /*子串口发送FIFO计数寄存器 						R*/
#define	SPAGE0_RFCNT(x)     	((x << 4)|0x0A) /*子串口接收FIFO计数寄存器 						R*/
#define	SPAGE0_FSR(x)     		((x << 4)|0x0B) /*子串口FIFO状态寄存器 								R*/
#define	SPAGE0_LSR(x)     		((x << 4)|0x0C) /*子串口接收状态寄存器 								R*/
#define	SPAGE0_FDAT(x)     		((x << 4)|0x0D) /*子串口FIFO数据寄存器 								R/W*/

#define	SPAGE1_BAUD1(x)     	((x << 4)|0x04) /*子串口波特率配置寄存器高字节 				R/W*/
#define	SPAGE1_BAUD0(x)     	((x << 4)|0x05) /*子穿裤波特率撇只寄存器低字节 				R/W*/
#define	SPAGE1_PRES(x)     		((x << 4)|0x06) /*子穿裤波特率配置寄存器小数部分 			R/W*/
#define	SPAGE1_RFTL(x)     		((x << 4)|0x07) /*子串口接收FIFO中断触发点配置寄存器 	R/W*/
#define	SPAGE1_TFTL(x)     		((x << 4)|0x08) /*子串口发送FIFO中断触发点配置寄存器 	R/W*/


#ifdef PKG_USING_WK2124

void EXHW_WK2124_Write_Reg(struct rt_spi_device *device, uint8_t reg, uint8_t dat);
uint8_t EXHW_WK2124_Read_Reg(struct rt_spi_device *device, uint8_t reg);
void EXHW_WK2124_Write_FIFO(struct rt_spi_device *device, uint8_t reg, uint8_t *buf, uint16_t len);
void EXHW_WK2124_Read_FIFO(struct rt_spi_device *device, uint8_t reg, uint8_t *buf, uint16_t len);
uint16_t Wk2124_SendBuf(struct rt_spi_device *device, uint8_t index, uint8_t *sendbuf,uint16_t len);
uint16_t Wk2124_GetBuf(struct rt_spi_device *device, uint8_t index, uint8_t *getbuf, uint16_t len);

void EXHW_WK2124_Init(struct rt_spi_device *device);

void EXHW_WK2124_Disable_Tx(struct rt_spi_device *device, uint8_t index);
void EXHW_WK2124_Enable_Tx(struct rt_spi_device *device, uint8_t index);
void EXHW_WK2124_Disable_Rx(struct rt_spi_device *device, uint8_t index);
void EXHW_WK2124_Enable_Rx(struct rt_spi_device *device, uint8_t index);
#endif

#endif
