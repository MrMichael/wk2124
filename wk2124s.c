/*
 *WK2124S 支持 SPI模式
 *WK2124S IRQ中断信号的输出低电平有效
 */

#include "wk2124s.h"

#ifdef PKG_USING_WK2124

void EXHW_WK2124_Write_Reg(struct rt_spi_device *device, uint8_t reg, uint8_t dat)
{
    static uint8_t sendbuf[2];

    sendbuf[0] = reg;
    sendbuf[1] = dat;

    rt_spi_send(device, sendbuf, 2);
}

uint8_t EXHW_WK2124_Read_Reg(struct rt_spi_device *device, uint8_t reg)
{
    static uint8_t sendbuf[2];
    static uint8_t recbuf[2];
    
    sendbuf[0] = 0x40 | reg;
    sendbuf[1] = 0x00;
    rt_spi_transfer(device, sendbuf, recbuf, 2);
    return recbuf[1];
}

void EXHW_WK2124_Write_FIFO(struct rt_spi_device *device, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t cnt = 0;
    uint8_t sendbuf[257];

    sendbuf[0] = 0x80 | reg;
    for(cnt = 0; (cnt < len)&&(cnt < 256); cnt++) {
        sendbuf[1+cnt] = buf[cnt];
    } 
    rt_spi_send(device, sendbuf, cnt+1);
}

void EXHW_WK2124_Read_FIFO(struct rt_spi_device *device, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t cnt = 0;
    uint8_t sendbuf[257];
    uint8_t recbuf[257];
    uint16_t i = 0;

    sendbuf[0] = 0xC0 | reg;
    for(cnt = 0; (cnt < len)&&(cnt < 256); cnt++) {
        sendbuf[1+cnt] = 0x00;
    } 
    rt_spi_transfer(device, sendbuf, recbuf, cnt+1);
    for ( i = 0; i < cnt; i++) {
        buf[i] = recbuf[1+i];
    }
}

uint16_t Wk2124_SendBuf(struct rt_spi_device *device, uint8_t index, uint8_t *sendbuf,uint16_t len)
{
    uint16_t ret = 0,tfcnt = 0,sendlen = 0;
    uint8_t  fsr = 0;
    
    fsr = EXHW_WK2124_Read_Reg(device, SPAGE0_FSR(index));
    if(~fsr & 0x02 )//子串口发送FIFO未满
    {
        tfcnt = EXHW_WK2124_Read_Reg(device, SPAGE0_TFCNT(index));//读子串口发送fifo中数据个数
        sendlen = 256 - tfcnt;//FIFO能写入的最多字节数

        if(sendlen < len){
            ret = sendlen; 
            EXHW_WK2124_Write_FIFO(device, SPAGE0_FDAT(index),sendbuf,sendlen);
        }else{
            EXHW_WK2124_Write_FIFO(device, SPAGE0_FDAT(index),sendbuf,len);
            ret = len;
        }
    }
    return ret;
}

uint16_t Wk2124_GetBuf(struct rt_spi_device *device, uint8_t index, uint8_t *getbuf, uint16_t len)
{
    uint16_t ret=0, rfcnt = 0;
    uint8_t fsr = 0;
    
    fsr = EXHW_WK2124_Read_Reg(device, SPAGE0_FSR(index));
    if(fsr & 0x08 )//子串口接收FIFO未空
    {
        rfcnt = EXHW_WK2124_Read_Reg(device, SPAGE0_RFCNT(index));//读子串口发送fifo中数据个数
        if(rfcnt == 0)//当RFCNT寄存器为0的时候，有两种情况，可能是256或者是0，这个时候通过FSR来判断，如果FSR显示接收FIFO不为空，就为256个字节
        {
            rfcnt = 256;
        }
        if (rfcnt < len)
        {
            EXHW_WK2124_Read_FIFO(device, SPAGE0_FDAT(index), getbuf, rfcnt);
            ret = rfcnt;
        } else {
            EXHW_WK2124_Read_FIFO(device, SPAGE0_FDAT(index), getbuf, len);
            ret = len;
        }

    }
     return ret;    
}

void EXHW_WK2124_Init(struct rt_spi_device *device)
{
    /*使能子串口1,2,3,4的时钟*/
    EXHW_WK2124_Write_Reg(device, GENA,0x0F);
    
    /*复位子串口1,2,3,4*/
    EXHW_WK2124_Write_Reg(device, GRST,0x0F);
    
    /*使能子串口1,2,3,4的全局中断 */
    EXHW_WK2124_Write_Reg(device, GIER,0x0F);
    
}

void EXHW_WK2124_Disable_Tx(struct rt_spi_device *device, uint8_t index)
{
    uint8_t scr = 0;

    scr = EXHW_WK2124_Read_Reg(device, SPAGE0_SCR(index)); 
    scr &= ~(1 << 1);
    EXHW_WK2124_Write_Reg(device, SPAGE0_SCR(index),scr);
}

void EXHW_WK2124_Enable_Tx(struct rt_spi_device *device, uint8_t index)
{
    uint8_t scr = 0;

    scr = EXHW_WK2124_Read_Reg(device,SPAGE0_SCR(index)); 
    scr |= (1 << 1);
    EXHW_WK2124_Write_Reg(device,SPAGE0_SCR(index),scr);
}

void EXHW_WK2124_Disable_Rx(struct rt_spi_device *device, uint8_t index)
{
    uint8_t scr = 0;
    scr = EXHW_WK2124_Read_Reg(device,SPAGE0_SCR(index)); 
    scr &= ~(1 << 0);
    EXHW_WK2124_Write_Reg(device,SPAGE0_SCR(index),scr);
}

void EXHW_WK2124_Enable_Rx(struct rt_spi_device *device, uint8_t index)
{
    uint8_t scr = 0,fcr = 0;
    
    //复位 n 串口的FIFO
    fcr = EXHW_WK2124_Read_Reg(device,SPAGE0_FCR(index)); 
    fcr |= (1 << 0);
    EXHW_WK2124_Write_Reg(device,SPAGE0_FCR(index),fcr);
    //使能串口 n  的接收
    scr = EXHW_WK2124_Read_Reg(device,SPAGE0_SCR(index)); 
    scr |= (1 << 0);
    EXHW_WK2124_Write_Reg(device,SPAGE0_SCR(index),scr);
}

#endif
