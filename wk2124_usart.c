#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>

#include "wk2124_usart.h"
#include "wk2124s.h"

#include <drv_log.h>
// #define DRV_DEBUG

#ifdef RT_USING_SERIAL
#ifdef RT_USING_SERIAL_V1
#ifdef PKG_USING_WK2124

// WK2124晶振频率，单位Hz
#if defined(WK2124_Fosc_1843200)
    #define WK2124_Fosc 1843200
#elif defined(WK2124_Fosc_3686400)
    #define WK2124_Fosc 3686400
#elif defined(WK2124_Fosc_7372800)
    #define WK2124_Fosc 7372800
#elif defined(WK2124_Fosc_11059200)
    #define WK2124_Fosc 11059200
#elif defined(WK2124_Fosc_14745600)
    #define WK2124_Fosc 14745600
#elif defined(WK2124_Fosc_8000000)
    #define WK2124_Fosc 8000000
#elif defined(WK2124_Fosc_16000000)
    #define WK2124_Fosc 16000000
#elif defined(WK2124_Fosc_24000000)
    #define WK2124_Fosc 24000000
#elif defined(WK2124_Fosc_32000000)
    #define WK2124_Fosc 32000000
#else
    #define WK2124_Fosc 11059200
#endif

#ifndef WK2124_IRQ_PIN
    #define WK2124_IRQ_PIN        17
#endif

struct rt_spi_device *wk2124_device = RT_NULL;

/* wk2124 uart driver */
struct wk2124_uart 
{
    uint8_t swk_index; 
    uint8_t irq_enable;
    struct rt_spi_device *spi_device;
};

/* 用于接收中断的信号量 */
static struct rt_semaphore irq_sem;

static rt_err_t wk2124_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct wk2124_uart *uart;
    uint16_t baudrate;
    uint8_t baudrate_h, baudrate_l, baudrate_dec, spage0_lcr;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);
    uart = (struct wk2124_uart *)serial->parent.user_data;

    /*切换到PAGE0页中的子串口寄存器组 */
    EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE(uart->swk_index),0x00);

    /*子串口 1 控制寄存器 */
    EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE0_SCR(uart->swk_index),0x00); //子串口 发送使能  接收使能

    spage0_lcr = 0;
    if (cfg->stop_bits > 0) {
        spage0_lcr |= 0x01;
    }
    if (cfg->parity == 1) {
        spage0_lcr &= 0xfb;
        spage0_lcr |= 0x02;
    } else if (cfg->parity == 2) {
        spage0_lcr &= 0xfd;
        spage0_lcr |= 0x04;
    }
    if (cfg->data_bits == 9) {
        spage0_lcr |= 0x08;
    }
    /*子串口 配置寄存器*/
    EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE0_LCR(uart->swk_index),spage0_lcr);    //子串口 正常输出,普通模式,8位数据位,0校验,1位停止位

    /*子串口 FIFO控制寄存器*/
    EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE0_FCR(uart->swk_index),0x03);    //子串口 发送触发点,接收触发点 
                                                                                  //使能 发送,接收FIFO 复位发送接收FIFO
    /*子串口 中断使能寄存器*/
    EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE0_SIER(uart->swk_index),0x00); //子串口 禁止接收FIFO数据错误中断
                                                                                                //禁止发送FIFO空中断
                                                                                                //禁止发送FIFO触点中断
                                                                                                //禁止接收FIFO接收超时中断
                                                                                                //禁止接收FIFO接收触点中断

    baudrate = WK2124_Fosc/cfg->baud_rate/16;
    baudrate_h = baudrate/0x100;
    baudrate_l = baudrate%0x100 -1;
    baudrate_dec = (uint8_t)(((WK2124_Fosc/cfg->baud_rate/16.0f) - baudrate)*16);

    /*切换到PAGE1页中的子串口寄存器组 */
    EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE(uart->swk_index),0x01);

    /*子串口1 波特率配置寄存器高字节 [Reg = 11.0592/(115200*16) = 6] */
    EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE1_BAUD1(uart->swk_index),baudrate_h);
    /*子串口1 波特率配置寄存器低字节 */
    EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE1_BAUD0(uart->swk_index),baudrate_l);
    /*子串口1 波特率配置寄存器小数部分*/
    EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE1_PRES(uart->swk_index),baudrate_dec);

    /*切换到PAGE0页中的子串口寄存器组 */
    EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE(uart->swk_index),0x00);

    return RT_EOK;
}

static rt_err_t wk2124_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct wk2124_uart *uart;
    RT_ASSERT(serial != RT_NULL);
    uart = (struct wk2124_uart *)serial->parent.user_data;
    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        // /* disable rx irq */
        /*切换到PAGE0页中的子串口寄存器组 */
        EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE(uart->swk_index),0x00);
        /*子串口 控制寄存器 */
        EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE0_SCR(uart->swk_index),0x00);	//子串口 发送不使能  接收不使能
        /*子串口 FIFO控制寄存器*/
        EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE0_FCR(uart->swk_index),0x03);	//子串口 发送触发点,接收触发点 
                                                                                    //不使能 发送,接收FIFO 复位发送接收FIFO
        /*子串口 中断使能寄存器*/
        EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE0_SIER(uart->swk_index),0x00); //子串口 禁止接收FIFO数据错误中断
                                                                                    //禁止发送FIFO空中断
                                                                                    //禁止发送FIFO触点中断
                                                                                    //禁止接收FIFO接收超时中断
                                                                                    //禁止接收FIFO接收触点中断
        uart->irq_enable = 0;
        break;
    case RT_DEVICE_CTRL_SET_INT:
        // /* enable rx irq */
        /*切换到PAGE0页中的子串口寄存器组 */
        EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE(uart->swk_index),0x00);
        /*子串口 控制寄存器 */
        EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE0_SCR(uart->swk_index),0x03);	//子串口 发送使能  接收使能
        /*子串口 FIFO控制寄存器*/
        EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE0_FCR(uart->swk_index),0x0F);	//子串口 发送触发点,接收触发点 
                                                                                    //使能 发送,接收FIFO 复位发送接收FIFO
        /*子串口 中断使能寄存器*/
        EXHW_WK2124_Write_Reg(uart->spi_device, SPAGE0_SIER(uart->swk_index),0x83); //子串口 使能接收FIFO数据错误中断
                                                                                    //禁止发送FIFO空中断
                                                                                    //禁止发送FIFO触点中断
                                                                                    //使能接收FIFO接收超时中断
                                                                                    //使能接收FIFO接收触点中断
        uart->irq_enable = 1;
        break;
    }
    return RT_EOK;
}

static int wk2124_putc(struct rt_serial_device *serial, char c)
{
    uint8_t sendbuf[10];
    struct wk2124_uart *uart;
    RT_ASSERT(serial != RT_NULL);
    uart = (struct wk2124_uart *)serial->parent.user_data;

    sendbuf[0] = c;
    Wk2124_SendBuf(uart->spi_device, uart->swk_index, sendbuf, 1);
    while(EXHW_WK2124_Read_Reg(uart->spi_device, SPAGE0_TFCNT(uart->swk_index))  > 0);
    while((EXHW_WK2124_Read_Reg(uart->spi_device, SPAGE0_FSR(uart->swk_index)) & 0x01) == 1);

    return 1;
}

static int wk2124_getc(struct rt_serial_device *serial)
{
    int ch;
    uint8_t recbuf[10];

    struct wk2124_uart *uart;
    RT_ASSERT(serial != RT_NULL);
    uart = (struct wk2124_uart *)serial->parent.user_data;

    ch = -1;    //确保没有接收数据时rt_device_read返回值为0
    if(Wk2124_GetBuf(uart->spi_device, uart->swk_index, recbuf, 1)) {
        ch = recbuf[0];
    }
    return ch;
}

static const struct rt_uart_ops wk2124_uart_ops =
{
    wk2124_configure,
    wk2124_control,
    wk2124_putc,
    wk2124_getc,
};

#if defined(PKG_USING_UART_SWK1)
/* UART1 device driver structure */
static struct wk2124_uart uart_swk1;
struct rt_serial_device serialswk1;
void WK2124_UART1_IRQHandler(void)
{
    struct wk2124_uart *uart;
    uart = &uart_swk1;
    volatile uint8_t uart_irq_stat = 0; 
    RT_ASSERT(uart != RT_NULL);
    RT_ASSERT(&serialswk1 != RT_NULL);
    RT_ASSERT(uart->spi_device->parent.type == RT_Device_Class_SPIDevice);
    
    if (uart->irq_enable == 0) {
       return;
    }
    /*判断串口的中断类型*/
    uart_irq_stat = EXHW_WK2124_Read_Reg(uart->spi_device, SPAGE0_SIFR(uart->swk_index));
    //子串口接收FIFO触点中断标志 ， 子串口接收FIFO超时中断标志
    if(uart_irq_stat & (3 << 0)) {    
        rt_hw_serial_isr(&serialswk1, RT_SERIAL_EVENT_RX_IND);
    }
}
#endif /* PKG_USING_UART_SWK1 */

#if defined(PKG_USING_UART_SWK2)
/* UART2 device driver structure */
static struct wk2124_uart uart_swk2; 
struct rt_serial_device serialswk2;
void WK2124_UART2_IRQHandler(void)
{
    struct wk2124_uart *uart;
    uart = &uart_swk2;
    volatile uint8_t uart_irq_stat = 0; 

    RT_ASSERT(uart != RT_NULL);
    RT_ASSERT(&serialswk2 != RT_NULL);
    RT_ASSERT(uart->spi_device->parent.type == RT_Device_Class_SPIDevice);
    
    if (uart->irq_enable == 0) {
       return;
    }
    /*判断串口的中断类型*/
    uart_irq_stat = EXHW_WK2124_Read_Reg(uart->spi_device, SPAGE0_SIFR(uart->swk_index));
    //子串口接收FIFO触点中断标志 ， 子串口接收FIFO超时中断标志
    if(uart_irq_stat & (3 << 0)) {    
        rt_hw_serial_isr(&serialswk2, RT_SERIAL_EVENT_RX_IND);
    }
}
#endif /* PKG_USING_UART_SWK2 */

#if defined(PKG_USING_UART_SWK3)
/* UART3 device driver structure */
static struct wk2124_uart uart_swk3;
struct rt_serial_device serialswk3;
void WK2124_UART3_IRQHandler(void)
{
    struct wk2124_uart *uart;
    uart = &uart_swk3;
    volatile uint8_t uart_irq_stat = 0; 

    RT_ASSERT(uart != RT_NULL);
    RT_ASSERT(&serialswk3 != RT_NULL);
    RT_ASSERT(uart->spi_device->parent.type == RT_Device_Class_SPIDevice);

    if (uart->irq_enable == 0) {
       return;
    }
    /*判断串口的中断类型*/
    uart_irq_stat = EXHW_WK2124_Read_Reg(uart->spi_device, SPAGE0_SIFR(uart->swk_index));
    //子串口接收FIFO触点中断标志 ， 子串口接收FIFO超时中断标志
    if(uart_irq_stat & (3 << 0)) {    
        rt_hw_serial_isr(&serialswk3, RT_SERIAL_EVENT_RX_IND);
    }
}
#endif /* PKG_USING_UART_SWK3 */

#if defined(PKG_USING_UART_SWK4)
/* UART4 device driver structure */
static struct wk2124_uart uart_swk4;
struct rt_serial_device serialswk4;
void WK2124_UART4_IRQHandler(void)
{
    struct wk2124_uart *uart;
    uart = &uart_swk4;
    volatile uint8_t uart_irq_stat = 0; 

    RT_ASSERT(uart != RT_NULL);
    RT_ASSERT(&serialswk4 != RT_NULL);
    RT_ASSERT(uart->spi_device->parent.type == RT_Device_Class_SPIDevice);

    if (uart->irq_enable == 0) {
       return;
    }
    /*判断串口的中断类型*/
    uart_irq_stat = EXHW_WK2124_Read_Reg(uart->spi_device, SPAGE0_SIFR(uart->swk_index));
    //子串口接收FIFO触点中断标志 ， 子串口接收FIFO超时中断标志
    if(uart_irq_stat & (3 << 0)) {    
        rt_hw_serial_isr(&serialswk4, RT_SERIAL_EVENT_RX_IND);
    }
}
#endif /* PKG_USING_UART_SWK4 */

/* 中断回调函数 */
void WK2124_IRQHandler(void *args)
{
    rt_sem_release(&irq_sem);
}

static void wk2124_irq_thread_entry(void *parameter)
{
    volatile uint8_t g_irq_stat = 0; 

    while (1)
    {
        rt_sem_take(&irq_sem, RT_WAITING_FOREVER);

        /*使能子串口1,2,3,4的时钟*/
        g_irq_stat = EXHW_WK2124_Read_Reg(wk2124_device, GIFR);
        if(g_irq_stat & (1 << 0)){//子串口 1 有中断
    #if defined(PKG_USING_UART_SWK1)
            WK2124_UART1_IRQHandler();
    #endif
        }
        if(g_irq_stat & (1 << 1)){//子串口 2 有中断
    #if defined(PKG_USING_UART_SWK2)
            WK2124_UART2_IRQHandler();
    #endif
        }
        if(g_irq_stat & (1 << 2)){//子串口 3 有中断
    #if defined(PKG_USING_UART_SWK3)
            WK2124_UART3_IRQHandler();
    #endif
        }
        if(g_irq_stat & (1 << 3)){//子串口 4 有中断
    #if defined(PKG_USING_UART_SWK4)
            WK2124_UART4_IRQHandler();
    #endif
        }
    }
}

int WK2124_IRQ_Init(void)
{
    rt_err_t ret = 0;

    /* 设置引脚为输入模式 */
    rt_pin_mode(WK2124_IRQ_PIN, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，下降沿模式，回调函数名为WK2124_IRQHandler */
    rt_pin_attach_irq(WK2124_IRQ_PIN, PIN_IRQ_MODE_FALLING, WK2124_IRQHandler, RT_NULL);
    
    /* 初始化信号量 */
    rt_sem_init(&irq_sem, "irq_sem", 0, RT_IPC_FLAG_FIFO);
    rt_thread_t thread = rt_thread_create("wk2124_irq", wk2124_irq_thread_entry, RT_NULL, 1024, RT_THREAD_PRIORITY_MAX / 6, 20);
    if (thread != RT_NULL) {
        rt_thread_startup(thread);
    }  else {
        return -RT_ERROR;
    }

    /* 使能中断 */
    ret = rt_pin_irq_enable(WK2124_IRQ_PIN, PIN_IRQ_ENABLE);
    return ret;
}


int hw_wk2124_usart_init(struct rt_spi_device *device)
{
    struct wk2124_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_err_t ret = 0;

#ifdef PKG_USING_UART_SWK1
    uart = &uart_swk1;
    uart->swk_index = 0;
    uart->irq_enable = 0;
    uart->spi_device = device;
    serialswk1.ops    = &wk2124_uart_ops;
    serialswk1.config = config;
    serialswk1.serial_rx  = RT_NULL;
    /* register UART1 device */
    ret = rt_hw_serial_register(&serialswk1, "uartswk1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
    if (ret) {
        LOG_E("rt_hw_serial_register %s error", "uartswk1");
    }
    RT_ASSERT(ret == RT_EOK);
#endif /* PKG_USING_UART_SWK1 */

#ifdef PKG_USING_UART_SWK2
    uart = &uart_swk2;
    uart->swk_index = 1;
    uart->irq_enable = 0;
    uart->spi_device = device;
    serialswk2.ops    = &wk2124_uart_ops;
    serialswk2.config = config;
    serialswk2.serial_rx  = RT_NULL;
    /* register UART2 device */
    ret = rt_hw_serial_register(&serialswk2, "uartswk2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
    if (ret) {
        LOG_E("rt_hw_serial_register %s error", "uartswk2");
    }
    RT_ASSERT(ret == RT_EOK);
#endif /* PKG_USING_UART_SWK2 */

#ifdef PKG_USING_UART_SWK3
    uart = &uart_swk3;
    uart->swk_index = 2;
    uart->irq_enable = 0;
    uart->spi_device = device;
    serialswk3.ops    = &wk2124_uart_ops;
    serialswk3.config = config;
    serialswk3.serial_rx  = RT_NULL;
    /* register UART3 device */
    ret = rt_hw_serial_register(&serialswk3, "uartswk3",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
    if (ret) {
        LOG_E("rt_hw_serial_register %s error", "uartswk3");
    }
    RT_ASSERT(ret == RT_EOK);
#endif /* PKG_USING_UART_SWK3 */

#ifdef PKG_USING_UART_SWK4
    uart = &uart_swk4;
    uart->swk_index = 3;
    uart->irq_enable = 0;
    uart->spi_device = device;
    serialswk4.ops    = &wk2124_uart_ops;
    serialswk4.config = config;
    serialswk4.serial_rx  = RT_NULL;
    /* register UART6 device */
    ret = rt_hw_serial_register(&serialswk4, "uartswk4",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
    if (ret) {
        LOG_E("rt_hw_serial_register %s error", "uartswk4");
    }
    RT_ASSERT(ret == RT_EOK);
#endif /* PKG_USING_UART_SWK4 */

    return ret;
}


int wk2124_spi_init(const char *spi_dev_name)
{
    RT_ASSERT(spi_dev_name);

    if (wk2124_device != RT_NULL)
    {
        return 0;
    }

    wk2124_device = (struct rt_spi_device *) rt_device_find(spi_dev_name);
    if (wk2124_device == RT_NULL)
    {
        LOG_E("You should attach [%s] into SPI bus firstly.", spi_dev_name);
        return -RT_ENOSYS;
    }

    /* check SPI device type */
    RT_ASSERT(wk2124_device->parent.type == RT_Device_Class_SPIDevice);

    /* configure SPI device*/
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;  /* SPI Compatible Modes 0 */
        cfg.max_hz = 40 * 1000 * 1000;                          /* SPI Interface with Clock Speeds Up to 40 MHz */
        if (rt_spi_configure(wk2124_device, &cfg)) {
            LOG_E("rt_spi_configure SPI device %s error.", spi_dev_name);
            return -RT_ERROR;
        }
    }

    if (rt_device_open((rt_device_t) wk2124_device, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        LOG_E("open WK2124 SPI device %s error.", spi_dev_name);
        return -RT_ERROR;
    }

    return RT_EOK;
}



int wk2124_device_init(void)
{
    rt_err_t ret = 0;

#ifdef WK2124_SPI_DEVICE
    rt_thread_mdelay(100);

    ret = wk2124_spi_init(WK2124_SPI_DEVICE);
    if (ret == RT_EOK) {
        rt_thread_mdelay(100);
        WK2124_IRQ_Init();
        EXHW_WK2124_Init(wk2124_device);
        rt_thread_mdelay(100);
        ret = hw_wk2124_usart_init(wk2124_device);
        if (ret != RT_EOK) {
            LOG_E("RT-Thread WK2124 package initialize fail.");
            return -RT_ERROR;
        }
    }  else {
        LOG_E("RT-Thread WK2124 package initialize fail.");
        return -RT_ERROR;
    }
#else
    return -RT_ERROR;
#endif
    LOG_I("RT-Thread WK2124 package initialize success.");
    return RT_EOK;
}

INIT_ENV_EXPORT(wk2124_device_init);

#endif /* PKG_USING_WK2124 */
#endif /* RT_USING_SERIAL_V1 */
#endif /* RT_USING_SERIAL */
