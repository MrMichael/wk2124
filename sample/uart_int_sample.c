#include <rtthread.h>
#include <rtdevice.h>

/* 将子串口1和子串口2连接测试 */
#define WK1_UART_NAME       "uartswk1"      /* 串口设备名称 */
#define WK2_UART_NAME       "uartswk2"      /* 串口设备名称 */

static rt_device_t serial_1, serial_2;
static struct rt_semaphore rx_sem;

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem);
    return RT_EOK;
}

static int uart_int_sample(void)
{
    rt_err_t ret = RT_EOK;
    char str[] = "hello RT-Thread!\r\n";
    char ch;

    /* 查找串口设备 */
    serial_1 = rt_device_find(WK1_UART_NAME);
    serial_2 = rt_device_find(WK2_UART_NAME);
    if (!serial_1)
    {
        rt_kprintf("find %s failed!\n", WK1_UART_NAME);
        return RT_ERROR;
    }
    if (!serial_2)
    {
        rt_kprintf("find %s failed!\n", WK2_UART_NAME);
        return RT_ERROR;
    }

    /* 初始化信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    /* 修改串口1配置 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_4800;
    config.rx_bufsz = 1024;
    config.tx_bufsz = 0;    //必须重新配置为0：阻塞发送，非阻塞接收
    rt_device_control(serial_1, RT_DEVICE_CTRL_CONFIG, &config);
    rt_device_control(serial_2, RT_DEVICE_CTRL_CONFIG, &config);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial_2, uart_input);
    /* 打开串口 */
    rt_device_open(serial_1, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
    rt_device_open(serial_2, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
    /* 发送字符串 */
    rt_device_write(serial_1, 0, str, (sizeof(str) - 1));

    while (1)
    {
        /* 阻塞等待串口接收中断 */
        ret = rt_sem_take(&rx_sem, rt_tick_from_millisecond(50));
        if (ret == RT_EOK)
        {
            while (rt_device_read(serial_2, -1, &ch, 1))
            {
                rt_kprintf("%s: %c\n", WK2_UART_NAME, ch);
            }
        }
        else
        {
            break;
        }
    }
    /* 关闭串口 */
    rt_device_close(serial_1);
    rt_device_close(serial_2);
    /* 脱离信号量 */
    rt_sem_detach(&rx_sem);

    return ret;
}

/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(uart_int_sample, uart device interrupt sample);
