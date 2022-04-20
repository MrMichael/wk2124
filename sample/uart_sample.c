#include <rtthread.h>
#include <rtdevice.h>

/* 将子串口1和子串口2连接测试 */
#define WK1_UART_NAME       "uartswk1"      /* 串口设备名称 */
#define WK2_UART_NAME       "uartswk2"      /* 串口设备名称 */

static rt_device_t serial_1, serial_2;

static int uart_sample(void)
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
    /* 修改串口配置 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_4800;
    config.rx_bufsz = 1024;
    config.tx_bufsz = 0;    //必须重新配置为0：阻塞发送，非阻塞接收
    rt_device_control(serial_1, RT_DEVICE_CTRL_CONFIG, &config);
    rt_device_control(serial_2, RT_DEVICE_CTRL_CONFIG, &config);
    /* 打开串口 */
    rt_device_open(serial_1, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
    rt_device_open(serial_2, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
    /* 发送字符串 */
    rt_device_write(serial_1, 0, str, (sizeof(str) - 1));

    while (1)
    {
        rt_thread_mdelay(100);
        if (rt_device_read(serial_2, -1, &ch, 1))
        {
            rt_kprintf("%s: %c\n", WK2_UART_NAME, ch);
        }
        else
        {
            break;
        }
    }
    /* 关闭串口 */
    rt_device_close(serial_1);
    rt_device_close(serial_2);

    return ret;
}

/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(uart_sample, uart device sample);
