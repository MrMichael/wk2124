# WK2124 软件包

## 1 介绍

WK2124 软件包是为[WK2124 SPI转四串口扩展芯片](http://www.wkmic.com/News_Show.php?theId=17)而开发的RT-Thread驱动包。通过`RT-Thread SPI` 设备和`RT-Thread UART` 设备驱动框架，为RT-Thread系统新增最多四路串口字符设备。与片上串口使用方式一致，实现了对串口设备进行open、close、read、write、control操作的功能，并支持轮询和中断接收两种模式。

WK2124 软件包为串口设备资源不足的产品提供串口扩展的方法，目前已在stm32f429验证通过。


### 1.1 目录结构

```shell
wk2124-latest/
├── ChangeLog.md		# 修改记录
├── LICENSE						# 软件包许可证
├── README.md			  # 软件包使用说明
├── sample						 # 串口使用程序样例
│   ├── uart_int_sample.c		# 中断模式
│   └── uart_sample.c				# 轮询模式
├── SConscript				  # RT-Thread 默认的构建脚本
├── wk2124s.c				  # wk2124 驱动
├── wk2124s.h				  # wk2124 驱动头文件
├── wk2124_usart.c      # wk2124 spi及uart驱动文件
└── wk2124_usart.h	    # wk2124 spi及uart驱动头文件
```

### 1.2 许可证

AT24CXX 软件包遵循  Apache-2.0 许可，详见 LICENSE 文件。

### 1.3 依赖

- RT-Thread 4.0.1+
- SPI 驱动：WK2124 设备使用 SPI 进行数据通讯，需要系统 SPI 驱动框架支持；
- PIN 驱动：用于处理设备中断引脚；



## 2 获取软件包

使用 `wk2124` 软件包需要在 RT-Thread 的包管理器中选择它，具体路径如下：

```
RT-Thread online packages
    peripheral libraries and drivers  --->
        [*] wk2124: spi wk2124 driver library.  --->
           --- wk2124: spi wk2124 driver library.
            [*]   Enable UART SWK1
            [*]   Enable UART SWK2
            [*]   Enable UART SWK3
            [*]   Enable UART SWK4
            WK2124 device configure  --->
            	(spi2dev) SPI device name
            	(17) IRQ pin number
            	(11059200) Set crystal frequency. Unit Hz
            Version (latest)  --->
```


每个功能的配置说明如下：

- Enable UART SWK1：使能串行字符设备uart swk1
- Enable UART SWK2：使能串行字符设备uart swk2
- Enable UART SWK3：使能串行字符设备uart swk3
- Enable UART SWK4：使能串行字符设备uart swk4
- WK2124 device configure：配置使用设备的参数
  - SPI device name：配置使用 SPI 的设备名称（注意需设置为**非 SPI 总线设备**）
  - IRQ pin number：配置设备连接的中断引脚号（根据实际使用引脚号修改）
  - Set crystal frequency：wk2124硬件电路的晶振频率（影响串口波特率）
- Version：配置软件包版本，默认最新版本。

然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。



## 3 使用 wk2124 软件包

按照前文介绍，获取 wk2124 软件包并配置参数项。软件包编译进RT-Thread系统以后，通过宏定义INIT_ENV_EXPORT(wk2124_device_init)，软件包会在开机时自动初始化，并产生串行字符设备。

```shell
# 初始化信息
\ | /
- RT -     Thread Operating System
 / | \     4.0.1 build Aug 25 2020
 2006 - 2019 Copyright by rt-thread team
.....
[I/drv] RT-Thread WK2124 package initialize success.
......
msh />

# 串行字符设备
msh />list_device
device           type         ref count
-------- -------------------- ----------
......
uartswk4 Character Device     0       
uartswk3 Character Device     0       
uartswk2 Character Device     0       
uartswk1 Character Device     0
......
```

uartswk1--4串行字符设备支持轮询和中断接收两种模式（不支持DMA模式），使用方法可参考[RT-Thread UART设备使用文档](https://www.rt-thread.org/document/site/programming-manual/device/uart/uart/)和[sample](sample/)文件夹。



## 4 注意事项

- 获取软件包时，需要注意正确配置使用的 SPI 设备名称、中断引脚号和芯片晶振频率；
- 推荐使用最新版，V1.0.0波特率计算存在BUG，且wk2124的串口关闭不彻底。



## 5 联系方式

* 维护：[liyaohong(MrMichael)](https://github.com/MrMichael)
* 主页：https://github.com/MrMichael/wk2124.git
* 邮箱：1035285359@qq.com

