# Monotroch
STM32F7 contest, the robot project

eeworld论坛举办的STM32F7大赛，独轮机器人自平衡项目。
本首研究和学习的目的开源些项目的控制源码。
开发工具使用Stm32CubeMX生成基本配置代码，编辑编译采用Keil ARM 5,MCU采用STM32F746-Discovery开发板，硬件模块包括MPU6050,串口蓝牙，L9110电机驱动。电机采用OKI42步进电机，机械部分下部行走轮，上部垂直惯性飞轮。
软件采用角度，速度双闭环非线性PID控制。

项目文件目录结构图：
\MONOTROCH
│  .gitignore
│  .mxproject
│  LICENSE
│  Monotroch.ioc
│  README.md
│  
├─Drv （外围设备驱动）
│  └─eMPL
│          dmpKey.h
│          dmpmap.h
│          inv_mpu.c
│          inv_mpu.h
│          inv_mpu_dmp_motion_driver.c
│          inv_mpu_dmp_motion_driver.h
│          
├─Inc  （所有的头文件）
│      FreeRTOSConfig.h
│      gpio.h
│      i2c.h
│      mpu.h
│      mxconstants.h
│      stm32f7xx_hal_conf.h
│      stm32f7xx_it.h
│      tim.h
│      up_computer.h
│      usart.h
│      
├─MDK-ARM  （项目文件）
│      Monotroch.uvguix.Goth
│      Monotroch.uvoptx
│      Monotroch.uvprojx
│      
└─Src   （源码）
        freertos.c
        gpio.c
        i2c.c
        main.c
        mpu.c
        retarget.c
        stm32f7xx_hal_msp.c
        stm32f7xx_it.c
        tim.c
        up_computer.c
        usart.c
        
