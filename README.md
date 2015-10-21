# Monotroch
STM32F7 contest, the robot project

eeworld论坛举办的STM32F7大赛，独轮机器人自平衡项目。
本首研究和学习的目的开源些项目的控制源码。
开发工具使用Stm32CubeMX生成基本配置代码，编辑编译采用Keil ARM 5,MCU采用STM32F746-Discovery开发板，硬件模块包括MPU6050,串口蓝牙，L9110电机驱动。电机采用OKI42步进电机，机械部分下部行走轮，上部垂直惯性飞轮。
软件采用角度，速度双闭环非线性PID控制。
