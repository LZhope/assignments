# Week 4 - Embedded programming

本周，我深入研究了两款主流嵌入式开发平台：Arduino Uno R4 和树莓派 4B。通过详细阅读技术文档和实践操作，我对这两个平台的硬件架构、开发特点和应用场景有了全面的认识。主要工作包括：

1. 系统性研究了 Arduino Uno R4 和树莓派 4B 的硬件规格、接口特性和开发生态
2. 对比分析了两个平台在处理能力、实时性、功耗和应用场景等方面的差异
3. 完成了两个典型的嵌入式控制案例：

   - 基于 Arduino Uno R4 平台，使用 C++ 开发了 WS2812 可编程 LED 灯带控制程序，实现了多种动态光效
   - 在树莓派 4B 上，使用 Python 编程实现了舵机的精确位置控制和运动序列编程


## 数据手册阅读
我阅读了 Arduino Uno R4和 树莓派 4B 的数据手册，重点关注了它们的 处理器架构、I/O 资源、通信接口、功耗管理等方面。

### Arduino Uno R4

Arduino Uno R4的核心MCU是瑞萨电子的RA4M1。该芯片基于ARM Cortex-M4架构，运行频率为48MHz，具有32KB SRAM和256KB闪存。相较于Uno R3，处理器升级为32位48MHz的Renesas RA4M1，性能大幅提升，SRAM从2KB增至32KB、闪存从32KB增至256KB，电源输入范围提升至6-24V，USB接口升级为Type-C，还新增了12位DAC、CAN总线，WiFi版增加了ESP32-S3模块支持WiFi和蓝牙。

![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/Snipaste_2025-02-17_00-56-20.jpg)


#### 1.1 核心参数

| Parameter          | Specification                                                          |
|-------------------|-----------------------------------------------------------------------|
| Name              | Arduino Uno R4                                                         |
| Microcontroller   | Renesas RA4M1 (ARM Cortex-M4)                                         |
| Operating Voltage | 3.3V                                                                   |
| Input Voltage     | 6-24V (recommended 7-12V)                                             |
| GPIO Pins         | 20 (14 digital I/O + 6 analog inputs)                                 |
| Digital Pins      | 14 (including 4 PWM outputs)                                          |
| PWM Pins          | 4 (GPIO 3/5/6/9/10)                                                  |
| Analog Input Pins | 6 (10-bit ADC)                                                        |
| I2C Port          | 1 (A4/SDA, A5/SCL)                                                   |
| UART Port         | 1 (Hardware Serial)                                                   |
| SPI Port          | 1 (ICSP header)                                                       |
| Flash Memory      | 256KB Flash                                                           |
| SRAM              | 32KB                                                                  |
| EEPROM            | 8KB                                                                   |
| Clock Speed       | 48MHz                                                                 |


Arduino UNO R4主板的详细PCB布局如下所示:

![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/20250217012147915.png)



---



### 2. 树莓派 4B
树莓派4B是一款搭载博通BCM2711芯片、四核Cortex-A72架构1.5GHz主频，有1GB-8GB内存，具双Micro-HDMI、双频无线、蓝牙5.0、千兆网口、4个USB口及40针GPIO接口，支持4K显示和4KP60硬件视频解码，5V/3A供电的微型计算机。

![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/20250217063131034.png)

#### 2.1 核心参数
| 参数项            | 规格描述                                                                 |
|-------------------|------------------------------------------------------------------------|
| 名称              | Raspberry Pi 4 Model B                                                |
| SoC              | Broadcom BCM2711                                                      |
| 工作电压          | 5V（通过USB-C）                                                       |
| 输入电压          | 5V                                                            |
| GPIO引脚数        | 40                                                                    |
| 数字引脚          | 40（所有GPIO均为数字引脚）                                            |
| PWM引脚           | 4个（GPIO12/13/18/19，2个独立通道）                                   |
| 模拟输入引脚       | 无（需外接ADC模块）                                                   |
| I2C端口           | 2个（I2C-0和I2C-1）                                                   |
| UART端口          | 2个（主UART + mini UART）                                             |
| SPI端口           | 2个（SPI0和SPI1）                                                     |
| 存储              | microSD卡（支持UAS启动）                                              |
| 内存             | 1/2/4/8GB LPDDR4                                                      |
| 视频输出          | 双micro HDMI（支持4K60）                                              |
| 网络接口          | 千兆以太网 + 双频WiFi 5                                               |
| 时钟速度          | 1.5GHz（四核Cortex-A72）                                              |

---

### 3. 架构对比分析
| 特性               | Arduino Uno R4           | 树莓派 4B               |
|--------------------|--------------------------|------------------------|
| **处理核心**       | Cortex-M4               | Cortex-A72 x4         |
| **无线连接**       | 需外接模块              | WiFi5 + BT5           |
| **存储扩展**       | 无                      | microSD卡             |
| **开发接口**       | Arduino IDE             | Python/C++/多语言     |
| **实时性能**       | μs级响应               | 无实时保证            |
| **典型功耗**       | 50mA（工作）           | 600mA（空闲）         |
| **外设接口**       | CAN总线                 | USB 3.0 x2            |
| **模拟输入**       | 6通道10位ADC           | 需外接ADC             |
| **PWM分辨率**      | 8位                    | 10位                 |
| **最佳适用场景**   | 工业控制/传感器采集     | 边缘计算/多媒体中心   |
---

## 案例1：Arduino Uno R4 ws2812控制
本案例使用 Arduino Uno R4 微控制器驱动两块定制的六边形 PCB 板。每块 PCB 板上均匀分布着 18 颗 WS2812 可编程 RGB LED。本案例目标是创建一个流畅的流水灯效果，使 LED 灯按照预设的顺序和时序在两块六边形板上循环点亮，形成连贯的视觉动画效果。

### **1. 硬件配置**
### **所需组件**
- ws2812灯带
- Arduino Uno R4
- 杜邦线

### **2. 安装 Arduino IDE**
1. 从 [Arduino 官方网站](https://www.arduino.cc/en/software) 下载 Arduino IDE。
2. 下载完成后，打开 Arduino IDE，并将 **Arduino Uno R4** 插入电脑。
3. 在开发板管理器中，选择 **Arduino Uno R4**，系统会提示下载支持 Uno R4 的软件包，点击 **下载** 即可。

![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/20250217021708311.png)

![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/20250217022019909.png)

### **3. Arduino 编程基础**
Arduino 编程结合了 **C 和 C++** 的特性，在新建的文件夹下，默认包含两个函数：`setup()` 和 `loop()`，它们的作用如下：

#### **setup() 函数**
- 主要用于初始化设置，如配置引脚模式、初始化串口通信波特率、变量赋值等。
- 该函数在程序运行时 **只执行一次**。

#### **loop() 函数**
- 该函数会不断循环执行，控制程序的主要功能，如 **读取传感器数据、控制执行器、处理数据** 等。

---


### **4. 硬件连接**
1. WS2812 LED 灯带有三根线：
- VCC（电源）：连接到 5V 电源
- GND（接地）：连接到 Arduino GND
- DIN（数据输入）：连接到 Arduino 数字引脚 2（支持 PWM）

2. 连接步骤：
- 电源连接：
* 将 VCC 线连接到外部 5V 电源
* 将 GND 线连接到电源接地和 Arduino GND
![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img-hwj/cc516ddb2145c3047fe242fca4d02ab.png)
![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img-hwj/712472f764bbc27df5a2cc7d245ab22.jpg)
![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img-hwj/0a61a1f42c86af50c04f2004b0e86c88_raw.mp4)
---

### **5. 代码实现**

```C
#include <FastLED.h>

// Configuration parameters
#define LED_PIN 6         // Control pin
#define NUM_LEDS 36       // Total number of LEDs
#define PCB_COUNT 2       // Number of PCBs
#define LEDS_PER_PCB 18   // Number of LEDs per PCB
#define COLUMNS 9         // Number of columns
#define UPDATE_INTERVAL 80 // Interval time

CRGB leds[NUM_LEDS];       // LED array
uint8_t currentColumn = 0; // Current active column
unsigned long lastUpdate;  // Time record

void setup() {
  FastLED.addLeds<WS2812, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(50);  // Set brightness
  FastLED.clear(true);
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentMillis;
    
    // Turn off all LEDs in the previous column
    for (uint8_t p = 0; p < PCB_COUNT; p++) {
      uint16_t base = p * LEDS_PER_PCB;
      uint8_t prevCol = (currentColumn + COLUMNS - 1) % COLUMNS;
      
      // Turn off the two symmetrical LEDs
      leds[base + prevCol] = CRGB::Black;
      leds[base + (LEDS_PER_PCB - 1) - prevCol] = CRGB::Black;
    }

    // Light up the current column
    for (uint8_t p = 0; p < PCB_COUNT; p++) {
      uint16_t base = p * LEDS_PER_PCB;
      
      // Set symmetrical LEDs color (example uses gradient color)
      leds[base + currentColumn] = CHSV(0, 255, 255);  
      leds[base + (LEDS_PER_PCB - 1) - currentColumn] = CHSV(0, 255, 255);
    }

    currentColumn = (currentColumn + 1) % COLUMNS;
    
    FastLED.show();
  }
}

```


## **案例2：树莓派 4B 控制 舵机**
本案例使用树莓派 4B 的 GPIO18 引脚通过 PWM 信号控制舵机，实现了 0-180 度范围内的精确角度定位。程序通过循环控制使舵机在 0°、90°、180° 三个位置之间往复运动，展示了基于 Python 的 RPi.GPIO 库实现舵机的精确位置控制和基础运动序列编程。
### **1. 硬件配置**
#### **所需组件**
- 树莓派 4B  
- 舵机  
- 5V/2A 电源  
- 杜邦线 

---

### **2. 安装树莓派系统**

1. **准备工作**：
   - 下载并安装 **Raspberry Pi Imager**（树莓派官方烧录工具）
   - 准备一张容量不小于 8GB 的 microSD 卡
   - 准备读卡器将 microSD 卡连接到电脑
![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/Snipaste_2025-02-17_15-44-37.jpg)

2. **系统烧录**：
   - 打开 Raspberry Pi Imager
   ![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/Snipaste_2025-02-17_15-44-37.jpg)
   - 点击"选择操作系统"，在列表中选择 **Raspberry Pi OS (64-bit)**，点击"选择存储卡"，选择要烧录的 microSD 卡
   ![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/Snipaste_2025-02-17_16-07-35.jpg)
   - 点击下一步后，可以进行一些预设：
     * WiFi 网络配置
     * 用户名和密码
     * 主机名
     * SSH 开关等
   - 点击"写入"开始烧录系统
   ![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/Snipaste_2025-02-17_15-50-02.jpg)
3. **首次启动**：
   - 将烧录好的 microSD 卡插入树莓派
   - 接通电源（推荐使用官方电源适配器，输出 5V/3A）
   - 等待系统自动完成初始化配置
   ![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/20250217183256231.png)


4. **VNC远程操控**：
   
    为了方便远程操作树莓派，我采用了 [VNC](https://www.realvnc.com/en/connect/download/viewer/)（Virtual Network Computing）远程桌面方案：

   1. 首先下载并安装 VNC Viewer
   ![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/Snipaste_2025-02-17_15-17-17.jpg)

   2. 在树莓派上启用 VNC Server（通过 raspi-config 的 Interface Options 中开启）
   ![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/Snipaste_2025-02-17_18-20-25.jpg)
   3. 在树莓派终端使用 `ifconfig` 命令查看树莓派的 IP 地址（本例中为 10.0.0.92）
   ![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/20250217185314037.png)
   4. 打开 VNC Viewer，在连接地址栏输入树莓派的 IP 地址
   ![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/Snipaste_2025-02-17_18-18-25.jpg)
   5. 输入树莓派的用户名和密码完成认证
   ![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/Snipaste_2025-02-17_18-19-18.jpg)
   6. 连接成功后即可通过 VNC 远程操控树莓派的图形界面
   ![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/Snipaste_2025-02-17_18-19-39.jpg)
5. **配置硬件接口**：
   - 打开终端，输入 `sudo raspi-config` 进入配置界面，并选择 "Interface Options"
   ![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/20250217185440028.png)
   - 依次启用以下接口：
     * GPIO（用于舵机控制）
     * PWM（用于生成舵机控制信号）

     ![](https://nexmaker-profabx.oss-cn-hangzhou.aliyuncs.com/img/liang/20250217190007139.png)

   - 完成后选择 "Finish" 并重启树莓派
   - 可以通过以下命令验证接口是否启用：
     * `gpio readall` 检查 GPIO 状态
     * `ls /dev/i2c*` 检查 I2C
     
---

### **3. 树莓派 Python 编程基础**
树莓派支持多种编程语言，但 Python 因其简洁性和丰富的库支持成为最受欢迎的选择。本案例使用 `RPi.GPIO` 库控制舵机，实现精确的角度控制和运动序列。

#### **代码编写**
#### **安装依赖库**
在树莓派终端运行以下命令，安装必要的库：
```bash
sudo apt update
sudo apt install python3-pip
pip3 install RPi.GPIO
```

代码实现

```python
import RPi.GPIO as GPIO
import time

# Configuration parameters
SERVO_PIN = 18          # Servo control pin (GPIO18)
FREQ = 50              # PWM frequency (50Hz)
MIN_DUTY = 2.5         # Minimum duty cycle (0 degrees)
MAX_DUTY = 12.5        # Maximum duty cycle (180 degrees)

# Initialize GPIO settings
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, FREQ)
pwm.start(0)

def angle_to_duty_cycle(angle):
    """Convert angle to duty cycle"""
    duty = MIN_DUTY + (MAX_DUTY - MIN_DUTY) * (angle / 180)
    return duty

def set_angle(angle):
    """Set servo angle"""
    duty = angle_to_duty_cycle(angle)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.3)  # Wait for the servo to reach position

try:
    while True:
        # Demonstrate servo movement sequence
        print("Moving to 0 degrees")
        set_angle(0)
        time.sleep(1)
        
        print("Moving to 90 degrees")
        set_angle(90)
        time.sleep(1)
        
        print("Moving to 180 degrees")
        set_angle(180)
        time.sleep(1)
        
        print("Moving to 90 degrees")
        set_angle(90)
        time.sleep(1)

except KeyboardInterrupt:
    print("Program exiting")
    pwm.stop()
    GPIO.cleanup()

```
---

### **4. 硬件连接与运行**
本案例用到了树莓派的 GPIO18 引脚，通过 PWM 信号控制舵机。
1. 舵机连接 ：
- 红线（VCC）：连接到树莓派的 5V 引脚
- 棕线（GND）：连接到树莓派的 GND 引脚
- 橙线（信号线）：连接到 GPIO18（PWM 引脚）

2. 运行程序
```bash
sudo /bin/python /home/xusun/demo.py
```


