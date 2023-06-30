# STM32F401CCU6	位置式PI 位置环

## 硬件资源

- STM32F401CCU6主控
- 直流有刷电机
- 有刷电机驱动板
- 2路PWM：控制2个电机
- 2路中断：检测2个电机编码器的A相
- 2个GPIO输入：读取2个电机编码器的B相
- 4个GPIO输出：控制2个电机的方向

### 定时器分配

| 定时器 | 工作模式 | 功能 |
|:-:|:-:|:-:|
| TIM1 | 1kHz 2路PWM | 用于PWM控制2个直流有刷电机 |

### 中断分配

| 定时器 | 工作模式 | 功能 |
|:-:|:-:|:-:|
| EXTI0 | 上升沿触发  下拉 | 检测电机1编码器 |
| EXTI1 | 上升沿触发  下拉 | 检测电机2编码器 |

## 调试过程
由于我之前用矩阵键盘调过参数，Kp、Ki和Kd的参数已经比较优秀，我便没有再使用串口或者按键去调参，Kp、Ki和Kd的值如下表：

|Kp|Ki|Kd|
|:-:|:-:|:-:|
|30|0.9|180|

另外，采样时间是**1ms**。

调试出来的波形图如下:
![位置式PID的波形图.png](./Pictures/位置式PI波形图.png)

我在这个调试中使用了「积分分离PID」，你也来试试吧（雾）。这是我大一上寒假刚开始学PID的时候自己想到的优化方法，当时并没有去了解抗饱和的方法，后来才了解到这个方法叫「积分分离PID」

我使用的阈值是500（我的电机转一圈单相有13000个脉冲）。
