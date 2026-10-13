#### 

| Connector | Connector Pin | Signal       | STM32 Pin | Alternate Function |
| --------- | ------------- | ------------ | --------- | ------------------ |
| **CN7**   | 17            | **STEP1**    | PA15      | TIM2_CH1           |
| **CN7**   | 13            | DIR1         | PA13      |                    |
| **CN7**   | 15            | EN1          | PA14      |                    |
| **CN7**   | 16            | 3V3          | +3V3      |                    |
| **CN7**   | 8             | GND          | GND       |                    |
| **CN7**   | 20            | GND          | GND       |                    |
| **CN7**   | 22            | GND          | GND       |                    |
| **CN7**   | 28            | EN2          | PA0       |                    |
| **CN7**   | 30            | **STEP2**    | PA1       | TIM5_CH2           |
| **CN7**   | 32            | DIR2         | PA4       |                    |
| **CN7**   | 34            | SERVO_PWM    | PB0       | TIM1_CH2N          |
| **CN7**   | 38            | BTN          | PC0       |                    |
| **CN10**  | 1             | DIR3         | PC9       |                    |
| **CN10**  | 3             | **STEP3**    | PB8       | TIM10_CH1          |
| **CN10**  | 13            | EN3          | PA6       |                    |
| **CN10**  | 25            | **I2C1_SCL** | PB10      | I2C2_SCL           |
| **CN10**  | 5             | **I2C1_SDA** | PB9       | I2C1_SDA           |
| **CN10**  | 15            | EN4          | PA7       |                    |
| **CN10**  | 17            | **STEP4**    | PB6       | TIM4_CH1           |
| **CN10**  | 31            | DIR4         | PB3       |                    |
| **CN10**  | 35            | USART2_TX    | PA2       | USART2_ TX         |
| **CN10**  | 37            | USART2_RX    | PA3       | USART2_RX          |
| **CN10**  | 20            | GND          | GND       |                    |
| **CN7**   | 36            | JOY_L_X (left stick X)   | PC1  | ADC1_IN11 |
| **CN7**   | 35            | JOY_L_Y (left stick Y)   | PC2  | ADC1_IN12 |
| **CN7**   | 37            | JOY_R_X (right stick X)  | PC3  | ADC1_IN13 |
| **CN10**  | 34            | JOY_R_Y (right stick Y)  | PC4  | ADC1_IN14 |
| **CN10**  | 6             | JOY_L_SEL (left button, active low, internal pull-up)  | PC5 | GPIO input |
| **CN10**  | 24            | JOY_R_SEL (right button, active low, internal pull-up) | PB1 | GPIO input |

Joystick power: both breakouts VCC -> 3V3 (CN7 pin 16) and GND -> GND. Never power them from 5V, the ADC inputs are 3.3 V only.

Arm control mapping: left stick Y = tip Z, right stick Y = tip Y, right stick X = tip X (left stick X unused).
The SEL buttons are wired but not used by the firmware yet.
