---
description: EC_HAL API
---

# Embedded Controller HAL

Written by:  Your Name



Program: 		C/C++

IDE/Compiler: Keil uVision 5

OS: 					WIn10

MCU:  				STM32F411RE, Nucleo-64



![image-20211029143156542](C:\Users\sktgt\AppData\Roaming\Typora\typora-user-images\image-20211029143156542.png)



## GPIO Digital In/Out 

### Header File

 `#include "ecGPIO.h"`



```c++
#include "stm32f411xe.h"

#ifndef __EC_GPIO_H
#define __EC_GPIO_H

#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

#define NO_PUPD 0x00
#define PU		0x01
#define PD      0x02
#define RESRV 	0x03

#define LS		0x00
#define MS		0x01
#define FS      0x02
#define HS 		0x03

#define HIGH 1
#define LOW  0

#define LED_PIN 	5
#define BUTTON_PIN 13


void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
void GPIO_pudr(GPIO_TypeDef* Port, int pin, int pudr);

#endif

```




### GPIO_init\(\)

Initializes GPIO pins with default setting and Enables GPIO Clock. Mode: In/Out/AF/Analog

```c++
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **mode**:   INPUT (0), OUTPUT (1),  AF(02), ANALOG (03)

  

**Example code**

```c++
GPIO_init(GPIOA, 5, OUTPUT);
GPIO_init(GPIOC, 13, INPUT); //GPIO_init(GPIOC, 13, 0);
```



### GPIO_mode\(\)

Configures  GPIO pin modes: In/Out/AF/Analog

```c++
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **mode**:   INPUT (0), OUTPUT (1),  AF(02), ANALOG (03)

  

**Example code**

```c++
GPIO_mode(GPIOA, 5, OUTPUT);
```



### GPIO_pudr\(\)

Configure Pull-up/down register

```c++
void GPIO_pupdr(GPIO_TypeDef *Port, int pin, int pudr);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **pudr**:   NO_PUPD (0), PU (1),  PD (2), RESRV (3)

  

**Example code**

```c++
GPIO_pudr(GPIOA, 5, NO_PUPD); // 0: No PUPD
```



### GPIO_otype(\)

Configure Output Type (OTYPE): Open-drain / Push-Pull

```c++
void GPIO_otype(GPIO_TypeDef *Port, int pin, int type);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **type**:   Output push-pull (reset state) (0), Open-drain (1)

  

**Example code**

```c++
GPIO_otype(GPIOA, 5, 0); // 0: Reset state
```



### GPIO_ospeed(\)

Configure Speed (OSPEED): Low / Medium / Fast / High 

```c++
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **speed**:   LS (0), MS (1), FS (2), HS (3) 

  

**Example code**

```c++
GPIO_ospeed(GPIOA, 5, MS); // 1: Medium speed
```



### GPIO_read(\)

Read data

```c++
int  GPIO_read(GPIO_TypeDef *Port, int pin);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15



### GPIO_write(\)

Output data from data register : H or L

```c++
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int Output);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **speed**: LOW (0), HIGH (1)

  

**Example code**

```c++
while(1){
	if(GPIO_read(GPIOC, BUTTON_PIN) == 0)	GPIO_write(GPIOA, LED_PIN, HIGH);
	else 				  				    GPIO_write(GPIOA, LED_PIN, LOW);
	}
```



## 7-segment control

### sevensegment_init()

Initialization segment register setting: mode, type, pull up/pull down, speed

```c++
void sevensegment_init();
```



**Example code**

```c++
void setup(void)
{
    RCC_HSI_init();      
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()   
		GPIO_pudr(GPIOC, BUTTON_PIN, PULLUP); 
		sevensegment_init();
}
```



### sevensegment_decode()

Configuration the number that appear on the 7-segment

```c++
void sevensegment_decode(uint8_t num);
```

**Parameters**

* **num:**  the number that appears on the 7-segment



**Example code**

```c++
while(1){      
	if(GPIO_read(GPIOC, BUTTON_PIN) && tempVar){
		cnt++;
		if (cnt > 9) cnt = 0; 
		tempVar=false;
	}
	else if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
		for(int i = 0; i < 500000;i++){} 
		tempVar=true;
	}						 
	sevensegment_decode(cnt % 10); 
}
```



## SysTick Function

### SysTick_init()

Initalization System Timer

```c++
void SysTick_init(uint32_t msec)
```

**Parameters**

* **msec:**  SysTick interval

**Example code**

```c++
void setup(void)
{
	RCC_PLL_init();
	SysTick_init(1000);
	sevensegment_init();
}
```



### delay_ms()

Stop code moving

```c++
void delay_ms(uint32_t msec)
```

**Parameters**

* **msec:**  delay time



### SysTick_reset()

Reset current value to zero

```c++
void SysTick_reset();
```



### SysTick_val()

Return system timer current value 

```c++
uint32_t SysTick_val();
```



**Example code**

```c++
while(1){
	sevensegment_decode(count);
	delay_ms(1000);
	count++;
	if (count > 9) count =0;
	SysTick_reset();
}
```



### SysTick_enable()

Start SysTick Timer

```c++
void SysTick_enable();
```



### SysTick_disable()

Disable SysTick Timer

```c++
void SysTick_disable();
```



**Example code**

```c++
SysTick_disable ();
// Select processor clock
// 1 = processor clock;  0 = external clock
SysTick->CTRL |= 1UL<<2U;

// SysTick Current Value Register
SysTick->VAL = 0;

// Enables SysTick exception request
// 1 = counting down to zero asserts the SysTick exception request
SysTick->CTRL |= 1UL<<1U;
	
// Enable SysTick IRQ and SysTick Timer
SysTick_enable();
```



## EXTI Function

### EXTI_init()

Initalization EXTI registor setting

```c++
void EXTI_init(GPIO_TypeDef *port, int pin, int trig_type, int priority);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **trig_type**:   FALL (0), RISE (1),  BOTH (2)
* **priority**:  0 to 59

**Example code**

```c++
void setup(void)
{
	RCC_PLL_init();
	SysTick_init(1000);
	sevensegment_init();
}
```



### EXTI_enable()

Enable EXTI

```c++
void EXTI_enable(uint32_t pin);
```

**Parameters**

* **pin**:  pin number (int) 0~15



### EXTI_disable()

Disable EXTI

```c++
void EXTI_disable(uint32_t pin);
```

**Parameters**

* **pin**:  pin number (int) 0~15



### is_pending_EXTI()

Make pending register

```c++
uint32_t is_pending_EXTI(uint32_t pin);
```

**Parameters**

* **pin**:  pin number (int) 0~15



### clear_pending_EXTI()

Clear pending register

```c++
void clear_pending_EXTI(uint32_t pin);
```

**Parameters**

* **pin**:  pin number (int) 0~15



**Example code**

```c++
if (is_pending_EXTI(BUTTON_PIN)) {
	LED_toggle();
	clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
}
```



## TIMER Function

### TIM_init()

Initalization TIMER setting

```c++
void TIM_init(TIM_TypeDef* timerx, uint32_t msec);
```

**Parameters**

* **timerx**  Timer Number,  TIM1~TIM11
* **msec**:  Time to make period

**Example code**

```c++
void TIM_INT_init(TIM_TypeDef* timerx, uint32_t msec){
// 1. Initialize Timer	
	TIM_init(timerx,msec);
	
// 2. Enable Update Interrupt
	uint32_t IRQn_reg =0;
	if(timerx ==TIM1)       IRQn_reg = TIM1_UP_TIM10_IRQn;
    
    NVIC_EnableIRQ(IRQn_reg);				
	NVIC_SetPriority(IRQn_reg,2);
}
```



### TIM_period_us()

Set the timer cycle in us units.

```c++
void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec);
```

**Parameters**

* **timerx**  Timer Number,  TIM1~TIM11
* **msec**:  Time to make period in us



### TIM_period_ms()

Set the timer cycle in ms units.

```c++
void TIM_period_ms(TIM_TypeDef *TIMx, uint32_t msec);
```

**Parameters**

* **timerx**  Timer Number,  TIM1~TIM11
* **msec**:  Time to make period in ms



### TIM_INT_init()

Update event interrupt

```c++
void TIM_INT_init(TIM_TypeDef* timerx, uint32_t msec);
```

**Parameters**

* **timerx**  Timer Number,  TIM1~TIM11
* **msec**:  Time to make period



### TIM_INT_enable();

Enable Timer update interrupt

```c++
void TIM_INT_enable(TIM_TypeDef* timerx);
```

**Parameters**

* **timerx**  Timer Number,  TIM1~TIM11



### TIM_INT_disable();

Enable Timer update interrupt

```c++
void TIM_INT_disable(TIM_TypeDef* timerx);
```

**Parameters**

* **timerx**  Timer Number,  TIM1~TIM11



### is_UIF();

Update interrupt flag pended

```c++
uint32_t is_UIF(TIM_TypeDef *TIMx);
```

**Parameters**

* **timerx**  Timer Number,  TIM1~TIM11



### clear_UIF();

Clear update interrupt flag

```c++
void clear_UIF(TIM_TypeDef *TIMx);
```

**Parameters**

* **timerx**  Timer Number,  TIM1~TIM11



## PWM Function

### PWM_init()

Initalization PWM out setting

```c++
void PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin);
```

**Parameters**

* **pwm**:  PWM variable
* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15



### PWM_period_us()

Set the PWM period in us units.

```c++
void PWM_period_us(PWM_t *PWM_pin, uint32_t usec);
```

**Parameters**

* **pwm**:  PWM variable
* **usec**:  Time to make period



### PWM_period_ms()

Set the PWM period in ms units.

```c++
void PWM_period_ms(PWM_t *PWM_pin, uint32_t msec);
```

**Parameters**

* **pwm**:  PWM variable
* **msec**:  Time to make period



### PWM_pulsewidth_ms()

Set the PWM pulse width

```c++
void PWM_pulsewidth_ms(PWM_t *pwm, float pulse_width_ms);
```

**Parameters**

* **pwm**:  PWM variable
* **pulse_width_ms**:  pulse width want to make



### PWM_duty()

Set the PWM duty

```c++
void PWM_duty(PWM_t *pwm, float duty)
```

**Parameters**

* **pwm**:  PWM variable
* **duty**:  duty want to make



## Stepper Motor Function

### Stepper_init()

Initalization 4 pin using at stepper motor

```c++
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15



### Stepper_setSpeed()

Convert rpm to milli sec, set Time delay between each step

```c++
void Stepper_setSpeed (long whatSpeed);
```

**Parameters**

* **whatSpeed:** Speed that we want to run motor



### Stepper_step()

run for n Step

```c++
void Stepper_step(int steps, int direction,int mode);
```

**Parameters**

* **steps:**  total number of steps of the motor we want to spin.
* **direction**:  run motor CW/CCW  (0: CCW, 1: CW)
* **mode**:  FULL: Full stepping sequence, HALF: half stepping sequence



### Stepper_step()

Stop the stepper motor

```c++
void Stepper_stop (void);
```

