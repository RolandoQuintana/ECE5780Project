/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
#include "motorDriver.h"

volatile int16_t error_integral = 0;    // Integrated error signal
volatile uint8_t duty_cycle = 0;    	// Output PWM duty cycle
volatile int16_t target_rpm = 0;    	// Desired speed target
volatile int16_t motor_speed = 0;   	// Measured motor speed
volatile int8_t adc_value = 0;      	// ADC measured motor current
volatile int16_t error = 0;         	// Speed error signal
volatile uint8_t Kp = 1;            	// Proportional gain
volatile uint8_t Ki = 1;            	// Integral gain

void motor_init(void) {
	pwm_init();
}



void pwm_init(void) {
  
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN_Msk;
	TIM3->PSC = 19;
	TIM3->ARR = 1000;
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN_Msk;
	TIM2->PSC = 19;
	TIM2->ARR = 1000;
	//TIM3->DIER |= (1<<0);
//BACK-LEFT
	TIM3->CCMR1 &= ~(1<<9);
	TIM3->CCMR1 &= ~(1<<8);   //Set CC2S to output
	TIM3->CCMR1 &=  ~(1<<1);
	TIM3->CCMR1 &=  ~(1<<0);   //Set CC1S to output
	
	TIM3->CCMR1 |= (1<<6);
	TIM3->CCMR1 |= (1<<5);
	TIM3->CCMR1 &= ~(1<<4);    //Set channel 1 to PWM mode 1
	
	TIM3->CCMR1 |= (1<<14);
	TIM3->CCMR1 |= (1<<13);
	TIM3->CCMR1 &= ~(1<<12);   //Set channel 2 to PWM mode 2
	
	TIM3->CCMR1 |= (1<<3);
	TIM3->CCMR1 |= (1<<11);    //Set channels 1 and 2 to preload enable


	
	//FRONT-LEFT
	TIM3->CCMR2 &= ~(1<<9);
	TIM3->CCMR2 &= ~(1<<8);   //Set CC2S to output
	
	TIM3->CCMR2 &=  ~(1<<1);
	TIM3->CCMR2 &=  ~(1<<0);   //Set CC1S to output
	
	TIM3->CCMR2 |= (1<<6);
	TIM3->CCMR2 |= (1<<5);
	TIM3->CCMR2 &= ~(1<<4);    //Set channel 1 to PWM mode 2
	
	TIM3->CCMR2 |= (1<<14);
	TIM3->CCMR2 |= (1<<13);
	TIM3->CCMR2 &= ~(1<<12);   //Set channel 2 to PWM mode 2
	
	TIM3->CCMR2 |= (1<<3);
	TIM3->CCMR2 |= (1<<11);    //Set channels 1 and 2 to preload enable

//	TIM3->CCER |= (1<<0);
//	TIM3->CCER |= (1<<4); //Set output enable







////////////////////////////////////////
	TIM3->CCER |= (1<<0);
	TIM3->CCER |= (1<<4); //Set output enable
	TIM3->CCER |= (1<<8);
	TIM3->CCER |= (1<<12); //Set output enable

	TIM3->CCR1 = 1100;
	TIM3->CCR2 = 1100;
	TIM3->CCR3 = 1100;
	TIM3->CCR4 = 1100;
	
	TIM3->CR1 |= (1<<0);
		
///////////TIM2////////////
	//BACK-RIGHT
	TIM2->CCMR1 &= ~(1<<9);
	TIM2->CCMR1 &= ~(1<<8);   //Set CC2S to output
	TIM2->CCMR1 &=  ~(1<<1);
	TIM2->CCMR1 &=  ~(1<<0);   //Set CC1S to output
	
	TIM2->CCMR1 |= (1<<6);
	TIM2->CCMR1 |= (1<<5);
	TIM2->CCMR1 &= ~(1<<4);    //Set channel 1 to PWM mode 2
	
	TIM2->CCMR1 |= (1<<14);
	TIM2->CCMR1 |= (1<<13);
	TIM2->CCMR1 &= ~(1<<12);   //Set channel 2 to PWM mode 2
	
	TIM2->CCMR1 |= (1<<3);
	TIM2->CCMR1 |= (1<<11);    //Set channels 1 and 2 to preload enable


	
	//FRONT-RIGHT
	TIM2->CCMR2 &= ~(1<<9);
	TIM2->CCMR2 &= ~(1<<8);   //Set CC2S to output
	
	TIM2->CCMR2 &=  ~(1<<1);
	TIM2->CCMR2 &=  ~(1<<0);   //Set CC1S to output
	
	TIM2->CCMR2 |= (1<<6);
	TIM2->CCMR2 |= (1<<5);
	TIM2->CCMR2 &= ~(1<<4);    //Set channel 3 to PWM mode 2
	
	TIM2->CCMR2 |= (1<<14);
	TIM2->CCMR2 |= (1<<13);
	TIM2->CCMR2 &= ~(1<<12);   //Set channel 4 to PWM mode 2
	
	TIM2->CCMR2 |= (1<<3);
	TIM2->CCMR2 |= (1<<11);    //Set channels 1 and 2 to preload enable



////////////////////////////////////////
	TIM2->CCER |= (1<<0);
	TIM2->CCER |= (1<<4); //Set output enable
	TIM2->CCER |= (1<<8);
	TIM2->CCER |= (1<<12); //Set output enable

	TIM2->CCR1 = 1100;
	TIM2->CCR2 = 1100;
	TIM2->CCR3 = 1100;
	TIM2->CCR4 = 1100;
	
	TIM2->CR1 |= (1<<0);
	
//PB10
//SET TO ALT FUNC
GPIOB->MODER &= ~(1<<20); //clear 0
GPIOB->MODER |= (1<<21); //clear 1
//SET push-pull
GPIOB->OTYPER &= ~(1<<10);//~(0x100); //clear 8th bit
//set to low speed  x0
GPIOB->OSPEEDR &= ~(1<<20);
//set PUPDR to pull-down 10
GPIOB->PUPDR &= ~(1<<20);
GPIOB->PUPDR &= (1<<21);

//PB11
//SET TO ALT FUNC
GPIOB->MODER &= ~(1<<22); //clear 0
GPIOB->MODER |= (1<<23); //clear 1
//SET push-pull
GPIOB->OTYPER &= ~(1<<11);//~(0x100); //clear 8th bit
//set to low speed  x0
GPIOB->OSPEEDR &= ~(1<<22);
//set PUPDR to pull-down 10
GPIOB->PUPDR &= ~(1<<22);
GPIOB->PUPDR &= (1<<23);
	


//PA0
//SET TO ALT FUNC
GPIOA->MODER &= ~(1<<0); //clear 0
GPIOA->MODER |= (1<<1); //clear 1
//SET push-pull
GPIOA->OTYPER &= ~(1<<0);//~(0x100); //clear 8th bit
//set to low speed  x0
GPIOA->OSPEEDR &= ~(1<<0);
//set PUPDR to pull-down 10
GPIOA->PUPDR &= ~(1<<0);
GPIOA->PUPDR &= (1<<1);

//PA1
//SET TO ALT FUNC
GPIOA->MODER &= ~(1<<2); //clear 0
GPIOA->MODER |= (1<<3); //clear 1
//SET push-pull
GPIOA->OTYPER &= ~(1<<1);//~(0x100); //clear 8th bit
//set to low speed  x0
GPIOA->OSPEEDR &= ~(1<<2);
//set PUPDR to pull-down 10
GPIOA->PUPDR &= ~(1<<2);
GPIOA->PUPDR &= (1<<3);

//PA2
//SET TO ALT FUNC
GPIOA->MODER &= ~(1<<4); //clear 0
GPIOA->MODER |= (1<<5); //clear 1
//SET push-pull
GPIOA->OTYPER &= ~(1<<2);//~(0x100); //clear 8th bit
//set to low speed  x0
GPIOA->OSPEEDR &= ~(1<<4);
//set PUPDR to pull-down 10
GPIOA->PUPDR &= ~(1<<4);
GPIOA->PUPDR &= (1<<5);

//PA3
//SET TO ALT FUNC
GPIOA->MODER &= ~(1<<6); //clear 0
GPIOA->MODER |= (1<<7); //clear 1
//SET push-pull
GPIOA->OTYPER &= ~(1<<3);//~(0x100); //clear 8th bit
//set to low speed  x0
GPIOA->OSPEEDR &= ~(1<<6);
//set PUPDR to pull-down 10
GPIOA->PUPDR &= ~(1<<6);
GPIOA->PUPDR &= (1<<7);


	


//PIN_C6
//SET as general Alternate Function
GPIOC->MODER &= ~(1<<12);//0x10000; //clear 12 bit
GPIOC->MODER |= (1<<13);//~(0x20000); // set the 13 bit 
//SET push-pull
GPIOC->OTYPER &= ~(1<<6);//~(0x100); //clear 8th bit
//set to low speed
GPIOC->OSPEEDR &= ~(1<<12);//~(0x10000); //clear 12th bit
//set no pull up or down
GPIOC->PUPDR &= ~(1<<12);//~(0x10000); //clear 12th bit
GPIOC->PUPDR |= (1<<13); // Clear the 13th bit

//PIN_C7
//SET as general Alternate Function
GPIOC->MODER &= ~(1<<14);//0x10000; //clear 14 bit
GPIOC->MODER |= (1<<15);//~(0x20000); // set the 15 bit 
//SET push-pull
GPIOC->OTYPER &= ~(1<<7);//~(0x100); //clear 8th bit
//set to low speed
GPIOC->OSPEEDR &= ~(1<<14);//~(0x10000); //clear 12th bit
//set no pull up or down
GPIOC->PUPDR &= ~(1<<14);//~(0x10000); //clear 12th bit
GPIOC->PUPDR |= (1<<15); // Clear the 13th bit

//PIN_C8
//SET as general Alternate Function
GPIOC->MODER &= ~(1<<16);//0x10000; //clear 12 bit
GPIOC->MODER |= (1<<17);//~(0x20000); // set the 13 bit 
//SET push-pull
GPIOC->OTYPER &= ~(1<<8);//~(0x100); //clear 8th bit
//set to low speed
GPIOC->OSPEEDR &= ~(1<<16);//~(0x10000); //clear 12th bit
//set no pull up or down
GPIOC->PUPDR &= ~(1<<16);//~(0x10000); //clear 12th bit
GPIOC->PUPDR &= ~(1<<17); // Clear the 13th bit

//PIN_C9
//SET as general Alternate Function
GPIOC->MODER &= ~(1<<18);//0x10000; //clear 14 bit
GPIOC->MODER |= (1<<19);//~(0x20000); // set the 15 bit 
//SET push-pull
GPIOC->OTYPER &= ~(1<<9);//~(0x100); //clear 8th bit
//set to low speed
GPIOC->OSPEEDR &= ~(1<<18);//~(0x10000); //clear 12th bit
//set no pull up or down
GPIOC->PUPDR &= ~(1<<18);//~(0x10000); //clear 12th bit
GPIOC->PUPDR &= ~(1<<19); // Clear the 13th bit


//PC6
GPIOC->AFR[0] &= ~(1<<27);
GPIOC->AFR[0] &= ~(1<<26);
GPIOC->AFR[0] &= ~(1<<25);
GPIOC->AFR[0] &= ~(1<<24);
//PC7
GPIOC->AFR[0] &= ~(1<<31);
GPIOC->AFR[0] &= ~(1<<30);
GPIOC->AFR[0] &= ~(1<<29);
GPIOC->AFR[0] &= ~(1<<28);
//PC8
GPIOC->AFR[1] &= ~(1<<3);
GPIOC->AFR[1] &= ~(1<<2);
GPIOC->AFR[1] &= ~(1<<1);
GPIOC->AFR[1] &= ~(1<<0);
//PC9
GPIOC->AFR[1] &= ~(1<<7);
GPIOC->AFR[1] &= ~(1<<6);
GPIOC->AFR[1] &= ~(1<<5);
GPIOC->AFR[1] &= ~(1<<4);

//PA0
GPIOA->AFR[0] &= ~(1<<3);
GPIOA->AFR[0] &= ~(1<<2);
GPIOA->AFR[0] |= (1<<1);
GPIOA->AFR[0] &= ~(1<<0);
//PA1
GPIOA->AFR[0] &= ~(1<<7);
GPIOA->AFR[0] &= ~(1<<6);
GPIOA->AFR[0] |= (1<<5);
GPIOA->AFR[0] &= ~(1<<4);
//PA2
GPIOA->AFR[0] &= ~(1<<11);
GPIOA->AFR[0] &= ~(1<<10);
GPIOA->AFR[0] |= (1<<9);
GPIOA->AFR[0] &= ~(1<<8);
//PA3
GPIOA->AFR[0] &= ~(1<<15);
GPIOA->AFR[0] &= ~(1<<14);
GPIOA->AFR[0] |= (1<<13);
GPIOA->AFR[0] &= ~(1<<12);

//PB10
GPIOB->AFR[1] &= ~(1<<11);
GPIOB->AFR[1] &= ~(1<<10);
GPIOB->AFR[1] |= (1<<9);
GPIOB->AFR[1] &= ~(1<<8);

//PB11
GPIOB->AFR[1] &= ~(1<<15);
GPIOB->AFR[1] &= ~(1<<14);
GPIOB->AFR[1] |= (1<<13);
GPIOB->AFR[1] &= ~(1<<12);

}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(uint32_t duty, int ch) {
    if(duty <= 100) {
		switch (ch)
		{
		case 1:
			TIM2->CCR2 = 0;
			TIM2->CCR1 = ((uint32_t)duty*TIM2->ARR)/100;  // Use linear transform to produce CCR1 value
			break;
		case 2:
			TIM2->CCR1 = 0;
			TIM2->CCR2 = ((uint32_t)duty*TIM2->ARR)/100;  // Use linear transform to produce CCR1 value
			break;
		case 3:
			TIM2->CCR4 = 0;
			TIM2->CCR3 = ((uint32_t)duty*TIM2->ARR)/100;  // Use linear transform to produce CCR1 value
			break;
		case 4:
			TIM2->CCR3 = 0;
			TIM2->CCR4 = ((uint32_t)duty*TIM2->ARR)/100;  // Use linear transform to produce CCR1 value
			break;
		case 5:
			TIM3->CCR2 = 0;
			TIM3->CCR1 = ((uint32_t)duty*TIM3->ARR)/100;  // Use linear transform to produce CCR1 value
			break;
		case 6:
			TIM3->CCR1 = 0;
			TIM3->CCR2 = ((uint32_t)duty*TIM3->ARR)/100;  // Use linear transform to produce CCR1 value
			break;
		case 7:
			TIM3->CCR4 = 0;
			TIM3->CCR3 = ((uint32_t)duty*TIM3->ARR)/100;  // Use linear transform to produce CCR1 value
			break;
		case 8:
			TIM3->CCR3 = 0;
			TIM3->CCR4 = ((uint32_t)duty*TIM3->ARR)/100;  // Use linear transform to produce CCR1 value
			break;
		
		default:
			break;
		}
        
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

void stopPWM(int ch) {
	
}