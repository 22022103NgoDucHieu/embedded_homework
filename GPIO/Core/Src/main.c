//#include "stm32f4xx.h"
//#include <stdint.h>
//
//int main(void)
//{
//    // Bật clock cho GPIOA và GPIOB
//    RCC->AHB1ENR |= (1 << 0); // Bật clock cho GPIOA
//    RCC->AHB1ENR |= (1 << 1); // Bật clock cho GPIOB
//
//    // Cấu hình input: PB10, PB4, PB
//    GPIOB->MODER &= ~(3 << (10*2)); // Cấu hình PB10 làm input
//    GPIOB->MODER &= ~(3 << (4*2));  // Cấu hình PB4 làm input
//    GPIOB->MODER &= ~(3 << (7*2));  // Cấu hình PB7 làm input
//
//    // Cấu hình output: PA0, PA1, PA4
//    GPIOA->MODER &= ~(3 << (0*2));
//    GPIOA->MODER |= (1 << (0*2));  // Đặt PA0 làm output
//    GPIOA->MODER &= ~(3 << (1*2));
//    GPIOA->MODER |= (1 << (1*2));  // Đặt PA1 làm output
//    GPIOA->MODER &= ~(3 << (4*2));
//    GPIOA->MODER |= (1 << (4*2));  // Đặt PA4 làm output
//
//    uint32_t button1, button2, button3; // Khai báo 3 biến 32-bit để lưu trạng thái của các nút bấm (PB10, PB4, PB7)
//
//    while (1)
//    {
//        button1 = (GPIOB->IDR & (1 << 10)) >> 10;
//        // Đọc trạng thái PB10: Lấy bit 10 từ IDR của PORT B, dịch phải 10 lần để đưa về 0 hoặc 1
//        button2 = (GPIOB->IDR & (1 << 4)) >> 4;
//        // Đọc trạng thái PB4: Lấy bit 4 từ IDR của PORT B, dịch phải 4 lần để đưa về 0 hoặc 1
//        button3 = (GPIOB->IDR & (1 << 7)) >> 7;
//        // Đọc trạng thái PB7: Lấy bit 7 từ IDR của PORT B, dịch phải 7 lần để đưa về 0 hoặc 1
//
//        // Điều khiển LED dựa trên trạng thái nút
//        if (button1 == 0) {   // Nếu PB10 = 0 (nút được nhấn, vì có pull-up bên ngoài nối xuống GND)
//            GPIOA->ODR |= (1 << 0);  // Bật LED trên PA0
//        } else {   // Nếu PB10 = 1 (nút không nhấn, mức cao do pull-up)
//            GPIOA->ODR &= ~(1 << 0); // Tắt LED trên PA0
//        }
//
//        if (button2 == 0) {      //PB4 điều khiển PA1
//            GPIOA->ODR |= (1 << 1);
//        } else {
//            GPIOA->ODR &= ~(1 << 1);
//        }
//
//        if (button3 == 0) {       //PB7 điều khiển PA4
//            GPIOA->ODR |= (1 << 4);
//        } else {
//            GPIOA->ODR &= ~(1 << 4);
//        }
//    }
//}


//#include <stm32f4xx_hal.h>
//#include <stdint.h>
//
//#define button_pressed 		0
//
//void init_led() {
//	RCC->AHB1ENR |= 1 << 0;
//	GPIOA->MODER |= 1 << 2;
//	GPIOA->MODER &= ~(1 << 3);
//	GPIOA->MODER |= 1 << 8;
//	GPIOA->MODER &= ~(1 << 9);
//	GPIOA->MODER |= 1 << 10;
//	GPIOA->MODER &= ~(1 << 11);
//}
//
//void init_button() {
//	RCC->AHB1ENR |= 1 << 2;
//	GPIOC->MODER &= ~(1 << 2);
//	GPIOC->MODER &= ~(1 << 3);
//	GPIOC->MODER &= ~(1 << 8);
//	GPIOC->MODER &= ~(1 << 9);
//	GPIOC->MODER &= ~(1 << 26);
//	GPIOC->MODER &= ~(1 << 27);
//}
//
//int main(void) {
//	init_led();
//	init_button();
//	uint32_t read_status1, read_status2, read_status3;
//	while (1) {
//			read_status1 = GPIOC->IDR & (1 << 1);
//			read_status2 = GPIOC->IDR & (1 << 4);
//			read_status3 = GPIOC->IDR & (1 << 13);
//			if (read_status1 == button_pressed) {
//				GPIOA->ODR |= 1 << 1;
//			} else {
//				GPIOA->ODR &= ~(1 << 1);
//			}
//			if (read_status2 == button_pressed) {
//				GPIOA->ODR |= 1 << 4;
//			} else {
//				GPIOA->ODR &= ~(1 << 4);
//			}
//			if (read_status3 == button_pressed) {
//				GPIOA->ODR |= 1 << 5;
//			} else {
//				GPIOA->ODR &= ~(1 << 5);
//			}
//		}
//}




//#include "stm32f4xx.h"
//#include <stdint.h>
//#include <main.h>
//// Hàm cấu hình hệ thống clock (84 MHz với HSI)
//void SystemClock_Config(void) {
//    // Bật HSI (16 MHz)
//    RCC->CR |= RCC_CR_HSION;
//    while (!(RCC->CR & RCC_CR_HSIRDY));
//
//    // Cấu hình PLL: 84 MHz (HSI * 336 / 8 / 2)
//    RCC->PLLCFGR = (336 << 6) | (8 << 24) | RCC_PLLCFGR_PLLSRC_HSI;
//    RCC->CR |= RCC_CR_PLLON;
//    while (!(RCC->CR & RCC_CR_PLLRDY));
//
//    // Chọn PLL làm nguồn clock hệ thống
//    RCC->CFGR |= RCC_CFGR_SW_PLL;
//    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
//
//    // Cấu hình bus: AHB = 84 MHz, APB1 = 42 MHz, APB2 = 84 MHz
//    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
//}
//
//// Hàm cấu hình GPIO
//void GPIO_Init(void) {
//    // Bật clock cho GPIOA
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
//
//    // PA0: Chân Analog cho ADC
//    GPIOA->MODER |= (3 << 0); // Chế độ Analog (11)
//
//    // PA8: Chân PWM (TIM1_CH1) - Alternate Function
//    GPIOA->MODER |= (2 << 16); // Chế độ AF (10)
//    GPIOA->AFR[1] |= (1 << 0); // AF1 (TIM1) cho PA8
//}
//
//// Hàm cấu hình ADC1 (PA0)
//void ADC1_Init(void) {
//    // Bật clock cho ADC1
//    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//
//    // Cấu hình ADC: 12-bit, single conversion
//    ADC1->CR1 = 0;                  // Độ phân giải 12-bit
//    ADC1->CR2 = ADC_CR2_ADON;       // Bật ADC
//    ADC1->SMPR2 = (3 << 0);         // Sampling time: 56 cycles cho kênh 0
//    ADC1->SQR3 = 0;                 // Chọn kênh 0 (PA0)
//}
//
//// Hàm đọc giá trị ADC
//uint16_t ADC1_Read(void) {
//    ADC1->CR2 |= ADC_CR2_SWSTART;   // Bắt đầu chuyển đổi
//    while (!(ADC1->SR & ADC_SR_EOC)); // Đợi hoàn tất chuyển đổi
//    return ADC1->DR;                // Trả về giá trị ADC (0-4095)
//}
//
//// Hàm cấu hình PWM (TIM1_CH1 trên PA8)
//void TIM1_PWM_Init(void) {
//    // Bật clock cho TIM1
//    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
//
//    // Cấu hình Timer: f = 1 kHz, PWM mode
//    TIM1->PSC = 83;                 // Prescaler: 84MHz / (83+1) = 1MHz
//    TIM1->ARR = 1000;               // Auto-reload: 1MHz / 1000 = 1kHz
//    TIM1->CCMR1 |= (6 << 4);        // PWM mode 1 (110)
//    TIM1->CCER |= TIM_CCER_CC1E;    // Bật kênh 1
//    TIM1->BDTR |= TIM_BDTR_MOE;     // Bật đầu ra chính (Main Output Enable)
//    TIM1->CR1 |= TIM_CR1_CEN;       // Bật Timer
//}
//
//// Hàm cập nhật PWM duty cycle
//void Set_PWM_Duty(uint16_t duty) {
//    if (duty > 1000) duty = 1000;   // Giới hạn duty cycle
//    if (duty < 0) duty = 0;         // Giới hạn dưới
//    TIM1->CCR1 = duty;              // Cập nhật giá trị PWM
//}
//
//// Hàm delay
//void Delay(uint32_t count) {
//    for (volatile uint32_t i = 0; i < count; i++);
//}
//
//int main(void) {
//    uint16_t adc_value = 0;
//    uint16_t pwm_value = 0;
//    uint16_t pwm_value1 = 0;
//
//    // Khởi tạo hệ thống
//    SystemClock_Config();
//    GPIO_Init();
//    ADC1_Init();
//    TIM1_PWM_Init();
//
//    while (1) {
//        // Đọc giá trị từ ADC
//        adc_value = ADC1_Read();
//
//        // Ánh xạ giá trị ADC (0-4095) sang PWM (0-1000)
//        // ADC thấp (tối) -> PWM cao (LED sáng max)
//        // ADC cao (sáng) -> PWM thấp (LED mờ min)
//        pwm_value1 = 1000- (4095 - adc_value) * 1000 / 4095;
//        if(pwm_value1 < 100) pwm_value = 0;
//        else pwm_value = pwm_value1;
//        // Cập nhật PWM
//        Set_PWM_Duty(pwm_value);
//
//        // Delay khoảng 100ms
//        Delay(100000);
//    }
//}


#include "stm32f4xx.h"

// Hàm cấu hình clock hệ thống
void SystemClock_Config(void);

int main(void)
{
    // Cấu hình clock hệ thống
    SystemClock_Config();

    // 1. Bật clock cho GPIOA và TIM2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // Bật clock GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;     // Bật clock TIM2

    // 2. Cấu hình PA0 làm alternate function (AF1 cho TIM2_CH1)
    GPIOA->MODER &= ~(GPIO_MODER_MODER0);   // Xóa cấu hình cũ
    GPIOA->MODER |= GPIO_MODER_MODER0_1;    // Chọn chế độ AF
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL0);    // Xóa AF cũ
    GPIOA->AFR[0] |= (1 << 0);             // Chọn AF1 (TIM2)

    // 3. Cấu hình Timer 2 cho PWM
    TIM2->CR1 = 0;                          // Reset thanh ghi điều khiển
    TIM2->PSC = 83;                        // Prescaler = 83+1 -> 84MHz/84 = 1MHz
    TIM2->ARR = 499;                       // Auto-reload = 999+1 -> 1kHz PWM
    TIM2->CCMR1 |= (6 << 4);               // PWM mode 1 cho CH1
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;        // Bật preload cho CH1
    TIM2->CCER |= TIM_CCER_CC1E;           // Bật output CH1
    TIM2->EGR |= TIM_EGR_UG;               // Update generation
    TIM2->CCR1 = 500;                      // Duty cycle ban đầu 50%
    TIM2->CR1 |= TIM_CR1_CEN;              // Bật Timer

    while (1)
    {
        // Thay đổi duty cycle để tạo âm thanh
        TIM2->CCR1 = 800;  // 50% duty cycle
        for(volatile uint32_t i = 0; i < 500000; i++);  // Delay đơn giản
        TIM2->CCR1 = 100;  // 25% duty cycle
        for(volatile uint32_t i = 0; i < 500000; i++);  // Delay đơn giản
    }
}

void SystemClock_Config(void)
{
    // Bật HSE
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    // Cấu hình Flash
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |
                 FLASH_ACR_DCEN | FLASH_ACR_LATENCY_2WS;

    // Cấu hình PLL (84MHz)
    RCC->PLLCFGR = (8 << 0) |      // PLLM = 8
                   (336 << 6) |    // PLLN = 336
                   (1 << 16) |     // PLLP = 4 (div 4)
                   RCC_PLLCFGR_PLLSRC_HSE;

    // Bật PLL
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    // Chọn PLL làm clock hệ thống
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // Cấu hình bus
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;   // AHB prescaler = 1
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 prescaler = 2
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;  // APB2 prescaler = 1
}
