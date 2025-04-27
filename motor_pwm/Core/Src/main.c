#include "stm32f4xx.h"

void TIM2_PWM_Init(void);
void GPIO_Init(void);
void EXTI_Init(void);
void Delay_ms(uint32_t ms);

volatile uint16_t pwm_value = 500;  // Mặc định 50% tốc độ
volatile int state = 0;  // Trạng thái điều khiển động cơ

int main(void) {
    GPIO_Init();
    TIM2_PWM_Init();
    EXTI_Init();

    while (1) {
        // Kiểm tra nút nhấn tăng tốc (PA0)
        if (GPIOA->IDR & (1 << 0)) {
            if (pwm_value < 1000) pwm_value += 100;  // Giới hạn max 1000
            TIM2->CCR1 = pwm_value;
            Delay_ms(200);  // Chống dội phím
        }

        if (GPIOA->IDR & (1 << 1)) {
                   if (pwm_value > 100) {
                       pwm_value -= 100;
                   } else {
                       pwm_value = 0; // Nếu pwm < 100, giảm tiếp sẽ dừng hẳn
                   }
                   TIM2->CCR1 = pwm_value;
                   Delay_ms(200);  // Chống dội phím
               }

        // Điều khiển hướng động cơ
        if (state == 0) {
            GPIOA->ODR |= (1 << 7);  // IN1 HIGH
            GPIOA->ODR &= ~(1 << 6); // IN2 LOW
        } else {
            GPIOA->ODR |= (1 << 6);  // IN1 HIGH
            GPIOA->ODR &= ~(1 << 7); // IN2 LOW
        }
    }
}

// ------------------------------
// ⚙ Khởi tạo GPIO
// ------------------------------
void GPIO_Init(void) {
    RCC->AHB1ENR |= (1 << 0); // Bật clock GPIOA
    RCC->AHB1ENR |= (1 << 2); // Bật clock GPIOC

    // PA6, PA7 là output (Điều khiển L298N)
    GPIOA->MODER |= (1 << (6 * 2)) | (1 << (7 * 2));
    GPIOA->MODER &= ~((1 << (6 * 2 + 1)) | (1 << (7 * 2 + 1)));

    GPIOA->OTYPER &= ~((1 << 6) | (1 << 7)); // Push-Pull
    GPIOA->OSPEEDR |= (3 << (6 * 2)) | (3 << (7 * 2)); // High Speed

    // PA0, PA1 là input (Nút nhấn)
    GPIOA->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)));

    // Kéo xuống GND (Pull-down)
    GPIOA->PUPDR |= (2 << (0 * 2)) | (2 << (1 * 2));

    // PC13 là input với pull-up
    GPIOC->MODER &= ~(3 << (13 * 2));  // Chế độ input (00)
    GPIOC->PUPDR &= ~(3 << (13 * 2));  // Xóa cấu hình cũ
    GPIOC->PUPDR |= (1 << (13 * 2));   // Kéo lên (Pull-up)
}

// ------------------------------
// ⚙ Cấu hình EXTI (Ngắt ngoài) cho PC13
// ------------------------------
void EXTI_Init(void) {
    RCC->APB2ENR |= (1 << 14);  // Bật clock SYSCFG

    SYSCFG->EXTICR[3] |= (2 << 4);  // EXTI13 kết nối với PC13

    EXTI->IMR |= (1 << 13);   // Bật ngắt EXTI13
    EXTI->FTSR |= (1 << 13);  // Kích hoạt ngắt cạnh xuống (Falling edge)

    NVIC_EnableIRQ(EXTI15_10_IRQn);  // Bật ngắt trong NVIC
    NVIC_SetPriority(EXTI15_10_IRQn, 2);  // Đặt mức ưu tiên
}

// ------------------------------
// ⚙ Trình xử lý ngắt EXTI (Ngắt PC13)
// ------------------------------
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1 << 13)) {  // Kiểm tra nếu PC13 gây ngắt
        state = !state;  // Đổi trạng thái
        EXTI->PR |= (1 << 13);  // Xóa cờ ngắt
    }
}

// ------------------------------
// ⚙ Cấu hình TIM2 PWM trên PA5
// ------------------------------
void TIM2_PWM_Init(void) {
    RCC->APB1ENR |= (1 << 0); // Bật clock TIM2
    RCC->AHB1ENR |= (1 << 0); // Bật clock GPIOA

    // Cấu hình PA5 là AF1 (TIM2_CH1)
    GPIOA->MODER |= (2 << (5 * 2));
    GPIOA->AFR[0] |= (1 << (5 * 4)); // AF1 cho PA5

    TIM2->PSC = 84 - 1;    // Chia tần số xuống 1MHz
    TIM2->ARR = 1000 - 1;  // PWM tần số 1kHz
    TIM2->CCR1 = pwm_value; // Mặc định duty cycle = 50%

    TIM2->CCMR1 |= (6 << 4); // Chọn PWM mode 1 cho CH1
    TIM2->CCER |= (1 << 0);  // Bật CH1 output
    TIM2->CR1 |= (1 << 0);   // Bật TIM2
}

// ------------------------------
// ⚙ Delay chống dội phím
// ------------------------------
void Delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) {
        __NOP();
    }
}
