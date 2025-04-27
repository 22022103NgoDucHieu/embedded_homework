#include "stm32f4xx.h"

// Định nghĩa chân GPIO
#define INTR_PIN    GPIO_IDR_IDR_0  // PC0: INTR từ PC3 của 8255A
#define IBF_PIN     GPIO_IDR_IDR_1  // PC1: IBF từ PC5 của 8255A

// Hàm Timer
void TIM2_Init(void);
void timer_delay_us(uint32_t us);
void timer_delay_ms(uint32_t ms);

// Hàm khởi tạo và điều khiển
void GPIO_Init(void);
void init_8255(void);
void write_8255_control(uint8_t value);
uint8_t read_portA(void);

int main(void)
{
    // Bật clock cho GPIOA, GPIOB, GPIOC
    RCC->AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2); // GPIOA, GPIOB, GPIOC

    // Khởi tạo GPIO
    GPIO_Init();

    // Khởi tạo TIM2
    TIM2_Init();

    // Cấu hình 8255A ở chế độ 1, cổng A làm đầu vào
    init_8255();
    timer_delay_ms(3000); // Chậm 3s sau khi khởi tạo 8255A

    while (1)
    {
        // Kiểm tra IBF (PC1)
        if (GPIOC->IDR & IBF_PIN) // IBF = 1: Buffer đầy
        {
            timer_delay_ms(3000); // Chậm 3s trước khi đọc

            // Đọc dữ liệu từ cổng A
            uint8_t data = read_portA();
            timer_delay_ms(3000); // Chậm 3s sau khi đọc

            // Không xuất ra LED, chỉ giữ dữ liệu trong biến 'data'
        }
    }
}

// Khởi tạo GPIO
void GPIO_Init(void)
{
    // PA0-PA7: Output (D0-D7, dùng để giao tiếp với 8255A)
    GPIOA->MODER &= ~(0xFFFF); // Xóa bit 0-15
    GPIOA->MODER |= (0x5555);  // Output (01) cho PA0-PA7
    GPIOA->OSPEEDR &= ~(0xFFFF); // Tốc độ thấp
    GPIOA->ODR &= ~(0xFF);       // Ban đầu đặt thấp

    // PB0 (A0), PB1 (A1), PB2 (RD), PB3 (WR): Output
    GPIOB->MODER &= ~(0xFF); // Xóa bit 0-7
    GPIOB->MODER |= (0x55);  // Output (01) cho PB0-PB3
    GPIOB->OSPEEDR &= ~(0xFF); // Tốc độ thấp
    GPIOB->ODR &= ~( (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) ); // Ban đầu đặt thấp

    // PC0 (INTR), PC1 (IBF): Input
    GPIOC->MODER &= ~( (3 << 0) | (3 << 2) ); // Input (00) cho PC0, PC1
}

// Khởi tạo TIM2
void TIM2_Init(void)
{
    // Bật clock cho TIM2
    RCC->APB1ENR |= (1 << 0); // TIM2EN

    // Cấu hình TIM2: 16MHz, đếm 1us
    TIM2->PSC = 16 - 1; // Prescaler = 16 → 16MHz / 16 = 1MHz (1us/tick)
    TIM2->ARR = 0xFFFF; // Auto-reload tối đa
    TIM2->CR1 = (1 << 0); // CEN: Bật Timer
}

// Độ trễ micro giây
void timer_delay_us(uint32_t us)
{
    TIM2->CNT = 0; // Reset bộ đếm
    while (TIM2->CNT < us); // Chờ đến khi đếm đủ micro giây
}

// Độ trễ mili giây
void timer_delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        timer_delay_us(1000); // 1000us = 1ms
    }
}

// Ghi giá trị vào thanh ghi điều khiển
void write_8255_control(uint8_t value)
{
    // Chọn thanh ghi điều khiển: A1 = 1, A0 = 0
    GPIOB->ODR |= (1 << 1);  // PB1 = 1 (A1)
    GPIOB->ODR &= ~(1 << 0); // PB0 = 0 (A0)

    // Đặt dữ liệu lên bus D0-D7 (PA0-PA7)
    GPIOA->ODR = (GPIOA->ODR & 0xFF00) | (value & 0xFF);

    // Kích hoạt WR (active low)
    GPIOB->ODR &= ~(1 << 3); // PB3 = 0 (WR)
    timer_delay_us(10);      // Độ trễ 10us cho timing
    GPIOB->ODR |= (1 << 3);  // PB3 = 1 (WR)
}

// Khởi tạo 8255A
void init_8255(void)
{
    uint8_t control_word = (1 << 7) | (1 << 5) | (1 << 4); // Mode 1, Port A Input
    write_8255_control(control_word);
}

// Đọc dữ liệu từ cổng A
uint8_t read_portA(void)
{
    uint8_t data;

    // Chuyển PA0-PA7 thành Input
    GPIOA->MODER &= ~(0xFFFF); // Input (00) cho PA0-PA7

    // Chọn cổng A: A1 = 0, A0 = 0
    GPIOB->ODR &= ~(1 << 1); // PB1 = 0 (A1)
    GPIOB->ODR &= ~(1 << 0); // PB0 = 0 (A0)

    // Kích hoạt RD (active low)
    GPIOB->ODR &= ~(1 << 2); // PB2 = 0 (RD)
    timer_delay_us(10);      // Độ trễ 10us cho timing
    data = GPIOA->IDR & 0xFF; // Đọc dữ liệu từ PA0-PA7
    GPIOB->ODR |= (1 << 2);  // PB2 = 1 (RD)

    // Chuyển PA0-PA7 về Output
    GPIOA->MODER |= (0x5555); // Output (01) cho PA0-PA7

    return data;
}
