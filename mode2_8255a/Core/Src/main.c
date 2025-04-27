#include "stm32f4xx.h"

// Định nghĩa chân điều khiển
#define CS_PIN         (1U << 0) // PB0
#define RD_PIN         (1U << 1) // PB1
#define WR_PIN         (1U << 2) // PB2
#define A0_PIN         (1U << 3) // PB3
#define A1_PIN         (1U << 4) // PB4

// Hàm nguyên mẫu
void SystemClock_Config(void);
void GPIO_Init(void);
void Write_Control_Word(uint8_t data);
void Write_PortA(uint8_t data);
uint8_t Read_PortA(void);
uint8_t Read_PortC(void);
void Delay_ms(uint32_t ms);

// Hàm chính
int main(void)
{
    // Khởi tạo hệ thống
    SystemClock_Config();
    GPIO_Init();

    // Cấu hình 8255A ở Mode 2 (Port C Upper/Lower là output)
    Write_Control_Word(0xC0);

    uint8_t output_data = 0xAA; // Mẫu dữ liệu kiểm tra (10101010)

    while (1)
    {
        // Ghi dữ liệu ra Port A
        Write_PortA(output_data);
        Delay_ms(1000); // Chờ 1 giây để quan sát LED

        // Kiểm tra tín hiệu STB (PC4)
        uint8_t portC = Read_PortC();
        if (portC & (1 << 4)) // Nếu STB = 1 (dữ liệu sẵn sàng)
        {
            uint8_t input_data = Read_PortA(); // Đọc từ Port A
            Write_PortA(input_data); // Ghi lại dữ liệu để kiểm tra
            Delay_ms(1000); // Chờ để quan sát dữ liệu đọc được
        }

        // Chuyển đổi mẫu dữ liệu
        output_data = (output_data == 0xAA) ? 0x55 : 0xAA;
    }
}

// Cấu hình clock (HSE 8 MHz, SYSCLK 84 MHz)
void SystemClock_Config(void)
{
    // Bật HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)); // Chờ HSE sẵn sàng

    // Cấu hình PLL: PLLM = 8, PLLN = 336, PLLP = /4
    RCC->PLLCFGR = (8 << 0) | (336 << 6) | (0 << 16) | RCC_PLLCFGR_PLLSRC_HSE;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Chờ PLL sẵn sàng

    // Đặt latency flash
    FLASH->ACR = FLASH_ACR_LATENCY_2WS;

    // Chọn PLL làm nguồn SYSCLK
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Chờ SYSCLK chuyển đổi

    // Cấu hình HCLK, PCLK1, PCLK2
    RCC->CFGR &= ~RCC_CFGR_HPRE;  // HCLK = SYSCLK
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // PCLK1 = HCLK/2
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // PCLK2 = HCLK
}

// Cấu hình GPIO
void GPIO_Init(void)
{
    // Bật clock cho GPIOA và GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // Cấu hình PA0-PA7 làm output
    GPIOA->MODER &= ~0xFFFF; // Xóa cấu hình cũ
    GPIOA->MODER |= 0x5555;  // Output mode (01 cho mỗi chân)
    GPIOA->OSPEEDR |= 0x5555; // Tốc độ cao

    // Cấu hình PB0-PB4 làm output
    GPIOB->MODER &= ~0xFF;   // Xóa cấu hình cũ
    GPIOB->MODER |= 0x55;    // Output mode
    GPIOB->OSPEEDR |= 0x55;  // Tốc độ cao
}

// Ghi Control Word vào 8255A
void Write_Control_Word(uint8_t data)
{
    GPIOB->BSRR = CS_PIN << 16;     // CS = 0
    GPIOB->BSRR = A0_PIN | A1_PIN;  // A0 = 1, A1 = 1 (Control Register)
    GPIOB->BSRR = WR_PIN << 16;     // WR = 0

    GPIOA->ODR = (GPIOA->ODR & 0xFF00) | (data & 0xFF); // Ghi D0-D7

    GPIOB->BSRR = WR_PIN;           // WR = 1
    GPIOB->BSRR = CS_PIN;           // CS = 1
}

// Ghi dữ liệu ra Port A
void Write_PortA(uint8_t data)
{
    GPIOB->BSRR = CS_PIN << 16;     // CS = 0
    GPIOB->BSRR = (A0_PIN | A1_PIN) << 16; // A0 = 0, A1 = 0 (Port A)
    GPIOB->BSRR = WR_PIN << 16;     // WR = 0

    GPIOA->ODR = (GPIOA->ODR & 0xFF00) | (data & 0xFF);

    GPIOB->BSRR = WR_PIN;
    GPIOB->BSRR = CS_PIN;
}

// Đọc dữ liệu từ Port A
uint8_t Read_PortA(void)
{
    uint8_t data;

    // Chuyển PA0-PA7 sang input
    GPIOA->MODER &= ~0xFFFF;        // Input mode (00)

    GPIOB->BSRR = CS_PIN << 16;     // CS = 0
    GPIOB->BSRR = (A0_PIN | A1_PIN) << 16; // A0 = 0, A1 = 0
    GPIOB->BSRR = RD_PIN << 16;     // RD = 0

    data = GPIOA->IDR & 0xFF;       // Đọc D0-D7

    GPIOB->BSRR = RD_PIN;           // RD = 1
    GPIOB->BSRR = CS_PIN;           // CS = 1

    // Khôi phục PA0-PA7 làm output
    GPIOA->MODER |= 0x5555;

    return data;
}

// Đọc Port C (kiểm tra tín hiệu bắt tay)
uint8_t Read_PortC(void)
{
    uint8_t data;

    GPIOA->MODER &= ~0xFFFF;        // Input mode

    GPIOB->BSRR = CS_PIN << 16;     // CS = 0
    GPIOB->BSRR = A0_PIN << 16;     // A0 = 0
    GPIOB->BSRR = A1_PIN;           // A1 = 1 (Port C)
    GPIOB->BSRR = RD_PIN << 16;     // RD = 0

    data = GPIOA->IDR & 0xFF;

    GPIOB->BSRR = RD_PIN;
    GPIOB->BSRR = CS_PIN;

    GPIOA->MODER |= 0x5555;         // Khôi phục output

    return data;
}

// Hàm delay đơn giản
void Delay_ms(uint32_t ms)
{
    // SYSCLK = 84 MHz, 1 ms ~ 84000 chu kỳ
    for (uint32_t i = 0; i < ms * 8400; i++)
    {
        __NOP();
    }
}
