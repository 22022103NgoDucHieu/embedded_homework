//master
#include "stm32f4xx.h"

void SystemClock_Config(void);
void SPI1_Init(void);
void GPIO_Init(void);
uint8_t SPI1_TransmitReceive(uint8_t data);

int main(void) {
    SystemClock_Config();
    GPIO_Init();
    SPI1_Init();

    uint8_t tx_data = 0xAA; // Dữ liệu gửi
    uint8_t rx_data = 0;    // Dữ liệu nhận
    uint8_t done = 0;       // Cờ hoàn tất

    while (1) {
        if (!done) {
            // Nháy LED khi chưa nhận được phản hồi
            GPIOA->ODR |= (1 << 8);  // Bật LED
            for (volatile int i = 0; i < 50000; i++);
            GPIOA->ODR &= ~(1 << 8); // Tắt LED
            for (volatile int i = 0; i < 50000; i++);

            GPIOA->ODR &= ~(1 << 4); // Kích hoạt NSS
            rx_data = SPI1_TransmitReceive(tx_data); // Gửi và nhận
            GPIOA->ODR |= (1 << 4);  // Tắt NSS

            if (rx_data == 0xBB) {
                done = 1; // Đánh dấu hoàn tất
                GPIOA->ODR &= ~(1 << 8); // Tắt LED vĩnh viễn
            }
        }
        // Khi done = 1, không làm gì nữa
    }
}

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3 << (4 * 2) | 3 << (8 * 2));
    GPIOA->MODER |= (1 << (4 * 2) | 1 << (8 * 2));
    GPIOA->MODER &= ~(3 << (5 * 2) | 3 << (6 * 2) | 3 << (7 * 2));
    GPIOA->MODER |= (2 << (5 * 2) | 2 << (6 * 2) | 2 << (7 * 2));
    GPIOA->AFR[0] &= ~(0xF << (5 * 4) | 0xF << (6 * 4) | 0xF << (7 * 4));
    GPIOA->AFR[0] |= (5 << (5 * 4) | 5 << (6 * 4) | 5 << (7 * 4));
}

void SPI1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 = 0;
    SPI1->CR1 |= (1 << 2);  // Master mode
    SPI1->CR1 |= (3 << 3);  // Baud rate = fPCLK/16
    SPI1->CR1 |= (1 << 9);  // Software NSS
    SPI1->CR1 |= (1 << 8);  // SSI = 1
    SPI1->CR1 |= (1 << 6);  // Kích hoạt SPI
}

uint8_t SPI1_TransmitReceive(uint8_t data) {
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_TXE));
    while (!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
}

void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CFGR = 0;
}


//slave
//#include "stm32f4xx.h"
//
//void SystemClock_Config(void);
//void SPI1_Init(void);
//void GPIO_Init(void);
//
//int main(void) {
//    SystemClock_Config();
//    GPIO_Init();
//    SPI1_Init();
//
//    uint8_t received_data = 0;
//
//    while (1) {
//        GPIOB->ODR |= (1 << 0); // LED sáng khi đợi
//
//        while (!(SPI1->SR & SPI_SR_RXNE));
//        received_data = SPI1->DR;
//
//        if (received_data == 0xAA) {
//            for (int i = 0; i < 5; i++) {
//                GPIOB->ODR &= ~(1 << 0);
//                for (volatile int j = 0; j < 50000; j++);
//                GPIOB->ODR |= (1 << 0);
//                for (volatile int j = 0; j < 50000; j++);
//            }
//            SPI1->DR = 0xBB;
//            while (SPI1->SR & SPI_SR_BSY);
//        }
//    }
//}
//
//void GPIO_Init(void) {
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
//    GPIOB->MODER &= ~(3 << (0 * 2));
//    GPIOB->MODER |= (1 << (0 * 2));
//    GPIOA->MODER &= ~(3 << (4 * 2) | 3 << (5 * 2) | 3 << (6 * 2) | 3 << (7 * 2));
//    GPIOA->MODER |= (2 << (4 * 2) | 2 << (5 * 2) | 2 << (6 * 2) | 2 << (7 * 2));
//    GPIOA->AFR[0] &= ~(0xF << (4 * 4) | 0xF << (5 * 4) | 0xF << (6 * 4) | 0xF << (7 * 4));
//    GPIOA->AFR[0] |= (5 << (4 * 4) | 5 << (5 * 4) | 5 << (6 * 4) | 5 << (7 * 4));
//}
//
//void SPI1_Init(void) {
//    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
//    SPI1->CR1 = 0;
//    SPI1->CR1 &= ~(1 << 2); // Slave mode
//    SPI1->CR1 |= (1 << 6);  // Kích hoạt SPI
//}
//
//void SystemClock_Config(void) {
//    RCC->CR |= RCC_CR_HSION;
//    while (!(RCC->CR & RCC_CR_HSIRDY));
//    RCC->CFGR = 0;
//}
