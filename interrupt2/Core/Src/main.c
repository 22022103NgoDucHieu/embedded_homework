//#include "stm32f4xx.h"
//#include <stdint.h>
//
//volatile uint8_t interrupt_flag = 0;   // Biến cờ để đánh dấu khi có ngắt từ PC13
//volatile uint32_t led_green_time = 0;  // Biến đếm thời gian sáng của LED xanh (PA5)
//
//void init_GPIO(void) {                // Hàm khởi tạo GPIO cho các chân
//    RCC->AHB1ENR |= (1 << 0);         // Kích hoạt clock cho PORTA: Đặt bit 0 trong AHB1ENR lên 1 để cấp nguồn cho GPIOA
//    RCC->AHB1ENR |= (1 << 2);         // Kích hoạt clock cho PORTC: Đặt bit 2 trong AHB1ENR lên 1 để cấp nguồn cho GPIOC
//
//    GPIOA->MODER |= (1 << (7 * 2));   // Thiết lập chân PA7 ở chế độ output
//    GPIOA->MODER &= ~(1 << (7 * 2 + 1));
//
//    GPIOA->MODER |= (1 << (5 * 2));   // Thiết lập chân PA5 ở chế độ output
//    GPIOA->MODER &= ~(1 << (5 * 2 + 1));
//
//    GPIOC->MODER &= ~(1 << (13 * 2)); // Thiết lập chân PC13 ở chế độ input
//    GPIOC->MODER &= ~(1 << (13 * 2 + 1)); // (nút có pull-up sẵn trên board)
//}
//
//void init_Interrupt(void) {           // Hàm cấu hình ngắt ngoài (EXTI) cho PC13
//    RCC->APB2ENR |= (1 << 14);        // Bật clock cho SYSCFG: Đặt bit 14 trong APB2ENR lên 1 để kích hoạt hệ thống cấu hình
//
//    SYSCFG->EXTICR[3] |= (2 << 4);    // Kết nối EXTI13 với PC13: Đặt bit 4-7 trong EXTICR[3] thành 0010 để chọn PORTC cho EXTI13
//
//    EXTI->IMR |= (1 << 13);           // Bật ngắt EXTI13: Đặt bit 13 trong IMR lên 1 để cho phép ngắt từ EXTI13
//    EXTI->FTSR |= (1 << 13);          // Kích hoạt ngắt cạnh xuống: Đặt bit 13 trong FTSR lên 1 để phát hiện cạnh xuống (nhấn nút)
//
//    NVIC_EnableIRQ(EXTI15_10_IRQn);   // Bật ngắt trong NVIC: Kích hoạt ngắt EXTI15_10 trong bộ điều khiển ngắt
//    NVIC_SetPriority(EXTI15_10_IRQn, 2); // Đặt mức ưu tiên cho ngắt: Gán mức ưu tiên 2 cho EXTI15_10_IRQn
//}
//
//void init_TIM2(void) {                // Hàm cấu hình Timer 2
//    RCC->APB1ENR |= (1 << 0);         // Kích hoạt clock cho TIM2
//
//    TIM2->PSC = 8399;     // Thiết lập Prescaler: Chia tần số clock (84 MHz) xuống 10 kHz (84 MHz / (8399 + 1) = 10 kHz)
//    TIM2->ARR = 999;      // Thiết lập Auto-Reload Register: Đặt giá trị tối đa là 999, tạo chu kỳ 100 ms (10 kHz / 1000 = 10 Hz)
//
//    TIM2->DIER |= (1 << 0);   // Bật ngắt khi Timer cập nhật: Đặt bit 0 trong DIER lên 1 để kích hoạt ngắt khi đếm đến ARR
//    TIM2->CR1 |= (1 << 0);    // Bật Timer 2: Đặt bit 0 trong CR1 lên 1 để bắt đầu đếm
//
//    NVIC_EnableIRQ(TIM2_IRQn);        // Bật ngắt TIM2 trong NVIC: Kích hoạt ngắt Timer 2
//    NVIC_SetPriority(TIM2_IRQn, 1);   // Đặt mức ưu tiên cho ngắt TIM2: Gán mức ưu tiên 1 (cao hơn EXTI để nhấp nháy LED đỏ ổn định)
//}
//
//void TIM2_IRQHandler(void) {          // Hàm xử lý ngắt của Timer 2
//    if (TIM2->SR & (1 << 0)) {        // Kiểm tra nếu Timer 2 gây ngắt: Kiểm tra bit 0 trong SR (Status Register) có được đặt không
//        static uint8_t toggle = 0;    // Biến tĩnh để lưu trạng thái nhấp nháy của LED đỏ
//        toggle = !toggle;             // Đảo trạng thái: Chuyển từ 0 sang 1 hoặc 1 sang 0 mỗi 100 ms
//        if (toggle) {                 // Nếu toggle = 1: Bật LED đỏ
//            GPIOA->ODR |= (1 << 7);   // Bật LED đỏ trên PA7: Đặt bit 7 trong ODR lên 1
//        } else {                      // Nếu toggle = 0: Tắt LED đỏ
//            GPIOA->ODR &= ~(1 << 7);  // Tắt LED đỏ trên PA7: Xóa bit 7 trong ODR về 0
//        }
//
//        if (interrupt_flag && led_green_time > 0) { // Kiểm tra nếu có ngắt và còn thời gian sáng LED xanh
//            led_green_time--;         // Giảm biến đếm thời gian: Giảm led_green_time mỗi 100 ms
//            if (led_green_time == 0) { // Nếu hết thời gian: Kiểm tra khi led_green_time về 0
//                GPIOA->ODR &= ~(1 << 5); // Tắt LED xanh trên PA5: Xóa bit 5 trong ODR về 0
//                interrupt_flag = 0;   // Xóa cờ ngắt: Đặt interrupt_flag về 0 để trở lại trạng thái ban đầu
//            }
//        }
//
//        TIM2->SR &= ~(1 << 0);        // Xóa cờ ngắt của Timer 2: Xóa bit 0 trong SR để tránh lặp lại ngắt
//    }
//}
//
//void EXTI15_10_IRQHandler(void) {     // Hàm xử lý ngắt cho các đường EXTI từ 10 đến 15
//    if (EXTI->PR & (1 << 13)) {       // Kiểm tra nếu PC13 gây ngắt: Kiểm tra bit 13 trong PR có được đặt lên 1 không
//        interrupt_flag = 1;           // Đặt cờ ngắt: Gán giá trị 1 cho interrupt_flag để báo hiệu có ngắt xảy ra
//        led_green_time = 5;           // Đặt thời gian sáng cho LED xanh: Gán 5 (5 x 100 ms = 0.5 giây)
//        GPIOA->ODR |= (1 << 5);       // Bật LED xanh trên PA5: Đặt bit 5 trong ODR lên 1 để xuất mức cao
//        EXTI->PR |= (1 << 13);        // Xóa cờ ngắt: Đặt bit 13 trong PR lên 1 để xóa trạng thái ngắt
//    }
//}
//
//int main(void) {                      // Hàm chính của chương trình
//    init_GPIO();                      // Gọi hàm khởi tạo GPIO: Thiết lập các chân PA7, PA5, PC13
//    init_Interrupt();                 // Gọi hàm cấu hình ngắt: Cấu hình EXTI13 cho PC13
//    init_TIM2();                      // Gọi hàm cấu hình Timer 2: Thiết lập TIM2 để điều khiển thời gian
//
//    while (1) {                       // Vòng lặp vô hạn để chương trình chạy liên tục
//    }
//}

#include "stm32f4xx.h"
#include <stdint.h>

volatile uint8_t exti0_flag = 0;       // Biến cờ cho ngắt từ PC0 (LED xanh)
volatile uint8_t exti1_flag = 0;       // Biến cờ cho ngắt từ PA0 (LED vàng)
volatile uint32_t led_green_time = 0;  // Thời gian sáng của LED xanh (PB0)
volatile uint32_t led_yellow_time = 0; // Thời gian sáng của LED vàng (PA6)

void init_GPIO(void) {
    RCC->AHB1ENR |= (1 << 0);         // Clock cho PORTA
    RCC->AHB1ENR |= (1 << 1);         // Clock cho PORTB
    RCC->AHB1ENR |= (1 << 2);         // Clock cho PORTC

    GPIOA->MODER |= (1 << (7 * 2));   // PA7 output (LED đỏ)
    GPIOA->MODER &= ~(1 << (7 * 2 + 1));
    GPIOB->MODER |= (1 << (0 * 2));   // PB0 output (LED xanh)
    GPIOB->MODER &= ~(1 << (0 * 2 + 1));
    GPIOA->MODER |= (1 << (6 * 2));   // PA6 output (LED vàng)
    GPIOA->MODER &= ~(1 << (6 * 2 + 1));

    GPIOC->MODER &= ~(1 << (7 * 2));  // PC7 input (nút)
    GPIOC->MODER &= ~(1 << (7 * 2 + 1));
    GPIOA->MODER &= ~(1 << (0 * 2));  // PA0 input (nút)
    GPIOA->MODER &= ~(1 << (0 * 2 + 1));
}

void init_Interrupt(void) {
    RCC->APB2ENR |= (1 << 14);        // Bật clock SYSCFG

    SYSCFG->EXTICR[1] |= (2 << 12);    // EXTI7 gắn với PC7 (PORTC)
    SYSCFG->EXTICR[0] |= (0 << 4);    // EXTI0 gắn với PA0 (PORTA)

    EXTI->IMR |= (1 << 0);            // Bật ngắt EXTI0
    EXTI->FTSR |= (1 << 0);           // Ngắt cạnh xuống EXTI0
    EXTI->IMR |= (1 << 7);            // Bật ngắt EXTI1
    EXTI->FTSR |= (1 << 7);           // Ngắt cạnh xuống EXTI1


    NVIC_EnableIRQ(EXTI0_IRQn);       // Bật EXTI0 trong NVIC
    NVIC_SetPriority(EXTI0_IRQn, 1);  // Ưu tiên 1
    NVIC_EnableIRQ(EXTI9_5_IRQn);       // Bật EXTI7 trong NVIC
    NVIC_SetPriority(EXTI9_5_IRQn, 2);  // Ưu tiên 2
}

void init_TIM2(void) {
    RCC->APB1ENR |= (1 << 0);         // Clock cho TIM2
    TIM2->PSC = 8399;                 // Prescaler: 10 kHz
    TIM2->ARR = 999;                  // Chu kỳ 100 ms
    TIM2->DIER |= (1 << 0);           // Bật ngắt Timer
    TIM2->CR1 |= (1 << 0);            // Bật Timer
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 0);   // Ưu tiên 1
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & (1 << 0)) {
        static uint8_t toggle = 0;
        toggle = !toggle;
        if (toggle) {
            GPIOA->ODR |= (1 << 7);   // Bật LED đỏ PA7
        } else {
            GPIOA->ODR &= ~(1 << 7);  // Tắt LED đỏ PA7
        }
        if (exti0_flag && led_green_time > 0) {
            led_green_time--;
            if (led_green_time == 0) {
                GPIOB->ODR &= ~(1 << 0); // Tắt LED xanh PB0
                exti0_flag = 0;
            }
        }
        if (exti1_flag && led_yellow_time > 0) {
            led_yellow_time--;
            if (led_yellow_time == 0) {
                GPIOA->ODR &= ~(1 << 6); // Tắt LED vàng PA6
                exti1_flag = 0;
            }
        }
        TIM2->SR &= ~(1 << 0); // Xóa cờ ngắt Timer
    }
}

void EXTI0_IRQHandler(void) {         // Ngắt EXTI0 cho PA0 (LED xanh)
    if (EXTI->PR & (1 << 0)) {        // Kiểm tra nếu PA0 gây ngắt
        if (!exti0_flag) {            // Chỉ bật nếu LED xanh chưa sáng
            exti0_flag = 1;
            led_green_time = 5;       // 0.5 giây
            GPIOB->ODR |= (1 << 0);   // Bật LED xanh PB0
        }
        EXTI->PR |= (1 << 0);         // Xóa cờ ngắt
    }
}

void EXTI9_5_IRQHandler(void) {         // Ngắt EXTI1 cho PC7 (LED vàng)
    if (EXTI->PR & (1 << 7)) {        // Kiểm tra nếu PC7 gây ngắt
        if (!exti1_flag) {            // Chỉ bật nếu LED vàng chưa sáng
            exti1_flag = 1;
            led_yellow_time = 5;      // 0.5 giây
            GPIOA->ODR |= (1 << 6);   // Bật LED vàng PA6
        }
        EXTI->PR |= (1 << 7);         // Xóa cờ ngắt
    }
}

	int main(void) {
    init_GPIO();
    init_Interrupt();
    init_TIM2();
    while (1) {
    	if (!(GPIOA->IDR & (1 << 0))) {
    	            NVIC_DisableIRQ(EXTI9_5_IRQn); // Tạm thời vô hiệu hóa EXTI7
    	        } else {
    	            NVIC_EnableIRQ(EXTI9_5_IRQn); // Kích hoạt lại EXTI7
    	        }
    }
}

//#include "stm32f4xx.h"
//#include <stdint.h>
//
//void init_GPIO(void) {
//    RCC->AHB1ENR |= (1 << 0);         // Kích hoạt clock cho PORTA
//    RCC->AHB1ENR |= (1 << 2);         // Kích hoạt clock cho PORTC
//
//    GPIOA->MODER |= (1 << (7 * 2));   // Thiết lập chân PA7 ở chế độ output (LED đỏ)
//    GPIOA->MODER &= ~(1 << (7 * 2 + 1)); // Xóa bit 15 về 0 để PA7 là output push-pull
//
//    GPIOA->MODER |= (1 << (6 * 2));   // Thiết lập chân PA6 ở chế độ output(LED xanh)
//    GPIOA->MODER &= ~(1 << (6 * 2 + 1)); // Xóa bit 13 về 0 để PA6 là output push-pull
//
//    GPIOC->MODER &= ~(1 << (2 * 2));  // Thiết lập chân PC2 ở chế độ input: Xóa bit 4 về 0 (IR sensor)
//    GPIOC->MODER &= ~(1 << (2 * 2 + 1)); // Xóa bit 5 về 0 để PC2 là input
//}
//
//void init_Interrupt(void) {           // Hàm cấu hình ngắt ngoài cho PC2
//    RCC->APB2ENR |= (1 << 14);        // Bật clock cho SYSCFG
//
//    SYSCFG->EXTICR[0] |= (2 << 8);    // Kết nối EXTI2 với PC2
//
//    EXTI->IMR |= (1 << 2);            // Bật ngắt EXTI2: Đặt bit 2 trong IMR lên 1 để cho phép ngắt từ PC2
//    EXTI->FTSR |= (1 << 2);           // Kích hoạt ngắt cạnh xuống
//    EXTI->RTSR |= (1 << 2);           // Kích hoạt ngắt cạnh lên
//    NVIC_EnableIRQ(EXTI2_IRQn);       // Bật ngắt EXTI2 trong NVIC
//    NVIC_SetPriority(EXTI2_IRQn, 1);  // Đặt mức ưu tiên 1 cho EXTI2
//}
//
//void init_TIM2(void) {                // Hàm cấu hình Timer 2 cho LED đỏ nhấp nháy
//    RCC->APB1ENR |= (1 << 0);         // Kích hoạt clock cho TIM2: Đặt bit 0 trong APB1ENR lên 1
//    TIM2->PSC = 8399;                 // Thiết lập Prescaler: Chia clock 84 MHz xuống 10 kHz (84 MHz / (8399 + 1))
//    TIM2->ARR = 999;                  // Thiết lập Auto-Reload: Đặt giá trị tối đa 999, tạo chu kỳ 100 ms
//    TIM2->DIER |= (1 << 0);           // Bật ngắt khi Timer cập nhật: Đặt bit 0 trong DIER lên 1
//    TIM2->CR1 |= (1 << 0);            // Bật Timer 2: Đặt bit 0 trong CR1 lên 1
//    NVIC_EnableIRQ(TIM2_IRQn);        // Bật ngắt TIM2 trong NVIC
//    NVIC_SetPriority(TIM2_IRQn, 0);   // Đặt mức ưu tiên 0 cho TIM2 (cao hơn EXTI2 để LED đỏ nhấp nháy ổn định)
//}
//
//void TIM2_IRQHandler(void) {          // Hàm xử lý ngắt của Timer 2 (LED đỏ nhấp nháy)
//    if (TIM2->SR & (1 << 0)) {        // Kiểm tra nếu Timer 2 gây ngắt: Kiểm tra bit 0 trong SR
//        static uint8_t toggle = 0;    // Biến tĩnh để lưu trạng thái nhấp nháy của LED đỏ
//        toggle = !toggle;             // Đảo trạng thái: Chuyển từ 0 sang 1 hoặc 1 sang 0 mỗi 100 ms
//        if (toggle) {                 // Nếu toggle = 1: Bật LED đỏ
//            GPIOA->ODR |= (1 << 7);   // Bật LED đỏ trên PA7: Đặt bit 7 trong ODR lên 1
//        } else {                      // Nếu toggle = 0: Tắt LED đỏ
//            GPIOA->ODR &= ~(1 << 7);  // Tắt LED đỏ trên PA7: Xóa bit 7 trong ODR về 0
//        }
//        TIM2->SR &= ~(1 << 0);        // Xóa cờ ngắt của Timer 2
//    }
//}
//
//void EXTI2_IRQHandler(void) {         // Hàm xử lý ngắt cho EXTI2 (PC2 - IR sensor)
//    if (EXTI->PR & (1 << 2)) {        // Kiểm tra nếu PC2 gây ngắt: Kiểm tra bit 2 trong PR
//        if (!(GPIOC->IDR & (1 << 2))) { // Nếu PC2 xuống thấp (có vật cản): Đọc IDR, kiểm tra bit 2 = 0
//            GPIOA->ODR |= (1 << 6);   // Bật LED xanh trên PA6: Đặt bit 6 trong ODR lên 1
//        } else {                      // Nếu PC2 lên cao (không còn vật cản): Bit 2 = 1
//            GPIOA->ODR &= ~(1 << 6);  // Tắt LED xanh trên PA6: Xóa bit 6 trong ODR về 0
//        }
//        EXTI->PR |= (1 << 2);         // Xóa cờ ngắt EXTI2: Đặt bit 2 trong PR lên 1
//    }
//}
//
//int main(void) {                      // Hàm chính
//    init_GPIO();                      // Khởi tạo GPIO
//    init_Interrupt();                 // Cấu hình ngắt ngoài cho PC2
//    init_TIM2();                      // Cấu hình Timer 2 cho LED đỏ
//    while (1) {                       // Vòng lặp vô hạn, không xử lý trong main
//    }
//}
