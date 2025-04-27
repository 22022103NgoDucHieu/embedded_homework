#include <main.h>
#include <stm32f4xx.h>

void init_GPIO(void) {
	RCC->AHB1ENR |= (1 << 0); // KÍch hoạt clock cho PORTA
	RCC->AHB1ENR |= (1 << 2);// KÍch hoạt clock cho PORTC

	GPIOA->MODER |= (1 << (5 * 2)); // Thiết lập chân PA5 ở chế độ output
	GPIOA->MODER &= ~(1 << (5 * 2 + 1));

	GPIOA->PUPDR |= (1 << (5 * 2)); //Thiết lập chế đôj mặc đinh của PA5 là pull-up
	GPIOA->PUPDR &= ~(1 << (5 * 2 + 1));

	GPIOC->MODER &= ~(1 << (13 * 2)); //Thiết lập chân PC13 ở chế độ output(đã được nối sẵn với nút của board theo kiểu pull-up)
	GPIOC->MODER &= ~(1 << (13 * 2 + 1));

}


void init_Interupt(void) {
	RCC->APB2ENR |= (1 << 14);  // Bật clock SYSCFG

	SYSCFG->EXTICR[3] |= (2 << 4);  // EXTI13 kết nối với PC13

	EXTI->IMR |= (1 << 13);   // Bật ngắt EXTI13
	EXTI->FTSR |= (1 << 13);  // Kích hoạt ngắt cạnh xuống (Falling edge)

	NVIC_EnableIRQ(EXTI15_10_IRQn);  // Bật ngắt trong NVIC
	NVIC_SetPriority(EXTI15_10_IRQn, 2);  // Đặt mức ưu tiên



}

void EXTI15_10_IRQHandler(void) { // hàm để bắt sự kiện ngắt ở các đường dẫn từ EXTI10 đến EXTI15
    if (EXTI->PR & (1 << 13)) {  // Kiểm tra nếu PC13 gây ngắt
    	uint32_t previous_state = (GPIOA->ODR >> 5) & 0x1; // đọc trạng thái chân PA5
    	       if (previous_state == 0) // nếu bằng 0 thì cấp điện áp
    	       {
    	                GPIOA->ODR |= 1 << 5;
    	       }
    	      else // nếu bằng 1 thì dừng cấp điện
    	       {
    	    	  	  	 GPIOA->ODR &= ~(1 << 5);
    	       }
        EXTI->PR |= (1 << 13);  // Xóa cờ ngắt
    }
}


int main(void) {
	init_GPIO();
	init_Interupt();

	while(1);
}
