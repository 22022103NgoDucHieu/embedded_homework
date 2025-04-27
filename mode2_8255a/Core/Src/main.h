/* Define to prevent recursive inclusion */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32f4xx_hal.h"

/* Exported functions prototypes */
void Error_Handler(void);

/* Private defines */
#define CS_Pin GPIO_PIN_0
#define CS_GPIO_Port GPIOB
#define RD_Pin GPIO_PIN_1
#define RD_GPIO_Port GPIOB
#define WR_Pin GPIO_PIN_2
#define WR_GPIO_Port GPIOB
#define A0_Pin GPIO_PIN_3
#define A0_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_4
#define A1_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
