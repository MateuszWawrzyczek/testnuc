/*
 * toggle_pins.h
 *
 *  Created on: Jan 16, 2025
 *      Author: matwa
 */

#ifndef INC_TOGGLE_PINS_H_
#define INC_TOGGLE_PINS_H_

//przełączanie pomiędzy pinami
void toggle_pins(void) {
    static uint8_t state = 0;

    switch (state) {
        case 0:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // PB0 HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);  // PB1 HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // PB2 LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // PB10 LOW

            break;
        case 1:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // PB0 LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    // PB1 HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);  // PB2 HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // PB10 LOW
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // PB0 LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // PB1 LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    // PB2 HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // PB10 HIGH
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // PB0 HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // PB1 LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // PB2 LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   // PB10 HIGH
            break;
    }

    state = (state + 1) % 4;

    HAL_Delay(1000);
}

#endif /* INC_TOGGLE_PINS_H_ */
