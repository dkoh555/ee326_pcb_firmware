/**
 * \file
 *
 * \brief User board configuration template
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

// USART AND SPI PINS ARE ALREADY DEFINED IN WIFI.H

// CAMERA PINS TO BE DEFINED IN CAMERA.H

#define BUTTON_PIN_1	//PIO_PA22_IDX     // NEED TO REPLACE PIN DEFINITION WITH NRST PIN
#define BUTTON_PIN_2	//PIO_PA22_IDX    // PROVISION BUTTON FOR ESP32

//
// Declaration of 'free' MCU pins connected to key ESP32 GPIOs (main.c) ------------------------------
//

#define COMM_GPIO_PIN_NUM		PIO_PA18_IDX
#define CLIENTS_GPIO_PIN_NUM	PIO_PA21_IDX
#define NET_GPIO_PIN_NUM		PIO_PA19_IDX
#define WIFI_RST_PIN_NUM		PIO_PA0_IDX

#endif // CONF_BOARD_H
