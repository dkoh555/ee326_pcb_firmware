/**
 * \file
 *
 * \brief User board initialization template
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

void board_init(void)
{
	//
	// Configuration of 'free' MCU pins connected to key ESP32 GPIOs (main.c) ------------------------------
	//
	
	ioport_set_pin_dir(COMM_GPIO_PIN_NUM, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(CLIENTS_GPIO_PIN_NUM, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(NET_GPIO_PIN_NUM, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(WIFI_RST_PIN_NUM, IOPORT_DIR_OUTPUT);
	
	
}
