/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include <timer_interface.h>
#include <wifi.h>
#include <camera.h>

int main (void)
{
	//
	// Initializing every required component of the device -------------------------------------------------
	//
	
	ioport_set_pin_level(WIFI_RST_PIN_NUM, HIGH); // Enable the ESP32 to 'run'
	// System init
	sysclk_init();
	wdt_disable(WDT);
	board_init();
	ioport_init();
	// Timeout timer init
	configure_tc();
	// Wifi init
	configure_wifi_comm_pin();
	configure_usart_wifi();
	configure_spi();
	spi_peripheral_initialize();
	configure_wifi_provision_pin();
	// Cam init
	init_camera();
	configure_camera();
	// testing write_wifi_command to ESP32 w/ ver
	write_wifi_command("ver\r\n", 5);
	// configuring the wifi module
	// configuring the indicators
	write_wifi_command("set wlan_gpio 25\r\n", 5);
	write_wifi_command("set websocket_gpio 26\r\n", 5);
	write_wifi_command("set ap_gpio 27\r\n", 5);
	// configuring the command complete GPIO
	write_wifi_command("set comm_gpio 21\r\n", 5);
	// configuring the websocket clients GPIO
	write_wifi_command("set clients_gpio 22\r\n", 5);
	// configuring the network GPIO
	write_wifi_command("set net_gpio 23\r\n", 5);
	// set SPI baud rate
	write_wifi_command("set spi_baud 100000\r\n", 5);

	//
	// Reset Wifi Module -----------------------------------------------------------------------------------
	//
	
	data_received_flag = false;
	while (!data_received_flag){ // "Reset wifi module" loop
		// Set reset to low then high to complete the ESP32 reset
		ioport_set_pin_level(WIFI_RST_PIN_NUM, LOW);
		delay_ms(100);
		ioport_set_pin_level(WIFI_RST_PIN_NUM, HIGH);
		delay_ms(100);
		
		while (!ioport_get_pin_level(NET_GPIO_PIN_NUM)){ // "Wait for connection" loop
			if (wifi_provision_flag){
				write_wifi_command("provision\r\n", 10);
				wifi_provision_flag = false;
			}
		}
		// Send "test" wifi command to ESP32
		write_wifi_command("test\r\n", 10);
		delay_ms(100);
	}
	
	/*
	g_p_uc_cap_dest_buf[0] = 0x12;
	g_p_uc_cap_dest_buf[1] = 0xFF;
	g_p_uc_cap_dest_buf[2] = 0xD8;
	g_p_uc_cap_dest_buf[3] = 0x10;
	g_p_uc_cap_dest_buf[4] = 0xFF;
	g_p_uc_cap_dest_buf[5] = 0xD9;
	g_p_uc_cap_dest_buf[6] = 0x12;
	
	find_image_len();
	*/
	
	while (1){
		if (wifi_provision_flag){
			write_wifi_command("provision\r\n", 10);
			wifi_provision_flag = false;
			while (!ioport_get_pin_level(NET_GPIO_PIN_NUM)){
			}
			continue;
		}
		if (!ioport_get_pin_level(NET_GPIO_PIN_NUM)){
			continue;
		}
		
		if (!ioport_get_pin_level(CLIENTS_GPIO_PIN_NUM)){
			continue;
		}
		
		if (start_capture()){
			write_image_to_web();
		}
	}
	
}
