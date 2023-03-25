/*
 * wifi.c
 *
 * Created: 2/13/2023 6:55:28 PM
 *  Author: kohda
 */ 

#include "wifi.h"
#include <string.h>
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"

volatile uint32_t received_byte_wifi = 0;
volatile bool new_rx_wifi = false;
volatile uint32_t input_pos_wifi = 0;
uint32_t counter = 0;

/** Size of the receive buffer used by the PDC, in bytes. */
#define BUFFER_SIZE         100

/** USART PDC transfer type definition. */
#define PDC_TRANSFER        1

/** USART FIFO transfer type definition. */
#define BYTE_TRANSFER       0

/** Max buffer number. */
#define MAX_BUF_NUM         1

/** All interrupt mask. */
#define ALL_INTERRUPT_MASK  0xffffffff

/** Timer counter frequency in Hz. */
#define TC_FREQ             1

#define STRING_EOL    "\r"
#define STRING_HEADER "-- USART Serial Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL


/** Current bytes in buffer. */
static uint32_t gs_ul_size_buffer = BUFFER_SIZE;

/** Current bytes in next buffer. */
static uint32_t gs_ul_size_nextbuffer = BUFFER_SIZE;

/** Byte mode read buffer. */
static uint32_t gs_ul_read_buffer = 0;

/** Current transfer mode. */
static uint8_t gs_uc_trans_mode = PDC_TRANSFER;

/** Buffer number in use. */
static uint8_t gs_uc_buf_num = 0;

/** PDC data packet. */
pdc_packet_t g_st_packet, g_st_nextpacket;

/** Pointer to PDC register base. */
Pdc *g_p_pdc;

/** Flag of one transfer end. */
static uint8_t g_uc_transend_flag = 0;

/** Flag indicating data received **/
volatile uint8_t data_received_flag = false;

/** Flag indicating success received **/
volatile uint8_t success_received_flag = false;

/** Flag indicating Provisioning Mode of ESP32 initiated**/
volatile uint8_t wifi_provision_flag = false;

/* Pointer to transfer buffer. */
volatile uint32_t transfer_index = 0;
volatile uint32_t transfer_len = 0;
volatile uint32_t buffer_size = 1000;

/**
 * \brief Interrupt handler for USART. Echo the bytes received and start the
 * next receive.
 */
void USART_Handler(void)
{
	uint32_t ul_status;

	/* Read USART Status. */
	ul_status = usart_get_status(BOARD_USART);

	if (ul_status & US_CSR_RXBUFF) {
		usart_read(BOARD_USART, &received_byte_wifi);
		new_rx_wifi = true;
		process_incoming_byte_wifi((uint8_t)received_byte_wifi);
	}
}

void process_incoming_byte_wifi(uint8_t in_byte) {
	input_line_wifi[input_pos_wifi++] = in_byte;
}

void wifi_command_response_handler(uint32_t ul_id, uint32_t ul_mask) {
	process_data_wifi();
	for (uint32_t jj = 0; jj < 1000; jj++) {
		input_line_wifi[jj] = 0;
	}
	input_pos_wifi = 0;
	data_received_flag = true;
}


void process_data_wifi() {
	if (strstr(input_line_wifi, "SUCCESS\r\n")) {
		success_received_flag = true;
	}
}

void wifi_provision_handler(uint32_t ul_id, uint32_t ul_mask){
	
	unused(ul_id);
	unused(ul_mask);
	
	wifi_provision_flag = true;
	
}

void configure_wifi_provision_pin(void){
	
	pmc_enable_periph_clk(PROVISION_BUTTON_ID);
	
	pio_set_debounce_filter(PROVISION_BUTTON_PIO, PROVISION_BUTTON_PIN_MSK, 10);
	
	pio_handler_set(PROVISION_BUTTON_PIO, PROVISION_BUTTON_ID, PROVISION_BUTTON_PIN_MSK, PROVISION_BUTTON_ATTR, wifi_provision_handler);
	
	NVIC_EnableIRQ((IRQn_Type)PROVISION_BUTTON_ID);
	
	pio_enable_interrupt(PROVISION_BUTTON_PIO, PROVISION_BUTTON_PIN_MSK);
	
}

void configure_wifi_comm_pin(void)
{
	/* Configure PIO clock. */
	pmc_enable_periph_clk(WIFI_COMM_ID);

	/* Initialize PIO interrupt handler, see PIO definition in conf_board.h
	**/
	pio_handler_set(WIFI_COMM_PIO, WIFI_COMM_ID, WIFI_COMM_PIN_NUM, WIFI_COMM_ATTR, wifi_command_response_handler);

	/* Enable PIO controller IRQs. */
	NVIC_EnableIRQ((IRQn_Type)WIFI_COMM_ID);

	/* Enable PIO interrupt lines. */
	pio_enable_interrupt(WIFI_COMM_PIO, WIFI_COMM_PIN_NUM);
}

void wifi_spi_handler(void)
{
	uint32_t new_cmd = 0;
	static uint16_t data;
	uint8_t uc_pcs;
	
	counter++;

	if (spi_read_status(SPI) & SPI_SR_RDRF) {
		spi_read(SPI, &data, &uc_pcs);
		
		if (transfer_len--) {
			spi_write(SPI, g_p_uc_cap_dest_buf[transfer_index++], 0, 0);
		}
	}
}

void configure_usart_wifi(void)
{
	gpio_configure_pin(PIN_USART0_RXD_IDX, PIN_USART0_RXD_FLAGS);
	gpio_configure_pin(PIN_USART0_TXD_IDX, PIN_USART0_TXD_FLAGS);
	
	static uint32_t ul_sysclk;
	const sam_usart_opt_t wifi_usart_settings = {
		WIFI_USART_BAUDRATE,
		WIFI_USART_CHAR_LENGTH,
		WIFI_USART_PARITY,
		WIFI_USART_STOP_BITS,
		WIFI_USART_MODE,
		/* This field is only used in IrDA mode. */
		0
	};
	
	/* Get system clock. */
	ul_sysclk = sysclk_get_cpu_hz();
	
	pmc_enable_periph_clk(WIFI_USART_ID);
	
	usart_init_rs232(WIFI_USART,&wifi_usart_settings,ul_sysclk);

	/* Disable all the interrupts. */
	usart_disable_interrupt(WIFI_USART, ALL_INTERRUPT_MASK);
	
	/* Enable TX & RX function. */
	usart_enable_tx(WIFI_USART);
	usart_enable_rx(WIFI_USART);

	usart_enable_interrupt(WIFI_USART, US_IER_RXRDY);

	/* Configure and enable interrupt of USART. */
	NVIC_EnableIRQ(WIFI_USART_IRQn);
}

void configure_spi()
{
	gpio_configure_pin(SPI_MISO_GPIO, SPI_MISO_FLAGS);
	gpio_configure_pin(SPI_MOSI_GPIO, SPI_MOSI_FLAGS);
	gpio_configure_pin(SPI_SPCK_GPIO, SPI_SPCK_FLAGS);
	gpio_configure_pin(SPI_NPCS0_GPIO, SPI_NPCS0_FLAGS);
	
	/* Configure SPI interrupts for slave only. */
	NVIC_DisableIRQ(SPI_IRQn);
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn, 0);
	NVIC_EnableIRQ(SPI_IRQn);
}

void spi_peripheral_initialize(void)
{
	spi_enable_clock(SPI);
	spi_disable(SPI);
	spi_reset(SPI);
	spi_set_slave_mode(SPI);
	spi_disable_mode_fault_detect(SPI);
	spi_set_peripheral_chip_select_value(SPI, SPI_CHIP_PCS);
	spi_set_clock_polarity(SPI, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI, SPI_CHIP_SEL, SPI_CSR_BITS_8_BIT);
	spi_enable_interrupt(SPI, SPI_IER_RDRF);
	spi_enable(SPI);
}

void prepare_spi_transfer(void)
{
	transfer_len = image_length;
	transfer_index = soi_offset_index;
}

void write_wifi_command(char* comm, uint8_t cnt)
{
	data_received_flag = false;
	counts = 0;
	usart_write_line(WIFI_USART, comm);
	while ((counts < cnt) && (!data_received_flag)){
	}
		
}

void write_image_to_web(void)
{
	if (image_length == 0){
		return;
	}
	
	prepare_spi_transfer();
	
	uint8_t msg[100];
	//sprintf(msg, "image_transfer %i\r\n", image_length);
	sprintf(msg, "image_test %i\r\n", image_length);
	write_wifi_command(msg, 5);
}