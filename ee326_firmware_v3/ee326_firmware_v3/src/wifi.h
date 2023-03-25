/*
 * wifi.h
 *
 * Created: 2/13/2023 6:55:40 PM
 *  Author: kohda
 */ 


#ifndef WIFI_H_
#define WIFI_H_

#include <asf.h>
#include <string.h>
#include <ov2640.h>
#include <timer_interface.h>
#include <camera.h>

// USART STUFF HERE

#define BOARD_ID_USART             ID_USART0
#define BOARD_USART                USART0
#define BOARD_USART_BAUDRATE       115200
#define USART_Handler              USART0_Handler
#define USART_IRQn                 USART0_IRQn
#define wifi_spi_handler			SPI_Handler

#define ALL_INTERRUPT_MASK  0xffffffff

/** USART0 pin RX */
#define PIN_USART0_RXD	          {PIO_PA5A_RXD0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_PULLUP}
#define PIN_USART0_RXD_IDX        (PIO_PA5_IDX)
#define PIN_USART0_RXD_FLAGS      (PIO_PERIPH_A | PIO_PULLUP)
/** USART0 pin TX */
#define PIN_USART0_TXD            {PIO_PA6A_TXD0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_PULLUP}
#define PIN_USART0_TXD_IDX        (PIO_PA6_IDX)
#define PIN_USART0_TXD_FLAGS      (PIO_PERIPH_A | PIO_PULLUP)
/*
#define WIFI_COMM_PIN_NUM			PIO_PB10
#define WIFI_COMM_PIO				PIOB
#define WIFI_COMM_ID				ID_PIOB
#define WIFI_COMM_MASK				PIO_PB10_IDX
#define WIFI_COMM_ATTR				PIO_IT_RISE_EDGE
*/

#define WIFI_COMM_PIN_NUM			PIO_PA18
#define WIFI_COMM_PIO				PIOA
#define WIFI_COMM_ID				ID_PIOA
#define WIFI_COMM_MASK				PIO_PA18_IDX
#define WIFI_COMM_ATTR				PIO_IT_RISE_EDGE

volatile char input_line_wifi[1000];
volatile uint32_t input_pos_wifi;
volatile uint8_t data_received_flag;
volatile uint8_t success_received_flag;
volatile uint8_t wifi_provision_flag;

void configure_wifi_comm_pin(void);
void process_incoming_byte_wifi(uint8_t in_byte);
void process_data_wifi();
void write_image_to_web(void);

#endif /* WIFI_H_ */

// BUTTON TO SET ESP32 INTO PROVISION MODE

#ifndef BUTTON_H_
#define BUTTON_H_

#include <asf.h>

#define PROVISION_BUTTON_PIN_MSK	PIO_PA22
#define PROVISION_BUTTON_PIO		PIOA
#define PROVISION_BUTTON_ID			ID_PIOA
#define PROVISION_BUTTON_ATTR		PIO_DEBOUNCE | PIO_IT_EDGE | PIO_PULLUP

volatile uint32_t button_flag;

void wifi_provision_handler(uint32_t ul_id, uint32_t ul_mask);

// SPI STUFF HERE

#define WIFI_USART					USART0
#define WIFI_USART_ID				ID_USART0
#define WIFI_USART_BAUDRATE			115200//5000000//921600//
#define WIFI_USART_HANDLER			USART0_Handler
#define WIFI_USART_IRQn				USART0_IRQn
#define WIFI_USART_CHAR_LENGTH		US_MR_CHRL_8_BIT
#define WIFI_USART_PARITY			US_MR_PAR_NO
#define WIFI_USART_STOP_BITS		US_MR_NBSTOP_1_BIT
#define WIFI_USART_MODE				US_MR_CHMODE_NORMAL

#define PINS_WIFI_USART				(PIO_PA5A_RXD0 | PIO_PA6A_TXD0)
#define PINS_WIFI_USART_FLAGS		(PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_WIFI_USART_MASK		(PIO_PA5A_RXD0 | PIO_PA6A_TXD0)
#define PINS_WIFI_USART_PIO			PIOA
#define PINS_WIFI_USART_ID			ID_PIOA
#define PINS_WIFI_USART_TYPE		PIO_PERIPH_A
#define PINS_WIFI_USART_ATTR		PIO_DEFAULT


/** SPI MISO pin definition. */
#define SPI_MISO_GPIO         (PIO_PA12_IDX)
#define SPI_MISO_FLAGS        (PIO_PERIPH_A | PIO_DEFAULT)
/** SPI MOSI pin definition. */
#define SPI_MOSI_GPIO         (PIO_PA13_IDX)
#define SPI_MOSI_FLAGS        (PIO_PERIPH_A | PIO_DEFAULT)
/** SPI SPCK pin definition. */
#define SPI_SPCK_GPIO         (PIO_PA14_IDX)
#define SPI_SPCK_FLAGS        (PIO_PERIPH_A | PIO_DEFAULT)
/** SPI chip select 0 pin definition. (Only one configuration is possible) */
#define SPI_NPCS0_GPIO         (PIO_PA11_IDX)
#define SPI_NPCS0_FLAGS        (PIO_PERIPH_A | PIO_DEFAULT)

/* Chip select. */
#define SPI_CHIP_SEL 0
#define SPI_CHIP_PCS spi_get_pcs(SPI_CHIP_SEL)

/* Clock polarity. */
#define SPI_CLK_POLARITY 0

/* Clock phase. */
#define SPI_CLK_PHASE 0//1

/* Delay before SPCK. */
#define SPI_DLYBS 0x40

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0x10

void configure_wifi_provision_pin(void);

volatile uint32_t transfer_index;
volatile uint32_t transfer_len;

void spi_peripheral_initialize(void);
void prepare_spi_transfer(void);
void configure_spi(void);
void configure_usart_wifi(void);
void write_wifi_command(char* comm, uint8_t cnt);

#endif /* BUTTON_H_ */