/*
 * camera.h
 *
 * Created: 2/13/2023 6:55:50 PM
 *  Author: kohda
 */ 


#ifndef CAMERA_H_
#define CAMERA_H_
#include <asf.h>
#include "ov2640.h"

//
// KEY DEFINITIONS -----------------------------------------------------------------------------------------
//
/* TWI board defines. */
#define ID_BOARD_TWI                   ID_TWI0
#define BOARD_TWI                      TWI0
#define BOARD_TWI_IRQn                 TWI0_IRQn

/** Configure TWI0 pins (for camera communications). */
#define CONF_BOARD_TWI0

/** Configure PCK0 pins (for camera communications). */
#define CONF_BOARD_PCK1

/******************************* TWI definition
 *********************************/
/** TWI0 Data pins definition */
#define TWI0_DATA_GPIO                 PIO_PA3_IDX
#define TWI0_DATA_FLAGS                (PIO_PERIPH_A | PIO_PULLUP)
#define TWI0_DATA_MASK                 PIO_PA3
#define TWI0_DATA_PIO                  PIOA
#define TWI0_DATA_ID                   ID_PIOA
#define TWI0_DATA_TYPE                 PIO_PERIPH_A
#define TWI0_DATA_ATTR                 PIO_DEFAULT

/** TWI0 clock pins definition */
#define TWI0_CLK_GPIO                  PIO_PA4_IDX
#define TWI0_CLK_FLAGS                 (PIO_PERIPH_A | PIO_PULLUP)
#define TWI0_CLK_MASK                  PIO_PA4
#define TWI0_CLK_PIO                   PIOA
#define TWI0_CLK_ID                    ID_PIOA
#define TWI0_CLK_TYPE                  PIO_PERIPH_A
#define TWI0_CLK_ATTR                  PIO_DEFAULT

/******************************* PCK1 definition
 *********************************/
/** PCK1 */
#define PIN_PCK1                       (PIO_PA17_IDX)
#define PIN_PCK1_FLAGS                 (PIO_PERIPH_B | PIO_DEFAULT)

/* Image sensor board defines. */
/** OV Data Bus pins */
#define OV_DATA_BUS_D0                 PIO_PA24_IDX
#define OV_DATA_BUS_D1                 PIO_PA25_IDX
#define OV_DATA_BUS_D2                 PIO_PA26_IDX
#define OV_DATA_BUS_D3                 PIO_PA27_IDX
#define OV_DATA_BUS_D4                 PIO_PA28_IDX
#define OV_DATA_BUS_D5                 PIO_PA29_IDX
#define OV_DATA_BUS_D6                 PIO_PA30_IDX
#define OV_DATA_BUS_D7                 PIO_PA31_IDX
#define OV_DATA_BUS_FLAGS              (PIO_INPUT | PIO_PULLUP)
#define OV_DATA_BUS_MASK               (0xFF000000UL)
#define OV_DATA_BUS_PIO                PIOA
#define OV_DATA_BUS_ID                 ID_PIOA
#define OV_DATA_BUS_TYPE               PIO_INPUT
#define OV_DATA_BUS_ATTR               PIO_DEFAULT

/** OV_VSYNC pin definition */
#define OV_VSYNC_GPIO                  PIO_PA15_IDX
#define OV_VSYNC_FLAGS                 (PIO_PULLUP | PIO_IT_RISE_EDGE)
#define OV_VSYNC_MASK                  PIO_PA15
#define OV_VSYNC_PIO                   PIOA
#define OV_VSYNC_ID                    ID_PIOA
#define OV_VSYNC_TYPE                  PIO_PULLUP

/** OV_HSYNC pin definition */
#define OV_HSYNC_GPIO                  PIO_PA16_IDX
#define OV_HSYNC_FLAGS                 (PIO_PULLUP | PIO_IT_RISE_EDGE)
#define OV_HSYNC_MASK                  PIO_PA16
#define OV_HSYNC_PIO                   PIOA
#define OV_HSYNC_ID                    ID_PIOA
#define OV_HSYNC_TYPE                  PIO_PULLUP

/** OV_RST pin definition */
#define OV_RST_GPIO                    PIO_PA20_IDX
#define OV_RST_FLAGS                   (PIO_OUTPUT_1 | PIO_DEFAULT)
#define OV_RST_MASK                    PIO_PA20
#define OV_RST_PIO                     PIOA
#define OV_RST_ID                      ID_PIOA
#define OV_RST_TYPE                    PIO_OUTPUT_1

//
// CAMERA.C FUNCTION DECLARATIONS ----------------------------------------------------------------------------
//
void configure_twi(void);
void init_camera(void);
void configure_camera(void);
uint8_t start_capture(void);
uint8_t find_image_len(void);
static void vsync_handler(uint32_t ul_id, uint32_t ul_mask);
static void init_vsync_interrupts(void);
static void pio_capture_init(Pio *p_pio, uint32_t ul_id);
static uint8_t pio_capture_to_buffer(Pio *p_pio, uint8_t *uc_buf, uint32_t ul_size);

//
// VARIABLE DECLARATIONS ------------------------------------------------------------------------------------
//
volatile uint32_t g_ul_vsync_flag; // Vsync flag used for interrupts
volatile char g_p_uc_cap_dest_buf[100000]; // Buffer to contain the image
volatile uint32_t image_buffer_length; // Length of buffer (array) containing image
volatile uint32_t soi_offset_index; // Index of start of image in buffer
volatile uint32_t eoi_offset_index; // Index of end of image in buffer
volatile uint32_t image_length; // Length of image in databus/buffer


#endif /* CAMERA_H_ */