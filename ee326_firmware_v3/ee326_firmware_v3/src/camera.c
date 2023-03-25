/*
 * camera.c
 *
 * Created: 2/13/2023 6:55:59 PM
 *  Author: kohda
 */ 

#include "camera.h"

//
// INITIALIZE VARIABLES ------------------------------------------------------------------------------------
//
volatile uint32_t g_ul_vsync_flag = false; // Vsync flag used for interrupts
volatile char g_p_uc_cap_dest_buf[100000]; // Buffer to contain the image
volatile uint32_t image_buffer_length = 100000; // Length of buffer (array) containing image
volatile uint32_t soi_offset_index = 0; // Index of start of image in buffer
volatile uint32_t eoi_offset_index = 0; // Index of end of image in buffer
volatile uint32_t image_length = 0; // Length of image in databus/buffer

// TWI clock frequency in Hz (400KHz)
#define TWI_CLK (400000UL);

// Configuration of TWI (two wire interface)
void configure_twi(void)
{
	
	twi_options_t opt;
	
	// Enable TWI peripheral
    pmc_enable_periph_clk(ID_BOARD_TWI);
	
	// Init TWI peripheral
	opt.master_clk = sysclk_get_cpu_hz();
	opt.speed = TWI_CLK;
	twi_master_init(BOARD_TWI, &opt);
	
	/* Configure TWI pins */
	#ifdef CONF_BOARD_TWI0
	gpio_configure_pin(TWI0_DATA_GPIO, TWI0_DATA_FLAGS);
	gpio_configure_pin(TWI0_CLK_GPIO, TWI0_CLK_FLAGS);
	#endif
	
	// Configure TWI interrupts
	NVIC_DisableIRQ(BOARD_TWI_IRQn);
	NVIC_ClearPendingIRQ(BOARD_TWI_IRQn);
	NVIC_SetPriority(BOARD_TWI_IRQn,0);
	NVIC_EnableIRQ(BOARD_TWI_IRQn);
	
	
}

// Configuration of camera pins, camera clock (XCLK), and calling the configure twi function
void init_camera(void)
{
    // Init Vsync handler
	init_vsync_interrupts();
	
	// Init PIO capture
	pio_capture_init(OV_DATA_BUS_PIO, OV_DATA_BUS_ID);
	
	/* Configure PCK1 pins */
	gpio_configure_pin(PIN_PCK1, PIN_PCK1_FLAGS);
	
	pmc_enable_pllbck(7, 0x1, 1); /* PLLB work at 96 Mhz */

	/* Configure Image sensor pins */
	//gpio_configure_pin(OV_SW_OVT_GPIO, OV_SW_OVT_FLAGS);
	gpio_configure_pin(OV_RST_GPIO, OV_RST_FLAGS);
	//gpio_configure_pin(OV_FSIN_GPIO, OV_FSIN_FLAGS);
	gpio_configure_pin(OV_HSYNC_GPIO, OV_HSYNC_FLAGS);
	gpio_configure_pin(OV_VSYNC_GPIO, OV_VSYNC_FLAGS);
	gpio_configure_pin(OV_DATA_BUS_D2, OV_DATA_BUS_FLAGS);
	gpio_configure_pin(OV_DATA_BUS_D3, OV_DATA_BUS_FLAGS);
	gpio_configure_pin(OV_DATA_BUS_D4, OV_DATA_BUS_FLAGS);
	gpio_configure_pin(OV_DATA_BUS_D5, OV_DATA_BUS_FLAGS);
	gpio_configure_pin(OV_DATA_BUS_D6, OV_DATA_BUS_FLAGS);
	gpio_configure_pin(OV_DATA_BUS_D7, OV_DATA_BUS_FLAGS);
	gpio_configure_pin(OV_DATA_BUS_D0, OV_DATA_BUS_FLAGS);
	gpio_configure_pin(OV_DATA_BUS_D1, OV_DATA_BUS_FLAGS);
	
	// Init PCK0 to work at 24MHz
	PMC->PMC_PCK[1] = (PMC_PCK_PRES_CLK_4 | PMC_PCK_CSS_PLLB_CLK);
	PMC->PMC_SCER = PMC_SCER_PCK1;
	while (!(PMC->PMC_SCSR & PMC_SCSR_PCK1)){
	}
		
	configure_twi();
	
	// OV initialization
	while (ov_init(BOARD_TWI) == 1){}
	
	configure_camera();
	delay_ms(3000);
	
}

// Configuration of OV2640 registers for desired operation
void configure_camera(void)
{
    ov_configure(BOARD_TWI,	JPEG_INIT);
	ov_configure(BOARD_TWI, YUV422);
	ov_configure(BOARD_TWI, JPEG);
	ov_configure(BOARD_TWI, JPEG_320x240);
	
}

// Captures an image after a rising edge of VSYNC, and gets image
// length. Returns 1 on success (i.e. a nonzero image length), 0 on error
uint8_t start_capture(void)
{
	/* Enable vsync interrupt*/
	pio_enable_interrupt(OV_VSYNC_PIO, OV_VSYNC_MASK);

	/* Capture acquisition will start on rising edge of Vsync signal.
	 * So wait g_vsync_flag = 1 before start process
	 */
	while (!g_ul_vsync_flag) {
	}

	/* Disable vsync interrupt*/
	pio_disable_interrupt(OV_VSYNC_PIO, OV_VSYNC_MASK);

	/* Enable pio capture*/
	pio_capture_enable(OV_DATA_BUS_PIO);

	/* Capture data and send it to external SRAM memory thanks to PDC
	 * feature */
	pio_capture_to_buffer(OV_DATA_BUS_PIO, g_p_uc_cap_dest_buf, image_buffer_length / 4);

	/* Wait end of capture*/
	while (!((OV_DATA_BUS_PIO->PIO_PCISR & PIO_PCIMR_RXBUFF) ==
			PIO_PCIMR_RXBUFF)) {
	}

	/* Disable pio capture*/
	pio_capture_disable(OV_DATA_BUS_PIO);
	
	/* Reset vsync flag*/
	g_ul_vsync_flag = false;
	
	if (find_image_len() == 1){
		return 1;
	}
	
	else {
		return 0;
	}
}

// Finds image length based on JPEG protocol. Returns 1 on success
// (i.e. able to find “end of image” and “start of image” markers), 0 on error
uint8_t find_image_len(void)
{
	uint32_t i = 0;
	while (i < image_buffer_length){
		if((g_p_uc_cap_dest_buf[i] == 0xFF) && (g_p_uc_cap_dest_buf[i+1] == 0xD8)){
			soi_offset_index = i;
			break;
		}
		i += 1;
	}
	
	while (i < image_buffer_length){
		if((g_p_uc_cap_dest_buf[i-1] == 0xFF) && (g_p_uc_cap_dest_buf[i] == 0xD9)){
			eoi_offset_index = i;
			break;
		}
		i += 1;
	}
	
	if (i >= image_buffer_length){
		return 0;
	}
	
	image_length = eoi_offset_index - soi_offset_index + 1;
	return 1;
}

// Handler for vertical synchronisation using by the OV image sensor.
static void vsync_handler(uint32_t ul_id, uint32_t ul_mask)
{
	unused(ul_id);
	unused(ul_mask);

	g_ul_vsync_flag = true;
}

// Intialize Vsync_Handler.
static void init_vsync_interrupts(void)
{
	/* Initialize PIO interrupt handler, see PIO definition in conf_board.h
	**/
	pio_handler_set(OV_VSYNC_PIO, OV_VSYNC_ID, OV_VSYNC_MASK,
			OV_VSYNC_TYPE, vsync_handler);

	/* Enable PIO controller IRQs */
	NVIC_EnableIRQ((IRQn_Type)OV_VSYNC_ID);
}

/**
 * \brief Initialize PIO capture for the OV image sensor communication.
 *
 * \param p_pio PIO instance to be configured in PIO capture mode.
 * \param ul_id Corresponding PIO ID.
 */
static void pio_capture_init(Pio *p_pio, uint32_t ul_id)
{
	/* Enable periphral clock */
	pmc_enable_periph_clk(ul_id);

	/* Disable pio capture */
	p_pio->PIO_PCMR &= ~((uint32_t)PIO_PCMR_PCEN);

	/* Disable rxbuff interrupt */
	p_pio->PIO_PCIDR |= PIO_PCIDR_RXBUFF;

	/* 32bit width*/
	p_pio->PIO_PCMR &= ~((uint32_t)PIO_PCMR_DSIZE_Msk);
	p_pio->PIO_PCMR |= PIO_PCMR_DSIZE_WORD;

	/* Only HSYNC and VSYNC enabled */
	p_pio->PIO_PCMR &= ~((uint32_t)PIO_PCMR_ALWYS);
	p_pio->PIO_PCMR &= ~((uint32_t)PIO_PCMR_HALFS);

}

/**
 * \brief Capture OV data to a buffer.
 *
 * \param p_pio PIO instance which will capture data from OV iamge sensor.
 * \param p_uc_buf Buffer address where captured data must be stored.
 * \param ul_size Data frame size.
 */
static uint8_t pio_capture_to_buffer(Pio *p_pio, uint8_t *uc_buf,
		uint32_t ul_size)
{
	/* Check if the first PDC bank is free */
	if ((p_pio->PIO_RCR == 0) && (p_pio->PIO_RNCR == 0)) {
		p_pio->PIO_RPR = (uint32_t)uc_buf;
		p_pio->PIO_RCR = ul_size;
		p_pio->PIO_PTCR = PIO_PTCR_RXTEN;
		return 1;
	} else if (p_pio->PIO_RNCR == 0) {
		p_pio->PIO_RNPR = (uint32_t)uc_buf;
		p_pio->PIO_RNCR = ul_size;
		return 1;
	} else {
		return 0;
	}
}
