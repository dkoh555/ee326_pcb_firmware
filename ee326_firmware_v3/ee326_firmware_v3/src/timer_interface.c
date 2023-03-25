/*
 * timer_interface.c
 *
 * Created: 12/8/2016 9:46:36 AM
 *  Author: Ilya
 */ 

#include "timer_interface.h"

void TC0_Handler(void)
{
	uint32_t ul_status;

	// Read TC0 status.
	ul_status = tc_get_status(TC0, 0);

	// RC compare.
	if ((ul_status & TC_SR_CPCS) == TC_SR_CPCS) {
		counts++;
	}
}

void configure_tc(void)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk;

	// Get system clock.
	ul_sysclk = sysclk_get_cpu_hz();

	// Configure PMC.
	pmc_enable_periph_clk(ID_TC0);

	// Configure TC for a 1Hz frequency and trigger on RC compare.
	tc_find_mck_divisor(TC_FREQ, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC0, 0, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC0, 0, (ul_sysclk / ul_div) / TC_FREQ);

	// Configure and enable interrupt on RC compare.
	NVIC_EnableIRQ((IRQn_Type) ID_TC0);
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
	
	// Start the timer
	tc_start(TC0, 0);
}