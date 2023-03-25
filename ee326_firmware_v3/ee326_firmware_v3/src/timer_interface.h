/*
 * timer_interface.h
 *
 * Created: 12/8/2016 9:46:51 AM
 *  Author: Ilya
 */ 


#ifndef TIMER_INTERFACE_H_
#define TIMER_INTERFACE_H_

#include <asf.h>
#include <string.h>
#include <stdio.h>

/** Timer counter frequency in Hz. */
#define TC_FREQ             1

volatile uint8_t counts;

void configure_tc(void);



#endif /* TIMER_INTERFACE_H_ */