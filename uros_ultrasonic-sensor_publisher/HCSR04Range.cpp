/*
 * HCSR04Range.cpp
 *
 *  Created on: 5 Sept 2023
 *      Author: jondurrant
 */

#include "HCSR04Range.h"
#include "hardware/gpio.h"
#include "GPIOInputMgr.h"
#include <cstdio>




HCSR04Range::HCSR04Range(uint8_t GPTrigger, uint8_t GPEcho){
	xGPTrigger = GPTrigger;
	xGPEchos[0] = GPEcho;
	xNumEchos = 1;

	gpio_init(GPTrigger);
	gpio_set_dir(GPTrigger, GPIO_OUT);

	GPIOInputMgr::getMgr()->addObserver(xGPEchos[0],  this);

}

HCSR04Range::~HCSR04Range() {
	// TODO Auto-generated destructor stub
}


void HCSR04Range::trigger(){
	gpio_put (xGPTrigger,true);

	xAlarm = add_alarm_in_us	(10,
		HCSR04Range::alarmCB,
		this,
		true
	);

}

int HCSR04Range::getDistanceMM(uint8_t index){

	if (index >= xNumEchos){
		return -1;
	}
	if (xEchoEnd[index] <= xEchoStart[index]){
		return -1;
	}
	uint64_t us = xEchoEnd[index] - xEchoStart[index];

	float mm = (0,034 * (float)us) / 2.0;
	return (int)mm;
}

/***
 * handle GPIO  events
 * @param gpio - GPIO number
 * @param events - Event
 */
 void HCSR04Range::handleGPIO(uint gpio, uint32_t events){
	 uint64_t now = to_us_since_boot (get_absolute_time());

	 //printf("CB %u 0X%X\n", gpio, events);
	 //find index
	 uint8_t index = -1;
	 for (uint i =0; i < xNumEchos; i++){
		 if (gpio == xGPEchos[i]){
			 index = i;
			 break;
		 }
	 }
	 if (index >= 0){
		 if ((events & GPIO_IRQ_EDGE_RISE)  > 0) {
			 xEchoStart[index] = now;
			 //printf("Start %u\n", index);
		 }
		 if ((events &  GPIO_IRQ_EDGE_FALL) > 0){
			 xEchoEnd[index] = now;
			 //printf("End %u\n", index);
		 }
	 }
 }


 int64_t HCSR04Range::alarmCB(alarm_id_t id, void *user_data){

	 HCSR04Range *pRange = (HCSR04Range *) user_data;
	 pRange->triggerEnd(id);

	 return 0;
 }

 void HCSR04Range::triggerEnd(alarm_id_t id){
	 if (id == xAlarm){
		 gpio_put (xGPTrigger, false);
	 }
 }


/***
* Add an additional Echo Device
* @param GPEcho
*/
void HCSR04Range::addEcho(uint8_t GPEcho){
	if (xNumEchos < MAX_HCSR04){
		xGPEchos[xNumEchos] = GPEcho;

		GPIOInputMgr::getMgr()->addObserver(GPEcho,  this);

		xNumEchos++;

	}
}
