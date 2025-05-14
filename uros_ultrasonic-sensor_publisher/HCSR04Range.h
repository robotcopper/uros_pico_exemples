/*
 * HCSR04Range.h
 *
 *  Created on: 5 Sept 2023
 *      Author: jondurrant
 */

#ifndef GPIOINT_SRC_HCSR04RANGE_H_
#define GPIOINT_SRC_HCSR04RANGE_H_

#include "pico/stdlib.h"
#include "GPIOObserver.h"
#include "pico/types.h"

#define MAX_HCSR04 			4

class HCSR04Range : public GPIOObserver {
public:
	HCSR04Range(uint8_t GPTrigger, uint8_t GPEcho);
	virtual ~HCSR04Range();

	/***
	 * Add an additional Echo Device
	 * @param GPEcho
	 */
	void addEcho(uint8_t GPEcho);

	/***
	 * Trigger a pulse
	 */
	void trigger();

	/***
	 * Read the most recent distance
	 * @return in mm
	 */
	int getDistanceMM(uint8_t index = 0);

	/***
	 * handle GPIO  events
	 * @param gpio - GPIO number
	 * @param events - Event
	 */
	virtual void handleGPIO(uint gpio, uint32_t events);

private:
	static int64_t alarmCB(alarm_id_t id, void *user_data);
	void triggerEnd(alarm_id_t id);


	uint8_t xGPTrigger;
	uint8_t xGPEchos[MAX_HCSR04];

	uint64_t xEchoEnd[MAX_HCSR04];
	uint64_t xEchoStart[MAX_HCSR04];

	uint8_t xNumEchos = 0;

	alarm_id_t xAlarm;


};

#endif /* GPIOINT_SRC_HCSR04RANGE_H_ */
