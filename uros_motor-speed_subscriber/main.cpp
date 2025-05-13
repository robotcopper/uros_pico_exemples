
#include <stdio.h>
#include "pico/stdlib.h"

#include <time.h>
extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "robot_msg/msg/motor_msgs.h"

#include "pico_uart_transports.h"
}

#include "hardware/pwm.h"


uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d);

volatile bool message_received = false;

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

typedef struct {
    int sign;       // 0 pour négatif, 1 pour positif
    int absValue;   // Valeur absolue du nombre
} SignAndAbs;

const uint Motor1_step_pin = 1;
const uint Motor1_dir_pin = 2;
const uint Motor2_step_pin = 4;
const uint Motor2_dir_pin = 5;
const uint Motor3_step_pin = 6;
const uint Motor3_dir_pin = 7;

SignAndAbs getSignAndAbs(int number);

rcl_subscription_t subscriber;
robot_msg__msg__MotorMsgs subscriber_msg;


enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

const char * subscriber_topic_name = "pico_subscriber_topic";

void subscription_callback(const void * msgin){
    const robot_msg__msg__MotorMsgs * msg = (const robot_msg__msg__MotorMsgs *)msgin;
    
    gpio_set_function(Motor1_step_pin, GPIO_FUNC_PWM);
    uint slice_num1 = pwm_gpio_to_slice_num(Motor1_step_pin);
    uint chan1 = pwm_gpio_to_channel(Motor1_step_pin);
    pwm_set_freq_duty(slice_num1, chan1, getSignAndAbs(msg->motor1).absValue, 50);
    pwm_set_enabled(slice_num1, true);
    gpio_put(Motor1_dir_pin, getSignAndAbs(msg->motor1).sign);
    
    gpio_set_function(Motor2_step_pin, GPIO_FUNC_PWM);
    uint slice_num2 = pwm_gpio_to_slice_num(Motor2_step_pin);
    uint chan2 = pwm_gpio_to_channel(Motor2_step_pin);
    pwm_set_freq_duty(slice_num2, chan2, getSignAndAbs(msg->motor2).absValue, 50);
    pwm_set_enabled(slice_num2, true);
    gpio_put(Motor2_dir_pin, getSignAndAbs(msg->motor2).sign);
    
    gpio_set_function(Motor3_step_pin, GPIO_FUNC_PWM);
    uint slice_num3 = pwm_gpio_to_slice_num(Motor3_step_pin);
    uint chan3 = pwm_gpio_to_channel(Motor3_step_pin);
    pwm_set_freq_duty(slice_num3, chan3, getSignAndAbs(msg->motor3).absValue, 50);
    pwm_set_enabled(slice_num3, true);
    gpio_put(Motor3_dir_pin, getSignAndAbs(msg->motor1).sign);
    
    message_received = true;
    gpio_put(LED_PIN, 1);
}

bool pingAgent(){
    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 100;
    const uint8_t attempts = 1;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK){
    	//gpio_put(LED_PIN, 0);
    	return false;
    } else {
    	//gpio_put(LED_PIN, 1);
    }
    return true;
}

void createEntities(){
	allocator = rcl_get_default_allocator();

	rclc_support_init(&support, 0, NULL, &allocator);

	rclc_node_init_default(&node, "pico_node", "", &support);

	const rosidl_message_type_support_t * type_support =
	    ROSIDL_GET_MSG_TYPE_SUPPORT(robot_msg, msg, MotorMsgs);
	    
	rclc_subscription_init_default(
	        &subscriber,
	        &node,
	        type_support,
	        subscriber_topic_name);

	rclc_executor_init(&executor, &support.context, 1, &allocator);

	rclc_executor_add_subscription(&executor,
			&subscriber,
			&subscriber_msg,
			&subscription_callback,
			ON_NEW_DATA);
}

void destroyEntities(){
	rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
	(void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

	rcl_subscription_fini(&subscriber, &node);
	rcl_timer_fini(&timer);
	rclc_executor_fini(&executor);
	rcl_node_fini(&node);
	rclc_support_fini(&support);
}


uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d) {
    // This function sets the PWM frequency and duty cycle for a given PWM slice and channel.

    // Define the system clock frequency (assuming it's 125,000,000 Hz)
    uint32_t clock = 125000000;

    if (f == 0) {
        // If the desired frequency is 0, set the duty cycle to 0% to stop the PWM.
        pwm_set_chan_level(slice_num, chan, 0);
        return 0;
    }

    // Calculate the clock divider for the desired frequency and check if there's a remainder
    // This divider value is used to set the PWM frequency
    uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);

    // Ensure that the divider is at least 16 (minimum value)
    if (divider16 / 16 == 0)
        divider16 = 16;

    // Calculate the wrap value, which determines the PWM frequency
    uint32_t wrap = clock * 16 / divider16 / f - 1;

    // Set the clock divider for the PWM slice, using integer and fractional parts
    pwm_set_clkdiv_int_frac(slice_num, divider16 / 16, divider16 & 0xF);

    // Set the wrap value, which determines the PWM frequency
    pwm_set_wrap(slice_num, wrap);

    // Set the PWM channel's level to determine the duty cycle
    pwm_set_chan_level(slice_num, chan, wrap * d / 100);

    // Enable the PWM slice
    pwm_set_enabled(slice_num, true);

    // Return the wrap value
    return wrap;
}

SignAndAbs getSignAndAbs(int number) {
    SignAndAbs result;

    if (number >= 0) {
        result.sign = 1;      // Signe positif
    } else {
        result.sign = 0;      // Signe négatif
    }

    result.absValue = abs(number); // Valeur absolue du nombre

    return result;
}

int main()
{
    stdio_init_all();

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    bool pulse;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    gpio_init(Motor1_dir_pin);
    gpio_set_dir(Motor1_dir_pin, GPIO_OUT);
    gpio_init(Motor2_dir_pin);
    gpio_set_dir(Motor2_dir_pin, GPIO_OUT);
    gpio_init(Motor3_dir_pin);
    gpio_set_dir(Motor3_dir_pin, GPIO_OUT);
    
    allocator = rcl_get_default_allocator();
    state = WAITING_AGENT;
 

    while (true){
    	switch (state) {
    	    case WAITING_AGENT:
    	      state = pingAgent() ? AGENT_AVAILABLE : WAITING_AGENT;
    	      gpio_put(LED_PIN, 0);
    	      break;
    	    case AGENT_AVAILABLE:
    	      createEntities();
    	      state = AGENT_CONNECTED ;
    	      break;
    	    case AGENT_CONNECTED:
    	      state = pingAgent() ? AGENT_CONNECTED : AGENT_DISCONNECTED;
    	      if (state == AGENT_CONNECTED) {
    	        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    	        //gpio_put(LED_PIN, 1);
    	      }
    	      //gpio_put(LED_PIN, 0);
    	      break;
    	    case AGENT_DISCONNECTED:
    	      destroyEntities();
    	      state = WAITING_AGENT;
    	      gpio_put(LED_PIN, 0);
    	      break;
    	    default:
    	      break;
    	  }
    	  
	if (message_received) {
            message_received = false; // Réinitialiser l'indicateur
        } else {
            gpio_put(LED_PIN, 0); // Éteindre la LED si aucun nouveau message n'a été reçu
        }
        
        //pulse = ! pulse;
        //gpio_put(LED_PIN, pulse);
        

    }
    return 0;
}
