#include <stdio.h>
#include "pico/stdlib.h" // Include the standard library for Raspberry Pi Pico

extern "C" {
#include <rcl/rcl.h> // Main ROS 2 client library
#include <rcl/error_handling.h> // Error handling for ROS 2
#include <rclc/rclc.h> // C library for ROS 2
#include <rclc/executor.h> // Executor for ROS 2
#include <rmw_microros/rmw_microros.h> // Middleware for micro-ROS

#include "sensor_msgs/msg/joint_state.h"
// #include "rosidl_runtime_c/primitives_sequence_functions.h"
// #include "rosidl_runtime_c/string_functions.h"
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "pico_uart_transports.h" // UART transport specific for Pico
}

#include <cmath>
#include "hardware/pwm.h"

constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN; // Define the LED pin number

rcl_subscription_t subscriber; // Declare the ROS 2 subscriber
sensor_msgs__msg__JointState subscriber_msg; // Declare the ROS 2 message

bool message_received = false; // Flag for message reception

const char * subscriber_topic_name = "topic_based_joint_commands"; //"pico_subscriber_topic";
const char * node_name = "pico_node";

// Define the states
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

rcl_node_t node; // Declare the ROS 2 node
rcl_allocator_t allocator; // Declare the memory allocator
rclc_support_t support; // Declare the ROS 2 support
rclc_executor_t executor; // Declare the ROS 2 executor

#define CHECK_RET(ret) if (ret != RCL_RET_OK) { rcl_reset_error(); } // Macro for silent error handling


typedef struct {
    int sign;       // 0 if negativ, 1 if positiv
    double absValue;   // abs of the number
} SignAndAbs;

const int nb_step_17hs192004s1 = 200;
const int micro_step = 8;
const int nb_step_per_revolution = micro_step * nb_step_17hs192004s1;

const uint Motor1_step_pin = 7; //GPIO7 / PIN10
const uint Motor1_dir_pin = 8; //GPIO8 / PIN11
const uint Motor2_step_pin = 10; //GPIO10 / PIN14
const uint Motor2_dir_pin = 11; //GPIO11 / PIN15
const uint Motor3_step_pin = 13; //GPIO13 / PIN17
const uint Motor3_dir_pin = 14; //GPIO14 / PIN19

static micro_ros_utilities_memory_conf_t conf = micro_ros_utilities_memory_conf_default;

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d) { // This function sets the PWM frequency and duty cycle for a given PWM slice and channel.
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

SignAndAbs getSignAndAbs(double number) {
    SignAndAbs result;

    if (number >= 0) {
        result.sign = 1;      // pisitiv
    } else {
        result.sign = 0;      // negativ
    }

    result.absValue = abs(number); // abs

    return result;
}

int velToStep(float vel){
    //return the frequency of the PWM that as to be send to match the velocity 
    return (vel * nb_step_per_revolution)/(M_PI);
}


void subscription_callback(const void * msgin){
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;

    // Copy incoming data
    subscriber_msg.velocity.size = msg->velocity.size;
    for (size_t i = 0; i < msg->velocity.size; i++) {
        subscriber_msg.velocity.data[i] = msg->velocity.data[i];
    }

    // Handle motors
    if (msg->velocity.size > 0) {
        SignAndAbs motor1 = getSignAndAbs(subscriber_msg.velocity.data[0]);
        uint slice_num1 = pwm_gpio_to_slice_num(Motor1_step_pin);
        uint chan1 = pwm_gpio_to_channel(Motor1_step_pin);
        pwm_set_freq_duty(slice_num1, chan1, velToStep(motor1.absValue), 50);
        gpio_put(Motor1_dir_pin, motor1.sign);

        if (msg->velocity.size > 1) {
            SignAndAbs motor2 = getSignAndAbs(subscriber_msg.velocity.data[1]);
            uint slice_num2 = pwm_gpio_to_slice_num(Motor2_step_pin);
            uint chan2 = pwm_gpio_to_channel(Motor2_step_pin);
            pwm_set_freq_duty(slice_num2, chan2, velToStep(motor2.absValue), 50);
            gpio_put(Motor2_dir_pin, motor2.sign);
        
            if (msg->velocity.size > 2) {
                SignAndAbs motor3 = getSignAndAbs(subscriber_msg.velocity.data[2]);
                uint slice_num3 = pwm_gpio_to_slice_num(Motor3_step_pin);
                uint chan3 = pwm_gpio_to_channel(Motor3_step_pin);
                pwm_set_freq_duty(slice_num3, chan3, velToStep(motor3.absValue), 50);
                gpio_put(Motor3_dir_pin, motor3.sign);
            }
        }
    }


    message_received = true;
    gpio_put(LED_PIN, 1);

}

bool pingAgent() {
    const int timeout_ms = 100; // Timeout of 100ms
    const uint8_t attempts = 1; // Number of attempts

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts); // Ping the micro-ROS agent
    return (ret == RCL_RET_OK); // Return true if ping succeeded, false otherwise
}

void createEntities() {
    allocator = rcl_get_default_allocator(); // Get the default memory allocator

    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator); // Initialize the support
    CHECK_RET(ret); // Check and handle the return value

    ret = rclc_node_init_default(&node, node_name, "", &support); // Initialize the node
    CHECK_RET(ret); // Check and handle the return value

    ret = rclc_subscription_init_best_effort(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            subscriber_topic_name); // Initialize the subscriber
    CHECK_RET(ret); // Check and handle the return value
    
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    CHECK_RET(ret); // Check and handle the return value
    
    ret = rclc_executor_add_subscription(&executor,
			&subscriber,
			&subscriber_msg,
			&subscription_callback,
			ON_NEW_DATA);
    CHECK_RET(ret); // Check and handle the return value

    // Configurez les tailles spécifiques selon vos besoins
    conf.max_basic_type_sequence_capacity = 3; // Définir la capacité pour position, velocity, et effort
    conf.max_string_capacity = 50;              // Capacité maximale des chaînes de caractères

    micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        &subscriber_msg,
        conf
    );
    
}

void destroyEntities() {

    // Nettoyez la mémoire si nécessaire
    // free(subscriber_msg.position.data);
    // free(subscriber_msg.velocity.data);
    // free(subscriber_msg.effort.data);
    // free(subscriber_msg.name.data);

    micro_ros_utilities_destroy_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        &subscriber_msg,
        conf
    );

    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context); // Get the RMW context
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0); // Set the destruction timeout
    
    rcl_ret_t ret;

    ret = rcl_subscription_fini(&subscriber, &node); // Finalize the subscriber
    CHECK_RET(ret); // Check and handle the return value
    
    ret = rclc_executor_fini(&executor); // Finalize the executor
    CHECK_RET(ret); // Check and handle the return value
    
    ret = rcl_node_fini(&node); // Finalize the node
    CHECK_RET(ret); // Check and handle the return value

    ret = rclc_support_fini(&support); // Finalize the support
    CHECK_RET(ret); // Check and handle the return value
}

void handle_state_waiting_agent() {
    state = pingAgent() ? AGENT_AVAILABLE : WAITING_AGENT; // If ping successful, go to AGENT_AVAILABLE, otherwise stay in WAITING_AGENT
}

void handle_state_agent_available() {
    createEntities(); // Create ROS 2 entities
    state = AGENT_CONNECTED; // Go to AGENT_CONNECTED state
}

void handle_state_agent_connected() {
    if (pingAgent()) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // Execute ROS 2 executor to process messages every 100 millisecond
    } else {
        state = AGENT_DISCONNECTED; // If ping fails, go to AGENT_DISCONNECTED
    }
}

void handle_state_agent_disconnected() {
    destroyEntities(); // Destroy ROS 2 entities
    state = WAITING_AGENT; // Return to WAITING_AGENT state
}

void state_machine() {
    switch (state) {
        case WAITING_AGENT:
            handle_state_waiting_agent(); // Handle WAITING_AGENT state
            break;
        case AGENT_AVAILABLE:
            handle_state_agent_available(); // Handle AGENT_AVAILABLE state
            break;
        case AGENT_CONNECTED:
            handle_state_agent_connected(); // Handle AGENT_CONNECTED state
            break;
        case AGENT_DISCONNECTED:
            handle_state_agent_disconnected(); // Handle AGENT_DISCONNECTED state
            break;
        default:
            break;
    }
}

int main() {
    stdio_init_all(); // Initialize standard I/O

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    ); // Set the custom serial transport for micro-ROS

    gpio_init(LED_PIN); // Initialize the LED pin
    gpio_set_dir(LED_PIN, GPIO_OUT); // Set the LED pin direction to output
    

    gpio_init(Motor1_dir_pin);
    gpio_set_dir(Motor1_dir_pin, GPIO_OUT);
    gpio_init(Motor1_step_pin);
    gpio_set_dir(Motor1_step_pin, GPIO_OUT);
    gpio_set_function(Motor1_step_pin, GPIO_FUNC_PWM);

    gpio_init(Motor2_dir_pin);
    gpio_set_dir(Motor2_dir_pin, GPIO_OUT);
    gpio_init(Motor2_step_pin);
    gpio_set_dir(Motor2_step_pin, GPIO_OUT);
    gpio_set_function(Motor2_step_pin, GPIO_FUNC_PWM);

    gpio_init(Motor3_dir_pin);
    gpio_set_dir(Motor3_dir_pin, GPIO_OUT);
    gpio_init(Motor3_step_pin);
    gpio_set_dir(Motor3_step_pin, GPIO_OUT);
    gpio_set_function(Motor3_step_pin, GPIO_FUNC_PWM);

    allocator = rcl_get_default_allocator(); // Get the default memory allocator
    state = WAITING_AGENT; // Initialize the state to WAITING_AGENT

    while (true) {
        state_machine(); // Handle the state machine

        if (message_received) {
            message_received = false; // Reset the flag
        } else {
            gpio_put(LED_PIN, 0); // Turn off the LED if no new message was received
        }
    }
    
    return 0; // End of the program
}
