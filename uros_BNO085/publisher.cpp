#include <stdio.h>
#include "pico/stdlib.h" // Include the standard library for Raspberry Pi Pico

extern "C" {
#include <rcl/rcl.h> // Main ROS 2 client library
#include <rcl/error_handling.h> // Error handling for ROS 2
#include <rclc/rclc.h> // C library for ROS 2
#include <rclc/executor.h> // Executor for ROS 2
#include <rmw_microros/rmw_microros.h> // Middleware for micro-ROS

#include "sensor_msgs/msg/imu.h"

#include "pico_uart_transports.h" // UART transport specific for Pico
}

#include <utils.h>
#include <bno08x.h>


constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN; // Define the LED pin number

rcl_publisher_t publisher; // Declare the ROS 2 publisher
sensor_msgs__msg__Imu publisher_msg; // Declare the ROS 2 message

bool message_send = false; // Flag for message sending

const char * publisher_topic_name = "pico_publisher_topic";
const char * node_name = "pico_node";
const int frec = 100; //publication frequency in Hz

char quaternion_str[100];
BNO08x IMU;

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
rcl_timer_t timer; // Declare the ROS 2 timer
rclc_executor_t executor; // Declare the ROS 2 executor

#define CHECK_RET(ret) if (ret != RCL_RET_OK) { rcl_reset_error(); } // Macro for silent error handling

// void getLatestHeading(float& i, float& j, float& k, float& real, float& radAccuracy, uint8_t& accuracy) 
void getLatestHeading(float& x,float& y,float& z,float& w,float& RadAccuracy,float& AngularVel_x,float& AngularVel_y,float& AngularVel_z,float& LinAccel_x,float& LinAccel_y,float& LinAccel_z){
    // If possible, update the heading with the latest from the IMU
    if (IMU.getSensorEvent() == true) {
        if (IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
            x = IMU.getQuatI();
            y = IMU.getQuatJ();
            z = IMU.getQuatK();
            w = IMU.getQuatReal();
            RadAccuracy = IMU.getQuatRadianAccuracy();
            AngularVel_x = IMU.getGyroX();
            AngularVel_y = IMU.getGyroY();
            AngularVel_z = IMU.getGyroZ();
            LinAccel_x = IMU.getLinAccelX();
            LinAccel_y = IMU.getLinAccelY();
            LinAccel_z = IMU.getLinAccelZ();
        }
    }
}

void publisher_content(rcl_timer_t *timer, int64_t last_call_time) {

    publisher_msg.header.stamp.sec = (uint16_t)(rmw_uros_epoch_millis()/1000);
    publisher_msg.header.stamp.nanosec = (uint32_t)rmw_uros_epoch_nanos();

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 0.0f;
    float RadAccuracy = 0.0f;
    float AngularVel_x = 0.0f;
    float AngularVel_y = 0.0f;
    float AngularVel_z = 0.0f;
    float LinAccel_x = 0.0f;
    float LinAccel_y = 0.0f;
    float LinAccel_z = 0.0f;
    getLatestHeading(x,y,z,w,RadAccuracy,AngularVel_x,AngularVel_y,AngularVel_z,LinAccel_x,LinAccel_y,LinAccel_z);

    publisher_msg.orientation.x = x;
    publisher_msg.orientation.y = y;
    publisher_msg.orientation.z = z;
    publisher_msg.orientation.w = w;
    double orientation_covariance[9] = {
        RadAccuracy * RadAccuracy, 0.0, 0.0,
        0.0, RadAccuracy * RadAccuracy, 0.0,
        0.0, 0.0, RadAccuracy * RadAccuracy
    };
    for (int i = 0; i < 9; ++i) {
        publisher_msg.orientation_covariance[i] = orientation_covariance[i];
    }
    publisher_msg.angular_velocity.x = AngularVel_x;
    publisher_msg.angular_velocity.y = AngularVel_y;
    publisher_msg.angular_velocity.z = AngularVel_z;
    publisher_msg.linear_acceleration.x = LinAccel_x;
    publisher_msg.linear_acceleration.y = LinAccel_y;
    publisher_msg.linear_acceleration.z = LinAccel_z;

    rcl_ret_t ret = rcl_publish(&publisher, &publisher_msg, NULL); 
    CHECK_RET(ret); 

    message_send = true; // Set the flag indicating the message was sent
    gpio_put(LED_PIN, 1); // Turn on the LED
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

    ret = rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            publisher_topic_name); // Initialize the publisher
    CHECK_RET(ret); // Check and handle the return value
    
    int period_ms = 1000 / frec;
    rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(period_ms),
		publisher_content);
    CHECK_RET(ret); // Check and handle the return value
	    
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    CHECK_RET(ret); // Check and handle the return value
    ret = rclc_executor_add_timer(&executor, &timer);
    CHECK_RET(ret); // Check and handle the return value
}

void destroyEntities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context); // Get the RMW context
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0); // Set the destruction timeout
    
    rcl_ret_t ret;

    ret = rcl_publisher_fini(&publisher, &node); // Finalize the publisher
    CHECK_RET(ret); // Check and handle the return value

    ret = rcl_node_fini(&node); // Finalize the node
    CHECK_RET(ret); // Check and handle the return value

    ret = rclc_support_fini(&support); // Finalize the support
    CHECK_RET(ret); // Check and handle the return value
    
    ret = rcl_timer_fini(&timer); // Finalize the timer
    CHECK_RET(ret); // Check and handle the return value
    
    ret = rclc_executor_fini(&executor); // Finalize the executor
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
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // Send content if connected
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


    i2c_inst_t* i2c_port0;
    initI2C(i2c_port0, false);
    //set up IMU
    while (IMU.begin(CONFIG::BNO08X_ADDR, i2c_port0)==false) {
        printf("BNO08x not detected at default I2C address. Check wiring. Freezing\n");
        scan_i2c_bus();
        sleep_ms(1000);
    }
    IMU.enableRotationVector();

    //initialisation of message constants
    rosidl_runtime_c__String str;
    char frame_id[100] = "bno085_imu";
    str.data = frame_id;
    str.size = strlen(str.data);
    str.capacity = str.size + 1;
    publisher_msg.header.frame_id = str;
    float AngularVelAccuracy = 0.054105; // 3.1°/s Gyro datasheet nominal accuracy in rad/s
    double angular_velocity_covariance[9] = {
        AngularVelAccuracy * AngularVelAccuracy, 0.0, 0.0,
        0.0, AngularVelAccuracy * AngularVelAccuracy, 0.0,
        0.0, 0.0, AngularVelAccuracy * AngularVelAccuracy
    };
    for (int i = 0; i < 9; ++i) {
        publisher_msg.angular_velocity_covariance[i] = angular_velocity_covariance[i];
    }
    float LinAccelAccuracy = 0.35; // 0.35 m/s^2 datasheet nominal accuracy
    double linear_acceleration_covariance[9] = {
        LinAccelAccuracy * LinAccelAccuracy, 0.0, 0.0,
        0.0, LinAccelAccuracy * LinAccelAccuracy, 0.0,
        0.0, 0.0, LinAccelAccuracy * LinAccelAccuracy
    };
    for (int i = 0; i < 9; ++i) {
        publisher_msg.linear_acceleration_covariance[i] = linear_acceleration_covariance[i];
    }


    allocator = rcl_get_default_allocator(); // Get the default memory allocator
    state = WAITING_AGENT; // Initialize the state to WAITING_AGENT

    while (true) {
        state_machine(); // Handle the state machine

        if (message_send) {
            message_send = false; // Reset the flag
        } else {
            gpio_put(LED_PIN, 0); // Turn off the LED if no new message was sent
        }
    }
    
    return 0; // End of the program
}
