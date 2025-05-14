#include <stdio.h>
#include "pico/stdlib.h" // Include the standard library for Raspberry Pi Pico

extern "C" {
#include <rcl/rcl.h> // Main ROS 2 client library
#include <rcl/error_handling.h> // Error handling for ROS 2
#include <rclc/rclc.h> // C library for ROS 2
#include <rclc/executor.h> // Executor for ROS 2
#include <rmw_microros/rmw_microros.h> // Middleware for micro-ROS

#include "nav_msgs/msg/odometry.h"

#include "pico_uart_transports.h" // UART transport specific for Pico
}

#include <QwiicOtos.h>


QwiicOTOS myOtos;
char str[100];

constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN; // Define the LED pin number

rcl_publisher_t publisher; // Declare the ROS 2 publisher
nav_msgs__msg__Odometry publisher_msg; // Declare the ROS 2 message

bool message_send = false; // Flag for message sending

const char * publisher_topic_name = "pico_publisher_topic";
const char * node_name = "pico_node";
const int frec = 100; //publication frequency in Hz

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

struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};

Quaternion rad_to_quaternion(double angle_in_radians) {
    Quaternion q;
    
    // Rotate around the Z axis
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(angle_in_radians / 2.0);
    q.w = cos(angle_in_radians / 2.0);
    
    return q;
}

void publisher_content(rcl_timer_t *timer, int64_t last_call_time) {

    publisher_msg.header.stamp.sec = (uint16_t)(rmw_uros_epoch_millis()/1000);
    publisher_msg.header.stamp.nanosec = (uint32_t)rmw_uros_epoch_nanos();

    sfe_otos_pose2d_t pos;
    myOtos.getPosition(pos);
    sfe_otos_pose2d_t vel;
    myOtos.getVelocity(vel);
    sfe_otos_pose2d_t posStdDev;
    myOtos.getPositionStdDev(posStdDev);
    sfe_otos_pose2d_t velStdDev;
    myOtos.getVelocityStdDev(velStdDev);

    publisher_msg.pose.pose.position.x= pos.x;
    publisher_msg.pose.pose.position.y= pos.y;
    Quaternion quaternion = rad_to_quaternion(pos.h);
    publisher_msg.pose.pose.orientation.x = quaternion.x;
    publisher_msg.pose.pose.orientation.y = quaternion.y;
    publisher_msg.pose.pose.orientation.z = quaternion.z;
    publisher_msg.pose.pose.orientation.w = quaternion.w;

    double pose_covariance[36] = {
        posStdDev.x * posStdDev.x, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, posStdDev.y * posStdDev.y, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, posStdDev.h * posStdDev.h,
    };
    for (int i = 0; i < 36; ++i) {
        publisher_msg.pose.covariance[i] = pose_covariance[i];
    }

    publisher_msg.twist.twist.linear.x = vel.x;
    publisher_msg.twist.twist.linear.y = vel.y;
    publisher_msg.twist.twist.linear.z = 0.0;
    publisher_msg.twist.twist.angular.x = 0.0;
    publisher_msg.twist.twist.angular.y = 0.0;
    publisher_msg.twist.twist.angular.z = vel.h;

    double twist_covariance[36] = {
        velStdDev.x * velStdDev.x, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, velStdDev.y * velStdDev.y, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, velStdDev.h * velStdDev.h,
    };
    for (int i = 0; i < 36; ++i) {
        publisher_msg.twist.covariance[i] = twist_covariance[i];
    }

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
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
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

    initI2C(false);

    // Calibrate the IMU, which removes the accelerometer and gyroscope offsets
    myOtos.calibrateImu();
    // Set the desired units meters for linear, and radians for angular.
    myOtos.setLinearUnit(kSfeOtosLinearUnitMeters);
    myOtos.setAngularUnit(kSfeOtosAngularUnitRadians);
    // Reset the tracking algorithm - this resets the position to the origin,
    // but can also be used to recover from some rare tracking errors
    myOtos.resetTracking();

    //initialisation of message constants
    rosidl_runtime_c__String frame_id_str;
    char frame_id[100] = "odom_PAA5160";
    frame_id_str.data = frame_id;
    frame_id_str.size = strlen(frame_id_str.data);
    frame_id_str.capacity = frame_id_str.size + 1;
    publisher_msg.header.frame_id = frame_id_str;
    rosidl_runtime_c__String child_frame_id_str;
    char child_frame_id[100] = "PAA5160";
    child_frame_id_str.data = child_frame_id;
    child_frame_id_str.size = strlen(frame_id_str.data);
    child_frame_id_str.capacity = child_frame_id_str.size + 1;
    publisher_msg.child_frame_id = child_frame_id_str;

    publisher_msg.pose.pose.position.z = 0.019; // sensor placed 19 mm from the ground


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
