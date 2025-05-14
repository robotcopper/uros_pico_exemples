
#include <stdio.h>
#include "pico/stdlib.h"

#include <time.h>
extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "nav_msgs/msg/odometry.h"
#include "robot_msg/msg/motor_msgs.h"

#include "pico_uart_transports.h"
}

#include <cmath>
#include "pimoroni-pico/libraries/breakout_paa5100/breakout_paa5100.hpp"
#include "pico/multicore.h"
#include "hardware/pwm.h"

//cs        = SPI_BG_FRONT_CS  = GPIO17;
//sck       = SPI_DEFAULT_SCK  = GPIO18;
//mosi      = SPI_DEFAULT_MOSI = GPIO19;
//miso      = SPI_DEFAULT_MISO = GPIO16;
//interrupt = SPI_BG_FRONT_PWM = GPIO20;

typedef pimoroni::BreakoutPAA5100 FlowSensor;

FlowSensor flo(pimoroni::BG_SPI_FRONT);

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d);

volatile bool message_received = false;

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

typedef struct {
    int sign;       // 0 pour négatif, 1 pour positif
    int absValue;   // Valeur absolue du nombre
} SignAndAbs;

const uint Motor1_step_pin = 0; // num du GPI0 pour les step
const uint Motor1_dir_pin = 1; // num du GPI0 pour la dir
const uint Motor2_step_pin = 2;
const uint Motor2_dir_pin = 3;
const uint Motor3_step_pin = 4;
const uint Motor3_dir_pin = 5;

SignAndAbs getSignAndAbs(int number);

rcl_publisher_t publisher;
nav_msgs__msg__Odometry publisher_msg;
rcl_subscription_t subscriber;
robot_msg__msg__MotorMsgs subscriber_msg;

struct timespec ts;
extern int clock_gettime(clockid_t unused, struct timespec *tp);

bool message_send = false;

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

const double FOV_DEG = 42.0;
const int RES_PIX = 35;
const int scaler = 5;
const double timer_periode = 0.02; //periode à la quelle le capteur est appelé. Estimation basé sur le temps entre deux publication
const double fov = FOV_DEG*(M_PI / 180.0);
const double z = 0.025; // capteur à 2.5cm du sol
const double cf = z*tan(fov/2)/(RES_PIX*scaler);

// Structure partagée pour la communication entre les cœurs
struct SharedData {
    volatile int16_t dx;
    volatile int16_t dy;
    volatile int32_t dist_x;
    volatile int32_t dist_y;
    volatile int32_t v_x;
    volatile int32_t v_y;
    volatile bool newDataAvailable;
};
SharedData sharedData = {0, 0, 0, 0, false};

uint64_t last_call_time = 0;
uint64_t last_dist_x = 0;
uint64_t last_dist_y = 0;

void sensor_reading_task() {
    while (true) {
    
        uint64_t current_time = time_us_64();
        
        double time_interval = (current_time - last_call_time) / 1e6; // Conversion en secondes
        last_call_time = current_time;
        
        int16_t dx, dy;
        flo.get_motion(dx, dy);
        
        double dist_x = -1 * cf * dy;
        double dist_y = cf * dx;
        
        double dist_interval_x = (dist_x - last_dist_x);
        double dist_interval_y = (dist_y - last_dist_y);
        last_dist_x = dist_x;
        last_dist_y = dist_y;
        
        double v_x = dist_interval_x/time_interval;
        double v_y = dist_interval_y/time_interval;

        // Mise à jour sécurisée des données partagées
        __atomic_store_n(&sharedData.dx, dx, __ATOMIC_SEQ_CST);
        __atomic_store_n(&sharedData.dy, dy, __ATOMIC_SEQ_CST);
	int32_t int_dist_x = static_cast<int32_t>(dist_x * 10000);
	int32_t int_dist_y = static_cast<int32_t>(dist_y * 10000);
	__atomic_store_n(&sharedData.dist_x, int_dist_x, __ATOMIC_SEQ_CST);
	__atomic_store_n(&sharedData.dist_y, int_dist_y, __ATOMIC_SEQ_CST);
	int32_t int_v_x = static_cast<int32_t>(v_x * 10000);
	int32_t int_v_y = static_cast<int32_t>(v_y * 10000);
	__atomic_store_n(&sharedData.v_x, int_v_x, __ATOMIC_SEQ_CST);
	__atomic_store_n(&sharedData.v_y, int_v_y, __ATOMIC_SEQ_CST);
        __atomic_store_n(&sharedData.newDataAvailable, true, __ATOMIC_SEQ_CST);

        //sleep_ms(20); // Temps de sommeil correspondant à timer_periode
    }
}

void subscription_callback(const void * msgin){
    const robot_msg__msg__MotorMsgs * msg = (const robot_msg__msg__MotorMsgs *)msgin;
    
    gpio_set_function(Motor1_step_pin, GPIO_FUNC_PWM);
    uint slice_num1 = pwm_gpio_to_slice_num(Motor1_step_pin); // PWM slice
    uint chan1 = pwm_gpio_to_channel(Motor1_step_pin); // PWM channel
    pwm_set_freq_duty(slice_num1, chan1, getSignAndAbs(msg->motor1).absValue, 50);    // This function sets the PWM frequency and duty cycle for a given PWM slice and channel.
    gpio_put(Motor1_dir_pin, getSignAndAbs(msg->motor1).sign);
    
    gpio_set_function(Motor2_step_pin, GPIO_FUNC_PWM);
    uint slice_num2 = pwm_gpio_to_slice_num(Motor2_step_pin);
    uint chan2 = pwm_gpio_to_channel(Motor2_step_pin);
    pwm_set_freq_duty(slice_num2, chan2, getSignAndAbs(msg->motor2).absValue, 50);
    gpio_put(Motor2_dir_pin, getSignAndAbs(msg->motor2).sign);
    
    gpio_set_function(Motor3_step_pin, GPIO_FUNC_PWM);
    uint slice_num3 = pwm_gpio_to_slice_num(Motor3_step_pin);
    uint chan3 = pwm_gpio_to_channel(Motor3_step_pin);
    pwm_set_freq_duty(slice_num3, chan3, getSignAndAbs(msg->motor3).absValue, 50);
    gpio_put(Motor3_dir_pin, getSignAndAbs(msg->motor3).sign);
    
    message_received = true;
    gpio_put(LED_PIN, 1);
}

void publisher_content(int16_t& dx, int16_t& dy, double& dist_x, double& dist_y, double& v_x, double& v_y, geometry_msgs__msg__Point& point, geometry_msgs__msg__PoseWithCovariance& PWC, geometry_msgs__msg__TwistWithCovariance& TC){

	clock_gettime(CLOCK_REALTIME, &ts);
	
	publisher_msg.header.stamp.sec = ts.tv_sec;
	publisher_msg.header.stamp.nanosec = ts.tv_nsec;

	//flo.get_motion(dx, dy);
	
	//dist_x = -1*cf*dy;
	//dist_y = cf*dx;
	
	TC.twist.linear.x = v_x;
	TC.twist.linear.y = v_y;
	
	publisher_msg.twist = TC;
	
	point.x+=dist_x;
	point.y+=dist_y;
	
	PWC.pose.position = point;
	publisher_msg.pose = PWC;
	
	rcl_ret_t ret = rcl_publish(&publisher, &publisher_msg, NULL);
	
	message_send = true;
	gpio_put(LED_PIN, 1);
}

bool pingAgent(){
    const int timeout_ms = 10; //attend un ping pendant 10ms
    const uint8_t attempts = 3; //teste jusqu'a 3 fois

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK){
    	return false;
    }
    return true;
}

void createEntities(){
	allocator = rcl_get_default_allocator();

	rclc_support_init(&support, 0, NULL, &allocator);

	rclc_node_init_default(&node, "pico_node", "", &support);

	rclc_publisher_init_default(
			&publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
			"pico_publisher_topic");

	const rosidl_message_type_support_t * type_support =
	    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);
	    	
	const rosidl_message_type_support_t * type_support2 =
	    ROSIDL_GET_MSG_TYPE_SUPPORT(robot_msg, msg, MotorMsgs);
	
	rclc_subscription_init_default(
	        &subscriber,
	        &node,
	        type_support2,
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
	rcl_publisher_fini(&publisher, &node);
	rcl_node_fini(&node);
	rclc_support_fini(&support);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
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
    
    flo.init();
    flo.set_rotation(FlowSensor::DEGREES_0);
    double dist_x = 0.0;
    double dist_y = 0.0;
    int16_t dx = 0;
    int16_t dy = 0;
    
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
    
    rosidl_runtime_c__String parent_frame_id;
    parent_frame_id.data = "odom";
    parent_frame_id.size = strlen(parent_frame_id.data);
    parent_frame_id.capacity = parent_frame_id.size + 1;

    publisher_msg.header.frame_id = parent_frame_id;
    
    rosidl_runtime_c__String childframe_id;
    childframe_id.data = "base_link";
    childframe_id.size = strlen(childframe_id.data);
    childframe_id.capacity = childframe_id.size + 1;

    publisher_msg.child_frame_id = childframe_id;
    
    geometry_msgs__msg__Point point;
    point.x=0.0;
    point.y=0.0;
    point.z= z;
    
    geometry_msgs__msg__Quaternion quaternion;
    quaternion.x=0.0;
    quaternion.y=0.0;
    quaternion.z=0.0;
    quaternion.w=1.0;
    
    geometry_msgs__msg__PoseWithCovariance PWC;
    PWC.pose.position = point;
    PWC.pose.orientation = quaternion;
    
    double position_covariance[36] = {
    0.0004, 0, 0, 0, 0, 0,
    0, 0.0004, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0.0004, 0, 0,
    0, 0, 0, 0, 0.0004, 0,
    0, 0, 0, 0, 0, 0.0004
    };

    for (int i = 0; i < 36; ++i) {
    	PWC.covariance[i] = position_covariance[i];
    }//cofiance sur la position à 1cm près (dans les conditions de vitesse normale

    
    publisher_msg.pose = PWC;
    
    geometry_msgs__msg__TwistWithCovariance TC;
    TC.twist.linear.x = 0.0;
    TC.twist.linear.y = 0.0;
    TC.twist.linear.z = 0.0;
    TC.twist.angular.x = 0.0;
    TC.twist.angular.y = 0.0;
    TC.twist.angular.z = 0.0;
    
    double velocity_covariance[36] = {
    0.0025, 0, 0, 0, 0, 0,
    0, 0.0025, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0.0025, 0, 0,
    0, 0, 0, 0, 0.0025, 0,
    0, 0, 0, 0, 0, 0.0025
    };
    
    for (int i = 0; i < 36; ++i) {
    	TC.covariance[i] = velocity_covariance[i];
    }//cofiance sur la vitesse à 5cm/s près
    
    publisher_msg.twist = TC;
    
    
    allocator = rcl_get_default_allocator();
    state = WAITING_AGENT;
    
    multicore_launch_core1(sensor_reading_task);

    while (true){
    	switch (state) {
    	    case WAITING_AGENT:
    	      state = pingAgent() ? AGENT_AVAILABLE : WAITING_AGENT;
    	      break;
    	    case AGENT_AVAILABLE:
    	      createEntities();
    	      state = AGENT_CONNECTED ;
    	      break;
    	    case AGENT_CONNECTED:
    	      state = pingAgent() ? AGENT_CONNECTED : AGENT_DISCONNECTED;
    	      if (state == AGENT_CONNECTED) {
    	        //publisher_content(dx, dy, dist_x, dist_y, point, PWC, TC);
    	        
    	        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		if (__atomic_load_n(&sharedData.newDataAvailable, __ATOMIC_SEQ_CST)) {
		    // Lecture des données du capteur
		    int16_t dx = __atomic_load_n(&sharedData.dx, __ATOMIC_SEQ_CST);
		    int16_t dy = __atomic_load_n(&sharedData.dy, __ATOMIC_SEQ_CST);
		    int32_t int_dist_x = __atomic_load_n(&sharedData.dist_x, __ATOMIC_SEQ_CST);
    		    int32_t int_dist_y = __atomic_load_n(&sharedData.dist_y, __ATOMIC_SEQ_CST);
		    double dist_x = static_cast<double>(int_dist_x) / 10000.0;
		    double dist_y = static_cast<double>(int_dist_y) / 10000.0;
		    int32_t int_v_x = __atomic_load_n(&sharedData.v_x, __ATOMIC_SEQ_CST);
    		    int32_t int_v_y = __atomic_load_n(&sharedData.v_y, __ATOMIC_SEQ_CST);
		    double v_x = static_cast<double>(int_v_x) / 10000.0;
		    double v_y = static_cast<double>(int_v_y) / 10000.0;

		    // Réinitialisation du drapeau
		    __atomic_store_n(&sharedData.newDataAvailable, false, __ATOMIC_SEQ_CST);

		    // Publication des données ROS
		    publisher_content(dx, dy, dist_x, dist_y, v_x, v_y, point, PWC, TC);
		}
    	      }
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
            //gpio_put(LED_PIN, 0); // Éteindre la LED si aucun nouveau message n'a été reçu
            
            //éteind par défaut le moteur
            gpio_set_function(Motor1_step_pin, GPIO_FUNC_PWM);
	    uint slice_num1 = pwm_gpio_to_slice_num(Motor1_step_pin);
    	    uint chan1 = pwm_gpio_to_channel(Motor1_step_pin);
    	    pwm_set_freq_duty(slice_num1, chan1, 0, 50);
    	    
    	    gpio_set_function(Motor2_step_pin, GPIO_FUNC_PWM);
	    uint slice_num2 = pwm_gpio_to_slice_num(Motor2_step_pin);
    	    uint chan2 = pwm_gpio_to_channel(Motor2_step_pin);
    	    pwm_set_freq_duty(slice_num2, chan2, 0, 50);
    	    
    	    gpio_set_function(Motor3_step_pin, GPIO_FUNC_PWM);
	    uint slice_num3 = pwm_gpio_to_slice_num(Motor3_step_pin);
    	    uint chan3 = pwm_gpio_to_channel(Motor3_step_pin);
    	    pwm_set_freq_duty(slice_num3, chan3, 0, 50);
        }
        
    	if (message_send) {
            message_send = false; // Réinitialiser l'indicateur
        } else {
            gpio_put(LED_PIN, 0); // Éteindre la LED si aucun nouveau message n'a été reçu
        }
        
    } 
    return 0;
}
