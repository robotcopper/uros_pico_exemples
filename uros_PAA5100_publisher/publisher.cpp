
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

#include "pico_uart_transports.h"
}

#include <cmath>
#include "pimoroni-pico/libraries/breakout_paa5100/breakout_paa5100.hpp"
#include "pico/multicore.h"

//cs        = SPI_BG_FRONT_CS  = GPIO17;
//sck       = SPI_DEFAULT_SCK  = GPIO18;
//mosi      = SPI_DEFAULT_MOSI = GPIO19;
//miso      = SPI_DEFAULT_MISO = GPIO16;
//interrupt = SPI_BG_FRONT_PWM = GPIO20;

typedef pimoroni::BreakoutPAA5100 FlowSensor;

FlowSensor flo(pimoroni::BG_SPI_FRONT);

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

rcl_publisher_t publisher;
nav_msgs__msg__Odometry publisher_msg;

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
}

void destroyEntities(){
	rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
	(void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

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
    	      break;
    	    default:
    	      break;
    	  }
    	  
    	if (message_send) {
            message_send = false; // Réinitialiser l'indicateur
        } else {
            gpio_put(LED_PIN, 0); // Éteindre la LED si aucun nouveau message n'a été reçu
        }
        
    } 
    return 0;
}
