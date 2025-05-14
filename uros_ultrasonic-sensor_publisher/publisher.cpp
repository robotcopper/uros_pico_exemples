
#include <stdio.h>
#include "pico/stdlib.h"

#include <time.h>
extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/time.h>
#include <rmw_microros/rmw_microros.h>

#include "robot_msg/msg/ultrasonic_msgs.h"
#include "sensor_msgs/msg/range.h"
#include "sensor_msgs/msg/imu.h"

#include "pico_uart_transports.h"
}

#include "HCSR04Range.h"

#include "pico/multicore.h"
#include "hardware/sync.h"

#include <array>
#include <algorithm>

#define NUM_SENSORS 4
#define TRIG	0
#define NORTH  	2
#define EAST  	3
#define SOUTH  	4
#define WEST  	5

HCSR04Range HCSR04ranges(TRIG, NORTH);
//sensor_msgs__msg__range UltraSound_sensor_0;

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

struct timespec ts;
extern int clock_gettime(clockid_t unused, struct timespec *tp);

constexpr size_t WINDOW_SIZE = 3.5;  // Taille de la fenêtre pour le filtrage
std::array<float, WINDOW_SIZE> readings = {};  // Tableau pour stocker les dernières lectures
size_t readingIndex = 0;  // Index pour le tableau des lectures
float getFilteredReading(float newReading);
float getMedianFilteredReading(float newReading);
float getCombinedFilteredReading(float newReading);

rcl_publisher_t publisher;
//robot_msg__msg__UltrasonicMsgs publisher_msg;
sensor_msgs__msg__Range publisher_msg;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

volatile enum states shared_state; // Variable partagée entre les cœurs

rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;

void publisher_content(){
	
       	HCSR04ranges.trigger();
	//sleep_ms(100);
	
	clock_gettime(CLOCK_REALTIME, &ts);

	//UltraSound_sensor_0.header.stamp.sec = ts.tv_sec;
    	//UltraSound_sensor_0.header.stamp.nanosec = ts.tv_nsec;
	publisher_msg.header.stamp.sec = ts.tv_sec;
	publisher_msg.header.stamp.nanosec = ts.tv_nsec;
	
        //UltraSound_sensor_0.range =10;//HCSR04ranges.getDistanceMM(0);
        
        float distanceInMeters = HCSR04ranges.getDistanceMM(0) / (float)100000;
        
	if (distanceInMeters>= 0.02 && distanceInMeters <= 4.0){
        	publisher_msg.range = distanceInMeters;//getFilteredReading(distanceInMeters);
       	        rcl_ret_t ret = rcl_publish(&publisher, &publisher_msg, NULL);
	}
	//publisher_msg.range = getFilteredReading(distanceInMeters);
	//publisher_msg.range = getMedianFilteredReading(distanceInMeters);
	//publisher_msg.range = getCombinedFilteredReading(distanceInMeters);
        //rcl_ret_t ret = rcl_publish(&publisher, &publisher_msg, NULL);
        
       	/*publisher_msg.range = distanceInMeters;
        rcl_ret_t ret = rcl_publish(&publisher, &publisher_msg, NULL);*/
        //rcl_ret_t ret = rcl_publish(&publisher, &publisher_msg, NULL);
        //publisher_msg.ranges.data[0] = UltraSound_sensor_0;


	//sleep_ms(200);
}

bool pingAgent(){
    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 100;
    const uint8_t attempts = 1;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK){
    	gpio_put(LED_PIN, 0);
    	return false;
    } else {
    	gpio_put(LED_PIN, 1);
    }
    return true;
}

void createEntities(){
	allocator = rcl_get_default_allocator();

	rclc_support_init(&support, 0, NULL, &allocator);

	rclc_node_init_default(&node, "pico_node", "", &support);

	rclc_publisher_init_best_effort(
			&publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
			"pico_publisher_topic");

	const rosidl_message_type_support_t * type_support =
	    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range);
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

void sensor_reading_task() {
    while (true) {
        enum states current_state = __atomic_load_n(&shared_state, __ATOMIC_SEQ_CST); // Lecture atomique de l'état
        if (current_state == AGENT_CONNECTED) {
            publisher_content();
        }
        sleep_ms(200);
    }
}

float getFilteredReading(float newReading) {
    // Ajouter la nouvelle lecture dans le tableau
    readings[readingIndex] = newReading;
    readingIndex = (readingIndex + 1) % WINDOW_SIZE;

    // Calculer la moyenne des lectures dans la fenêtre
    float sum = 0.0;
    for (float reading : readings) {
        sum += reading;
    }
    return sum / WINDOW_SIZE;
}

float getMedianFilteredReading(float newReading) {
    readings[readingIndex] = newReading;
    readingIndex = (readingIndex + 1) % WINDOW_SIZE;

    std::array<float, WINDOW_SIZE> sortedReadings = readings;
    std::sort(sortedReadings.begin(), sortedReadings.end());

    return sortedReadings[WINDOW_SIZE / 2];
}

float getMedianOfReadings() {
    std::array<float, WINDOW_SIZE> sortedReadings = readings;
    std::sort(sortedReadings.begin(), sortedReadings.end());
    return sortedReadings[WINDOW_SIZE / 2];
}

float getCombinedFilteredReading(float newReading) {
    readings[readingIndex] = newReading;
    readingIndex = (readingIndex + 1) % WINDOW_SIZE;

    // Obtenez la médiane pour supprimer les valeurs aberrantes
    float median = getMedianOfReadings();

    // Calcul de la moyenne pour lisser la sortie
    float sum = 0.0;
    for (float reading : readings) {
        sum += reading;
    }
    float average = sum / WINDOW_SIZE;

    // Utilisez une combinaison de médiane et de moyenne
    return (median + average) / 2.0;
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
    //gpio_put(LED_PIN, pulse);

    //gpio_init(LED_PIN);
    //gpio_set_dir(LED_PIN, GPIO_OUT);
    //gpio_put(LED_PIN, 1);
    //sleep_ms(500);
    //gpio_put(LED_PIN, 0);
    
    /*for(int i=0; i < NUM_SENSORS; i++){
        sensor_msgs__msg__Range UltraSound_sensor_{i};
    }*/
    
    //UltraSound_sensor_0.radiation_type = 0; // ULTRASOUND radiation_type
    publisher_msg.radiation_type = 0; 
    
    rosidl_runtime_c__String str;
    str.data = "UltraSound_sensor_1";
    str.size = strlen(str.data);
    str.capacity = str.size + 1;
    //UltraSound_sensor_0.header.frame_id = str;
    publisher_msg.header.frame_id = str;
    
    //UltraSound_sensor_0.header.frame_id = String(UltraSound_sensor_1);
    
    /*UltraSound_sensor_0.field_of_view = 0.26; //Angle de mesure efficace 15 ° = 0.26rad
    UltraSound_sensor_0.min_range = 0.02;  // distance minimal détectable 2cm = 0.02m
    UltraSound_sensor_0.max_range = 4.0;   // distance maximal détectable 4m*/
    publisher_msg.field_of_view = 0.26;
    publisher_msg.min_range = 0.02;
    publisher_msg.max_range = 4.0;
    
    //publisher_msg.ranges.push_back(UltraSound_sensor_0);

    allocator = rcl_get_default_allocator();
    shared_state = WAITING_AGENT;
    
    HCSR04ranges.addEcho(EAST);
    HCSR04ranges.addEcho(SOUTH);
    HCSR04ranges.addEcho(WEST);
    
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
    	      /*if (state == AGENT_CONNECTED) {
    	        publisher_content();
    	      }*/
    	      break;
    	    case AGENT_DISCONNECTED:
    	      destroyEntities();
    	      state = WAITING_AGENT;
    	      break;
    	    default:
    	      break;
    	}
        __atomic_store_n(&shared_state, state, __ATOMIC_SEQ_CST);
        //pulse = ! pulse;
        //gpio_put(LED_PIN, pulse);
       sleep_ms(200);
    }
    return 0;
}
