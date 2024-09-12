/*
Developed by T.S
S Academy
*/

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#include <geometry_msgs/msg/twist.h>

#define LED_PIN 2   //13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\



rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor_pub;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;
geometry_msgs__msg__Twist msg;
geometry_msgs__msg__Twist msg_cmd;


bool micro_ros_init_successful;

volatile int counterL_forward = 0;
volatile int counterR_forward = 0;
volatile int counterL_backward = 0;
volatile int counterR_backward = 0;

long time_cal = 0;
long time_cal2 = 0;

long EncoderVal[2] = {0,0};
double Vels[4] = {0,0,0,0};

float WheelDiameter = 0.067;      //set to your wheel of robot
int TPR = 210;    


enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;



void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

    //motor_write();
    
    msg.linear.x = 0;
    msg.linear.y = 0;     
  
    
    rcl_publish(&publisher, &msg, NULL);
    
  }
}



// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "motor_control", "", &support));


   //RCCHECK(rclc_publisher_init_best_effort(
   RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "motor_vel"));    

  // create timer,
 /* const unsigned int timer_timeout = 25;  //1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));  */

    
  // create executor
  executor_pub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));


  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
 
  rcl_timer_fini(&timer); 
  rclc_executor_fini(&executor_pub);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  rcl_publisher_fini(&publisher, &node);
 
}


void IRAM_ATTR ISR1() {    
    counterL_forward++;
}

void IRAM_ATTR ISR2() {    
    counterL_backward++;
}

void IRAM_ATTR ISR3() {    
    counterR_forward++;
}

void IRAM_ATTR ISR4() {    
    counterR_backward++;
}


void speed_cal_forward(){
  if(time_cal == 0) {
    time_cal = millis();
    return;
  }

    EncoderVal[0] = counterL_forward;
    counterL_forward = 0;

    EncoderVal[1] = counterR_forward;
    counterR_forward = 0;

  long aTime = millis();
  int DTime = aTime-time_cal;
  time_cal = aTime;

  //calculate short term measured velocity
  Vels[0] = (TicksToMeters(EncoderVal[0])/DTime)*1000;
  Vels[1] = (TicksToMeters(EncoderVal[1])/DTime)*1000;
}

void speed_cal_backward(){
    if(time_cal2 == 0) {
    time_cal2 = millis();
    return;
  }

    EncoderVal[0] = counterL_backward;
    counterL_backward = 0;

    EncoderVal[1] = counterR_backward;
    counterR_backward = 0;

  long aTime = millis();
  int DTime = aTime-time_cal2;
  time_cal2 = aTime;

  Vels[2] = (TicksToMeters(EncoderVal[0])/DTime)*1000;
  Vels[3] = (TicksToMeters(EncoderVal[1])/DTime)*1000;
  }

double TicksToMeters(int Ticks){
    return (Ticks*3.14*WheelDiameter)/TPR;
}


void setup() {
  set_microros_transports();
  /*pinMode(LED_PIN, OUTPUT);
  pinMode(13,OUTPUT);  //DIR1
  pinMode(12,OUTPUT);  //PWM1
  pinMode(14,OUTPUT);  //DIR2
  pinMode(27,OUTPUT);  //PWM2
   */
  state = WAITING_AGENT;

   attachInterrupt(2, ISR1, FALLING);  //L_A  forward
   attachInterrupt(5, ISR2, FALLING);  //L_B  back    
   
   attachInterrupt(4, ISR3, FALLING);  //R_B  forward
   attachInterrupt(16, ISR4, FALLING);  //R_A   back
}


void loop() {
  
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      //EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      //EXECUTE_EVERY_N_MS(10, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      //if (state == AGENT_CONNECTED) {
      //  rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)); //RCL_MS_TO_NS(100));
        //rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(1)); //RCL_MS_TO_NS(100));          
        //motor_write();      
      //}
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  
   delay(20);
   speed_cal_forward();
   speed_cal_backward(); 
   
      msg.linear.x = Vels[0];
      msg.linear.y = Vels[1];
      msg.angular.x = Vels[2];
      msg.angular.y = Vels[3];
      
   rcl_publish(&publisher, &msg, NULL);
  
}
