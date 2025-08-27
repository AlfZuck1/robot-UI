#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>

// Wifi 
char ssid[] = "Totalplay-2.4G-E310";
char password[] = "t9sX9mGm5fE9fGmJ";
char agent_ip[] = "192.168.101.119"; 
uint16_t agent_port = 8888;

// RCL variables
rcl_subscription_t sub;
rcl_node_t node;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;
std_msgs__msg__Float64 msg;
float effort = 0.0;

// Error handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define LED_PIN 13

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(300);
  }
}

// Callback to receive
void effort_callback(const void *msgin) {
  const std_msgs__msg__Float64 *m = (const std_msgs__msg__Float64 *)msgin;
  effort = (float)m->data;
  int pwm = (int)(fabs(effort) * 255);
  if (pwm > 255) pwm = 255;
  Serial.print("Effort received: ");
  Serial.print(effort);
  Serial.print(" -> PWM: ");
  Serial.println(pwm);
}

void setup() {
  Serial.begin(115200);

  // Test wifi connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wifi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");


  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
        &sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "/joint6_controller/commands"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub, &msg, &effort_callback, ON_NEW_DATA));
  Serial.println("Subscriber ready");
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
  delay(50);
}
