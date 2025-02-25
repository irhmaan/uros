#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <std_srvs/srv/set_bool.h>
#include <std_msgs/msg/string.h>
#include <rclc/executor.h>
#include <driver/adc_common.h>
#include <rosidl_runtime_c/string_functions.h>
#include <esp32-hal-ledc.h>
#include <sys/param.h>

#define LED_PIN GPIO_NUM_2
#define LEDC_TIMER_12_BIT 12
#define BASE_FREQ 5000

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_service_t led_srv;
rcl_service_t fade_led_srv;

std_srvs__srv__SetBool_Request req;
std_srvs__srv__SetBool_Response res;

std_srvs__srv__SetBool_Request fade_req;
std_srvs__srv__SetBool_Response fade_res;

bool isFadeEnabled = false;

void configure_led()
{
  if (isFadeEnabled)
  {
    ledcSetup(0, BASE_FREQ, LEDC_TIMER_12_BIT);
    ledcAttachPin(LED_PIN, 0);
  }
  else
  {

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << LED_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
  }
}

void ledCAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255)
{
  uint32_t duty = (4095 / valueMax) * MIN(value, valueMax);

  // write duty to ledc
  ledcWrite(channel, duty);
}

void fadeLedTask(void *pvParams)
{
  static int brightness = 0;
  int fadeAmount = 5;

  while (isFadeEnabled)
  {
    ledCAnalogWrite(0, brightness);
    // change the brightness for next time through the loop:
    brightness += fadeAmount;

    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 255)
    {
      fadeAmount = -fadeAmount;
    }
    // wait for 30 milliseconds to see the dimming effect
    vTaskDelay(pdMS_TO_TICKS(30));
  }
  vTaskDelete(NULL);
}

void fadeLed(const void *fade_req, void *fade_res)
{
  int brightness = 0;
  int fadeAmount = 5;

  std_srvs__srv__SetBool_Request *request = (std_srvs__srv__SetBool_Request *)fade_req;
  std_srvs__srv__SetBool_Response *response = (std_srvs__srv__SetBool_Response *)fade_res;

  isFadeEnabled = request->data;

  if (isFadeEnabled)
  {
    xTaskCreate(fadeLedTask, "fadeLedTask", 2048, NULL, 1, NULL);
    response->success = true;
    rosidl_runtime_c__String__assign(&response->message, "Led Fading Enabled");
  }
  else
  {
    response->success = false;
    rosidl_runtime_c__String__assign(&response->message, "Led Fading Disabled");
  }
}

void led_service_callback(const void *req_msg, void *res_msg)
{
  //? Take fours args F, B, L, R
  //? Move the bot as per given direction.
  //? Also try to add, some speed control in the node.

  std_srvs__srv__SetBool_Request *request = (std_srvs__srv__SetBool_Request *)req_msg;
  std_srvs__srv__SetBool_Response *response = (std_srvs__srv__SetBool_Response *)res_msg;

  gpio_set_level(LED_PIN, request->data ? 1 : 0);

  rosidl_runtime_c__String__assign(&response->message, request->data ? "LED is ON" : "LED is OFF");
}

void app_main()
{
  configure_led();

  // Initialize ROS 2
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "led_control", "", &support);

  // Create service
  rclc_service_init_default(
      &led_srv,
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
      "switch_led");

  // create fade led service
  rclc_service_init_default(
      &fade_led_srv,
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
      "fade_led");

  // Spin
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_service(&executor, &led_srv, &req, &res, led_service_callback);
  rclc_executor_add_service(&executor, &fade_led_srv, &fade_req, &fade_res, fadeLed);

  while (true)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
