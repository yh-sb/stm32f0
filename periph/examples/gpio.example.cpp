// Example for STM32F072DISCOVERY development board

#include "periph/gpio.hpp"
#include "periph/systick.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace periph;

static void heartbeat_task(void *pvParameters)
{
    gpio *green_led = (gpio *)pvParameters;
    while(1)
    {
        green_led->toggle();
        vTaskDelay(500);
    }
}

int main(void)
{
    systick::init();
    static gpio green_led(2, 9, gpio::mode::DO, 0);
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE,
        &green_led, 1, nullptr);
    
    vTaskStartScheduler();
}
