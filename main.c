
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "usrcmd.h"

#include "bluetoothManager.h"
#include "cycfg_bt_settings.h"
#include "bt_platform_cfg_settings.h"

volatile int uxTopUsedPriority ;
TaskHandle_t blinkTaskHandle;


void blink_task(void *arg)
{
    cyhal_gpio_init(CYBSP_USER_LED,CYHAL_GPIO_DIR_OUTPUT,CYHAL_GPIO_DRIVE_STRONG,0);

    for(;;)
    {
    	cyhal_gpio_toggle(CYBSP_USER_LED);
    	vTaskDelay(500);
    }
}


int main(void)
{
    uxTopUsedPriority = configMAX_PRIORITIES - 1 ; // enable OpenOCD Thread Debugging

    /* Initialize the device and board peripherals */
    cybsp_init() ;
    __enable_irq();

    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);



    cybt_platform_config_init(&bt_platform_cfg_settings);
    wiced_bt_stack_init (app_bt_management_callback, &wiced_bt_cfg_settings);

    // Stack size in WORDs
    // Idle task = priority 0
    xTaskCreate(blink_task, "blinkTask", configMINIMAL_STACK_SIZE,0 /* args */ ,0 /* priority */, &blinkTaskHandle);
    xTaskCreate(usrcmd_task, "usrcmd_task", configMINIMAL_STACK_SIZE*4,0 /* args */ ,0 /* priority */, 0);
    vTaskStartScheduler();
}

/* [] END OF FILE */
