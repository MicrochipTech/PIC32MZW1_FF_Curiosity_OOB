/*******************************************************************************
 System Tasks File

  File Name:
    tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled tasks.

  Description:
    This file contains source code necessary to maintain system's polled tasks.
    It implements the "SYS_Tasks" function that calls the individual "Tasks"
    functions for all polled MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "configuration.h"
#include "definitions.h"


// *****************************************************************************
// *****************************************************************************
// Section: RTOS "Tasks" Routine
// *****************************************************************************
// *****************************************************************************
/* Handle for the APP_Tasks. */
TaskHandle_t xAPP_Tasks;

void _APP_Tasks(  void *pvParameters  )
{   
    while(1)
    {
        APP_Tasks();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
/* Handle for the APP_WIFI_Tasks. */
TaskHandle_t xAPP_WIFI_Tasks;

void _APP_WIFI_Tasks(  void *pvParameters  )
{   
    while(1)
    {
        APP_WIFI_Tasks();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
/* Handle for the MSD_APP_Tasks. */
TaskHandle_t xMSD_APP_Tasks;

void _MSD_APP_Tasks(  void *pvParameters  )
{   
    while(1)
    {
        MSD_APP_Tasks();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
/* Handle for the APP_CONTROL_Tasks. */
TaskHandle_t xAPP_CONTROL_Tasks;

void _APP_CONTROL_Tasks(  void *pvParameters  )
{   
    while(1)
    {
        APP_CONTROL_Tasks();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
/* Handle for the MQTT_APP_Tasks. */
TaskHandle_t xMQTT_APP_Tasks;

void _MQTT_APP_Tasks(  void *pvParameters  )
{   
    while(1)
    {
        MQTT_APP_Tasks();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
/* Handle for the APP_TOUCH_Tasks. */
TaskHandle_t xAPP_TOUCH_Tasks;

void _APP_TOUCH_Tasks(  void *pvParameters  )
{   
    while(1)
    {
        APP_TOUCH_Tasks();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void _NET_PRES_Tasks(  void *pvParameters  )
{
    while(1)
    {
        NET_PRES_Tasks(sysObj.netPres);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


void _SYS_FS_Tasks(  void *pvParameters  )
{
    while(1)
    {
        SYS_FS_Tasks();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}



void _DRV_BA414E_Tasks(  void *pvParameters  )
{
    while(1)
    {
        DRV_BA414E_Tasks(sysObj.ba414e);
    }
}

void _USB_DEVICE_Tasks(  void *pvParameters  )
{
    while(1)
    {
				 /* USB Device layer tasks routine */
        USB_DEVICE_Tasks(sysObj.usbDevObject0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void _DRV_MEMORY_0_Tasks(  void *pvParameters  )
{
    while(1)
    {
        DRV_MEMORY_Tasks(sysObj.drvMemory0);
        vTaskDelay(DRV_MEMORY_RTOS_DELAY_IDX0 / portTICK_PERIOD_MS);
    }
}

void _DRV_USBFS_Tasks(  void *pvParameters  )
{
    while(1)
    {
				 /* USB FS Driver Task Routine */
        DRV_USBFS_Tasks(sysObj.drvUSBFSObject);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void _TCPIP_STACK_Task(  void *pvParameters  )
{
    while(1)
    {
        TCPIP_STACK_Task(sysObj.tcpip);
        vTaskDelay(4 / portTICK_PERIOD_MS);
    }
}

void _SYS_CMD_Tasks(  void *pvParameters  )
{
    while(1)
    {
        SYS_CMD_Tasks();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


static void _WDRV_PIC32MZW1_Tasks(  void *pvParameters  )
{
    while(1)
    {
        SYS_STATUS status;

        WDRV_PIC32MZW_Tasks(sysObj.drvWifiPIC32MZW1);

        status = WDRV_PIC32MZW_Status(sysObj.drvWifiPIC32MZW1);

        if ((SYS_STATUS_ERROR == status) || (SYS_STATUS_UNINITIALIZED == status))
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
}

void _SYS_WIFI_Task(  void *pvParameters  )
{
    while(1)
    {
        SYS_WIFI_Tasks(sysObj.syswifi);
        vTaskDelay(4 / portTICK_PERIOD_MS);
    }
}




// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/
void SYS_Tasks ( void )
{
    /* Maintain system services */
    

    xTaskCreate( _SYS_FS_Tasks,
        "SYS_FS_TASKS",
        SYS_FS_STACK_SIZE,
        (void*)NULL,
        SYS_FS_PRIORITY,
        (TaskHandle_t*)NULL
    );


    xTaskCreate( _SYS_CMD_Tasks,
        "SYS_CMD_TASKS",
        SYS_CMD_RTOS_STACK_SIZE,
        (void*)NULL,
        SYS_CMD_RTOS_TASK_PRIORITY,
        (TaskHandle_t*)NULL
    );




    /* Maintain Device Drivers */
        xTaskCreate( _DRV_MEMORY_0_Tasks,
        "DRV_MEM_0_TASKS",
        DRV_MEMORY_STACK_SIZE_IDX0,
        (void*)NULL,
        DRV_MEMORY_PRIORITY_IDX0,
        (TaskHandle_t*)NULL
    );

    xTaskCreate( _WDRV_PIC32MZW1_Tasks,
        "WDRV_PIC32MZW1_Tasks",
        1024,
        (void*)NULL,
        1,
        (TaskHandle_t*)NULL
    );




    /* Maintain Middleware & Other Libraries */
    
    xTaskCreate( _NET_PRES_Tasks,
        "NET_PRES_Tasks",
        NET_PRES_RTOS_STACK_SIZE,
        (void*)NULL,
        NET_PRES_RTOS_TASK_PRIORITY,
        (TaskHandle_t*)NULL
    );



    xTaskCreate( _DRV_BA414E_Tasks,
        "DRV_BA414E_Tasks",
        DRV_BA414E_RTOS_STACK_SIZE,
        (void*)NULL,
        DRV_BA414E_RTOS_TASK_PRIORITY,
        (TaskHandle_t*)NULL
    );


    /* Create OS Thread for USB_DEVICE_Tasks. */
    xTaskCreate( _USB_DEVICE_Tasks,
        "USB_DEVICE_TASKS",
        1024,
        (void*)NULL,
        1,
        (TaskHandle_t*)NULL
    );



    xTaskCreate( _TCPIP_STACK_Task,
        "TCPIP_STACK_Tasks",
        TCPIP_RTOS_STACK_SIZE,
        (void*)NULL,
        TCPIP_RTOS_PRIORITY,
        (TaskHandle_t*)NULL
    );


    xTaskCreate( _SYS_WIFI_Task,
        "SYS_WIFI_Tasks",
        SYS_WIFI_RTOS_SIZE,
        (void*)NULL,
        SYS_WIFI_RTOS_PRIORITY,
        (TaskHandle_t*)NULL
    );




    /* Maintain the application's state machine. */
        /* Create OS Thread for APP_Tasks. */
    xTaskCreate((TaskFunction_t) _APP_Tasks,
                "APP_Tasks",
                256,
                NULL,
                1,
                &xAPP_Tasks);

    /* Create OS Thread for APP_WIFI_Tasks. */
    xTaskCreate((TaskFunction_t) _APP_WIFI_Tasks,
                "APP_WIFI_Tasks",
                1024,
                NULL,
                1,
                &xAPP_WIFI_Tasks);

    /* Create OS Thread for MSD_APP_Tasks. */
    xTaskCreate((TaskFunction_t) _MSD_APP_Tasks,
                "MSD_APP_Tasks",
                2048,
                NULL,
                1,
                &xMSD_APP_Tasks);

    /* Create OS Thread for APP_CONTROL_Tasks. */
    xTaskCreate((TaskFunction_t) _APP_CONTROL_Tasks,
                "APP_CONTROL_Tasks",
                512,
                NULL,
                1,
                &xAPP_CONTROL_Tasks);

    /* Create OS Thread for MQTT_APP_Tasks. */
    xTaskCreate((TaskFunction_t) _MQTT_APP_Tasks,
                "MQTT_APP_Tasks",
                1024,
                NULL,
                1,
                &xMQTT_APP_Tasks);

    /* Create OS Thread for APP_TOUCH_Tasks. */
    xTaskCreate((TaskFunction_t) _APP_TOUCH_Tasks,
                "APP_TOUCH_Tasks",
                1024,
                NULL,
                1,
                &xAPP_TOUCH_Tasks);




    /* Start RTOS Scheduler. */
    
     /**********************************************************************
     * Create all Threads for APP Tasks before starting FreeRTOS Scheduler *
     ***********************************************************************/
    vTaskStartScheduler(); /* This function never returns. */

}

/*******************************************************************************
 End of File
 */

