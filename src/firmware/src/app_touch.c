/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_touch.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app_touch.h"
#include "app_sock_data.h"
#include "system/console/sys_console.h"
#include "led_driver.h"
#include "touch/touch_api_ptc.h"
#include "system/console/sys_console.h"
#include "system/time/sys_time.h"
#include "peripheral/wdt/plib_wdt.h"
#include "peripheral/adchs/plib_adchs.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_TOUCH_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_TOUCH_DATA app_TouchData;
bool g_enableTouch=0;
extern volatile uint8_t measurement_done_touch;
extern volatile uint8_t h_pos;
extern volatile uint8_t v_pos;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_TOUCH_Initialize ( void )

  Remarks:
    See prototype in app_touch.h.
 */

void APP_TOUCH_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_TouchData.state = APP_TOUCH_STATE_INIT;
    APP_Sock_Initialize();
    ADCHS_ModulesDisable(ADCHS_MODULE7_MASK);
    app_TouchData.xpos = 0;
    app_TouchData.ypos = 255;
    app_TouchData.clr =1;
    app_TouchData.touchActive=0;


    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_TOUCH_Tasks ( void )

  Remarks:
    See prototype in app_touch.h.
 */

void APP_TOUCH_Tasks ( void ) {
    switch (app_TouchData.state) {
        case APP_TOUCH_STATE_INIT:
        {
            init_led_driver();
            app_TouchData.state = APP_TOUCH_STATE_SERVICE_TASKS;

            break;
        }
        case APP_TOUCH_STATE_SERVICE_TASKS:
        {
            if (g_enableTouch==1)
            {
                ADCHS_ModulesEnable(ADCHS_MODULE7_MASK);
                g_tcpServHandle = SYS_NET_Open(&g_tcpServCfg, TcpServCallback, 0); 
                if(g_tcpServHandle != SYS_MODULE_OBJ_INVALID)
                    SYS_CONSOLE_PRINT("TCP Server socket opened\r\n");
                else
                    SYS_CONSOLE_PRINT("TCP socket open request failed\r\n");
                app_TouchData.state = APP_TOUCH_STATE_PROCESS_TOUCH;
                break;
            }
        }
        case APP_TOUCH_STATE_PROCESS_TOUCH:
        {
            if (g_enableTouch==0)
            {
                ADCHS_ModulesDisable(ADCHS_MODULE7_MASK);
                if(g_tcpServHandle != SYS_MODULE_OBJ_INVALID)
                {
                    SYS_NET_Close(g_tcpServHandle);
                    g_tcpServHandle = SYS_MODULE_OBJ_INVALID;
                    SYS_CONSOLE_PRINT("TCP socket closed\r\n");
                }
                app_TouchData.state = APP_TOUCH_STATE_SERVICE_TASKS;
                break;
            }
            else
            {
                touch_process();
                if (measurement_done_touch == 1u)
                {
                    measurement_done_touch = 0u;
                    led_decode_position();
                    break;
                }
                APP_Sock_Tasks();
                break;
            }
        }
        default:
        {
            break;
        }

    }
}

/*******************************************************************************
 End of File
 */
