/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

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
/*******************************************************************************
Copyright (C) 2020 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system/net/sys_net.h"
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
SYS_MODULE_OBJ g_tcpServHandle = SYS_MODULE_OBJ_INVALID;
SYS_NET_Config g_tcpServCfg;
extern APP_TOUCH_DATA app_TouchData;

uint8_t recv_buffer[100], data_buf[50],count=0;;
uint32_t g_Xpos=0,g_Ypos=0;
bool b_sendCoords;

#define TOUCH_CORDS    "%u,%u;" 

   
void sendCoords()
{ 
    int32_t ret = 0,retry=0;
    if ((true == b_sendCoords) &&  (1 == app_TouchData.touchActive))
    {
        size_t nChars;
        if (g_Xpos != app_TouchData.xpos || g_Ypos != app_TouchData.ypos)
        {
            g_Xpos = app_TouchData.xpos;
            g_Ypos = app_TouchData.ypos;
            nChars = sprintf((char *)data_buf,TOUCH_CORDS,app_TouchData.xpos,app_TouchData.ypos);

                if (SYS_NET_GetStatus (g_tcpServHandle) == SYS_NET_STATUS_CONNECTED)
                {
                   
                    do{                    
                        ret = SYS_NET_SendMsg(g_tcpServHandle,(uint8_t *)data_buf , nChars); 
                       // SYS_CONSOLE_PRINT("sendCoords(): %d bytes sent to client\r\n", ret);
                        retry++;
                    }while((ret<=0) && (retry<10));
                }
            
        }
        else
        {
            // Do not send the same co-ords again 
        }
        
    }
    else
    {
        //Do not send 
    }
}
void TcpServCallback(uint32_t event, void *data, void* cookie)
{
    switch(event)
    {
        case SYS_NET_EVNT_CONNECTED:
        {
            SYS_CONSOLE_PRINT("TcpServCallback(): Status UP\r\n");
            b_sendCoords = true;
            break;
        }

        case SYS_NET_EVNT_DISCONNECTED:
        {
            SYS_CONSOLE_PRINT("TcpServCallback(): Status DOWN\r\n");
            b_sendCoords = false;
            break;
        }

        case SYS_NET_EVNT_RCVD_DATA:
        {
			int32_t cumm_len = 0;
            int32_t len = 100;
            while(len == 100)
            {
                len = SYS_NET_RecvMsg(g_tcpServHandle, recv_buffer, 100);
                if(len>0)
                {
                    if(cumm_len == 0)
                    {
                        uint8_t buffer[33];
                        int32_t tmp_len = (len > 32)? 32 : len;

                        memcpy(buffer, recv_buffer, tmp_len);
                        buffer[tmp_len] = '\0';

                    }
                    cumm_len += len;
                }
            }
            break;
        }
        
        case SYS_NET_EVNT_LL_INTF_DOWN:
        {
            /* 
            ** User needs to take a decision if they want to close the socket or
            ** wait for the Lower layer to come up
             */
            SYS_CONSOLE_PRINT("TcpServCallback(): Lower Layer Down\r\n");
            b_sendCoords = false;
            break;
        }
        
        case SYS_NET_EVNT_LL_INTF_UP:
        {
            /* 
            ** The lower layer was down and now it has come up again. 
            ** The socket was in connected state all this while
             */
            SYS_CONSOLE_PRINT("TcpServCallback(): Lower Layer Up\r\n");
            break;
        }                
        
        case SYS_NET_EVNT_SERVER_AWAITING_CONNECTION:
        {
            /* 
            ** The server is awaiting connection from the client
             */
            SYS_CONSOLE_PRINT("TcpServCallback(): Server Awaiting Connection\r\n");
            break;
        }                
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Sock_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_Sock_Initialize ( void )
{
    SYS_CONSOLE_MESSAGE("APP_Initialize\n");
    b_sendCoords = false;
    g_tcpServCfg.mode = SYS_NET_INDEX1_MODE;
    g_tcpServCfg.port = SYS_NET_INDEX1_PORT;
    g_tcpServCfg.enable_tls = SYS_NET_INDEX1_ENABLE_TLS;
    g_tcpServCfg.ip_prot = SYS_NET_INDEX1_IPPROT;
    g_tcpServCfg.enable_reconnect = SYS_NET_INDEX1_RECONNECT;

    
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Sock_Tasks ( void )
{
    SYS_NET_Task(g_tcpServHandle);
    sendCoords();
}


/*******************************************************************************
 End of File
 */
