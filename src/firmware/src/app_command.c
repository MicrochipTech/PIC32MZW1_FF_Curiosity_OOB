/*******************************************************************************
  Sample Application

  File Name:
    app_commands.c

  Summary:
    commands for the tcp client demo app.

  Description:
    
 *******************************************************************************/


// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2021 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#include "tcpip/tcpip.h"
#include "app_commands.h"
#include "app.h"
#include "config.h"
#include <wolfssl/ssl.h>
#include "task.h"
#include "wolfcrypt/error-crypt.h"
#include "cryptoauthlib.h"
#include "wdrv_pic32mzw_common.h"
#include "wdrv_pic32mzw_assoc.h"

#define ENABLE_DTMFG_TEST

#if defined(TCPIP_STACK_COMMAND_ENABLE)

static void _APP_Commands_GetUnixTime(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_GetRSSI(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_GetRTCC(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_ControlTouch(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
#ifdef ENABLE_DTMFG_TEST
static void _GPIO_loopback_test(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
bool GPIO_loopback_test(char port_in, uint8_t pin_in, char port_out, uint8_t pin_out);
bool GPIO_PIN_INP_Enable(char port, uint8_t pin);
bool GPIO_PIN_OUT_Enable(char port, uint8_t pin);
uint8_t GPIO_PIN_Set(char port, uint8_t pin, uint8_t value);
int8_t GPIO_PIN_Get(char port, uint8_t pin);
#endif

static const SYS_CMD_DESCRIPTOR appCmdTbl[] = {
    {"unixtime", _APP_Commands_GetUnixTime, ": Unix Time"},
    {"rssi", _APP_Commands_GetRSSI, ": Get current RSSI"},
    {"rtcc", _APP_Commands_GetRTCC, ": Get uptime"},
    {"touch", _APP_Commands_ControlTouch, ": Control Touch"},
    #ifdef ENABLE_DTMFG_TEST 
    {"loopback", _GPIO_loopback_test, ": GPIO Loopback Test"},
    #endif
};

#ifdef ENABLE_DTMFG_TEST 
/* This API(Command Loopback) is used only for DTMFG test*/

static void _GPIO_loopback_test
(
    SYS_CMD_DEVICE_NODE* pCmdIO, 
    int argc, 
    char** argv
)
{

//    const void* cmdIoParam = pCmdIO->cmdIoParam;
//    (*pCmdIO->pCmdApi->print)(cmdIoParam, "\tGPIO_loopback_test\r\n");
    
    
    char port_in = argv[1][0];
    uint8_t pin_in = atoi(argv[2]);
    char port_out = argv[3][0];
    uint8_t pin_out = atoi(argv[4]);

    GPIO_loopback_test(port_in,pin_in,port_out,pin_out);
}

bool GPIO_PIN_INP_Enable(char port, uint8_t pin)
{
    switch(port)
    {
        case 'A':
        case 'a':
            ANSELACLR |= (1U << pin);
            while((ANSELACLR & (1U << pin)) == (1U << pin));
            TRISASET |= (1U << pin); //input enable
            while((TRISASET & (1U << pin)) == (1U << pin));
            break;
        case 'B':
        case 'b':
            
            ANSELBCLR |= (1U << pin);
            while((ANSELBCLR & (1U << pin)) == (1U << pin));
            TRISBSET |= (1U << pin); //input enable
            while((TRISBSET & (1U << pin)) == (1U << pin));
            break;
        case 'C':
        case 'c':
            TRISCSET |= (1U << pin); //input enable
            while((TRISCSET & (1U << pin)) == (1U << pin));
            break;
        case 'K':
        case 'k':
            TRISKSET |= (1U << pin); //input enable
            while((TRISKSET & (1U << pin)) == (1U << pin));
            break;
        default :
            return false;
    }
    return true;
}



bool GPIO_PIN_OUT_Enable(char port, uint8_t pin)
{
    switch(port)
    {
        case 'A':
        case 'a':
            ANSELACLR |= (1U << pin);
            while((ANSELACLR & (1U << pin)) == (1U << pin));
            TRISACLR |= (1U << pin); //input enable
            while((TRISACLR & (1U << pin)) == (1U << pin));
            break;
        case 'B':
        case 'b':
            ANSELBCLR |= (1U << pin);
            while((ANSELBCLR & (1U << pin)) == (1U << pin));
            TRISBCLR |= (1U << pin); //input enable
            while((TRISBCLR & (1U << pin)) == (1U << pin));
            break;
        case 'C':
        case 'c':
            TRISCCLR |= (1U << pin); //input enable
            while((TRISCCLR & (1U << pin)) == (1U << pin));
            break;
        case 'K':
        case 'k':
            TRISKCLR |= (1U << pin); //input enable
            while((TRISKCLR & (1U << pin)) == (1U << pin));
            break;
        default :
            return false;
    }
    return true;
}

int8_t GPIO_PIN_Get(char port, uint8_t pin)
{
    switch(port)
    {
        case 'A':
        case 'a':
            return ((PORTA >> pin) & 0x1U);
        case 'B':
        case 'b':
            return ((PORTB >> pin) & 0x1U);
        case 'C':
        case 'c':
            return ((PORTC >> pin) & 0x1U);
        case 'K':
        case 'k':
            return ((PORTK >> pin) & 0x1U);
        default : 
            return -1;
    }
}


uint8_t GPIO_PIN_Set(char port, uint8_t pin, uint8_t value)
{
    switch(port)
    {
        case 'A':
        case 'a':
            if(value == 1){
                LATASET |= (1U << pin); //input enable
                while((LATASET & (1U << pin)) == (1U << pin));
                break;
            }
            LATACLR |= (1U << pin);
            while((LATACLR & (1U << pin)) == (1U << pin));
            break;
        case 'B':
        case 'b':
            if(value == 1){
                LATBSET |= (1U << pin); //input enable
                while((LATBSET & (1U << pin)) == (1U << pin));
                break;
            }
            LATBCLR |= (1U << pin);
            while((LATBCLR & (1U << pin)) == (1U << pin));
            break;
        case 'C':
        case 'c':
            if(value == 1){
                LATCSET |= (1U << pin); //input enable
                while((LATCSET & (1U << pin)) == (1U << pin));
                break;
            }
            LATCCLR |= (1U << pin);
            while((LATCCLR & (1U << pin)) == (1U << pin));
            break;
        case 'K':
        case 'k':
            if(value == 1){
                LATKSET |= (1U << pin); //input enable
                while((LATKSET & (1U << pin)) == (1U << pin));
                break;
            }
            LATKCLR |= (1U << pin);
            while((LATKCLR & (1U << pin)) == (1U << pin));
            break;
        default :
            return false;
    }
    return true;
}
bool GPIO_loopback_test(char port_in, uint8_t pin_in, char port_out, uint8_t pin_out)
{
    bool ret = false;

    /* Diabling Pheripherals for DTMFG test*/
    I2C1CONCLR = 0x00009000;
    I2C1CONSET = 0x00001000;

    /* Disabling SPI1*/
    SPI1CON = 0; 
    
    /*Disabling OC2*/
    OC2CONCLR = _OC2CON_ON_MASK;

    if(GPIO_PIN_INP_Enable(port_in, pin_in))
    {
        if(GPIO_PIN_OUT_Enable(port_out, pin_out))
        {
            if(GPIO_PIN_Set(port_out, pin_out, 0))
            {
                if(GPIO_PIN_Get(port_in, pin_in) == 0)
                {
                    if(GPIO_PIN_Set(port_out, pin_out, 1))
                    {
                        if(GPIO_PIN_Get(port_in, pin_in) == 1)
                        {
                            if(GPIO_PIN_Set(port_out, pin_out, 0))
                            {
                                if(GPIO_PIN_Get(port_in, pin_in) == 0)
                                {
                                    SYS_CONSOLE_PRINT(TERM_GREEN"PASSED pair : %c%d - %c%d\r\n"TERM_RESET,port_in, pin_in, port_out, pin_out);
                                    ret=true;
                                }
                                else
                                {
                                    SYS_CONSOLE_PRINT(TERM_RED"ERROR!! Output pin %c%d : Cleared , Input pin %c%d : SET\r\n"TERM_RESET,port_out, pin_out,port_in, pin_in);
                                }
                            }
                            else
                            {
                                SYS_CONSOLE_PRINT(TERM_RED"ERROR!! Output pin %c%d : unable to CLEAR\r\n"TERM_RESET,port_out, pin_out);
                            }
                        }
                        else
                        {
                            SYS_CONSOLE_PRINT(TERM_RED"ERROR!! Output pin %c%d : SET , Input pin %c%d : CLEARED"TERM_RESET,port_out, pin_out,port_in, pin_in);
                        }
                    }
                    else
                    {
                        SYS_CONSOLE_PRINT(TERM_RED"ERROR!! Output pin %c%d : unable to SET\r\n"TERM_RESET,port_out, pin_out);
                    }
                }
                else
                {
                    SYS_CONSOLE_PRINT(TERM_RED"ERROR!! Output pin %c%d : Cleared , Input pin %c%d : SET\r\n"TERM_RESET,port_out, pin_out,port_in, pin_in);
                }
            }
            else
            {
                SYS_CONSOLE_PRINT(TERM_RED"ERROR!! Output pin %c%d : unable to CLEAR\r\n"TERM_RESET,port_in, pin_in);
            }
        }
        else
        {
            SYS_CONSOLE_PRINT(TERM_RED"ERROR!! Unable to set Output pin \r\n"TERM_RESET,port_in, pin_in);
        }
            
    }
    else
    {
        SYS_CONSOLE_PRINT(TERM_RED"ERROR!! Unable to set Output pin \r\n"TERM_RESET,port_in, pin_in);
    }
    return ret;
}
#endif    

bool APP_Commands_Init() {
    if (!SYS_CMD_ADDGRP(appCmdTbl, sizeof (appCmdTbl) / sizeof (*appCmdTbl), "app", ": app commands")) {
        SYS_ERROR(SYS_ERROR_ERROR, "Failed to create TCPIP Commands\r\n", 0);
        return false;
    }

    return true;
}

static void _AssociationRSSICallback(DRV_HANDLE handle, WDRV_PIC32MZW_ASSOC_HANDLE assocHandle, int8_t rssi) {
    SYS_CONSOLE_PRINT(TERM_YELLOW "Connected RSSI: %d \r\n" TERM_RESET, rssi);
}

void _APP_Commands_GetRSSI(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    if (app_controlData.wifiCtrl.wifiConnected) {
        WDRV_PIC32MZW_STATUS ret;
        ret = WDRV_PIC32MZW_AssocRSSIGet(app_controlData.rssiData.assocHandle, NULL, _AssociationRSSICallback);
        if (ret != WDRV_PIC32MZW_STATUS_RETRY_REQUEST) {
            (*pCmdIO->pCmdApi->print)(cmdIoParam, TERM_RED "Failed getting RSSI. Driver error (%d)\r\n" TERM_RESET, ret);
        }
    } else {
        (*pCmdIO->pCmdApi->print)(cmdIoParam, TERM_RED "RSSI: WI-Fi not connected.\r\n" TERM_RESET);
    }
}

void _APP_Commands_GetRTCC(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    struct tm *sys_time = &app_controlData.rtccData.sys_time;
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "RTCC: "TERM_YELLOW" %d-%d-%d %d:%d:%d\r\n"TERM_RESET, sys_time->tm_mday, sys_time->tm_mon, sys_time->tm_year, sys_time->tm_hour, sys_time->tm_min, sys_time->tm_sec);
}

void _APP_Commands_GetUnixTime(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    uint32_t sec = TCPIP_SNTP_UTCSecondsGet();
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "Time from SNTP: %d\r\n", sec);
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "Low Rez Timer: %d\r\n", SYS_TIME_CounterGet() /
            SYS_TIME_FrequencyGet());

}

void _APP_Commands_ControlTouch(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    const void* cmdIoParam = pCmdIO->cmdIoParam; 
    if ((argc >= 1) && (!strcmp(argv[1], "start"))) {
        g_enableTouch = 1;
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "Touch:  Enabled\r\n");
    }
    else if ((argc >= 1) && (!strcmp(argv[1], "stop"))){ 
        g_enableTouch = 0;
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "Touch:  Disabled\r\n");
    }
   else {
        
       (*pCmdIO->pCmdApi->print)(cmdIoParam, "Touch:  Invalid arguments\r\n");
   
   }
}
#endif
