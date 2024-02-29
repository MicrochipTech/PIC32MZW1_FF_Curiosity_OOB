/* ************************************************************************** */
/** led_driver.c

  @Description
    Drives QT8 LEDs based on x/y touch position output by Qtouch modular library.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "definitions.h"
#include "led_driver.h"


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

extern qtm_surface_cs2t_control_t qtm_surface_cs_control1;

static uint8_t i2c_write_buf[3];

uint8_t h_pos = 0u;
uint8_t v_pos = 0u;

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

void led_gpio_update(uint8_t data, uint8_t ver_or_hor){
    while(I2C1_IsBusy()){};
    if (ver_or_hor == LED_HOR) {
        i2c_write_buf[0] = LED_HOR_REG_ADDR; // reg address
        i2c_write_buf[1] = data;
        I2C1_Write(LED_DRIVER_ADDR, i2c_write_buf, 2);
    } else {
        i2c_write_buf[0] = LED_VER_REG_ADDR; // reg address
        i2c_write_buf[1] = data;
        I2C1_Write(LED_DRIVER_ADDR, i2c_write_buf, 2);
    }
}

void led_reset(void){
    led_gpio_update(0u, LED_HOR);
    led_gpio_update(0u, LED_VER);
}

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */
uint32_t i = 0;

void init_led_driver(void){  
    i2c_write_buf[0] = LED_VER_DIR_ADDR;                        // reg direction
    i2c_write_buf[1] = 0x00;                                    // output
    while(I2C1_IsBusy()){};
    I2C1_Write(LED_DRIVER_ADDR, i2c_write_buf, 2);
    while(I2C1_IsBusy()){};
    I2C1_Write(LED_DRIVER_ADDR, i2c_write_buf, 2);

    i2c_write_buf[0] = LED_VER_REG_ADDR;                        // reg address
    i2c_write_buf[1] = 0x1F;                                    // all low 0X1F
    while(I2C1_IsBusy()){};
    I2C1_Write(LED_DRIVER_ADDR, i2c_write_buf, 2);  
 
    i2c_write_buf[0] = LED_HOR_DIR_ADDR;                        // reg direction
    i2c_write_buf[1] = 0x00;                                    // output
    while(I2C1_IsBusy()){};
    I2C1_Write(LED_DRIVER_ADDR, i2c_write_buf, 2);

    i2c_write_buf[0] = LED_HOR_REG_ADDR;                        // reg address
    i2c_write_buf[1] = 0x1F;                                    // all low 0X1F
    while(I2C1_IsBusy()){};
    I2C1_Write(LED_DRIVER_ADDR, i2c_write_buf, 2); 
}

extern APP_TOUCH_DATA app_TouchData;
uint8_t get_pos(uint8_t pos){
    if(pos == 16){
        return (uint8_t)5;
    }else if(pos == 8){
        return (uint8_t)4;
    }else if(pos == 4){
        return (uint8_t)3;
    }else if(pos == 2){
        return (uint8_t)2;
    }else if(pos == 1){
        return (uint8_t)1;
    }
    return 0;
}

void led_decode_position (void) {
    led_reset();
    if (qtm_surface_cs_control1.qtm_surface_contact_data[1].qt_contact_status & TOUCH_ACTIVE) {
        app_TouchData.clr=1;
        app_TouchData.xpos=0;
        app_TouchData.touchActive=1;
        h_pos = (get_surface_position(0u, 0u) / 51u);
        h_pos = (1u << (4u - h_pos)) | 0x20;
        led_gpio_update(h_pos, LED_HOR);
        app_TouchData.ypos=255;
        v_pos = (get_surface_position(1u, 0u) / 51u);
        v_pos = (1u << (4u - v_pos));
        led_gpio_update(v_pos, LED_VER);
       
    }
    else
    {
        if (qtm_surface_cs_control1.qtm_surface_contact_data[0].qt_contact_status & TOUCH_ACTIVE) {
            app_TouchData.clr=0;
            app_TouchData.touchActive=1;
            app_TouchData.xpos=get_surface_position(0u, 0u);
            h_pos = (app_TouchData.xpos / 51u);
            h_pos = (1u << (4u - h_pos));
            led_gpio_update(h_pos, LED_HOR);
            app_TouchData.ypos=get_surface_position(1u, 0u);
            v_pos = (app_TouchData.ypos / 51u);
            v_pos = (1u << (4u - v_pos));
            led_gpio_update(v_pos, LED_VER);
           
        } 
        else{
            app_TouchData.touchActive=0;
        }
    }    
}


/* *****************************************************************************
 End of File
 */
