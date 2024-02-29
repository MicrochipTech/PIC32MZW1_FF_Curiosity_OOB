/* ************************************************************************** */
/** LED Driver

  @Description
    Drives QT8 LEDs based on x/y touch position output by Qtouch modular library.
 */
/* ************************************************************************** */

#ifndef _EXAMPLE_FILE_NAME_H    /* Guard against multiple inclusion */
#define _EXAMPLE_FILE_NAME_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

#define LED_DRIVER_ADDR  0x20
#define LED_HOR_DIR_ADDR 0x00
#define LED_VER_DIR_ADDR 0x01
#define LED_HOR_REG_ADDR 0x14
#define LED_VER_REG_ADDR 0x15
#define LED_HOR 0u
#define LED_VER 1u

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

void init_led_driver(void);
void led_gpio_update(uint8_t data, uint8_t ver_or_hor);
void led_reset(void);
void led_decode_position(void);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
