/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
 *  Corporation. All rights reserved. This software, including source code, documentation and  related 
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
 * (United States and foreign), United States copyright laws and international treaty provisions. 
 * Therefore, you may use this Software only as provided in the license agreement accompanying the 
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit 
 * products. Any reproduction, modification, translation, compilation,  or representation of this 
 * Software except as specified above is prohibited without the express written permission of 
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
 * the Software without notice. Cypress does not assume any liability arising out of the application 
 * or use of the Software or any product or circuit  described in the Software. Cypress does 
 * not authorize its products for use in any products where a malfunction or failure of the 
 * Cypress product may reasonably be expected to result  in significant property damage, injury 
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
 * to indemnify Cypress against all liability.
 */

/** @file
 *
 * Battery Service Sample Application (GATT Server database definitions)
 *
 */
#include "wiced.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
/* Primary Service GATT service */
    HANDLE_BS_GATT_SERVICE = 0x1,

/* Primary Service GAP service */
    HANDLE_BS_GAP_SERVICE = 0x14,
        HANDLE_BS_GAP_SERVICE_CHAR_DEV_NAME,
        HANDLE_BS_GAP_SERVICE_CHAR_DEV_NAME_VAL,

        HANDLE_BS_GAP_SERVICE_CHAR_DEV_APPEARANCE,
        HANDLE_BS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,

        /*primary Service of home automation*/
            HANDLE_AUTOMATION_SERVICE = 0x50,
                HANDLE_AUTOMATION_SERVICE_CHAR_FAN_CONTROL,
                HANDLE_AUTOMATION_SERVICE_CHAR_FAN_CONTROL_VAL,
                HANDLE_AUTOMATION_SERVICE_FAN_CONTROL_CHAR_CFG_DESC,

                HANDLE_AUTOMATION_SERVICE_CHAR_LIGHT_CONTROL,
                HANDLE_AUTOMATION_SERVICE_CHAR_LIGHT_CONTROL_VAL,
                HANDLE_AUTOMATION_SERVICE_LIGHT_CONTROL_CHAR_CFG_DESC,

                HANDLE_AUTOMATION_SERVICE_CHAR_LIGHT_BRIGHTNESS_CONTROL,
                HANDLE_AUTOMATION_SERVICE_CHAR_LIGHT_BRIGHTNESS_CONTROL_VAL,
                HANDLE_AUTOMATION_SERVICE_LIGHT_BRIGHTNESS_CONTROL_CHAR_CFG_DESC,


                HANDLE_AUTOMATION_SERVICE_CHAR_FAN_SPEED_CONTROL,
                HANDLE_AUTOMATION_SERVICE_CHAR_FAN_SPEED_CONTROL_VAL,
                HANDLE_AUTOMATION_SERVICE_FAN_SPEED_CONTROL_CHAR_CFG_DESC,

};


extern const uint8_t  gatt_db[];
extern const uint16_t gatt_db_size;

#ifdef __cplusplus
}
#endif
