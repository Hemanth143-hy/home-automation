/*home_automation*/

#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_uuid.h"
#include "wiced_gki.h"
#include "wiced_bt_app_common.h"
#include "wiced_result.h"
#include "wiced_hal_platform.h"
#include "wiced_memory.h"
#include "wiced_transport.h"
#include "gatt_server_db.h"
#include "wiced_hal_nvram.h"
#include "wiced_timer.h"
#include "wiced_hal_puart.h"
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"

#include <stdbool.h>


#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif

/******************************************************************************
 *                             External Definitions
 ******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_app_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[];

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define AUTOMATION_SERVICE_VS_ID                     WICED_NVRAM_VSID_START
#define AUTOMATION_SERVICE_LOCAL_KEYS_VS_ID          ( AUTOMATION_SERVICE_VS_ID + 1 )
#define AUTOMATION_SERVICE_PAIRED_KEYS_VS_ID         AUTOMATION_SERVICE_LOCAL_KEYS_VS_ID + 1

/******************************************************************************
 *                                Structures
 ******************************************************************************/
/* Host information saved in  NVRAM */
typedef PACKED struct
{
    BD_ADDR  bdaddr;                               /* BD address of the bonded host */
    uint16_t characteristic_client_configuration;  /* Current value of the client configuration descriptor */
} host_info_t;

#pragma pack()

typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void     *p_attr;
} attribute_t;

/*typedef struct {
    uint8_t fan_state;  // Fan state: OFF (false), ON (true)
} FanControlProperties_t;
*/



/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
/* transport configuration */
const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    { WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD },
    { 0, 0},
    NULL,
    NULL,
    NULL
};


/* Holds the host info saved in the NVRAM */
host_info_t automation_service_hostinfo;

uint16_t global_conn_id = 0;

#define LIGHT_STATE_OFF  0
#define LIGHT_STATE_ON   1
uint8_t light_state;

uint8_t automation_service_device_name[]          = "Home_automation";                     //GAP Service characteristic Device Name
uint8_t automation_service_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };

attribute_t gatt_user_attributes[] =
{
    { HANDLE_BS_GAP_SERVICE_CHAR_DEV_NAME_VAL,             sizeof( automation_service_device_name ),   automation_service_device_name },
    { HANDLE_BS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,       sizeof(automation_service_appearance_name), automation_service_appearance_name},
    {HANDLE_AUTOMATION_SERVICE_FAN_CONTROL_CHAR_CFG_DESC,                           2 ,                (void*)&automation_service_hostinfo.characteristic_client_configuration},
    {HANDLE_AUTOMATION_SERVICE_LIGHT_CONTROL_CHAR_CFG_DESC,                         2 ,                (void*)&automation_service_hostinfo.characteristic_client_configuration},
    {HANDLE_AUTOMATION_SERVICE_LIGHT_BRIGHTNESS_CONTROL_CHAR_CFG_DESC,              2 ,                (void*)&automation_service_hostinfo.characteristic_client_configuration},
    {HANDLE_AUTOMATION_SERVICE_FAN_SPEED_CONTROL_CHAR_CFG_DESC,                     2 ,                (void*)&automation_service_hostinfo.characteristic_client_configuration},

};


/*****************************************************************************
 *                           Function Prototypes
 *****************************************************************************/

static wiced_result_t             automation_service_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void                       automation_service_application_init();
static wiced_bt_gatt_status_t     automation_service_gatt_cback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );
static void                       automation_service_load_keys_for_address_resolution( void );
static void                       automation_service_set_advertisement_data();
static wiced_bt_gatt_status_t     automation_service_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t     automation_service_gatts_connection_up(wiced_bt_gatt_connection_status_t *p_status);
static wiced_bt_gatt_status_t     automation_service_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t     automation_service_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data );
static wiced_bt_gatt_status_t     automation_service_gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data );
static wiced_bt_gatt_status_t     automation_service_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
static void                       automation_service_smp_bond_result( uint8_t result );
void store_connection_id(uint16_t conn_id);
static void                     automation_service_fan_control(bool fan_state);
static void                     automation_service_light_control(bool light_state);
static void                     automation_service_light_brightness_control(uint8_t light_control);
static void                     automation_service_fan_speed_control(uint8_t fan_control);


//void                            configure_pwm();
//void wiced_hal_pwm_change_values(uint8_t pin, uint32_t duty_cycle, uint32_t frequency);

#ifdef ENABLE_HCI_TRACE
static void                       automation_service_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
#endif

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
#ifndef CYW20735B0
APPLICATION_START( )
#else
void application_start( void )
#endif
{
    wiced_result_t  result;
    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE

    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );  //to see the traces of output

#if ( defined(CYW20706A2) || defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW43012C0) )

    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()

    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED

    // HCI debug interface to be parsed by ClientControl/BtSpy.

    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must

    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    //wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    WICED_BT_TRACE( " automation  Service Server Start\n" );
    // Register call back and configuration with stack
       wiced_bt_stack_init( automation_service_management_cback ,
                       &wiced_app_cfg_settings, wiced_app_cfg_buf_pools );
}
static void automation_service_application_init()
{
    wiced_bt_gatt_status_t gatt_status;
#ifndef CYW20735B1
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif
    /* Register for gatt event notifications */
    gatt_status = wiced_bt_gatt_register(&automation_service_gatt_cback );
    WICED_BT_TRACE( "wiced_bt_gatt_register: %d\n", gatt_status );
    /* Initialize GATT database */
    gatt_status = wiced_bt_gatt_db_init((uint8_t *)gatt_db, gatt_db_size );
    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);
#ifdef ENABLE_HCI_TRACE
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace( automation_service_hci_trace_cback );
#endif

#ifdef CYW20706A2
    /* Enable privacy to advertise with RPA */
    wiced_bt_ble_enable_privacy ( WICED_TRUE );
#endif
    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);


    /* Load the address resolution DB with the keys stored in the NVRAM */
    automation_service_load_keys_for_address_resolution();

    automation_service_set_advertisement_data();

    /* start LE advertising */
    wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
    WICED_BT_TRACE("Waiting for automation Service to connect...\n");

    //automation_service_fan_control(fan_state);
   /* wiced_init_timer( &automation_service_timer, automation_service_timer_expiry_handler, 0,
            WICED_MILLI_SECONDS_TIMER );*/
}

static void automation_service_set_advertisement_data()
{
    wiced_result_t              result;
    wiced_bt_ble_advert_elem_t  adv_elem[3];
    uint8_t ble_advertisement_flag_value        = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem                            = 0;
    uint16_t automation_service_uuid    = UUID_SERVICE_HOME_AUTOMATION;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = 1;
    adv_elem[num_elem].p_data       = &ble_advertisement_flag_value;
    num_elem ++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_16;
    adv_elem[num_elem].p_data       = (uint8_t *)&automation_service_uuid;
    num_elem ++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)wiced_app_cfg_settings.device_name);
    adv_elem[num_elem].p_data       = (uint8_t *)wiced_app_cfg_settings.device_name;

    num_elem++;

    result = wiced_bt_ble_set_raw_advertisement_data(num_elem,adv_elem);

    WICED_BT_TRACE("wiced_bt_ble_set_advertisement_data %d\n",result);
}

static wiced_bt_gatt_status_t automation_service_gatt_cback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;
    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = automation_service_gatts_conn_status_cb( &p_data->connection_status );
            uint16_t conn_id = p_data->connection_status.conn_id;
            store_connection_id(conn_id);
            WICED_BT_TRACE("Connection ID: %d\n", conn_id);
            break;
        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = automation_service_gatts_req_cb( &p_data->attribute_request );
            break;
        default:
            break;
    }
    return result;
}
static wiced_bt_gatt_status_t automation_service_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    if ( p_status->connected )
    {
        return automation_service_gatts_connection_up( p_status );
    }
    return automation_service_gatts_connection_down( p_status );
}
/* This function is invoked when connection is established */
static wiced_bt_gatt_status_t automation_service_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;
    uint8_t        bytes_written = 0;
    WICED_BT_TRACE( "automation_service_conn_up %B id:%d\n:", p_status->bd_addr, p_status->conn_id);
    /* Stop advertising */
    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
    WICED_BT_TRACE( "Stopping Advertisements%d\n", result );
    /* Updating the bd address in the  host info in NVRAM  */
    memcpy( automation_service_hostinfo.bdaddr, p_status->bd_addr, sizeof( BD_ADDR ) );
    /* Save the  host info in NVRAM  */
    bytes_written = wiced_hal_write_nvram( AUTOMATION_SERVICE_VS_ID, sizeof(automation_service_hostinfo), (uint8_t*)&automation_service_hostinfo, &result );
    WICED_BT_TRACE("NVRAM write %d\n", bytes_written);
    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t automation_service_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;
    WICED_BT_TRACE( "connection_down %B conn_id:%d reason:%d\n", p_status->conn_id, p_status->reason );
    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
    WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", result );
    return WICED_BT_SUCCESS;
}

static wiced_result_t automation_service_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;
    uint8_t                          *p_keys;

    WICED_BT_TRACE("automation_service_management_cback: %x\n", event );
    switch( event )
    {
    /* Bluetooth stack enabled */
    case BTM_ENABLED_EVT:
        automation_service_application_init();
        break;
    case BTM_DISABLED_EVT:
        break;
    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        WICED_BT_TRACE("numeric_value: %d \n", p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
        break;
    case BTM_PASSKEY_NOTIFICATION_EVT:
        WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
        break;
    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap   = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data       = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req       = BTM_LE_AUTH_REQ_SC_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size   = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys      = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys      = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        break;
    case BTM_PAIRING_COMPLETE_EVT:
        p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
        WICED_BT_TRACE( "Pairing Complete: %d",p_info->reason);
        automation_service_smp_bond_result( p_info->reason );
        break;
    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
        p_keys = (uint8_t*)&p_event_data->paired_device_link_keys_update;
        wiced_hal_write_nvram ( AUTOMATION_SERVICE_PAIRED_KEYS_VS_ID, sizeof( wiced_bt_device_link_keys_t ), p_keys ,&result );
        WICED_BT_TRACE("keys save to NVRAM %B result: %d \n", p_keys, result);
        break;
    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_keys = (uint8_t *)&p_event_data->paired_device_link_keys_request;
        wiced_hal_read_nvram( AUTOMATION_SERVICE_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p_keys, &result );
        WICED_BT_TRACE("keys read from NVRAM %B result: %d \n", p_keys, result);
        break;
    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
        p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
        wiced_hal_write_nvram ( AUTOMATION_SERVICE_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
        WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
        break;
    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram( AUTOMATION_SERVICE_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
        WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
        break;
    case BTM_ENCRYPTION_STATUS_EVT:
        p_status = &p_event_data->encryption_status;
        WICED_BT_TRACE("Encryption Status Event: bd ( %B ) res %d \n", p_status->bd_addr, p_status->result);
      //  automation_service_encryption_changed( p_status->result, p_status->bd_addr );
        break;
    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
        break;
    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\n", *p_mode);
        if ( *p_mode == BTM_BLE_ADVERT_OFF )
        {
                result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
                WICED_BT_TRACE("wiced_bt_start_advertisements: %d\n", result );
        }
        else
        {
          WICED_BT_TRACE("ADV stop\n");
         }
        break;
    default:
        break;
    }

    return result;
}
static void automation_service_load_keys_for_address_resolution( void )
{
    wiced_bt_device_link_keys_t link_keys;
      wiced_result_t              result;
      uint8_t                     *p;
    memset( &link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
    p = (uint8_t*)&link_keys;
    wiced_hal_read_nvram( AUTOMATION_SERVICE_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p, &result);
    if(result == WICED_BT_SUCCESS)
    {
#ifdef CYW20706A2
        result = wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys, link_keys.key_data.ble_addr_type );
#else
        result = wiced_bt_dev_add_device_to_address_resolution_db(&link_keys);
#endif
    }
    WICED_BT_TRACE("automation_service_load_keys_for_address_resolution %B result:%d \n", p, result );
}
static wiced_bt_gatt_status_t automation_service_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_result_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE( "automation_service_gatts_req_cb. conn %d, type %d\n", p_data->conn_id, p_data->request_type );

    switch ( p_data->request_type )
    {
    case GATTS_REQ_TYPE_READ:

        result = automation_service_gatts_req_read_handler( p_data->conn_id, &(p_data->data.read_req) );
        break;

    case GATTS_REQ_TYPE_WRITE:

       result = automation_service_gatts_req_write_handler( p_data->conn_id, &(p_data->data.write_req) );
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        break;

    case GATTS_REQ_TYPE_MTU:
        break;

    case GATTS_REQ_TYPE_CONF:
        break;

   default:
        break;
    }

    return result;
}

/*
 * Find attribute description by handle
 */
attribute_t * automation_service_get_attribute( uint16_t handle )
{
    int i;
    for ( i = 0; i <  sizeof( gatt_user_attributes ) / sizeof( gatt_user_attributes[0] ); i++ )
    {
        if ( gatt_user_attributes[i].handle == handle )
        {
            return ( &gatt_user_attributes[i] );
        }
    }
    WICED_BT_TRACE( "attr not found:%x\n", handle );
    return NULL;
}
static wiced_bt_gatt_status_t automation_service_gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    attribute_t *puAttribute;
    int   attr_len_to_copy;
    if (( puAttribute = automation_service_get_attribute(p_read_data->handle) ) == NULL)
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\n", p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }
     attr_len_to_copy = puAttribute->attr_len;
    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy );
    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int  to_copy = attr_len_to_copy - p_read_data->offset;
        if ( to_copy > *p_read_data->p_val_len )
        {
            to_copy = *p_read_data->p_val_len;
        }
        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;
        memcpy( p_read_data->p_val, from, to_copy);
    }
    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t automation_service_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_data)
{
    if (p_data == NULL || p_data->p_val == NULL || p_data->val_len != sizeof(uint8_t))
    {
        return WICED_BT_GATT_INVALID_ATTR_LEN;
    }
    uint16_t attr_handle = p_data->handle;
    uint8_t control_value = *(p_data->p_val);
    switch (attr_handle)
    {
    case HANDLE_AUTOMATION_SERVICE_CHAR_FAN_CONTROL_VAL:
        if (control_value != 0 && control_value != 1)
        {
            return WICED_BT_GATT_ERROR;
        }
        automation_service_fan_control(control_value);
        break;
    case HANDLE_AUTOMATION_SERVICE_CHAR_LIGHT_CONTROL_VAL:
        if (control_value != 0 && control_value != 1)
        {
            return WICED_BT_GATT_ERROR;
        }
        automation_service_light_control(control_value);
        break;
    case HANDLE_AUTOMATION_SERVICE_CHAR_LIGHT_BRIGHTNESS_CONTROL_VAL:
        if (control_value != 0 && control_value != 1 && control_value != 2  && control_value != 3)
        {
                  return WICED_BT_GATT_ERROR;
        }
        automation_service_light_brightness_control(control_value);
        break;
    case HANDLE_AUTOMATION_SERVICE_CHAR_FAN_SPEED_CONTROL_VAL:
           if (control_value != 0 && control_value != 1 && control_value != 2  && control_value != 3)
           {
                     return WICED_BT_GATT_ERROR;
           }
           automation_service_fan_speed_control(control_value);
           break;
    default:
        return WICED_BT_GATT_ERROR;
    }
    return WICED_BT_GATT_SUCCESS; // Return success status
}

static void automation_service_smp_bond_result( uint8_t result)
{
    wiced_result_t status;
    uint8_t written_byte = 0;
    WICED_BT_TRACE( "Home_automation, bond result: %d\n", result);
}
void store_connection_id(uint16_t conn_id) {
    global_conn_id = conn_id;
}
static void automation_service_fan_control(bool fan_state) {
    if(fan_state == 1) {
        WICED_BT_TRACE("POWER ON  FAN");
        wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_1,0);
        } else {
        WICED_BT_TRACE("POWER OFF FAN");
        wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_1,1);
    }
}
static void automation_service_light_control(bool light_state) {

    if(light_state == 1) {
        WICED_BT_TRACE("POWER ON LIGHT");
        wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_2,0);
        } else {
        WICED_BT_TRACE("POWER OFF LIGHT");
        wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_2,1);

    }
}
static void automation_service_light_brightness_control(uint8_t light_control)\
{
    if (light_control == 0)
    {
        automation_service_light_control(light_state);
        WICED_BT_TRACE("light_OFF");
    } else if (light_control == 1)
    {
        WICED_BT_TRACE("LOW brightness nearly 30%");
    } else if (light_control == 2)
    {
        WICED_BT_TRACE("MEDIUM BIRGHTNESS NEARLY 60%");
    } else if (light_control == 3)
    {
        WICED_BT_TRACE("HIGH total 100%");
    } else
    {
        return;
    }
}
static void   automation_service_fan_speed_control(uint8_t fan_control)
{
        if (fan_control == 0)
        {
            automation_service_fan_control(fan_control);
            WICED_BT_TRACE("FAN_OFF");
        }
        else if (fan_control == 1)
        {
            WICED_BT_TRACE("LOW_SPEED");
        }
        else if (fan_control == 2)
        {
            WICED_BT_TRACE("MEDIUM_SPEED");
        } else if (fan_control == 3)
        {
            WICED_BT_TRACE("HIGH_SPEED");
        } else
        {
            return;
        }
}
