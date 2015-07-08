#include "softdevice_handler.h"
#include "ble.h"
#include "string.h"
#include "ble_advertising.h"
#include "bsp.h"
#include "ble_srv_common.h"
#include "ble_conn_params.h"
#include "app_timer_appsh.h"
#include "ble_hci.h"
#include "app_uart.h"
#include "SEGGER_RTT.h"
#include "app_timer_appsh.h"
#include "app_scheduler.h"
#include "ble_gap.h"
#include "ble_bas.h"

#define UNKNOWN_UUID_BASE {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define UNKNOWN_UUID_SERVICE 0x1523
#define UNKNOWN_UUID_LED_CHAR 0x1525

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1	                                        /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define DEVICE_NAME                     "JithinFinroboticsR"                        /**< Name of device. Will be included in the advertising data. */

#define P_DATA                          "Hello"
#define P_S_DATA                        "World"

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            (2 + BSP_APP_TIMERS_NUMBER)                 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static void sleep_mode_enter(void);
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
static void conn_params_error_handler(uint32_t nrf_error);
static void bas_init(void);
static void custom_service_init(void);

ble_uuid_t m_adv_uuids[] = {{BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE}};

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */

void uart_event_handle(app_uart_evt_t * p_event);
    
/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
	uint32_t 						 err_code;
	static ble_gap_evt_auth_status_t m_auth_status;
	bool 							 master_id_matches;
	ble_gap_sec_kdist_t * 			 p_distributed_keys;
	ble_gap_enc_info_t * 			 p_enc_info;
	ble_gap_irk_t *                  p_id_info;
    ble_gap_sign_info_t *            p_sign_info;
	
	static ble_gap_enc_key_t         m_enc_key;           /**< Encryption Key (Encryption Info and Master ID). */
    static ble_gap_id_key_t          m_id_key;            /**< Identity Key (IRK and address). */
    static ble_gap_sign_info_t       m_sign_key;          /**< Signing Key (Connection Signature Resolving Key). */
    static ble_gap_sec_keyset_t      m_keys = {.keys_periph = {&m_enc_key, &m_id_key, &m_sign_key}};
	
	switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
			err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
			break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params,
                                                   &m_keys);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle,
                                                 NULL,
                                                 0,
                                                 BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            master_id_matches  = memcmp(&p_ble_evt->evt.gap_evt.params.sec_info_request.master_id,
                                        &m_enc_key.master_id,
                                        sizeof(ble_gap_master_id_t)) == 0;
            p_distributed_keys = &m_auth_status.kdist_periph;

            p_enc_info  = (p_distributed_keys->enc  && master_id_matches) ? &m_enc_key.enc_info : NULL;
            p_id_info   = (p_distributed_keys->id   && master_id_matches) ? &m_id_key.id_info   : NULL;
            p_sign_info = (p_distributed_keys->sign && master_id_matches) ? &m_sign_key         : NULL;

            err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, p_id_info, p_sign_info);
                APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	on_ble_evt(p_ble_evt);
	ble_conn_params_on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}
	
/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
	ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init()
{
	uint32_t err_code;
	
	//Initialize softdevice handler
	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);
	
	// Enable BLE stack 
	ble_enable_params_t ble_enable_params;
	memset(&ble_enable_params, 0, sizeof(ble_enable_params));
	
	ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
	err_code = sd_ble_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);
	
	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);
	
	// Register with the SoftDevice handler module for System events.
	err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init()
{
	uint32_t err_code;
	ble_gap_conn_params_t gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;
	
    ble_gap_addr_t addr;
    
    err_code = sd_ble_gap_address_get(&addr);
    APP_ERROR_CHECK(err_code);
      
    addr.addr_type=BLE_GAP_ADDR_TYPE_PUBLIC;
    addr.addr[5]=0x80;
    addr.addr[4]=0xB3;
    addr.addr[3]=0xD5;
    addr.addr[2]=0x0C;
    addr.addr[1]&=0x0f;
    addr.addr[1]|=0x80;
	 

  	err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);
    APP_ERROR_CHECK(err_code);
       
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	
	err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);
	
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);
    
	memset(&gap_conn_params, 0, sizeof(gap_conn_params));
	
	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
	gap_conn_params.slave_latency 	  = SLAVE_LATENCY;
	
	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = sd_ble_gap_tx_power_set(0);
    APP_ERROR_CHECK(err_code);
//    SEGGER_RTT_printf(0, "errorcode = %d \n", err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init(void)
{
	uint32_t      err_code;
	ble_advdata_t advdata;
	
	// Build advertising data struct to pass into @ref ble_advertising_init.
	memset(&advdata, 0, sizeof(advdata));
	
    ble_advdata_manuf_data_t        manuf_data;     // Variable to hold manufacturer specific data
    uint8_t data[]                      = {1,2,3};  // Our data to adverise
    manuf_data.data.p_data              = data;     
    manuf_data.data.size                = sizeof(data);
    
	advdata.name_type = BLE_ADVDATA_SHORT_NAME;
    advdata.short_name_len = 6;                     // Advertise only first 6 letters of name
	advdata.include_appearance = true;
	advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;
    advdata.p_manuf_specific_data   = &manuf_data;  // Load the manufacturer specific data into advertising packet
    
    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    ble_advdata_manuf_data_t manuf_data_response;
    uint8_t data_response[]          = {4,5,6};     // Our data to adverise
    manuf_data_response.data.p_data  = data_response;     
    manuf_data_response.data.size    = sizeof(data_response);
    ble_advdata_t advdata_response;
    
    // Always initialize all fields in structs to zero or you might get unexpected behaviour
    memset(&advdata_response, 0, sizeof(advdata_response));
    advdata_response.name_type = BLE_ADVDATA_NO_NAME;
    advdata_response.p_manuf_specific_data       = &manuf_data_response;   // Load the manufacturer specific data into advertising packet
    
    err_code = ble_advertising_init(&advdata, &advdata_response, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}
	
/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;
       
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
            
        default:
            break;
    }
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 */
void services_init(void)
{
    bas_init();
    custom_service_init();
}

static void custom_service_init(void)
{
    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t base_uuid = {UNKNOWN_UUID_BASE};
    uint8_t base_uuid_type;
    
    err_code = sd_ble_uuid_vs_add(&base_uuid, &base_uuid_type);
    APP_ERROR_CHECK(err_code);
    
    ble_uuid.type = base_uuid_type;
    ble_uuid.uuid = UNKNOWN_UUID_SERVICE;
        
    // Add service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, NULL);
    APP_ERROR_CHECK(err_code);
}
static void bas_init(void)
{
    uint32_t err_code;
    ble_bas_t p_bas;
    ble_bas_init_t p_bas_init;
    
    memset(&p_bas_init,0,sizeof(p_bas_init));
    
    p_bas_init.evt_handler = NULL;
    p_bas_init.support_notification = true;
    p_bas_init.p_report_ref = NULL;
    p_bas_init.initial_batt_level = 100;
    
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&p_bas_init.battery_level_char_attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_bas_init.battery_level_report_read_perm);
    
    ble_bas_init(&p_bas,&p_bas_init);
}

/**@brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing security parameters.
 */
void sec_params_init(void)
{
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
}

/**@brief Function for initializing bsp module.
 */
static void bsp_module_init(void)
{
    uint32_t err_code;
    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    printf("%d\n",err_code);
    //APP_ERROR_CHECK(err_code);
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

void myble_init(void)
{
    uint32_t err_code;
    
    timers_init();
    uart_init();
	ble_stack_init();
    bsp_module_init();
    scheduler_init();
    gap_params_init();
	advertising_init();
	services_init();
    conn_params_init();
    sec_params_init();
      
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}
