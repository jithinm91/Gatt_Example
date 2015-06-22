#include "myble.h"
#include "bsp.h"
#include "app_scheduler.h"
#include "ble_advertising.h"
#include "app_timer.h"
#include "nrf_delay.h"

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

#define APP_TIMER_PRESCALER             0          

static void bsp_module_init(void);
static void scheduler_init(void);
static void timers_init(void);
static void power_manage(void);

int main()
{
    uint32_t err_code;
    
	// Initialize
    timers_init();
	ble_stack_init();
	scheduler_init();
	gap_params_init();
	advertising_init();
	services_init();
    conn_params_init();
    sec_params_init();

    // Start execution
    // timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    // Enter main loop
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }
}

/**@brief Function for initializing bsp module.
 */
static void bsp_module_init(void)
{
    uint32_t err_code;
    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
