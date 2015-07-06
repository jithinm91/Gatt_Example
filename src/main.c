#include "myble.h"
#include "bsp.h"
#include "app_scheduler.h"
#include "ble_advertising.h"
#include "app_timer_appsh.h"
#include "softdevice_handler.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

static void scheduler_init(void);
static void power_manage(void);

int main()
{
    
    
	// Initialize
    
	scheduler_init();
	myble_init();

    // Start execution
    // timers_start();
    
    SEGGER_RTT_WriteString(0, "Hello World!\n");
    char c = 0;
    // Enter main loop
    for (;;)
    {
        c = SEGGER_RTT_WaitKey(); // will block until data is available
        if(c == 'r'){
            SEGGER_RTT_printf(0, "%sResetting in %d second..%s\n", RTT_CTRL_BG_BRIGHT_RED, 1, RTT_CTRL_RESET);
            nrf_delay_ms(1000);
            sd_nvic_SystemReset();
        }
        app_sched_execute();
        power_manage();
    }
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
