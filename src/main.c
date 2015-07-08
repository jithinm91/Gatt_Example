#include "myble.h"
#include "bsp.h"
#include "ble_advertising.h"
#include "softdevice_handler.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"

static void scheduler_init(void);
static void power_manage(void);

int main()
{
    // Initialize
    
	myble_init();

    // Start execution
    // timers_start();
    
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




/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
