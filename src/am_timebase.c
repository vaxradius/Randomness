
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#include "am_mcu_apollo.h"
#include "am_util_delay.h"

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
volatile unsigned long long g_ui64SysTickWrappedTime = 0;

#define AM_CORECLK_HZ     AM_HAL_CLKGEN_FREQ_MAX_HZ
#define AM_CORECLK_KHZ    (AM_CORECLK_HZ/1000)
#define AM_GET_MS_TICK    (((uint64_t)(0xFFFFFF - am_hal_systick_count()) + g_ui64SysTickWrappedTime)/AM_CORECLK_KHZ)

//*****************************************************************************
//
// Systick ISR.
//
//*****************************************************************************
void
SysTick_Handler(void)
{
    /* Add enough cycles to account for one full cycle of systick */
    g_ui64SysTickWrappedTime += 0x01000000;
}


void am_timebase_busy_wait(uint32_t delay)
{
    if (am_hal_burst_mode_status() == AM_HAL_BURST_MODE)
        am_util_delay_ms(delay*2);
    else
        am_util_delay_ms(delay);
}

volatile uint32_t g_ui32SysTickLast = 0xFFFFFF;
volatile uint64_t g_ui64SysTickWrappedTimeLast = 0;

uint32_t am_timebase_get_tick_ms(void)
{
    uint32_t critical;
    uint32_t tick;
    uint32_t systick;
    uint32_t CoreClk;

    critical = am_hal_interrupt_master_disable();

    if (am_hal_burst_mode_status() == AM_HAL_BURST_MODE)
        CoreClk = AM_CORECLK_KHZ*2;
    else
        CoreClk = AM_CORECLK_KHZ;

    systick = am_hal_systick_count();
    if (g_ui64SysTickWrappedTime == g_ui64SysTickWrappedTimeLast && systick > g_ui32SysTickLast)
        tick =(((0xFFFFFF - systick) + (0xFFFFFF - g_ui32SysTickLast) + g_ui64SysTickWrappedTime)/CoreClk);
    else
        tick =(((0xFFFFFF - systick) + g_ui64SysTickWrappedTime)/CoreClk);

    g_ui32SysTickLast = systick;
    g_ui64SysTickWrappedTimeLast = g_ui64SysTickWrappedTime;

    am_hal_interrupt_master_set(critical);
    return tick;
}

#if 0
uint64_t am_timebase_get_tick(void)
{
    uint32_t critical;
    uint64_t tick;
    uint32_t systick;
    uint32_t CoreClk;

    critical = am_hal_interrupt_master_disable();

    systick = am_hal_systick_count();
    if (g_ui64SysTickWrappedTime == g_ui64SysTickWrappedTimeLast && systick > g_ui32SysTickLast)
        tick =((uint64_t)(0xFFFFFF - systick) + (uint64_t)(0xFFFFFF - g_ui32SysTickLast) + g_ui64SysTickWrappedTime);
    else
        tick =((uint64_t)(0xFFFFFF - systick) + g_ui64SysTickWrappedTime);

    g_ui32SysTickLast = systick;
    g_ui64SysTickWrappedTimeLast = g_ui64SysTickWrappedTime;

    am_hal_interrupt_master_set(critical);
	
    return tick;
}
#else
 uint32_t am_timebase_get_tick(unsigned long long* mytick)
{
    uint32_t critical;
    uint32_t tickms;
		unsigned long long tick;
    uint32_t systick;
    uint32_t CoreClk;

    critical = am_hal_interrupt_master_disable();

    if (am_hal_burst_mode_status() == AM_HAL_BURST_MODE)
        CoreClk = AM_CORECLK_KHZ*2;
    else
        CoreClk = AM_CORECLK_KHZ;

    systick = am_hal_systick_count();
    if (g_ui64SysTickWrappedTime == g_ui64SysTickWrappedTimeLast && systick > g_ui32SysTickLast)
        tick =(((0xFFFFFF - (unsigned long long)systick) + (0xFFFFFF - (unsigned long long)g_ui32SysTickLast) + g_ui64SysTickWrappedTime));
    else
        tick =(((0xFFFFFF - (unsigned long long)systick) + (unsigned long long)g_ui64SysTickWrappedTime));
		*mytick = tick;
		tickms = tick/CoreClk;

    g_ui32SysTickLast = systick;
    g_ui64SysTickWrappedTimeLast = g_ui64SysTickWrappedTime;

    am_hal_interrupt_master_set(critical);
    return tickms;
}
#endif

void am_timebase_init(void)
{
    am_hal_systick_load(0x00FFFFFF);
    am_hal_systick_int_enable();
    am_hal_systick_start();
}
