/*
 * S32K144 CAN Demo - Phase 1: Timer & Sensor Simulation (SAFE VERSION)
 *
 * Purpose: 10ms periodic timer interrupt for sensor value generation
 * CVD var.draw: sensor_data[0]=tick_count, [1]=rpm, [2]=temp, [3]=throttle
 */

#include "S32K144.h"
#include "device_registers.h"
#include "s32_core_cm4.h"

/* SysTick Register Definition for Cortex-M4 */
#ifndef SysTick
typedef struct {
    volatile uint32_t CTRL;   /* Offset: 0x00  Control and Status Register */
    volatile uint32_t LOAD;   /* Offset: 0x04  Reload Value Register */
    volatile uint32_t VAL;    /* Offset: 0x08  Current Value Register */
    volatile uint32_t CALIB;  /* Offset: 0x0C  Calibration Register */
} SysTick_Type;

#define SysTick_BASE  (0xE000E010UL)
#define SysTick       ((SysTick_Type *)SysTick_BASE)
#endif

/* Array index definitions for readability */
#define IDX_TICK_COUNT    0
#define IDX_SENSOR_RPM    1
#define IDX_SENSOR_TEMP   2
#define IDX_SENSOR_THROTTLE 3
#define IDX_TIMER_FLAG    4

/* Global sensor data array - visible in CVD debugger */
/* MUST be static for CVD var.draw! */
static volatile uint32_t sensor_data[5] = {0, 2000, 80, 50, 0};

/* Separate arrays for each sensor - for CVD var.draw */
#define HISTORY_SIZE 2000

volatile uint32_t tick_history[HISTORY_SIZE] = {0};
volatile uint32_t rpm_history[HISTORY_SIZE] = {0};
volatile uint32_t temp_history[HISTORY_SIZE] = {0};
volatile uint32_t throttle_history[HISTORY_SIZE] = {0};
volatile uint32_t history_index = 0;

/* Function prototypes */
void Flash_Protection_Init(void);
void SystemClock_Init(void);
void SysTick_Init(void);

/*
 * Main function
 */
int main(void)
{
    /* Initialize flash protection FIRST! */
    Flash_Protection_Init();
    
    /* Initialize system */
    SystemClock_Init();
    SysTick_Init();

    /* Main loop */
    while(1)
    {
        /* Wait for timer event */
        if (sensor_data[IDX_TIMER_FLAG]) {
            sensor_data[IDX_TIMER_FLAG] = 0;

            /* Future: CAN TX here */
            __asm("nop");
        }
    }

    return 0;
}

/*
 * Flash Protection Configuration
 * Protect critical regions (Vector Table, startup code)
 * MUST be called FIRST to prevent flash corruption!
 */
void Flash_Protection_Init(void)
{
    /* Read current value first - SAFE pattern */
    volatile uint8_t fstat = IP_FTFC->FSTAT;
    
    /* Wait for flash ready (CCIF bit = 1) */
    while(!(fstat & FTFC_FSTAT_CCIF_MASK)) {
        fstat = IP_FTFC->FSTAT;
    }
    
    /* Protect first 32KB (0x0000_0000 ~ 0x0000_7FFF) */
    volatile uint8_t fprot_val;
    
    fprot_val = IP_FTFC->FPROT[3];
    fprot_val |= 0xFF;
    IP_FTFC->FPROT[3] = fprot_val;
    
    fprot_val = IP_FTFC->FPROT[2];
    fprot_val |= 0xFF;
    IP_FTFC->FPROT[2] = fprot_val;
    
    fprot_val = IP_FTFC->FPROT[1];
    fprot_val |= 0xFF;
    IP_FTFC->FPROT[1] = fprot_val;
    
    fprot_val = IP_FTFC->FPROT[0];
    fprot_val |= 0xFF;
    IP_FTFC->FPROT[0] = fprot_val;
}

/*
 * System Clock Configuration - Safe Version
 * SOSC: 8MHz crystal
 * SPLL: 160MHz (8MHz * 40 / 2)
 * Core/System: 80MHz
 */
void SystemClock_Init(void)
{
    uint32_t timeout;

    /* SRAM debug: Skip if already configured */
    if (((IP_SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT) == 6) {
        return;
    }

    /* 1. SOSC Configuration */
    IP_SCG->SOSCCSR &= ~SCG_SOSCCSR_SOSCEN_MASK;
    IP_SCG->SOSCCFG = SCG_SOSCCFG_RANGE(2) | SCG_SOSCCFG_EREFS_MASK;
    IP_SCG->SOSCDIV = SCG_SOSCDIV_SOSCDIV2(1);
    IP_SCG->SOSCCSR = SCG_SOSCCSR_SOSCEN_MASK;

    timeout = 100000;
    while(!(IP_SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) && timeout--);
    if (timeout == 0) return;

    /* 2. SPLL Configuration */
    IP_SCG->SPLLCSR &= ~SCG_SPLLCSR_SPLLEN_MASK;
    IP_SCG->SPLLCFG = SCG_SPLLCFG_MULT(24);
    IP_SCG->SPLLDIV = SCG_SPLLDIV_SPLLDIV1(1) | SCG_SPLLDIV_SPLLDIV2(1);
    IP_SCG->SPLLCSR = SCG_SPLLCSR_SPLLEN_MASK;

    timeout = 100000;
    while(!(IP_SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK) && timeout--);
    if (timeout == 0) return;

    /* 3. Switch to SPLL - Safe Step-by-Step */
    uint32_t rccr_temp;
    
    rccr_temp = IP_SCG->RCCR;
    rccr_temp &= ~(SCG_RCCR_DIVCORE_MASK | SCG_RCCR_DIVBUS_MASK | SCG_RCCR_DIVSLOW_MASK);
    rccr_temp |= SCG_RCCR_DIVCORE(1) | SCG_RCCR_DIVBUS(1) | SCG_RCCR_DIVSLOW(2);
    IP_SCG->RCCR = rccr_temp;
    
    for(volatile int i = 0; i < 100; i++);
    
    rccr_temp &= ~SCG_RCCR_SCS_MASK;
    rccr_temp |= SCG_RCCR_SCS(6);
    IP_SCG->RCCR = rccr_temp;

    timeout = 10000;
    while((((IP_SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT) != 6) && timeout--);
}

/*
 * SysTick Timer Init - 100% SAFE!
 * Core clock: 80MHz, Period: 10ms, Reload: 800,000 - 1
 */
void SysTick_Init(void)
{
    SysTick->CTRL = 0;
    SysTick->LOAD = 800000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = (1 << 2) | (1 << 1) | (1 << 0);
}

/*
 * SysTick Interrupt Handler
 * Called every 10ms (100Hz)
 */
void SysTick_Handler(void)
{
    /* Increment tick counter */
    sensor_data[0]++;

    /* Sensor value simulation */
    
    /* RPM: Sawtooth wave 2000~8000 (period = 6000 ticks = 60 sec) */
    sensor_data[1] = 2000 + (sensor_data[IDX_TICK_COUNT] % 6000);

    /* Temperature: Slowly increase 80~120 (step every 100 ticks = 1 sec) */
    sensor_data[2] = 80 + ((sensor_data[IDX_TICK_COUNT] / 100) % 40);

    /* Throttle: Simple pattern 0~100 */
    uint32_t phase = (sensor_data[IDX_TICK_COUNT] / 50) % 4;
    switch(phase) {
        case 0: sensor_data[IDX_SENSOR_THROTTLE] = 50; break;
        case 1: sensor_data[IDX_SENSOR_THROTTLE] = 100; break;
        case 2: sensor_data[IDX_SENSOR_THROTTLE] = 50; break;
        case 3: sensor_data[IDX_SENSOR_THROTTLE] = 0; break;
    }

    /* Update history buffers */
    rpm_history[history_index] = sensor_data[IDX_SENSOR_RPM];
    temp_history[history_index] = sensor_data[IDX_SENSOR_TEMP];
    throttle_history[history_index] = sensor_data[IDX_SENSOR_THROTTLE];
    history_index = (history_index + 1) % HISTORY_SIZE;

    /* Set flag for main loop */
    sensor_data[IDX_TIMER_FLAG] = 1;
}



