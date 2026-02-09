/*
 * S32K144 CAN Demo - Phase 2: CAN Message Transmission
 *
 * Purpose: 10ms periodic CAN message transmission with sensor simulation
 * 
 * CAN Messages:
 *   - 0x100: RPM (uint32_t, Little-endian) @ 10ms
 *   - 0x101: Temperature (uint32_t, Little-endian) @ 10ms
 *   - 0x102: Throttle (uint32_t, Little-endian) @ 10ms
 *
 * Hardware:
 *   - CAN0_RX: PTE4
 *   - CAN0_TX: PTE5
 *   - Baudrate: 500kbps
 *
 * Build: Use S32K144_64_ram.ld (SRAM execution for fast iteration)
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

/* CAN Message IDs */
#define CAN_MSG_ID_RPM        0x100
#define CAN_MSG_ID_TEMP       0x101
#define CAN_MSG_ID_THROTTLE   0x102

/* Array index definitions for readability */
#define IDX_TICK_COUNT    0
#define IDX_SENSOR_RPM    1
#define IDX_SENSOR_TEMP   2
#define IDX_SENSOR_THROTTLE 3
#define IDX_TIMER_FLAG    4

#ifndef IP_CAN0
#endif
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

/* CAN transmission counters for debugging */
volatile uint32_t can_tx_count[3] = {0, 0, 0}; // RPM, Temp, Throttle

/* Function prototypes */
void Flash_Protection_Init(void);
void SystemClock_Init(void);
void SysTick_Init(void);
void CAN0_Init(void);
void CAN0_SendMessage(uint32_t msg_id, uint32_t data);

/*
 * Main function
 */
int main(void)
{
    /* Initialize flash protection FIRST! */
    Flash_Protection_Init();
    
    /* Initialize system */
    SystemClock_Init();
    CAN0_Init();
    SysTick_Init();

    /* Main loop */
    while(1)
    {
        /* Wait for timer event */
        if (sensor_data[IDX_TIMER_FLAG]) {
            sensor_data[IDX_TIMER_FLAG] = 0;

            /* CAN TX - transmit sensor data */
            CAN0_SendMessage(CAN_MSG_ID_RPM, sensor_data[IDX_SENSOR_RPM]);
            CAN0_SendMessage(CAN_MSG_ID_TEMP, sensor_data[IDX_SENSOR_TEMP]);
            CAN0_SendMessage(CAN_MSG_ID_THROTTLE, sensor_data[IDX_SENSOR_THROTTLE]);
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
 * CAN0 Initialization
 * 
 * Configuration:
 *   - Baudrate: 500kbps
 *   - Bit timing: Based on 80MHz peripheral clock (Bus clock / 2 = 40MHz CAN clock)
 *   - Message Buffer 0: TX for RPM
 *   - Message Buffer 1: TX for Temperature
 *   - Message Buffer 2: TX for Throttle
 *   - Pins: PTE4 (RX), PTE5 (TX)
 */
void CAN0_Init(void)
{
    /* 1. Enable CAN0 clock */
    IP_PCC->PCCn[PCC_FlexCAN0_INDEX] = 0;  /* Disable first */
    IP_PCC->PCCn[PCC_FlexCAN0_INDEX] = PCC_PCCn_CGC_MASK | PCC_PCCn_PCS(1);  /* Clock source: Bus clock (80MHz) */

    /* 2. Enable PORTE clock for CAN pins */
    IP_PCC->PCCn[PCC_PORTE_INDEX] = PCC_PCCn_CGC_MASK;

    /* 3. Configure CAN0 pins: PTE4 (CAN0_RX), PTE5 (CAN0_TX) */
    IP_PORTE->PCR[4] = PORT_PCR_MUX(5);  /* PTE4 = CAN0_RX, ALT5 */
    IP_PORTE->PCR[5] = PORT_PCR_MUX(5);  /* PTE5 = CAN0_TX, ALT5 */

    /* 4. Enter Freeze Mode for configuration */
    IP_CAN0->MCR |= CAN_MCR_FRZ_MASK;    /* Enable Freeze mode */
    IP_CAN0->MCR |= CAN_MCR_HALT_MASK;   /* Request Freeze mode entry */
    
    /* Wait for Freeze Mode Acknowledge */
    while(!(IP_CAN0->MCR & CAN_MCR_FRZACK_MASK));

    /* 5. Module Configuration */
    IP_CAN0->MCR |= CAN_MCR_SOFTRST_MASK;  /* Soft reset */
    while(IP_CAN0->MCR & CAN_MCR_SOFTRST_MASK);  /* Wait for reset complete */

    IP_CAN0->MCR = CAN_MCR_MAXMB(3);  /* Use 3 message buffers (0, 1, 2) */

    /* 6. Bit Timing Configuration for 500kbps
     * 
     * CAN Clock = Bus Clock / 2 = 80MHz / 2 = 40MHz
     * Bit Rate = 500kbps
     * 
     * Bit Time = CAN Clock / (PRESDIV + 1) / (1 + PSEG1 + PSEG2 + PROPSEG)
     * 500kbps = 40MHz / (PRESDIV + 1) / (1 + PSEG1 + PSEG2 + PROPSEG)
     * 
     * Let: PRESDIV = 4, then Time Quantum = 40MHz / 5 = 8MHz
     * Bit Time = 8MHz / 500kHz = 16 TQ
     * 
     * Standard CAN bit structure:
     * SYNC_SEG = 1 TQ (fixed)
     * PROP_SEG = 2 TQ
     * PHASE_SEG1 = 7 TQ
     * PHASE_SEG2 = 6 TQ
     * Total = 1 + 2 + 7 + 6 = 16 TQ
     * 
     * SJW (Synchronization Jump Width) = min(PHASE_SEG1, PHASE_SEG2) = 4 TQ (typical)
     * Sample Point = (1 + PROP_SEG + PHASE_SEG1) / 16 = 10/16 = 62.5%
     */
    IP_CAN0->CTRL1 = CAN_CTRL1_PRESDIV(4)      /* Prescaler = 5 */
                   | CAN_CTRL1_PROPSEG(1)      /* PROP_SEG = 2 TQ */
                   | CAN_CTRL1_PSEG1(6)        /* PHASE_SEG1 = 7 TQ */
                   | CAN_CTRL1_PSEG2(5)        /* PHASE_SEG2 = 6 TQ */
                   | CAN_CTRL1_RJW(3);         /* SJW = 4 TQ */

    /* 7. Initialize Message Buffers as INACTIVE */
    for(int i = 0; i < 3; i++) {
        IP_CAN0->RAMn[i * 4 + 0] = 0x08000000;  /* CS: CODE = 0b1000 (TX INACTIVE) */
        IP_CAN0->RAMn[i * 4 + 1] = 0x00000000;  /* ID */
        IP_CAN0->RAMn[i * 4 + 2] = 0x00000000;  /* DATA 0-3 */
        IP_CAN0->RAMn[i * 4 + 3] = 0x00000000;  /* DATA 4-7 */
    }

    /* 8. Exit Freeze Mode */
    IP_CAN0->MCR &= ~CAN_MCR_HALT_MASK;
    while(IP_CAN0->MCR & CAN_MCR_FRZACK_MASK);

    /* 9. Wait for module ready */
    while(IP_CAN0->MCR & CAN_MCR_NOTRDY_MASK);
}

/*
 * CAN0 Message Transmission
 * 
 * Uses Message Buffers 0-2 for transmission in round-robin fashion
 * Data format: Little-endian uint32_t
 * 
 * Parameters:
 *   msg_id: CAN message identifier (11-bit standard ID)
 *   data: 32-bit data to transmit (Little-endian)
 */
void CAN0_SendMessage(uint32_t msg_id, uint32_t data)
{
    static uint8_t mb_index = 0;  /* Round-robin MB selection */
    uint32_t cs_word;

    /* Select message buffer (0, 1, or 2) */
    uint32_t mb_offset = mb_index * 4;

    /* Wait if MB is busy (previous transmission not complete) */
    while(IP_CAN0->RAMn[mb_offset] & 0x01000000);  /* Wait for CODE != 0b1100 (TX busy) */

    /* 1. Write Control/Status word - INACTIVE state */
    cs_word = 0x08000000              /* CODE = 0b1000 (TX INACTIVE) */
            | (4 << 16);              /* DLC = 4 bytes */
    IP_CAN0->RAMn[mb_offset + 0] = cs_word;

    /* 2. Write Message ID (Standard 11-bit ID) */
    IP_CAN0->RAMn[mb_offset + 1] = (msg_id << 18);  /* ID in bits[28:18] for standard ID */

    /* 3. Write Data - Little Endian uint32_t */
    IP_CAN0->RAMn[mb_offset + 2] = data;

    /* 4. Activate transmission - Set CODE to TX_DATA (0b1100) */
    cs_word = 0x0C000000              /* CODE = 0b1100 (TX DATA) */
            | (4 << 16);              /* DLC = 4 bytes */
    IP_CAN0->RAMn[mb_offset + 0] = cs_word;

    /* 5. Update transmission counter for debugging */
    if (msg_id == CAN_MSG_ID_RPM) can_tx_count[0]++;
    else if (msg_id == CAN_MSG_ID_TEMP) can_tx_count[1]++;
    else if (msg_id == CAN_MSG_ID_THROTTLE) can_tx_count[2]++;

    /* 6. Move to next MB for next transmission */
    mb_index = (mb_index + 1) % 3;
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
