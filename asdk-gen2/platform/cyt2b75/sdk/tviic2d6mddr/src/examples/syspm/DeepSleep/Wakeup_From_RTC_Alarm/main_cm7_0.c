/***************************************************************************//**
* \file main_cm0plus.c
*
* \version 1.0
*
* \brief Main example file for CM0+
*
********************************************************************************
* \copyright
* Copyright 2016-2020, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_project.h"
#include "cy_device_headers.h"

/*******************************************************************************
*    Parameters definitions
*******************************************************************************/

#if (CY_USE_PSVP == 1)
    #define USER_LED_PORT           CY_BSP_BB_USER_LED1_PORT
    #define USER_LED_PIN            CY_BSP_BB_USER_LED1_PIN
    #define USER_LED_PIN_MUX        CY_BSP_BB_USER_LED1_PIN_MUX
#else
    #define USER_LED_PORT           CY_USER_LED2_PORT
    #define USER_LED_PIN            CY_USER_LED2_PIN
    #define USER_LED_PIN_MUX        CY_USER_LED2_PIN_MUX
#endif

/* Date Format setting constants */
#define RTC_MM_DD_YYYY                (0u)
#define RTC_DD_MM_YYYY                (1u)
#define RTC_YYYY_MM_DD                (2u)

/* Initial Time and Date definitions */
#define RTC_INITIAL_DATE_SEC          (0u)
#define RTC_INITIAL_DATE_MIN          (0u)
#define RTC_INITIAL_DATE_HOUR         (12u)
#define RTC_INITIAL_DATE_HOUR_FORMAT  (CY_RTC_24_HOURS)
#define RTC_INITIAL_DATE_DOW          (1u)
#define RTC_INITIAL_DATE_DOM          (4u)
#define RTC_INITIAL_DATE_MONTH        (9u)
#define RTC_INITIAL_DATE_YEAR         (17u)

/** The instance-specific configuration structure. This should be used in the 
*  associated RTC_Init() function.
*/
const cy_stc_rtc_config_t RTC_config =
{
    /* Initiate time and date */
    .sec       = RTC_INITIAL_DATE_SEC, 
    .min       = RTC_INITIAL_DATE_MIN,
    .hour      = RTC_INITIAL_DATE_HOUR,
    .hrMode    = RTC_INITIAL_DATE_HOUR_FORMAT,
    .dayOfWeek = RTC_INITIAL_DATE_DOW,
    .date      = RTC_INITIAL_DATE_DOM,
    .month     = RTC_INITIAL_DATE_MONTH,
    .year      = RTC_INITIAL_DATE_YEAR,
};

/* This configuration will generate interrupt in every minute whenever
   SECONDS match. 
   Example: 00:05, 01:05, 02:05...
   Note: Alarm interrupt will not happen based on every SECONDS 
   configuration.
   Example: 00:05, 00:10, 00:15... */ 
const cy_stc_rtc_alarm_t alarm = 
{
    .sec          = 05u,
    .sec_en       = CY_RTC_ALARM_ENABLE,
    .min          = 00u,
    .min_en       = CY_RTC_ALARM_DISABLE,
    .hour         = 00u,
    .hour_en      = CY_RTC_ALARM_DISABLE,
    .dayOfWeek    = 01u,
    .dayOfWeek_en = CY_RTC_ALARM_DISABLE,
    .date         = 04u,
    .date_en      = CY_RTC_ALARM_DISABLE,
    .month        = 11u,
    .month_en     = CY_RTC_ALARM_DISABLE,
    .alm_en       = CY_RTC_ALARM_ENABLE
};

const cy_str_rtc_dst_t RTC_dstConfig;

cy_stc_gpio_pin_config_t user_led0_port_pin_cfg =
{
    .outVal    = 0u,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = USER_LED_PIN_MUX,
    .intEdge   = 0u,
    .intMask   = 0u,
    .vtrip     = 0u,
    .slewRate  = 0u,
    .driveSel  = 0u,
};

// Initialize all SYSTEM_INT_STRUCTS
const cy_stc_sysint_irq_t irq_cfg =
{
    .sysIntSrc = srss_interrupt_backup_IRQn,
    .intIdx    = CPUIntIdx2_IRQn,
    .isEnabled = true,
};


/** RTC_rtcDstStatus variable which is for DST feature and is 
*  called in Cy_RTC_Interrupt() PDL driver function. This variable is 
*  defined as true if DST is enabled and as false if DST is disabled
*/
bool RTC_rtcDstStatus = false;

void RTC_Handler(void)
{
    Cy_Rtc_Interrupt(&RTC_dstConfig, RTC_rtcDstStatus);
}

void Cy_Rtc_Alarm1Interrupt(void)
{
   /* Clear any pending interrupts */
   Cy_Rtc_ClearInterrupt(CY_RTC_INTR_ALARM1);
}

int main(void)
{
    uint8_t count = 0u;
    
    __enable_irq(); /* Enable global interrupts. */
    SystemInit();

    /* Example had been originally tested with "cache off", so ensure that caches are turned off (may have been enabled by new startup.c module) */
    SCB_DisableICache(); // Disables and invalidates instruction cache
    SCB_DisableDCache(); // Disables, cleans and invalidates data cache

    Cy_GPIO_Pin_Init(USER_LED_PORT, USER_LED_PIN, &user_led0_port_pin_cfg);

    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, RTC_Handler);
    NVIC_SetPriority(irq_cfg.intIdx, 3ul);
    NVIC_EnableIRQ(irq_cfg.intIdx);
    
    /* Set the ILO_0 as the clock source to the RTC block */
    Cy_Rtc_clock_source(CY_RTC_CLK_SRC_ILO_0);
    
    /* Wait for alarm to be set */
    while(Cy_Rtc_Init(&RTC_config) != CY_RET_SUCCESS);

   /* Clear any pending interrupts */
    Cy_Rtc_ClearInterrupt(CY_RTC_INTR_ALARM1);

    /*Configures the source (Alarm1) that trigger the interrupts */
    Cy_Rtc_SetInterruptMask(CY_RTC_INTR_ALARM1);

    /* Wait for alarm to be set */
    while(Cy_Rtc_SetAlarmDateAndTime(&alarm,CY_RTC_ALARM_1) != CY_RET_SUCCESS);

    for(;;) 
    {
        // wait 0.05 [s]
        Cy_SysTick_DelayInUs(50000ul);

        Cy_GPIO_Inv(USER_LED_PORT, USER_LED_PIN);

        count++;
        if (count > 200u)
        {
            Cy_GPIO_Write(USER_LED_PORT, USER_LED_PIN, 0u);
            count = 0u;
            Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
        }
    }
}

/* [] END OF FILE */
