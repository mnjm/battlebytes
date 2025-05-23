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

cy_stc_gpio_pin_config_t user_led_port_pin_cfg =
{
    .outVal    = 0x00,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = CY_BSP_BB_USER_LED0_PIN_MUX,
    .intEdge   = 0,
    .intMask   = 0,
    .vtrip     = 0,
    .slewRate  = 0,
    .driveSel  = 0,
    .vregEn    = 0,
    .ibufMode  = 0,
    .vtripSel  = 0,
    .vrefSel   = 0,
    .vohSel    = 0,
};

void irqFaultReport0Handler(void)
{
    cy_en_sysflt_source_t status;
    uint32_t faultData;

    /* Clear Interrupt flag */
    Cy_SysFlt_ClearInterrupt(FAULT_STRUCT0);
    
    status = Cy_SysFlt_GetErrorSource(FAULT_STRUCT0);

    if(status != CY_SYSFLT_NO_FAULT)
    {
        if(status == CY_SYSFLT_SRSS_FAULT_MCWDT0) // MCWDT0 fault
        {
            faultData = Cy_SysFlt_GetData0(FAULT_STRUCT0);
            if(faultData & 0x00000001)
            {
                 // subcounter 0 lower limit fault
            }
            else if(faultData & 0x00000002)
            {
                // won't reach here
                Cy_GPIO_Inv(CY_BSP_BB_USER_LED0_PORT, CY_BSP_BB_USER_LED0_PIN); // subcounter 0 upper limit fault
            }
            else if(faultData & 0x00000004)
            {
                 // subcounter 1 lower limit fault
            }
            else if(faultData & 0x00000008)
            {
                // Immediately after LED1 light up, the MCU will be reset.
                Cy_GPIO_Inv(CY_BSP_BB_USER_LED1_PORT, CY_BSP_BB_USER_LED1_PIN); // subcounter 1 upper limit fault
            }
        }
    }

    Cy_SysFlt_ClearStatus(FAULT_STRUCT0);
}

int main(void)
{
    SystemInit();

    __enable_irq(); /* Enable global interrupts. */

    /*********************************************************************/
    /*****                 Fault report settings                     *****/
    /*********************************************************************/
    {
        Cy_SysFlt_ClearStatus(FAULT_STRUCT0); // clear status (typically this process is done by boot code)
        Cy_SysFlt_SetMaskByIdx(FAULT_STRUCT0, CY_SYSFLT_SRSS_FAULT_MCWDT0); // enalbe Fault MCWDT #0
        Cy_SysFlt_SetInterruptMask(FAULT_STRUCT0);
    }

    /*********************************************************************/
    /*****                     Port settings                         *****/
    /*********************************************************************/
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    user_led_port_pin_cfg.hsiom  = CY_BSP_BB_USER_LED0_PIN_MUX;
    Cy_GPIO_Pin_Init(CY_BSP_BB_USER_LED0_PORT, CY_BSP_BB_USER_LED0_PIN, &user_led_port_pin_cfg);

    user_led_port_pin_cfg.hsiom  = CY_BSP_BB_USER_LED1_PIN_MUX;
    Cy_GPIO_Pin_Init(CY_BSP_BB_USER_LED1_PORT, CY_BSP_BB_USER_LED1_PIN, &user_led_port_pin_cfg);

    user_led_port_pin_cfg.hsiom  = CY_BSP_BB_USER_LED2_PIN_MUX;
    Cy_GPIO_Pin_Init(CY_BSP_BB_USER_LED2_PORT, CY_BSP_BB_USER_LED2_PIN, &user_led_port_pin_cfg);

    // Not Lighting LED2 indicates reset occurred.
    Cy_SysTick_DelayInUs(1000000);

    Cy_GPIO_Inv(CY_BSP_BB_USER_LED2_PORT, CY_BSP_BB_USER_LED2_PIN);

    /*********************************************************************/
    /*****                   Interrupt setting                       *****/
    /*********************************************************************/
    {
        cy_stc_sysint_irq_t irq_cfg;
        irq_cfg = (cy_stc_sysint_irq_t){
            .sysIntSrc  = cpuss_interrupts_fault_0_IRQn,
            .intIdx     = CPUIntIdx2_IRQn,
            .isEnabled  = true,
        };
        Cy_SysInt_InitIRQ(&irq_cfg);
        Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, irqFaultReport0Handler);
        NVIC_SetPriority(irq_cfg.intIdx, 0);
        NVIC_EnableIRQ(irq_cfg.intIdx);
    }

    /*********************************************************************/
    /*****                        Set actions                        *****/
    /*********************************************************************/
    Cy_MCWDT_SetLowerAction(MCWDT0, CY_MCWDT_COUNTER0, CY_MCWDT_ACTION_NONE);
    Cy_MCWDT_SetUpperAction(MCWDT0, CY_MCWDT_COUNTER0, CY_MCWDT_ACTION_NONE);
    Cy_MCWDT_SetWarnAction(MCWDT0, CY_MCWDT_COUNTER0, CY_MCWDT_WARN_ACTION_NONE);

    Cy_MCWDT_SetLowerAction(MCWDT0, CY_MCWDT_COUNTER1, CY_MCWDT_ACTION_NONE);
    Cy_MCWDT_SetUpperAction(MCWDT0, CY_MCWDT_COUNTER1, CY_MCWDT_ACTION_FAULT_THEN_RESET); // sub counter 1 upper limit causes to reset
    Cy_MCWDT_SetWarnAction(MCWDT0, CY_MCWDT_COUNTER1, CY_MCWDT_WARN_ACTION_NONE);

    Cy_MCWDT_SetSubCounter2Action(MCWDT0, CY_MCWDT_CNT2_ACTION_NONE);

    /*********************************************************************/
    /*****                      Set limit values                     *****/
    /*********************************************************************/
    Cy_MCWDT_SetLowerLimit(MCWDT0, CY_MCWDT_COUNTER0, 0, 0);
    Cy_MCWDT_SetWarnLimit(MCWDT0, CY_MCWDT_COUNTER0, 0, 0);
    Cy_MCWDT_SetUpperLimit(MCWDT0, CY_MCWDT_COUNTER0, 16000, 0); // about 0.5 s

    Cy_MCWDT_SetLowerLimit(MCWDT0, CY_MCWDT_COUNTER1, 0, 0);
    Cy_MCWDT_SetWarnLimit(MCWDT0, CY_MCWDT_COUNTER1, 0, 0);
    Cy_MCWDT_SetUpperLimit(MCWDT0, CY_MCWDT_COUNTER1, 32000, 0); // about 1 s

    Cy_MCWDT_SetToggleBit(MCWDT0, CY_MCWDT_CNT2_MONITORED_BIT15); // means 32768 count period


    /*********************************************************************/
    /*****                        Set options                        *****/
    /*********************************************************************/
    Cy_MCWDT_SetAutoService(MCWDT0, CY_MCWDT_COUNTER0, 0u); // disable
    Cy_MCWDT_SetAutoService(MCWDT0, CY_MCWDT_COUNTER1, 0u); // disable

    Cy_MCWDT_SetSleepDeepPause(MCWDT0, CY_MCWDT_COUNTER0, 1u); // enable
    Cy_MCWDT_SetSleepDeepPause(MCWDT0, CY_MCWDT_COUNTER1, 1u); // enable
    Cy_MCWDT_SetSleepDeepPause(MCWDT0, CY_MCWDT_COUNTER2, 1u); // enable

    Cy_MCWDT_SetDebugRun(MCWDT0, CY_MCWDT_COUNTER0, 1u); // enable
    Cy_MCWDT_SetDebugRun(MCWDT0, CY_MCWDT_COUNTER1, 1u); // enable
    Cy_MCWDT_SetDebugRun(MCWDT0, CY_MCWDT_COUNTER2, 1u); // enable

    Cy_MCWDT_Enable(MCWDT0, CY_MCWDT_CTR_Msk, 0u); // enable all counter

    while(Cy_MCWDT_GetEnabledStatus(MCWDT0, CY_MCWDT_COUNTER0) != 1u);
    while(Cy_MCWDT_GetEnabledStatus(MCWDT0, CY_MCWDT_COUNTER1) != 1u);
    while(Cy_MCWDT_GetEnabledStatus(MCWDT0, CY_MCWDT_COUNTER2) != 1u);
    
    for(;;)
    {
#if 0 // If you want to trigger reset, remove this code.
        uint16_t lowerCounter = Cy_MCWDT_GetLowerLimit(MCWDT0, CY_MCWDT_COUNTER1);
        uint16_t countValue = (uint16_t)Cy_MCWDT_GetCount(MCWDT0, CY_MCWDT_COUNTER1);

        if(countValue > lowerCounter)
        {
            Cy_MCWDT_ClearWatchdog(MCWDT0, CY_MCWDT_COUNTER1);
            Cy_MCWDT_WaitForCounterReset(MCWDT0, CY_MCWDT_COUNTER1);
        }
#endif
    }
}



/* [] END OF FILE */
