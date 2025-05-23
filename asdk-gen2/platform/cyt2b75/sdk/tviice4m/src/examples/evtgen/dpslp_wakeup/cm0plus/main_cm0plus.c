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

#if (CY_USE_PSVP == 1)
    #define USER_LED_PORT           CY_LED0_PORT
    #define USER_LED_PIN            CY_LED0_PIN
    #define USER_LED_PIN_MUX        CY_LED0_PIN_MUX
#else
    #define USER_LED_PORT           CY_BSP_USER_LED1_PORT
    #define USER_LED_PIN            CY_BSP_USER_LED1_PIN
    #define USER_LED_PIN_MUX        CY_BSP_USER_LED1_PIN_MUX 
#endif

cy_stc_gpio_pin_config_t user_led0_port_pin_cfg    = {
    .outVal = 0x00,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = USER_LED_PIN_MUX,
    .intEdge = 0,
    .intMask = 0,
    .vtrip = 0,
    .slewRate = 0,
    .driveSel = 0,
    .vregEn = 0,
    .ibufMode = 0,
    .vtripSel = 0,
    .vrefSel = 0,
    .vohSel = 0,
};

cy_stc_evtgen_struct_context_t evtgenStruct0Context;
cy_stc_evtgen_struct_config_t evtgenTestStructureConfig;
cy_stc_evtgen_config_t evtgenTestConfig;

uint8_t g_flag = 0;

void irqEVTGEN_sleep(void)
{
    Cy_Evtgen_ClearStructInterruptDeepSleep(EVTGEN0,0);
}

void irqEVTGEN_active(void)
{
    Cy_Evtgen_ClearStructInterrupt(EVTGEN0,0);
    Cy_Evtgen_DeinitializeCompStruct(EVTGEN0,0);
    Cy_Evtgen_InitializeCompStruct(EVTGEN0,0, &evtgenTestStructureConfig, &evtgenStruct0Context);
    g_flag = 1;
}

int main(void)
{
    SystemInit();

    cy_stc_sysint_irq_t irq_cfg;

    __enable_irq(); /* Enable global interrupts. */
    /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableApplCore(CY_CORTEX_M4_APPL_ADDR); 

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    Cy_GPIO_Pin_Init(USER_LED_PORT, USER_LED_PIN, &user_led0_port_pin_cfg);
    Cy_GPIO_Write(USER_LED_PORT, USER_LED_PIN, 1);

    /*******************************************/
    /*        Deinitialize peripherals         */
    /*******************************************/
    Cy_Evtgen_DeinitializeCompStruct(EVTGEN0,0);
    Cy_Evtgen_Deinitialize(EVTGEN0);
    NVIC_ClearPendingIRQ(CPUIntIdx3_IRQn);

    /*******************************************/
    /*  Interrupt setting for Event Generator  */
    /*******************************************/
    irq_cfg.sysIntSrc  = evtgen_0_interrupt_dpslp_IRQn;
    irq_cfg.intIdx     = CPUIntIdx3_IRQn;
    irq_cfg.isEnabled  = true;
    Cy_SysInt_InitIRQ(&irq_cfg);
	Cy_SysInt_SetSystemIrqVector(evtgen_0_interrupt_dpslp_IRQn, irqEVTGEN_sleep);
	
    irq_cfg.sysIntSrc  = evtgen_0_interrupt_IRQn;
    irq_cfg.intIdx     = CPUIntIdx3_IRQn;
    irq_cfg.isEnabled  = true;
    Cy_SysInt_InitIRQ(&irq_cfg);
	Cy_SysInt_SetSystemIrqVector(evtgen_0_interrupt_IRQn, irqEVTGEN_active);
    NVIC_EnableIRQ(CPUIntIdx3_IRQn);

    /*********************/
    /*  Enable clk_hf1   */
    /*********************/
  #if (CY_USE_PSVP == 1)
    Cy_SysClk_HfClkEnable(CY_SYSCLK_HFCLK_1);
    Cy_SysClk_HfClockSetSource(CY_SYSCLK_HFCLK_1, CY_SYSCLK_HFCLK_IN_CLKPATH0);  
  #else
    Cy_SysClk_ClkPathSetSource(2u, CY_SYSCLK_CLKPATH_IN_IMO);
    Cy_SysClk_HfClkEnable(CY_SYSCLK_HFCLK_1);
    Cy_SysClk_HfClockSetSource(CY_SYSCLK_HFCLK_1, CY_SYSCLK_HFCLK_IN_CLKPATH2);
  #endif

    /*******************************************/
    /*  Initialize and start Event generator   */
    /*******************************************/
  #if (CY_USE_PSVP == 1)
    evtgenTestConfig.frequencyRef = 24000000; // clk_ref = clk_hf1 = CLK_PATH0 -> 24,000,000 for PSVP test.
    evtgenTestConfig.frequencyLf = 32000;     // clk_lf = 32,000 for PSVP test
  #else
    evtgenTestConfig.frequencyRef = 8000000; //  clk_ref = clk_hf1 = CLK_PATH2 (IMO) -> 8,000,000 for silicon
    evtgenTestConfig.frequencyLf = 32000;     // clk_lf = 32,000 for silicon
  #endif
    
    evtgenTestConfig.frequencyTick = 1000000; // Setting 1,000,000 Hz for event generator clock (clk_ref_div)
    evtgenTestConfig.ratioControlMode = CY_EVTGEN_RATIO_CONTROL_SW;
    evtgenTestConfig.ratioValueDynamicMode = CY_EVTGEN_RATIO_DYNAMIC_MODE0;
    Cy_Evtgen_Initialize(EVTGEN0,&evtgenTestConfig);

    /*******************************************/
    /*  Initialize comparator structure 0      */
    /*******************************************/
    evtgenTestStructureConfig.functionalitySelection = CY_EVTGEN_DEEPSLEEP_FUNCTIONALITY;
    evtgenTestStructureConfig.triggerOutEdge = CY_EVTGEN_EDGE_SENSITIVE;
    evtgenTestStructureConfig.valueDeepSleepComparator = 2000000; // In active functionality, this value is used for making period of interrupts/triggers
                                                                  // ,000,000 / 1,000,000 (clk_ref_div) = 1[s]
    evtgenTestStructureConfig.valueActiveComparator    = 4000000; // In active functionality, this value is used for making period of interrupts/triggers
                                                                  // 1,000,000 / 1,000,000 (clk_ref_div) = 1[s]
    Cy_Evtgen_InitializeCompStruct(EVTGEN0,0, &evtgenTestStructureConfig, &evtgenStruct0Context);
    /* Put the system to DeeSleep */

    g_flag = 1;

    while(1)
    {
        while(g_flag == 0);
        g_flag = 0;
        Cy_GPIO_Write(USER_LED_PORT, USER_LED_PIN, 0);
        Cy_SysPm_DeepSleep((cy_en_syspm_waitfor_t)CY_SYSPM_WAIT_FOR_INTERRUPT);
        Cy_GPIO_Write(USER_LED_PORT, USER_LED_PIN, 1);
    }


}

/* [] END OF FILE */
