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
    #define USER_LED1_PORT           CY_LED0_PORT
    #define USER_LED1_PIN            CY_LED0_PIN
    #define USER_LED1_PIN_MUX        CY_LED0_PIN_MUX

    #define USER_LED2_PORT           CY_LED1_PORT
    #define USER_LED2_PIN            CY_LED1_PIN
    #define USER_LED2_PIN_MUX        CY_LED1_PIN_MUX

    #define USER_LED3_PORT           CY_LED2_PORT 
    #define USER_LED3_PIN            CY_LED2_PIN
    #define USER_LED3_PIN_MUX        CY_LED2_PIN_MUX
    
    #define USER_BUTTON1_PORT        CY_BUTTON1_PORT
    #define USER_BUTTON1_PIN         CY_BUTTON1_PIN
    #define USER_BUTTON1_PIN_MUX     CY_BUTTON1_PIN_MUX
    #define USER_BUTTON1_IRQN        CY_BUTTON1_IRQN

    #define USER_BUTTON2_PORT        CY_BUTTON2_PORT
    #define USER_BUTTON2_PIN         CY_BUTTON2_PIN
    #define USER_BUTTON2_PIN_MUX     CY_BUTTON2_PIN_MUX
    #define USER_BUTTON2_IRQN        CY_BUTTON2_IRQN

    #define USER_BUTTON3_PORT        CY_BUTTON3_PORT
    #define USER_BUTTON3_PIN         CY_BUTTON3_PIN
    #define USER_BUTTON3_PIN_MUX     CY_BUTTON3_PIN_MUX
    #define USER_BUTTON3_IRQN        CY_BUTTON3_IRQN

    #define USER_BUTTON4_PORT        CY_BUTTON4_PORT
    #define USER_BUTTON4_PIN         CY_BUTTON4_PIN
    #define USER_BUTTON4_PIN_MUX     CY_BUTTON4_PIN_MUX
    #define USER_BUTTON4_IRQN        CY_BUTTON4_IRQN
#else
    #define USER_LED1_PORT           CY_USER_LED1_PORT
    #define USER_LED1_PIN            CY_USER_LED1_PIN
    #define USER_LED1_PIN_MUX        CY_USER_LED1_PIN_MUX

    #define USER_LED2_PORT           CY_USER_LED2_PORT
    #define USER_LED2_PIN            CY_USER_LED2_PIN
    #define USER_LED2_PIN_MUX        CY_USER_LED2_PIN_MUX

    #define USER_LED3_PORT           CY_USER_LED3_PORT
    #define USER_LED3_PIN            CY_USER_LED3_PIN
    #define USER_LED3_PIN_MUX        CY_USER_LED3_PIN_MUX

    #define USER_BUTTON1_PORT        CY_USER_BUTTON_UP_PORT
    #define USER_BUTTON1_PIN         CY_USER_BUTTON_UP_PIN
    #define USER_BUTTON1_PIN_MUX     CY_USER_BUTTON_UP_PIN_MUX
    #define USER_BUTTON1_IRQN        CY_USER_BUTTON_UP_IRQN

    #define USER_BUTTON2_PORT        CY_USER_BUTTON_LEFT_PORT
    #define USER_BUTTON2_PIN         CY_USER_BUTTON_LEFT_PIN
    #define USER_BUTTON2_PIN_MUX     CY_USER_BUTTON_LEFT_PIN_MUX
    #define USER_BUTTON2_IRQN        CY_USER_BUTTON_LEFT_IRQN

    #define USER_BUTTON3_PORT        CY_USER_BUTTON_RIGHT_PORT
    #define USER_BUTTON3_PIN         CY_USER_BUTTON_RIGHT_PIN
    #define USER_BUTTON3_PIN_MUX     CY_USER_BUTTON_RIGHT_PIN_MUX
    #define USER_BUTTON3_IRQN        CY_USER_BUTTON_RIGHT_IRQN

    #define USER_BUTTON4_PORT        CY_USER_BUTTON_DOWN_PORT
    #define USER_BUTTON4_PIN         CY_USER_BUTTON_DOWN_PIN
    #define USER_BUTTON4_PIN_MUX     CY_USER_BUTTON_DOWN_PIN_MUX
    #define USER_BUTTON4_IRQN        CY_USER_BUTTON_DOWN_IRQN
#endif

// Button Reset Associations    Indication      LED2    LED1    LED0
#define BUTTON1_WDT             1       //      0       0       1            
#define BUTTON2_ACT_FAULT       2       //      0       1       0
#define BUTTON3_SOFT            3       //      0       1       1
#define BUTTON4_MCWDT0          4       //      1       0       0
// XRES                                 //      1       0       1

// Static Function Declarations
static void Sample_WDTInitForReset(void);
static void Sample_MCWDT0InitForReset(void);
static void Sample_ButtonInitForReset(void);
static void Sample_ActiveMPUAccessFaultInitForReset(void);

// GPIO configuration for LEDs and Buttons 
cy_stc_gpio_pin_config_t user_led0_port_pin_cfg =             
{                                                  
    .outVal = 0x00,                                
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,    
    .hsiom = USER_LED1_PIN_MUX,                           
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

cy_stc_gpio_pin_config_t user_led1_port_pin_cfg =             
{                                                  
    .outVal = 0x00,                                
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,    
    .hsiom = USER_LED2_PIN_MUX,                           
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

cy_stc_gpio_pin_config_t user_led2_port_pin_cfg =             
{                                                  
    .outVal = 0x00,                                
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,    
    .hsiom = USER_LED3_PIN_MUX,                           
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

cy_stc_gpio_pin_config_t user_button1_port_pin_cfg = 
{
    .outVal = 0x00,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom = USER_BUTTON1_PIN_MUX,
    .intEdge = CY_GPIO_INTR_FALLING,
    .intMask = 1,
    .vtrip = 0,
    .slewRate = 0,
    .driveSel = 0,
    .vregEn = 0,
    .ibufMode = 0,
    .vtripSel = 0,
    .vrefSel = 0,
    .vohSel = 0,
};

cy_stc_gpio_pin_config_t user_button2_port_pin_cfg = 
{
    .outVal = 0x00,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom = USER_BUTTON2_PIN_MUX,
    .intEdge = CY_GPIO_INTR_FALLING,
    .intMask = 1,
    .vtrip = 0,
    .slewRate = 0,
    .driveSel = 0,
    .vregEn = 0,
    .ibufMode = 0,
    .vtripSel = 0,
    .vrefSel = 0,
    .vohSel = 0,
};

cy_stc_gpio_pin_config_t user_button3_port_pin_cfg = 
{
    .outVal = 0x00,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom = USER_BUTTON3_PIN_MUX,
    .intEdge = CY_GPIO_INTR_FALLING,
    .intMask = 1,
    .vtrip = 0,
    .slewRate = 0,
    .driveSel = 0,
    .vregEn = 0,
    .ibufMode = 0,
    .vtripSel = 0,
    .vrefSel = 0,
    .vohSel = 0,
};

cy_stc_gpio_pin_config_t user_button4_port_pin_cfg = 
{
    .outVal = 0x00,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom = USER_BUTTON4_PIN_MUX,
    .intEdge = CY_GPIO_INTR_FALLING,
    .intMask = 1,
    .vtrip = 0,
    .slewRate = 0,
    .driveSel = 0,
    .vregEn = 0,
    .ibufMode = 0,
    .vtripSel = 0,
    .vrefSel = 0,
    .vohSel = 0,
};

uint8_t sButtonNumber = 0;
volatile uint32_t testRead0;
volatile uint32_t testRead1;

// Button Interrupt Handler
void ButtonIntHandler(void)
{
    uint32_t intStatus;

    /* If button1 falling edge detected */
    intStatus = Cy_GPIO_GetInterruptStatusMasked(USER_BUTTON1_PORT, USER_BUTTON1_PIN);
    if (intStatus != 0uL)
    {
        Cy_GPIO_ClearInterrupt(USER_BUTTON1_PORT, USER_BUTTON1_PIN);
        sButtonNumber = BUTTON1_WDT;
    }
    
    /* If button2 rising edge detected */
    intStatus = Cy_GPIO_GetInterruptStatusMasked(USER_BUTTON2_PORT, USER_BUTTON2_PIN);
    if (intStatus != 0uL)
    {
        Cy_GPIO_ClearInterrupt(USER_BUTTON2_PORT, USER_BUTTON2_PIN);
        sButtonNumber = BUTTON2_ACT_FAULT;
    }
    
    /* If button3 falling edge detected */
    intStatus = Cy_GPIO_GetInterruptStatusMasked(USER_BUTTON3_PORT, USER_BUTTON3_PIN);
    if (intStatus != 0uL)
    {
        Cy_GPIO_ClearInterrupt(USER_BUTTON3_PORT, USER_BUTTON3_PIN);
        sButtonNumber = BUTTON3_SOFT;
    }
    
    /* If button5 rising edge detected */
    intStatus = Cy_GPIO_GetInterruptStatusMasked(USER_BUTTON4_PORT, USER_BUTTON4_PIN);
    if (intStatus != 0uL)
    {
        Cy_GPIO_ClearInterrupt(USER_BUTTON4_PORT, USER_BUTTON4_PIN);
        sButtonNumber = BUTTON4_MCWDT0;
    }
}

int main(void)
{
    SystemInit();

    uint32_t tRstReason = 0u;
    bool tResetPerformed = false;
    
    __enable_irq(); /* Enable global interrupts. */
    
    /* Enable CM7_0/1. CY_CORTEX_M7_APPL_ADDR is calculated in linker script, check it in case of problems. */
    Cy_SysEnableApplCore(CORE_CM7_0, CY_CORTEX_M7_0_APPL_ADDR);
    //Cy_SysEnableApplCore(CORE_CM7_1, CY_CORTEX_M7_1_APPL_ADDR); 
    
    /* Configure the 3 LEDs */
    Cy_GPIO_Pin_Init(USER_LED1_PORT, USER_LED1_PIN, &user_led0_port_pin_cfg);
    Cy_GPIO_Pin_Init(USER_LED2_PORT, USER_LED2_PIN, &user_led1_port_pin_cfg);
    Cy_GPIO_Pin_Init(USER_LED3_PORT, USER_LED3_PIN, &user_led2_port_pin_cfg);
    
    /* Read the RESET reason */
    tRstReason = Cy_SysReset_GetResetReason();
    
    /* activate LEDs as a combination based on type of a reset and lock in 
       a while loop */
    if( ( tRstReason & CY_SYSRESET_WDT ) == CY_SYSRESET_WDT )
    {
        // '001'
        Cy_GPIO_Write(USER_LED1_PORT, USER_LED1_PIN, 1);
        Cy_GPIO_Write(USER_LED2_PORT, USER_LED2_PIN, 0);
        Cy_GPIO_Write(USER_LED3_PORT, USER_LED3_PIN, 0);
        
        tResetPerformed = true;
    }
    else if( ( tRstReason & CY_SYSRESET_ACT_FAULT ) == CY_SYSRESET_ACT_FAULT )
    {
        // '010'
        Cy_GPIO_Write(USER_LED1_PORT, USER_LED1_PIN, 0);
        Cy_GPIO_Write(USER_LED2_PORT, USER_LED2_PIN, 1);
        Cy_GPIO_Write(USER_LED3_PORT, USER_LED3_PIN, 0);  
      
        tResetPerformed = true;
    }
    else if( ( tRstReason & CY_SYSRESET_SOFT ) == CY_SYSRESET_SOFT )
    {
        // '011'
        Cy_GPIO_Write(USER_LED1_PORT, USER_LED1_PIN, 1);
        Cy_GPIO_Write(USER_LED2_PORT, USER_LED2_PIN, 1);
        Cy_GPIO_Write(USER_LED3_PORT, USER_LED3_PIN, 0);         
        
        tResetPerformed = true;
    }
    else if( ( tRstReason & CY_SYSRESET_MCWDT0 ) == CY_SYSRESET_MCWDT0 )
    {
        // '100'
        Cy_GPIO_Write(USER_LED1_PORT, USER_LED1_PIN, 0);
        Cy_GPIO_Write(USER_LED2_PORT, USER_LED2_PIN, 0);
        Cy_GPIO_Write(USER_LED3_PORT, USER_LED3_PIN, 1);       
        
        tResetPerformed = true;
    }
    else if( ( tRstReason & CY_SYSRESET_XRES ) == CY_SYSRESET_XRES )
    {
        // '101'
        Cy_GPIO_Write(USER_LED1_PORT, USER_LED1_PIN, 1);
        Cy_GPIO_Write(USER_LED2_PORT, USER_LED2_PIN, 0);
        Cy_GPIO_Write(USER_LED3_PORT, USER_LED3_PIN, 1);  
        
        // tResetPerformed = true;
        Cy_SysReset_ClearAllResetReasons();
    }
    
    // Block here, if a reset was initiated during the last iteration
    if( tResetPerformed == true )
    {
        // Clear all reset reasons
        Cy_SysReset_ClearAllResetReasons();
      
        while(1);
    }

    // Buttons Init
    Sample_ButtonInitForReset();
    
    for(;;)
    {
        switch( sButtonNumber )
        {
        case BUTTON1_WDT:
          // WDT Init for Reset
          Sample_WDTInitForReset();
          // Reset the button state
          sButtonNumber = 0;
          break;
          
        case BUTTON2_ACT_FAULT:
          Sample_ActiveMPUAccessFaultInitForReset();
          // Reset the button state
          sButtonNumber = 0;
          break;
          
        case BUTTON3_SOFT: // Soft Reset
          NVIC_SystemReset();
          // Reset the button state
          sButtonNumber = 0;
          break;
          
        case BUTTON4_MCWDT0:
          // MCWDT Init for Reset
          Sample_MCWDT0InitForReset();
          // Reset the button state
          sButtonNumber = 0;
          break;
          
        default: // Do Nothing
          break;          
        }

        
    }
}

static void Sample_WDTInitForReset(void)
{
    /*-----------------------*/
    /* Configuration for WDT */
    /*-----------------------*/
    Cy_WDT_Init();                      /* Upper Limit: 1sec and reset */
    Cy_WDT_Unlock();
    Cy_WDT_SetUpperLimit(64000);        /* Upper Limit: 2sec (override) */
    Cy_WDT_SetDebugRun(CY_WDT_ENABLE);  /* This is necessary when using debugger */
    Cy_WDT_Lock();
    Cy_WDT_Enable();
}

static void Sample_MCWDT0InitForReset(void)
{
    /*********************************************************************/
    /*****      Set Core DeepSleep of which can pause counter        *****/
    /*********************************************************************/
    Cy_MCWDT_CpuSelectForDpSlpPauseAction(MCWDT0, CY_MCWDT_PAUSED_BY_DPSLP_CM0);

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
    Cy_MCWDT_SetUpperLimit(MCWDT0, CY_MCWDT_COUNTER0, 100, 0);

    Cy_MCWDT_SetLowerLimit(MCWDT0, CY_MCWDT_COUNTER1, 0, 0);
    Cy_MCWDT_SetWarnLimit(MCWDT0, CY_MCWDT_COUNTER1, 0, 0);
    Cy_MCWDT_SetUpperLimit(MCWDT0, CY_MCWDT_COUNTER1, 64000, 0);  /* 2 sec when clk_lf = 32KHz */

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

    // Enable all the counters
    Cy_MCWDT_Enable(MCWDT0, CY_MCWDT_CTR_Msk, 0u);

    while(Cy_MCWDT_GetEnabledStatus(MCWDT0, CY_MCWDT_COUNTER0) != 1u);
    while(Cy_MCWDT_GetEnabledStatus(MCWDT0, CY_MCWDT_COUNTER1) != 1u);
    while(Cy_MCWDT_GetEnabledStatus(MCWDT0, CY_MCWDT_COUNTER2) != 1u);
}

static void Sample_ButtonInitForReset(void)
{
    // Mapping Buttons for a reset case
    Cy_GPIO_Pin_Init(USER_BUTTON1_PORT, USER_BUTTON1_PIN, &user_button1_port_pin_cfg);
    Cy_GPIO_Pin_Init(USER_BUTTON2_PORT, USER_BUTTON2_PIN, &user_button2_port_pin_cfg);
    Cy_GPIO_Pin_Init(USER_BUTTON3_PORT, USER_BUTTON3_PIN, &user_button3_port_pin_cfg);
    Cy_GPIO_Pin_Init(USER_BUTTON4_PORT, USER_BUTTON4_PIN, &user_button4_port_pin_cfg);
    
    /* Setup GPIO for BUTTON1 interrupt */
    cy_stc_sysint_irq_t irq_cfg =
    {
        .sysIntSrc  = USER_BUTTON1_IRQN,
        .intIdx     = CPUIntIdx2_IRQn,
        .isEnabled  = true,
    };
    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, ButtonIntHandler);

    /* Setup GPIO for BUTTON2/BUTTON3 interrupt */
    irq_cfg.sysIntSrc = USER_BUTTON2_IRQN;
    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, ButtonIntHandler);
    
    /* Setup GPIO for BUTTON4 interrupt */
    irq_cfg.sysIntSrc = USER_BUTTON4_IRQN;
    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, ButtonIntHandler);

    NVIC_SetPriority(CPUIntIdx2_IRQn, 3);
    NVIC_ClearPendingIRQ(CPUIntIdx2_IRQn);
    NVIC_EnableIRQ(CPUIntIdx2_IRQn);
}

static void Sample_ActiveMPUAccessFaultInitForReset(void)
{
    cy_stc_smpu_cfg_t smpuStructTemp = { 0 };
    cy_stc_sysflt_t tFlt_Temp = {0};
   
    /*********************************************************************/
    /*****                 Fault report settings                     *****/
    /*********************************************************************/
    {
        Cy_SysFlt_ClearStatus(FAULT_STRUCT0); // clear status (typically this process is done by boot code)
        Cy_SysFlt_SetMaskByIdx(FAULT_STRUCT0, CY_SYSFLT_MPU_0); // enalbe Fault Master 0 (CM0+) MPU
        Cy_SysFlt_SetInterruptMask(FAULT_STRUCT0);
    }
    
    tFlt_Temp.ResetEnable = true;
    tFlt_Temp.OutputEnable = false;
    tFlt_Temp.TriggerEnable = false;
    Cy_SysFlt_Init(FAULT_STRUCT0, &tFlt_Temp);

    {
        cy_stc_sysint_irq_t irq_cfg;
        irq_cfg = (cy_stc_sysint_irq_t){
            .sysIntSrc  = cpuss_interrupts_fault_0_IRQn,
            .intIdx     = CPUIntIdx2_IRQn,
            .isEnabled  = true,
        };
        irq_cfg.sysIntSrc = cpuss_interrupts_fault_0_IRQn;
        Cy_SysInt_InitIRQ(&irq_cfg);
        NVIC_SetPriority(CPUIntIdx2_IRQn, 0);
        NVIC_ClearPendingIRQ(CPUIntIdx2_IRQn);
        NVIC_EnableIRQ(CPUIntIdx2_IRQn);
    }

    /* 1. Setting for MSx_CTL */
    if(Cy_Prot_ConfigBusMaster(CPUSS_MS_ID_CM0, 1 /* privireged */, 0 /* secure */, 1 << (6-1) /* enable context 6 */ )
                               != CY_PROT_SUCCESS)
    {
        while(1); // Setting error
    }

    /* 2. Setting for MPU PC = "6" */
    /* This time MPU_PC can be set to only "6" because only SMPU_MS_CTL_PC_MASK[6] = "1"*/
    if(Cy_Prot_SetActivePC(CPUSS_MS_ID_CM0, 6) != CY_PROT_SUCCESS)
    {
        while(1); // Setting error
    }

    /* 3. Setting for SMPU_STRUCT (using STRUCT 2 and 3 for this time )*/
    /* 0x28020000 ~ 0x28027FFF: Only masters which have "5" as PC value can access this area */
    /* 0x28028000 ~ 0x2802FFFF: Only masters which have "6" as PC value can access this area */
    smpuStructTemp.regionSize = CY_PROT_SIZE_32KB; // 32KB: 0x8000 Byte
    smpuStructTemp.subregions = 0x00;
    smpuStructTemp.userPermission = CY_PROT_PERM_RWX;
    smpuStructTemp.privPermission = CY_PROT_PERM_RWX;
    smpuStructTemp.secure = 0; // Non secure
    smpuStructTemp.pcMatch = 0;

    smpuStructTemp.address = (uint32_t*)(0x28020000);
    smpuStructTemp.pcMask = 1 << (5 - 1); // enable context 5
    if(Cy_Prot_ConfigSmpuSlaveStruct(PROT_SMPU_SMPU_STRUCT2, &smpuStructTemp) != CY_PROT_SUCCESS)
    {
        while(1); // Setting error
    }

    smpuStructTemp.address = (uint32_t*)(0x28028000);
    smpuStructTemp.pcMask = 1 << (6 - 1); // enable context 6
    if(Cy_Prot_ConfigSmpuSlaveStruct(PROT_SMPU_SMPU_STRUCT3, &smpuStructTemp) != CY_PROT_SUCCESS)
    {
        while(1); // Setting error
    }

    /* 4. Enable SMPU_STRUCT */
    if(Cy_Prot_EnableSmpuSlaveStruct(PROT_SMPU_SMPU_STRUCT2) != CY_PROT_SUCCESS)
    {
        while(1); // Setting error
    }
    if(Cy_Prot_EnableSmpuSlaveStruct(PROT_SMPU_SMPU_STRUCT3) != CY_PROT_SUCCESS)
    {
        while(1); // Setting error
    }

    /* 5. Memory access test1. This area can be accessed by masters which have "6" as PC value */
    /* CM0+ has "6" as PC value, then these accesses are expected to be allowed */
    testRead0 = 0;
    *(uint32_t*)(0x28028000) = 0xA5A5A5A5;
    testRead0 = *(uint32_t*)(0x28028000);

    /* 6. Memory access test2. This area can be accessed by masters which have "5" as PC value */
    /* CM0+ has "6" as PC value, then these accesses are expected to be restricted and cause HardFault */
    testRead1 = 0;
    *(uint32_t*)(0x28020000) = 0xA5A5A5A5;
    testRead1 = *(uint32_t*)(0x28020000);
}


/* [] END OF FILE */
