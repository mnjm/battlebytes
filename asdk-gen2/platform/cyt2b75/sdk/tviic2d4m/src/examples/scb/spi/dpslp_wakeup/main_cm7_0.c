/***************************************************************************//**
* \file main_cm7_0.c
*
* \version 1.0
*
* \brief Main example file for CM7
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

/*
    This example let TVII go to DeepSleep after start.
    If TVII received SPI message, TVII wakes up and turn on a LED.
    After few seconds from waking up, TVII will turn the LED off
    and will go to DeepSleep again.
*/

/* Device specific port settings */
#if (CY_USE_PSVP == 1)
 #error "This example is not supposed to work with PSVP environment."
#endif

#define CY_SPI_SCB_TYPE      SCB0
#define CY_SPI_SCB_MISO_PORT GPIO_PRT5
#define CY_SPI_SCB_MISO_PIN  2
#define CY_SPI_SCB_MISO_MUX  P5_2_SCB0_SPI_MISO
#define CY_SPI_SCB_MOSI_PORT GPIO_PRT5
#define CY_SPI_SCB_MOSI_PIN  1
#define CY_SPI_SCB_MOSI_MUX  P5_1_SCB0_SPI_MOSI
#define CY_SPI_SCB_CLK_PORT  GPIO_PRT5
#define CY_SPI_SCB_CLK_PIN   0
#define CY_SPI_SCB_CLK_MUX   P5_0_SCB0_SPI_CLK
#define CY_SPI_SCB_SEL_PORT  GPIO_PRT4
#define CY_SPI_SCB_SEL_PIN   4
#define CY_SPI_SCB_SEL_MUX   P4_4_SCB0_SPI_SELECT3
#define CY_SPI_SCB_SS_IDX    3
#define CY_SPI_SCB_PCLK      PCLK_SCB0_CLOCK
#define CY_SPI_SCB_IRQN      scb_0_interrupt_IRQn
#define SOURCE_CLOCK_FRQ     80000000ul

/* User setting value */
#define SCB_SPI_BAUDRATE     125000ul /* Please set baudrate value of SPI you want */
#define SCB_SPI_OVERSAMPLING 16ul     /* Please set oversampling of SPI you want */
#define SCB_SPI_CLOCK        (SCB_SPI_BAUDRATE * SCB_SPI_OVERSAMPLING)

#define DIVIDER_NO_1 (1u)

#define USER_LED_PORT           CY_USER_LED2_PORT
#define USER_LED_PIN            CY_USER_LED2_PIN
#define USER_LED_PIN_MUX        CY_USER_LED2_PIN_MUX

cy_stc_scb_spi_context_t Context_SCB0_SPI;
bool SCB0CompletedFlag = false;

cy_stc_gpio_pin_config_t user_led_port_pin_cfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = USER_LED_PIN_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

static cy_stc_gpio_pin_config_t SPI_port_pin_cfg =
{
    .outVal    = 0ul,
    .driveMode = 0ul,            /* Will be updated in runtime */
    .hsiom     = HSIOM_SEL_GPIO, /* Will be updated in runtime */
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

static cy_stc_sysint_irq_t irq_cfg =
{
    .sysIntSrc  = CY_SPI_SCB_IRQN,
    .intIdx     = CPUIntIdx3_IRQn,
    .isEnabled  = true,
};

static const cy_stc_scb_spi_config_t SCB_SPI_cfg =
{
    .spiMode                    = CY_SCB_SPI_SLAVE,       /*** Specifies the mode of operation    ***/
    .subMode                    = CY_SCB_SPI_MOTOROLA,    /*** Specifies the submode of SPI operation    ***/
    .sclkMode                   = CY_SCB_SPI_CPHA0_CPOL0, /*** Clock is active low, data is changed on first edge ***/
    .oversample                 = SCB_SPI_OVERSAMPLING,   /*** SPI_CLOCK divided by SCB_SPI_OVERSAMPLING should be baudrate  ***/
    .rxDataWidth                = 16ul,                   /*** The width of RX data (valid range 4-16). It must be the same as \ref txDataWidth except in National sub-mode. ***/
    .txDataWidth                = 16ul,                   /*** The width of TX data (valid range 4-16). It must be the same as \ref rxDataWidth except in National sub-mode. ***/
    .enableMsbFirst             = true,                   /*** Enables the hardware to shift out the data element MSB first, otherwise, LSB first ***/
    .enableFreeRunSclk          = false,                  /*** Enables the master to generate a continuous SCLK regardless of whether there is data to send  ***/
    .enableInputFilter          = false,                  /*** Enables a digital 3-tap median filter to be applied to the input of the RX FIFO to filter glitches on the line. ***/
    .enableMisoLateSample       = true,                   /*** Enables the master to sample MISO line one half clock later to allow better timings. ***/
    .enableTransferSeperation   = true,                   /*** Enables the master to transmit each data element separated by a de-assertion of the slave select line (only applicable for the master mode) ***/
    .ssPolarity0                = false,                  /*** SS0: active low ***/
    .ssPolarity1                = false,                  /*** SS1: active low ***/
    .ssPolarity2                = false,                  /*** SS2: active low ***/
    .ssPolarity3                = false,                  /*** SS3: active low ***/
    .enableWakeFromSleep        = true,                   /*** When set, the slave will wake the device when the slave select line becomes active. Note that not all SCBs support this mode. Consult the device datasheet to determine which SCBs support wake from deep sleep. ***/
    // In case of using high level APIs, an user does not need to set these parameter. These parameter will be set in high level APIs such as "Cy_SCB_SPI_Transfer".
    .txFifoTriggerLevel         = 0ul,                    /*** Do not care ***/
    .rxFifoTriggerLevel         = 0ul,                    /*** Do not care ***/
    .rxFifoIntEnableMask        = 0ul,                    /*** Do not care  ***/
    .txFifoIntEnableMask        = 0ul,                    /*** Do not care  ***/
    .masterSlaveIntEnableMask   = 0ul,                    /*** Do not care  ***/
    .enableSpiDoneInterrupt     = false,
    .enableSpiBusErrorInterrupt = false,
};

/******   Clock Output Utilities   ******/
cy_stc_gpio_pin_config_t clkOutPortConfig =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = CY_HF3_CLK_OUT_PIN_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

void SetPeripheFracDiv24_5(uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum)
{
    uint64_t temp = ((uint64_t)sourceFreq << 5ull);
    uint32_t divSetting;

    divSetting = (uint32_t)(temp / targetFreq);
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(CY_SPI_SCB_PCLK), 
                                   CY_SYSCLK_DIV_24_5_BIT, divNum, 
                                   (((divSetting >> 5ul) & 0x00000FFFul) - 1ul), 
                                   (divSetting & 0x0000001Ful));
}

void irqSCB0(void)
{
    /***Interrupt flag clear***/
    Cy_SCB_ClearSpiInterrupt(CY_SPI_SCB_TYPE, CY_SCB_SPI_INTR_WAKEUP);

    SCB0CompletedFlag = true;
}

int main(void)
{
    SystemInit();

    __enable_irq(); /* Enable global interrupts. */

    /*************************************/
    /*      LED Port Initializing        */
    /*************************************/
    Cy_GPIO_Pin_Init(USER_LED_PORT, USER_LED_PIN, &user_led_port_pin_cfg);

    /******************************************************/
    /******* Calculate divider setting for SCB0 ***********/
    /******************************************************/
    Cy_SysClk_PeriphAssignDivider(CY_SPI_SCB_PCLK, CY_SYSCLK_DIV_24_5_BIT, DIVIDER_NO_1);
    SetPeripheFracDiv24_5(CY_SPI_SCB_PCLK, SOURCE_CLOCK_FRQ, DIVIDER_NO_1);
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(CY_SPI_SCB_PCLK), CY_SYSCLK_DIV_24_5_BIT, 1ul);

    /*******************************************/
    /*   De-initialization for peripherals     */
    /*******************************************/
    Cy_SCB_SPI_DeInit(CY_SPI_SCB_TYPE);

    /*******************************************/
    /* Interrupt setting for SPI communication */
    /*******************************************/
    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, irqSCB0);
    NVIC_EnableIRQ(irq_cfg.intIdx);

    /**************************************/
    /* Port Setting for SPI communication */
    /**************************************/
    SPI_port_pin_cfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    SPI_port_pin_cfg.hsiom     = CY_SPI_SCB_MISO_MUX;
    Cy_GPIO_Pin_Init(CY_SPI_SCB_MISO_PORT, CY_SPI_SCB_MISO_PIN, &SPI_port_pin_cfg);

    SPI_port_pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;
    SPI_port_pin_cfg.hsiom     = CY_SPI_SCB_MOSI_MUX;
    Cy_GPIO_Pin_Init(CY_SPI_SCB_MOSI_PORT, CY_SPI_SCB_MOSI_PIN, &SPI_port_pin_cfg);

    SPI_port_pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;
    SPI_port_pin_cfg.hsiom     = CY_SPI_SCB_CLK_MUX;
    Cy_GPIO_Pin_Init(CY_SPI_SCB_CLK_PORT, CY_SPI_SCB_CLK_PIN, &SPI_port_pin_cfg);
    
    SPI_port_pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;
    SPI_port_pin_cfg.hsiom     = CY_SPI_SCB_SEL_MUX;
    Cy_GPIO_Pin_Init(CY_SPI_SCB_SEL_PORT, CY_SPI_SCB_SEL_PIN, &SPI_port_pin_cfg);

    /********************************************/
    /* SCB initialization for SPI communication */
    /********************************************/
    Cy_SCB_SPI_Init(CY_SPI_SCB_TYPE, &SCB_SPI_cfg, &Context_SCB0_SPI);

    /*** Configuring SCB for EZSPI Mode ***/
    Cy_SCB_SetEzMode(CY_SPI_SCB_TYPE, true);

    /*** Externally clocked SPI interrupt mask ***/
    Cy_SCB_SetSpiInterruptMask(CY_SPI_SCB_TYPE, CY_SCB_SPI_INTR_WAKEUP);

    Cy_SCB_SPI_SetActiveSlaveSelect(CY_SPI_SCB_TYPE, CY_SPI_SCB_SS_IDX);
    Cy_SCB_SPI_Enable(CY_SPI_SCB_TYPE);

    /* Start output internal clock */
    Cy_GPIO_Pin_Init(CY_HF3_CLK_OUT_PORT, CY_HF3_CLK_OUT_PIN, &clkOutPortConfig);

    for(;;)
    {
        /* turn off the LED */
        Cy_GPIO_Write(USER_LED_PORT, USER_LED_PIN, 0ul);

        /* Put the system to DeeSleep */
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);

        /* turn on the LED */
        Cy_GPIO_Write(USER_LED_PORT, USER_LED_PIN, 1ul);

        while(SCB0CompletedFlag == false);
        SCB0CompletedFlag = false;

        /* wait 1 [s] */
        Cy_SysTick_DelayInUs(1000000ul);
    }
}



/* [] END OF FILE */
