/***************************************************************************//**
* \file main_cm7_0.c
*
* \version 1.0
*
* \brief Main example file for CM7_0
*
********************************************************************************
* \copyright
* Copyright 2016-2020, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/*****************************************************************************
* Include files
*****************************************************************************/
#include "cy_project.h"
#include "cy_device_headers.h"
#include <stdio.h>

/*****************************************************************************
* Local pre-processor symbols/macros ('define')
*****************************************************************************/

//=============================================================================================
// The SMIF init step may cause issues and lock up the system if there are issues with the
// SMIF devices (e.g. not present on the connected board, or disabled by jumpers, etc.)
// With following defines you can quickly include/exclude SMIF init:
// Just set SMIF0_EXT_FREQ_MHZ/SMIF1_EXT_FREQ_MHZ to 0 or non-0
//=============================================================================================

#if (CY_USE_PSVP == 1)
    #define DISP0_TYPE              CY_GFXENV_DISP_TYPE_800_480_60_TTL
    #define DISP1_TYPE              CY_GFXENV_DISP_TYPE_800_480_60_TTL

    #define SMIF0_EXT_FREQ_MHZ      0
    #define SMIF1_EXT_FREQ_MHZ      0
#else
    #define DISP0_TYPE              CY_GFXENV_DISP_TYPE_800_480_60_FPD_VESA  // or CY_GFXENV_DISP_TYPE_800_480_60_FPD_JEIDA
  #ifdef VIDEOSS0_FPDLINK1
    #define DISP1_TYPE              CY_GFXENV_DISP_TYPE_800_480_60_FPD_VESA  // or CY_GFXENV_DISP_TYPE_800_480_60_FPD_JEIDA
  #else
    #define DISP1_TYPE              CY_GFXENV_DISP_TYPE_800_480_60_TTL
  #endif

  // Notes:
  // - Frequency ratio of SMIF0_EXT_FREQ_MHZ and SMIF1_EXT_FREQ_MHZ needs to be 1, 2, 4 or 8 (or 1, 0.5, 0.25, 0.125 respectively)
  // - Try reducing frequency in case of issues, maximum frequencies may only be applicable for boards with soldered MCUs
  //   or depending on the board/revision some modifications may be necessary (e.g. resistor removal to reduce stubs)    

  #if defined(tviic2d6m)
  
    // Only revision C of tviic2d6m and the corresponding 327-BGA board is supported!
    #define SMIF0_EXT_FREQ_MHZ      166
    #define SMIF1_EXT_FREQ_MHZ      166
    
  #elif defined(tviic2d6mddr)
    
    /* SMIF0_CS0 - OctalFlash support at 100MHz maz and SMIF1_CS1 - HyperRAM is not tested  */ 
    #define SMIF0_EXT_FREQ_MHZ      100
    #define SMIF1_EXT_FREQ_MHZ      0
    
  #elif defined(tviic2d4m)

    #define SMIF0_EXT_FREQ_MHZ      133
    #define SMIF1_EXT_FREQ_MHZ      133
    
  #endif
#endif

#if (CY_USE_PSVP == 1)
    #define USER_LED_PORT           CY_BSP_BB_USER_LED1_PORT
    #define USER_LED_PIN            CY_BSP_BB_USER_LED1_PIN
    #define USER_LED_PIN_MUX        CY_BSP_BB_USER_LED1_PIN_MUX
#else
    #define USER_LED_PORT               CY_USER_LED2_PORT
    #define USER_LED_PIN                CY_USER_LED2_PIN
    #define USER_LED_PIN_MUX            CY_USER_LED2_PIN_MUX

    /* Reset pin used by CPU to reset SMIF0_CS0 device */
    #define CY_USE_SMIF0_RESET_PORT     GPIO_PRT4
    #define CY_USE_SMIF0_RESET_PIN      1
    #define CY_USE_SMIF0_RESET_PIN_MUX  P4_1_GPIO
#endif

#define LED_TOGGLE_TIMER_ID         0
#define EXTMEM_TEST_TIMER_ID        1

#define EXTMEM_TEST_SIZE_KB         512
#define EXTMEM_FLASH_BASE           CY_GFXENV_FASTINIT_EXTMEM_FLASH_BASE
#define EXTMEM_RAM_BASE             CY_GFXENV_FASTINIT_EXTMEM_RAM_BASE

#define EXTMEM_FLASH_ENABLED        (u8Smif0ExtClockMhz != 0)
#define EXTMEM_RAM_ENABLED          (u8Smif1ExtClockMhz != 0)

/*****************************************************************************
* Global variable definitions (declared in header file with 'extern')
*****************************************************************************/
cy_stc_gpio_pin_config_t user_led_port_pin_cfg =             
{                                                  
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


cy_stc_mpu_region_cfg_t astcMpuCfg[] =
{
    {.addr = EXTMEM_FLASH_BASE, .size = CY_MPU_SIZE_256MB, .permission = CY_MPU_ACCESS_P_FULL_ACCESS, .attribute = CY_MPU_ATTR_NORM_MEM_WB, .execute = CY_MPU_INST_ACCESS_EN, .srd = 0x00u, .enable = CY_MPU_ENABLE},
    {.addr = EXTMEM_RAM_BASE,   .size = CY_MPU_SIZE_256MB, .permission = CY_MPU_ACCESS_P_FULL_ACCESS, .attribute = CY_MPU_ATTR_NORM_MEM_WB, .execute = CY_MPU_INST_ACCESS_EN, .srd = 0x00u, .enable = CY_MPU_ENABLE},
};


// Create vars, so that frequencies can be easily changed by debugger without reflashing
uint8_t u8Smif0ExtClockMhz = SMIF0_EXT_FREQ_MHZ;
uint8_t u8Smif1ExtClockMhz = SMIF1_EXT_FREQ_MHZ;

/*****************************************************************************
* Local type definitions ('typedef')
*****************************************************************************/

/*****************************************************************************
* Local function prototypes ('static')
*****************************************************************************/
static void TestExtMemories(void);
static void LedToggleCallback(void);
static void ButtonCallback(uint8_t u8ButtonId, cy_button_en_state_t enState);

/*****************************************************************************
* Local variable definitions ('static')
*****************************************************************************/


/*****************************************************************************
* Function implementation - global ('extern') and local ('static')
*****************************************************************************/


/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
* \brief  Main function
*
* \return does not return
*
*******************************************************************************/
int main(void)
{
    __enable_irq();

    SystemInit();    

    SCB_DisableICache();
    SCB_DisableDCache();

    // Ensure that the address space for the external memories is assigned the desired memory attributes
    //Cy_MPU_Setup(astcMpuCfg, CY_GET_ARRAY_LEN(astcMpuCfg), CY_MPU_USE_DEFAULT_MAP_AS_BG, CY_MPU_DISABLED_DURING_FAULT_NMI);

    //Cy_SMIF_SwitchBridge(SMIF0, false); // Disable SMIF bridge to enable cache support for SMIF XIP address space

    Cy_GPIO_Pin_Init(USER_LED_PORT, USER_LED_PIN, &user_led_port_pin_cfg);
    
    if(EXTMEM_FLASH_ENABLED)
    {
        user_led_port_pin_cfg.hsiom = CY_USE_SMIF0_RESET_PIN_MUX;
        Cy_GPIO_Pin_Init(CY_USE_SMIF0_RESET_PORT, CY_USE_SMIF0_RESET_PIN, &user_led_port_pin_cfg);
        
        /* Reset the Flash memory */
        Cy_GPIO_Write(CY_USE_SMIF0_RESET_PORT, CY_USE_SMIF0_RESET_PIN, 0);
        Cy_SysTick_DelayInUs(50000);
        Cy_GPIO_Write(CY_USE_SMIF0_RESET_PORT, CY_USE_SMIF0_RESET_PIN, 1);
    }

    // Enable clocks/pin/buttons/semihosting/extmem
    cy_gfxenv_en_result_t result;
    result = Cy_GfxEnv_FastInit(u8Smif0ExtClockMhz, u8Smif1ExtClockMhz, ButtonCallback);
    CY_ASSERT(result == CY_GFXENV_SUCCESS);
    
    // Output test picture on Display #0 and #1
    result = Cy_GfxEnv_EnableTestImage(DISP0_TYPE, DISP1_TYPE);
    CY_ASSERT(result == CY_GFXENV_SUCCESS);
    
    printf("Hello World via UART Semihosting!\n");
    
    Cy_SwTmr_Wait(0, 500, NULL); // Wait 500 ms
    
    // Start a periodic SW timer with 500ms interval
    Cy_SwTmr_Start(LED_TOGGLE_TIMER_ID, 500, true, LedToggleCallback);
    
    // Start a periodic SW timer with 100ms interval
    Cy_SwTmr_Start(EXTMEM_TEST_TIMER_ID, 100, true, TestExtMemories);

    for(;;)
    {
        // Poll and operate on expired (non high-prio) SW timers
        Cy_SwTmr_Main();
    }
}


/*******************************************************************************
* Function Name: TestExtMemories
****************************************************************************//**
*
* \brief  Calculates checksum of the content of the enabled memories and compares
*         with expected checksum. Updates test pattern in RAM connected to SMIF1
*
* \return none
*
*******************************************************************************/
static void TestExtMemories(void)
{
    static bool     bFirstRun = true;
    static bool     bFirstChecksumMismatchDetected = false;
    static uint32_t u32ExpectedChecksumExtFlash = 0;
    static uint32_t u32Counter = 0;
    
    uint32_t u32ExpectedChecksumExtRam = 0;    
    uint32_t u32ActualChecksumExtFlash = 0;
    uint32_t u32ActualChecksumExtRam   = 0;
    
    if(EXTMEM_FLASH_ENABLED)
    {
        for(uint32_t u32Word = 0; u32Word < (EXTMEM_TEST_SIZE_KB * 1024UL / 4); u32Word++)
        {
            u32ActualChecksumExtFlash += ((uint32_t *) EXTMEM_FLASH_BASE)[u32Word];
        }        
    }
    
    if(bFirstRun == true)
    {
        bFirstRun = false;
        u32ExpectedChecksumExtFlash = u32ActualChecksumExtFlash;
        return;
    }    
    
    if(EXTMEM_RAM_ENABLED)
    {
        for(uint32_t u32Word = 0; u32Word < (EXTMEM_TEST_SIZE_KB * 1024UL / 4); u32Word++)
        {
            uint32_t u32Pattern = ((uint16_t)(u32Word + u32Counter) << 16) + (uint16_t) (u32Word + u32Counter);
            ((uint32_t *) EXTMEM_RAM_BASE)[u32Word] = u32Pattern;
            u32ExpectedChecksumExtRam += u32Pattern;
        }
        
        for(uint32_t u32Word = 0; u32Word < (EXTMEM_TEST_SIZE_KB * 1024UL / 4); u32Word++)
        {
            u32ActualChecksumExtRam += ((uint32_t *) EXTMEM_RAM_BASE)[u32Word];
        }
    }
    
    if((u32ExpectedChecksumExtFlash != u32ActualChecksumExtFlash) ||
       (u32ExpectedChecksumExtRam   != u32ActualChecksumExtRam)     )
    {
        if(bFirstChecksumMismatchDetected == false)
        {
            bFirstChecksumMismatchDetected = true;
            Cy_SwTmr_Stop(LED_TOGGLE_TIMER_ID);
            // Restart timer with higher frequency to indicate the mismatch
            // Start a periodic SW timer with 100ms interval
            Cy_SwTmr_Start(LED_TOGGLE_TIMER_ID, 100, true, LedToggleCallback);
        }
    }
    
    u32Counter++;
}


/*******************************************************************************
* Function Name: LedToggleCallback
****************************************************************************//**
*
* \brief  Toggles an LED
*
* \return none
*
*******************************************************************************/
static void LedToggleCallback(void)
{
    Cy_GPIO_Inv(USER_LED_PORT, USER_LED_PIN);    
}



/*******************************************************************************
* Function Name: ButtonCallback
****************************************************************************//**
*
* \brief  Handles button events
*
* \param  u8ButtonId : ID of the button that caused the callback
* \param  enState    : New state of the button
*
* \return none
*
*******************************************************************************/
static void ButtonCallback(uint8_t u8ButtonId, cy_button_en_state_t enState)
{
    // Return if button was released only
    if (enState == StateHigh)
    {
        return;
    }

    // Handle the pressed button
    switch (u8ButtonId)
    {         
        case CY_GFXENV_BUTTON_ID_WAKE:
        case CY_GFXENV_BUTTON_ID_BB_1:
            break;
        case CY_GFXENV_BUTTON_ID_LEFT:
        case CY_GFXENV_BUTTON_ID_BB_2:
            break;
        case CY_GFXENV_BUTTON_ID_RIGHT:
        case CY_GFXENV_BUTTON_ID_BB_3:
            break;
        case CY_GFXENV_BUTTON_ID_UP:
        case CY_GFXENV_BUTTON_ID_BB_4:
            break;
        default:
            break;
    }
}

/* [] END OF FILE */
