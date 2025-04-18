/***************************************************************************//**
* \file system_cytviic2d6mddr.h
* \version 1.0
*
* \brief Device system header file.
*
********************************************************************************
* \copyright
* Copyright 2018-2021, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

 /**
* \defgroup group_system_cfg System Configuration (System)
* \{
* This driver provides global system_cfg defines and API function definitions.
*
* \defgroup group_system_cfg_macros Macro
* \defgroup group_system_cfg_functions Functions
* \defgroup group_system_cfg_data_structures Data structures
* \defgroup group_system_cfg_enums Enumerated types
*/

#ifndef SYSTEM_TVIIC2D6MDDR_H
#define SYSTEM_TVIIC2D6MDDR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**  Start address of the Cortex-M7_0 application */
#ifndef CY_CORTEX_M7_0_APPL_ADDR
    // This symbol is defined in the linker script
    extern char *  __cm7_0_vector_base_linker_symbol;
    #define CY_CORTEX_M7_0_APPL_ADDR      ((uint32_t) &__cm7_0_vector_base_linker_symbol)
#endif

/**  Start address of the Cortex-M7_1 application */
#ifndef CY_CORTEX_M7_1_APPL_ADDR
    // This symbol is defined in the linker script
    extern char *  __cm7_1_vector_base_linker_symbol;
    #define CY_CORTEX_M7_1_APPL_ADDR      ((uint32_t) &__cm7_1_vector_base_linker_symbol)
#endif
    

// Macro to provide a delay option
#define DELAY(loop_till)     do { for( volatile uint32_t loop_cnt = 0; loop_cnt < loop_till; loop_cnt++); } while(0)

// This cycle value should be less than 16777216 (0x01000000, 24bits)
#define DELAY_CORE_CYCLE(cycle) {\
    SysTick->CTRL = 0;\
    SysTick->LOAD = cycle;\
    SysTick->VAL  = 0;\
    SysTick->CTRL = 5;\
    while((SysTick->CTRL & 0x00010000) == 0);\
    SysTick->CTRL = 0;\
}\


/**
* \addtogroup group_system_cfg_macros
* \{
*/
#define CY_SYS_CM_STATUS_OFF                    (0u)    /**< The Cortex-M core is off. */
#define CY_SYS_CM_STATUS_RESET                  (1u)    /**< The Cortex-M core is in reset. */
#define CY_SYS_CM_STATUS_RETAINED               (2u)    /**< The Cortex-M core is retained. */
#define CY_SYS_CM_STATUS_ENABLED                (3u)    /**< The Cortex-M core is enabled */


/*******************************************************************************
* VCCD selection
*******************************************************************************/
#define CY_SYS_VCCD_INTERNAL                    (0u)
#define CY_SYS_VCCD_PMIC                        (1u)

#define CY_SYS_VCCD_SOURCE                      CY_SYS_VCCD_PMIC


/*******************************************************************************
* LPDDR4 power sequence selection
*******************************************************************************/
#define CY_SYS_LPDDR4_SEQ_HW                    0u      /* LPDDR4 memory power sequencing based on hardware */
#define CY_SYS_LPDDR4_SEQ_SW                    1u      /* LPDDR4 memory power sequencing based on software */

/* Defines the power sequence mechanism for LPDDR4 interface */
#define CY_SYS_USE_LPDDR4_SEQ                   CY_SYS_LPDDR4_SEQ_HW

/* Defines the millisecond delay between the power rails */
#define CY_SYS_LPDDR4_POWER_DELAY_US            5000u

/* Defines power goods monitor power rails */
#define CY_SYS_LPDDR4_PG_BYPASS                 0u
#define CY_SYS_LPDDR4_PG_POLL                   1u
    
/*******************************************************************************
* System power selection
*******************************************************************************/

/* Enable the LPDDR4 power */
#define CY_SYS_LPDDR_POWER_EN

/* Enable the SMIF IO power */
#define CY_SYS_SMIF_POWER_EN


/*******************************************************************************
* PLL400#0 leading to CM7 cores output frequency selection
*******************************************************************************/
#define CY_SYS_PLL400M_0_160MHz                 (160000000UL)
#define CY_SYS_PLL400M_0_320MHz                 (320000000UL)

#define CY_SYS_PLL400M_0_FREQ                   CY_SYS_PLL400M_0_320MHz

#if (CY_SYS_VCCD_SOURCE == CY_SYS_VCCD_INTERNAL) && (CY_SYS_PLL400M_0_FREQ != CY_SYS_PLL400M_0_160MHz)
  #error "The Core 7 frequency is too high to fulfill spec for internal VCCD."
#endif

/*******************************************************************************
* System Clock Initialization
*******************************************************************************/
#if (CY_USE_PSVP == 1)
    #define CY_INITIAL_TARGET_FAST0_FREQ        ( 26600000UL)

    #define CY_INITIAL_TARGET_FAST1_FREQ        ( 26600000UL)

    #define CY_INITIAL_TARGET_PERI_FREQ         ( 26600000UL)

    #define CY_INITIAL_TARGET_SLOW_FREQ         ( 26600000UL)

    #define CY_CLK_ECO_FREQ_HZ                  ( 10000000UL)

    #define CY_CLK_IMO_FREQ_HZ                  (  8000000UL)

    /** WCO frequency in Hz */
    #define CY_CLK_WCO_FREQ_HZ                  ( 26600000UL)
    
    /** HVILO0 frequency in Hz */
    #define CY_CLK_HVILO0_FREQ_HZ               (    12800UL)

    /** HVILO1 frequency in Hz */
    #define CY_CLK_HVILO1_FREQ_HZ               (    12800UL)

#else
    #define CY_INITIAL_TARGET_FAST0_FREQ        (CY_SYS_PLL400M_0_FREQ)

    #define CY_INITIAL_TARGET_FAST1_FREQ        (CY_SYS_PLL400M_0_FREQ)

    #define CY_INITIAL_TARGET_PERI_FREQ         ( 80000000UL)

    #define CY_INITIAL_TARGET_SLOW_FREQ         ( 80000000UL)

    #define CY_CLK_ECO_FREQ_HZ                  ( 16000000UL)

    #define CY_CLK_IMO_FREQ_HZ                  (  8000000UL)

    /** WCO frequency in Hz */
    #define CY_CLK_WCO_FREQ_HZ                  (    32768UL)
    
    /** HVILO0 frequency in Hz */
    #define CY_CLK_HVILO0_FREQ_HZ               (    32768UL)

    /** HVILO1 frequency in Hz */
    #define CY_CLK_HVILO1_FREQ_HZ               (    32768UL)
#endif

#define CY_CLK_EXT_FREQ_HZ                      (  8000000UL)

/** ECO parameters, pick from the ECO datasheet, tuned for ABM10-16MHz **/
#define SUM_LOAD_SHUNT_CAP_IN_PF                (10ul)

#if CY_CLK_ECO_FREQ_HZ < 16000000ul
#define ESR_IN_OHM                              (250ul)
#elif CY_CLK_ECO_FREQ_HZ < 20000000ul
#define ESR_IN_OHM                              (150ul)
#elif CY_CLK_ECO_FREQ_HZ < 30000000ul
#define ESR_IN_OHM                              (100ul)
#else
#define ESR_IN_OHM                              (70ul)
#endif

#define MIN_NEG_RESISTANCE                      (5 * ESR_IN_OHM)
#define MAX_DRIVE_LEVEL_IN_UW                   (100ul)


/* Do not use these definitions directly in your application */
#define CY_DELAY_MS_OVERFLOW_THRESHOLD          (0x8000u)
#define CY_DELAY_1K_THRESHOLD                   (1000u)
#define CY_DELAY_1K_MINUS_1_THRESHOLD           (CY_DELAY_1K_THRESHOLD - 1u)
#define CY_DELAY_1M_THRESHOLD                   (1000000u)
#define CY_DELAY_1M_MINUS_1_THRESHOLD           (CY_DELAY_1M_THRESHOLD - 1u)

#define CORE_CM7_0			        0 // Cortex-M7 core 0
#define CORE_CM7_1			        1 // Cortex-M7 core 1 
#define CORE_MAX			        2 // Error Selection
    
/************************User Configurable Macro Definitions*******************/

// Macro to select the Clock Source
#define CY_SYSTEM_USE_IMO                       0
#define CY_SYSTEM_USE_EXT                       1
#define CY_SYSTEM_USE_ECO                       2

#define CY_SYSTEM_USE_CLOCK                     CY_SYSTEM_USE_ECO

// Definition whether WCO enabled or not
#define CY_SYSTEM_WCO_ENABLE

// Definition of this macro disables the WDT (Enabled in SROM)
#define CY_SYSTEM_WDT_DISABLE


/** \} group_system_cfg_macros */

/**************************Definition of global variables**********************/

/** \addtogroup group_system_cfg_data_structures
* \{
*/
  
extern uint32_t SystemCoreClock;
extern uint32_t cy_delayFreqHz;
extern uint32_t cy_delayFreqKhz;
extern uint32_t cy_delayFreqMhz;
extern uint32_t cy_delay32kMs;

/** \} group_system_cfg_data_structures */

/******************************Definition of global functions******************/

/**
* \addtogroup group_system_cfg_functions
* \{
*/

/**
  \brief Setup the microcontroller system.

   Initialize the System and update the SystemCoreClock variable.
 */
extern void     SystemInit (void);

extern uint32_t Cy_SaveIRQ(void);
extern void     Cy_RestoreIRQ(uint32_t saved);

/**
  \brief  Update SystemCoreClock variable.

   Updates the SystemCoreClock with current core Clock retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate (void);

extern void Cy_SystemInitFpuEnable(void);

extern uint32_t Cy_SysGetApplCoreStatus(uint8_t core);
extern void     Cy_SysEnableApplCore(uint8_t core, uint32_t vectorTableOffset);
extern void     Cy_SysDisableApplCore(uint8_t core);
extern void     Cy_SysRetainApplCore(uint8_t core);
extern void     Cy_SysResetApplCore(uint8_t core);

/**
 * \brief  Sytem power sequencing functions.
 * Enable or disable the LPDDR and SMIF power sequence.
 */

#ifdef CY_SYS_SMIF_POWER_EN
/* Configure pins for hsio power enable */
extern void Cy_SystemHsioPowerConfig(void);

/* SMIF (HSIO_ENH) sequencing function declaration */
extern void Cy_SystemHsioEnhPowerUp(void);
extern void Cy_SystemHsioEnhPowerDown(void);
#endif  /* CY_SYS_SMIF_POWER_EN */

#ifdef CY_SYS_LPDDR_POWER_EN
/* Configure pins for lpddr power enable */
extern void Cy_SystemLpddrPowerConfig(void);

/* LPDDR4 sequencing function declaration */
extern void Cy_SystemLpdd4PowerUp(uint8_t isPollPg, uint32_t delayMs);
extern void Cy_SystemLpdd4PowerDown(uint8_t isPollPg, uint32_t delayMs);
#endif  /* CY_SYS_LPDDR_POWER_EN */


/** \} group_system_cfg_functions */

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_TVIIC2D6MDDR_H */

