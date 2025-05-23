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

#include <stdio.h>
#include <stdarg.h>

#include "cy_project.h"
#include "cy_device_headers.h"

/* Select UART Echo Type                                                   */
/*                                                                         */
/* Use Low-Level API                                                       */
/*  (1) Polling & Receive by 1 byte unit                                   */
/* Use High-Level API                                                      */
/*  (2) Interrupt & Receive by 1 byte unit                                 */
/*  (3) Interrupt & Receive by threshold byte unit (E_UART_RECV_THRESHOLD) */
/*  (4) Interrupt & Receive using Ring Buffer (Half of RX FIFO size)       */
//#define E_UART_ECHO_POLLING_1BYTE
#define E_UART_ECHO_INTR_1BYTE
//#define E_UART_ECHO_INTR_THRESHOLD
//#define E_UART_ECHO_INTR_RINGBUF

/* Select Baud Rate */
#define E_UART_BAUD_115200  115200
#define E_UART_BAUD_57600   57600
#define E_UART_BAUD_38400   38400
#define E_UART_BAUD_19200   19200
#define E_UART_BAUD_9600    9600
#define E_UART_BAUD_2400    2400
#define E_UART_BAUD_1200    1200
#define E_UART_BAUD         E_UART_BAUD_115200

#define SCB_PERI_CLOCK_24MHz    (24000000)
#define SCB_PERI_CLOCK_26MHz    (26600000)
#define SCB_PERI_CLOCK          SCB_PERI_CLOCK_26MHz

/* Local Definision */
#define E_UART_RECV_THRESHOLD    8
#define E_UART_RING_BUF_SIZE     512
#define E_UART_USER_BUF_SIZE     512
#define E_UART_RX_INTR_FACTER     (                              \
                                 CY_SCB_UART_RX_TRIGGER      |   \
                               /*CY_SCB_UART_RX_NOT_EMPTY    | */\
                               /*CY_SCB_UART_RX_FULL         | */\
                                 CY_SCB_UART_RX_OVERFLOW     |   \
                                 CY_SCB_UART_RX_UNDERFLOW    |   \
                                 CY_SCB_UART_RX_ERR_FRAME    |   \
                                 CY_SCB_UART_RX_ERR_PARITY   |   \
                                 CY_SCB_UART_RX_BREAK_DETECT |   \
                                 0                               \
                                )
#define E_UART_TX_INTR_FACTER     (                              \
                                 CY_SCB_UART_TX_TRIGGER      |   \
                               /*CY_SCB_UART_TX_NOT_FULL     | */\
                               /*CY_SCB_UART_TX_EMPTY        | */\
                                 CY_SCB_UART_TX_OVERFLOW     |   \
                               /*CY_SCB_UART_TX_UNDERFLOW    | */\
                                 CY_SCB_UART_TX_DONE         |   \
                               /*CY_SCB_UART_TX_NACK         | */\
                               /*CY_SCB_UART_TX_ARB_LOST     | */\
                                 0                               \
                                )

/* Local Functions Declaration */
void Peripheral_Initialization(void);
void Scb_UART_IntrISR(void);
void Scb_UART_Event(uint32_t locEvents);
void Term_Printf(void *fmt, ...);
void Term_Input(void);

/* Local Macro for Termnal Print */
#define TERM_PRINT_H(...) Term_Printf(__VA_ARGS__)
//#define TERM_PRINT_L(...) Term_Printf(__VA_ARGS__)
#define TERM_PRINT_L(...)

/* SCB - UART Configuration */
cy_stc_scb_uart_context_t   g_stc_uart_context;
cy_stc_scb_uart_config_t    g_stc_uart_config = {
                                                   .uartMode                   = CY_SCB_UART_STANDARD,
                                                   .oversample                 = 1,
                                                   .dataWidth                  = 8,
                                                   .enableMsbFirst             = false,
                                                   .stopBits                   = CY_SCB_UART_STOP_BITS_1,
                                                   .parity                     = CY_SCB_UART_PARITY_NONE,
                                                   .enableInputFilter          = false,
                                                   .dropOnParityError          = false,
                                                   .dropOnFrameError           = false,
                                                   .enableMutliProcessorMode   = false,
                                                   .receiverAddress            = 0,
                                                   .receiverAddressMask        = 0,
                                                   .acceptAddrInFifo           = false,
                                                   .irdaInvertRx               = false,
                                                   .irdaEnableLowPowerReceiver = false,
                                                   .smartCardRetryOnNack       = false,
                                                   .enableCts                  = false,
                                                   .ctsPolarity                = CY_SCB_UART_ACTIVE_LOW,
                                                   .rtsRxFifoLevel             = 0,
                                                   .rtsPolarity                = CY_SCB_UART_ACTIVE_LOW,
                                                   .breakWidth                 = 0,
                                                   .rxFifoTriggerLevel         = 0,
                                                   .rxFifoIntEnableMask        = E_UART_RX_INTR_FACTER,
                                                   .txFifoTriggerLevel         = 0,
                                                   .txFifoIntEnableMask        = E_UART_TX_INTR_FACTER
                                                };

/* Local Variables */
uint8_t                     g_uart_out_data[128];                       // TX Buffer for Terminal Print
uint8_t                     g_uart_in_data[128];                        // RX Buffer
uint8_t                     g_uart_rx_ring[E_UART_RING_BUF_SIZE] = {0}; // RX Ring Buffer
uint8_t                     g_uart_user_buf[E_UART_USER_BUF_SIZE];      // User Buffer for coping from Ring Buffer

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    SystemInit();

    uint8_t     comment[] = "UART Test\n\r";
    uint32_t    major = CY_SCB_DRV_VERSION_MAJOR;
    uint32_t    minor = CY_SCB_DRV_VERSION_MINOR;

    /* Enable CM7_0/1. CY_CORTEX_M7_APPL_ADDR is calculated in linker script, check it in case of problems. */
    Cy_SysEnableApplCore(CORE_CM7_0, CY_CORTEX_M7_0_APPL_ADDR);
    Cy_SysEnableApplCore(CORE_CM7_1, CY_CORTEX_M7_1_APPL_ADDR);

    /* Initialize Port and Clock */
    Peripheral_Initialization();

    /* Initilize & Enable UART */
    Cy_SCB_UART_DeInit(CY_USB_SCB_UART_TYPE);
    Cy_SCB_UART_Init(CY_USB_SCB_UART_TYPE, &g_stc_uart_config, &g_stc_uart_context);
    Cy_SCB_UART_RegisterCallback(CY_USB_SCB_UART_TYPE, (scb_uart_handle_events_t)Scb_UART_Event, &g_stc_uart_context);
#if defined(E_UART_ECHO_INTR_RINGBUF)
    Cy_SCB_UART_StartRingBuffer(CY_BSP_USB_UART_TYPE, (void *)g_uart_rx_ring, E_UART_RING_BUF_SIZE, &g_stc_uart_context);
#endif
    Cy_SCB_UART_Enable(CY_USB_SCB_UART_TYPE);

    /* Enable Interrupt */
#if defined (E_UART_ECHO_INTR_1BYTE) | defined(E_UART_ECHO_INTR_THRESHOLD) | defined(E_UART_ECHO_INTR_RINGBUF)
    NVIC_EnableIRQ(CPUIntIdx2_IRQn);
#endif

    /* Opening UART Comment */
    TERM_PRINT_H("\n%s (ver=%d.%d)\n\r", comment, major, minor);
#if   defined(E_UART_ECHO_POLLING_1BYTE)
    TERM_PRINT_H("POLLING 1BYTE ECHO\n\r");
#elif defined(E_UART_ECHO_INTR_1BYTE)
    TERM_PRINT_H("INTR 1BYTE ECHO\n\r");
#elif defined(E_UART_ECHO_INTR_THRESHOLD)
    TERM_PRINT_H("INTR THRESHOLD ECHO(%d)\n\r", E_UART_RECV_THRESHOLD);
#elif defined(E_UART_ECHO_INTR_RINGBUF)
    TERM_PRINT_H("INTR RINGBUFFER ECHO(64: half size of FIFO)\n\r");
#endif

#if defined(E_UART_ECHO_INTR_THRESHOLD)
    /* UART Echo Test (High-Level)                                            */
    /* (3) Interrupt & Receive by threshold byte unit (E_UART_RECV_THRESHOLD) */
    Cy_SCB_UART_Receive(CY_BSP_USB_UART_TYPE, g_uart_in_data, E_UART_RECV_THRESHOLD, &g_stc_uart_context);
#endif

    for(;;)
    {
#if defined(E_UART_ECHO_POLLING_1BYTE)
        /* UART Echo Test (Low-Level)           */
        /* (1) Polling & Receive by 1 byte unit */
        Term_Input();
#endif

#if defined(E_UART_ECHO_INTR_RINGBUF)
        /* UART Echo Test (High-Level)                                       */
        /* (4) Interrupt & Receive using Ring Buffer (Half of RX FIFO size)  */
        /*     -> Copy receive data to User Buffer from Ring Buffer and Echo */
        uint32_t num = Cy_SCB_UART_GetNumInRingBuffer(CY_BSP_USB_UART_TYPE, &g_stc_uart_context);
        if (num != 0) {
            Cy_SCB_UART_Receive(CY_BSP_USB_UART_TYPE, &g_uart_user_buf[0], num, &g_stc_uart_context);
            Cy_SCB_UART_Transmit(CY_BSP_USB_UART_TYPE, &g_uart_user_buf[0], num, &g_stc_uart_context);
        }
#endif
    }
}

// CLK_PERI/Oversample/baud
// Fout = Fin / (INT24_DIV + (FRAC5_DIV/32))
// PSVP Clock -> 26.6MHz
// 115200 = (26600000/8) / (INT24_DIV + (FRAC5_DIV/32))
// (INT24_DIV + (FRAC5_DIV/32)) = 28.86
// (28 + (2/32)) ~ 52.08 -> integer divide value of 28 is INT24_DIV=28+1=29

void SetPeripheFracDiv24_5(uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum)
{
    uint64_t temp = ((uint64_t)sourceFreq << 5ull);
    uint32_t divSetting;

    divSetting = (uint32_t)(temp / targetFreq);
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(CY_USB_SCB_UART_PCLK), 
                                   CY_SYSCLK_DIV_24_5_BIT, divNum, 
                                   (((divSetting >> 5u) & 0x00000FFF) - 1u), 
                                   (divSetting & 0x0000001F));
}

void Peripheral_Initialization(void)
{
    cy_stc_gpio_pin_config_t    stc_port_pin_cfg_uart = {0};
    cy_stc_sysint_irq_t         stc_sysint_irq_cfg_uart;

    /*-----------------------------*/
    /* Port Configuration for UART */
    /*-----------------------------*/

    stc_port_pin_cfg_uart.driveMode = CY_GPIO_DM_HIGHZ;
    stc_port_pin_cfg_uart.hsiom     = CY_USB_SCB_UART_RX_PIN_MUX;
    Cy_GPIO_Pin_Init(CY_USB_SCB_UART_RX_PORT, CY_USB_SCB_UART_RX_PIN, &stc_port_pin_cfg_uart);

    stc_port_pin_cfg_uart.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    stc_port_pin_cfg_uart.hsiom     = CY_USB_SCB_UART_TX_PIN_MUX;
    Cy_GPIO_Pin_Init(CY_USB_SCB_UART_TX_PORT, CY_USB_SCB_UART_TX_PIN, &stc_port_pin_cfg_uart);

    Cy_SysClk_HfClkEnable(CY_SYSCLK_HFCLK_2);

    /*-----------------------------*/
    /* Clock Configuration for UART */
    /*-----------------------------*/

    /* Assign a programmable divider */
    Cy_SysClk_PeriphAssignDivider(CY_USB_SCB_UART_PCLK, CY_SYSCLK_DIV_24_5_BIT, 0u);

#if (CY_USE_PSVP == 1)    
    /* Sets the 24.5 divider */
  #if   (E_UART_BAUD == E_UART_BAUD_115200)
    SetPeripheFracDiv24_5(E_UART_BAUD, SCB_PERI_CLOCK/8, 0u);
    g_stc_uart_config.oversample = 8;
  #elif (E_UART_BAUD == E_UART_BAUD_57600)
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(CY_BSP_USB_UART_PCLK), CY_SYSCLK_DIV_24_5_BIT, 0u, 51u,  3u);   // 57588
    g_stc_uart_config.oversample = 8;
  #elif (E_UART_BAUD == E_UART_BAUD_38400)
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(CY_BSP_USB_UART_PCLK), CY_SYSCLK_DIV_24_5_BIT, 0u, 77u,  4u);   // 38400
    g_stc_uart_config.oversample = 8;
  #elif (E_UART_BAUD == E_UART_BAUD_19200)
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(CY_BSP_USB_UART_PCLK), CY_SYSCLK_DIV_24_5_BIT, 0u, 155u, 8u);   // 19200
    g_stc_uart_config.oversample = 8;
  #elif (E_UART_BAUD == E_UART_BAUD_9600)
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(CY_BSP_USB_UART_PCLK), CY_SYSCLK_DIV_24_5_BIT, 0u, 311u, 16u);  // 9600
    g_stc_uart_config.oversample = 8;
  #elif (E_UART_BAUD == E_UART_BAUD_2400)
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(CY_BSP_USB_UART_PCLK), CY_SYSCLK_DIV_24_5_BIT, 0u, 624u, 0u);   // 2400
    g_stc_uart_config.oversample = 16;
  #elif (E_UART_BAUD == E_UART_BAUD_1200)
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(CY_BSP_USB_UART_PCLK), CY_SYSCLK_DIV_24_5_BIT, 0u, 1249, 0u);   // 1200
    g_stc_uart_config.oversample = 16;
  #else 
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(CY_BSP_USB_UART_PCLK), CY_SYSCLK_DIV_24_5_BIT, 0u, 25u,  3u);   // 115246
    g_stc_uart_config.oversample = 8;
  #endif
#else // CY_USE_PSVP
    /* Sets the 24.5 divider */
  #if   (E_UART_BAUD == E_UART_BAUD_115200)
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(CY_USB_SCB_UART_PCLK), CY_SYSCLK_DIV_24_5_BIT, 0u, 85u, 26u); // Divider 86.8125 --> 80MHz / 86.8125 / 8 (oversampling) = 115190 Hz
    g_stc_uart_config.oversample = 8;
  #else 
    #error "Baudrate dividers not yet calculated for 80MHz. Please add on your own!"
  #endif    
#endif

    /* Enable peripheral divider */
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(CY_USB_SCB_UART_PCLK), CY_SYSCLK_DIV_24_5_BIT, 0u);

    /*-----------------------------------*/
    /* Interrrupt Configuration for UART */
    /*-----------------------------------*/

    /* Int2 -> CY_BSP_USB_UART_TYPE */
    stc_sysint_irq_cfg_uart.sysIntSrc = CY_USB_SCB_UART_IRQN;
    stc_sysint_irq_cfg_uart.intIdx    = CPUIntIdx2_IRQn;
    stc_sysint_irq_cfg_uart.isEnabled = true;
    Cy_SysInt_InitIRQ(&stc_sysint_irq_cfg_uart);
    Cy_SysInt_SetSystemIrqVector(stc_sysint_irq_cfg_uart.sysIntSrc, Scb_UART_IntrISR);
}

void Scb_UART_IntrISR(void)
{
#if defined(E_UART_ECHO_INTR_1BYTE)
    /* UART Echo Test (High-Level)            */
    /* (2) Interrupt & Receive by 1 byte unit */
    uint32_t num = Cy_SCB_UART_GetNumInRxFifo(CY_USB_SCB_UART_TYPE);
    if (num != 0) {
        Cy_SCB_UART_Receive(CY_USB_SCB_UART_TYPE, &g_uart_in_data[0], num, &g_stc_uart_context);
        Cy_SCB_UART_Transmit(CY_USB_SCB_UART_TYPE, &g_uart_in_data[0], num, &g_stc_uart_context);
        Cy_SCB_SetRxFifoLevel(CY_USB_SCB_UART_TYPE, 0);
    }
#endif

    /* UART interrupt handler */
    Cy_SCB_UART_Interrupt(CY_USB_SCB_UART_TYPE, &g_stc_uart_context);
    NVIC_ClearPendingIRQ(CPUIntIdx2_IRQn);
}

void Scb_UART_Event(uint32_t locEvents)
{
    switch (locEvents) {

        case CY_SCB_UART_TRANSMIT_IN_FIFO_EVENT:
            break;

        case CY_SCB_UART_TRANSMIT_DONE_EVENT:
            break;

        case CY_SCB_UART_RECEIVE_DONE_EVENT:
#if defined(E_UART_ECHO_INTR_THRESHOLD)
            /* UART Test (High-Level)                                                 */
            /* (3) Interrupt & Receive by threshold byte unit (E_UART_RECV_THRESHOLD) */
            Cy_SCB_UART_Transmit(CY_BSP_USB_UART_TYPE, &g_uart_in_data[0], E_UART_RECV_THRESHOLD, &g_stc_uart_context);
            Cy_SCB_UART_Receive(CY_BSP_USB_UART_TYPE, &g_uart_in_data[0], E_UART_RECV_THRESHOLD, &g_stc_uart_context);
#endif
            /* Re-Enable Interrupt */
            Cy_SCB_SetRxInterruptMask(CY_USB_SCB_UART_TYPE, g_stc_uart_config.rxFifoIntEnableMask);
            break;

        case CY_SCB_UART_RB_FULL_EVENT:
            break;

        case CY_SCB_UART_RECEIVE_ERR_EVENT:
            break;

        case CY_SCB_UART_TRANSMIT_ERR_EVENT:
            break;

        default:
            break;
    }
}

void Term_Printf(void *fmt, ...)
{
    va_list arg;

    /* UART Print */
    va_start(arg, fmt);
    vsprintf((char*)&g_uart_out_data[0], (char*)fmt, arg);
    while (Cy_SCB_UART_IsTxComplete(CY_USB_SCB_UART_TYPE) != true) {};
    Cy_SCB_UART_PutArray(CY_USB_SCB_UART_TYPE, g_uart_out_data, strlen((char *)g_uart_out_data));
    va_end(arg);
}

void Term_Input(void)
{
    uint32_t    rx_num;

    /* Check Receive Data */
    rx_num = Cy_SCB_UART_GetNumInRxFifo(CY_USB_SCB_UART_TYPE);
    if (0u == rx_num) return;

    /* UART In/Out */
#if 0
    uint32_t    cmd;
    cmd = Cy_SCB_UART_Get(CY_BSP_USB_UART_TYPE);
    while (0UL == Cy_SCB_UART_Put(CY_BSP_USB_UART_TYPE, cmd)) {}
#endif
#if 1
    Cy_SCB_UART_GetArray(CY_USB_SCB_UART_TYPE, g_uart_in_data, rx_num);
    Cy_SCB_UART_PutArray(CY_USB_SCB_UART_TYPE, g_uart_in_data, rx_num);
#endif
#if 0
    Cy_SCB_UART_GetArrayBlocking(CY_BSP_USB_UART_TYPE, g_uart_in_data, rx_num);
    Cy_SCB_UART_PutArrayBlocking(CY_BSP_USB_UART_TYPE, g_uart_in_data, rx_num);
    Cy_SCB_UART_PutString(CY_BSP_USB_UART_TYPE, (char_t *)g_uart_in_data);
#endif
}

/* [] END OF FILE */
