/***************************************************************************//**
* \file main_cm7_1.c
*
* \brief
* Main file for CM7_0
*
********************************************************************************
* \copyright
* Copyright 2016-2019, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_project.h"
#include "cy_device_headers.h"
#include "example_config.h"

//==========================================================================================================
#if (EXAMPLE_CONFIG_SELECTED == EXAMPLE_CONFIG__CM0P_TO_CM7_1)

cy_stc_gpio_pin_config_t user_led0_port_pin_cfg =             
{                                                  
    .outVal = 0x00,                                
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,    
    .hsiom = CY_LED0_PIN_MUX,                           
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
    .hsiom = CY_LED1_PIN_MUX,                           
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
    .hsiom = CY_LED2_PIN_MUX,                           
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

cy_stc_gpio_pin_config_t user_led3_port_pin_cfg =             
{                                                  
    .outVal = 0x00,                                
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,    
    .hsiom = CY_LED3_PIN_MUX,                           
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

typedef struct
{
    uint32_t clientId;  /* PIPE I/F internal use area. Must be first element of the structure. */
    uint32_t LedData;
} custom_data_t;

custom_data_t pipeData;

/* Call when the other end point send the data to this end point */
void ReceivedCallback(uint32_t *msgPtr)
{
    /* New data comes */
    custom_data_t * data = (custom_data_t *)msgPtr;
    pipeData.LedData = data->LedData;
    switch(pipeData.LedData)
    {
        case 0u:    /* OFF OFF OFF OFF */
            Cy_GPIO_Write(CY_LED0_PORT, CY_LED0_PIN, 0u);
            Cy_GPIO_Write(CY_LED1_PORT, CY_LED1_PIN, 0u);
            Cy_GPIO_Write(CY_LED2_PORT, CY_LED2_PIN, 0u);
            Cy_GPIO_Write(CY_LED3_PORT, CY_LED3_PIN, 0u);
            break;
        case 1u:    /* ON OFF OFF OFF */
            Cy_GPIO_Write(CY_LED0_PORT, CY_LED0_PIN, 1u);
            Cy_GPIO_Write(CY_LED1_PORT, CY_LED1_PIN, 0u);
            Cy_GPIO_Write(CY_LED2_PORT, CY_LED2_PIN, 0u);
            Cy_GPIO_Write(CY_LED3_PORT, CY_LED3_PIN, 0u);
            break;
        case 2u:    /* OFF ON OFF OFF */
            Cy_GPIO_Write(CY_LED0_PORT, CY_LED0_PIN, 0u);
            Cy_GPIO_Write(CY_LED1_PORT, CY_LED1_PIN, 1u);
            Cy_GPIO_Write(CY_LED2_PORT, CY_LED2_PIN, 0u);
            Cy_GPIO_Write(CY_LED3_PORT, CY_LED3_PIN, 0u);
            break;
        case 3u:    /* OFF OFF ON OFF */
            Cy_GPIO_Write(CY_LED0_PORT, CY_LED0_PIN, 0u);
            Cy_GPIO_Write(CY_LED1_PORT, CY_LED1_PIN, 0u);
            Cy_GPIO_Write(CY_LED2_PORT, CY_LED2_PIN, 1u);
            Cy_GPIO_Write(CY_LED3_PORT, CY_LED3_PIN, 0u);
            break;
        case 4u:    /* OFF OFF OFF ON */
            Cy_GPIO_Write(CY_LED0_PORT, CY_LED0_PIN, 0u);
            Cy_GPIO_Write(CY_LED1_PORT, CY_LED1_PIN, 0u);
            Cy_GPIO_Write(CY_LED2_PORT, CY_LED2_PIN, 0u);
            Cy_GPIO_Write(CY_LED3_PORT, CY_LED3_PIN, 1u);
            break;
        default:
            break;
    }
}


cy_stc_ipc_pipe_config_t pipeConfig = 
{
    .epIndexForThisCpu = EP_INDEX_THIS_CPU,
    .epConfigData      = CY_IPC_PIPE_ENDPOINTS_DEFAULT_CONFIG,
};


int main(void)
{
    SystemInit();

    // Example had been originally tested with "cache off", so ensure that caches are turned off (may have been enabled by new startup.c module)
    SCB_DisableICache(); // Disables and invalidates instruction cache
    SCB_DisableDCache(); // Disables, cleans and invalidates data cache

    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
      
    Cy_GPIO_Pin_Init(CY_LED0_PORT, CY_LED0_PIN, &user_led0_port_pin_cfg);
    Cy_GPIO_Pin_Init(CY_LED1_PORT, CY_LED1_PIN, &user_led1_port_pin_cfg);
    Cy_GPIO_Pin_Init(CY_LED2_PORT, CY_LED2_PIN, &user_led2_port_pin_cfg);
    Cy_GPIO_Pin_Init(CY_LED3_PORT, CY_LED3_PIN, &user_led3_port_pin_cfg);

    /* Initialize PIPE */
    Cy_IPC_Pipe_Init(&pipeConfig);

    /* Register data received callback */
    Cy_IPC_Pipe_RegisterCallback
    (
        ReceivedCallback,
        0x01u  /* Accept Client ID = 0x01 */
    );

    /* Already registered the handler to the system interrup structure.
     * So just enable corresponding IRQ channel.
     */
    NVIC_ClearPendingIRQ(pipeConfig.epConfigData[EP_INDEX_THIS_CPU].ipcCpuIntIdx);
    NVIC_EnableIRQ(pipeConfig.epConfigData[EP_INDEX_THIS_CPU].ipcCpuIntIdx);


    for(;;)
    {
        /* Do nothing here */
        /* When data comes from the other core, ReceivedCallback is invoked. */
    }
}


//==========================================================================================================
#elif (EXAMPLE_CONFIG_SELECTED == EXAMPLE_CONFIG__CM7_1_TO_CM7_0)

typedef struct
{
    uint32_t clientId;  /* PIPE I/F internal use area. Must be first element of the structure. */
    uint32_t LedData;
} custom_data_t;

custom_data_t pipeData = 
{
    .clientId = 0x01u,
    .LedData  = 0x00u
};

bool released;

cy_en_ipc_pipe_status_t status;

cy_stc_ipc_pipe_config_t pipeConfig = 
{
    .epIndexForThisCpu = EP_INDEX_THIS_CPU,
    .epConfigData      = CY_IPC_PIPE_ENDPOINTS_DEFAULT_CONFIG,
};

/* Call when the other end point release the handle corresponding this end point */
void ReleaseCallback(void)
{
    /* Notified core already got the data. */
    /* Update send data for next transmission. */
    released = true;
}


int main(void)
{
    SystemInit();

    // Example had been originally tested with "cache off", so ensure that caches are turned off (may have been enabled by new startup.c module)
    SCB_DisableICache(); // Disables and invalidates instruction cache
    SCB_DisableDCache(); // Disables, cleans and invalidates data cache

    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
      
    /* Initialize PIPE */
    Cy_IPC_Pipe_Init(&pipeConfig);

    /* Already registered the handler to the system interrup structure.
     * So just enable corresponding IRQ channel.
     */
    NVIC_ClearPendingIRQ(pipeConfig.epConfigData[EP_INDEX_THIS_CPU].ipcCpuIntIdx);
    NVIC_EnableIRQ(pipeConfig.epConfigData[EP_INDEX_THIS_CPU].ipcCpuIntIdx);

    for(;;)
    {

        /* Send data */
        released = false;
        status = Cy_IPC_Pipe_SendMessage
        (
            EP_INDEX_OTHER_CPU,
            &pipeData,
            ReleaseCallback
        );
        
        /* When the data successfully into the pipe */
        if (status == CY_IPC_PIPE_SUCCESS)
        {
            while(!released)
            {
                /* wait until the data has been received by the other core */
            }
            /* Update the data to be sent */
            pipeData.LedData = (pipeData.LedData + 1) % 5;
            // Wait 0.5 [s]
            Cy_SysTick_DelayInUs(500000);
        }

    }
}




//==========================================================================================================
#else
    
int main(void)
{
    SystemInit();

    // Example had been originally tested with "cache off", so ensure that caches are turned off (may have been enabled by new startup.c module)
    SCB_DisableICache(); // Disables and invalidates instruction cache
    SCB_DisableDCache(); // Disables, cleans and invalidates data cache

    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    for(;;)
    {
        /* Do nothing here */
    }
}

#endif




/* [] END OF FILE */
