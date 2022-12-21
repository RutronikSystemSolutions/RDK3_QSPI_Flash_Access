/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK3_QSPI_Flash_Access
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-12-21
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg_qspi_memslot.h"

/***************************************************************************
* Global constants
***************************************************************************/
#define SMIF_PRIORITY           (1u)     		/* SMIF interrupt priority */
#define PACKET_SIZE             (256u)  		/* The memory Read/Write packet */
#define TIMEOUT_1_MS            (1000ul) 		/* 1 ms timeout for all blocking functions */
#define QSPI_FREQ            	(80000000ul)	/* The QSPI peripheral clock frequency*/
#define USER_DATA               (0xA0)   		/* User data for memory write and read */
#define FLASH_ADDR				(0x00)			/* Flash memory address*/
#define SECTOR_SIZE				(0x1000)		/* Flash memory sector size*/
#define NUM_BYTES_PER_LINE      (16u)     /* Used when array of data is printed on the console */

/***************************************************************************
* Global variables and functions
***************************************************************************/
void handle_error(void);
void Isr_SMIF(void);
cy_rslt_t Init_SMIF(void);
void print_array(char *message, uint8_t *buf, uint32_t size);

/*QSPI PSRAM object*/
cyhal_qspi_t qspi_flash_obj;

/* SMIF configuration parameters */
cy_stc_smif_config_t SMIFConfig =
{
    /* .mode           */ CY_SMIF_NORMAL,      /* Mode of operation */
    /* .deselectDelay  */ 2U,      			/* Minimum duration of SPI deselection */
    /* .rxClockSel     */ CY_SMIF_SEL_INVERTED_INTERNAL_CLK,     /* Clock source for the receiver clock */
    /* .blockEvent     */ CY_SMIF_BUS_ERROR    /* What happens when there is a read
                                                * to an empty RX FIFO or write to a full TX FIFO
                                                */
};

cyhal_qspi_slave_pin_config_t qspi_flash_pins =
{
		.io[0] = QSPI_IO0,
		.io[1] = QSPI_IO1,
		.io[2] = QSPI_IO2,
		.io[3] = QSPI_IO3,
		.io[4] = NC,
		.io[5] = NC,
		.io[6] = NC,
		.io[7] = NC,
		.ssel = FLASH_SSEL
};


int main(void)
{
    cy_rslt_t result;
    cy_en_smif_status_t result_smif;
    uint8_t mem_buffer[PACKET_SIZE];    /* Buffer to read memory in burst mode */
    uint8_t pttrn=USER_DATA;
    uint32_t index;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    printf("\x1b[2J\x1b[;H");
    printf("RDK3 Flash Access Application.\r\n\r\n");

    /*Initialize QSPI Flash Interface*/
    result = cyhal_qspi_init(&qspi_flash_obj, QSPI_CLK, &qspi_flash_pins, QSPI_FREQ, 0, NULL);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initializes SMIF block*/
    result = Init_SMIF();
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("SMIF initialization failure!\r\n");
    	handle_error();
    }

    /*Select the Flash as active device*/
    result = cyhal_qspi_select_active_ssel(&qspi_flash_obj, FLASH_SSEL);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Erase the Flash memory sector*/
    result_smif =  Cy_SMIF_MemEraseSector(qspi_flash_obj.base, &S25FL064L_SlaveSlot_1, FLASH_ADDR, SECTOR_SIZE, &qspi_flash_obj.context);
    if (result_smif != CY_SMIF_SUCCESS)
    {
    	printf("FLASH erase error.");
    	{handle_error();}
    }

    /*Read the data from the Flash*/
    memset(mem_buffer, 0x00, PACKET_SIZE);
    result_smif = Cy_SMIF_MemRead( qspi_flash_obj.base, &S25FL064L_SlaveSlot_1, FLASH_ADDR, mem_buffer, PACKET_SIZE, &qspi_flash_obj.context );
    if (result_smif != CY_SMIF_SUCCESS)
    {
    	printf("FLASH read error.");
    	{handle_error();}
    }

    print_array("Flash Erased:", mem_buffer, PACKET_SIZE);

	/*Write the data to the Flash*/
	memset(mem_buffer, pttrn, PACKET_SIZE);
	result_smif = Cy_SMIF_MemWrite( qspi_flash_obj.base, &S25FL064L_SlaveSlot_1, FLASH_ADDR, mem_buffer, PACKET_SIZE, &qspi_flash_obj.context );
    if (result_smif != CY_SMIF_SUCCESS)
    {
    	printf("FLASH write error.");
    	{handle_error();}
    }

    /*Read the data from the Flash*/
    memset(mem_buffer, 0x00, PACKET_SIZE);
    result_smif = Cy_SMIF_MemRead( qspi_flash_obj.base, &S25FL064L_SlaveSlot_1, FLASH_ADDR, mem_buffer, PACKET_SIZE, &qspi_flash_obj.context );
    if (result_smif != CY_SMIF_SUCCESS)
    {
    	printf("FLASH read error.");
    	{handle_error();}
    }

    print_array("Flash Programmed:", mem_buffer, PACKET_SIZE);

    /*Check the data consistency*/
    for(index=0; index < PACKET_SIZE; index++)
    {
    	if( mem_buffer[index] != pttrn)
    	{
    		printf("FLASH read error.");
    		handle_error();
    	}
    }
    printf("FLASH check: PASS.\r\n");

    for (;;)
    {
    	CyDelay(500);
    	cyhal_gpio_toggle(LED2);
    }
}



/*******************************************************************************
* Function Name: Isr_SMIF
********************************************************************************
*
* The ISR for the SMIF interrupt. All Read/Write transfers to/from the external
* memory are processed inside the SMIF ISR.
*
*******************************************************************************/
void Isr_SMIF(void)
{
    Cy_SMIF_Interrupt(qspi_flash_obj.base, &qspi_flash_obj.context);
}

/*******************************************************************************
* Function Name: Initialize_SMIF
********************************************************************************
*
* This function initializes the SMIF block
*
*******************************************************************************/
cy_rslt_t Init_SMIF(void)
{
	cy_rslt_t result = CY_RSLT_SUCCESS;
	cy_en_smif_status_t smifStatus;
	cy_stc_sysint_t smifIntConfig =
	{
			.intrSrc = 2u,
			.intrPriority = SMIF_PRIORITY
	};

	cy_en_sysint_status_t intrStatus = Cy_SysInt_Init(&smifIntConfig, Isr_SMIF);
	if(0u != intrStatus)
	{
		return CY_RSLT_TYPE_ERROR;
	}

	smifStatus = Cy_SMIF_Init(qspi_flash_obj.base, &SMIFConfig, TIMEOUT_1_MS, &qspi_flash_obj.context);
	if(0u != smifStatus)
	{
		return CY_RSLT_TYPE_ERROR;
	}

	/* Configure slave select and data select. These settings depend on the pins
	 * selected in the Device and QSPI configurators.
	 */
	Cy_SMIF_SetDataSelect(qspi_flash_obj.base, qspi_flash_obj.slave_select, S25FL064L_SlaveSlot_1.dataSelect);
	Cy_SMIF_Enable(qspi_flash_obj.base, &qspi_flash_obj.context);
	smifStatus = Cy_SMIF_Memslot_Init(qspi_flash_obj.base, (cy_stc_smif_block_config_t *)&smifBlockConfig, &qspi_flash_obj.context);
	if(0u != intrStatus)
	{
		return CY_RSLT_TYPE_ERROR;
	}

	Cy_SMIF_SetMode(qspi_flash_obj.base, CY_SMIF_NORMAL);
	NVIC_EnableIRQ(smifIntConfig.intrSrc); /* Enable the SMIF interrupt */

	return result;
}

/*******************************************************************************
* Function Name: print_array
****************************************************************************//**
* Summary:
*  Prints the content of the buffer to the UART console.
*
* Parameters:
*  message - message to print before array output
*  buf - buffer to print on the console.
*  size - size of the buffer.
*
*******************************************************************************/
void print_array(char *message, uint8_t *buf, uint32_t size)
{
    printf("%s (%lu bytes):\r\n", message, (unsigned long)size);
    printf("-------------------------\r\n");

    for(uint32_t index = 0; index < size; index++)
    {
        printf("0x%02X ", buf[index]);

        if(0u == ((index + 1) % NUM_BYTES_PER_LINE))
        {
            printf("\r\n");
        }
    }
}

void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    cyhal_gpio_write(LED3, CYBSP_LED_STATE_ON);

    CY_ASSERT(0);
}

/* [] END OF FILE */
