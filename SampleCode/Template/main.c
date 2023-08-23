/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include "EEPROM_Emulate.h"

#define LED_R									(PH0)
#define LED_Y									(PH1)
#define LED_G									(PH2)

#define DATA_FLASH_OFFSET  						(0x1E000)		//START : 120K , 4 Kbytes page erase 

#define DATA_FLASH_AMOUNT						(24)
#define DATA_FLASH_PAGE  						(2)

typedef enum{
	flag_DEFAULT = 0 ,
		
	flag_Read_Data ,	
	flag_Write_Data ,
	
	flag_END	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

extern int IsDebugFifoEmpty(void);

int set_data_flash_base(uint32_t u32DFBA)
{
    uint32_t   au32Config[2];          /* User Configuration */

    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();

    /* Check if Data Flash Size is 64K. If not, to re-define Data Flash size and to enable Data Flash function */
    if (FMC_ReadConfig(au32Config, 2) < 0)
        return -1;

    if (((au32Config[0] & 0x01) == 1) || (au32Config[1] != u32DFBA) )
    {
        FMC_ENABLE_CFG_UPDATE();
		
		FMC_Erase(FMC_CONFIG_BASE);
		
        au32Config[0] &= ~0x1;
        au32Config[1] = u32DFBA;
        if (FMC_WriteConfig(au32Config, 2) < 0)
            return -1;

        FMC_ReadConfig(au32Config, 2);
        if (((au32Config[0] & 0x01) == 1) || (au32Config[1] != u32DFBA))
        {
            printf("Error: Program Config Failed!\n");
            /* Disable FMC ISP function */
            FMC_Close();
            SYS_LockReg();
            return -1;
        }

        printf("chip reset\n");
        /* Reset Chip to reload new CONFIG value */
        //SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
        NVIC_SystemReset();
    }
	
    return 0;                          /* success */
}


void Emulate_EEPROM_WriteTest(void)
{
	uint8_t cnt = 0;
	uint8_t i = 0;
	static uint8_t incr_base = 0;

	printf("%s , incr_base : 0x%2X\r\n" , __FUNCTION__ , incr_base);

    SYS_UnlockReg();
    FMC_Open();
	
    FMC_ENABLE_AP_UPDATE();
	for (i = 0 ; i < DATA_FLASH_AMOUNT; i ++)
	{
		Write_Data(i%DATA_FLASH_AMOUNT, incr_base + (cnt++) );
	}

	incr_base++;	//incr_base += 0x10;

    FMC_DISABLE_AP_UPDATE();
    FMC_Close();
    SYS_LockReg();
}

void Emulate_EEPROM_ReadTest(void)
{
	uint8_t i = 0;
	uint8_t cnt = 0;

	printf("%s\r\n" , __FUNCTION__);

    SYS_UnlockReg();
    FMC_Open();

	for (i = 0 ; i < DATA_FLASH_AMOUNT; i ++)
	{
		Read_Data(i%DATA_FLASH_AMOUNT, &cnt );
		printf("0x%2X , ", cnt);
		if ((i+1)%8 ==0)
		{
			printf("\r\n");
		}
	}

    FMC_Close();
    SYS_LockReg();
	
}

void Emulate_EEPROM_Process(void)
{
	uint8_t string[] = "\r\nEmulate_EEPROM_Process finish !\r\n\r\n" ; 

	if (is_flag_set(flag_Read_Data))
	{
		set_flag(flag_Read_Data , DISABLE);

		Emulate_EEPROM_ReadTest();
		
		UART_Write(UART0 , string , strlen((char*)string) );
	}

	if (is_flag_set(flag_Write_Data))
	{
		set_flag(flag_Write_Data , DISABLE);

		Emulate_EEPROM_WriteTest();
	}

	
}

void Emulate_EEPROM(void)
{
//    SYS_UnlockReg();

    /* Enable FMC ISP function */
//    FMC_Open();

	//need to manual set data flash address ?
//    if (set_data_flash_base(DATA_FLASH_OFFSET) < 0)
//    {
//        printf("Failed to set Data Flash base address!\r\n");
//    }

	/* Test Init_EEPROM() */
	Init_EEPROM(DATA_FLASH_AMOUNT, DATA_FLASH_PAGE);
	Search_Valid_Page();	
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1':
				Emulate_EEPROM_ReadTest();
				break;	
			case '2':
				Emulate_EEPROM_WriteTest();
				break;	
			
			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();
			
				break;		
			
		}
	}
}

void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}

void TMR1_IRQHandler(void)
{
//	static uint32_t log = 0;	
	static uint16_t CNT = 0;
	static uint16_t CNT_read = 0;
	static uint16_t CNT_write = 0;
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
	
		if (CNT++ > 1000)
		{		
			CNT = 0;
//			printf("%s : %2d\r\n" , __FUNCTION__ , log++);

			LED_G ^= 1;
		}

		if (CNT_write++ >= 500)
		{		
			CNT_write = 0;
			set_flag(flag_Write_Data , ENABLE);
		}

		if (CNT_read++ >= 500)
		{		
			CNT_read = 0;
			set_flag(flag_Read_Data , ENABLE);
		}	
		
    }
}

void TIMER1_HW_Init(void)
{
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void TIMER0_HW_Init(void)
{
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
}

void TIMER0_Polling(uint32_t u32Usec)
{
	TIMER_Delay(TIMER0, u32Usec);
}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

	TIMER0_HW_Init();
	TIMER1_HW_Init();
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{	

    SYS_Init();
	UART0_Init();

	Emulate_EEPROM();
	
	LED_Init();
	TIMER1_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
//		TIMER0_Polling(1000);

//		Emulate_EEPROM_Process();
		LED_Y ^= 1;	
    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
