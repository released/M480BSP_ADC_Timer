/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "misc_config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_TIMER_PERIOD_SPECIFIC                 (flag_PROJ_CTL.bit1)
#define FLAG_PROJ_REVERSE2                 		        (flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                              (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;


uint32_t u32AVDDVoltage = 0;

#define VBG_VOLTAGE                                     (1200)
/* The last line of GetAVDDCodeByADC() need revise when ADC_SAMPLE_COUNT is changed. */
#define ADC_SAMPLE_COUNT                                (8)
#define ADC_SAMPLE_POWER	 				            (3)
#define ADC_SAMPLE_DROP 						        (4ul)
#define ADCTotalLength                                  (ADC_SAMPLE_COUNT+ADC_SAMPLE_DROP)   // drop first 4 ADC result 
#define ADC_DIGITAL_SCALE(void) 		                (0xFFFU >> ((0) >> (3U - 1U)))		//0: 12 BIT 

volatile int16_t g_iConversionData[ADCTotalLength] = {0};

#define ENABLE_ADC_TRIG_BY_TIMER
/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

void delay_ms(uint16_t ms)
{
	#if 1
    uint32_t tickstart = get_tick();
    uint32_t wait = ms;
	uint32_t tmp = 0;
	
    while (1)
    {
		if (get_tick() > tickstart)	// tickstart = 59000 , tick_counter = 60000
		{
			tmp = get_tick() - tickstart;
		}
		else // tickstart = 59000 , tick_counter = 2048
		{
			tmp = 60000 -  tickstart + get_tick();
		}		
		
		if (tmp > wait)
			break;
    }
	
	#else
	TIMER_Delay(TIMER0, 1000*ms);
	#endif
}

void TIMER3_Init(void)
{

    /* Set timer3 periodic time-out period is 3us if timer clock is 12 MHz */
    // TIMER_SET_CMP_VALUE(TIMER3, 36);//TIMER3->CMP = 36;
    TIMER_SET_CMP_VALUE(TIMER3, 5);//TIMER3->CMP = 36;

    /* Start timer counter in periodic mode and enable timer interrupt trigger EADC */
    TIMER3->CTL = TIMER_PERIODIC_MODE;
    TIMER3->TRGCTL |= TIMER_TRGCTL_TRGEADC_Msk;

}

void Sort_tab(uint16_t tab[], uint8_t length)
{
	uint8_t l = 0x00, exchange = 0x01; 
	uint16_t tmp = 0x00;

	/* Sort tab */
	while(exchange==1) 
	{ 
		exchange=0; 
		for(l=0; l<length-1; l++) 
		{
			if( tab[l] > tab[l+1] ) 
			{ 
				tmp = tab[l]; 
				tab[l] = tab[l+1]; 
				tab[l+1] = tmp; 
				exchange=1; 
			}
		}
	} 
}

void EADC00_IRQHandler(void)
{
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

uint16_t ADC_To_Voltage(uint16_t adc_value)
{
	uint16_t volt = 0;

	// volt = (uint16_t) (AVdd*adc_value)/ADC_DIGITAL_SCALE();
	volt = (float) (u32AVDDVoltage*adc_value)/ADC_DIGITAL_SCALE();
	
	#if 1   //debug
    printf("%s[0x%4X,%4d]%4dmv,AVdd = %4dmv\r\n",__FUNCTION__ ,adc_value ,adc_value, volt, u32AVDDVoltage);
	#endif

	return volt;	
}


uint16_t get_ADC_Channel_By_Timer(uint8_t ch)
{
    uint16_t  val = 0;
    uint32_t module_num = 1;
    uint32_t module_mask = BIT1;
  	volatile uint32_t sum = 0;
    uint16_t tmp = 0;

    /* Set the ADC operation mode as single, input mode as single-end and enable the analog input channel 3 */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    EADC_ConfigSampleModule(EADC, module_num, EADC_TIMER3_TRIGGER , ch);
    EADC_SetExtendSampleTime(EADC, module_num, 0xFF);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Enable sample module A/D ADINT0 interrupt. */
    EADC_ENABLE_INT(EADC, BIT0);
    /* Enable sample module 0 interrupt. */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, module_mask);
    NVIC_EnableIRQ(EADC00_IRQn);
    
    TIMER_Start(TIMER3);

    for ( tmp = 0 ; tmp < (ADCTotalLength) ; tmp++)
    {  
        __WFI();

        while(EADC_GET_DATA_VALID_FLAG(EADC, (module_mask)) != (module_mask));

        g_iConversionData[tmp] = EADC_GET_CONV_DATA(EADC, module_num);  

    }

    TIMER_Stop(TIMER3);

    EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, module_mask);
    EADC_DISABLE_INT(EADC, BIT0);
    NVIC_DisableIRQ(EADC00_IRQn);

    // printf("0X%4X:adc voltage is %dmV if Reference voltage is %4d mv\n", val ,(u32AVDDVoltage*val)/4095 , u32AVDDVoltage);

    Sort_tab( (uint16_t*) g_iConversionData,ADCTotalLength);
    for (tmp = ADC_SAMPLE_DROP/2; tmp < ADCTotalLength - ADC_SAMPLE_DROP/2; tmp++)
    {
        sum += g_iConversionData[tmp];
    }

    sum = sum >> ADC_SAMPLE_POWER;
    val = sum;

    return (val);    
}

uint16_t get_ADC_Channel(uint8_t ch)
{
    uint16_t  val = 0;
    uint32_t module_num = 1;
    uint32_t module_mask = BIT1;
  	volatile uint32_t sum = 0;
    uint16_t tmp = 0;

    /* Set the ADC operation mode as single, input mode as single-end and enable the analog input channel 3 */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    EADC_ConfigSampleModule(EADC, module_num, EADC_ADINT0_TRIGGER , ch);
    EADC_SetExtendSampleTime(EADC, module_num, 0xFF);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Enable sample module A/D ADINT0 interrupt. */
    // EADC_ENABLE_INT(EADC, BIT0);
    /* Enable sample module 0 interrupt. */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, module_mask);
    NVIC_EnableIRQ(EADC00_IRQn);
    
    for ( tmp = 0 ; tmp < (ADCTotalLength) ; tmp++)
    {  
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);
        EADC_ENABLE_INT(EADC, BIT0);
        EADC_START_CONV(EADC, module_mask);

        __WFI();

        while(EADC_GET_DATA_VALID_FLAG(EADC, (module_mask)) != (module_mask));

        g_iConversionData[tmp] = EADC_GET_CONV_DATA(EADC, module_num);  

    }

    EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, module_mask);
    EADC_DISABLE_INT(EADC, BIT0);
    NVIC_DisableIRQ(EADC00_IRQn);

    // printf("0X%4X:adc voltage is %dmV if Reference voltage is %4d mv\n", val ,(u32AVDDVoltage*val)/4095 , u32AVDDVoltage);

    Sort_tab( (uint16_t*) g_iConversionData,ADCTotalLength);
    for (tmp = ADC_SAMPLE_DROP/2; tmp < ADCTotalLength - ADC_SAMPLE_DROP/2; tmp++)
    {
        sum += g_iConversionData[tmp];
    }

    sum = sum >> ADC_SAMPLE_POWER;
    val = sum;

    return (val);    
}

uint32_t GetAVDDVoltage(void)
{
    uint32_t  u32ConversionResult = 0;
    uint32_t  u32MvAVDD;
    uint32_t u32Count, u32Sum, u32Data, u32TimeOutCnt;

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    /* Set sample module 16 external sampling time to 0xF */
    EADC_SetExtendSampleTime(EADC, 16, 0xFF);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    // /* Enable the sample module 16 interrupt.  */
    // EADC_ENABLE_INT(EADC, BIT0);//Enable sample module A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT16);//Enable sample module 16 interrupt.
    NVIC_EnableIRQ(EADC00_IRQn);

    u32Sum = 0;

    /* sample times are according to ADC_SAMPLE_COUNT definition */
    for(u32Count = 0; u32Count < ADC_SAMPLE_COUNT; u32Count++)
    {
        /* Delay for band-gap voltage stability */
        CLK_SysTickDelay(100);
        EADC_ENABLE_INT(EADC, BIT0);//Enable sample module A/D ADINT0 interrupt.

        /* Start A/D conversion */
        EADC_START_CONV(EADC, BIT16);

        u32Data = 0;
        /* Wait conversion done */
        u32TimeOutCnt = SystemCoreClock;

        __WFI();
        
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT16);

        while(EADC_GET_DATA_VALID_FLAG(EADC, (BIT16)) != (BIT16))
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for EADC conversion done time-out!\r\n");
                break;
            }            
        }

        EADC_DISABLE_INT(EADC, BIT0); 
        /* Get the conversion result */
        u32Data = EADC_GET_CONV_DATA(EADC, 16);
        /* Sum each conversion data */
        u32Sum += u32Data;
    }

    u32ConversionResult = u32Sum >> ADC_SAMPLE_POWER;

    EADC_Close(EADC);

    /* u32ConversionResult = VBG * 4096 / Vref, Vref = AVDD */
    /* => AVDD = VBG * 4096 / u32ConversionResult */
    u32MvAVDD = (VBG_VOLTAGE << 12) / u32ConversionResult;

    printf("Conversion result: 0x%X\r\n", u32ConversionResult);

    return (uint32_t)u32MvAVDD;
}


void ADC_Init(void)
{
    u32AVDDVoltage = GetAVDDVoltage();
    printf("AVDD Voltage: %dmV\r\n", u32AVDDVoltage);

    #if defined (ENABLE_ADC_TRIG_BY_TIMER)
    TIMER3_Init();
    #endif

    // get_ADC_Channel(11);
}

//
// check_reset_source
//
uint8_t check_reset_source(void)
{
    uint32_t src = SYS_GetResetSrc();

    if ((SYS->CSERVER & SYS_CSERVER_VERSION_Msk) == 0x1)    // M48xGCAE
    {
		printf("PN : M48xGCAE\r\n");
    }
    else    // M48xIDAE
    {
		printf("PN : M48xIDAE\r\n");
    }

    SYS->RSTSTS |= 0x1FF;
    printf("Reset Source <0x%08X>\r\n", src);

    #if 1   //DEBUG , list reset source
    if (src & BIT0)
    {
        printf("0)POR Reset Flag\r\n");       
    }
    if (src & BIT1)
    {
        printf("1)NRESET Pin Reset Flag\r\n");       
    }
    if (src & BIT2)
    {
        printf("2)WDT Reset Flag\r\n");       
    }
    if (src & BIT3)
    {
        printf("3)LVR Reset Flag\r\n");       
    }
    if (src & BIT4)
    {
        printf("4)BOD Reset Flag\r\n");       
    }
    if (src & BIT5)
    {
        printf("5)System Reset Flag \r\n");       
    }
    if (src & BIT6)
    {
        printf("6)HRESET Reset Flag \r\n");       
    }
    if (src & BIT7)
    {
        printf("7)CPU Reset Flag\r\n");       
    }
    if (src & BIT8)
    {
        printf("8)CPU Lockup Reset Flag\r\n");       
    }
    #endif
    
    if (src & SYS_RSTSTS_PORF_Msk) {
        SYS_ClearResetSrc(SYS_RSTSTS_PORF_Msk);
        
        printf("power on from POR\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_PINRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_PINRF_Msk);
        
        printf("power on from nRESET pin\r\n");
        return FALSE;
    } 
    else if (src & SYS_RSTSTS_WDTRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_WDTRF_Msk);
        
        printf("power on from WDT Reset\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_LVRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_LVRF_Msk);
        
        printf("power on from LVR Reset\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_BODRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_BODRF_Msk);
        
        printf("power on from BOD Reset\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_SYSRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_SYSRF_Msk);
        
        printf("power on from System Reset\r\n");
        return FALSE;
    } 
    else if (src & SYS_RSTSTS_CPURF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_CPURF_Msk);

        printf("power on from CPU reset\r\n");
        return FALSE;         
    }    
    else if (src & SYS_RSTSTS_CPULKRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_CPULKRF_Msk);
        
        printf("power on from CPU Lockup Reset\r\n");
        return FALSE;
    }   
    
    printf("power on from unhandle reset source\r\n");
    return FALSE;
}

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 200) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_SPECIFIC = 1;
		}
        
		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;
    uint16_t val = 0;

    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        // printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PH0 ^= 1;             
    }

    if (FLAG_PROJ_TIMER_PERIOD_SPECIFIC)
    {
        FLAG_PROJ_TIMER_PERIOD_SPECIFIC = 0;

        #if defined (ENABLE_ADC_TRIG_BY_TIMER)
        val = get_ADC_Channel_By_Timer(11);
        ADC_To_Voltage(val);
        #else
        val = get_ADC_Channel(11);
        ADC_To_Voltage(val);
        #endif
    }

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
		printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
                SYS_UnlockReg();
				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
                // SYS_ResetCPU();     // Not reset I/O and peripherals
                SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
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

	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());    	

//    printf("Product ID 0x%8X\n", SYS->PDID);
	
	#endif	

    #if 0
    printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH0MFP_Msk)) | (SYS_GPH_MFPL_PH0MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH1MFP_Msk)) | (SYS_GPH_MFPL_PH1MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH2MFP_Msk)) | (SYS_GPH_MFPL_PH2MFP_GPIO);

	//EVM LED
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
//    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /***********************************/
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
	
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HIRC, 0);

    /***********************************/
    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
	
    /***********************************/
    CLK_EnableModuleClock(EADC_MODULE);
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    // Set multi-function pins for EADC channels. //
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB11MFP_Msk)) | SYS_GPB_MFPH_PB11MFP_EADC0_CH11;

    GPIO_DISABLE_DIGITAL_PATH(PB, BIT11);

    SYS_SetVRef(SYS_VREFCTL_VREF_PIN);

    /***********************************/
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

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

	GPIO_Init();
	UART0_Init();
	TIMER1_Init();
    check_reset_source();

    // SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    ADC_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
