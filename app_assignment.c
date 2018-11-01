/*
*********************************************************************************************************
*********************************************************************************************************
*
*                                             INCLUDE FILES
*
*********************************************************************************************************
*********************************************************************************************************
*/

#include  <math.h>
#include  <lib_math.h>
#include  <cpu_core.h>
#include  <app_cfg.h>
#include  <os.h>
#include  <fsl_os_abstraction.h>
#include  <system_MK64F12.h>
#include  <board.h>
#include  <bsp_ser.h>
#include "fsl_interrupt_manager.h"
#include "fsl_gpio_common.h"

/*
*********************************************************************************************************
*********************************************************************************************************
*
*                                            LOCAL DEFINES
*             		[FTM0_MOD values to set to obtain the desired frequencies]
*
*********************************************************************************************************
*********************************************************************************************************
*/

#define mod_freq_10_Hz	 46875
#define mod_freq_20_Hz	 23438
#define ADC_offset	 	 1024

/*
*********************************************************************************************************
*********************************************************************************************************
*
*                                       LOCAL GLOBAL VARIABLES
*
*********************************************************************************************************
*********************************************************************************************************
*/

static  OS_TCB       AppStartupTaskTCB;
static  CPU_STK      AppStartupTaskStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB       AppTaskTCB;
static  CPU_STK      AppTaskStk[APP_CFG_TASK_START_STK_SIZE];

static	uint16_t	 RGB_COLOR;

static	uint16_t	 new_adc_val;
static 	uint16_t	 old_adc_val;

static	uint16_t	 new_range;
static	uint16_t	 old_range;

static	uint32_t	 NEW_MOD;
static	uint32_t	 OLD_MOD;

static	OS_SEM		 S;

/*
*********************************************************************************************************
*********************************************************************************************************
*
*                                      LOCAL FUNCTION PROTOTYPES
*
*********************************************************************************************************
*********************************************************************************************************
*/

static  void  AppStartupTask (void  *p_arg);
static  void  AppTask (void  *p_arg);

static  void  setup_ftm0(void);
static	void  update_ftm0(void);
static  void  BSP_FTM0_int_hdlr(void);

static	void  setup_ADC0(void);
static	void  ADC_read16b(void);
static	void  ADC0_int_hdlr(void);

/*
*********************************************************************************************************
*********************************************************************************************************
*
*                                             	MAIN
*
*********************************************************************************************************
*********************************************************************************************************
*/

int  main (void)
{
    OS_ERR   os_err;

#if (CPU_CFG_NAME_EN == DEF_ENABLED)
    CPU_ERR  cpu_err;
#endif


    new_adc_val = old_adc_val = 0;								// Variables initialization
    new_range = old_range = -1;
    NEW_MOD = OLD_MOD = 0xFFFF;
    RGB_COLOR = BOARD_GPIO_LED_GREEN;

    OSSemCreate(&S,"ADC_OS_SEM", 0, &os_err);					// Create the semaphore S

    hardware_init();
    GPIO_DRV_Init(switchPins, ledPins);

    setup_ftm0();
    setup_ADC0();


#if (CPU_CFG_NAME_EN == DEF_ENABLED)
    CPU_NameSet((CPU_CHAR *)"MK64FN1M0VMD12",
                (CPU_ERR  *)&cpu_err);
#endif

    BSP_Ser_Init(115200u);
    OSA_Init();

    OSTaskCreate(&AppStartupTaskTCB,							// Create the start task
                 "App Startup Task",
                 AppStartupTask,
                 0u,
                 APP_CFG_TASK_START_PRIO,
                 &AppStartupTaskStk[0u],
                 (APP_CFG_TASK_START_STK_SIZE / 10u),
                 APP_CFG_TASK_START_STK_SIZE,
                 0u,
                 0u,
                 0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                 &os_err);

    OSA_Start();							 					// Start multitasking (i.e. give control to uC/OS-III).

    while (DEF_ON) {											// Should Never Get Here
        ;
    }
}
/*
*********************************************************************************************************
*                                          TASK
*
* Argument(s) : p_arg   is the argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Notes       : (1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                   used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/
static  void  AppStartupTask (void *p_arg)
{
	 OS_ERR    os_err;

	    (void)p_arg;                                                // See Note #1


	    CPU_Init();                                                 // Initialize the uC/CPU Services.
	    Mem_Init();                                                 // Initialize the Memory Management Module
	    Math_Init();                                                // Initialize the Mathematical Module

	#if OS_CFG_STAT_TASK_EN > 0u
	    OSStatTaskCPUUsageInit(&os_err);                            // Compute CPU capacity with no task running
	#endif

	#ifdef CPU_CFG_INT_DIS_MEAS_EN
	    CPU_IntDisMeasMaxCurReset();
	#endif

    OSTaskCreate(&AppTaskTCB,
                 "App Task",
                 AppTask,
                 0u,
                 APP_CFG_TASK_START_PRIO,
                 &AppTaskStk[0u],
                 (APP_CFG_TASK_START_STK_SIZE / 10u),
                 APP_CFG_TASK_START_STK_SIZE,
                 0u,
                 0u,
                 0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                 &os_err);

}

static  void  AppTask (void *p_arg)
{
	OS_ERR    os_err;

    (void)p_arg;

    GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_RED);
    GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_GREEN);
    GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_BLUE);

    while(DEF_TRUE)
    {
    	ADC_read16b();

    	OSSemPend( &S, 0, OS_OPT_PEND_BLOCKING, NULL, &os_err);				// Put semaphore S in Pending status

    	if(new_adc_val != old_adc_val)
    	{
    		if((new_adc_val >= 0)&&(new_adc_val <= 19859))
    	    {
    			new_range = 1;
    			if(new_adc_val >= 9930)
    				NEW_MOD = mod_freq_20_Hz;
    			else
    				NEW_MOD = mod_freq_10_Hz;
    	    }
    		else if((new_adc_val >= 19860)&&(new_adc_val <= 39718))
    	    {
    			new_range = 2;
    			if(new_adc_val >= 29789)
    				NEW_MOD = mod_freq_20_Hz;
    			else
    				NEW_MOD=mod_freq_10_Hz;
    	    }
    		else if((new_adc_val >= 39719)&&(new_adc_val <= 59577))
    	    {
    			new_range = 3;
    			if(new_adc_val >= 49648)
    				NEW_MOD = mod_freq_20_Hz;
    			else
    				NEW_MOD = mod_freq_10_Hz;
    	    }
    		else
    	    {
    	    	new_range = 4;
    	    }
    	}

    	if(new_range != old_range)
    	{
    		if(new_range == 1)
    		{
    			GPIO_DRV_SetPinOutput(RGB_COLOR);
    			RGB_COLOR = BOARD_GPIO_LED_GREEN;
    		}
    		else if(new_range == 2)
    		{
    			GPIO_DRV_SetPinOutput(RGB_COLOR);
    			RGB_COLOR = BOARD_GPIO_LED_BLUE;
    		}
    		else if(new_range == 3)
    		{
    			GPIO_DRV_SetPinOutput(RGB_COLOR);
    			RGB_COLOR = BOARD_GPIO_LED_RED;
    		}
    		else if(new_range == 4)
    		{
    			GPIO_DRV_SetPinOutput(RGB_COLOR);
    			RGB_COLOR = BOARD_GPIO_LED_RED;
    			GPIO_DRV_ClearPinOutput(BOARD_GPIO_LED_RED);
    		}

    		old_range = new_range;
    	}

    	if(NEW_MOD != OLD_MOD)
    	{
    		update_ftm0();
    		OLD_MOD = NEW_MOD;
    	}

    	OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &os_err);

    }
}

static  void  setup_ftm0(void)
{
	INT_SYS_EnableIRQ(FTM0_IRQn);									// Enable the interrupt signals on FTM0_IRQn
	INT_SYS_InstallHandler(FTM0_IRQn, BSP_FTM0_int_hdlr);			// ISR

	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK; 								// Turn on FTM0 timer
	FTM0_CONF = 0xC0; 												// Set the timer in Debug mode, with BDM mode = 0xC0
	FTM0_FMS =  0x0;												// Enable modifications to the FTM0 configuration
	FTM0_MODE |= (FTM_MODE_WPDIS_MASK|FTM_MODE_FTMEN_MASK);			// Allowing writing in the registers
}

static	void  setup_ADC0(void)
{
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	ADC0_CFG1 |= ADC_CFG1_MODE(3);									// 16bits ADC
	ADC0_SC1A |= ADC_SC1_ADCH(31);									// Disable the module, ADCH = 11111
	ADC0_OFS = ADC_offset;											// Recorded value != 0 when input = 0V

	INT_SYS_EnableIRQ(ADC0_IRQn);									// Enable the interrupt signals on ADC0_IRQn
	INT_SYS_InstallHandler(ADC0_IRQn, ADC0_int_hdlr);				// ISR
}

static	void  update_ftm0(void)
{
	FTM0_SC = FTM_SC_CLKS(0x0);										// Reset the FTM counter
	FTM0_CNTIN = FTM_CNTIN_INIT(0);									// Initial value of 16 bit counter
	FTM0_MOD = FTM_MOD_MOD(NEW_MOD);								// MOD UPDATE TO ACT ON FREQUENCY

	FTM0_SC = (FTM_SC_PS(7) | 										// 00 No clock selected (This in effect disables the FTM counter.)
			   FTM_SC_CLKS(0x1)|									// 01 System clock
			   FTM_SC_TOIE_MASK);									// 10 Fixed frequency clock
																	// 11 External clock
}


static void ADC_read16b(void)
{
	ADC0_SC1A = (12|ADC_SC1_AIEN_MASK);								// 0 Conversion complete interrupt disabled
																	// 1 Conversion complete interrupt enabled			[AIEN bit]
}

// Interrupt handler for FTM0
void BSP_FTM0_int_hdlr(void)
{
	CPU_CRITICAL_ENTER();											// Disable interrupts
	OSIntEnter();                                               	// Tell the OS that we are starting an ISR

	FTM0_SC &= 0x7F;

		if(new_range != 4)
			GPIO_DRV_TogglePinOutput(RGB_COLOR);					// Blink the RGB led

	CPU_CRITICAL_EXIT();											// Re-enable interrupts
	OSIntExit();													// Tell the OS we are exiting from an ISR

}

// Interrupt handler for ACD0
static	void  ADC0_int_hdlr(void)
{
	OS_ERR    os_err;

	CPU_CRITICAL_ENTER();											// Disable interrupts
	OSIntEnter();                                               	// Tell the OS that we are starting an ISR

	old_adc_val = new_adc_val;										// Update the previous value
	new_adc_val = ADC0_RA;											// Store a new read converted value

	ADC0_SC1A = ADC_SC1_ADCH(31);

	OSSemPost( &S, OS_OPT_POST_1 |
			OS_OPT_POST_NO_SCHED, &os_err);							// Put semaphore S in Post mode

	CPU_CRITICAL_EXIT();											// Re-enable interrupts
	OSIntExit();													// Tell the OS we are exiting from an ISR
}
