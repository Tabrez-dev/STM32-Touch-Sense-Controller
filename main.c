#include <stdint.h>

// Define TSC interrupt status and clear bits (assumed as bit0 for EOAF)
#define TSC_ISR_EOAF   (1U << 0)
#define TSC_ICR_EOAIC  (1U << 0)

uint32_t group1_threshold = 1900;
uint32_t group2_threshold = 3200;
uint32_t group3_threshold = 3300;
extern void _estack(void); //defined in linker script

volatile uint32_t sensor_data[3];
volatile uint8_t new_data_available = 0;

/*----------------------------------------------------------------------------
  Peripheral Register Definitions
  (Addresses are written as base + offset for clarity)
  ----------------------------------------------------------------------------*/
// RCC base: 0x40021000
uint32_t volatile *pRCC_AHBENR  = (uint32_t*)(0x40021000 + 0x14);  // RCC_AHBENR: enables GPIO clocks; for GPIOA, bit 17
uint32_t volatile *pRCC_APB2ENR = (uint32_t*)(0x40021000 + 0x18);  // RCC_APB2ENR: enables SYSCFG (bit 0) and USART1 (bit 14)

// For TSC, PA2 and PA3 (group 1) ,PA6 and  PA7 (group 2), PB0 and PB1 (group 3)
uint32_t volatile *pGPIOA_MODER = (uint32_t*)(0x48000000 + 0x00);   // GPIOA mode register
uint32_t volatile *pGPIOA_AFRH  = (uint32_t*)(0x48000000 + 0x24);   // GPIOA alternate function high register
uint32_t volatile *pGPIOA_AFRL  = (uint32_t*)(0x48000000 + 0x20);   

// For TSC
uint32_t volatile *pGPIOB_MODER = (uint32_t*)(0x48000400 + 0x00);   // GPIOB mode register
uint32_t volatile *pGPIOB_AFRL  = (uint32_t*)(0x48000400 + 0x20);
uint32_t volatile *pGPIOB_AFRH  = (uint32_t*)(0x48000400 + 0x24);   
//For user LEDs
uint32_t volatile *pGPIOC_MODER = (uint32_t*)(0x48000800 + 0x00);   // GPIOC mode register
uint32_t volatile *pGPIOC_ODR   = (uint32_t*)(0x48000800 + 0x14);   // GPIOC output data register

// EXTI base: 0x40010400
uint32_t volatile *pEXTI_IMR  = (uint32_t*)(0x40010400 + 0x00);      // EXTI Interrupt Mask Register
uint32_t volatile *pEXTI_RTSR = (uint32_t*)(0x40010400 + 0x08);      // EXTI Rising Trigger Selection Register
uint32_t volatile *pEXTI_PR   = (uint32_t*)(0x40010400 + 0x14);      // EXTI Pending Register

// NVIC: using its ISER register; base address 0xE000E100
uint32_t volatile *pNVIC_ISER = (uint32_t*)(0xE000E100 + 0x00);      // NVIC Interrupt Set-Enable Register

// USART1 base: 0x40013800
uint32_t volatile *pUSART1_CR  = (uint32_t*)(0x40013800 + 0x00);      // USART1 Control Register
uint32_t volatile *pUSART1_BRR = (uint32_t*)(0x40013800 + 0x0C);      // USART1 Baud Rate Register
uint32_t volatile *pUSART1_ISR = (uint32_t*)(0x40013800 + 0x1C);      // USART1 Interrupt and Status Register
uint32_t volatile *pUSART1_TDR = (uint32_t*)(0x40013800 + 0x28);      // USART1 Transmit Data Register


uint32_t volatile *pTSC_CR     = (uint32_t*)(0x40024000 + 0x0000);  
uint32_t volatile *pTSC_IER    = (uint32_t*)(0x40024000 + 0x0004);  
uint32_t volatile *pTSC_ICR    = (uint32_t*)(0x40024000 + 0x0008);  
uint32_t volatile *pTSC_ISR    = (uint32_t*)(0x40024000 + 0x000C);  
uint32_t volatile *pTSC_IOHCR  = (uint32_t*)(0x40024000 + 0x0010);  
uint32_t volatile *pTSC_IOASCR = (uint32_t*)(0x40024000 + 0x0018);  
uint32_t volatile *pTSC_IOSCR  = (uint32_t*)(0x40024000 + 0x0020);  
uint32_t volatile *pTSC_IOCCR  = (uint32_t*)(0x40024000 + 0x0028);  
uint32_t volatile *pTSC_IOGCSR = (uint32_t*)(0x40024000 + 0x0030);  
uint32_t volatile *pTSC_IOG1CR = (uint32_t*)(0x40024000 + 0x0034);  
uint32_t volatile *pTSC_IOG2CR = (uint32_t*)(0x40024000 + 0x0038);  
uint32_t volatile *pTSC_IOG3CR = (uint32_t*)(0x40024000 + 0x003C);  



/*----------------------------------------------------------------------------
  Function Prototypes
  ----------------------------------------------------------------------------*/
void usart1_init(void);
void usart1_send_char(char c);
void usart1_send_string(const char *s);
void usart1_send_uint32(uint32_t num);

int main(void);

void _reset(void);
void NMI_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TSC_IRQHandler(void);
void Default_Handler(void);

/*----------------------------------------------------------------------------
  USART1 Initialization
  ----------------------------------------------------------------------------*/
void usart1_init(void)
{
	// Enable clock for GPIOA: set bit 17 in RCC_AHBENR (IOPAEN)
	*pRCC_AHBENR |= (1 << 17);

	// Enable clocks for SYSCFG (bit 0) and USART1 (bit 14) in RCC_APB2ENR
	*pRCC_APB2ENR |= (1 << 0) | (1 << 14);

	// Configure PA9 and PA10 for alternate function mode (for USART1)
	*pGPIOA_MODER &= ~((0x3U << 18) | (0x3U << 20));  // clear existing bits
	*pGPIOA_MODER |= ((0x2U << 18) | (0x2U << 20));   // set to alternate function mode

	// Set alternate function AF1 for PA9 and PA10 (in AFRH; PA8-15 share this register)
	*pGPIOA_AFRH &= ~((0xFU << 4) | (0xFU << 8));
	*pGPIOA_AFRH |= ((0x1U << 4) | (0x1U << 8));      // AF1: USART1

	// Disable USART1 before configuration
	*pUSART1_CR &= ~(1U << 0);

	// Configure USART1 for desired baud rate
	uint32_t FREQ = 8000000; // Peripheral clock frequency
	uint32_t baud = 115200;  // Desired baud rate
	uint32_t usartdiv = FREQ / baud;
	*pUSART1_BRR = usartdiv;

	// Enable USART1: set UE (bit 0) and TE (bit 3) in CR
	*pUSART1_CR = (1U << 0) | (1U << 3);
}

/*----------------------------------------------------------------------------
  Simple USART1 send functions
  ----------------------------------------------------------------------------*/
// Send a single character over USART1.
void usart1_send_char(char c)
{
	// Wait until TXE (Transmit Data Register Empty, bit 7) is set in USART1_ISR.
	while(((*pUSART1_ISR) & (1 << 7)) == 0);
	*pUSART1_TDR = c;
}

// Send a null-terminated string over USART1.
void usart1_send_string(const char *s)
{
	while(*s)
	{
		usart1_send_char(*s++);
	}
}

static inline uint32_t div10(uint32_t n) {
	return (uint32_t)(((uint64_t)n * 0xCCCCCCCDULL) >> 35);
}

static inline uint32_t mod10(uint32_t n) {
	uint32_t q = div10(n);
	return n - q * 10;
}

void usart1_send_uint32(uint32_t num)
{
	char buffer[11];  // up to 10 digits + null terminator
	int i = 0;
	if (num == 0) {
		usart1_send_char('0');
		return;
	}
	while (num) {
		buffer[i++] = (char)('0' +mod10(num));
		num = div10(num);
	}
	while (i--) {
		usart1_send_char(buffer[i]);
	}
}



/*----------------------------------------------------------------------------
  TSC initialisation
  ----------------------------------------------------------------------------*/

void TSC_init(void){
	//1. Initialise clocks
	*pRCC_AHBENR |= (1 << 24);
	*pRCC_AHBENR |= (1 << 18);

	/* 2) Configure GPIOs for TSC electrodes as per schematic:
	   - PA2 (LS_P1)
	   - PA3 (Capacitor electrode)
	   - PA6 (LS_P1)
	   - PA7 (Capacitor electrode)
	   - PB0 (LS_P1)
	   - PB1 (Capacitor electrode)
	   All pins: Alternate Function mode (0x2), no pull-up/down,
	   and assigned alternate function TSC_AF. */

	//PA2:
	//AF3 TSC_G1_IO3
	*pGPIOA_MODER &= ~(0x3U << 4);
	*pGPIOA_MODER |= (0x2U << 4);
	*pGPIOA_AFRL &= ~(0xFU << 8);
	*pGPIOA_AFRL |= (0x3U << 8);

	//PA3:
	*pGPIOA_MODER &= ~(0x3U << 6);
	*pGPIOA_MODER |= (0x2U << 6);
	*pGPIOA_AFRL &= ~(0xFU << 12);
	*pGPIOA_AFRL |= (0x3U << 12);

	//PA6:
	*pGPIOA_MODER &= ~(0x3U << 12);
	*pGPIOA_MODER |= (0x2U << 12);
	*pGPIOA_AFRL &= ~(0xFU << 24);
	*pGPIOA_AFRL |= (0x3U << 24);

	//PA7:
	*pGPIOA_MODER &= ~(0x3U << 14);
	*pGPIOA_MODER |= (0x2U << 14);
	*pGPIOA_AFRL &= ~(0xFU << 28);
	*pGPIOA_AFRL |= (0x3U << 28);

	//PB0:
	*pGPIOB_MODER &= ~(0x3U << 0);
	*pGPIOB_MODER |= (0x2U << 0);
	*pGPIOB_AFRL &= ~(0xFU << 0);
	*pGPIOB_AFRL |= (0x3U << 0);

	//PB1:
	*pGPIOB_MODER &= ~(0x3U << 2);
	*pGPIOB_MODER |= (0x2U << 2);
	*pGPIOB_AFRL &= ~(0xFU << 4);
	*pGPIOB_AFRL |= (0x3U << 4);

	/* 3) Configure the TSC peripheral registers as per RM0091 example:
	   (1) Configure TSC_CR:
	   - fPGCLK = fHCLK/32 (using PGPSC bits)
	   - Pulse high = 2 x tPGCLK (CTPH bits)
	   - Pulse low  = 2 x tPGCLK (CTPL bits)
	   - Max count value = 16383 pulses (MCV bits)
	   - Enable TSC (TSCE bit)
	 */

	*pTSC_CR = ((0x5U<<12) | (0x1U<<28) | (0x1U<<24) | (0x6U<<5) | (0x1U<<0));
	//2. Disable hysterisis
	*pTSC_IOHCR &= ~((0xCU<<0) | (0xCU<<6) | (0xCU<<10));
	//3. Enable sampling on PA3, PA7, PB1
	*pTSC_IOSCR |= ((0x1U<<3) | (0x1U<<7) | (0x1U<<10));
	//4. Enable sensing channels on PA2,PA6,PB0 
	*pTSC_IOCCR |= ((0x1U << 2) | (0x1U << 6) | (0x1U << 9));
	//5. Enable analog groups in TSC
	*pTSC_IOGCSR |= ((0x1U<<0) | (0x1U<<1) | (0x1U<<2));
	//6. Disable end of acquisition interrupt
	*pTSC_IER |= (1U << 0);

	//7. Enable Interrupt mode
	*pNVIC_ISER |= (1U << 8);



}

void led_init(void){

	// Initialize GPIOC LED pins: Set PC6, PC7, PC8, and PC9 as general-purpose output.
	*pRCC_AHBENR |= (0x1U << 19);

	*pGPIOC_MODER &= ~((0x3U << (6*2)) | (0x3U << (7*2)) | (0x3U << (8*2)) | (0x3U << (9*2)));
	*pGPIOC_MODER |=  ((0x1U << (6*2)) | (0x1U << (7*2)) | (0x1U << (8*2)) | (0x1U << (9*2)));

}

void update_leds(uint32_t g1, uint32_t g2, uint32_t g3)
{
	// Turn off all LEDs first
	*pGPIOC_ODR &= ~((0x1U << 6) | (0x1U << 7) | (0x1U << 8) | (0x1U << 9));

	// Assume no touch detected initially; light blue LED.
	uint8_t led_to_light = 7;  // PC7 for blue LED

	if (g1 < group1_threshold)
	{
		led_to_light = 6;  // red LED on PC6
	}
	if (g2 < group2_threshold)
	{
		led_to_light = 8;  // orange LED on PC8
	}
	if (g3 < group3_threshold)
	{
		led_to_light = 9;  // green LED on PC9
	}

	*pGPIOC_ODR |= (0x1U << led_to_light);

}



/*----------------------------------------------------------------------------
  Main Function
  ----------------------------------------------------------------------------*/
int main(void)
{
	// Initialization functions
	usart1_init();
	TSC_init();
	led_init();

	// Start the first TSC acquisition
	*pTSC_CR |= (1U << 1);

	while (1)
	{
		// Check if new sensor data is available
		if (new_data_available)
		{
			// Process the sensor data (update LEDs, debug output, etc.)
			update_leds(sensor_data[0], sensor_data[1], sensor_data[2]);
			usart1_send_string("G1: ");
			usart1_send_uint32(sensor_data[0]);
			usart1_send_string("  G2: ");
			usart1_send_uint32(sensor_data[1]);
			usart1_send_string("  G3: ");
			usart1_send_uint32(sensor_data[2]);
			usart1_send_string("\n");

			// Clear the flag before restarting acquisition
			new_data_available = 0;

			// Restart TSC acquisition for continuous sampling
			*pTSC_CR |= (1U << 1);
		}
	}

	return 0;
}


/*----------------------------------------------------------------------------
  TSC_IRQHandler for linear touch pad
  ----------------------------------------------------------------------------*/

void TSC_IRQHandler(void)
{
	// Check if EOAF interrupt is set
	if ((*pTSC_ISR) & TSC_ISR_EOAF)
	{
		// Clear EOAF flag
		*pTSC_ICR = TSC_ICR_EOAIC;

		// Quickly capture sensor data
		sensor_data[0] = *pTSC_IOG1CR;
		sensor_data[1] = *pTSC_IOG2CR;
		sensor_data[2] = *pTSC_IOG3CR;

		// Set flag to indicate new data is ready for processing
		new_data_available = 1;

	}
}



/*----------------------------------------------------------------------------
  Reset Handler: Called upon microcontroller reset.
  ----------------------------------------------------------------------------*/
//Startup code
__attribute__((naked, noreturn)) 
	void _reset(void){
		//long is quivalent to uint32_t
		extern long _sbss, _ebss, _sdata, _edata, _srcdata;

		for(long *dst = &_sbss; dst < &_ebss;dst++) *dst=0;
		for(long *dst= &_sdata, *src= &_srcdata;dst< &_edata;) *dst++=*src++;

		main();
		for(;;) (void) 0; //infinite loop incase main() returns

	}
/*----------------------------------------------------------------------------
  Default Handler: Catches unused interrupts.
  ----------------------------------------------------------------------------*/
void Default_Handler(void)
{
	while(1);
}

/*----------------------------------------------------------------------------
  Minimal Vector Table placed in .isr_vector section
  ----------------------------------------------------------------------------*/
// Minimal vector table as per Cortex‑M0 generic user guide (6 system exceptions + 32 IRQs)
__attribute__((section(".vectors")))
void (*const vector_table[7+32])(void) = {
	(void (*)(void))&_estack,  // Use the address of _estack
	_reset,
	NMI_Handler,
	HardFault_Handler,
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler, 
	Default_Handler,
	Default_Handler, 
	Default_Handler,  
	Default_Handler, 
	Default_Handler, 
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler, 
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	
	TSC_IRQHandler,  
	
	Default_Handler,  
	Default_Handler,  
	Default_Handler, 
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler,  
	Default_Handler   
};

