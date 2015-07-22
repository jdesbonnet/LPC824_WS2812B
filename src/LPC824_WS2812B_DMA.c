/*
===============================================================================
 Name        : LPC824_ADC_DMA_example.c
 Author      : Joe Desbonnet, jdesbonnet@gmail.com
 Version     : 1.0 (18 July 2015)
 Copyright   : None.
 Description : Example of driving a WS2812B RGB LED module using the SCT and DMA.

 This example is built upon examples provided by NXP for the LPC824. I recommend having
 chapters 21 (ADC), 11,12 (DMA), 16 (SCT) of UM10800 LPC82x User Manual for reference.

 There is a dependency on the lpc_chip_82x project/library which can be downloaded from:
 https://www.lpcware.com/content/nxpfile/lpcopen-software-development-platform-lpc8xx-packages

===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

//
// Hardware configuration
//

#define UART_BAUD_RATE 9600

// Define function to pin mapping. Pin numbers here refer to PIO0_n
// and is not the same as a package pin number.
// Use PIO0_0 and PIO0_4 for UART RXD, TXD (same as ISP)
#define PIN_UART_RXD 0
#define PIN_UART_TXD 4
// General purpose debug pin
#define PIN_DEBUG 15
#define PIN_WS2812B 14

#define DMA_BUFFER_SIZE 4

// Ping-pong DMA buffers
static DMA_CHDESC_T dmaDescA;
static DMA_CHDESC_T dmaDescB;

static uint32_t dma_buffer[DMA_BUFFER_SIZE] = {0x100,0x200,0x300,0x500};

/**
 * @brief Pulse debugging pin to indicate an event on a oscilloscope trace.
 * @param n Number of times to pulse the pin.
 * @return None
 */
static void debug_pin_pulse (int n)
{
	int i;
	for (i = 0; i < n; i++) {
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_DEBUG, true);
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_DEBUG, false);
	}
}

static volatile bool dmaDone;
/**
 * @brief	DMA Interrupt Handler
 * @return	None
 */
void DMA_IRQHandler(void)
{
	// Pulse debug pin so can see when each DMA block ends on scope trace.
	debug_pin_pulse (8);

	// Clear DMA interrupt for the channel
	Chip_DMA_ClearActiveIntAChannel(LPC_DMA, DMA_CH0);

	// Increment the DMA counter. When 3 the main loop knows we're done.
	//dmaBlockCount++;

	dmaDone = true;
}

static int stopColors[] = {0x0000ff, 0x00ffff, 0x00ff00, 0xffff00,0xff0000};


/**
 * @param color 0x00bbrrgg
 *
 */
void ws2812b_bitbang (uint32_t color) {
	// Color


	int i,j;

	uint32_t c;

	while (1) {

		for (j = 0; j < 16; j++) {
			//color = 0x00ff00ff;
			c = color;

			for (i = 0; i < 24; i++) {
				Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_WS2812B, 1);
				if (c & 1) {
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_WS2812B, 0);
				} else {
					__NOP();
					__NOP();
					Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_WS2812B, 0);
					__NOP();
					__NOP();
				}
				c >>= 1;
			}

			//color++;
		}

		// RESET

		for (j = 0; j < 192; j++) {
			__NOP();
		}

	}

}

/**
 *
 */
void dma_sct_start() {

	//
	// Setup SCT
	//

	Chip_SCT_Init(LPC_SCT);

	/* Stop the SCT before configuration */
	Chip_SCTPWM_Stop(LPC_SCT);

	// Match/capture mode register. (ref UM10800 section 16.6.11, Table 232, page 273)
	// Determines if match/capture operate as match or capture. Want all match.
	LPC_SCT->REGMODE_U = 0;

	// Event 0 control: (ref UM10800 section 16.6.25, Table 247, page 282).
	// set MATCHSEL (bits 3:0) = MATCH0 register(0)
	// set COMBMODE (bits 13:12)= MATCH only(1)
	// So Event0 is triggered on match of MATCH0
	LPC_SCT->EV[0].CTRL = (0 << 0) | 1 << 12;
	// Event enable register (ref UM10800 section 16.6.24, Table 246, page 281)
	// Enable Event0 in State0 (default state). We are not using states,
	// so this enables Event0 in the default State0.
	// Set STATEMSK0=1
	LPC_SCT->EV[0].STATE = 1 << 0;

	// Configure Event2 to be triggered on Match2
	LPC_SCT->EV[2].CTRL = (2 << 0) // The match register (MATCH2) associated with this event
	| (1 << 12); // COMBMODE=1 (MATCH only)
	LPC_SCT->EV[2].STATE = 1; // Enable Event2 in State0 (default state)

	/* Clear the output in-case of conflict */
	int pin = 0;
	LPC_SCT->RES = (LPC_SCT->RES & ~(3 << (pin << 1))) | (0x01 << (pin << 1));

	/* Set and Clear do not depend on direction */
	LPC_SCT->OUTPUTDIRCTRL = (LPC_SCT->OUTPUTDIRCTRL
			& ~((3 << (pin << 1)) | SCT_OUTPUTDIRCTRL_RESERVED));

	// Set SCT Counter to count 32-bits and reset to 0 after reaching MATCH0
	Chip_SCT_Config(LPC_SCT, SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L);

	// Setup SCT for ADC/DMA sample timing.
	uint32_t clock_hz = Chip_Clock_GetSystemClockRate();
	Chip_SCT_SetMatchCount(LPC_SCT, SCT_MATCH_2, 10);
	Chip_SCT_SetMatchCount(LPC_SCT, SCT_MATCH_0, 20);
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_2, 10);
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, 20);

	// Using SCT0_OUT3 to trigger ADC sampling
	// Set SCT0_OUT3 on Event0 (Event0 configured to occur on Match0)
	LPC_SCT->OUT[3].SET = 1 << 0;
	// Clear SCT0_OUT3 on Event2 (Event2 configured to occur on Match2)
	LPC_SCT->OUT[3].CLR = 1 << 2;

	// SwitchMatrix: Assign SCT_OUT3 to external pin for debugging
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_MovablePinAssign(SWM_SCT_OUT3_O, PIN_WS2812B);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	// Start SCT
	Chip_SCT_ClearControl(LPC_SCT, SCT_CTRL_HALT_L | SCT_CTRL_HALT_H);

	// Done with ADC sampling, stop and switch off SCT, ADC
	//Chip_SCT_DeInit(LPC_SCT);
	//NVIC_DisableIRQ(DMA_IRQn);

	/* DMA initialization - enable DMA clocking and reset DMA if needed */
	Chip_DMA_Init(LPC_DMA);
	/* Enable DMA controller and use driver provided DMA table for current descriptors */
	Chip_DMA_Enable(LPC_DMA);
	Chip_DMA_SetSRAMBase(LPC_DMA, DMA_ADDR(Chip_DMA_Table));

	/* Setup channel 0 for the following configuration:
	 - High channel priority
	 - Interrupt A fires on descriptor completion */
	Chip_DMA_EnableChannel(LPC_DMA, DMA_CH0);
	Chip_DMA_EnableIntChannel(LPC_DMA, DMA_CH0);
	Chip_DMA_SetupChannelConfig(LPC_DMA, DMA_CH0,
			(DMA_CFG_HWTRIGEN
					//| DMA_CFG_PERIPHREQEN  //?? what's this for???
					| DMA_CFG_TRIGTYPE_EDGE | DMA_CFG_TRIGPOL_HIGH
					| DMA_CFG_TRIGBURST_BURST | DMA_CFG_BURSTPOWER_1
					| DMA_CFG_CHPRIORITY(0)));

	// Use SCT to trigger DMA xfer
	Chip_DMATRIGMUX_SetInputTrig(LPC_DMATRIGMUX, DMA_CH0, DMATRIG_SCT0_DMA0);

	// note that addresses must
	// be the END address for source and destination, not the starting address.
	// DMA operations moves from end to start. [Ref ].
	dmaDescB.xfercfg = (
	DMA_XFERCFG_CFGVALID  // Channel descriptor is considered valid
	| DMA_XFERCFG_SETINTA // DMA Interrupt A (A vs B can be read in ISR)
			| DMA_XFERCFG_WIDTH_8 // 8,16,32 bits allowed
			| DMA_XFERCFG_SRCINC_1 // increment src by widthx1
			| DMA_XFERCFG_DSTINC_0 // do not increment dst
			| DMA_XFERCFG_XFERCOUNT(DMA_BUFFER_SIZE));
	dmaDescB.source = DMA_ADDR(&dma_buffer[DMA_BUFFER_SIZE]-1);
	dmaDescB.dest = DMA_ADDR(&LPC_SCT->MATCHREL[1].U);
	dmaDescB.next = DMA_ADDR(0); // no more descriptors

	// ADC data register is source of DMA
	dmaDescA.source = DMA_ADDR(&dma_buffer[DMA_BUFFER_SIZE]-1);
	dmaDescA.dest = DMA_ADDR(&LPC_SCT->MATCHREL[1].U);
	//dmaDescA.next = (uint32_t)&dmaDescB;
	dmaDescA.next = DMA_ADDR(0);

	// Enable DMA interrupt. Will be invoked at end of DMA transfer.
	NVIC_EnableIRQ(DMA_IRQn);

	/* Setup transfer descriptor and validate it */
	Chip_DMA_SetupTranChannel(LPC_DMA, DMA_CH0, &dmaDescA);
	Chip_DMA_SetValidChannel(LPC_DMA, DMA_CH0);

	// Setup data transfer and hardware trigger
	// See "Transfer Configuration registers" UM10800, §12.6.18, Table 173, page 179
	Chip_DMA_SetupChannelTransfer(LPC_DMA, DMA_CH0, (
	DMA_XFERCFG_CFGVALID  // Channel descriptor is considered valid
	//| DMA_XFERCFG_RELOAD  // Causes DMA to move to next descriptor when complete
			| DMA_XFERCFG_SETINTA // DMA Interrupt A (A vs B can be read in ISR)
			//| DMA_XFERCFG_SWTRIG  // When written by software, the trigger for this channel is set immediately.
			| DMA_XFERCFG_WIDTH_32 // 8,16,32 bits allowed
			| DMA_XFERCFG_SRCINC_1 // increment src by 1 x width
			| DMA_XFERCFG_DSTINC_0 // do not increment dst
			| DMA_XFERCFG_XFERCOUNT(DMA_BUFFER_SIZE)));

}


#define BUFFER_SIZE 24

// 0b11000000
#define T0 0xc0
// 0b11111000
#define T1 0xf8

/* Tx buffer */
static uint8_t TxBuf0[BUFFER_SIZE] = {
		T0,T0,T0,T0, T0,T0,T0,T0, // G
		T1,T1,T1,T1, T0,T0,T0,T0, // R
		T0,T0,T0,T0, T0,T0,T0,T0  // B
};
static uint8_t TxBuf1[BUFFER_SIZE] = {
		T1,T1,T1,T1, T0,T0,T0,T0, // G
		T1,T1,T1,T1, T0,T0,T0,T0, // R
		T0,T0,T0,T0, T0,T0,T0,T0  // B
};
static uint8_t TxBuf2[BUFFER_SIZE] = {
		T1,T1,T1,T1, T0,T0,T0,T0, // G
		T0,T0,T0,T0, T0,T0,T0,T0, // R
		T0,T0,T0,T0, T0,T0,T0,T0  // B
};
static uint8_t TxBuf3[BUFFER_SIZE] = {
		T0,T0,T0,T0, T0,T0,T0,T0, // G
		T0,T0,T0,T0, T0,T0,T0,T0, // R
		T1,T1,T1,T1, T1,T1,T1,T1  // B
};
/* Rx buffer */
//static uint16_t RxBuf[BUFFER_SIZE];

void set_rgb (uint8_t *buf, uint32_t rgb) {
	int i;
	for (i = 16; i < 24; i++) {
		buf[i] = rgb&1 ? T1 : T0;
		rgb >>= 1;
	}
	for (i = 0; i < 8; i++) {
		buf[i] = rgb&1 ? T1 : T0;
		rgb >>= 1;
	}
	for (i = 8; i < 16; i++) {
		buf[i] = rgb&1 ? T1 : T0;
		rgb >>= 1;
	}
}


/**
 * Write to WS2812B using SPI MOSI
 */
void dma_spi_start () {

	//
	// Setup SPI
	//
	Chip_SPI_Init(LPC_SPI0);
	Chip_SPI_ConfigureSPI(LPC_SPI0,
			SPI_MODE_MASTER
						 //SPI_MODE_TEST |	/* Enable master/Slave mode */
						  | SPI_CLOCK_CPHA0_CPOL0 	/* Set Clock polarity to 0 */
						  | SPI_CFG_MSB_FIRST_EN /* Enable MSB first option */
						  | SPI_CFG_SPOL_LO);	/* Chipselect is active low */


	LPC_SPI0->DIV = 3;

	// Assign MOSI pin. Other SPI lines not of interest in this application.
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_MovablePinAssign(SWM_SPI0_MOSI_IO, PIN_WS2812B);
	Chip_SWM_MovablePinAssign(SWM_SPI0_SCK_IO, PIN_DEBUG);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	Chip_SPI_EnableMasterMode(LPC_SPI0);
	Chip_SPI_Enable(LPC_SPI0);

	int i,j,k;
	for (i =0; i < 100000000; i++) {


		/* Clear status */
		Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_CLR_RXOV
				| SPI_STAT_CLR_TXUR
				| SPI_STAT_CLR_SSA
				| SPI_STAT_CLR_SSD);
		Chip_SPI_SetControlInfo(LPC_SPI0, 8,
				SPI_TXCTL_ASSERT_SSEL
				| SPI_TXCTL_EOF
				| SPI_TXCTL_RXIGNORE);

		//set_rgb(TxBuf, 0xff0000);


		// RESET
		for (j = 0; j < 100; j++) {
			while (!(LPC_SPI0->STAT&SPI_STAT_TXRDY)) {}
			LPC_SPI0->TXDAT = 0;
		}

		for (k = 0; k < 4; k++) {
			for (j = 0; j < 24; j++) {
				while (!(LPC_SPI0->STAT&SPI_STAT_TXRDY)) {}
				LPC_SPI0->TXDAT = TxBuf0[j];
			}
		}

		for (k = 0; k < 4; k++) {
			for (j = 0; j < 24; j++) {
				while (!(LPC_SPI0->STAT&SPI_STAT_TXRDY)) {}
				LPC_SPI0->TXDAT = TxBuf1[j];
			}
		}

		for (k = 0; k < 4; k++) {
			for (j = 0; j < 24; j++) {
				while (!(LPC_SPI0->STAT&SPI_STAT_TXRDY)) {}
				LPC_SPI0->TXDAT = TxBuf2[j];
			}
		}

		for (k = 0; k < 4; k++) {
			for (j = 0; j < 24; j++) {
				while (!(LPC_SPI0->STAT&SPI_STAT_TXRDY)) {}
				LPC_SPI0->TXDAT = TxBuf3[j];
			}
		}

		Chip_SPI_SendLastFrame_RxIgnore(LPC_SPI0,0,8);


		/* Make sure the last frame sent completely*/
		//while (!(Chip_SPI_GetStatus(LPC_SPI0) & SPI_STAT_SSD)) {}
		Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_CLR_SSD);



	}



		/* DMA initialization - enable DMA clocking and reset DMA if needed */
		Chip_DMA_Init(LPC_DMA);
		/* Enable DMA controller and use driver provided DMA table for current descriptors */
		Chip_DMA_Enable(LPC_DMA);
		Chip_DMA_SetSRAMBase(LPC_DMA, DMA_ADDR(Chip_DMA_Table));

		/* Setup channel 0 for the following configuration:
		   - High channel priority
		   - Interrupt A fires on descriptor completion */
		Chip_DMA_EnableChannel(LPC_DMA, DMA_CH0);
		Chip_DMA_EnableIntChannel(LPC_DMA, DMA_CH0);
		Chip_DMA_SetupChannelConfig(LPC_DMA, DMA_CH0,
				(
						//DMA_CFG_HWTRIGEN
						DMA_CFG_PERIPHREQEN  //?? what's this for???
						//| DMA_CFG_TRIGTYPE_EDGE
						//| DMA_CFG_TRIGPOL_HIGH
						//| DMA_CFG_TRIGBURST_BURST
						| DMA_CFG_TRIGBURST_SNGL
						//| DMA_CFG_BURSTPOWER_1
						 | DMA_CFG_CHPRIORITY(0)
						 ));

		// Use SCT to trigger DMA xfer
		//Chip_DMATRIGMUX_SetInputTrig(LPC_DMATRIGMUX, DMA_CH0, DMATRIG_SCT0_DMA0);

		// note that addresses must
		// be the END address for source and destination, not the starting address.
		// DMA operations moves from end to start. [Ref ].

		dmaDescB.xfercfg = DMA_XFERCFG_CFGVALID
											| DMA_XFERCFG_SETINTA
											| DMA_XFERCFG_SWTRIG
											 | DMA_XFERCFG_WIDTH_8
											 | DMA_XFERCFG_SRCINC_1
											 | DMA_XFERCFG_DSTINC_0
											 | DMA_XFERCFG_XFERCOUNT(DMA_BUFFER_SIZE);
		dmaDescB.source = DMA_ADDR(&dma_buffer[DMA_BUFFER_SIZE]-1) ;
		dmaDescB.dest = DMA_ADDR ( &LPC_SPI0->TXDAT );
		dmaDescB.next = DMA_ADDR(0); // no more descriptors


		// ADC data register is source of DMA
		dmaDescA.source = DMA_ADDR(&dma_buffer[DMA_BUFFER_SIZE]-1) ;
		dmaDescA.dest =  DMA_ADDR ( &LPC_SCT->MATCHREL[1].U );
		//dmaDescA.next = (uint32_t)&dmaDescB;
		dmaDescA.next = DMA_ADDR(0);



		// Enable DMA interrupt. Will be invoked at end of DMA transfer.
		NVIC_EnableIRQ(DMA_IRQn);

		/* Setup transfer descriptor and validate it */
		Chip_DMA_SetupTranChannel(LPC_DMA, DMA_CH0, &dmaDescA);
		Chip_DMA_SetValidChannel(LPC_DMA, DMA_CH0);

		// Setup data transfer and hardware trigger
		// See "Transfer Configuration registers" UM10800, §12.6.18, Table 173, page 179
		Chip_DMA_SetupChannelTransfer(LPC_DMA, DMA_CH0,
				 (
					DMA_XFERCFG_CFGVALID  // Channel descriptor is considered valid
					//| DMA_XFERCFG_RELOAD  // Causes DMA to move to next descriptor when complete
					| DMA_XFERCFG_SETINTA // DMA Interrupt A (A vs B can be read in ISR)
					//| DMA_XFERCFG_SWTRIG  // When written by software, the trigger for this channel is set immediately.
					| DMA_XFERCFG_WIDTH_8 // 8,16,32 bits allowed
					| DMA_XFERCFG_SRCINC_1 // increment src by 1 x width
					| DMA_XFERCFG_DSTINC_0 // do not increment dst
					| DMA_XFERCFG_XFERCOUNT(DMA_BUFFER_SIZE)
					)
					);

}


int main(void) {

	//
	// Initialize GPIO
	//
	Chip_GPIO_Init(LPC_GPIO_PORT);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, PIN_DEBUG);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, PIN_WS2812B);


	//
	// Initialize UART
	//

	// Assign pins: use same assignment as serial bootloader
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, PIN_UART_TXD);
	Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, PIN_UART_RXD);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	Chip_UART_Init(LPC_USART0);
	Chip_UART_ConfigData(LPC_USART0,
			UART_CFG_DATALEN_8
			| UART_CFG_PARITY_NONE
			| UART_CFG_STOPLEN_1);

	Chip_Clock_SetUSARTNBaseClockRate((UART_BAUD_RATE * 16), true);
	Chip_UART_SetBaud(LPC_USART0, UART_BAUD_RATE);
	Chip_UART_TXEnable(LPC_USART0);
	Chip_UART_Enable(LPC_USART0);







	// Done. Sleep forever.
	while (1) {

		dmaDone = false;
		debug_pin_pulse(8);
		dma_spi_start();
		while (!dmaDone) {
			__WFI();
		}
	}

}
