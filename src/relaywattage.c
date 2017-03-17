/* High power AC relays + wattage measurement module based on STM32F373CBT6, 4 channels version
 *
 *    Copyright (C) 2017  Vasim V.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with this program. */

#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <inttypes.h>
#include <relaywattage.h>

// Data calculated from ADC readings (in millivolts/millidegrees)
// Supply voltage
volatile int32_t Vdd = 0;
// Voltage of VDDA (used as Vref)
volatile int32_t Vref = 0;
// Chip's temperature
volatile int32_t Tchip = 0;

// Data calculated from SDADC readings
// Voltage on AC line (volts * 10)
volatile float AcVoltage = 0.0;
// Board temperature (millidegrees)
volatile int32_t Tboard = 0;

// Set by TIM7 interrupt routine (output data to serial interface)
volatile uint8_t FlagReport = 0;

// ADC average values
uint16_t AdcData[MY_ADC_CHANNELS];

// ADC DMA buffer
volatile uint16_t AdcBuf[MAX_ADC_COUNTS][MY_ADC_CHANNELS];
volatile uint16_t CntAdc = 0;
volatile uint16_t MaxCntAdc = 0;

// SDADC values
volatile uint16_t SdAdcData[MY_SDADC_CHANNELS];

// Power consumption measured
volatile uint64_t AcConsumed[MY_AC_CHANNELS];

// AC channels state
volatile uint8_t AcState[MY_AC_CHANNELS];

// Zero current offset (averaged)
volatile uint16_t AcZero[MY_AC_CHANNELS];

// Zero current offset (summarized before averaged)
uint32_t AcZeroSum[MY_AC_CHANNELS];

// Number of counts in AcZeroSum
uint16_t CntAcZero[MY_AC_CHANNELS];

// Sum of AC voltage measures
volatile uint64_t AcVoltageSum = 0;

// Measures counter (between reports)
volatile uint32_t Counts = 0;

// UART Receive buffer (just one character actually)
char RecvBuf[1];

// Command to process
volatile char CmdNext[MAX_COMMAND_LENGTH];
// We have a command in the buffer, waiting to process it
volatile uint8_t FlagProcessCmd = 0;

// Buffer for commands waiting for processing
char CmdBuf[256];
int CmdBufLength = 0;

// Serial output buffer
char seroutbuf[256];

#ifdef DEBUG_COLLECT
uint16_t AcCollect[DEBUG_COLLECT];
uint32_t CurrCollect[DEBUG_COLLECT];
uint16_t NCollected = 0;
uint16_t MinCollected = 0xffff;
uint16_t MaxCollected = 0;
uint32_t AcPower = 0;
#endif

va_list args;
int nlen;
volatile uint8_t txDoneFlag = 1;
volatile uint8_t FlushPrevCmd = 0;

#ifdef SERIAL_DEBUG
#define debug SerialPrint
#else
#ifdef USB_DEBUG
#define debug UsbPrint
#else
void debug(char *fmt, ...) {
}
#endif
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    txDoneFlag = 1;
} // void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)

void SerialPrint(char *fmt, ...) {
    va_start(args, fmt);
    nlen = vsnprintf(seroutbuf, sizeof(seroutbuf), fmt, args);
    va_end(args);
    while (!txDoneFlag);
    txDoneFlag = 0;
    while ((HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) && (HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_RX));
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *) seroutbuf, nlen);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
    while ((HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) && (HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_RX));
} // void SerialPrint(char *fmt, ...)

// UART receiver callback, puts received char in CmdBuf (and to CmdNext if available)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	char *pNext;
	int len;

#ifdef SERIAL_DEBUG
	// Echo commands when debugging
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *) RecvBuf, 1);
#endif
	// Remove previous command from the buffer
	if (FlushPrevCmd) {
		pNext = memchr(CmdBuf, '\n', CmdBufLength);
		if (pNext) {
			len = pNext - CmdBuf + 1;
			if ((CmdBufLength > len) && ((*(pNext + 1) == '\r') || (*(pNext + 1) == '\n') || (*(pNext + 1) == '\0'))) {
				pNext++;
				len++;
			}
			if (len < CmdBufLength)
				memcpy(CmdBuf, pNext + 1, CmdBufLength - len);
			CmdBufLength = CmdBufLength - len;
			if (CmdBufLength < 0)
				CmdBufLength = 0;
		} else {
			// Shouldn't happen, resetting buffer
			CmdBufLength = 0;
		}
		FlushPrevCmd = 0;
	}
	// Check if we have enough space in buffer for new chars
	if (CmdBufLength < sizeof(CmdBuf)) {
		CmdBuf[CmdBufLength] = *RecvBuf;
		CmdBufLength++;
	} else {
		// Reset incoming buffer if no EOL found and full
		if (!memchr(CmdBuf, '\n', CmdBufLength))
			CmdBufLength = 0;
	}
} // void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

void CheckNextCmd() {
	char *pNext;

	// Check if we can process and have new commands waiting in CmdBuf
	if (FlagProcessCmd || (CmdBufLength <= 0))
		return;
	// Find EOL and process the command
	pNext = memchr(CmdBuf, '\n', CmdBufLength);
	if (pNext) {
		// Found EOL char in the buffer, moving command to CmdNext processing buffer
		int len = pNext - CmdBuf + 1;

		*pNext = '\0';
		if (len > (sizeof(CmdNext) - 1))
			len = sizeof(CmdNext) - 1;
		memcpy(CmdNext, CmdBuf, len);
		CmdNext[len] = '\0';
		FlagProcessCmd = 1;
		FlushPrevCmd = 1;
	}
} // void CheckNextCmd()

#ifdef USB_DEBUG
char usboutbuf[256];

va_list args;
int nlen;

int VCP_write(const void *pBuffer, int size) {
    if (size > CDC_DATA_HS_OUT_PACKET_SIZE)
    {
        int offset;
        for (offset = 0; offset < size; offset++)
        {
            int todo = MIN(CDC_DATA_HS_OUT_PACKET_SIZE,
                           size - offset);
            int done = VCP_write(((char *)pBuffer) + offset, todo);
            if (done != todo)
                return offset + done;
        }

        return size;
    }

    USBD_CDC_HandleTypeDef *pCDC =
            (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    // XXX - set timeout?
    while(pCDC->TxState) {  HAL_Delay(10); } //Wait for previous transfer

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)pBuffer, size);
    if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK)
        return 0;
    return size;
} // int VCP_write(const void *pBuffer, int size)

void UsbPrint(char *fmt, ...) {
	   // Check if we're really connected
	   if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
		   return;
	   va_start(args, fmt);
	   nlen = vsnprintf(usboutbuf, sizeof(usboutbuf), fmt, args);
	   va_end(args);
	   // CDC_Transmit_FS((uint8_t *) usboutbuf, nlen);
	   VCP_write(usboutbuf, nlen);
} // void UsbPrint(char *fmt, ...)
#endif

// Start 12-bit ADC (11 ADC channels, temperature, vrefint, vbat)
void StartAdc() {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AdcBuf[CntAdc], MY_ADC_CHANNELS);
} // void StartAdc()

// Stop ADC conversion
void StopAdc() {
	HAL_ADC_Stop_DMA(&hadc1);
} // void StopAdc()

// Sum'ming new ADC readings for averaging
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	CntAdc++;
	if (CntAdc >= MAX_ADC_COUNTS)
		CntAdc = 0;

	StartAdc();
} // void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)

// Calculate ADC stuff using internal Vrefint and calibration values
void CalculateAdc() {
	uint16_t i, j;
	uint32_t tCnt;
	uint32_t tSum;

	// debug("CntAdc: %d\n", CntAdc);
	// Average values from ADC readings
	tCnt = CntAdc;
	StopAdc();
	if (CntAdc > MaxCntAdc)
		MaxCntAdc = CntAdc;

	for (i = 0; i < MY_ADC_CHANNELS; i++) {
		tSum = 0;
		for (j = 0; j < tCnt; j++)
			tSum += (uint32_t) AdcBuf[j][i];
		AdcData[i] = (uint16_t) (tSum / tCnt);
	}

	CntAdc = 0;
	StartAdc();

	// Calculate voltages based on calibration values and internal Vref
	Vref = (VREF_CORRECTION * VREFINT_CAL) / AdcData[9];
	Vdd = (Vref * AdcData[10]) / 2048;
	// Calculate temperature based on calibration values
	// Tchip = (AdcData[8] * VREFINT_CAL) / AdcData[8] - TS_CAL1;
    // Tchip =  Tchip * ((110000 - 30000) / (TS_CAL2 - TS_CAL1)) + 30000;
	Tchip = (((int32_t) AdcData[8]) *  Vdd / (int32_t) 3300) - (int32_t) TS_CAL1;
	Tchip *= (int32_t)(110000 - 30000);
	Tchip = Tchip / (int32_t)( TS_CAL2 - TS_CAL1);
	Tchip += 30000;
} // void Calculate_ADC()

// Start SDADC (2 channels, AC voltage, board temperature)
void StartSdAdc() {
	HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_3);
	HAL_SDADC_PollForCalibEvent(&hsdadc1, HAL_MAX_DELAY);
	HAL_SDADC_CalibrationStart(&hsdadc2, SDADC_CALIBRATION_SEQ_3);
	HAL_SDADC_PollForCalibEvent(&hsdadc2, HAL_MAX_DELAY);
	HAL_Delay(50);
//	HAL_SDADC_InjectedStart_DMA(&hsdadc1, SdAdcData, 1);
//	HAL_SDADC_Start_DMA(&hsdadc1, &SdAdcData, 1);
//	HAL_SDADC_InjectedStart_DMA(&hsdadc2, SdAdcData + sizeof(SdAdcData[0]), 1);
//	HAL_SDADC_Start_DMA(&hsdadc2, &SdAdcData + sizeof(SdAdcData[0]), 1);
} // void StartSdAdc()

// Start timers
void StartTimers() {
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
} // void StartTimers()

// Start debug serial interface
void StartSerial() {
#ifdef SERIAL_DEBUG
	debug("Serial debug!\n");
#endif
	HAL_UART_Receive_DMA(&huart1, (uint8_t *) RecvBuf, sizeof(RecvBuf));
} // void StartSerial()

// Change alarm pin state
void TurnAlarm(uint8_t flag) {
	HAL_GPIO_WritePin(ALARM_PORT, ALARM_PIN, flag ? GPIO_PIN_SET : GPIO_PIN_RESET);
} // void TurnAlarm(uint8_t flag)

// Switch bi-stable relays state (with 250ms+50ms delay!) on the selected channel
void TurnChn(uint8_t channel, uint8_t flag) {
	// Set appropriate pins state
	switch (channel) {
	case 0:
		HAL_GPIO_WritePin(CHN0_PINA_PORT, CHN0_PINA_PIN, flag ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CHN0_PINB_PORT, CHN0_PINB_PIN, flag ? GPIO_PIN_RESET : GPIO_PIN_SET);
		break;
	case 1:
		HAL_GPIO_WritePin(CHN1_PINA_PORT, CHN1_PINA_PIN, flag ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CHN1_PINB_PORT, CHN1_PINB_PIN, flag ? GPIO_PIN_RESET : GPIO_PIN_SET);
		break;
	case 2:
		HAL_GPIO_WritePin(CHN2_PINA_PORT, CHN2_PINA_PIN, flag ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CHN2_PINB_PORT, CHN2_PINB_PIN, flag ? GPIO_PIN_RESET : GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(CHN3_PINA_PORT, CHN3_PINA_PIN, flag ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CHN3_PINB_PORT, CHN3_PINB_PIN, flag ? GPIO_PIN_RESET : GPIO_PIN_SET);
		break;
	default:
		return;
	}
	// Make delay for relays to switch
	HAL_Delay(250);
	// Reset pins state to remove voltage from relays
	switch (channel) {
	case 0:
		HAL_GPIO_WritePin(CHN0_PINA_PORT, CHN0_PINA_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CHN0_PINB_PORT, CHN0_PINB_PIN, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(CHN1_PINA_PORT, CHN1_PINA_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CHN1_PINB_PORT, CHN1_PINB_PIN, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(CHN2_PINA_PORT, CHN2_PINA_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CHN2_PINB_PORT, CHN2_PINB_PIN, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(CHN3_PINA_PORT, CHN3_PINA_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CHN3_PINB_PORT, CHN3_PINB_PIN, GPIO_PIN_RESET);
		break;
	default:
		return;
	}
	HAL_Delay(50);
	// Memorize current state in the array
	AcState[channel] = flag;
} // void TurnChn(uint8_t channel, uint8_t flag)

uint32_t LastChnUpdate[MY_AC_CHANNELS];

// Returns number of channel in Adc* array
int ChnNum(int channel) {
	switch(channel) {
	case 0:
		return CHN0_ADC_NUM;
	case 1:
		return CHN1_ADC_NUM;
	case 2:
		return CHN2_ADC_NUM;
	case 3:
		return CHN3_ADC_NUM;
	default:
		break;
	}
	return 0;
} // int ChnNum(int channel)

// Update zero current offsets on all inactive channels
void UpdateZeroCurrent() {
	int i;
	uint16_t tAdc;

	for (i = 0; i < MY_AC_CHANNELS; i++) {
		// Check if channel state was inactive and use it for AcZero
		tAdc = AdcData[ChnNum(i)];
		if ((AcState[i] == 0) && (tAdc > MIN_ADC_ZERO) && (tAdc < MAX_ADC_ZERO)) {
			AcZeroSum[i] += (uint32_t) tAdc;
			// If AcZero is totally empty - fill it with first value
			if (AcZero[i] == 0)
				AcZero[i] = tAdc;
			CntAcZero[i]++;
			// Average multiple counts to use it as zero current
			if (CntAcZero[i] > 2000) {
				LastChnUpdate[i] = HAL_GetTick();
				AcZero[i] = (uint16_t) (AcZeroSum[i] / (uint32_t) CntAcZero[i]);
				AcZeroSum[i] = 0;
				CntAcZero[i] = 0;
			}
		}
	}
} // void UpdateZeroCurrent()

// XXX - Read and set channels states from the FLASH area
void GetStatesFlash() {

} // void GetStateFlash()

// Setup things before run
void Setup() {
#ifdef DEBUG
//	DBGMCU->CR = 0x7;
#endif
	memset((void *) AcConsumed, 0, sizeof(AcConsumed));
	memset((void *) AdcData, 0, sizeof(AdcData));
	memset((void *) CntAcZero, 0, sizeof(CntAcZero));
	TurnAlarm(0);
	TurnChn(0, 0);
	TurnChn(1, 0);
	TurnChn(2, 0);
	TurnChn(3, 0);
	StartSerial();
	StartSdAdc();
	StartTimers();
	StartAdc();
	// Make delay to stabilize voltages
	HAL_Delay(500);
	memset((void *) AcConsumed, 0, sizeof(AcConsumed));
	memset((void *) LastChnUpdate, 0, sizeof(LastChnUpdate));
} // void Setup()

// Process command
// Format: <Nchannel>:<Value>\n
void ProcessCmd(char *Cmd) {
	char *pSplit = memchr(Cmd, ':', MAX_COMMAND_LENGTH);
	int Chn, Val;

	// Wrong format, doing nothing
	if (!pSplit)
		return;
#ifdef DEBUG
	SerialPrint("Cmd: %s\n", Cmd);
#endif
	Chn = atoi(CmdNext);
	Val = atoi(pSplit + 1);
	switch (Chn) {
	case 0:
	case 1:
	case 2:
	case 3:
		TurnChn(Chn, Val ? 1 : 0);
		break;
	case 100:
		TurnAlarm(Val ? 1 : 0);
		break;
	default:
		break;
	}
} // void ProcessCmd(char *Cmd)

// Calculate current on all channels and update consumed power values
void measure_int() {
	int i;
	uint32_t chn;

	CalculateAdc();

	// Measure AC voltage and the board temperature
	HAL_SDADC_InjectedStart(&hsdadc2);
	HAL_SDADC_InjectedStart(&hsdadc1);

	while (HAL_SDADC_PollForInjectedConversion(&hsdadc1, 0) == HAL_TIMEOUT);

	// Calculate AC voltage and temperature
	SdAdcData[0] = 32768 + (int16_t) HAL_SDADC_InjectedGetValue(&hsdadc1, &chn);
	SdAdcData[1] = (32768 + (int16_t) HAL_SDADC_InjectedGetValue(&hsdadc2, &chn)) / AC_DIVIDER;

	// Sum AC RMS voltage to calculate real voltage later
	AcVoltageSum += (uint64_t) SdAdcData[1];

	// Check all channels and calculate consumed power
	for (i = 0; i < MY_AC_CHANNELS; i++)
		// Check active channels only
		if (AcState[i]) {
			AcConsumed[i] += (uint64_t) SdAdcData[1] * (uint64_t) abs(AdcData[ChnNum(i)] - AcZero[0]);
		}

	Counts++;

	// Restart ADC
	// StartAdc();
#ifdef DEBUG_COLLECT
	if (NCollected < DEBUG_COLLECT) {
		AcCollect[NCollected] = SdAdcData[1];
		CurrCollect[NCollected] = ((AdcData[ChnNum(0)] - AcZero[0]) * Vref * 10000) / (4096 * CURRENT_MV_PER_AMP);
		NCollected++;
		if (MinCollected > SdAdcData[1])
			MinCollected = SdAdcData[1];
		if (MaxCollected < SdAdcData[1])
			MaxCollected = SdAdcData[1];
		AcPower += abs(CurrCollect[NCollected]) * SdAdcData[1];
	}
#endif
} // void measure_int()

// TIM7 interrupt
void timer_int() {
	debug("TIM7 int!\n");
	FlagReport = 1;
} // void timer_int()

void loop() {
	CheckNextCmd();
	if (FlagProcessCmd) {
		debug("Processing new command: %s", CmdNext);
		ProcessCmd(CmdNext);
		FlagProcessCmd = 0;
	}
	UpdateZeroCurrent();
	// XXX - add read from flash after delay
	if (FlagReport) {
		debug("Reporting (measures: %d)\n", Counts);
		for (int i = 0; i < MY_AC_CHANNELS; i++) {
			SerialPrint("%d:%u\n", i,
					(uint32_t) ((AcConsumed[i] * (uint64_t) Vref) / ((uint64_t) Counts * (uint64_t) 4096 * (uint64_t) CURRENT_MV_PER_AMP)));
			AcConsumed[i] = 0;
		}
		// Calculate Vrms with some correction
		AcVoltage = (AcVoltageSum * 111.0) / (Counts * 1000.0);
		SerialPrint("100:%u\n", (uint32_t) ((AcVoltageSum * (uint64_t) 111) / (uint64_t) Counts));
		Counts = 0;
		AcVoltageSum = 0;
		FlagReport = 0;

#ifdef DEBUG
		debug("Vref: %d, Vdd: %d, Tchip %d\n", Vref, Vdd, Tchip);
		debug("0: %d, 1: %d, 2: %d, 3: %d, 4: %d, 5: %d, 6: %d, 7: %d, 8: %d, 9: %d, 10: %d, TempNTC: %d, AC: %d\n",
				AdcData[0], AdcData[1], AdcData[2], AdcData[3],
				AdcData[4], AdcData[5], AdcData[6], AdcData[7],
				AdcData[8], AdcData[9], AdcData[10],
				SdAdcData[0], SdAdcData[1]);
		debug("0: %d, 1: %d, 2: %d, 3: %d, 4: %d, 5: %d, 6: %d, 7: %d, 8: %d, 9: %d, 10: %d, TempNTC: %d, AC: %d\n",
				AdcBuf[0][0], AdcBuf[0][1], AdcBuf[0][2], AdcBuf[0][3],
				AdcBuf[0][4], AdcBuf[0][5], AdcBuf[0][6], AdcBuf[0][7],
				AdcBuf[0][8], AdcBuf[0][9], AdcBuf[0][10],
				SdAdcData[0], SdAdcData[1]);
		debug("Zero:\n0: %d, 1: %d, 2: %d, 3:%d\n", AcZero[0], AcZero[1], AcZero[2], AcZero[3]);
		debug("MaxCntAdc: %d\n", MaxCntAdc);
		MaxCntAdc = 0;

		#ifdef DEBUG_COLLECT
		if (NCollected >= DEBUG_COLLECT) {
			for (int i = 0; i < NCollected; i++)
				debug("C: %4d:%5d:%7d\n", i, AcCollect[i], CurrCollect[i]);
			debug("Min: %d, Max: %d, RMS: %d, Power: %u\n", MinCollected, MaxCollected, (MaxCollected * 1000) / 1414, AcPower);
			MinCollected = 0xffff;
			MaxCollected = 0;
			NCollected = 0;
			AcPower = 0;
		}
#endif
#endif
	}
} // void loop()
