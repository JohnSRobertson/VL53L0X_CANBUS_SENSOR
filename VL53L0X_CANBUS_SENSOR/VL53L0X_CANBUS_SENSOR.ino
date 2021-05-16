/*
 Name:		VL53L0X_CANBUS_SENSOR.ino
 Created:	7/15/2020 4:09:58 PM
 Author:	STS89
*/


#include <SD.h>
#include <SPI.h>
#include <mcp_can.h> 
#include <string.h>
/* Can-Bus Shield HW - Ebay Order # 302214924267-1617262650020 - Mar 25, 2019 
 * Software Library and similar product
 * https://github.com/Seeed-Studio/CAN_BUS_Shield
 * http://henrysbench.capnfatz.com/henrys-bench/arduino-projects-tips-and-more/arduino-can-bus-module-pin-outs-and-schematics/
 */
#include "Seeed_vl53l0x.h"
/* VL53L0X Breakout Board HW - EBay Order # 282250413266-1960392575018 - Mar 04, 2019 
 * https://github.com/Seeed-Studio/Grove-Ranging-sensor-VL53L0X
 */

const bool DEBUG = true;  // True sprint debug info via serial port; disable in production
const bool SD_CARD = true; // Log to an SD card if enabled.
const String logFileNameBase = "DLog";         // Datalog file name prefix 
const String logFileNum  = "DLogN.txt";  // File containing the number suffix of the last logFile written.

/*
 * ATOMIC_BLOCK processing (to disable interrupt processing for key code blocks) is not currently defined within the
 * Arduino environment for the SAMP D21 processor.  For the msTimer variable (which is only written within the loop to reset), 
 * the risk of not disabling interrupt processing during the reset is assumed to be not critical (and thus ignored here).
 *
 * Should this become an issue, it appears the code found in
 * https://github.com/WestfW/SAMD10-experiments/blob/master/D10-LED_TOGGLE0/src/UserSource/ticker_rtc.c#L24
 * illustrates how to disable interrupts during critical code areas.
*/

/*
   CAN CONFIGURATION
*/
const long CAN_ADDR_RF1 = 0x186AA; // Decimal 100010 - Address of the 1st Laser Rangefinder
const long CAN_ADDR_RF2 = 0x186AB; // Decimal 100011 - Address of the 2nd Laser Rangefinder
const bool CAN_ADDR_EXT_FLAG = true;
// TODO: Determine size and format of CAN data buffers
const byte CAN_DATA_SENSOR1_BUF_SIZE = 8; // Distance measurement buffer length
// TODO: Send some sensor status information???
const int  CAN_XMIT_INTERVAL_MILLISEC = 100;  // CAN message update interval for the distance data in milliSeconds
const uint32_t sampleRate = 1024000; //sample rate, determines how often SAMP TC5_Handler is called 1024000 = 1 milliSec, 1000 = ~1 Sec

// Seeduino XIAO Pin Assignments
const byte SPI_CS_CAN_BUS_PIN = 2;  // SPI select pin for the CAN BUS interface module
const byte SPI_CS_SD_CARD_PIN = 3;  // SPI select pin for the SD Card
/*
 * SPI and SD interface connections - doc only -- do not uncomment
 * SPI_MOSI_PIN = 10; 
 * SPI_MISO_PIN =  9; 
 * SPI_CLK_PIN  =  8; 
 * I2C_SDA_PIN  =  4; 
 * I2C_SCL_PIN  =  5; 
*/

// Measurement Status Constants
const int16_t OUT_OF_RANGE      = -50;
const int16_t MEASUREMENT_ERROR = -75;
const int16_t NO_MEASUREMENT    = -100;
 
/*
 * Global Objects and Variables
 */
MCP_CAN CAN(SPI_CS_CAN_BUS_PIN); // initialize the MCP 2515 CAN Protocol Object
Seeed_vl53l0x VL53L0X_1;         // 1st Laser Rangefinder Object
Seeed_vl53l0x VL53L0X_2;         // 2nd Laser Rangefinder Object -- future use

volatile int msTimer = 0;
byte canDataBuf[CAN_DATA_SENSOR1_BUF_SIZE]; // documentation:= {dist0, dist1, dist2, dist3} - values of multiple int16 distances recorded prior to each CAN XMIT
int16_t distance1 = 0;
byte baseIndex = 0;
byte sampleNum = 0;
 
bool sdValid = false; // flag indicating SD card successfully initialized
String logFileName = "";  // The SD logfile name with a unique suffix computed for each logging instance
String progId;  // A string containing the sketch path and compilation time to allow a version id to be noted in the data
String logMsg;  // A string containing the status feedback
int32_t seq = 0;  // sequential row id with the SD datafile



void setup() {
	SerialUSB.begin(115200);
	delay(5000);
	Serial.println("Starting");
	displayRunningSketch();
	initSdCard();
	initCanBus();
	delay(500);
	initVL53L0X();
	tcConfigure(sampleRate); //configure the interrupt timer to run at <sampleRate> Hertz x 1000
	tcStartCounter(); //starts the timer
	delay(500);
	Serial.println("Exit Setup");
}
/*
 * MAIN LOOP 
 * (a) continously read rangefinder sensor and store results in CAN data buffers
 * (b) transmit CAN message every time XMIT interval exceeded
 * (c) reset data buffers and (safely) reset interval counter. Counter is incremented every mSec by hardware timer
 */
void loop() {
	distance1 = measureDistance(); // currently blocks during read - by default takes about 30 mSec.
    // store multiple samples (prior to CAN XMIT) in a transmit buf
	baseIndex = sampleNum * 2;
	canDataBuf[baseIndex]     = highByte(distance1);
	canDataBuf[baseIndex + 1] = lowByte(distance1);
	sampleNum++;

	// xmit the CAN data at periodic intervals
	if (msTimer >= CAN_XMIT_INTERVAL_MILLISEC) {
		// reset the CAN_BUS XMIT timer.  Note: this is not an interrupt safe operation!
		msTimer = 0;
		
		// send the CAN message
		CAN.sendMsgBuf(CAN_ADDR_RF1, CAN_ADDR_EXT_FLAG, CAN_DATA_SENSOR1_BUF_SIZE, canDataBuf);

		// write to an SD card if logging enabled
		writeSd(seq, canDataBuf, CAN_DATA_SENSOR1_BUF_SIZE);

		// reset counters and data buf to known values
		seq++;
		sampleNum = 0;
        // reset the CAN data buffer with known "no measurement" value
		for (byte n = 0; n < CAN_DATA_SENSOR1_BUF_SIZE; n++) {
            canDataBuf[n] = 0xFF;
		}		
	}
}

// SAMP hardware interrupt timer used to count at precise 1 mSec intervals
void TC5_Handler(void) {
	msTimer++;
	TC5->COUNT16.INTFLAG.bit.MC0 = 1; //Writing a 1 to INTFLAG.bit.MC0 clears the interrupt so that it will run again
}


/*
 * Perform a single measurement from the rangefinder.  Postive values are distance in mm;  negative values are invalid readings.
 */
int16_t measureDistance() {
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	// why is this needed?
	//memset(&RangingMeasurementData, 0, sizeof(VL53L0X_RangingMeasurementData_t));
	Status = VL53L0X_1.PerformSingleRangingMeasurement(&RangingMeasurementData);
	if (VL53L0X_ERROR_NONE == Status) {
		if (RangingMeasurementData.RangeMilliMeter >= 2000) {
			Serial.println("out of range!!");
			return (OUT_OF_RANGE);
		}
		else {
			if (DEBUG) {
				Serial.print("Measured distance:");
				Serial.print(RangingMeasurementData.RangeMilliMeter);
				SerialUSB.println(" mm");
			}
			return(RangingMeasurementData.RangeMilliMeter);
		}
	}
	else {
		if (DEBUG) {
			Serial.print("mesurement failed !! Status code =");
			Serial.println(Status);
		}
		return (MEASUREMENT_ERROR);
	}
}




void initSdCard() {
	// see if the card is present, can be initialized, and set the logfile name
	if (SD_CARD) {
		File logFileHandle;
		File logFileNumHandle;
		String nextLogFileName;
		String curNum;
		int nextNum;

		if (SD.begin(SPI_CS_SD_CARD_PIN)) {
			sdValid = true;

			// determine log file name for this logging session
			if (SD.exists(logFileNum)) {
				logFileNumHandle = SD.open(logFileNum, FILE_READ);
				if (logFileNumHandle) {
					curNum = logFileNumHandle.readStringUntil('\n');
					nextNum = curNum.toInt() + 1;
					logFileNumHandle.close();
					logFileName = logFileNameBase + "_" + String(nextNum) + ".txt";
					
				}
				else {
					Serial.print("SD Card: Failure opening " + logFileNum + " to read previous log suffix");
				}
				SD.remove(logFileNum);
				logFileNumHandle = SD.open(logFileNum, FILE_WRITE);
				if (logFileNumHandle) {
					logFileNumHandle.println(nextNum);
					logFileNumHandle.close();
				}
				else {
					Serial.print("SD Card: Failure opening " + logFileNum + " to write new log suffix");
				}
			}
			else {
				logFileNumHandle = SD.open(logFileNum, FILE_WRITE);
				if (logFileNumHandle) {
					logFileNumHandle.println("0");
					logFileNumHandle.close();
				}
				else {
					Serial.print("SD Card: Failure opening " + logFileNum + " to create initial '0' log suffix");
				}
				logFileName = logFileNameBase + "_0.txt";
			}

			// open the logfile and write the header information
			logFileHandle = SD.open(logFileName, FILE_WRITE);
			if (logFileHandle) {
				logFileHandle.println("# " + progId);
				logFileHandle.println("Seq, D0, D1, D2, D3, b0, b1, b2, b3, b4, b5, b6, b7");
				logFileHandle.close();
			}
			else {
				Serial.print("SD Card: Failure opening " + logFileName + " to write header information");
			}

		} else {
			sdValid = false;
			if (DEBUG) {
				Serial.println("SD Card: Card not present (or begin() failed)");
			}
		}
	}
	else {
		sdValid = false;
	}
}

void writeSd(int32_t n, uint8_t dbuf[], uint8_t dbufSize) {
	if (sdValid) {
         // form the csv delimited data row
		//seq#, dist1, dist2, dist3, dist4, byte0, byte1 ... byte7
		uint16_t x; 

		x = (dbuf[0] * 256) + dbuf[1];
		String d0 = String(x);

		x = (dbuf[2] * 256) + dbuf[3];
		String d1 = String(x);

		x = (dbuf[4] * 256) + dbuf[5];
		String d2 = String(x);

		x = (dbuf[6] * 256) + dbuf[7];
		String d3 = String(x);

		String b0 = String(dbuf[0]);
		String b1 = String(dbuf[1]);
		String b2 = String(dbuf[2]);
		String b3 = String(dbuf[3]);
		String b4 = String(dbuf[4]);
		String b5 = String(dbuf[5]);
		String b6 = String(dbuf[6]);
		String b7 = String(dbuf[7]);

		String line = String(n) + ',' + d0 + ',' + d1 + ',' + d2 + ',' + d3 + ','
			+ b0 + ',' + b1 + ',' + b2 + ',' + b3 + ',' + b4 + ',' + b5 + ','+ b6 + ',' + b7;

		File logFileHandle = SD.open(logFileName, FILE_WRITE);
		if (logFileHandle) {
			logFileHandle.println(line);
			logFileHandle.close();
		}
	}
}

/*
   INITIALZE THE CAN-BUS INTERFACE
*/
void initCanBus() {
	SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); // Initialize the SPI interface clock rate & other characteristics
	while (CAN_OK != CAN.begin(CAN_1000KBPS, MCP_8MHz))
	{
		logMsg = "initCanBus: begin() failed.  Retrying in 100 ms";
		Serial.println(logMsg);
		logDebug(logMsg);
		delay(100);
	}
	SPI.endTransaction();
	logMsg = "initCanBus: successful";
	Serial.println(logMsg);
	logDebug(logMsg);
}


void initVL53L0X() {
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	
	while((Status = VL53L0X_1.VL53L0X_common_init()) != VL53L0X_ERROR_NONE) { 
        logMsg = "initVL53L0X: common_init() failed with status " + String(Status) + " - retrying";
		Serial.println(logMsg);
		logDebug(logMsg);
		delay(100);
		// char buf[VL53L0X_MAX_STRING_LENGTH];
		// VL53L0X_GetPalErrorString(Status, buf);
		// Serial.print("API Status:");
		// Serial.print(Status);
		// Serial.print("API error string:");
		// Serial.println(buf);
	}
	logMsg = "initVL53L0X: common_init() successful";
	Serial.println(logMsg);
	logDebug(logMsg);

	while ((Status = VL53L0X_1.VL53L0X_single_ranging_init()) != VL53L0X_ERROR_NONE) {
		logMsg = "initVL53L0X: single_ranging_init() failed with status " + String(Status) + " - retrying";
		Serial.println(logMsg);
		logDebug(logMsg);
		delay(100);
	}
	logMsg = "initVL53L0X: single_ranging_init() successful";
	Serial.println(logMsg);
	logDebug(logMsg);
}


// displays at startup the Sketch running in the Arduino
void displayRunningSketch(void) {
	String path = __FILE__;
	progId = "Arduino File: " + path + " Compliled: " + __DATE__  + " " + __TIME__;

	Serial.print("\nFile: ");
	Serial.println(__FILE__);
	Serial.print("Compiled: ");
	Serial.print(__DATE__);
	Serial.print(" at ");
	Serial.print(__TIME__);
	Serial.print("\n");
}

// Display and log debug information
void logDebug(String s) {
	if (DEBUG) {
		Serial.println(s);
		if (sdValid) {
			File dataFile = SD.open(logFileName, FILE_WRITE);
			if (dataFile) {
				dataFile.println("# " + s);
				dataFile.close();
			}
		}
	}
}

/*
 *  TIMER SPECIFIC FUNCTIONS FOLLOW
 *  you shouldn't change these unless you know what you're doing
 *  https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f
 */

 //Configures the TC to generate output events at the sample frequency.
 //Configures the TC in Frequency Generation mode, with an event output once
 //each time the audio sample frequency period expires.
void tcConfigure(int sampleRate)
{
	// select the generic clock generator used as source to the generic clock multiplexer
	GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
	while (GCLK->STATUS.bit.SYNCBUSY);

	tcReset(); //reset TC5

	// Set Timer counter 5 Mode to 16 bits, it will become a 16bit counter ('mode1' in the datasheet)
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
	// Set TC5 waveform generation mode to 'match frequency'
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	//set prescaler
	//the clock normally counts at the GCLK_TC frequency, but we can set it to divide that frequency to slow it down
	//you can use different prescaler divisons here like TC_CTRLA_PRESCALER_DIV1 to get a different range
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE; //it will divide GCLK_TC frequency by 1024
	//set the compare-capture register. 
	//The counter will count up to this value (it's a 16bit counter so we use uint16_t)
	//this is how we fine-tune the frequency, make it count to a lower or higher value
	//system clock should be 1MHz (8MHz/8) at Reset by default
	TC5->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock / sampleRate);

	while (tcIsSyncing());

	// Configure interrupt request
	NVIC_DisableIRQ(TC5_IRQn);
	NVIC_ClearPendingIRQ(TC5_IRQn);
	NVIC_SetPriority(TC5_IRQn, 0);
	NVIC_EnableIRQ(TC5_IRQn);

	// Enable the TC5 interrupt request
	TC5->COUNT16.INTENSET.bit.MC0 = 1;
	while (tcIsSyncing()); //wait until TC5 is done syncing 
}

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
	return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
	while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
	TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while (tcIsSyncing());
	while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
	TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
	while (tcIsSyncing());
}

