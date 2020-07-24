/*
 Name:		VL53L0X_CANBUS_SENSOR.ino
 Created:	7/15/2020 4:09:58 PM
 Author:	STS89
*/


#include <SD.h>
#include <SPI.h>
#include <mcp_can.h>  
/* Can-Bus Shield HW - Ebay Order # 302214924267-1617262650020 - Mar 25, 2019 
 * Software Library and similar product
 * https://github.com/Seeed-Studio/CAN_BUS_Shield
 * http://henrysbench.capnfatz.com/henrys-bench/arduino-projects-tips-and-more/arduino-can-bus-module-pin-outs-and-schematics/
 */
#include "Seeed_vl53l0x.h"
/* EBay Hardware Purchase - */


/*
   CAN CONFIGURATION
*/
const long CAN_ADDR_RF1 = 0x186AA; // Decimal 100010 - Address of the 1st Laser Rangefinder
const long CAN_ADDR_RF2 = 0x186AB; // Decimal 100011 - Address of the 2nd Laser Rangefinder
const bool CAN_ADDR_EXT_FLAG = true;
// TODO: Determine size and format of CAN data buffers
const byte CAN_DATA_STATUS_BUF_SIZE = 2; // ABS Health and Status data buffer
const byte CAN_DATA_WS_BUF_SIZE = 8;     // ABS Wheel Speed Sensors data buffer

const int  CAN_XMIT_INTERVAL_MILLISEC = 100;  // interval between CAN message updates on the ABS data in milliSeconds


const byte SPI_CS_PIN = 10;
//const byte SPI_MOSI_PIN = 10; // Seeduino XIAO - documentation only - do not uncomment
//const byte SPI_MISO_PIN =  9; // Seeduino XIAO - documentation only
//const byte SPI_CLK_PIN =   8; // Seeduino XIAO - documentation only
//const byte I2C_SDA_PIN = 4; // Seeduino XIAO - documentation only
//const byte I2C_SCL_PIN = 5; // Seeduino XIAO - documentation only

MCP_CAN CAN(SPI_CS_PIN); // initialize the MCP 2515 CAN Protocol Object
Seeed_vl53l0x VL53L0X;
const int chipSelect = 3;
int seq = 0;
int distance = 0;
int t1 = 0;
int t2 = 0;
int t3 = 0;
int t4 = 0;
int t5 = 0;

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL SerialUSB
#else
#define SERIAL Serial
#endif



void setup() {
	SERIAL.begin(115200);
	displayRunningSketch();
	initSdCard();
	writeSdHeader();
	initCanBus();
	initVL53L0X();
	delay(500);
}

// the loop function runs over and over again until power down or reset
void loop() {
  
}

// displays at startup the Sketch running in the Arduino
void displayRunningSketch(void) {
	String the_path = __FILE__;
	int slash_loc = the_path.lastIndexOf('/');
	String the_cpp_name = the_path.substring(slash_loc + 1);
	int dot_loc = the_cpp_name.lastIndexOf('.');
	String the_sketchname = the_cpp_name.substring(0, dot_loc);

	Serial.print("\nArduino is running Sketch: ");
	Serial.println(the_sketchname);
	Serial.print("File: ");
	Serial.println(__FILE__);
	Serial.print("Compiled on: ");
	Serial.print(__DATE__);
	Serial.print(" at ");
	Serial.print(__TIME__);
	Serial.print("\n");
}


void initSdCard() {
	// see if the card is present and can be initialized:
	if (!SD.begin(chipSelect)) {
		Serial.println("Card failed, or not present");
		// don't do anything more:
		while (1);
	}
}

void writeSdHeader() {
	//if (SD.exists("datalog.txt")) {
	//  SD.remove("datalog.txt");
	//}
	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	if (dataFile) {
		dataFile.println("Seq, Range(mm), t1, t2, t3, t4, t5");
		dataFile.close();
	}
}

/*
   INITIALZE THE CAN-BUS INTERFACE
*/
void initCanBus() {
	SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); // Initialize the SPI interface clock rate & other characteristics
	while (CAN_OK != CAN.begin(CAN_1000KBPS, MCP_8MHz))
	{
		Serial.println("CAN BUS Interface Init Failed - Retrying");
		delay(100);
	}
	SPI.endTransaction();
	Serial.println("CAN BUS Interface Init OK!");
}

void initVL53L0X() {
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	Status = VL53L0X.VL53L0X_common_init();
	if (VL53L0X_ERROR_NONE != Status) {
		SERIAL.println("start vl53l0x measurement failed!");
		VL53L0X.print_pal_error(Status);
		while (1);
	}
	VL53L0X.VL53L0X_continuous_ranging_init();
	if (VL53L0X_ERROR_NONE != Status) {
		SERIAL.println("start vl53l0x mesurement failed!");
		VL53L0X.print_pal_error(Status);
		while (1);
	}
}

