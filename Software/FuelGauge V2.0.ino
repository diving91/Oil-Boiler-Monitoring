/* Fuel Sensor V2.0 - Diving91 - 2016
*  Mysensors Library used	: 2.0.0-beta
*  Repeater Mode		: NO
*  Power Supply			: From Mains, No Battery
*  HW type			: Arduino Pro Mini 3.3V 8MHz
*  Node Name			: Fuel Sensor 
*  Purpose			: Monitoring of an oil fired boiler & Tune boiler water temperature based on External temperature provided by the controller
*  Description
*	When boiler starts, Fuel Valve ON info is sent to the controller and time counter is started. A LED_PIN is set to HIGH
*	When Boiler stops,
		LED_PIN is set to LOW
		Fuel Valve OFF info is sent to the controller
		Fuel volume is computed, stored to EEPROM (in milli liters) in a circular buffer and sent to the controller (in liters)
		Sketch requests External temperature value to the controller, and compare it to HighTemp, LowTemp threshold values, 
		and set RELAY_PIN based on this comparison
*	The controller can perform 3 actions
* 		Reset Fuel volume: The sketch will set volume to 0, request UTC time and send back LOCALE (GMT+1 Europe DST) time to the controller
*		Set HighTemp threshold: The sketch stores it in EEPROM and send it back to controller
*		Set LowTemp threshold: The sketch stores it in EEPROM and send it back to controller
*	The RELAY_PIN is used to set boiler water temperature to 2 predetermined values (depends on your boiler characteritics)
*		Eg: When External temperature > 5.0C, boiler water temperature of 65C is enough
*		Eg: When External temperature < -1.0C, boiler water temperature of 70C or 72C is better		
*  Usage :
* 	When starting the node, take care the boiler is OFF as this is what the sketch assumes
*	Fuel Boiler valve is ON when VALVE_PIN is LOW
*	Boiler Temperature HIGH when RELAY_PIN is LOW
*	Before first usage, erase EEPROM with 0xFF
* 	At first usage Reset Fuel volume Counter and Send HighTemp & LowTemp Values
*	At power on or after reset, Relay is set for HighTemp mode
*	At power on or after reset, the sketch sends to controller stored values for Fuel Volume, HighTemp, LowTemp, so that the controller is initialized
*	A DBG_PIN is used to determine at power on, if sketch debug information has to be sent on serial console or not
*	debug info from MySensors are controller at sketch compile time, not during run time
* V2.0 Addition :
*	Add ability to send volume in its seconds equivalent to let Controllers do the conversion when desired
* 	Add some wait() during presentation for better reliability 
*	Some bug fixes
*	Add 1-wire DS18B20 sensors reading and sending to Mysensors - Values are sent every custom defined time with & without redundancy
*	To be used with 2 sensors to monitor Water temperature out & in from the water path
*	When returning water temperature is too high, this indicates your Boiler water temperature might be too high
*	If no DS18B20 devices are attached, info is not presented to Mysensors controller
*/


// Enable MySensors debug prints to serial monitor
//#define MY_DEBUG
//#define MY_DEBUG_VERBOSE

// Enable and select radio type attached
#define MY_RADIO_NRF24
// Redefine CE/CS pin to ease routing PCB
#define MY_RF24_CE_PIN 10
#define MY_RF24_CS_PIN 9
		
#include <SPI.h>
#include <MySensor.h>
#include <Bounce2.h>
#include <Time.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* START MySensors definition */
#define NODE_VERSION 	"2.0"
#define NODE_NAME 	"Fuel Sensor"
//Info IDs
#define FUEL_STATE_ID 	1 	// S_CUSTOM V_VAR1 Boolean Info for Valve state
#define FUEL_VOLUME_ID 	2 	// S_CUSTOM V_VAR1 Float Info for cumulative fuel volume used since last reset action
#define RST_TIME_ID 	3 	// S_CUSTOM V_VAR1 String dd/mm/yyyy mm:hh on when Fuel Volume has been reset to 0.0
#define TEMP_HIGH_I_ID 	4 	// S_TEMP V_TEMP Info to reflect received Action on TEMP_HIGH_A_ID
#define TEMP_LOW_I_ID	5 	// S_TEMP V_TEMP Info to reflect received Action on TEMP_LOW_A_ID
#define TEMP_ORDER_ID	6 	// S_CUSTOM V_VAR1 Boolean Info for Relay state
#define TEMP_EXTERN_ID	7 	// S_CUSTOM V_VAR5 Float Info for External temperature - Info Requested to the controller as Virtual info
//Action parameter IDs
#define FUEL_RESET_ID 	8 	// V_VAR1 Parameter to reset fuel volume, Used as a trigger
#define TEMP_HIGH_A_ID 	9 	// V_TEMP Parameter to set HIGH temperature value for Switching relay ON
#define TEMP_LOW_A_ID 	10 	// V_TEMP Parameter to set LOW temperature value for Switching relay OFF
// 1-Wire IDs start
#define DS18B20_BASE_ID	11	// S_TEMP V_TEMP Info to reflect 1-Wire water temperature (2 sensors water-out & water-in)
/* END mysensors definition */

/* START Pinout definition */
#define VALVE_PIN  	3  	// Input Pin for Fuel valve Solenoid
#define LED_PIN 	4 	// Output Pin for LED reflecting the Fuel valve state
#define DBG_PIN		5 	// Input Pin for DBG. When low, serial print are enabled
#define RELAY_PIN 	6 	// Output Pin for RELAY used to adjust Boiler Temperature
#define ONE_WIRE_BUS 	7	// Pin were sensors are attached
/* END Pinout definition */

/* START DEBUG utility */
bool debugMode;
#define DBG_START(x)	if (debugMode) Serial.begin(x);
#define DBG_PRINT(x)	if (debugMode) Serial.print(x);
#define DBG_PRINTLN(x)	if (debugMode) Serial.println(x);
/* END DEBUG utility */

/* START sketch Macro definition */
#define LED_TOGGLE digitalWrite (LED_PIN, !digitalRead(LED_PIN))
#define VALVE_ON debouncer.fell()
#define VALVE_OFF debouncer.rose()
#define RELAY_WINTER_MODE HIGH
#define RELAY_SUMMER_MODE LOW
// time to volume convertion
// 1 US Gallon = 3,785411784 liters
// 0.6 US Gallon/h = 2,27124707 liters/hour
// 0,630901964 ml/seconds
// 37,85411784 ml/minutes
#define OUTFLOW 0.6
#define DS18B20_MAX_SENSORS 2 		// Max Number of supported devices
#define DS18B20_PRECISION 10 		// 10 bits resolution - conv time 187.5ms - resolution 0.25C
#define DS18B20_TIME 15000		// 15s between 2 communications - This time must be > device conversion time - see Datasheet
#define DS18B20_NO_REDUNDANCY true	// Send temperature only if changed? true = Yes / false = No
/* END sketch Macro definition */

/* START EEPROM Definition - 328P has 1024Bytes */
#define EEADDR0	EEPROM_LOCAL_CONFIG_ADDRESS 	// 4bytes for rstTime
#define EEADDR1	EEADDR0 + 4 			// 4bytes for highTemp
#define EEADDR2	EEADDR1 + 4 			// 4bytes for lowTemp
#define EECADDR EEADDR2 + 4			// Start of Circular buffer
#define EVSIZE	5				// EEPROM Var size - 4bytes for volume + 1byte flag
#define ECLEN	50				// EEPROM Circular buffer Lenght - Circular Buffer size = ECLEN x EVSIZE
#define EFREE	0xFF				// When 0xFF encountered, EEPROM location is NAN
#define EBUSY	0xFE				// When 0xFE encountered, EEPROM location is 4bytes float
/* END EEPROM Definition - 328P has 1024Bytes */

/* START Define Globals */
unsigned long previousMillis;
unsigned long fuelTime;
float volume;
float highTemp,lowTemp;
unsigned long rstTime;
char timeRstString[16];
int eeVolumeIndex; // EEPROM Address pointer for Fuel Volume;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DSsensors(&oneWire);
int nbDSSensors; // Number of 1-Wire temperature Sensors found
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
float lastTemperature[DS18B20_MAX_SENSORS];
unsigned long DS18B20Millis;
bool newDS18B20ConvToStart; 

Bounce debouncer = Bounce(); 

MyMessage msgFuelState(FUEL_STATE_ID, V_VAR1);
MyMessage msgFuelVolume(FUEL_VOLUME_ID, V_VAR1);
MyMessage msgRstTime(RST_TIME_ID, V_VAR1);
MyMessage msgTempHighI(TEMP_HIGH_I_ID, V_TEMP);
MyMessage msgTempLowI(TEMP_LOW_I_ID, V_TEMP);
MyMessage msgTempOrder(TEMP_ORDER_ID, V_VAR1);
MyMessage msgDS18B20(DS18B20_BASE_ID,V_TEMP);
/* END Define Globals */

void before() {
}

void setup() {  
	// Setup DBG_PIN, Activate internal pull-up & Read debugMode value
	pinMode(DBG_PIN,INPUT);
	digitalWrite(DBG_PIN,HIGH);
	debugMode = digitalRead(DBG_PIN) ? false:true;
	
	DBG_START(115200);
	DBG_PRINTLN("Fuel Sensor Started");

	//setup LED_PIN and switch off
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, 0);
  
	// Setup VALVE_PIN and Activate internal pull-up
	pinMode(VALVE_PIN,INPUT);
	digitalWrite(VALVE_PIN,HIGH);
	// After setting up the VALVE_PIN, setup debouncer
	debouncer.attach(VALVE_PIN);
	debouncer.interval(5);
	
	// Setup RELAY_PIN and swith pin to RELAY_SUMMER_MODE
	pinMode(RELAY_PIN,OUTPUT);
	digitalWrite(RELAY_PIN,RELAY_SUMMER_MODE);
	
	// Init EEPROM index of Fuel volume
	findEEVolumeIndex();
	
	// Read Fuel volume from EEPROM
	eeReadVolume();
	DBG_PRINTLN("EEPROM index: " + String(eeVolumeIndex) + " - Fuel Volume: " + String(volume,3));
	
	// Read Last Reset Time from EEPROM
	EEPROM.get(EEADDR0, rstTime);
	setTime(rstTime);
	sprintf(timeRstString, "%02d/%02d/%04d %02d:%02d", day(), month(), year(), hour(), minute());
	
	// Read last highTemp & lowTemp from EEPROM
	EEPROM.get(EEADDR1, highTemp);
	EEPROM.get(EEADDR2, lowTemp);
	
	if (debugMode) {dumpCircularBuffer();}

	// START Setup for DS18B20 1-Wire temperature Sensors
	// Start up the library & Grab a count of devices on the wire
	DSsensors.begin();
	nbDSSensors = DSsensors.getDeviceCount(); 
	if (nbDSSensors == 0) {DBG_PRINTLN("No 1-Wire Temp sensors found");}
	else {
		// Loop through each devices, display their address and resolution. If not the desired resolution, change it
		// If more than DS18B20_MAX_SENSORS sensors are found, only the first ones will be taken into account
		// myDevice 0 addr: 28 78971E000080 60
		// myDevice 1 addr: 28 738A1E000080 08
		for (int i=0;i<nbDSSensors; i++) {
			// Search the wire for address
			if(DSsensors.getAddress(tempDeviceAddress, i)) {
				// Check DS18B20 device type
				if (DSsensors.validAddress(tempDeviceAddress) && (tempDeviceAddress[0]==0x28)) {
					DBG_PRINT("DS18B20 " + String(i,DEC) + ": Addr=");
					printAddress(tempDeviceAddress);
					DBG_PRINTLN(" Res="+String(DSsensors.getResolution(tempDeviceAddress),DEC)+"bits");
					// Set desired device resolution
					if (DSsensors.getResolution(tempDeviceAddress) != DS18B20_PRECISION) {
						DSsensors.setResolution(tempDeviceAddress, DS18B20_PRECISION);
						DBG_PRINTLN("-> Set Res " + String(DS18B20_PRECISION) + "bits");
					}
				}
				else { // This is not a DS18B20 - could be a DS18S20 or DS182x
					DBG_PRINT("Unsupported Device" + String(i,DEC) + ": Addr=");
					printAddress(tempDeviceAddress);
					DBG_PRINTLN("");
				}
			}
			else {
				DBG_PRINTLN("Ghost device at " + String(i, DEC) + ". Check power and cabling");
			}
		}
		DSsensors.setWaitForConversion(false); // Do not block/wait during conversion time
		DBG_PRINTLN("Sensors comm interval: " + String(DS18B20_TIME/1000.0, 0) + "sec");
		newDS18B20ConvToStart = true;
	}
 	// END Setup for DS18B20 1-Wire temperature Sensors
}

void presentation() {
	// Send MySensors Sketch information & nodes presentation
	sendSketchInfo(NODE_NAME, NODE_VERSION);
	present(FUEL_STATE_ID, S_CUSTOM); wait(250);
	present(FUEL_VOLUME_ID, S_CUSTOM); wait(250);
	present(RST_TIME_ID, S_CUSTOM); wait(250);
	present(TEMP_HIGH_I_ID, S_TEMP); wait(250);
	present(TEMP_LOW_I_ID, S_TEMP); wait(250);
	present(TEMP_ORDER_ID, S_CUSTOM); wait(250);
	present(TEMP_EXTERN_ID, S_CUSTOM); wait(250);
	for (int i=0; i<nbDSSensors && i<DS18B20_MAX_SENSORS; i++) {
		present(DS18B20_BASE_ID + i,S_TEMP); wait(250);
	}
	// Send Fuel valve position with assumption that FUEL valve is OFF when node starts 
	wait(1000);
	send(msgFuelState.set(false)); wait(250);
	// Send last Fuel volume
	send(msgFuelVolume.set(volume / 1000.0,1)); wait(250);
	// Send Last Reset Time from EEPROM
	send(msgRstTime.set(timeRstString)); wait(250);
	// Send last highTemp & lowTemp from EEPROM
	send(msgTempHighI.set(highTemp,1)); wait(250);
	send(msgTempLowI.set(lowTemp,1)); wait(250);
	// Send Relay position information (false mean RELAY_SUMMER_MODE
	send(msgTempOrder.set(false)); wait(250);
	// We do not Send water temperature information, they will be sent during the loop
	DBG_PRINTLN("Fuel Sensor Presentation Complete");
}

void loop() {
	// Get the Fuel valve state
	bool valveStateChange = debouncer.update();
	bool valveState = debouncer.read();
	//Serial.println("Debug state  "+String(valveState, DEC)+" change "+String(valveStateChange, DEC));
 	
	if (VALVE_ON) {
		previousMillis = millis();
		send(msgFuelState.set(true));
		LED_TOGGLE; 
		DBG_PRINTLN("Fuel Valve is ON");
	}
	if (VALVE_OFF) {
		fuelTime = millis()- previousMillis ; //Time in ms
		send(msgFuelState.set(false));
		LED_TOGGLE; 
		DBG_PRINT("Fuel Valve is OFF - Volume=");
		// Determine new Fuel volume, send it to Controller & Store in EEPROM
		volume += ((float)fuelTime * 3.785411784 * OUTFLOW)/3600.0 ; // Volume in ml
		// If you prefer to return amount of seconds instead of Volume and let the Mysensors controller compute the volume, use the below formula
		//volume += (float)fuelTime ; //  Cumulative ON Time in ms
		DBG_PRINTLN(String(volume,3));
		send(msgFuelVolume.set(volume / 1000.0,1));  // Volume in liters or equialent seconds
		eeWriteVolume(volume);
		request(TEMP_EXTERN_ID, V_VAR5); // Get External temperature from Controller, will call receive function
	}

	//START Handling DS18B20 1-Wire temperature Sensors
	if (nbDSSensors > 0) {
		if (newDS18B20ConvToStart) { // Issue a global temperature request, using skipROM feature ie all devices start their conversion 
			DSsensors.requestTemperatures(); 
			newDS18B20ConvToStart = false;
			DS18B20Millis = millis();
		}

		if (millis() - DS18B20Millis > DS18B20_TIME) { // Timer time reached, it's time to send temperature info
			newDS18B20ConvToStart = true;
			for (int i=0; i<nbDSSensors && i<DS18B20_MAX_SENSORS ; i++) { // Loop on all sensors
				float temperature = DSsensors.getTempCByIndex(i);
				if (!DS18B20_NO_REDUNDANCY || (DS18B20_NO_REDUNDANCY && (lastTemperature[i] != temperature))) {
					send(msgDS18B20.setSensor(DS18B20_BASE_ID + i).set(temperature,2));
					lastTemperature[i]=temperature;
					DBG_PRINTLN("Dev" + String(i,DEC) + " : " + String(temperature,2));
				}
			}
		}
	}
	//STOP Handling DS18B20 1-Wire temperature Sensors
}

void receive(const MyMessage &message) {
	//Handle Reset Fuel Volume message - Set Volume to 0.0, Store in EEPROM, Get time when Volume has been Resetted, Send time to Controller
	if ((message.type==V_VAR1) && (message.sensor==FUEL_RESET_ID)) {
		DBG_PRINTLN("Fuel Volume RESET action received")
		volume = 0.0 ;
		send(msgFuelVolume.set(0.0,1));
		eeWriteVolume(volume);
		requestTime(); // Will call receiveTime() function
	}
	//Handle Temperature High message - Get it from Controller, store in EEPROM and send it back
	if ((message.type==V_TEMP) && (message.sensor==TEMP_HIGH_A_ID)) {
		highTemp = atof(message.data);
		DBG_PRINTLN("TEMP HIGH received: " + String(highTemp,1));
		EEPROM.put(EEADDR1,highTemp);
		send(msgTempHighI.set(highTemp,1));
	}
	//Handle Temperature Low message - Get it from Controller, store in EEPROM and send it back
	if ((message.type==V_TEMP) && (message.sensor==TEMP_LOW_A_ID)) {
		lowTemp = atof(message.data);
		DBG_PRINTLN("TEMP LOW received: " + String(lowTemp,1));
		EEPROM.put(EEADDR2,lowTemp);
		send(msgTempLowI.set(lowTemp,1));
	}
	//Handle External Temperature message - Get it From Controller and Set Relay accordingly
	if ((message.type==V_VAR5) && (message.sensor==TEMP_EXTERN_ID)) {
		DBG_PRINTLN("TEMP EXTERN received: " + String(message.data));
		if (atof(message.data) >= highTemp) {
			digitalWrite(RELAY_PIN,RELAY_SUMMER_MODE); 
			send(msgTempOrder.set(false));
			DBG_PRINTLN("SET Relay for HIGH Temperature Condition");
		}
		if (atof(message.data) <= lowTemp) {
			digitalWrite(RELAY_PIN,RELAY_WINTER_MODE);
			send(msgTempOrder.set(true));
			DBG_PRINTLN("SET Relay for LOW Temperature Condition");
		}
	}
}

// Handler for requestTime()
// Get time from Controller and send it back formatted as string to the Controller
// Assumption that an UTC time is received by the controller and returns a local time
// DST computed based on source info //https://en.wikipedia.org/wiki/Summer_Time_in_Europe - Formulas valid until 2099
void receiveTime(unsigned long time) {
	unsigned long st,wt;

	// Set UTC time and get current year
	setTime(time); //UTC time
	int y = year();
	//Set Summer time transition and get its timestamp
	setTime(1,0,0,(31 - ((((5 * y) / 4) + 4) % 7)),3,y);
	st=now();
	//Set Winter time transition and get its timestamp
	setTime(1,0,0,(31 - ((((5 * y) / 4) + 1) % 7)),10,y);
	wt=now();
	//Compare current UTC time to check if within DST time window and adjust local time accordingly
	if ((time > st) && (time < wt)) {rstTime = time + 7200 ;}
	else {rstTime = time + 3600 ;}
	setTime(rstTime);
	EEPROM.put(EEADDR0, rstTime);
	// Send local time string to controller
	sprintf(timeRstString, "%02d/%02d/%04d %02d:%02d", day(), month(), year(), hour(), minute());
	DBG_PRINTLN("Reset Time " + String(timeRstString));
	send(msgRstTime.set(timeRstString));
}

// Find EEPROM Pointer of Fuel Volume
void findEEVolumeIndex() {
	int eeAddr = EECADDR;
	byte flag;
	
	for (int i = 0; i < ECLEN; i++) {
		eeVolumeIndex = i;
		EEPROM.get(eeAddr, flag);
		if (flag == EBUSY) {break;}
		eeAddr += EVSIZE;
	}
}

// Write Fuel volume at next EEPROM circular buffer index and free current index position
void eeWriteVolume(float volume) {
	// Free current position in circular buffer and point to next position
	EEPROM.put(EECADDR + eeVolumeIndex * EVSIZE, EFREE);
	eeVolumeIndex = (eeVolumeIndex + 1) % ECLEN;
	// Write Fuel volume into new buffer position
	EEPROM.put(EECADDR + eeVolumeIndex * EVSIZE, EBUSY);	
	EEPROM.put(EECADDR + 1 + eeVolumeIndex * EVSIZE, volume);
}

// Read Fuel volume from current EEPROM circular buffer index
void eeReadVolume() {
	EEPROM.get(EECADDR + 1 + eeVolumeIndex * EVSIZE, volume);
}

// Dump EEPROM Fuel volume Circular Buffer
void dumpCircularBuffer() {
	byte b,b1,b2,b3,b4;
	float f;
	int eeAddr = EECADDR;

	for (int i = 0; i < ECLEN; i++) {
		EEPROM.get(eeAddr, b);
		EEPROM.get(eeAddr+1, b1);
		EEPROM.get(eeAddr+2, b2);
		EEPROM.get(eeAddr+3, b3);
		EEPROM.get(eeAddr+4, b4);
		EEPROM.get(eeAddr+1, f);
		DBG_PRINTLN("Index " + String(i) + " - " + String(b) + " / " + String(b1) + " " + String(b2) + " " + String(b3) + " " + String(b4) + " / " + String(f,3));
		eeAddr += EVSIZE;
	}
}

// Function to print a 1-wire device address
void printAddress(DeviceAddress deviceAddress) {
	for (uint8_t i = 0; i < 8; i++) {
		if (deviceAddress[i] < 16) DBG_PRINT("0");
		DBG_PRINT(String(deviceAddress[i], HEX));
	}
}