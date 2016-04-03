# Oil-Boiler-Monitoring
MySensors sketch to Monitor an oil fired boiler &amp; Tune boiler water temperature based on External temperature provided by Mysensors controller

/* Fuel Sensor V1.0 - Diving91 - 2016
*  Mysensors Library used	: 2.0.0-beta
*  Repeater Mode		: NO
*  Power Supply			: From Mains, No Battery
*  HW type			: Arduino Pro Mini 3.3V 8MHz
*  Node Name			: Fuel Sensor 
*  Purpose			: Monitoring of an oil fired boiler & Tune boiler water temperature based on External temperature provided by the controller
*  Description
*	When boiler starts, Fuel Valve ON info is sent to the controller and time counter is started. A LED_PIN is set to HIGH
*	When Boilier stops,
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
*		Eg: When External temperature > 5.0°C, boiler water temperature of 65°C is enough
*		Eg: When External temperature < -1.0°C, boiler water temperature of 70°C or 72°C is better		
*  Usage :
* 	When starting the node, take care the boiler is OFF as this is what the sketch assumes
*	Fuel Boiler valve is ON when VALVE_PIN is LOW
*	Boiler Temperature HIGH when RELAY_PIN is LOW
*	Before first usage, erase EEPROM with 0xFF
* 	At first usage Reset Fuel volume Counter and Send HighTemp & LowTemp Values
*	At power on or after reset, Relay is set for HighTemp mode
*	At power on or after reset, the sketch sends to controller store values for Fuel Volume, HighTemp, LowTemp, so that the controller is initialized
*	A DBG_PIN is used to determine at power on, if sketch debug information has to be sent on serial console or not
*	debug info from MySensors are controller at sketch compile time, not during run time
*/
