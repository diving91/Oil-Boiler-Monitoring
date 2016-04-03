# Oil-Boiler-Monitoring
MySensors sketch to Monitor an oil fired boiler &amp; Tune boiler water temperature based on External temperature provided by Mysensors controller

Fuel Sensor V1.0 - Diving91 - 2016 <br>
Mysensors Library used	: 2.0.0-beta<br>
Repeater Mode		: NO<br>
Power Supply		: From Mains, No Battery<br>
HW type			: Arduino Pro Mini 3.3V 8MHz<br>
Node Name		: Fuel Sensor<br> 
Purpose			: Monitoring of an oil fired boiler & Tune boiler water temperature based on External temperature provided by the controller<br>
<br>
<b>Description</b><br>
When boiler starts, Fuel Valve ON info is sent to the controller and time counter is started. A LED_PIN is set to HIGH
<br>
When Boiler stops,<br>
	- LED_PIN is set to LOW<br>
	- Fuel Valve OFF info is sent to the controller<br>
	- Fuel volume is computed, stored to EEPROM (in milli liters) in a circular buffer and sent to the controller (in liters)<br>
	- Sketch requests External temperature value to the controller, and compare it to HighTemp, LowTemp threshold values, and set RELAY_PIN based on this comparison<br>
<br>
The controller can perform 3 actions<br>
 	- Reset Fuel volume: The sketch will set volume to 0, request UTC time and send back LOCALE (GMT+1 Europe DST) time to the controller<br>
	- Set HighTemp threshold: The sketch stores it in EEPROM and send it back to controller<br>
	- Set LowTemp threshold: The sketch stores it in EEPROM and send it back to controller<br>
<br>
The RELAY_PIN is used to set boiler water temperature to 2 predetermined values (depends on your boiler characteritics)<br>
Eg: When External temperature > 5.0°C, boiler water temperature of 65°C is enough<br>
Eg: When External temperature < -1.0°C, boiler water temperature of 70°C or 72°C is better<br>
<br>
<b>Usage</b><br>
* 	When starting the node, take care the boiler is OFF as this is what the sketch assumes<br>
*	Fuel Boiler valve is ON when VALVE_PIN is LOW<br>
*	Boiler Temperature HIGH when RELAY_PIN is LOW<br>
*	Before first usage, erase EEPROM with 0xFF<br>
* 	At first usage Reset Fuel volume Counter and Send HighTemp & LowTemp Values<br>
*	At power on or after reset, Relay is set for HighTemp mode<br>
*	At power on or after reset, the sketch sends to controller stored values for Fuel Volume, HighTemp, LowTemp, so that the controller is initialized<br>
*	A DBG_PIN is used to determine at power on, if sketch debug information has to be sent on serial console or not
*	debug info from MySensors are controller at sketch compile time, not during run time<br>

