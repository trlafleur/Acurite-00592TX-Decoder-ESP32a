# Acurite-00592TX-Decoder-ESP32a

I had been testing an Acurite 00986, Refrigerated, Freezer monitor...

I build a decoder for it, and discovered that the range once inside a refrigerated is short, as little as 6ft from the transmitter to the receiver. This is also true with the Acu-Rite display receiver... so I was disappointed...

Acurite also has a series of outdoor sensor, part numbers: 00592TX or 06002M (or one with an LCD display: 06044.) 
These sensor are design for outdoor uses and have a range of "330ft" as oppose to "75ft" for the 00989 device.
They can be order for about $13 each, or in kits with various displays or weather station options.
They are also AcuRite Environment Systems compatible if needed.

In testing from inside a refrigerated, the range was an easy 50ft...

So I modified my decoder to use these devices. The new decoder support up to 3 channels of these sensors and support MQTT over WiFi for monitoring, and E-mail or SMS messages for alarms.

It can also be use with a small local OLED display... Its based on an ESP32 development board ($10) with an RXB6 receiver ($2), but also works fine with an ESP8266.

The 00986, would TX every 2 min, and that was ok and uses 2x AA batteries. The 00592 device, TX every 18 sec and also uses 2X AA batteries. I'm not sure on battery life on the 00592 at this point, but the TX range was a great improvement and that was my goal..

Current code is on GitHub along with the older 00986 code...

https://github.com/trlafleur/Acurite-00592TX-Decoder-ESP32a

https://www.acurite.com/catalogsearch/result/?q=06044

https://www.acurite.com/kbase/592TXR.html

https://www.acurite.com/kbase/cooking-thermometers/00986_Thermometer.html

So with any luck, I will NEVER again need to be concern with someone leaving the refrigerated or freezer door open and loosing allot of food.


~~~
/**********************************************************************
 * Arduino code to decode the Acurite 00592TX wireless temperature sensor
 *
 * The 00592TX wireless temperature probe contains a 433.92 MHz
 *  wireless transmitter. The temperature from the sensor is
 *  sent approximately every 18 seconds. Their are three sensor
 *  ID's: A, B and C
 *  
 *  This device is also known as 00592TX, 06002M and 06044 and 
 *  as other devices...
 *  
 * The 00592TX typically sends three SYNC pulse + DATA stream
 * per temperature reading. 
 * 
 * The 00592TX usually starts the data sync bits right after
 * the RF sync pulses which are random length and polarity.
 *
 * The 00592TX first emits a random length string of 
 * random width hi/lo pulses, most like to provide radio
 * radio AGC synchronization.
 *
 * The probe then emits 4 data sync pulses of approximately 50% 
 * duty cycle and 1.2 ms period. The sync pulses start with a 
 * high level and continue for 4 high / low pulses.
 *
 * The data bits immediately follow the fourth low of the data
 * sync pulses. Data bits are sent every ~0.61 msec as:
 *
 * 1 bit ~0.4 msec high followed by ~0.2 msec low
 * 0 bit ~0.2 msec high followed by ~0.4 msec low
 *
 * The 00592TX sends the 4 sync pulses followed by
 * 7 bytes of data equalling 56 bits.
 *
 * The code below works by receiving a level change interrupt 
 * on each changing edge of the data stream from the RF module
 * and recording the time in uSec between each edge.
 *
 * 8 measured hi and lo pulses in a row, 4 high and 4 low, of 
 * approximately 600 uSec each constitue a sync stream.
 *
 * The remaining 56 bits of data, or 112 edges, are measured
 * and converted to 1s and 0s by checking the high to low
 * pulse times.
 *
 * The first 4 pulses, or 8 edges, are the sync pulses followed
 * by the 56 bits, or 112 edges, of the data pulses.
 *
 * We measure 8 sync edges followed by 112 data edges so the 
 * time capture buffer needs to be at least 120 bytes long.
 *
 * The data stream is 7 bytes long.
 * The first and second bytes are unique address bytes per probe.
 *   The upper two bits of the first byte are the probe channel indicator:
 *   
 *   11 = channel A   --> Refrigerator
 *   10 = channel B   --> Freezer
 *   00 = channel C   --> Extra
 *   
 *   The remaining 6 bits of the first byte and the 8 bits of the second
 *   byte are a unique identifier per device. If you need more that 3 
 *   adding a check by serial number could expand this gateway.
 *   
 * The next byte is a status byte, normal = 0x44,
 *   0x84 if battery is low. Vbatt < ~2.5v is low battery
 *   
 * The next byte is humidity and is encoded as the
 *   lower 7 bits
 *   
 * The next two bytes are the temperature. The temperature is encoded as the
 *   lower 7 bits of both bytes with the most significant bit being an
 *   even parity bit.  The MSB will be set if required to insure an even
 *   number of bits are set to 1 in the byte. If the least significant
 *   seven bits have an even number of 1 bits set the MSB will be 0,
 *   otherwise the MSB will be set to 1 to insure an even number of bits.
 *   
 *   Temperature Range: -40º to 158º F, -40º C to 70º C
 *   Humidity Range: 1% to 99% Relative Humidity
 *   
 * The last byte is a simple running sum, modulo 256, of the previous 6 data bytes.
 * 
 * The block of data is sent 3 time, we only decode one of the blocks
 *
 *  
 *  MQTT
 *    A message sent to this device by topic: SUBSCRIBE_TOPIC, with a "R"
 *     in the 1st byte will reset all Min/Max settings
 *     
 *    Sensor data is sent to MQTT server if using a ESP32 or ESP8266 processor
 *     
 *  MQTT Data Sent:
 *    Temperature, Min, Max, Humidity and Battery Status for the devices
 *     Alarms for temperature and Low Battery
 *    
 *  E-Mail:
 *    If enable, alarms are also send via E-mail or SMS
 *     http://www.emailtextmessages.com/
 *  
 *  Integration time for alarms can be set for each sensor.     
 *  
 *  Radio:
 *   Using an RXB6 or equivalent, connect to 3.3v, gnd and connect dataout
 *    to interrupt pin on CPU.
 *    
 *   RFM69 connect DIO-2 to interrupt pin on CPU.
 *    
 *   Antenna is 17.2cm long at 433MHz
 *  
 * *********************************************************************
 * Ideas on decoding protocol and prototype code from
 * Ray Wang (Rayshobby LLC) http://rayshobby.net/?p=8998
 *
 * Code based on Ray Wang's humidity _display.ino source.
 *
 * *********************************************************************
 * 
 CHANGE LOG:
 *
 *  DATE         REV  DESCRIPTION
 *  -----------  ---  ----------------------------------------------------------
 *  13-Apr-2018 1.0g  TRL - First Build
 *  14-Apr-2018 1.0h  TRL - Now you can have MQTT or E-mail, or both
 *  
 *  Notes:  1)  Tested with Arduino 1.8.5
 *          2)  Testing with a 433Mhz RFM69 
 *                RFM69OOK lib from https://github.com/kobuki/RFM69OOK
 *                DIO2 connected to pin interrupt pin.
 *          3)  Tested with a RXB6 and Aurel RX-MID receivers
 *          4)  Tested using a TTGO R1 ESP32 module
 *          5)  ESP32 and ESP8266 supported sending data via MQTT and E-Mail
 *          6)  ESP8266 tested with a NodeMCU 1.0
 *          7)  Added E-mail-SMS Support NOTE:You must edit Gsender.h with your E-mail info
 *          8)
 *          
 *  Todo:   1) Fix issues with RFM69 receiver, work in progress, not working
 *          2) 
 *          3) 
 *          4) 
 *          5) 
 * 
 * Tom Lafleur --> tom@lafleur.us
 * 
 */
~~~
