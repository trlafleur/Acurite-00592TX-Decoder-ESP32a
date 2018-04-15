
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
 * The next byte seems to always be 0x44, for all of
 *   the probes I have tested (a sample of 6 probes).
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
 *     Alarms for emperature and Low Battery
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

 /* ************************************************************* */
#define VERBOSE_OUTPUT
//#define DISPLAY_BIT_TIMING
//#define DISPLAY_DATA_BYTES
//#define MyDEBUG
#define IF_MQTT
#define IF_EMAIL
//#define RFM69

#define SKETCHNAME    "Started, Acu-Rite 00592TX Decoder, "
#define SKETCHVERSION "Ver: 1.0h"

#define OLED U8X8_SSD1306_128X64_NONAME_HW_I2C                // OLED-Display on board

// On the Arduino connect the data pin, the pin that will be 
// toggling with the incomming data from the RF module, to
// a pin that can be configured for interrupt 
// on change, change to high or low.

#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>         // http://arduino.cc/en/Reference/Wire ??
#include "Gsender.h"
#include "MovingAverage.h"
#include <PubSubClient.h>

// OLED Display 
#include <U8g2lib.h>      // https://github.com/olikraus/u8g2

#ifdef RFM69
  #include <RFM69OOK.h>
  #include <SPI.h>
  #include <RFM69OOKregisters.h>
  RFM69OOK radio;
#endif

/* ************************************************************* */
// Select processor includes
#ifdef ARDUINO_ARCH_ESP32
  #include <WiFi.h>
  #include <esp_wps.h>
  #include <WiFiClientSecure.h>
#endif

#ifdef ARDUINO_ARCH_ESP8266
  #include <ESP8266WiFi.h>
  #include <WiFiClientSecure.h>
#endif

// Ring buffer size has to be large enough to fit
// data and sync signal, at least 120
// round up to 128 for now
#define RING_BUFFER_SIZE  128

#define SYNC_HIGH       600
#define SYNC_LOW        600
#define SYNC_TOLL       100           // +- Tolerance for sync pulse

#define PULSE_LONG      400
#define PULSE_SHORT     220
#define PULSE_TOLL      100           // +- Tolerance for bit timming

#define BIT1_HIGH       PULSE_LONG
#define BIT1_LOW        PULSE_SHORT
#define BIT0_HIGH       PULSE_SHORT
#define BIT0_LOW        PULSE_LONG

#define PULSE_SHORT_NOISE     PULSE_SHORT - PULSE_TOLL      // anything shorter that this is noise
#define PULSE_LONG_NOISE      SYNC_HIGH + SYNC_TOLL         // anything longer that this is noise

// create an instance of WiFi client 
  WiFiClient espClient;

  // WiFi information
  // change it with your ssid-password
  const char* ssid = "MySSID";                    // <----------- Change This
  const char* password = "MyPass";                // <----------- Change This
    
#ifdef IF_EMAIL
   const char* MySendToAddress = "MyEmail";                                            // <----------- Change This for E-Mail
// const char* MySendToAddress = "MyPhone@vtext.com";                                  // <----------- Change This for SMS
//      For SMS format, see -->   http://www.emailtextmessages.com/
#endif

  uint8_t connection_state = 0;                    // Connected to WIFI or not
  uint16_t reconnect_interval = 10000;             // If not connected wait time to try again
  
    // MQTT Server IP Address or FQDN
const char* mqtt_server = "192.168.167.32";         // <----------- Change This

#ifdef IF_MQTT
// create an instance of PubSubClient
    PubSubClient client(espClient);
#else
    // create a null instance of PubSubClient
    PubSubClient client;
#endif

    // My topics, format header is RSF=Home, S1=Sensor number
    #define ATEMP_TOPIC     "RSF/S1/A/Temp"                                           // <----------- Change These as needed
    #define BTEMP_TOPIC     "RSF/S1/B/Temp"
    #define CTEMP_TOPIC     "RSF/S1/C/Temp"

    #define AHUM_TOPIC      "RSF/S1/A/Hum"                       
    #define BHUM_TOPIC      "RSF/S1/B/Hum"
    #define CHUM_TOPIC      "RSF/S1/C/Hum"
    
    #define ABATT_TOPIC     "RSF/S1/A/BATT"
    #define BBATT_TOPIC     "RSF/S1/B/BATT"
    #define CBATT_TOPIC     "RSF/S1/C/BATT"
    
    #define AALARM_TOPIC    "RSF/S1/A/ALARM"
    #define BALARM_TOPIC    "RSF/S1/B/ALARM"
    #define CALARM_TOPIC    "RSF/S1/C/ALARM"
    
    #define BattALARM_TOPIC "RSF/S1/BATT/ALARM"
    
    #define AMIN_TOPIC      "RSF/S1/A/MIN"
    #define AMAX_TOPIC      "RSF/S1/A/MAX"
    #define BMIN_TOPIC      "RSF/S1/B/MIN"
    #define BMAX_TOPIC      "RSF/S1/B/MAX"
    #define CMIN_TOPIC      "RSF/S1/C/MIN"
    #define CMAX_TOPIC      "RSF/S1/C/MAX"
    
    #define SUBSCRIBE_TOPIC "RSF/S1/+/RESET"

// create an instance for Moving Average
  MovingAverage <float> AAVD (60);        // create a moving average over last n values         // <----------- Change These as needed
  MovingAverage <float> BAVD (60);        // 60 * ~18  sec = 1080sec = 18min
  MovingAverage <float> CAVD (60);

  #define MAX_ATEMP     45.0                // Max temperature for refrigerator, device A       // <----------- Change Theses as needed
  #define MAX_BTEMP     20.0                // Max temperature for freezer, device B
  #define MAX_CTEMP     99.0                // Max temperature for device C
    
  #define AlarmTimeToWait          120L             // Wait this amount of time for next alarm message, in Minutes  // <----------- Change These as needed
  #define BattAlarmTimeToWait     1440L             // Wait this amount of time for next battery alarm message, in Minutes
  
  unsigned long LastTimeA  = 0;                     // This is ID: A, used for the refrigerator
  unsigned long LastTimeB  = 0;                     // This is ID: B, used for the freezer
  unsigned long LastTimeC  = 0;                     // This is ID: C, Extra unit
  unsigned long LastTimeBatt = 0;                   // Low battery
  
  bool A_Flag = false;                              // This is ID: A, used for the refrigerator
  bool B_Flag = false;                              // This is ID: B, used for the freezer
  bool C_Flag = false;                              // This is ID: C, Extra unit
  bool Batt_Flag = false;                           // This is low battery flag
  
  unsigned long currentMillis = 0;                  // a 1 Minute clock timer
  unsigned long interval = 60000;                   // = 60 sec --> 1 Minure
  unsigned long previousMillis = 0;
  unsigned long Minute = 0;
  
  unsigned long BlockFailCounter  = 0;
  unsigned long CSFailCounter    = 0;
  
  char msg [60];                              // char string buffer
  char msg1[60];                              // char string buffer

/* ************************************************************* */
#define SYNCPULSECNT      4                   // 4 sync pulses (8 edges)
#define SYNCPULSEEDGES    (SYNCPULSECNT * 2 )

#define DATABYTESCNT      7                   // Number of bytes to look for 
#define DATABITSCNT       (DATABYTESCNT * 8)  // Number of bits to look for
#define DATABITSEDGES     (DATABITSCNT * 2)   // Number of edges to look for

// The pulse durations are measured in micro seconds between pulse edges.
unsigned long pulseDurations[RING_BUFFER_SIZE];   // where we store the pulse edges
unsigned int syncIndex  = 0;                  // index of the last bit time of the sync signal
unsigned int dataIndex  = 0;                  // index of the first bit time of the data bits (syncIndex+1)
bool         syncFound = false;               // true if sync pulses found
bool         received  = false;               // true if enough sync pulses bits are found
unsigned int changeCount = 0;                 // Count of pulses edges

unsigned char dataBytes[DATABYTESCNT];        // Decoded data storage
unsigned long mytime = 0;                     // event time
float temp = 0;                               // temperature
int   hum  = 0;

// The Min-Max for each sensor
float AMinTemp = 158;                         // Max temp is 158deg
float AMaxTemp = -40;
float BMinTemp = 158;                         // Max temp is 158deg
float BMaxTemp = -40;
float CMinTemp = 158;                         // Max temp is 158deg
float CMaxTemp = -40;


/* ************************************************************* */
#ifdef ARDUINO_ARCH_ESP32
  /* pin that is attached to interrupt */
  #define DATAPIN          12                 // interrupt pin
  byte interruptPin = DATAPIN;
  #define MyInterrupt (digitalPinToInterrupt(interruptPin))
  
  #define MyLED             2
  
  // define below are use in debug as trigers to logic analyzer
  #define MySync            36                // Trigger on Sync found
  #define MyBit             37                // Trigger on bit edge
  #define MyFrame           38                // Trigger at end of frame

  
  // Hardware pin definitions for TTGOv1 Board with OLED SSD1306 I2C Display
  #define OLED_RST 16         // ESP32 GPIO16 (Pin16) -- SD1306 Reset
  #define OLED_SCL 15         // ESP32 GPIO15 (Pin15) -- SD1306 Clock
  #define OLED_SDA 4          // ESP32 GPIO4  (Pin4)  -- SD1306 Data


/* ************************************************************* */
#elif ARDUINO_ARCH_ESP8266

  #define DATAPIN            2                // 2 is interrupt
  byte interruptPin = DATAPIN;
  #define MyInterrupt (digitalPinToInterrupt(interruptPin))
 
  // Note: their are two LED on the NodeMCU Rev1 board
  // D0-->16 on the board and D4-->2 on the ESP12 that is connected to U1-TXD
  #define MyLED             16
  
  // define below are use in debug as trigers to logic analyzer
  #define MySync            3                // Trigger on Sync found
  #define MyBit             4                // Trigger on bit edge
  #define MyFrame           5                // Trigger at end of frame

  
// Hardware pin definitions for OLED SSD1306 I2C Display
//  #define OLED_RST 16         // ESP32 GPIO16 (Pin16) -- SD1306 Reset
//  #define OLED_SCL 15         // ESP32 GPIO15 (Pin15) -- SD1306 Clock
//  #define OLED_SDA 4          // ESP32 GPIO4  (Pin4)  -- SD1306 Data

  
#else
  #error CPU undefined.....
#endif

// create an instance for OLED Display
#ifdef OLED
    OLED u8x8(OLED_RST, OLED_SCL, OLED_SDA);
#else
   U8X8_NULL u8x8;
#endif


/* ************************************************************* */
#ifdef OLED
void init_display(void) 
{
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.clear();
    u8x8.setFlipMode(1);
}
#endif // OLED


/* ************************************************************* */
  void receivedCallback(char* topic, byte* payload, unsigned int length) 
  {
    Serial.println("Message received: ");
    Serial.println(topic);
  
    Serial.print("payload: ");
    for (int i = 0; i < length; i++) 
    {
      Serial.print((char)payload[i]);
    }
    Serial.println();
    if ((char)payload[0] == 'R') 
    {
      Serial.println("Reset Received");
      // reset the Min-Max for each sensor
      AMinTemp = 158;                         // Max temp is 158deg
      AMaxTemp = -40;
      BMinTemp = 158;                         // Max temp is 158deg
      BMaxTemp = -40;
      CMinTemp = 158;                         // Max temp is 158deg
      CMaxTemp = -40;
    }
  }


/* ************************************************************* */
  void mqttconnect() 
{
    /* Loop until reconnected */
    while (!client.connected()) 
    {
      Serial.println();
      Serial.print("MQTT connecting to: ");
      Serial.println (mqtt_server);
      
      /* client ID */
       String clientId = WiFi.macAddress();                // use our MAC address as MQTT Client ID
   
      /* connect now */
      if (client.connect(clientId.c_str()))                  
      {
        Serial.print("MQTT connected, Client ID: ");
        Serial.println(clientId);
        /* subscribe topic with default QoS 0*/
       client.subscribe(SUBSCRIBE_TOPIC);
      } else 
      {
        Serial.print("failed, status code =");
        Serial.print(client.state());
        Serial.println("try again in 5 seconds");
        /* Wait 5 seconds before retrying */
        delay(5000);
      }
    }   // End of: while (!client.connected()) 

  }   // End of:  mqttconnect() 


/* ************************************************************* */
uint8_t WiFiConnect(const char* nSSID = nullptr, const char* nPassword = nullptr)
{
    static uint32_t attempt = 0;
   
   WiFi.disconnect(); 
   Serial.print("Connecting WiFi to: ");
   if(nSSID) {
       WiFi.begin(nSSID, nPassword);  
       Serial.println(nSSID);
   } else {
       WiFi.begin(ssid, password);
       Serial.println(ssid);
   }

   uint8_t i = 0;
   while( (WiFi.status()!= WL_CONNECTED) && (i++ < 50) )     // wait for a connection
   {
       delay(250);
       Serial.print(".");
   }
   
   ++attempt;

   if (attempt >= 15)                                     // well, we have a problem, lets reboot...
    {
      Serial.println ("Unable to connect to WiFi, Rebooting...");
           ESP.restart();     // <---------------- experiment
    }
   Serial.println("");
     if(i >= 51)                                          // if we can't make a connection
     {
       Serial.print("Connection: TIMEOUT on attempt: ");
       Serial.println(attempt);
       WiFi.disconnect(); 
       return false;
     }
   Serial.println("Connection: ESTABLISHED");             // Ok, all is good, we have a connection
   Serial.print  ("Got IP address: ");
   Serial.println(WiFi.localIP());
   return true;
}


/* ************************************************************* */
void Awaits()
{
   uint32_t ts = millis();
   while(!connection_state)
   {
       delay(200);
       if(millis() > (ts + reconnect_interval) && !connection_state)
       {
           connection_state = WiFiConnect();
           ts = millis();
       }
   }
}


 /* ************************************************************* */
 /*
 *    Will print 8-bit formatted hex
 */
void PrintHex8(uint8_t *data, uint8_t length) 
{
   char tmp[length*2+1];
   byte first;
   int j = 0;
   for (uint8_t i = 0; i < length; i++) 
     {
       first = (data[i] >> 4) | 48;
       if (first > 57) tmp[j] = first + (byte)39;
       else tmp[j] = first ;
       j++;
    
       first = (data[i] & 0x0F) | 48;
       if (first > 57) tmp[j] = first + (byte)39; 
       else tmp[j] = first;
       j++;
     }
   tmp[length*2] = 0;
   Serial.print("0x");
   Serial.print(tmp);
}


//* ************************************************************* */
// Prints a byte as binary with leading zero's
void printBits(byte myByte)
{
   for(byte mask = 0x80; mask; mask >>= 1)
   {
     if(mask  & myByte)
         Serial.print('1');
     else
         Serial.print('0');
   }
}


//* ************************************************************* */
//    Checksum of bits
uint8_t CheckSum(uint8_t const message[], unsigned nBytes) 
{
    unsigned int sum = 0;
    unsigned i;  
    for (i = 0; i <= nBytes; ++i) 
    {
      sum = sum + message[i];
    }
    sum = (sum & 0x000000ff);  
    return ((uint8_t) sum );
}


/* ************************************************************* */
/*
 * Look for the sync pulse train of 4 high-low pulses of
 * 600 uS high and 600 uS low.
 * idx is index of last captured bit duration.
 * Search backwards 8 times looking for 4 pulses
 * approximately 600uS long.
 *
 */
bool isSync(unsigned int idx) 
{
   // check if we've received 4 sync pulses of correct timing
   for( int i = 0; i < SYNCPULSEEDGES; i += 2 )
   {
      unsigned long t1 = pulseDurations[(idx+RING_BUFFER_SIZE-i) % RING_BUFFER_SIZE];
      unsigned long t0 = pulseDurations[(idx+RING_BUFFER_SIZE-i-1) % RING_BUFFER_SIZE];      
      
      // If any of the preceeding 8 pulses are out of bounds, short or long,
      // return false, no sync found
      if( t0<(SYNC_HIGH-SYNC_TOLL) || t0>(SYNC_HIGH+SYNC_TOLL) || t1<(SYNC_LOW-SYNC_TOLL)  || t1>(SYNC_LOW+SYNC_TOLL) )
              { return false; }
    }
   return true;
}


/* ************************************************************* */
/* Interrupt  handler 
 * Set to interrupt on edge (level change) high or low transition.
 * Change the state of the Arduino LED on each interrupt. 
 */
void interrupt_handler() 
{
   volatile static unsigned long duration = 0;
   volatile static unsigned long lastTime = 0;
   volatile static unsigned int ringIndex = 0;
   volatile static unsigned int syncCount = 0;
   volatile static unsigned int bitState  = 0;

   // Ignore this interrupt if we haven't finished processing the previous 
   // received block in the main loop.
   if( received == true ) {return;}                     // return, we are not finish with processor last block

   bitState = digitalRead (DATAPIN);
   digitalWrite(MyLED, bitState);                       // LED to show receiver activity

   // calculating timing since last change
   long time = micros();
   duration = time - lastTime;
   lastTime = time;

   // Known errors in bit stream are runt's --> short and long pulses.
   // If we ever get a really short, or really long 
   // pulse's we know there is an error in the bit stream
   // and should start over.
   if ( (duration > (PULSE_LONG_NOISE)) || (duration < (PULSE_SHORT_NOISE)) )    // This pulse must be noise...   
   {
      received = false;
      syncFound = false;
      changeCount = 0;                                  // restart, start looking for sync and data bits again
   }

   // if we have good data, store data in ring buffer
   ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
   pulseDurations[ringIndex] = duration;
   changeCount++;                                       // found another edge

#ifdef MyDEBUG
      digitalWrite(MyBit, !digitalRead(MyBit) );        // LED to show we have a bit
     // digitalWrite (MyBit, LOW);          
     // delayMicroseconds (5);
     // digitalWrite (MyBit, HIGH);
#endif

   // detected sync signal
   if( isSync (ringIndex) )                              // check for sync on each bit received
   {
      syncFound = true;
      changeCount = 0;                                   // lets restart looking for data bits again
      syncIndex = ringIndex;
      dataIndex = (syncIndex + 1) % RING_BUFFER_SIZE;

#ifdef MyDEBUG
      digitalWrite(MySync, !digitalRead(MySync) );        // LED to show we have sync
//      digitalWrite (MySync, LOW);          
//      delayMicroseconds (5);
//      digitalWrite (MySync, HIGH);
#endif    
    }
    
   // If a sync has been found, then start looking for the
   //  data bit edges in DATABITSEDGES
   if( syncFound )
   {       
      // not enough bits yet?, so no full message block has been received yet
      if( changeCount < DATABITSEDGES )            
        { received = false; }
      
      else 
      
      if( changeCount >= DATABITSEDGES )            // check for too many bits
        {      
          changeCount = DATABITSEDGES;              // lets keep bits we have, checksum will kill this block if bad
          detachInterrupt(MyInterrupt);             // disable interrupt to avoid new data corrupting the buffer
          received = true;   
        }
           
#ifdef MyDEBUG
        digitalWrite(MyFrame, !digitalRead(MyFrame) );         // LED to show that we have full block of data
//        digitalWrite (MyFrame, LOW); 
//        delayMicroseconds (50);
//        digitalWrite (MyFrame, HIGHƒ);
#endif  

    }    // end of if syncFound
}    // end of interrupt_handler


const char compile_date[]  = __DATE__ ", " __TIME__;

/* ************************************************************* */
/* ************************************************************* */
/* ************************************************************* */
void setup()
{
   Serial.begin(115200);
   delay(2000);
   
   Serial.println("");
   Serial.print(SKETCHNAME);
#ifdef RFM69
   Serial.println("RFM69");
#else
   Serial.println("External Receiver");
#endif
  Serial.println (SKETCHVERSION);
  Serial.println (compile_date);
  Serial.println("");
 
   pinMode(DATAPIN, INPUT);              // data interrupt pin set for input
   
   pinMode(MyLED, OUTPUT);               // LED output
   digitalWrite (MyLED, LOW);

#ifdef MyDEBUG
   pinMode(MySync, OUTPUT);              // sync bit output
   digitalWrite (MySync, LOW);
   
   pinMode(MyBit, OUTPUT);               // data bit output
   digitalWrite (MyBit, LOW);
   
   pinMode(MyFrame, OUTPUT);             // end of frame bit output
   digitalWrite (MyFrame, LOW);
#endif


#ifdef RFM69
    pinMode( 14, INPUT);        // RFM69 RST
    //digitalWrite (14, LOW);
    pinMode(26, INPUT);         // DIO-0
    pinMode(33, INPUT);         // DIO-1
    pinMode(32, INPUT);         // DIO-2  This is where we get the RX data --> comnnected to interrupt pin
    radio.initialize();
    //radio.setBandwidth(OOK_BW_10_4);
    radio.setRSSIThreshold(-70);
    radio.setFixedThreshold(20);
    radio.setSensitivityBoost(SENSITIVITY_BOOST_HIGH);
    radio.setFrequencyMHz(433.92);
    radio.receiveBegin();
#endif

// Setup WiFi
// WiFi.config(ip, dns, gateway, subnet);   // if using static addressing

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  connection_state = WiFiConnect();

  if(!connection_state)                     // if not connected to WIFI
       Awaits();                            // constantly trying to connect

    delay (1000);

  /* configure the MQTT server with IPaddress and port */
  client.setServer(mqtt_server, 1883);
  
  /* this receivedCallback function will be invoked 
  when client received subscribed topic */
  client.setCallback(receivedCallback);
  

#ifdef OLED
// initialize the OLED display  
    init_display();     
    u8x8.drawString(0, 0,"00592 Decoder");
    u8x8.setCursor(0,1);
    u8x8.printf("%d.%d.%d.%d",WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
#endif

   pinMode(MyInterrupt, INPUT_PULLUP);
   attachInterrupt(MyInterrupt, interrupt_handler, CHANGE);
   
}   // end of setup


/* ************************************************************* */
/*
 * Convert pulse durations to bits.
 * 
 * 1 bit ~0.4 msec high followed by ~0.2 msec low
 * 0 bit ~0.2 msec high followed by ~0.4 msec low
 * 
 */
int convertTimingToBit(unsigned int t0, unsigned int t1) 
{
   if( t0 > (BIT1_HIGH-PULSE_TOLL) && t0 < (BIT1_HIGH+PULSE_TOLL) && t1 > (BIT1_LOW-PULSE_TOLL) && t1 < (BIT1_LOW+PULSE_TOLL) )
      { return 1; }
   else if( t0 > (BIT0_HIGH-PULSE_TOLL) && t0 < (BIT0_HIGH+PULSE_TOLL) && t1 > (BIT0_LOW-PULSE_TOLL) && t1 < (BIT0_LOW+PULSE_TOLL) )
      { return 0; }
   return -1;                   // error, if undefined bit timimg
}


/* ************************************************************* */
// 00592TX send's a meassge every ~18 sec, so lets average temperature
// over a number of sample, if is greater that our alarm settings, we need to send
// an alarm, but only once every so many minutes.
void MaxSensorAAlarm (float temp)
{ 
  if (A_Flag == false)                                      // see if this is 1st time here for this alarm...
   { 
      snprintf (msg, 10, "%6.2f", temp);
      client.publish (AALARM_TOPIC, msg);

#ifdef IF_EMAIL
     Gsender *gsender = Gsender::Instance();                // Getting pointer to class instance
     String subject = "RSF Sensor Alarm! ID=A";
     sprintf (msg, "Alarm set at: %6.2fF,  Temperature is: %6.2fF\n", MAX_ATEMP, temp);
     if(gsender->Subject(subject)->Send(MySendToAddress, msg)) 
     {
         Serial.println("E-mail Message sent.");
     } else 
     {
         Serial.print("Error, sending message: ");
         Serial.println(gsender->getError());
     }

#endif
      Serial.println ("Sensor Alarm, ID: A"); 
          
      A_Flag = true;
      LastTimeA = Minute;                                   // save the current time
   }
  else
  {
    if ( Minute >= (LastTimeA + AlarmTimeToWait ) )         // see if it time to re-send alarm
      { A_Flag = false; }                                   // Yes, reset alarm flag
  }
}


/* ************************************************************* */
void MaxSensorBAlarm(float temp)
{
  if (B_Flag == false)                                      // see if this is 1st time here for this alarm...
   {
      snprintf (msg, 10, "%6.2f", temp);
      client.publish (BALARM_TOPIC, msg);

#ifdef IF_EMAIL
     Gsender *gsender = Gsender::Instance();                // Getting pointer to class instance
     String subject = "RSF Sensor Alarm! ID=B";
     sprintf (msg,  "Alarm set at: %6.2fF,  Temperature is: %6.2fF\n", MAX_BTEMP, temp);
     if(gsender->Subject(subject)->Send(MySendToAddress, msg)) 
     {
         Serial.println("E-Mail Message sent.");
     } else 
     {
         Serial.print("Error sending message: ");
         Serial.println(gsender->getError());
     }

#endif
      Serial.println ("Sensor Alarm, ID: B");    
      B_Flag = true;
      LastTimeB = Minute;                                    // save the current time
   }
  else
  {
    if ( Minute >= (LastTimeB + AlarmTimeToWait ) )         // see if it time to re-send alarm
      { B_Flag = false; }                                   // Yes, reset alarm flag
  }
}


/* ************************************************************* */
void MaxSensorCAlarm(float temp)
{
  if (C_Flag == false)                                      // see if this is 1st time here for this alarm...
   {
      snprintf (msg, 10, "%6.2f", temp);
      client.publish (CALARM_TOPIC, msg);

#ifdef IF_EMAIL
     Gsender *gsender = Gsender::Instance();                // Getting pointer to class instance
     String subject = "RSF Sensor Alarm! ID=C";
     sprintf (msg, "Alarm set at: %6.2fF,  Temperature is: %6.2fF\n", MAX_CTEMP, temp);
     if(gsender->Subject(subject)->Send(MySendToAddress, msg)) 
     {
         Serial.println("E-Mail Message sent.");
     } else 
     {
         Serial.print("Error sending message: ");
         Serial.println(gsender->getError());
     }

#endif
      Serial.println ("Sensor Alarm, ID: C");    
      C_Flag = true;
      LastTimeC = Minute;                                   // save the current time
   }
  else
  {
    if ( Minute >= (LastTimeC + AlarmTimeToWait ) )         // see if it time to re-send alarm
      { C_Flag = false; }                                   // Yes, reset alarm flag
  }
}

/* ************************************************************* */
void BatteryLowAlarm (int device)
{
  device = (device & 0x02);                                 // only two bits are used
  if (Batt_Flag == false)                                   // see if this is first time here for this alarm...
   {                                                        // we should do this for each sensor, but maybe next time
      if (device == 0x03) { snprintf (msg, 50, "Battery Low, Sensor: A"); }
      if (device == 0x02) { snprintf (msg, 50, "Battery Low, Sensor: B"); }
      if (device == 0x01) { snprintf (msg, 50, "Battery Low, Sensor: Unknown"); } 
      if (device == 0x00) { snprintf (msg, 50, "Battery Low, Sensor: C"); }
        Serial.println (msg); 
        
      client.publish (BattALARM_TOPIC, msg);


#ifdef IF_EMAIL
     Gsender *gsender = Gsender::Instance();    // Getting pointer to class instance
     
     String subject = "RSF Low Battery Alarm!";
     
     // We will get message body from msg buffer above
     if(gsender->Subject(subject)->Send(MySendToAddress, msg)) 
       { 
           Serial.println("E-Mail Message sent.");
       } 
     else 
       {
           Serial.print("Error sending message: ");
           Serial.println(gsender->getError());
       }
#endif
      Batt_Flag = true;
      LastTimeBatt = Minute;                    // save the current time
   }
  else
    {
      if ( Minute >= (LastTimeBatt + BattAlarmTimeToWait ) )  // see if it time to re-send alarm
            { Batt_Flag = false; }                               // Yes, reset alarm flag
    }
}


/* ************************************************************* */
int acurite_getHumidity (uint8_t byte) 
{
    // range: 1 to 99 %RH
    int humidity = byte & 0x7F;
    return humidity;
}


/* ************************************************************* */
float acurite_getTemp_6044M(byte hibyte, byte lobyte) 
{
  // range -40 to 158 F, -40º C to 70º C --> returns temp in deg C
  int highbits = (hibyte & 0x0F) << 7;
  int lowbits = lobyte & 0x7F;
  int rawtemp = highbits | lowbits;
  float temp = (rawtemp / 10.0) - 100;
  return temp;
}


/* ************************************************************* */
float convCF(float c) 
{
  return ((c * 1.8) + 32);
}


/* ************************************************************* */
uint16_t acurite_txr_getSensorId(uint8_t byte)
{
    return ((byte & 0xc0) >> 6);
}

/* ************************************************************* */
uint16_t acurite_txr_getSensorSN(uint8_t hibyte, uint8_t lobyte)
{
    return ((hibyte & 0x3f) << 8) | lobyte;
}


/* ************************************************************* */
bool acurite_txr_getBattery(uint8_t byte)
{
  if ((dataBytes[byte] & 0x20) == 0x20 )    // check if battery is low
  { return true; }
  return false;
}

/* ************************************************************* */
void MQTT_Send (void)
{
// Send sensor ID --> A-B-C temperature, humidity, battery status, and alarm status to MQTT server

           temp = convCF (acurite_getTemp_6044M(dataBytes[4], dataBytes[5]));    // Get sensor values, and convert from C to F
           hum = acurite_getHumidity (dataBytes[3]);
           
           byte ID = acurite_txr_getSensorId(dataBytes[0]);           // Only a 2 bit ID
           
           bool Battery = acurite_txr_getBattery(dataBytes[4]);
           
           snprintf (msg,  10, "%6.2f", temp);                         // format temperature message
           snprintf (msg1, 10, "%d", hum);                             // format humidity message
           
           switch (ID)
           {
           case 0x03: // Sensor A
             {
                if (temp > AMaxTemp) {AMaxTemp = temp;}                     // lets set new Min-Max
                if (temp < AMinTemp) {AMinTemp = temp;}
                
                client.publish (ATEMP_TOPIC, msg);
                client.publish (AHUM_TOPIC, msg1);
                snprintf (msg, 10, "%6.2f", AMinTemp);
                client.publish (AMIN_TOPIC, msg);                           // send min temperature              
                snprintf (msg, 10, "%6.2f", AMaxTemp);
                client.publish (AMAX_TOPIC, msg);                           // send max temperature
                 
                if (Battery) {client.publish (ABATT_TOPIC, "Low Battery"); BatteryLowAlarm (ID);}
    
                float intTemp = AAVD.CalculateMovingAverage(temp);          // add temp to average calculator
                if (intTemp >= MAX_ATEMP)  { MaxSensorAAlarm(intTemp); }    // do we have an alarm? Yes
                  else { A_Flag = false; }                                  // no alarm now
                  
                u8x8.setCursor(0,3);
                u8x8.clearLine(3);
                u8x8.printf("Sensor A:%6.2fF",temp );
                break;
             }
           
           case 0x02:   // Sensor B
             {
                if (temp > BMaxTemp) {BMaxTemp = temp;}                     // lets set new Min-Max
                if (temp < BMinTemp) {BMinTemp = temp;}
                
                client.publish (BTEMP_TOPIC, msg);
                client.publish (BHUM_TOPIC, msg1);
                snprintf (msg, 10, "%6.2f", BMinTemp);
                client.publish (BMIN_TOPIC, msg);                           // send min temperature              
                snprintf (msg, 10, "%6.2f", BMaxTemp);
                client.publish (BMAX_TOPIC, msg);                           // send max temperature
                 
                if (Battery) {client.publish (BBATT_TOPIC, "Low Battery"); BatteryLowAlarm (ID);}
                
                float intTemp = BAVD.CalculateMovingAverage(temp);          // add temp to average calculator
                if (intTemp >= MAX_BTEMP) { MaxSensorBAlarm(intTemp); }     // do we have an alarm? Yes
                    else { B_Flag = false; }                                // no alarm now
                    
                u8x8.setCursor(0,5);
                u8x8.clearLine(5);
                u8x8.printf("Sensor B:%6.2fF",temp ); 
                break;        
             }
           
           case 0x00:   // Sensor C
           {
              if (temp > CMaxTemp) {CMaxTemp = temp;}                     // lets set new Min-Max
              if (temp < CMinTemp) {CMinTemp = temp;}
              
              client.publish (CTEMP_TOPIC, msg);
              client.publish (CHUM_TOPIC, msg1);
              snprintf (msg, 10, "%6.2f", CMinTemp);
              client.publish (CMIN_TOPIC, msg);                           // send min temperature              
              snprintf (msg, 10, "%6.2f", CMaxTemp);
              client.publish (CMAX_TOPIC, msg);                           // send max temperature
              
              if (Battery) {client.publish (CBATT_TOPIC, "Low Battery"); BatteryLowAlarm (ID);}
  
              float intTemp = CAVD.CalculateMovingAverage(temp);          // add temp to average calculator
              if (intTemp >= MAX_CTEMP)  { MaxSensorCAlarm(intTemp); }    // do we have an alarm? Yes
                else { C_Flag = false; }                                  // no alarm now
                
              u8x8.setCursor(0,7);
              u8x8.clearLine(7);
              u8x8.printf("Sensor C:%6.2fF",temp );
              break;
           } 

            default:
              {
                Serial.println ("***Got Sensor ID=0, Error");
                break;
              }
           }    // End of: switch...                    
}   // End of: MQTTSend


/* ************************************************************* */
// Also 00592TX, 06002M, 06044 and others....
void decode_Acurite_6044(byte dataBytes[])
{
  Serial.print("Acurite 06044, ID: ");
  
  byte ID = acurite_txr_getSensorId(dataBytes[0]);
    if ( ID == 0x3) Serial.print("A");
    if ( ID == 0x2) Serial.print("B");
    if ( ID == 0x0) Serial.print("C");
    
  Serial.print(", SN: 0x");
  Serial.print(acurite_txr_getSensorSN(dataBytes[0], dataBytes[1]), HEX);
  
  Serial.print("; Temp - ");
  Serial.print(convCF(acurite_getTemp_6044M(dataBytes[4], dataBytes[5])));
  
  Serial.print("F; Humidity - ");
  Serial.print(acurite_getHumidity(dataBytes[3]));
  
  Serial.print("%;");
    if (acurite_txr_getBattery(dataBytes[4]) ) { Serial.println(""); Serial.print("Battery Low"); }
  Serial.println();
}


/* ************************************************************* */
/* ************************************************************* */
/* ************************************************************* */
/*
 * Main Loop
 * Wait for received to be true, meaning a sync stream plus
 * all of the data bit edges have been found.
 * Convert all of the pulse timings to bits and calculate
 * the results.
 */
void loop()
{
#ifdef IF_MQTT
  /* if client was disconnected then try to reconnect again */
   if (!client.connected()) { mqttconnect(); }
   
  /* this function will listen for incomming MQTT
  subscribed topic-process-invoke receivedCallback */
   client.loop();
#endif

// lets setup a long duration timer at 1 minute tick
  currentMillis = millis ();                                  // get current time
  if (currentMillis - previousMillis >= interval)
        { previousMillis = currentMillis; Minute++; }         // add one to minute couter if time..

   if( received == true )                                     // check to see if we have a full block of bits to decode
   {
      // disable interrupt to avoid new data corrupting the buffer
      detachInterrupt(MyInterrupt);
      
      unsigned int startIndex, stopIndex, ringIndex;
      bool fail = false;

/* ************************************************************* */
// Print the bit stream for debugging. 
// Generates a lot of chatter, normally disable this. 
// T0 is the length of the up pulse, or pre-pulse, T1 is the down pulse, or data pulse   
#ifdef DISPLAY_BIT_TIMING
      Serial.println("");
      Serial.print("syncFound = ");
      Serial.println(syncFound);
      Serial.print("changeCount = ");
      Serial.println(changeCount);

      Serial.print("syncIndex = ");
      Serial.println(syncIndex);

      Serial.print("dataIndex = ");
      Serial.println(dataIndex);

      ringIndex = (syncIndex - (SYNCPULSEEDGES-1) ) % RING_BUFFER_SIZE;

      for ( int i = 0; i < (SYNCPULSECNT + DATABITSCNT); i++ )
      {
         int bit = convertTimingToBit( pulseDurations[ringIndex % RING_BUFFER_SIZE], 
                                       pulseDurations[(ringIndex + 1) % RING_BUFFER_SIZE] );
         Serial.print("bit ");
         Serial.print( i );
         Serial.print(" =  ");
         Serial.print(bit);
         Serial.print(", t1 = ");
         Serial.print(pulseDurations [ringIndex % RING_BUFFER_SIZE]);
         Serial.print(", t0 = ");
         Serial.println(pulseDurations [(ringIndex + 1) % RING_BUFFER_SIZE]);

         ringIndex += 2;
      }
#endif // endif of: DISPLAY_BIT_TIMING


/* ************************************************************* */
// Build a byte array with the raw data received

      fail = false;                                       // reset bit decode error flag

      // clear the data bytes array
      for( int i = 0; i < DATABYTESCNT; i++ )    { dataBytes[i] = 0; }
        
      ringIndex = (syncIndex +1 ) % RING_BUFFER_SIZE;

      for( int i = 0; i < DATABITSCNT; i++ )
      {
         int bit = convertTimingToBit ( pulseDurations[ringIndex % RING_BUFFER_SIZE], 
                                          pulseDurations[(ringIndex +1 ) % RING_BUFFER_SIZE] );                                                                           
         if( bit < 0 )                                    // check for a valid bit, ie: 1 or zero, -1 = error
           { fail = true; break; }                        // exit loop
         else
           { dataBytes[i/8] |= bit << ( 7 - (i % 8)); }   // pack into a byte       
         ringIndex += 2;
      }

#ifdef DISPLAY_DATA_BYTES

      if (fail )
        { Serial.println("Data Byte Display --> Decoding error."); }
      else
        {
          Serial.println("");
          for( int i = 0; i < DATABYTESCNT; i++ )
            {
              PrintHex8 (&dataBytes[i], 1);
              Serial.print(",");
            }
          Serial.print("  ");
  
          for( int i = 0; i < DATABYTESCNT; i++ )
            {
              printBits(dataBytes[i]);
              Serial.print(",");
            }
          Serial.println("");
        }

#endif  // end of: DISPLAY_DATA_BYTES


/* ************************************************************* */
// lets extract data from the sensor
// all data bytes are now in dataBytes[DATABYTESCNT]

        if (!fail)                                                  // if fail, we decoded some of the bits are wrong, so don't process this block
        {
          if ( CheckSum (dataBytes, 5) == dataBytes[6] )            // if Check Sum is good... 
          {                   
              Serial.print(Minute);                                 // Time stamp for display
              Serial.print(" ");   
              decode_Acurite_6044(dataBytes);                       // decode all of the bits in the block
             
#ifdef MyDEBUG
              Serial.print (Minute);         
              Serial.print (" Block Error Counter: ");              // Print CS and Block decode errors counters
              Serial.print (BlockFailCounter);
              Serial.print (", CS Error Counter: ");
              Serial.println (CSFailCounter);
              
#endif    //  End of MyDEBUG
                        
          // Send data to MQTT server and E-Mail for alerts
              MQTT_Send ();                                         // sending MQTT messages
        
            }   // End of: if ( CheckSum (dataBytes, 5) == dataBytes[6] )
          else  
            {              
#ifdef MyDEBUG
              Serial.print (Minute);                               // Print Check Sum information
              Serial.print ( " CS: is: ");
              Serial.print ( dataBytes[6], HEX );
              Serial.print ( " Should be: ");
              Serial.println ( CheckSum (dataBytes, 5) , HEX);     // I know, this is a waist of time to do this again...
#endif
              CSFailCounter++;                                    // if Check Sum is bad, keep count
              }
          
        }   // End of if (!fail)...
        
        else  { BlockFailCounter++; }                              // if block decode is bad, keep count
          
      received = false;
      syncFound = false;
      
      delay (250);                                                // this will eliminate 2nd and 3rd block of data if its sent
      
      // re-enable interrupt
      attachInterrupt (MyInterrupt, interrupt_handler, CHANGE);
      
   }    // end of:  if( received == true )

}   // end of: loop

// the very end......

