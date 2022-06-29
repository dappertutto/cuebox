#include <Adafruit_VL53L1X.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <vl53l1x_class.h>
#include <vl53l1x_error_codes.h>


#define IRQ_PIN 2
#define XSHUT_PIN 3

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

//Code designed for the Olimex ESP32-POE-ISO variant.
//Code by Luke Dzwonczyk, Jeremy Wagner & Olimex

// this version 2021 05 04 integrates Luke's /getIP and /getPort

#include <Arduino.h>
#include <ETH.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <LiquidCrystal_I2C.h>

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#define ETH_PHY_POWER 12
#define PIN_MAX_VALUE 4095

// pin lists

const int button_pin = 34;
const int RedledPin = 3;
const int YellowledPin = 33;
const int GreenledPin = 5;

// the OSC addresses that will be used

char osc_addr_button[8] = "/button";

/* below the screen display i2c address is 0x27. Some screens are 0x3F, and it's possible others are neither of these two addresses. There are i2c scanner programs to help identify the address if no luck */

// display vars
int lcdColumns = 20;
int lcdRows = 4;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);


bool in_light_phrase = false;
unsigned long light_phrase_start_time = -1;
int light_phrase_delay_time = -1;

// networking vars
WiFiUDP Udp;
unsigned int local_port = 5002;
IPAddress dest_ip;
unsigned int dest_port = 5003;

// ip reporting vars
bool send_ip = false;
bool report_ip = false;


// debugging and latency
// remove before release
boolean debug = false;

void setup() {
  Serial.begin(115200);  
  


  Serial.println(F("Adafruit VL53L1X sensor demo"));

  Wire.begin();
  if (! vl53.begin(0xEACC, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  // initialize LCD
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();
  display_text("Cuebox", "");

  WiFi.onEvent(WiFiEvent);
  ETH.begin();
  Udp.begin(local_port);


  pinMode(button_pin, INPUT);
  pinMode(RedledPin, OUTPUT);
  pinMode(YellowledPin, OUTPUT);
  pinMode(GreenledPin, OUTPUT);
  

}


void loop() {
  // Receive data from Max
  int packetSize = Udp.parsePacket(); // Get the current header packet length
  if (packetSize) {                   // If data is available
    dest_ip = Udp.remoteIP();
    OSCBundle bundle_in;
    while (packetSize--) {
      bundle_in.fill(Udp.read());
    }
    if (!bundle_in.hasError()) {
      bundle_in.dispatch("/backlight", control_backlight);
      bundle_in.dispatch("/display", cue_display);
      bundle_in.dispatch("/identify", identify);
      bundle_in.dispatch("/light_phrase", light_phrase);
      bundle_in.dispatch("/getIP", set_send_ip);
      bundle_in.dispatch("/reportIP", set_report_ip);
    }
    if (debug) {
      char buf[packetSize];
      Udp.read(buf, packetSize); // Read the current packet data
      Serial.println();
      Serial.print("Received: ");
      Serial.println(buf);
      Serial.print("From IP: ");
      Serial.println(Udp.remoteIP());
      Serial.print("From Port: ");
      Serial.println(Udp.remotePort());
    }
  }



  // Read pins
  OSCBundle bundle;

  // read button
  bundle.add(osc_addr_button).add(analogRead(button_pin));

  //read VL53L0
  int16_t distance;

  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(" mm");
    bundle.add("/VL53L1").add(distance);
  }   else {
    bundle.add("/VL53L1").add(-1);
    // data is read out, time for another reading!
    vl53.clearInterrupt();


  }


  

    if (send_ip || report_ip) {
    bundle.add("/ip").add(ETH.localIP().toString().c_str());
    bundle.add("/port").add(local_port);
    send_ip = false;
  }

  //check if in light phrase
  if (in_light_phrase) {
    update_light_phrase();
  }

  // Send values to Max
  Udp.beginPacket(dest_ip, dest_port);
  bundle.send(Udp);
  Udp.endPacket();
  bundle.empty();

  

  if (debug) {
    delay(2000);
  }
}
  


void light_phrase(OSCMessage &msg){
      light_phrase_delay_time = msg.getInt(0);
          // light phrase ---> what I'd have ideally is a default 2000 ms phrase, but /lightPhrase to define a different speed.
        // The bigger problem, though, is that this code doesn't work, at least where its placed in the void loop
      light_phrase_start_time = millis();
      in_light_phrase = true;
//      digitalWrite(GreenledPin, LOW);
//      digitalWrite(RedledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
//      millis(val);                  // wait for a second
//      digitalWrite(RedledPin, LOW);    // turn the LED off by making the voltage LOW
//      digitalWrite(YellowledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
//      delay(val);
//      digitalWrite(GreenledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
//      digitalWrite(YellowledPin, LOW);    // turn the LED off by making the voltage LOW
//      delay(1.5*val);
//      digitalWrite(GreenledPin, LOW);
      
}

void update_light_phrase() {
  // runs every loop
  unsigned long current_time = millis();
  int elapsed_time = (int)(current_time - light_phrase_start_time);
  if (elapsed_time < light_phrase_delay_time) {
    digitalWrite(GreenledPin, LOW);
    digitalWrite(RedledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else if (elapsed_time < light_phrase_delay_time * 2) {
    digitalWrite(RedledPin, LOW);    // turn the LED off by making the voltage LOW
    digitalWrite(YellowledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else if (elapsed_time < light_phrase_delay_time * 3.5){
    digitalWrite(GreenledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(YellowledPin, LOW);    // turn the LED off by making the voltage LOW
  } else {
    digitalWrite(GreenledPin, LOW);
    in_light_phrase = false;
  }
}


/* Expect OSC message to addr /backlight to be a 0 or 1 */
void control_backlight(OSCMessage &msg) {
  int val = msg.getInt(0);
  if (val) {
    lcd.backlight();
  } else {
    lcd.noBacklight();
  }
}



//method to find breaks for text wrapping on display
int findBreaks(String thing, int l){
  for(int i=l-1; i>0;i--){
    Serial.println(thing.charAt(i));
    if(thing.charAt(i)==' '){
      return i;
    }
  }
  return l;
}

/* Display the current IP on the I2C display
  You can index an IPAdress like an array
  example: access the first octet of 'ip' via ip[0] */
void display_ip() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cuebox IP : Port");
  lcd.setCursor(0, 1);
  lcd.print(ETH.localIP());
  lcd.print(" : ");
  lcd.print(local_port);
}

void identify(OSCMessage &msg){
  display_ip();
}


/* Clear the display and show the given arguments */
void display_text(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

/* to center text on display, change each lcd.setCursor to (offset, x). As of now, it's left-justified */

void cue_display(OSCMessage &msg){
  lcd.clear();
  char text[96];
  int a = msg.getString(0,text,96);
  String t = String(text);
  if (t.length() > 80) {
    lcd.setCursor(0, 0);
    lcd.print("Error:");
    lcd.setCursor(0,2);
    lcd.print("Message too long");
  } else {
    
    int last = findBreaks(t,20)+1;
    int offset=(20-last) / 2;
    lcd.setCursor(0, 0);
    lcd.print(t.substring(0, last));
    t.remove(0, last);
    
    last = findBreaks(t,20)+1;
    offset = (20-last) / 2;
    lcd.setCursor(0, 1);
    lcd.print(t.substring(0, last));
    t.remove(0, last);

    last=findBreaks(t, 20)+1;
    offset = (20-last) / 2;
    lcd.setCursor(0, 2);
    lcd.print(t.substring(0, last));
    t.remove(0, last);

    last=findBreaks(t, 20)+1;
    offset = (20-last) / 2;
    lcd.setCursor(0, 3);
    lcd.print(t.substring(0, last));


  }
}

void configure_port(String addr, String val) {

}

void set_send_ip(OSCMessage &msg) {
  send_ip = true;
}

void set_report_ip(OSCMessage &msg) {
  report_ip = msg.getInt(0);
}


static bool eth_connected = false;

void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      display_text("Ethernet Starting", "Standby..");
      //set eth hostname here
      ETH.setHostname("Stompbox_001");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      display_text("Ethernet Connected", "...Resolving IP...");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      display_ip();
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      display_text( "**Connection Lost**",
                    " ***Check Cable*** ");
      eth_connected = false;  
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
} 
