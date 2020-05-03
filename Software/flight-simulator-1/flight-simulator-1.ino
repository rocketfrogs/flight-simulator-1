/////////////////////////////////////////////////////////////////////////
//////////////////////// functions declarations ///////////////////////////
/////////////////////////////////////////////////////////////////////////

void move_down(int pwm_speed = 1023);
void move_up(int pwm_speed = 1023);
void hold_position();

// Special declarations for Interrupt pins
ICACHE_RAM_ATTR void encoder();
ICACHE_RAM_ATTR void triggered_limit_switch();



/************************* WiFi Access Point *********************************/

#include <ESP8266WiFi.h>        // Include the Wi-Fi library
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "credentials.h"        // Include Credentials (you need to create that file in the same folder if you cloned it from git)

#define DEVICE_NAME "piston-3"

/*
Content of "credentials.h" that matters for this section

// WIFI Credentials

#define WIFI_SSID        "[REPLACE BY YOUR WIFI SSID (2G)]"     // The SSID (name) of the Wi-Fi network you want to connect to
#define WIFI_PASSWORD    "[REPLACE BY YOUR WIFI PASSWORD]"      // The password of the Wi-Fi 
*/

const char* ssid     = WIFI_SSID;         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = WIFI_PASSWORD;     // The password of the Wi-Fi 

/************************* MQTT Setup *********************************/

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "credentials.h"

/*
// MQTT Credentials

Content of "credentials.h" that matters for this section

#define AIO_SERVER      "[REPLACE BY YOUR MQTT SERVER IP ADDRESS OR ITS FQDN]"
#define AIO_SERVERPORT  [REPLACE BY THE PORT NUMBER USED FOR THE MQTT SERVICE ON YOUR MQTT SERVEUR (DEFAULT IS 1883)]       // use 8883 for SSL"
#define AIO_USERNAME    ""  // USE THIS IF YOU HAVE USERNAME AND PASSWORD ENABLED ON YOUR MQTT SERVER
#define AIO_KEY         ""  // USE THIS IF YOU HAVE USERNAME AND PASSWORD ENABLED ON YOUR MQTT SERVER
*/

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feeds for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish stat_position = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/flight-simulator-1/" DEVICE_NAME "/stat/position"); // to change for each pistons

// Setup a feeds for subscribing to changes.
Adafruit_MQTT_Subscribe cmnd_position = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/flight-simulator-1/" DEVICE_NAME "/cmnd/position"); // to change for each pistons
Adafruit_MQTT_Subscribe cmnd_speed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/flight-simulator-1/" DEVICE_NAME "/cmnd/speed"); // to change for each pistons


// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();


///////////////////// initialize variables  ///////////////////////////

const float max_position = 725;              // CHANGE TO THE RIGHT VALUE !!! (value in mm)
const float acceptable_position_delta = 10;  // CHANGE TO THE RIGHT VALUE !!! (value in mm)
const float encoder_step = 725./1131;        // encoder measures 1131 ticks for a corresponding distance of 725mm

float target_position = 0;
int pwm_speed = 1023;    // pwm speed set to 10% of max speed by default
volatile float measured_position = 1000000;  // we initialize at 1 000 000 so the piston think it's very high and try to go down to whatever position it is asked to by MQTT then get initialize when it run into the limit switch


enum {going_up,going_down,stopped} piston_status; // we declare an enum type with possible status for the piston 

// Encoder 

#define PIN_encoder_A D5
#define PIN_encoder_B D6

// limit switch
#define PIN_limit_switch D7

// motor control
#define PIN_motor_enable_1 D2
#define PIN_motor_enable_2 D3
#define PIN_motor_PWM D1





///////////////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////                         ///////////
//////  ///////////                         ///////////
//\        ////////          SETUP          ///////////
///\      /////////                         ///////////
////\    //////////                         ///////////
/////\  ///////////////////////////////////////////////
///////////////////////////////////////////////////////
void setup() {

///////////////////////////////////////////////////////
///////////////////// Start Serial ////////////////////
///////////////////////////////////////////////////////

  Serial.begin(115200); // Start the Serial communication to send messages to the computer

  delay(10);
  Serial.println('\n');

///////////////////////////////////////////////////////
//////////////////// Start Wifi ///////////////////////
///////////////////////////////////////////////////////
  
  WiFi.begin(ssid, password);             // Connecting to the network
  Serial.print("Connecting to ");
  Serial.print(ssid); Serial.println(" ...");

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
    Serial.print(++i); Serial.print(' ');
  }

////////////////// Initialize OTA /////////////////////

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(DEVICE_NAME);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  Serial.println('\n');
  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());         // Send the IP address of the ESP8266 to the computer


///////////////////////////////////////////////////////
////////////// Suscribing to MQTT topics //////////////
///////////////////////////////////////////////////////

  // Setup MQTT subscription feeds.
  mqtt.subscribe(&cmnd_position);
  mqtt.subscribe(&cmnd_speed);



///////////////////////////////////////////////////////
////////////////// Initialize PINs ////////////////////
///////////////////////////////////////////////////////

// INPUTS
// (Info => https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)
pinMode(PIN_encoder_A, INPUT_PULLUP);                                                        // there is a 10K ohm pull-UP connected to that pin
attachInterrupt(digitalPinToInterrupt(PIN_encoder_A), encoder, RISING);                      // the hall sensor goes to GROUND when it sense a magnetic field but there are NAN gates between the hall sensors and the PIN so the logic is inverted

pinMode(PIN_encoder_B, INPUT_PULLUP);                                                        // there is a 10K ohm pull-UP connected to that pin // We do not need interrupt on B as we base interuption on Encoder_A signal

pinMode(PIN_limit_switch, INPUT);                                                            // there is a 10K ohm pull-DOWN connected to that pin 
attachInterrupt(digitalPinToInterrupt(PIN_limit_switch), triggered_limit_switch, RISING);    // there is a NAN gate between the switch and the PIN so the logic is inverted (Rising = Linit Switch triggered)
if (digitalRead(PIN_limit_switch) == HIGH) {
  // If the switch is already pressed at start (meaning it is already fully down), we call the  triggered_limit_switch() interrupt fonction now as it will dot see a FALLING edge and therefore will not be able to initialize itself
  triggered_limit_switch();
}

// OUTPUTS
pinMode(PIN_motor_enable_1, OUTPUT); 
pinMode(PIN_motor_enable_2, OUTPUT); 
pinMode(PIN_motor_PWM, OUTPUT); 

///// initialization of the piston
// The piston will be innitialized at start
  

}
///////////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  /// ///////////                        ///////////
///  ///   /////////                        ///////////
///          ///////      END OF SETUP      ///////////
////////   /////////                        ///////////
//////// ///////////                        ///////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////



///////////////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////                         ///////////
//////  ///////////                         ///////////
//\        ////////           LOOP          ///////////
///\      /////////                         ///////////
////\    //////////                         ///////////
/////\  ///////////////////////////////////////////////
///////////////////////////////////////////////////////

void loop() {
  ArduinoOTA.handle();

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here


delay(50);

// We read control targets from the MQTT server

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(250))) {
    
  /// We check position target from MQTT
    if (subscription == &cmnd_position) {
      Serial.print(F("Got: "));
      Serial.println((char *)cmnd_position.lastread);
      int temp_target_position = atof((char *)cmnd_position.lastread);
      if (temp_target_position >= 0 && temp_target_position <= max_position) {
        target_position = temp_target_position;
        Serial.print("Received New target_position : ");
        Serial.println(target_position);
      } else {
        Serial.print("Received UNVALID New target_position : ");
        Serial.println((char *)cmnd_position.lastread);
      }
    }

    /// We check speed target from MQTT
    if (subscription == &cmnd_speed) {
      Serial.print(F("Got: "));
      Serial.println((char *)cmnd_speed.lastread);
      float pwm_speed_percent = atof((char *)cmnd_speed.lastread);
      if (pwm_speed_percent >= 0 && pwm_speed_percent <= 100) {
        Serial.print("Received New target_speed : ");
        Serial.println(pwm_speed_percent);
        pwm_speed = pwm_speed_percent * 1023 / 100; // we calculate the pwm speed from the % value
        Serial.print("Calculated pwm_speed : ");
        Serial.println(pwm_speed);
      } else {
        Serial.print("Received INVALID New speed : ");
        Serial.println((char *)cmnd_speed.lastread);
      }
      
    }

 }

/// We read current position
// => done in the Encoder interrupt 

/// We compare target versus current position
/// 3 cases plus actions

if (measured_position <= (target_position - acceptable_position_delta) ) {
  // we are below acceptable target => we go up
  move_up(pwm_speed);
} else if (measured_position >= (target_position + acceptable_position_delta) ) {
  // we are above acceptable target => we go down
  move_down(pwm_speed);
} else {
  // We are in the acceptable range => we stay where we are 
  hold_position();
}

/// we repport status and publish to mqtt

  // Now we can publish stuff!
  Serial.print(F("\nSending stat_position val "));
  Serial.print(measured_position);
  Serial.print("...");
  if (! stat_position.publish(measured_position)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }


  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  */


}
///////////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  /// ///////////                        ///////////
///  ///   /////////                        ///////////
///          ///////       END OF LOOP      ///////////
////////   /////////                        ///////////
//////// ///////////                        ///////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////



///////////////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////                         ///////////
//////  ///////////                         ///////////
//\        ////////        FUNCTIONS        ///////////
///\      /////////                         ///////////
////\    //////////                         ///////////
/////\  ///////////////////////////////////////////////
///////////////////////////////////////////////////////


///////////////////////////////////////////////////////
//////////////// MQTT_connect Function ////////////////
///////////////////////////////////////////////////////

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(250);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}



///////////////////////////////////////////////////////
//// TRIGGERED LIMIT SWITCH - INTERRUPTION FUNCTION ///
///////////////////////////////////////////////////////

ICACHE_RAM_ATTR void triggered_limit_switch() {
  measured_position = 0.0;
  if (piston_status == going_down){ // if we were going down
    hold_position(); // we stop
  }
}



///////////////////////////////////////////////////////
/////////// ENCODER - INTERRUPTION FUNCTION ///////////
///////////////////////////////////////////////////////

ICACHE_RAM_ATTR void encoder() {
  if (digitalRead(PIN_encoder_B)) { // we check if the other encoder signal is UP or DOWN
    measured_position-=encoder_step;
  } else {
    measured_position+=encoder_step;
  }

  if (measured_position > max_position && piston_status == going_up) {
    hold_position(); // we stop
  }
}



///////////////////////////////////////////////////////
///////////////// MOVE DOWN - FUNCTION ////////////////
///////////////////////////////////////////////////////

void move_down(int pwm_speed){    // we set the default speed to max speed in case it's not specified ;) 

// check the limit switch
  if(digitalRead(PIN_limit_switch) == LOW){                                            // the switch is NOT pressed
    Serial.println("move_down");
    // limit switch unpressed => we CAN go down :)

    // setting the motor enable PINs to the reverse conf
    digitalWrite(PIN_motor_enable_1, HIGH); // Controled by ground so LOW = ON
    digitalWrite(PIN_motor_enable_2, LOW); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, pwm_speed);
    piston_status = going_down;
    Serial.println("STATUS => going Down");
    
  } else {                                                                            // the switch IS pressed
    Serial.println("asked to move_down but already at the bottom !");
    // else the limit switch is reached and we brake

    // setting the motor enable PINs to breaking conf
    digitalWrite(PIN_motor_enable_1, HIGH); // Controled by ground so HIGH = OFF
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, 0);
    Serial.println("STATUS => Stopped");
    piston_status = stopped;
    
  }
}



///////////////////////////////////////////////////////
///////////////// MOVE UP - FUNCTION //////////////////
///////////////////////////////////////////////////////

void move_up(int pwm_speed){    // we set the default speed to max speed in case it's not specified ;) 

// check the position of the piston
  if(measured_position < max_position){    // we make sure the piston position is not above the max
    Serial.println("move_up");
    // setting the motor enable PINs to the forward conf
    digitalWrite(PIN_motor_enable_1, LOW);  // Controled by ground so LOW = ON
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, pwm_speed);
    piston_status = going_up;
    Serial.println("STATUS => going Up");
    
  } else {
    // else the max position is reached and we brake
    Serial.println("move_up: already at the top");

    // setting the motor enable PINs to breaking conf
    digitalWrite(PIN_motor_enable_1, HIGH); // Controled by ground so HIGH = OFF
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, 0);
    piston_status = stopped;
    Serial.println("STATUS => Stopped");
    
  }
}



///////////////////////////////////////////////////////
////////////// HOLD POSITION - FUNCTION ///////////////
///////////////////////////////////////////////////////

void hold_position(){    // we press the brake :) 
    Serial.println("hold!");

    // setting the motor enable PINs to breaking conf
    digitalWrite(PIN_motor_enable_1, HIGH); // Controled by ground so HIGH = OFF
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, 0);
    piston_status = stopped;
    Serial.println("STATUS => Stopped");

}
///////////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  /// ///////////                        ///////////
///  ///   /////////                        ///////////
///          ///////    END OF FUNCTIONS    ///////////
////////   /////////                        ///////////
//////// ///////////                        ///////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
