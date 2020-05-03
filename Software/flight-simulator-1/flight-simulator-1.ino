/////////////////////////////////////////////////////////////////////////
//////////////////////// Configuration //////////////////////////////////
/////////////////////////////////////////////////////////////////////////
// Uncomment to enable the features
//#define ENABLE_OTA
//#define ENABLE_SERIAL
#define ENABLE_ENCODER_INTERRUPTS

/////////////////////////////////////////////////////////////////////////
//////////////////////// functions declarations /////////////////////////
/////////////////////////////////////////////////////////////////////////

void move_down(int pwm_speed = 1023);
void move_up(int pwm_speed = 1023);
void hold_position();
void encoder_up();
void encoder_down();
void encoder_error();
void report_status();

// Special declarations for Interrupt pins
ICACHE_RAM_ATTR void triggered_limit_switch();



/************************* WiFi Access Point *********************************/

#include <ESP8266WiFi.h>        // Include the Wi-Fi library
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#ifdef ENABLE_OTA
  #include <ArduinoOTA.h>
#endif
#include "credentials.h"        // Include Credentials (you need to create that file in the same folder if you cloned it from git)
#include "encoder.h"

#define DEVICE_NAME "piston-1"

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
const float encoder_step = 725./4524;        // encoder measures 4524 ticks for a corresponding distance of 725mm

float target_position = 0;
int pwm_speed = 1023;    // pwm speed set to 10% of max speed by default
volatile float measured_position = 1000000;  // we initialize at 1 000 000 so the piston think it's very high and try to go down to whatever position it is asked to by MQTT then get initialize when it run into the limit switch

// we declare an enum type with possible status for the piston
enum {
  GOINP_UP,
  GOINP_DOWN,
  STOPPED
} piston_direction; 

#ifdef ENABLE_SERIAL
// Array used to display the piston direction on the Serial console
const char* PISTON_DIRECTION_MESSAGES[] = {
  "GOING_UP  ",
  "GOING_DOWN",
  "STOPPED   "
};
#endif

enum {
  TRACKING_CALIBRATION_1, // after boot, quickly go down to find the limit switch
  TRACKING_CALIBRATION_2, // then go back up until the limit switch is released
  TRACKING_CALIBRATION_3, // and finally go back down slowly to fine-tune the zero
  TRACKING_NOMINAL, // the system is running nominally and is tracking "target_position"
  TRACKING_HALT, // the system is halted (encoder problem)
} tracking_status;

#ifdef ENABLE_SERIAL
// Array used to display the tracking status on the Serial console
const char* TRACKING_STATUS_MESSAGES[] = {
  "CALIBRATION_1",
  "CALIBRATION_2",
  "CALIBRATION_3",
  "NOMINAL      ",
  "HALT         "
};
#endif

// Encoder 

#define PIN_encoder_A D6
#define PIN_encoder_B D5

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

#ifdef ENABLE_SERIAL
  Serial.begin(115200); // Start the Serial communication to send messages to the computer
  delay(10);
  Serial.println('\n');
#endif

///////////////////////////////////////////////////////
//////////////////// Start Wifi ///////////////////////
///////////////////////////////////////////////////////
  
  WiFi.begin(ssid, password);             // Connecting to the network
#ifdef ENABLE_SERIAL
  Serial.print("Connecting to ");
  Serial.print(ssid); Serial.println(" ...");
#endif

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
#ifdef ENABLE_SERIAL
    Serial.print(++i); Serial.print(' ');
#endif
  }

#ifdef ENABLE_OTA
////////////////// Initialize OTA /////////////////////
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(DEVICE_NAME);

#ifdef ENABLE_SERIAL
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
#endif /* ENABLE_SERIAL */
  ArduinoOTA.begin();
#endif /* ENABLE_OTA */

#ifdef ENABLE_SERIAL
  Serial.println('\n');
  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());         // Send the IP address of the ESP8266 to the computer
#endif

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
  analogWriteFreq(15000);

  ///////////////////////////////////////////////////////
  ///////// Start with the calibration //////////////////
  ///////////////////////////////////////////////////////

  tracking_status = TRACKING_CALIBRATION_1;

  ///////////////////////////////////////////////////////
  ///////// Initialize the position encoder /////////////
  ///////////////////////////////////////////////////////

  Encoder.begin(PIN_encoder_A, PIN_encoder_B, encoder_up, encoder_down, encoder_error,
#ifdef ENABLE_ENCODER_INTERRUPTS  
  true);
#else
  false);
#endif

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
#ifdef ENABLE_OTA
  ArduinoOTA.handle();
#endif

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  switch(tracking_status) {
    case TRACKING_CALIBRATION_1:
    {
      // Calibration step 1: move the piston down until we reach the limit switch
      if (digitalRead(PIN_limit_switch) == HIGH) {
        // piston at the bottom, move up
        move_up();
        tracking_status = TRACKING_CALIBRATION_2;
      } else {
        // piston not at the bottom, move down
        move_down();
      }
      break;
    }
    case TRACKING_CALIBRATION_2:
    {
      // Piston is going up just above the switch
      if (measured_position > 40) { // 40mm above the limit switch
        // move down slowly
        move_down(1023*.4); // note: the piston doesn't have enough power to toggle the limit switch at 30%
        tracking_status = TRACKING_CALIBRATION_3;
      }
      break;
    }
    case TRACKING_CALIBRATION_3:
    {
      // Piston is going down slowly, the limit switch interrupt will make the transition to TRACKING_NORMAL
      break;
    }
    case TRACKING_NOMINAL:
    {
      // In this mode, we track the target position from MQTT
      // We read control targets from the MQTT server
      Adafruit_MQTT_Subscribe *subscription;
      if ((subscription = mqtt.readSubscription(0))) {
        float value = atof((char *)subscription->lastread);

        /// We check position target from MQTT
        if (subscription == &cmnd_position) {
          if (value >= 0 && value <= max_position) {
            target_position = value;
          } else {
            Serial.println("Received INVALID New target_position");
          }
        }

        /// We check speed target from MQTT
        else if (subscription == &cmnd_speed) {
          if (value >= 0 && value <= 100) {
            pwm_speed = value * 1023 / 100; // we calculate the pwm speed from the % value
          } else {
#ifdef ENABLE_SERIAL
            Serial.println("Received INVALID New speed");
#endif
          }
        }
      }

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

      break;
    }
    case TRACKING_HALT:
      hold_position();
      break;
  }

  report_status();

#ifndef ENABLE_ENCODER_INTERRUPTS  
  Encoder.interrupt();
#endif
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

#ifdef ENABLE_SERIAL
  Serial.print("Connecting to MQTT... ");
#endif

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
#ifdef ENABLE_SERIAL
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
#endif
       mqtt.disconnect();
       delay(250);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
#ifdef ENABLE_SERIAL
  Serial.println("MQTT Connected!");
#endif
}



///////////////////////////////////////////////////////
//// TRIGGERED LIMIT SWITCH - INTERRUPTION FUNCTION ///
///////////////////////////////////////////////////////

ICACHE_RAM_ATTR void triggered_limit_switch() {
  if (tracking_status <= TRACKING_CALIBRATION_3) {
    measured_position = 0.0;
    if (tracking_status == TRACKING_CALIBRATION_3) {
      tracking_status = TRACKING_NOMINAL;
    }
  }
  if (piston_direction == GOINP_DOWN){ // if we were going down
    hold_position(); // we stop
  }
}



///////////////////////////////////////////////////////
/////////////// ENCODER - CALLBACKS ///////////////////
///////////////////////////////////////////////////////

// This function is called from encoder.h when the encoder is detecting a movement toward the top
void encoder_up() {
  measured_position+=encoder_step;
  if (measured_position > max_position + 10 && piston_direction == GOINP_UP) {
    // This should not happen, the loop() function should have stopped the piston before.
    // Emergency stop!
    hold_position();
    tracking_status = TRACKING_HALT;
  }
}

// This function is called from encoder.h when the encoder is detecting a movement toward the bottom
void encoder_down() {
  measured_position-=encoder_step;
}

// This function is called from encoder.h when the encoder is detecting an issue
void encoder_error() {
#ifdef ENABLE_SERIAL
  Serial.println("Encoder error, aborting!");
#endif
  hold_position();
  tracking_status = TRACKING_HALT;
}

///////////////////////////////////////////////////////
///////////////// MOVE DOWN - FUNCTION ////////////////
///////////////////////////////////////////////////////

void move_down(int pwm_speed){    // we set the default speed to max speed in case it's not specified ;) 

// check the limit switch
  if(digitalRead(PIN_limit_switch) == LOW){                                            // the switch is NOT pressed
    // limit switch unpressed => we CAN go down :)

    // setting the motor enable PINs to the reverse conf
    digitalWrite(PIN_motor_enable_1, HIGH); // Controled by ground so LOW = ON
    digitalWrite(PIN_motor_enable_2, LOW); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, pwm_speed);
    piston_direction = GOINP_DOWN;
    
  } else {                                                                            // the switch IS pressed
#ifdef ENABLE_SERIAL
    Serial.println("asked to move_down but already at the bottom !");
#endif
    // else the limit switch is reached and we brake

    // setting the motor enable PINs to breaking conf
    digitalWrite(PIN_motor_enable_1, HIGH); // Controled by ground so HIGH = OFF
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, 0);
    piston_direction = STOPPED;
    
  }
}



///////////////////////////////////////////////////////
///////////////// MOVE UP - FUNCTION //////////////////
///////////////////////////////////////////////////////

void move_up(int pwm_speed){    // we set the default speed to max speed in case it's not specified ;) 

// check the position of the piston
  if(measured_position < max_position){    // we make sure the piston position is not above the max
    // setting the motor enable PINs to the forward conf
    digitalWrite(PIN_motor_enable_1, LOW);  // Controled by ground so LOW = ON
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, pwm_speed);
    piston_direction = GOINP_UP;
    
  } else {
    // else the max position is reached and we brake
#ifdef ENABLE_SERIAL
    Serial.println("move_up: already at the top");
#endif
    // setting the motor enable PINs to breaking conf
    digitalWrite(PIN_motor_enable_1, HIGH); // Controled by ground so HIGH = OFF
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, 0);
    piston_direction = STOPPED;    
  }
}



///////////////////////////////////////////////////////
////////////// HOLD POSITION - FUNCTION ///////////////
///////////////////////////////////////////////////////

void hold_position(){    // we press the brake :) 
    // setting the motor enable PINs to breaking conf
    digitalWrite(PIN_motor_enable_1, HIGH); // Controled by ground so HIGH = OFF
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, 0);
    piston_direction = STOPPED;
}

///////////////////////////////////////////////////////
////////////// REPORT THE SYSTEM STATUS ///////////////
///////////// ON THE CONSOLE AND ON MQTT //////////////
///////////////////////////////////////////////////////

#define REPORT_STATUS_PERIOD_MS 500
void report_status()
{
  static unsigned long previous_millis = 0;
  if (millis() - previous_millis < REPORT_STATUS_PERIOD_MS) {
    // too early to report the status, abort now
    return;
  }

#ifdef ENABLE_SERIAL
  Serial.print(TRACKING_STATUS_MESSAGES[tracking_status]);
  Serial.print(" | ");
  Serial.print(PISTON_DIRECTION_MESSAGES[piston_direction]);
  Serial.print(" | ");
  Serial.print(measured_position);
  Serial.print(" -> ");
  Serial.print(target_position);
  Serial.print(" | ");
  Serial.println(pwm_speed);
#endif

  /// we repport status and publish to mqtt
  stat_position.publish(measured_position);

  previous_millis = millis();
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
