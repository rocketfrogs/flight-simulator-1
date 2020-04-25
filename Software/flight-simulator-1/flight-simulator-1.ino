// function definitions
void move_down(int pwm_speed = 1023);
void move_up(int pwm_speed = 1023);
void hold_position();
void encoder();
void triggered_limit_switch();

/************************* WiFi Access Point *********************************/

#include <ESP8266WiFi.h>        // Include the Wi-Fi library
#include "credentials.h"        // Include Credentials (you need to create that file in the same folder if you cloned it from git)

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
Adafruit_MQTT_Publish stat_position = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/flight-simulator-1/piston-1/stat/position"); // to change for each pistons

// Setup a feeds for subscribing to changes.
Adafruit_MQTT_Subscribe cmnd_position = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/flight-simulator-1/piston-1/cmnd/position"); // to change for each pistons
Adafruit_MQTT_Subscribe cmnd_speed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/flight-simulator-1/piston-1/cmnd/speed"); // to change for each pistons


// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();


///////////////////// initialize variables  ///////////////////////////


int target_position = 0;
int target_speed = 10;  // speed set to 10% of max speed by default
int pwm_speed = 100;    // pwm speed set to 10% of max speed by default
int measured_position = 0;

int max_position = 500 ;                      // CHANGE TO THE RIGHT VALUE !!! (value in mm)
int  acceptable_position_delta = 10 ;         // CHANGE TO THE RIGHT VALUE !!! (value in mm)
int encoder_ratio = 10;                       // CHANGE TO THE RIGHT VALUE !!!

enum {going_up,going_down,stopped} piston_status; // we declare an enum type with possible status for the piston 

// Encoder 

#define PIN_encoder_A D1
int encoder_A = LOW;
#define PIN_encoder_B D2
int encoder_B = LOW;
int encoder_counter = 0;

// limit switch
#define PIN_limit_switch D0
int limit_switch = LOW;  // HIGH = Trigerred = connected to ground here

// motor control
#define PIN_motor_enable_1 D5
int motor_enable_1 = HIGH;          // the enable pin are controled by ground so we initialize them at HIGH <=> OFF
#define PIN_motor_enable_2 D6
int motor_enable_2 = HIGH;          // the enable pin are controled by ground so we initialize them at HIGH <=> OFF
#define PIN_motor_PWM D7
int motor_PWM = LOW;                // the PWM pin is controled by positive PWM value so we initialize it at LOW <=> OFF


void setup() {

  Serial.begin(115200); // Start the Serial communication to send messages to the computer

  delay(10);
  Serial.println('\n');
  
  WiFi.begin(ssid, password);             // Connect to the network
  Serial.print("Connecting to ");
  Serial.print(ssid); Serial.println(" ...");

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
    Serial.print(++i); Serial.print(' ');
  }

  Serial.println('\n');
  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());         // Send the IP address of the ESP8266 to the computer

  // Setup MQTT subscription feeds.
  mqtt.subscribe(&cmnd_position);




/////////////////////// Initialize PINs //////////////////////////

// INPUTS
// (Info => https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)
pinMode(PIN_encoder_A, INPUT_PULLUP); 
attachInterrupt(digitalPinToInterrupt(PIN_encoder_A), encoder, RISING);

pinMode(PIN_encoder_B, INPUT_PULLUP);                                               // We do not need interrupt on B as we base interuption on Encoder_A signal

pinMode(PIN_limit_switch, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(PIN_limit_switch), triggered_limit_switch, FALLING);    // The switch connects signal to ground when reached
 

// OUTPUTS
pinMode(PIN_motor_enable_1, OUTPUT); 
pinMode(PIN_motor_enable_2, OUTPUT); 
pinMode(PIN_motor_PWM, OUTPUT); 

///// initialization of the piston

// tell the piston to go down slowly
move_down(100);    // we call the move down function with a 100 out of 1023 value
  

}


void loop() {

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
      Serial.println(temp_target_position);
      }
    }

    /// We check speed target from MQTT
    if (subscription == &cmnd_speed) {
      Serial.print(F("Got: "));
      Serial.println((char *)cmnd_speed.lastread);
      int tmp_target_speed = atof((char *)cmnd_speed.lastread);
      if (tmp_target_speed >= 0 && tmp_target_speed <= 100) {
      target_speed = tmp_target_speed;
      Serial.print("Received New target_speed : ");
      Serial.println(target_speed);
      pwm_speed = target_speed * 1023 / 100; // we calculate the pwm speed from the % value
      Serial.print("Calculated pwm_speed : ");
      Serial.println(pwm_speed);
      } else {
      Serial.print("Received UNVALID New onoff status : ");
      Serial.println(tmp_target_speed);
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


//////////////////////////////////////////////////////////
///////////////////// FUNCTIONS //////////////////////////
//////////////////////////////////////////////////////////

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


////////////////////////////////////////// LIMIT SWITCH INTERRUPTION FUNCTION ///////////////////////////////////////////

void triggered_limit_switch() {
  limit_switch = HIGH; // we mark the switch as being trigerred
  measured_position = 0;
  encoder_counter = 0;
  if (piston_status == going_down){ // if we were going down
    hold_position(); // we stop
  }
}

////////////////////////////////////////// ENCODER INTERRUPTION FUNCTION ///////////////////////////////////////////

void encoder() {
  if (digitalRead(PIN_encoder_B)) { // we check if the other encoder signal is UP or DOWN
    encoder_counter++;
  } else {
    encoder_counter--;
  }
  measured_position = encoder_counter / encoder_ratio;

  if (measured_position > max_position) {
    hold_position(); // we stop
  }
  
}


////////////////////////////////////////// MOVE DOWN FUNCTION ///////////////////////////////////////////

void move_down(int pwm_speed){    // we set the default speed to max speed in case it's not specified ;) 

// check the limit switch
  if(limit_switch == LOW){
    // limit switch unpressed => we CAN go down :)

    // setting the motor enable PINs to the reverse conf
    digitalWrite(PIN_motor_enable_1, LOW); // Controled by ground so LOW = ON
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, pwm_speed);
    piston_status = going_down;
    
  } else {
    // else the limit switch is reached and we brake

    // setting the motor enable PINs to breaking conf
    digitalWrite(PIN_motor_enable_1, HIGH); // Controled by ground so HIGH = OFF
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, 0);
    piston_status = stopped;
    
  }
}


////////////////////////////////////////// MOVE UP FUNCTION ///////////////////////////////////////////

void move_up(int pwm_speed){    // we set the default speed to max speed in case it's not specified ;) 

// check the position of the piston
  if(measured_position < max_position){    // we make sure the piston position is not above the max

    // setting the motor enable PINs to the forward conf
    digitalWrite(PIN_motor_enable_1, LOW);  // Controled by ground so LOW = ON
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, pwm_speed);
    piston_status = going_up;
    
  } else {
    // else the max position is reached and we brake

    // setting the motor enable PINs to breaking conf
    digitalWrite(PIN_motor_enable_1, HIGH); // Controled by ground so HIGH = OFF
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, 0);
    piston_status = stopped;
    
  }

  limit_switch = LOW; // we are in move up mode so the switch is reset to LOW

}


////////////////////////////////////////// HOLD POSITION FUNCTION ///////////////////////////////////////////

void hold_position(){    // we press the brake :) 

    // setting the motor enable PINs to breaking conf
    digitalWrite(PIN_motor_enable_1, HIGH); // Controled by ground so HIGH = OFF
    digitalWrite(PIN_motor_enable_2, HIGH); // Controled by ground so HIGH = OFF
    // setting the speed
    analogWrite(PIN_motor_PWM, 0);
    piston_status = stopped;

}
