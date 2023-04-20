/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/
#include "Keypad.h"
#include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "WiFi.h"
#include <ESP32Servo.h>
#include <iostream>
#include <deque>
// Replace the next variables with your SSID/Password combination
const char* ssid = "OPPOA96";
const char* password = "12345678";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "192.168.237.200";

#define SERVO_PIN 13 // ESP32 pin GIOP13 connected to servo motor

Servo servoMotor;
WiFiClient espClient;
PubSubClient client(espClient);

char msg[50];
int value = 0;
int pos = 0; 
bool dock_flag = true;
const char* device_name = "ESP32";
std::deque<int> table_number_queue;
int table_number;
/* Setting up the keypad */
const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
bool wlc_flag = true;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

byte rowPins[ROWS] = {33, 12, 14, 26}; 
byte colPins[COLS] = {25, 32, 27};


Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS); 

void setup() {
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoMotor.setPeriodHertz(50);
  servoMotor.attach(SERVO_PIN, 500, 2500);  // attaches the servo on ESP32 pin

  Serial.println("OLED FeatherWing test");
  /* SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally */
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

  Serial.println("OLED begun");

  /* Show image buffer on the display hardware. */
  /* Since the buffer is intialized with an Adafruit splashscreen */
  /* internally, this will display the splashscreen. */
  display.display();
  delay(1000);

  /* Clear the buffer. */
  display.clearDisplay();
  display.display();

  Serial.println("IO test");

  /* text display tests */
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.setTextWrap(false);
  display.display(); // actually display all of the above
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    attempts++;
  if(attempts > 15){
    ESP.restart();
  }    
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    display.fillCircle(64, 32, 20, SSD1306_WHITE);
    display.display();
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(device_name)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/input");
      display.clearDisplay();
      display.display();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2.5 seconds");
      // Wait 5 seconds before retrying
      delay(2500);
    }
  }
}

void display_message(char msg[]){
  
  int x = 0;
  int minX = -12 * strlen(msg);
  for (x = display.width();x > minX; x -= 4 ){
    display.clearDisplay();
    display.setCursor(0,0);
    display.setCursor(x, 10);
    display.print(msg);
    display.display();
    delay(10);
  }

}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();


  if (String(topic) == "esp32/input") {
    Serial.println("Changing dock_flag to ");
    if(messageTemp == "1"){
      //client.publish("esp32/input", "DOCKED");
      Serial.println("True");
      dock_flag = true;
    }
    else if(messageTemp == "0"){
      //client.publish("esp32/input", "NOT_DOCKED");
      Serial.println("False");
      dock_flag = false;
    }
  }
}

void open_servo(){
  for (pos = 93; pos >= 0; pos -= 1) { // goes from 0 degrees to 180 degrees
		// in steps of 1 degree
	  servoMotor.write(pos);    // tell servo to go to position in variable 'pos'
		delay(10);             // waits 10ms for the servo to reach the position
	} 
	
  display_message("Publishing table number");

  for (pos = 0; pos <= 93; pos += 1) { // goes from 180 degrees to 0 degrees
		  servoMotor.write(pos);  // tell servo to go to position in variable 'pos'
		  delay(10);             // waits 15ms for the servo to reach the position
   }
}

void loop() {
  client.loop();
  char key = keypad.getKey();

  if (!client.connected()) {
    reconnect();
  }
  if (wlc_flag){
    display_message("Press 1-6 for table number");
    display_message("'#' to confirm table number");
    display_message("'*' to delete table number");
    wlc_flag = false;
  }
  
  if (table_number_queue.size() == 1 && dock_flag == true) {
    
    display_message("Publishing number from queue");
    open_servo();
    char msg[50]; 
    snprintf(msg, 50, "%d",table_number_queue[0]) ;
    client.publish("esp32/output", msg);
    table_number_queue.pop_front();
    table_number = 0;
    
    
  }
  else if (key == '*'){
    display.clearDisplay();
    display.setCursor(0,0);
    display.display();
    table_number = 0;
    
  }   
  else if (key && key != '#') {

    display.clearDisplay();
    display.setCursor(0,0);
    display.print(key);
    display.display();
    table_number = int(key) - 48;
  }
  else if (key == '#' && table_number){
      table_number_queue.push_back(table_number);
      Serial.print("Number in queue: ");
      Serial.print(table_number_queue[0]);     
      if (dock_flag == false){
        if (table_number_queue.size() > 1){
          display_message("Please wait for delivery bot to return");
          table_number_queue.pop_back();
          table_number = 0;
        }
        else {
          display_message("Number added to queue ");
          table_number = 0;
        }
      }

      if (dock_flag == true){
          
          open_servo();
          char msg[50]; 
          snprintf(msg, 50, "%d",table_number_queue[0]) ;
          client.publish("esp32/output", msg);
        
          table_number_queue.pop_front();
          table_number = 0;
          
      }


            
    }
  }

