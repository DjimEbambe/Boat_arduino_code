/*
 * Boat project
 * By group 1,
 * o2, parcours ouvert ULC-ICAM, 2022
 */
#include <SPI.h>
#include <WiFiNINA.h>

#include <Arduino_LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <Servo.h>
Servo myservo;

#include "arduino_secrets.h" 
const int enB = 6;
const int in3 = 2;
const int in4 = 4;
const int safran = 3;
int angle = 90;

// Defines
#define SAMPLE_RATE 10  // in Hz

// Constructors
Madgwick filter;  // Madgwick algorithm for roll, pitch, and yaw calculations

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key index number (needed only for WEP)
int speed = 0; 
String value = "";

int status = WL_IDLE_STATUS;
WiFiServer server(80);

void setup() {
  Serial.begin(9600);      // initialize serial communication
   myservo.attach(safran);
   myservo.write(angle);

  Serial.print("LSM6DS3 IMU initialization ");
   if (IMU.begin()) {  // initialize IMU
      Serial.println("completed successfully.");
   } else {
      Serial.println("FAILED.");
      IMU.end();
      while (1);
   }
   Serial.println();
   filter.begin(SAMPLE_RATE);  // initialize Madgwick filter

  pinMode(9, OUTPUT);      
    // put your setup code here, to run once:
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4,OUTPUT);
  //pinMode(button, INPUT);
  
  //initial rotation directon
  digitalWrite(in3, HIGH);
  digitalWrite(in4,LOW);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status
}


void loop() {
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            //HTTP header
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:application/x-www-form-urlencoded");
            client.println();
            //HTTP body
            client.print(value);
            //The HTTP response ends with another blank line:
            client.println();
            //break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // speed of motor
        if (currentLine.endsWith("speed")) {              
          String request = currentLine; 
          request.replace("GET /", " ");
          request.replace("speed", " ");
          int values = request.toInt();         
          int pwmOutput = map(values, 0, 100, 0, 255); //Map the SeekBar from 0 to 100 
          analogWrite(enB, pwmOutput); //Send PWM signal to L298N Enable pin
          value = String(values);
        }

        // controle safran

        // controle safran Left
        if (currentLine.endsWith("safranLeft")) {              
          String request = currentLine; 
          request.replace("GET /", " ");
          request.replace("safranLeft", " ");
          int values = request.toInt();

          
            int left= myservo.read() - 5;
            myservo.write(left); 
           
          
        }

        // controle safran Right
        if (currentLine.endsWith("safranRight")) {              
          String request = currentLine; 
          request.replace("GET /", " ");
          request.replace("safranRight", " ");
          int values = request.toInt();
          
            int right= myservo.read() + 5;
            myservo.write(right);  
             
        }

        if (currentLine.endsWith("data")) {              
          
          char buffer[5];    // string buffer for use with dtostrf() function
          float ax, ay, az;  // accelerometer values
          float gx, gy, gz;  // gyroscope values
          if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()
              && IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {
                
              filter.updateIMU(gx, gy, gz, ax, ay, az);  // update roll, pitch, and yaw values
              // Print rotation angles
              int gite = filter.getRoll();
              int cap = filter.getYaw();
              
              value = "";
              value += gite;
              value += "F";
              
              value += cap;
              value += "F";

              value += ax;
              value += "F";
              
              
          }
          
        }
        
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
  
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}


// Prints IMU values.
void printValues() {
   char buffer[8];    // string buffer for use with dtostrf() function
   float ax, ay, az;  // accelerometer values
   float gx, gy, gz;  // gyroscope values

   // Retrieve and print IMU values
   if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()
      && IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {
      Serial.print("ax = ");  Serial.print(dtostrf(ax, 4, 1, buffer));  Serial.print(" g, ");
      Serial.print("ay = ");  Serial.print(dtostrf(ay, 4, 1, buffer));  Serial.print(" g, ");
      Serial.print("az = ");  Serial.print(dtostrf(az, 4, 1, buffer));  Serial.print(" g, ");
      Serial.print("gx = ");  Serial.print(dtostrf(gx, 7, 1, buffer));  Serial.print(" °/s, ");
      Serial.print("gy = ");  Serial.print(dtostrf(gy, 7, 1, buffer));  Serial.print(" °/s, ");
      Serial.print("gz = ");  Serial.print(dtostrf(gz, 7, 1, buffer));  Serial.println(" °/s");
   }
}


// Prints rotation angles (roll, pitch, and yaw) calculated using the
// Madgwick algorithm.
// Note: Yaw is relative, not absolute, based on initial starting position.
// Calculating a true yaw (heading) angle requires an additional data source,
// such as a magnometer.
void printRotationAngles() {
   char buffer[5];    // string buffer for use with dtostrf() function
   float ax, ay, az;  // accelerometer values
   float gx, gy, gz;  // gyroscope values

   if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()
      && IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {
      filter.updateIMU(gx, gy, gz, ax, ay, az);  // update roll, pitch, and yaw values

      // Print rotation angles
      Serial.print("Roll = ");  Serial.print(dtostrf(filter.getRoll(), 4, 0, buffer)); Serial.print(" °, ");
      Serial.print("Pitch = ");  Serial.print(dtostrf(filter.getPitch(), 4, 0, buffer)); Serial.print(" °, ");
      Serial.print("Yaw = ");  Serial.print(dtostrf(filter.getYaw(), 4, 0, buffer)); Serial.println(" °");
   }
}

void gite() {
   char buffer[5];    // string buffer for use with dtostrf() function
   float ax, ay, az;  // accelerometer values
   float gx, gy, gz;  // gyroscope values

   if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()
      && IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {
        
      filter.updateIMU(gx, gy, gz, ax, ay, az);  // update roll, pitch, and yaw values
      // Print rotation angles
      int gite = filter.getRoll();
   }
}
