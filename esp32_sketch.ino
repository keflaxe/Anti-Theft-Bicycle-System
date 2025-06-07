#include <Keypad.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <base64.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <ESPSupabase.h>

#define OLED_MOSI 13
#define OLED_CLK 14
#define OLED_DC 11
#define OLED_CS 10
#define OLED_RST 12

#define RXD1 18 // connect to TX pin of GPS module
#define TXD1 17

#define S1 36 // Main Servo
#define S2 35 // Micro Servo

#define BUZZ 21
#define RLED 47
#define GLED 48

#define GPS_BAUD 115200

// Supabase credentials
const char* supabaseUrl = "";
const char* supabaseKey = "";

Supabase supabase;
String tableName = "gps";

TinyGPSPlus gps;

Servo servo1;
Servo servo2;

double lat;
double lon;

// Create an instance of the HardwareSerial class for Serial 1
HardwareSerial gpsSerial(1);

// Helper to pad numbers with zero
String padStart(int num) {
  return num < 10 ? "0" + String(num) : String(num);
}

const int x_out = 1;
const int y_out = 2; 
const int z_out = 9; 
int x_adc_value, y_adc_value, z_adc_value; 
double x_g_value, y_g_value, z_g_value;

unsigned long lastIMUAlertTime = 0; // Time in millis
const unsigned long imuCooldown = 1 * 60 * 1000; // 2 minutes in milliseconds

int otp;
String otpStr;

const char* ssid = "";
const char* password = "";

// Twilio credentials
const char* accountSID = "";
const char* authToken = "";
const char* messagingServiceSid = "";
const char* toPhoneNumber = "";
char messageBody[30];
const char* warningMsg = "Alert Message : Unauthorized Cycle Motion Detected";

// Create the OLED display
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RST, OLED_CS);

const byte ROWS = 4;
const byte COLS = 3;

char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

byte rowPins[ROWS] = {8, 16, 15, 7};
byte colPins[COLS] = {6, 5, 4};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

char keyBuffer[4];
int keyCount = 0;

void sendSMS(String msg) {
  HTTPClient http;
  String url = "https://api.twilio.com/2010-04-01/Accounts/" + String(accountSID) + "/Messages.json";

  http.begin(url);
  String auth = base64::encode(String(accountSID) + ":" + String(authToken));
  http.addHeader("Authorization", "Basic " + auth);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String body = "To=" + String(toPhoneNumber) +
                "&MessagingServiceSid=" + String(messagingServiceSid) +
                "&Body=" + msg;

  int httpResponseCode = http.POST(body);

  Serial.print("HTTP Response code: ");
  Serial.println(httpResponseCode);
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(response);
  }

  http.end();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start");
  pinMode(BUZZ, OUTPUT);
  pinMode(RLED, OUTPUT);
  pinMode(GLED, OUTPUT);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected!");

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD1, TXD1);
  supabase.begin(supabaseUrl, supabaseKey);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(S1, 500, 2400);
  servo2.attach(S2, 500, 2400);

  // Start OLED
  display.begin(0, true); // we dont use the i2c address but we will reset!
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 32);
  display.println("Cycle Locked : Press * to generate OTP");
  display.display();

  xTaskCreatePinnedToCore(gps_control,"GPS",4096,NULL,3,NULL,0);
  xTaskCreatePinnedToCore(keypad_control,"Keypad",6144,NULL,3,NULL,1);
}

void keypad_control(void *pvParameters) {
  while (1) {

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 32);
    display.println("Cycle Locked : Press * to generate OTP");
    display.display();

    digitalWrite(GLED, LOW);
    digitalWrite(RLED, HIGH);

    char key = keypad.getKey();

    x_adc_value = analogRead(x_out); /* Digital value of voltage on x_out pin */ 
    y_adc_value = analogRead(y_out); /* Digital value of voltage on y_out pin */ 
    z_adc_value = analogRead(z_out); /* Digital value of voltage on z_out pin */ 

    x_g_value = ( ( ( (double)(x_adc_value * 3.3)/4096) - 1.65 ) / 0.330 ); /* Acceleration in x-direction in g units */ 
    y_g_value = ( ( ( (double)(y_adc_value * 3.3)/4096) - 1.65 ) / 0.330 ); /* Acceleration in y-direction in g units */ 
    z_g_value = ( ( ( (double)(z_adc_value * 3.3)/4096) - 1.80 ) / 0.330 ); /* Acceleration in z-direction in g units */

    if ((fabs(x_g_value) + fabs(y_g_value) + fabs(z_g_value) > 5) && (millis() - lastIMUAlertTime > imuCooldown)) {
      
      sendSMS(warningMsg);
      Serial.println("Message Sent");
      digitalWrite(BUZZ, HIGH);
      vTaskDelay(2000/portTICK_PERIOD_MS);
      digitalWrite(BUZZ, LOW);
      lastIMUAlertTime = millis(); // Update the timestamp of last alert
    }

    if (key == '*'){
      Serial.println("Star Pressed");

      otp = random(1000,10000);
      otpStr = String(otp);
      sendSMS("Your OTP is: " + otpStr);

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SH110X_WHITE);
      display.setCursor(0, 32);
      display.println("Enter OTP : ");
      display.display();

      for(;;){

      char enteredKey = keypad.getKey();

      if (enteredKey != NO_KEY && enteredKey != '*' && enteredKey != '#') {
      Serial.print("Key Pressed: ");
      Serial.println(enteredKey);
      display.setTextSize(1);
      display.setTextColor(SH110X_WHITE);
      display.setCursor(70 + 7*keyCount, 32);
      display.println(enteredKey);
      display.display();
      //vTaskDelay(200/portTICK_PERIOD_MS);

      if (keyCount < 4) {
        keyBuffer[keyCount] = enteredKey;
        keyCount++;
      }

      if (keyCount == 4) {
        Serial.print("Sequence: ");
        for (int i = 0; i < 4; i++) {
          Serial.print(keyBuffer[i]);
        }
        Serial.println();

        if (keyBuffer[0] == otpStr[0] && keyBuffer[1] == otpStr[1] && keyBuffer[2] == otpStr[2] && keyBuffer[3] == otpStr[3]) {
          vTaskDelay(1000/portTICK_PERIOD_MS);
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SH110X_WHITE);
          display.setCursor(0, 32);
          display.println("Access Granted");
          display.display();
          vTaskDelay(2000/portTICK_PERIOD_MS);
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SH110X_WHITE);
          display.setCursor(0, 32);
          display.println("Cycle Unlocked, Press '#' to lock cycle");
          display.display();
          vTaskDelay(100/portTICK_PERIOD_MS);
          servo2.write(0);
          vTaskDelay(1000/portTICK_PERIOD_MS);
          servo1.writeMicroseconds(2500);
          vTaskDelay(4000/portTICK_PERIOD_MS);
          digitalWrite(GLED, HIGH);
          digitalWrite(RLED, LOW);
          for(;;){
            char keySec = keypad.getKey();
            if (keySec == '#'){
              keyCount = 0;
              memset(keyBuffer, '\0', sizeof(keyBuffer));
              servo1.writeMicroseconds(500);
              vTaskDelay(4000/portTICK_PERIOD_MS);
              servo2.write(180);
              vTaskDelay(1000/portTICK_PERIOD_MS);
              break;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
          }
          break;
        }

        else{
          vTaskDelay(1000/portTICK_PERIOD_MS);
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SH110X_WHITE);
          display.setCursor(0, 32);
          display.println("Permission Denied");
          display.display();
          digitalWrite(BUZZ, HIGH);
          vTaskDelay(2000 / portTICK_PERIOD_MS);
          digitalWrite(BUZZ, LOW);
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SH110X_WHITE);
          display.setCursor(0, 32);
          display.println("Enter OTP : ");
          display.display();
        }

        // Reset buffer
        keyCount = 0;
        memset(keyBuffer, '\0', sizeof(keyBuffer));
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Prevent WDT reset

    }
  }
  vTaskDelay(10 / portTICK_PERIOD_MS); // Prevent CPU hogging
 }
} 

void gps_control(void *pvParameters){
  while (1){
    unsigned long start = millis();

    while (millis() - start < 1000) {
      while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
      }

      if (gps.location.isUpdated()) {
        Serial.println("------ GPS Update ------");
        Serial.print("LAT: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("LONG: "); 
        Serial.println(gps.location.lng(), 6);
        Serial.print("SPEED (km/h): "); 
        Serial.println(gps.speed.kmph()); 
        Serial.print("ALT (m): "); 
        Serial.println(gps.altitude.meters());
        Serial.print("HDOP: "); 
        Serial.println(gps.hdop.value() / 100.0); 
        Serial.print("Satellites: "); 
        Serial.println(gps.satellites.value()); 
        Serial.print("Time UTC: ");
        Serial.println(String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day()) + " " +
                      String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
        Serial.println("------------------------");
      }
    }

    if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
      lat = gps.location.lat();
      lon = gps.location.lng();

      // Format timestamp in ISO 8601 (UTC)
      String timestamp = String(gps.date.year()) + "-" + 
                        padStart(gps.date.month()) + "-" + 
                        padStart(gps.date.day()) + "T" + 
                        padStart(gps.time.hour()) + ":" + 
                        padStart(gps.time.minute()) + ":" + 
                        padStart(gps.time.second()) + "Z";

      // Create JSON payload
      String jsonData = "{";
      jsonData += "\"timestamp\": \"" + timestamp + "\", ";
      jsonData += "\"latitude\": " + String(lat, 6) + ", ";
      jsonData += "\"longitude\": " + String(lon, 6);
      jsonData += "}";

      // Send to Supabase
      int response = supabase.insert(tableName, jsonData, false);
      if (response == 201) {
        Serial.println("Data inserted successfully!");
      } else {
        Serial.print("Failed to insert data. HTTP response: ");
        Serial.println(response);
      }
    } else {
      Serial.println("Waiting for valid GPS fix and time...");
    }

    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void loop() {}
