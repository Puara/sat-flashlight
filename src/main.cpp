//****************************************************************************//
// Puara based Magic Wand (with BNO055 + BMP280 IMU)                          //
// Société des Arts Technologiques (SAT)                                      //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
//****************************************************************************//


#include "Arduino.h"
// ======== tft screen ========
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);

// ======== BNO IMU ========
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055();

// Include Puara's module manager
// If using Arduino.h, include it before including puara.h
#include "puara.h"

#include <iostream>

// Initialize Puara's module manager
Puara puara;

/*
 * Include CNMAT's OSC library (by Adrian Freed)
 * This library was chosen as it is widely used, but it can be replaced by any
 * other OSC library of choice
 */
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>

// UDP instances to let us send and receive packets
WiFiUDP Udp;

// Dummy sensor data
float sensor;
        
// Create bundle to allow sending multiple OSC messages at once
OSCBundle bundle;

// Append message specific address
OSCMessage msgOrientation;
OSCMessage msgAcceleration;
OSCMessage msgGyroscope;
OSCMessage msgPosition;

// Base address of OSC messages
std::string baseOSC;

// Offset orientation when calibrating
float xOffset = 0;
float yOffset;
float zOffset;

float xPosition;
float yPosition;

const bool calibrateOffset = true;
const bool useIP2 = false; // Set to true if you want to send OSC messages to IP2

float offsetValue(float currentValue, float  offsetAmount, float minValue, float maxValue){
    if(calibrateOffset){ // Keep original values if false
        currentValue -= offsetAmount;
    }
    //Loopback if value is outside defined range
    if( currentValue > maxValue){
        currentValue = minValue + (currentValue - maxValue);
    }
    if( currentValue < minValue){
        currentValue = maxValue - (minValue - currentValue);
    }
    
    return currentValue;
}
void findXY(sensors_event_t data) {

    float xAngle = (offsetValue(data.orientation.x, xOffset, 0, 360));
    float yAngle = (offsetValue(data.orientation.y, yOffset, -180, 180));
    float zAngle = (offsetValue(data.orientation.z, zOffset, -90, 90));

    // Convert angles to radians and find X and Y positions
    float x = sin(yAngle*PI/180);
    float y = sin(zAngle*PI/180);
    xPosition = (x * (cos(xAngle*PI/180))) - (y * (sin(xAngle*PI/180)));
    yPosition = (x * (sin(xAngle*PI/180))) + (y * (cos(xAngle*PI/180)));
    
    Serial.print("X Position: ");
    Serial.println(xPosition);
    Serial.print("Y Position: ");
    Serial.println(yPosition);
}

void offsetXAngle(OSCMessage &msg) {
    if (msg.getInt(0) == 1) {
        // Recalibrate the heading offset
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        xOffset = orientationData.orientation.x;
        Serial.print("Heading offset set to: ");
        Serial.println(xOffset);
    }
}

void checkIncomingOSC() {
    
    OSCBundle bundle;
    
    int size = Udp.parsePacket();
    if (size > 0) {
        while (size --){
            bundle.fill(Udp.read());
        }
        if (!bundle.hasError()) {
            // Offset angle to compensate for drift
            bundle.dispatch("/reCalibrate", offsetXAngle);
        } else{
            OSCErrorCode error = bundle.getError();
            Serial.print("Error: ");
            Serial.println(error);
        }
    }
}

void setup() {
    #ifdef Arduino_h
        Serial.begin(115200);
    #endif

    /*
     * the Puara start function initializes the spiffs, reads config and custom json
     * settings, start the wi-fi AP/connects to SSID, starts the webserver, serial 
     * listening, MDNS service, and scans for WiFi networks.
     */
    puara.start();

    // Start the UDP instances 
    Udp.begin(puara.LocalPORT());

    baseOSC = ("/" + puara.dmi_name()).c_str();
    
    // Set message specific address
    msgOrientation.setAddress((baseOSC + "/Orientation").c_str());
    msgAcceleration.setAddress((baseOSC + "/Acceleration").c_str());
    msgGyroscope.setAddress((baseOSC + "/Gyroscope").c_str());
    msgPosition.setAddress((baseOSC + "/Position").c_str());

    //=== turn on and init the tft screen ===
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
    tft.init(135, 240); // Init ST7789 240x135
    tft.setRotation(3); // rotates the screen

    //=== init BNO IMU ===
    if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("No BNO055 detected... Check your wiring or I2C ADDR!");
        while (1)
        ;
    }

    bno.setExtCrystalUse(true);

    delay(1000);
    sensors_event_t initialOrientation;
    bno.getEvent(&initialOrientation, Adafruit_BNO055::VECTOR_EULER);
    yOffset = initialOrientation.orientation.y;
    zOffset = initialOrientation.orientation.z;


    Serial.println("setup completed successfully");

}

void loop() {

    checkIncomingOSC();

    /* Get a new event per sensor */
    sensors_event_t orientationData, angVelocityData, accelerometerData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    // ---- onw clean CSV line for Edge Impulse -----------------------
    Serial.printf("%f,%f,%f,%f,%f,%f\n", accelerometerData.acceleration.x,
                    accelerometerData.acceleration.y,
                    accelerometerData.acceleration.z,
                    angVelocityData.acceleration.x, angVelocityData.acceleration.y,
                    angVelocityData.acceleration.z);
    // ----------------------------------------------------------------

    // Pass the address of orientationData to findXY
    findXY(orientationData);

    /* 
     * Sending OSC messages.
     * If you're not planning to send messages to both addresses (OSC1 and OSC2),
     * it is recommended to set the address to 0.0.0.0 to avoid cluttering the 
     * network (WiFiUdp will print an warning message in those cases).
     */
    if (puara.IP1_ready()) { // set namespace and send OSC message for address 1
    
        bundle.add(msgOrientation.add((offsetValue(orientationData.orientation.x, xOffset, 0, 360))).add(offsetValue(orientationData.orientation.y, yOffset, -180, 180)).add(offsetValue(orientationData.orientation.z, zOffset, -90, 90)));
        bundle.add(msgAcceleration.add(accelerometerData.acceleration.x).add(accelerometerData.acceleration.y).add(accelerometerData.acceleration.z));
        bundle.add(msgGyroscope.add(angVelocityData.acceleration.x).add(angVelocityData.acceleration.y).add(angVelocityData.acceleration.z));
        bundle.add(msgPosition.add(xPosition).add(yPosition));
        
        Udp.beginPacket(puara.IP1().c_str(), puara.PORT1());
        bundle.send(Udp);
        Udp.endPacket();
        
        if(useIP2){ // Disabled as to not clutter the serial output when IP2 is not used
            Udp.beginPacket(puara.IP2().c_str(), puara.PORT2());
            bundle.send(Udp);
            Udp.endPacket();
        }
        
        // Clear OSC 
        bundle.empty();
        msgOrientation.empty();
        msgAcceleration.empty();
        msgGyroscope.empty();
        msgPosition.empty();
    }

    /* Display the floating point orientation data and IP address */
    canvas.fillScreen(ST77XX_BLACK);
    canvas.setCursor(0, 10);
    canvas.setTextSize(2);

    canvas.setTextColor(ST77XX_MAGENTA);
    canvas.print("X: ");
    canvas.print((offsetValue(orientationData.orientation.x, xOffset, 0, 360)), 4);
    canvas.setTextColor(ST77XX_WHITE);
    canvas.print("\nY: ");
    canvas.print((offsetValue(orientationData.orientation.y, yOffset, -90, 90)), 4);
    canvas.setTextColor(ST77XX_CYAN);
    canvas.print("\nZ: ");
    canvas.print((offsetValue(orientationData.orientation.z, zOffset, -180, 180)), 4);

    canvas.setTextColor(ST77XX_GREEN);
    canvas.print("\nIP: ");
    canvas.print(puara.staIP().c_str());

    canvas.setTextColor(ST77XX_YELLOW);
    canvas.print("\nOffsetXYZ: ");
    canvas.print(xOffset);
    canvas.print(", ");
    canvas.print(yOffset);
    canvas.print(", ");
    canvas.print(zOffset);
    
    tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
    
    delay(10);
}