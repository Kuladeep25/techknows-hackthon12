# techknows-hackthon12

//    TOPIC NAME: 
        SAMRT INTELLIGENT ACCIDENT DETECTION SYSTEM

//    TEAM NAME:
        Dev dynamos
        
//    TEAM MEMBERS and MAIL IDs:
        T.K.Sreeya    sreeyatk@gmail.com
        U.Kuladeep    kuladeepu2544@gmail.com
        E.Shriya      shriyareddyettadi@gmail.com
        R.Ganesh      rganeshsivasaicharan@gmail.com
      
        
// Algorithm 1: For Accident Detection

Input Data: Value of force (F) and speed (S)
Output: Accident status
    acc ← 0
    if (F > Tforce & S > Tspeed) then acc ← 1
    else if (F > Tforce|S > Tspeed)
          acc ← 1
    end if
    if acc= 1 then
         Activate alarm and set alarm timer (AT = 0)
         Alarm_OFF_timer ← Alarm_OFF()
         If (AT >= 30 seconds) then
                    status= Accident_detected
        else
                    status= no_accident
                    Get nearby mechanics details from database and send message to the owner
        end if
    end if
    if (status= Accident_detected) then
             final_status = call Deep_Learning_Module()
             if (final_status= Accident_detected) then
                       Get accident location from GPS
                       call rescue_operation_module()
Else
           Get nearby mechanics details from database and send message to the owner

           
// Algorithm 2: For Rescue Operation

Input Data: longitude and latitude
Output: Inform to all emergency services
          Slat = starting latitude
          elat = end latitude
          s1on = starting longitude
          e1on = end longitude
          dis_lat = elat – slat
          dis_lon = elon – slon
          Dist = R * H
          Find nearby police station and hospital using Haversine
          Hostp = nearby_hospital
          Police_station = nearby_police_station
         Get car details, vehicle details, mechanic details
         Send message to cloud and all emergency services using GSM module


// libraries used in this code.

#include <Adafruit_GPS.h>
#include <Adafruit_LIS3DH.h>
#include <HX711.h>
#include <GSM.h>


// Arduino IDE implementing cpp code

#define Tforce 100 // Threshold force value (adjust as needed)
#define Tspeed 50 // Threshold speed value (adjust as needed)
#define R 6371 // Earth radius in km
#define H 0.1 // Threshold distance in km for nearby services

bool alarmActivated = false;
unsigned long alarmStartTime = 0;
bool accidentDetected = false;
bool rescueOperationInitiated = false;

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() {
  // Input data
  int forceValue = readForceValue(); // Function to read force value from sensor
  int speedValue = readSpeedValue(); // Function to read speed value from sensor

  // Algorithm 1: Accident Detection
  if (forceValue > Tforce && speedValue > Tspeed) {
    accidentDetected = true;
  } else if (forceValue > Tforce || speedValue > Tspeed) {
    accidentDetected = true;
  }

  if (accidentDetected && !alarmActivated) {
    activateAlarm();
    alarmStartTime = millis();
  }

  if (accidentDetected && (millis() - alarmStartTime >= 30000)) { // 30 seconds
    rescueOperationInitiated = true;
  }

  // Algorithm 2: Rescue Operation
  if (rescueOperationInitiated) {
    float latitude = readLatitude(); // Function to read latitude from GPS
    float longitude = readLongitude(); // Function to read longitude from GPS

    // Find nearby police station and hospital
    float nearbyHospitalDistance = haversine(latitude, longitude, hospitalLatitude, hospitalLongitude);
    float nearbyPoliceStationDistance = haversine(latitude, longitude, policeStationLatitude, policeStationLongitude);

    // If hospital or police station is within threshold distance, send message
    if (nearbyHospitalDistance < H || nearbyPoliceStationDistance < H) {
      sendRescueOperationMessage();
    }

    // Get car details, vehicle details, mechanic details
    // Send message to cloud and all emergency services using GSM module

    // Reset variables for next iteration
    resetVariables();
  }

  delay(1000); // Adjust delay as needed
}

int readForceValue() {
  // Read force value from sensor (e.g., weight sensor)
  // Return the read value
}

int readSpeedValue() {
  // Read speed value from sensor (e.g., accelerometer)
  // Return the read value
}

float readLatitude() {
  // Read latitude from GPS module
  // Return the read value
}

float readLongitude() {
  // Read longitude from GPS module
  // Return the read value
}

void activateAlarm() {
  // Activate alarm (e.g., sound buzzer, turn on LED)
  alarmActivated = true;
}

void resetVariables() {
  alarmActivated = false;
  accidentDetected = false;
  rescueOperationInitiated = false;
}

float haversine(float lat1, float lon1, float lat2, float lon2) {
  // Calculate haversine distance between two points
  // Return the calculated distance
}

void sendRescueOperationMessage() {
  // Send rescue operation message to emergency services
}

