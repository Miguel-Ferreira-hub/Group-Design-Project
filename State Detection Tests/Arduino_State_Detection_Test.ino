// ================= LIBRARY IMPORTS =======================
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <math.h>

// ================= FLIGHT STATE ==========================
enum FlightPhase {
  PRELAUNCH,
  LAUNCH,
  BURNOUT,
  APOGEE,
  DESCENT,
  LANDED,
};

FlightPhase flightPhase = PRELAUNCH;

// ================= SERVO MOTOR AND SD CARD ===============
Servo para1;
Servo para2;

File simFile;  // for DATA.CSV
File logFile;  // for TEST.csv
const int chipSelect = 53;

// ================= STATE DETECTION VARIABLES =============
const int N = 100;
float values[N];
int count = 0;
const int neutral = 90;
float last_time = 0;
float last_alt = 0;
float last_vel = 0;
const int N_vel = 40;
float values_vel[N_vel];
int count_vel = 0;
float diff_vel = 0;
const int N_alt = 10;
float values_alt[N_alt];
int count_alt = 0;
float diff_alt = 0;

// ================= SETUP() ===============================
void setup() {
  para1.attach(13); // Set parachute servo 1 pin
  para2.attach(8); // set parachute servo 2 pin
  para1.write(90); // Set parachute servo 1 to 0 degrees
  para2.write(90); // Set parachute servo 2 to 0 degrees
  Serial.begin(115200); // Begin serial connection

  // SD Card
  pinMode(53, OUTPUT); // Set SD card pin
  digitalWrite(53, HIGH); // Set SD card pin to high always
  Serial.println("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("SD init failed!");
    while (1);
  }
  Serial.println("SD ready");

  Serial.println("Logging...");
  Serial.println("All Systems Ready");
  // Open files
  simFile = SD.open("DATA.CSV", FILE_READ);
  logFile = SD.open("TEST.csv", FILE_WRITE);
  if (logFile) {
    logFile.println("Time (s),Altitude (m),Velocity (ms-1),Acceleration (ms-2),State Detection");
    logFile.flush();
  }
}

// ================= LOOP() ================================
void loop() {
  float time, accel, vel, alt;
  // Read simulated data
  if (simFile.available()) {
    String line = simFile.readStringUntil('\n');
    int firstComma = line.indexOf(',');
    int secondComma = line.indexOf(',', firstComma + 1);
    int thirdComma = line.indexOf(',', secondComma + 1);

    String timestr = line.substring(0, firstComma);
    String accelstr = line.substring(firstComma + 1, secondComma);
    String velstr = line.substring(secondComma + 1, thirdComma);
    String altstr = line.substring(thirdComma + 1);

    time = timestr.toFloat()*1000;
    accel = accelstr.toFloat();
    vel = velstr.toFloat();
    alt = altstr.toFloat();
  }
  else {
    Serial.println("End of simulation file");
    while(1); // stop
  }
  String Communication; // Variable for state communication

  // ================= STATE DETECTION =======================
  // Launch
  static unsigned long launch_time = 0; // Time variable for detection
  if (accel > 20 && flightPhase == PRELAUNCH) { // Threshold for detection
    if (launch_time == 0) launch_time = last_time; // Record first detection
    if (time - launch_time > 300) { // Difference between system start and first detection
      Communication = "Launch Detected"; // Detect launch if counter exceeds 300ms
      Serial.println(Communication); // Send communication to serial monitor
      flightPhase = LAUNCH;
    } 
  } else { 
    launch_time = 0; // Reset launch timer
  }
  // Burnout
  static unsigned long burnout_time = 0; // Time variable for detection
  if (accel < 20 && flightPhase == LAUNCH) { // Threshold and conditions for detection 
    if (burnout_time == 0) burnout_time = last_time; // Record first detection
    if (time - burnout_time > 300) { // Difference between system start and first detection
      Communication = "Burnout Detected"; // Detect burnout if counter exceeds 300ms
      Serial.println(Communication); // Send communication to serial monitor
      flightPhase = BURNOUT; // Set burnout phase
    } 
  } else {
    burnout_time = 0; // Reset burnout timer
  }
  // Apogee
  if (flightPhase == BURNOUT) { // Check for apogee after burnout
    values_alt[count_alt] = alt; // Store altitude readings
    count_alt++;
    if (count_alt >= N_alt) {
      count_alt = 0;  // wrap around
    }
    static bool buffer_full = false;
    if (count_alt == 0) {
      buffer_full = true;
    }
    if (buffer_full) {
      int oldest_index = count_alt;
      int newest_index = (count_alt - 1 + N_alt) % N_alt;
      diff_alt = values_alt[newest_index] - values_alt[oldest_index]; // Compare sequential altitude readings
      if (diff_alt < 0) {   
        Communication = "Apogee Detected"; // Detect apogee and change state
        Serial.println(Communication);
        flightPhase = APOGEE;
        Serial.println("Parachute Deployed");
        para1.write(20);// Deploy parachute
        para2.write(160);
      }
    }
  }
  // Descent
  if (flightPhase == APOGEE) { // Only check if apogee has been detected
    static bool buffer_full = false;
    // Store new velocity sample
    values_vel[count_vel] = vel; // Store velocity readings
    count_vel++;
    // Wrap around
    if (count_vel >= N_vel) {
      count_vel = 0;
      buffer_full = true;
    }
    if (buffer_full) {
      int oldest_index = count_vel;  
      int newest_index = (count_vel - 1 + N_vel) % N_vel;
      diff_vel = values_vel[newest_index] - values_vel[oldest_index]; // Compare sequential altitude readings
      if (diff_vel < 0) {   // adjust threshold!
        Communication = "Descent Detected";
        Serial.println(Communication);
        flightPhase = DESCENT;
      }
    }
  }
  // Landing
  if (flightPhase == DESCENT) { // Condition for detection
    values[count] = alt; // Add samples to array
    count++; // increase count for each sample
    if (count >= N) { // Condition for computing standard deviation
      float mean = 0; // mean
      for (int i=0; i < N; i++) mean += values[i]; // Iterate through samples
      mean /= N; // Calculate mean
      float sumSqDiff = 0; // Sum of squares
      for (int i = 0; i < N; i++) { // Iterate through samples
      sumSqDiff += (values[i] - mean) * (values[i] - mean); // Calculate sum of squares
    }
    float stdDev = sqrt(sumSqDiff / N); // Standard deviation of samples
    if (stdDev < 0.30) { // Condition for rocket having landed
        Communication = "Landed"; // Detect landing
    Serial.println(Communication); // Send communication to serial monitor
    flightPhase = LANDED;
      }
      count = 0; // Reset count
    }
  }

  // ================= CSV AND DATA ==========================
  String row = // CSV row
    String(time) + "," + String(alt) + "," + String(vel) + "," + String(accel) + "," + Communication;
  logFile.println(row); // Apend row
  logFile.flush(); // Write row
  Serial.print("Time: " + String(time)) + ",";
  Serial.print("Altitude: " + String(alt)) + ",";
  Serial.print("Velocity: " + String(vel)) + ",";
  Serial.println("Acceleration: " + String(accel)) + ",";
  last_time = time;
  last_alt = alt;
  last_vel = vel;
}