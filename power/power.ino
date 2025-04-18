/**
   Main
   ====

  Copy binary to output (dist) folder:
  copy /Y ..\output\power.ino.zip dist
*/

/*
ToDo:
-cadence falsch?
-testen mit uhr
-temperatur immer 25°?
*/

#include "Arduino_LSM6DS3.h"
#include "Adafruit_Sensor.h"
#include "board.h"
#include <Wire.h>
#include <bluefruit.h>
#include <SPI.h>
#include "HX711_ADC.h"
#include <variant.h>
#include <nrf52840.h>


#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Adafruit_TinyUSB.h> // for Serial

//#define DEBUG

#define DEV_NAME "Cycle Power Meter"
//#define NRF52840_XXAA
#define gn 9.81 // gravity constant

// Virtufit Etappe I: crank length, in meters
// (to be added to the calibration procedure!)
#define CRANK_RADIUS 0.175

// Milliseconds to wait before go to sleep: 900000 = 15 minutes 
#define MILLIS_TO_SLEEP 900000 

// HX711 on-board hardware switch default is 10 Hz (alternative: 80 Hz)
#define HX711_RATE 80 

// Interval for publishing the cadence and power to bluetooth when not pedaling
#define STOPPED_BLE_UPDATE_INTERVAL 2000

// The minimum time between two measurements in milliseconds
// to prevent spikes in the avgForce calculation 
// The HX711 rate is 10 Hz, and the HX711 library smoothens the data over the last 32 samples = 3.2 seconds 
// and we assume that this 3.2 seconds is the slowest cycle possible (for which we still want all measurements)
#define CRANK_MINIMUM_ROTATION_TIME 500

// If the number of radians per seconds is less than this, we assume the user stopped pedaling
#define STAND_STILL_RPS (0.25 * PI)

// Pin-outs
#define LED_PIN LED_BUILTIN
#define SD_CS_PIN PIN_A3
#define GYRO_INT_PIN PIN_LSM6DS3TR_C_INT1

// Interrupt related variables (must be volatile)
volatile long timeFirstSleepCheck=0;
volatile long connectedStart=0;
volatile uint8_t newLoadDataReady=0;
volatile uint8_t newLoadDataReady_prev=0;
volatile uint8_t newZrotDataReady=0;
volatile uint8_t connection_count = 0; // bluetooth connection count
volatile int last_connection_count=0; // for automatically printing the help text

// Bluetooth
#define PWR_MEAS_CHAR_LEN 8 // Bluetooth package length
bool show_values=false; // print raw values
int16_t test_power=0; // for testing
uint16_t test_totalCrankRev=0; // for testing
uint16_t test_totalCrankRev_inc=0; // for testing

#define LOAD_OFFSET_DEFAULT 8745984
#define LOAD_MULTIPLIER_DEFAULT 1702.34
#define CALIBRATIONS_FILENAME    "/calibrations.txt"

typedef struct settings_struct {
  unsigned char calibrated; 

  int gyro_offset = 0;
  long load_offset = 0;
  float load_multiplier = 0.0;
} nvram_settings_struct;
nvram_settings_struct nvram_settings;

// Max. 2 hours of data @ 70 rpm
#define LASTSESSIONDATAINDEX_MAX 70*60*2

typedef struct lastSession_struct {
  uint16_t totalCrankRevs;
  long millis;
  uint16_t power;
  uint16_t cadence;
  uint16_t avgForce;
} lastSessionData_struct;
lastSessionData_struct lastSessionData[LASTSESSIONDATAINDEX_MAX]; 
long lastSessionDataIndex=0;

//HX711 pins:
#define HX711_dout D1 //mcu > HX711 dout pin (was: 4)
#define HX711_sck D0 //mcu > HX711 sck pin (was: 5)

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

//HX711 EEPROM calibration settings saving/loading
const int calVal_eepromAdress = 0;
unsigned long t = 0;

//Internal filesystem
using namespace Adafruit_LittleFS_Namespace;
File file(InternalFS);

// Session stats
static long lastSessionStart;
static long lastSessionEnd = 0;
static long lastSessionTotalCount = 0;
static float lastSessionTotalPower = 0.0;

// Last measured/calculated values 
// TODO: Localize these variables (to the main loop)
static long lastBluetoothUpdate = millis();
static float avgRad;
static float Zroll, Ztilt; 

static bool pedaling = false;
static uint16_t totalCrankRevs = 0; 
static float mps = 0.0;
static float avgForce = 0.0;
static int16_t power = 0;
static long bluetoothTime = 0; // the time as reported to the bluetooth host
static float force_avg, force_cnt;

// Initialize timers
static long lastMeasurement = millis();
static long lastStopMessage = millis();
static long lastBatteryUpdate = millis();


//
// Setup
//
void setup() {
  Wire.begin();

  Serial.begin(115200);

  Serial.println("Setup startet");
  int cnt=0;
  while ( !Serial && (cnt++ < 300)) delay(10);   // for nrf52840 with native usb

  timeFirstSleepCheck=0;
  lastSessionStart = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);

  // Setup, calibrate our other components
  gyroSetup();
  loadSetup();
  bleSetup();
  setupBattery();

  Serial.printf("Setup completed.\n\n");
  Serial.printf("Enter 'h' for help.\n\n");
  delay(200);
}

//
// Main loop
//
void loop() {
  // Get moving average velocity in rad per second
  float Zrot = getZrot();
  boolean angleEvent = calcAngle(Zrot); 
  avgRad = MA_cadence(Zrot, angleEvent);
   
  if (newLoadDataReady) {
    getCumulatedForce(&force_avg);
    force_cnt += 1.0;
    newLoadDataReady_prev = newLoadDataReady;
    newLoadDataReady = 0;     
  }
  // Get the crank Z position
  //getZtilt(&Zroll, &Ztilt);

  // Check if we stopped pedaling
  if ((avgRad <= STAND_STILL_RPS) && ((millis() - lastStopMessage) >= STOPPED_BLE_UPDATE_INTERVAL)) 
  {
    pedaling = false;

    publishAndStoreCycleInfo_Stopped();
  }
  else 
  {
    if (angleEvent == true)
    { 
      pedaling = true;
      
      publishAndStoreCycleInfo();
    }
  }

  // Print help text on bluetooth connection
  printHelpOnConnect();

  // Check if we can go to sleep
  gyroCheckSleepy(pedaling);

  // Publish battery-level over bluetooth every 5 minutes
  if ((millis() - lastBatteryUpdate) > (1000 * 60 * 5)) {
    blePublishBatt();
    lastBatteryUpdate = millis();
  }

  // Read user input
  readUserInput();
  
  // Request CPU to enter low-power mode until an event/interrupt occurs
  // but mind this bug: https://github.com/adafruit/Adafruit_nRF52_Arduino/issues/637
  waitForEvent();
}

// Publish and store cycle-info to the bluetooth host
void publishAndStoreCycleInfo() 
{
  //printfLog("force_cnt: %0.0f",force_cnt);
  //printfLog("force_avg: %0.0f",force_avg);
  // Get the moving average force from the load cell (library)
  avgForce = getAvgForce();

  // Get the circular velocity of the rider's foot in m/s
  mps = CRANK_RADIUS * avgRad; // (2*PI*r) * avgRad/(2*PI) = r * avgRad

  // Multiply it all by 2, because we only have the sensor on 1/2 the cranks
  power = 2.0 * mps * avgForce;

  // As per Zrot/Ztilt measurement, 1 full crank-rotatation has been performed
  totalCrankRevs++;

  // Because the published cadence is determined by the delta-time (in ms),
  // we estimate the delta-time from the last gyroscope crank-speed measurements (avgRad)
  // assuming exactly 1 crank-rotation has passed.
  // This provides a smoother cadence profile than using millis() to calculate the passed time.
  float deltaTime = (millis() - lastMeasurement);//float deltaTime = 2000.f * PI / avgRad;  // 1000 [ms] * (2*PI) / avgRad 
  bluetoothTime += deltaTime;

  // Show the values (to check if the Ztilt is close to 0 when measuring)
  if (show_values) {
      printfLog("%.0fN, %.2fm/s, %dW, %0.0f/%0.0f, %d, %d/%d\n", avgForce, mps, power, Ztilt, Zroll,totalCrankRevs, newLoadDataReady_prev, newZrotDataReady);
  }

  // Publish the measurements to the bluetooth host
  blePublishPower(power, totalCrankRevs, bluetoothTime);

  // Update last session stats
  lastSessionTotalPower += power;
  lastSessionTotalCount++;
  lastSessionEnd = millis();

  // Store session data
  if (lastSessionDataIndex < LASTSESSIONDATAINDEX_MAX) {
    lastSessionData[lastSessionDataIndex].totalCrankRevs = totalCrankRevs;
    lastSessionData[lastSessionDataIndex].millis = deltaTime;
    lastSessionData[lastSessionDataIndex].power = (uint16_t) power;
    lastSessionData[lastSessionDataIndex].cadence = (uint16_t) ((60000.f/deltaTime)+0.5); // (60*1000/deltaTime) = cadence in rpm
    lastSessionData[lastSessionDataIndex].avgForce = (uint16_t) (avgForce+0.5);
    lastSessionDataIndex++;
  }

  // Reset timers
  newZrotDataReady = 0;
  lastMeasurement = millis();
  lastBluetoothUpdate = millis();
}

// Publish and store cycle-info to the bluetooth host when we are stopped (not pedaling)
void publishAndStoreCycleInfo_Stopped() 
{
  if((test_power>0) || (test_totalCrankRev_inc>0))
  {
    // We are faking a measurement
    test_totalCrankRev += test_totalCrankRev_inc;
    printfLog("Fake: Force=%d  Crank revs=%d\n", test_power, test_totalCrankRev);
    blePublishPower(test_power, test_totalCrankRev, millis());
  }
  else
  {
    // We are not pedaling. Report this to the bluetooth host
    blePublishPower(0, totalCrankRevs, millis()); // zero power, no cadence (resend same totalCrankRevs)
    if (show_values) {
        printfLog("%.1fN * %.2fm/s = %dW (Z=%0.0f/%0.0f, STOP)\n", getAvgForce(), CRANK_RADIUS * avgRad, 0, Ztilt, Zroll);
    }
  }
  
  // Reset timers
  newZrotDataReady = 0;
  lastMeasurement = millis();
  lastBluetoothUpdate = millis();
  lastStopMessage = millis();
}

// Print help text after an arbitrary wait time (to allow the user to press UART in the App)
void printHelpOnConnect() {
  if (connection_count > 0) {
    if(connection_count != last_connection_count) {
      if (connectedStart == 0) connectedStart = millis();
      if((millis() - connectedStart) > (1000*6))
      {
        last_connection_count = connection_count;
        connectedStart = 0;

        printHelp();
      }
    }
  }
  else
  {
    last_connection_count = 0;
  }
}

void printLastSessionStats() {
  float duration_min = (lastSessionEnd - lastSessionStart) / (1000.f * 60.f);
  if ((duration_min > 0) && (lastSessionTotalCount > 0) && (totalCrankRevs > 0)) {
    printfLog("Session stats (since last restart):\n");
    printfLog("Duration: %.0f minutes\n", duration_min);
    printfLog("   Power: %.1f W\n", lastSessionTotalPower / lastSessionTotalCount);
    printfLog(" Cadence: %.1f rpm (incl. stops)\n\n", totalCrankRevs / duration_min);
  }
  else {
    printfLog("No stats available yet.\n\n");
  }
}

void printLastSessionData() {
  float duration_min = (lastSessionEnd - lastSessionStart) / (1000.f * 60.f);
  if ((duration_min > 0) && (lastSessionTotalCount > 0) && (totalCrankRevs > 0)) {
    printfLog("crankRevs, crankRotation [ms], power [W], cadence [rpm], force [N]\n");
    for (long i = 0; i < lastSessionDataIndex; i++) {
      printfLog("%d, %d, %d, %d, %d\n", 
        lastSessionData[i].totalCrankRevs,
        lastSessionData[i].millis,
        lastSessionData[i].power,
        lastSessionData[i].cadence,
        lastSessionData[i].avgForce);
    }
  }
  else {
    printfLog("No data available yet.\n\n");
  }
}

void printHelp() {
  printfLog("=================\n");
  printfLog("Power Cycle Meter\n");
  printfLog("=================\n\n");

  if (nvram_settings.load_offset == LOAD_OFFSET_DEFAULT) {
    printfLog("\nLoad-cell defaults loaded:\n");
  }
  printfLog("    Load offset cal: %d\n",nvram_settings.load_offset);
  printfLog("Load multiplier cal: %.1f\n\n",nvram_settings.load_multiplier); 

  printfLog("        Temperature: %.1f Celsius\n", getTemperature());
  blePublishBatt(); // Broadcast and show battery level 

  printfLog("Commands:\n");
  printfLog(" h : show this Help text\n");
  printfLog(" l : show Last session stats\n");
  printfLog(" d : show last session Data (csv)\n");
  printfLog(" m : Monitor power & cadence\n");
  printfLog(" f : Fake power & cadence\n");
  printfLog(" c : Calibrate load sensor\n");
  printfLog(" s : enter Sleep mode\n");
  printfLog(" g : debug Gyro-Data\n");
  printfLog("\n");
}

//
// Read user input (via serial and/or bluetooth)
//
void readUserInput() {
  char buf[64]={'\0'};
  GetUserInput(buf);
  if (buf[0] == 'g') debugGyro();
  if (buf[0] == 'c') calibrateLoadCell();
  if (buf[0] == 's') enterSleepMode();
  if (buf[0] == 'h') printHelp(); 
  if (buf[0] == 'l') printLastSessionStats(); 
  if (buf[0] == 'd') printLastSessionData(); 
  if (buf[0] == 'f') {
    if (test_power > 0) {
      test_power = 0;
      test_totalCrankRev = 0;
      test_totalCrankRev_inc = 0;
    }
    else {
      testBT(); //test bluetooth
    }
  }
  if (buf[0] == 'm') {
    if (show_values) {
      show_values = false;
    }
    else {
      printfLog("avgForce, mps, power, Ztilt/Zroll,totalCrankRevs, #LoadData/#ZrotData\n");
      show_values = true;
    }
  }
}
float MA_cadence(float value, boolean event) {
    static const int maxValues = 256; // Maximale Anzahl an Werten
    static float values[maxValues]; // Array für Werte
    static int current = 0; // Index für den aktuellen Wert
    static int cvalues = 0; // Anzahl der gelesenen Werte (<= maxValues)
    static float sum = 0.0; // Rolling Sum
    static int cnt = 0; // Zähler für Aufrufe zwischen Events

    static unsigned long prevTime1 = 0;
    unsigned long thisTime1 = millis();

    if((thisTime1 - prevTime1) >= 10){
      
      cnt++; // Erhöhe den Zähler bei jedem Aufruf
      //if (cnt >= maxValues) cnt = maxValues;
  
      // Füge den neuen Wert zur Summe hinzu und speichere ihn im Array
      if (cvalues < maxValues) {
          values[current] = value; // Speichere den neuen Wert
          sum += value; // Neuen Wert zur Summe hinzufügen
          current++; // Erhöhe den aktuellen Index
  
          if (current >= maxValues) current = 0; // Index zurücksetzen, wenn er das Ende erreicht hat
  
          cvalues++; // Erhöhe die Anzahl der gelesenen Werte nur bis zur maximalen Größe
      } else {
          sum -= values[current]; // Ältesten Wert von der Summe abziehen
          values[current] = value; // Ersetze den ältesten Wert mit dem neuesten
          sum += value; // Neuen Wert zur Summe hinzufügen
  
          current++; // Erhöhe den aktuellen Index
  
          if (current >= maxValues) current = 0; // Index zurücksetzen, wenn er das Ende erreicht hat
      }
      prevTime1 = thisTime1;
    }
    if (event) {
        //printfLog("cnt: %d\n", cnt);
        if (cnt > 0) { 
            sum = 0.0;     // Summe zurücksetzen
            //cvalues = 0; // Anzahl der gültigen Werte zurücksetzen

            for (int i = 0; i < cnt && i < maxValues; i++) {
                sum += values[(current - (cnt%maxValues) + i + maxValues) % maxValues]; 
                //cvalues++;
            }
        }else{
          return 0.0;
        }
        int cnt_help = cnt;
        cnt = 0;
        //printfLog("sum: %f.0\n", sum);
        return sum/float(cnt_help);
        //current = 0; // Aktuellen Index zurücksetzen
    }else{
      return sum/float(cvalues);
    }

}

float getAvgForce(){
  float smth = force_avg;
  float smth2 = force_cnt;
  force_avg = 0.0;
  force_cnt = 0.0;
  return (smth/smth2);
}

boolean calcAngle(float gyro){
  
  static unsigned long prevTime = 0;
  unsigned long thisTime = millis();
  bool crankEvent = false; 
  float angleChange = 0.0;
  static float lastAngle = 0.0;
  
  unsigned long timediff = thisTime - prevTime;
  gyro = gyro * 180.0 / PI;

  if(timediff >= 10){
    angleChange = gyro * timediff / 1000.0; // deg/s * s = deg    
    if(angleChange > 0.0f){    //nur vorwärts wird gezählt
      lastAngle = lastAngle + angleChange;     
    }    
    prevTime = thisTime;
  } 
  
  if(lastAngle >= 360.0f){
    lastAngle = 0.0;
    crankEvent = true;    
  }  
  //update LCET and cumrev every 360° revolution of crank
  if (crankEvent){    
    crankEvent = false;
    return true;
  } else {
    return false;
  }    
  
}
