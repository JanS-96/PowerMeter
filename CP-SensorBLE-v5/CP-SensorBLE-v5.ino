#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include <nrf52840.h>
#include <nrfx_saadc.h>
#include <AnalogIn.h>
#include <pinDefinitions.h>
#include "HX711.h"
#include "mbed.h"

using namespace std::chrono;

typedef struct CPMStruct{
  int16_t Flags; //B0011010110010000
  int16_t InstPower; // Watt
  uint16_t CumRev; // unitless
  uint16_t LastCrEvtTime; // 1/1024s, M=1 b =-10   
}__attribute__((packed));

const int cpm_union_size = sizeof(CPMStruct);
typedef union btPacket{
  CPMStruct cpvalues;
  byte byteArray[cpm_union_size];
};
btPacket cpm;

byte SensLoc = 6; //Right Crank Location
int oldBatteryLevel = 0;  // last battery level reading from analog input
unsigned long features = 0x00210008; //verändert bis alles funktioniert //B01011101000000001000100000000000 LSB->MSB, LSO->MSO 0xBA001100
const int ledPin = LED_BUILTIN; // pin to use for the LED
const int sckPin = D0;
const int dtPin = D1;
int tempe = 0;
float temperatur = 0.00;
float crankLength = 0.175;//meter
long zeroOffset = 0;
float calibrationValue = 1702.34; //hardcoded for now, change for 1kg / 9,81
float force = 0.0;
float work = 0.0;
byte desc[] = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00,0x01, 0x00, 0x00, 0x14, 0xFF};
float lastAngle = 0;
float compAngle = 0.0f;

HX711 scale;

BLEService CPService("1818"); //Cycling Power Service
BLEService BatteryService("180F"); 
BLEService tempService("181A"); // Bluetooth® Low Energy environmental sensing Service
BLEIntCharacteristic switchCharacteristic("2A6E", BLERead | BLENotify);
BLEDescriptor ESMeasurement("290C", desc, 11);
BLEDescriptor chName("2901", "Temperatur");
BLEUnsignedCharCharacteristic BatteryLevel("2A19", BLERead | BLENotify);
BLECharacteristic CPMeasurement("2A63", BLENotify, 8);//19 Octets
BLEUnsignedLongCharacteristic CPFeature("2A65", BLERead);//4 Octets
BLEByteCharacteristic SensorLocation("2A5D", BLERead);

void setup() {
  Serial.begin(115200);
  //while(!Serial); 
    
  pinMode(ledPin, OUTPUT);
  pinMode(P0_14, OUTPUT);
  digitalWrite(P0_14, LOW);

  scale.begin(dtPin, sckPin);  

  delay(1000);
  byte zerocnt = 0;
  while(zerocnt < 20){
    if(scale.is_ready()){
      zeroOffset += scale.read();
      zerocnt++;
    }
  }
  zeroOffset = zeroOffset / zerocnt;

  scale.power_down();

  cpm.cpvalues = {
    0x0028,
    0,
    0,
    0    
  };
  
  // add Descriptors to Characteristics
  switchCharacteristic.addDescriptor(ESMeasurement);//temp
  switchCharacteristic.addDescriptor(chName);//temp
  
  BLEinit();
  
  // start advertising
  BLE.advertise();
}

void loop() { 
  
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
    if (myIMU.begin() != 0) {
      while(1);//Serial.println("Device error");
    }else{
      //Serial.println("Device OK!");
    }
    scale.power_up();
    
    while (central.connected()) {       
      thread_sleep_for(1);          
//      if(BatteryLevel.value() <= 10){
//        NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter; // Battery Protection
//      }      
      getScaleData(false);
      
      unsigned long thisTime = millis();       
      static unsigned long oldTime = 0;                   
      float gyro = myIMU.readFloatGyroZ();
      uint16_t lcet = (uint16_t)(calcCrankData(gyro)*1024.0f/1000.0f);      
      
      if((thisTime-oldTime) > 1000){                
        monitor_battery_level();        
        cpm.cpvalues.LastCrEvtTime = lcet;     
        calcInstPower();        

        CPMeasurement.writeValue(&cpm.byteArray, 8);               
        temperatur = myIMU.readTempC();             
        temperatur = temperatur*100;
        tempe = int(temperatur);
        switchCharacteristic.writeValue(tempe);        
        oldTime = thisTime;     
      }     
    }
    scale.power_down();
    cpm.cpvalues.InstPower = 0;
    work = 0;
    BLE.advertise();
  }  
}

void BLEinit(void){
  if (!BLE.begin()) {
    //Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }
  
  // set advertised local name and service UUID:
  BLE.setLocalName("PowerMeter");
  BLE.setDeviceName("PowerMeter");
  BLE.setAppearance(0x0484);
  BLE.setAdvertisedService(CPService);
 
  // add the characteristic to the service
  CPService.addCharacteristic(CPFeature);
  CPService.addCharacteristic(CPMeasurement);  
  CPService.addCharacteristic(SensorLocation);
  tempService.addCharacteristic(switchCharacteristic);
  BatteryService.addCharacteristic(BatteryLevel);

  // add service
  BLE.addService(CPService);
  BLE.addService(BatteryService);
  BLE.addService(tempService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);
  CPFeature.writeValue(features);
  CPMeasurement.writeValue(&cpm.byteArray, 8);
  SensorLocation.writeValue(SensLoc);
  monitor_battery_level();
}

void calcInstPower(void){  
  if(compAngle < 10.0f){
    cpm.cpvalues.InstPower = 0; 
  }  
  compAngle = 0.0f;  
}

unsigned long calcCrankData(float gyro){  
  static uint16_t cumrev = 0;  
  static unsigned long prevEvent = 0;
  static unsigned long prevTime = 0;
  unsigned long thisTime = millis();
  bool crankEvent = false; 
  float angleChange = 0.0;
  
  uint16_t timediff = thisTime - prevTime;

  if(timediff >= 10){
    angleChange = gyro * timediff / 1000.0; // deg/s * s = deg    
    if(angleChange > 0.0f){    //nur vorwärts wird gezählt
      lastAngle = lastAngle + angleChange;
      compAngle += angleChange;
    }    
    prevTime = thisTime;
  } 
  
  if(lastAngle >= 360.0f){
    lastAngle = 0;
    crankEvent = true;    
  }  
  //update LCET and cumrev every 360° revolution of crank
  if (crankEvent){
    cumrev += 1;    
    cpm.cpvalues.CumRev = cumrev;

    getScaleData(true);
    cpm.cpvalues.InstPower = (uint16_t)(2.0f * work / ((thisTime - prevEvent) / 1000.0f)); // verdoppelt wegen einseitiger messung
           
    prevEvent = thisTime;
    crankEvent = false;
  }    
  return prevEvent;
}

long scaleReading(void){
  static long reading = 0;
  static long mean = 0;    
  static int counter = 0;
 
  if(counter == 10){
    mean = (long)(reading / 10);
    reading = 0;    
    counter = 0;  
  }
  if(scale.is_ready()){
    reading += scale.read();     
    counter+=1; 
  }
  return mean;
}

void getScaleData(boolean crEvent){
  static long sclread = 0;  
  float force = 0.0;  
  static int counter = 0;  

  if(crEvent){    
    sclread = sclread / counter; //average force over rotation
    force = (float)((sclread - zeroOffset) / calibrationValue);
    work = force * crankLength * 2.0 * PI;
    
    counter = 0;
    sclread = 0;
  }else{
    sclread += scaleReading();      
    counter+=1;
  }    
}

class HackAnalogIn: public mbed::AnalogIn 
{
  using mbed::AnalogIn::AnalogIn;
  public:
    analogin_t getAnalogIn_t();
};

analogin_t HackAnalogIn::getAnalogIn_t() 
{
  return this->_adc;
}

void startReadingBatteryLevel(nrf_saadc_value_t* buffer) 
{
  auto pin = PIN_VBAT;
  PinName name = analogPinToPinName(pin);
  if (name == NC)
  {
    return;
  }
  HackAnalogIn* adc = static_cast<HackAnalogIn*>(analogPinToAdcObj(pin));
  if (adc == NULL)
  {
    adc = new HackAnalogIn(name);
    analogPinToAdcObj(pin) = static_cast<mbed::AnalogIn*>(adc);
#ifdef ANALOG_CONFIG
    if (isAdcConfigChanged)
    {
      adc->configure(adcCurrentConfig);
    }
#endif
  }

  nrfx_saadc_buffer_convert(buffer, 1);
  nrfx_err_t ret = nrfx_saadc_sample();
  if (ret == NRFX_ERROR_BUSY)
  {
    // failed to start sampling
    return;
  }
}

nrf_saadc_value_t BatteryLvl = { 0 };

float vBat = 0.0;

void monitor_battery_level(){
  // Monitor the Battery Level
  startReadingBatteryLevel(&BatteryLvl);
  // check if ADC conversion has completed
  if (nrf_saadc_event_check(NRF_SAADC_EVENT_DONE))
  {
    // ADC conversion completed. Reading is stored in BatteryLvl
    nrf_saadc_event_clear(NRF_SAADC_EVENT_DONE);
     vBat = (float)BatteryLvl / 4096 * 3.43 / 510 * (1000 + 510);
    // write value to characteristic or things you want to do

    if (BatteryLvl != oldBatteryLevel){            
      int batLvl = mapVolt(vBat);
      BatteryLevel.writeValue(batLvl);
      //Serial.println(vBat);       
      oldBatteryLevel = BatteryLvl;     
    }
  }
}
int mapVolt(float x)
{
  int percent = 0;
  float volt[] = {3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82, 3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20};
  for(int i = 20; i >= 0; i--){
    if(x <= volt[i]){
      percent = i*5;
    }     
  }
  if(x > 4.20){
    percent = 100;
  }
  return percent;
}
