// Enable and select radio type attached
#define MY_RADIO_RF24

#define MY_NODE_ID 1
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC
#define MY_DEBUG

#include <MySensors.h>  
#include <OneWire.h>
#include <TempSensorDiscovery.h>
#include <util/atomic.h>

#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define ONE_WIRE_BUS 8 // Pin where dallase sensor is connected 
#define SLEEP_TIME_MS 5000 // Sleep time between reads (in milliseconds)
#define HOT_WATER_INTERRUPT_PIN 2
#define COLD_WATER_INTERRUPT_PIN 3

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature tempSensors(&oneWire);
TempSensorDiscovery tempSensorDiscovery(&tempSensors);

long lastSendTime = 0;
float lastTemperature[TEMP_DISCOVERY_MAX_ATTACHED_DS18B20];
bool receivedConfig = false;
bool metric = true;
// Initialize temperature message
MyMessage temperatureMsg(0, V_TEMP);
MyMessage waterFlowMsg(0, V_FLOW);
MyMessage waterVolumeMsg(0, V_VOLUME);

uint8_t sensorsIndicies[TEMP_DISCOVERY_MAX_ATTACHED_DS18B20];
uint8_t tempSensorsCount;

volatile long hotWaterCounter;
volatile long coldWaterCounter;

void hotWaterISR();
void coldWaterISR();
float getWaterVolume(long sensorPulses);
float getWaterFlow(long sensorPulses, long inTime);

void before()
{
  
}

void setup()  
{ 
   Serial.begin(MY_BAUD_RATE);
   // requestTemperatures() will not block current thread
   tempSensors.setCheckForConversion(true);
   tempSensorsCount = tempSensorDiscovery.discover(sensorsIndicies);   

   // Set interrupts for water flow sensors
   attachInterrupt(digitalPinToInterrupt(HOT_WATER_INTERRUPT_PIN), hotWaterISR, FALLING);
   attachInterrupt(digitalPinToInterrupt(COLD_WATER_INTERRUPT_PIN), coldWaterISR, FALLING);
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Boiler room monitor", "0.0.1");

  // Send initial temp sensor
  for (uint8_t i=0; i<tempSensorsCount; i++) {
     present(sensorsIndicies[i], S_TEMP);
  }
  // Hot water flow meter
  present(20, S_WATER);
  // Cold water flow meter
  present(21, S_WATER);
}

void loop()     
{     
  // Fetch temperatures from Dallas sensors
  tempSensors.requestTemperatures();
  unsigned long realEplasedTime = millis() - lastSendTime;
  if (realEplasedTime > SLEEP_TIME_MS) {
    
    lastSendTime = millis();

    long hotWaterImpulses, coldWaterImpulses;
    // code with interrupts blocked (consecutive atomic operations will not get interrupted)
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      hotWaterImpulses = hotWaterCounter;
      hotWaterCounter = 0;

      coldWaterImpulses = coldWaterCounter;
      coldWaterCounter = 0;      
    }

    send(waterVolumeMsg.setSensor(20).set(getWaterVolume(hotWaterImpulses), 2));
    send(waterFlowMsg.setSensor(20).set(getWaterFlow(hotWaterImpulses, realEplasedTime), 2));
    
    send(waterVolumeMsg.setSensor(21).set(getWaterVolume(coldWaterImpulses), 2));
    send(waterFlowMsg.setSensor(21).set(getWaterFlow(coldWaterImpulses, realEplasedTime), 2));


    if (tempSensors.isConversionComplete()) {
      
      for (uint8_t i=0; i<tempSensorsCount; i++) {
        DeviceAddress addr;
        tempSensorDiscovery.loadSensorAddressByIndex(addr, sensorsIndicies[i]);

        if (tempSensors.isConnected(addr)) {
          // Fetch and round temperature to one decimal
          float temperature = static_cast<float>(static_cast<int>((getControllerConfig().isMetric?tempSensors.getTempCByIndex(i):tempSensors.getTempFByIndex(i)) * 10.)) / 10.;
          if (temperature != -127.00 && temperature != 85.00) {
            send(temperatureMsg.setSensor(i).set(temperature, 1));
            continue;
          }            
        } 
        // error
      }
    }
  }
}



void hotWaterISR() {
  hotWaterCounter++;
}

void coldWaterISR() {
  coldWaterCounter++;
}

float getWaterVolume(long sensorPulses) {
  return float(sensorPulses) / 288.0;
}

float getWaterFlow(long sensorPulses, long inTime) {
  return float(sensorPulses) * (60000.0 / float(inTime)) / (4.8 * 60.0);
}