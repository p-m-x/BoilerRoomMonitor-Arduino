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
#define ONE_WIRE_BUS 4 // Pin where dallase sensor is connected 
#define STATUS_LED 6
#define SLEEP_TIME_MS 1000 // Sleep time between reads (in milliseconds)
#define FULL_SEND_INTERVAL 60000

#define HOT_WATER_INTERRUPT_PIN 2
#define COLD_WATER_INTERRUPT_PIN 3
#define HOT_WATER_METER_SENSOR_ID TEMP_DISCOVERY_MAX_ATTACHED_DS18B20 + 1
#define COLD_WATER_METER_SENSOR_ID TEMP_DISCOVERY_MAX_ATTACHED_DS18B20 + 2

// Indicies for values stored in lastValues array
#define SV_HOT_WATER_VOLUME TEMP_DISCOVERY_MAX_ATTACHED_DS18B20 + 1
#define SV_HOT_WATER_FLOW TEMP_DISCOVERY_MAX_ATTACHED_DS18B20 + 2
#define SV_COLD_WATER_VOLUME TEMP_DISCOVERY_MAX_ATTACHED_DS18B20 + 3
#define SV_COLD_WATER_FLOW TEMP_DISCOVERY_MAX_ATTACHED_DS18B20 + 4

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature tempSensors(&oneWire);
TempSensorDiscovery tempSensorDiscovery(&tempSensors);

long lastSendTime = 0;
long lastFullSendTime = 0;
bool receivedConfig = false;
bool metric = true;
// Initialize temperature message
MyMessage temperatureMsg(0, V_TEMP);
MyMessage waterFlowMsg(0, V_FLOW);
MyMessage waterVolumeMsg(0, V_VOLUME);

uint8_t tempSensorsIndicies[TEMP_DISCOVERY_MAX_ATTACHED_DS18B20];
uint8_t tempSensorsCount;

volatile long hotWaterCounter;
volatile long coldWaterCounter;

float hotWaterTotalVolumeLiters = 0.000;
float coldWaterTotalVolumeLiters = 0.000;

float lastValues[25];

void hotWaterISR();
void coldWaterISR();
float getWaterVolume(long sensorPulses);
float getWaterFlow(float flowedVolume, long inTime);
bool checkIfSensorValueChanged(uint8_t sensorID, float currentValue);
void setSensorLastValue(uint8_t sensorID, float value);
float getSensorLastValue(uint8_t sensorID);

void before()
{
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  tempSensorsCount = tempSensorDiscovery.discover(tempSensorsIndicies); 
}

void setup()  
{ 
  pinMode(HOT_WATER_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(COLD_WATER_INTERRUPT_PIN, INPUT_PULLUP);
  
  Serial.begin(MY_BAUD_RATE);

  // requestTemperatures() will not block current thread
  tempSensors.setCheckForConversion(true);    

  // Set interrupts for water flow sensors
  attachInterrupt(digitalPinToInterrupt(HOT_WATER_INTERRUPT_PIN), hotWaterISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(COLD_WATER_INTERRUPT_PIN), coldWaterISR, FALLING);
  _delay_ms(1000);
  digitalWrite(STATUS_LED, HIGH);
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Boiler room monitor", "0.0.1");

  // Send initial temp sensor
  for (uint8_t i=0; i<tempSensorsCount; i++) {
     present(tempSensorsIndicies[i], S_TEMP);
  }
  // Hot water flow meter
  present(HOT_WATER_METER_SENSOR_ID, S_WATER);
  // Cold water flow meter
  present(COLD_WATER_METER_SENSOR_ID, S_WATER);
}

void loop()     
{     
  // Fetch temperatures from Dallas sensors
  tempSensors.requestTemperatures();
  unsigned long realEplasedTime = millis() - lastSendTime;
  boolean fullSend = false;
  if ((millis() - lastFullSendTime) > FULL_SEND_INTERVAL) {
    fullSend = true;
    lastFullSendTime = millis();
    sendHeartbeat();
  }
  
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

    hotWaterTotalVolumeLiters = hotWaterTotalVolumeLiters + getWaterVolume(hotWaterImpulses);
    float hotWaterFlow = getWaterFlow(hotWaterTotalVolumeLiters - getSensorLastValue(SV_HOT_WATER_VOLUME), realEplasedTime);

    if (checkIfSensorValueChanged(SV_HOT_WATER_VOLUME, hotWaterTotalVolumeLiters) || fullSend) {
      digitalWrite(STATUS_LED, LOW);
      send(waterVolumeMsg.setSensor(HOT_WATER_METER_SENSOR_ID).set(hotWaterTotalVolumeLiters, 3));
      setSensorLastValue(SV_HOT_WATER_VOLUME, hotWaterTotalVolumeLiters);
    }
    if (checkIfSensorValueChanged(SV_HOT_WATER_FLOW, hotWaterFlow) || fullSend) {
      digitalWrite(STATUS_LED, LOW);
      send(waterFlowMsg.setSensor(HOT_WATER_METER_SENSOR_ID).set(hotWaterFlow, 2));
      setSensorLastValue(SV_HOT_WATER_FLOW, hotWaterFlow);
    }
    
    coldWaterTotalVolumeLiters = coldWaterTotalVolumeLiters + getWaterVolume(coldWaterImpulses);
    float coldWaterFlow = getWaterFlow(coldWaterTotalVolumeLiters - getSensorLastValue(SV_COLD_WATER_VOLUME), realEplasedTime);

    if (checkIfSensorValueChanged(SV_COLD_WATER_VOLUME, coldWaterTotalVolumeLiters) || fullSend) {
      digitalWrite(STATUS_LED, LOW);
      send(waterVolumeMsg.setSensor(COLD_WATER_METER_SENSOR_ID).set(coldWaterTotalVolumeLiters, 3));
      setSensorLastValue(SV_COLD_WATER_VOLUME, coldWaterTotalVolumeLiters);
    }
    if (checkIfSensorValueChanged(SV_COLD_WATER_FLOW, coldWaterFlow) || fullSend) {
      digitalWrite(STATUS_LED, LOW);
      send(waterFlowMsg.setSensor(COLD_WATER_METER_SENSOR_ID).set(coldWaterFlow, 2));
      setSensorLastValue(SV_COLD_WATER_FLOW, coldWaterFlow);
    }
    
    if (tempSensors.isConversionComplete()) {
      
      for (uint8_t i=0; i<tempSensorsCount; i++) {
        DeviceAddress addr;
        tempSensorDiscovery.loadSensorAddressByIndex(addr, tempSensorsIndicies[i]);

        if (tempSensors.isConnected(addr)) {
          // Fetch and round temperature to one decimal
          float temperature = static_cast<float>(static_cast<int>((getControllerConfig().isMetric?tempSensors.getTempCByIndex(i):tempSensors.getTempFByIndex(i)) * 10.)) / 10.;
          if (temperature != -127.00 && temperature != 85.00 && (checkIfSensorValueChanged(i, temperature) || fullSend)) {
            digitalWrite(STATUS_LED, LOW);
            setSensorLastValue(i, temperature);
            send(temperatureMsg.setSensor(i).set(temperature, 1));
            continue;
          }            
        } 
      }
    }
    digitalWrite(STATUS_LED, HIGH);
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

float getWaterFlow(float flowedVolume, long inTime) {
  return flowedVolume * (60000.0 / float(inTime));
}

bool checkIfSensorValueChanged(uint8_t sensorID, float currentValue) {
  return abs(lastValues[sensorID] - currentValue) > 0.000001;
}

void setSensorLastValue(uint8_t sensorID, float value) {
  lastValues[sensorID] = value;
}

float getSensorLastValue(uint8_t sensorID) {
  return lastValues[sensorID];
}
