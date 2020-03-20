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
#define SLEEP_TIME_MS 10000 // Sleep time between reads (in milliseconds)
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
MyMessage msgTemperature(0, V_TEMP);
MyMessage msgWaterFlow(0, V_FLOW);
MyMessage msgWaterVolume(0, V_VOLUME);
MyMessage msgInfo(0, V_TEXT);

uint8_t tempSensorsIndicies[TEMP_DISCOVERY_MAX_ATTACHED_DS18B20];
uint8_t tempSensorsCount;

volatile long hotWaterCounter;
volatile long coldWaterCounter;

float hotWaterTotalVolumeLiters = 0.000;
float coldWaterTotalVolumeLiters = 0.000;

float lastValues[25];
boolean sensorsInitialized = false;

void hotWaterISR();
void coldWaterISR();
float getWaterVolume(long sensorPulses);
float getWaterFlow(float flowedVolume, long inTime);
bool checkIfSensorValueChanged(uint8_t sensorID, float currentValue);
void setSensorLastValue(uint8_t sensorID, float value);
float getSensorLastValue(uint8_t sensorID);
void receive(const MyMessage &message);

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
  sendSketchInfo("Boiler room monitor", "0.0.2");
  for (uint8_t i=0; i<tempSensorsCount; i++) {
    present(i, S_TEMP, "Temperature sensor");     
    present(i, S_INFO, "Info");
  }
  // Hot water flow meter
  present(HOT_WATER_METER_SENSOR_ID, S_WATER, "Hot water flow meter");
  // Cold water flow meter
  present(COLD_WATER_METER_SENSOR_ID, S_WATER, "Cold water flow meter");
  
}

void loop()     
{     

  if (!sensorsInitialized) {
    Serial.println("Sending initial value");
    for (uint8_t i=0; i<tempSensorsCount; i++) {
      send(msgTemperature.setSensor(i).set(0));   
    }
    send(msgWaterFlow.setSensor(HOT_WATER_METER_SENSOR_ID).set(0));
    send(msgWaterFlow.setSensor(COLD_WATER_METER_SENSOR_ID).set(0));

    send(msgWaterVolume.setSensor(HOT_WATER_METER_SENSOR_ID).set(0));
    send(msgWaterVolume.setSensor(COLD_WATER_METER_SENSOR_ID).set(0));

    Serial.println("Requesting initial value from controller");
    for (uint8_t i=0; i<tempSensorsCount; i++) {
      request(i, V_TEMP);
      wait(2000, C_SET, V_TEMP);
    }

    request(HOT_WATER_METER_SENSOR_ID, V_FLOW);
    wait(2000, C_SET, V_FLOW);
    request(COLD_WATER_METER_SENSOR_ID, V_FLOW);
    wait(2000, C_SET, V_FLOW);
    request(HOT_WATER_METER_SENSOR_ID, V_VOLUME);
    wait(2000, C_SET, V_VOLUME);    
    request(COLD_WATER_METER_SENSOR_ID, V_VOLUME);
    wait(2000, C_SET, V_VOLUME);   

    return;
  }
  // Fetch temperatures from Dallas sensors
  tempSensors.requestTemperatures();
  unsigned long realEplasedTime = millis() - lastSendTime;
  boolean fullSend = false;
  if ((millis() - lastFullSendTime) > FULL_SEND_INTERVAL) {
    fullSend = true;
    lastFullSendTime = millis();
    sendHeartbeat();
    Serial.println("FULL SEND");
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
      send(msgWaterVolume.setSensor(HOT_WATER_METER_SENSOR_ID).set(hotWaterTotalVolumeLiters, 3));
      setSensorLastValue(SV_HOT_WATER_VOLUME, hotWaterTotalVolumeLiters);
    }
    if (checkIfSensorValueChanged(SV_HOT_WATER_FLOW, hotWaterFlow) || fullSend) {
      digitalWrite(STATUS_LED, LOW);
      send(msgWaterFlow.setSensor(HOT_WATER_METER_SENSOR_ID).set(hotWaterFlow, 2));
      setSensorLastValue(SV_HOT_WATER_FLOW, hotWaterFlow);
    }
    
    coldWaterTotalVolumeLiters = coldWaterTotalVolumeLiters + getWaterVolume(coldWaterImpulses);
    float coldWaterFlow = getWaterFlow(coldWaterTotalVolumeLiters - getSensorLastValue(SV_COLD_WATER_VOLUME), realEplasedTime);

    if (checkIfSensorValueChanged(SV_COLD_WATER_VOLUME, coldWaterTotalVolumeLiters) || fullSend) {
      digitalWrite(STATUS_LED, LOW);
      send(msgWaterVolume.setSensor(COLD_WATER_METER_SENSOR_ID).set(coldWaterTotalVolumeLiters, 3));
      setSensorLastValue(SV_COLD_WATER_VOLUME, coldWaterTotalVolumeLiters);
    }
    if (checkIfSensorValueChanged(SV_COLD_WATER_FLOW, coldWaterFlow) || fullSend) {
      digitalWrite(STATUS_LED, LOW);
      send(msgWaterFlow.setSensor(COLD_WATER_METER_SENSOR_ID).set(coldWaterFlow, 2));
      setSensorLastValue(SV_COLD_WATER_FLOW, coldWaterFlow);
    }
    
    if (tempSensors.isConversionComplete()) {
      
      for (uint8_t i=0; i<tempSensorsCount; i++) {
        DeviceAddress addr;
        tempSensorDiscovery.loadSensorAddressByIndex(addr, tempSensorsIndicies[i]);

        if (tempSensors.isConnected(addr)) {
          // Fetch and round temperature to one decimal
          float temperature = static_cast<float>(static_cast<int>((getControllerConfig().isMetric?tempSensors.getTempCByIndex(i):tempSensors.getTempFByIndex(i)) * 10.)) / 10.;
          if (checkIfSensorValueChanged(i, temperature) || fullSend) {
            digitalWrite(STATUS_LED, LOW);
            setSensorLastValue(i, temperature);
            if (temperature != -127.00 && temperature != 85.00) {                            
              send(msgTemperature.setSensor(i).set(temperature, 1));
              send(msgInfo.setSensor(i).set(""));
              continue;
            } else {
              send(msgInfo.setSensor(i).set("Read error"));
            }
          } 
        } else {
          setSensorLastValue(i, -127.00);
          send(msgInfo.setSensor(i).set("Not connected"));
        }
      }
    }
    digitalWrite(STATUS_LED, HIGH);
  }
}

void receive(const MyMessage &message) 
{
  if (message.isAck()) {
     Serial.println("This is an ack from gateway");
     return;
  }

  if (!sensorsInitialized && message.getType() == V_TEMP) {
    Serial.println("Receiving initial value from controller");
    sensorsInitialized = true;
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
