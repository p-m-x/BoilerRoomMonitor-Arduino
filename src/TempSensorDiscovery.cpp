#include <TempSensorDiscovery.h>

TempSensorDiscovery::TempSensorDiscovery(DallasTemperature *sensors) {
    EEPROM.begin();
    _sensors = sensors;    
    _sensors->begin();
}

uint8_t TempSensorDiscovery::discover(uint8_t (&indicies)[TEMP_DISCOVERY_MAX_ATTACHED_DS18B20]) {
    
    _sensorCount = _sensors->getDS18Count();
    Serial.print("DS18B20 sensors count: ");
    Serial.println(_sensorCount);

    // Find if saved sensors are connected and mark it in EEPROM
    for (uint8_t i=0; i<TEMP_DISCOVERY_MAX_ATTACHED_DS18B20; i++) {
        if (!isActive(i)) {
            continue;
        }
        DeviceAddress sensorAddr;
        loadSensorAddressByIndex(sensorAddr, i);
        Serial.print("Searching sensor: ");
        printSensorAddress(sensorAddr);
        Serial.print(" connected: ");
        Serial.print(_sensors->isConnected(sensorAddr));
        Serial.println();
        EEPROM.update(calculateMemStart(i), _sensors->isConnected(sensorAddr));
    }

    // Now, find rest of DS18B20 devices and store them in EEPROM
    for (uint8_t i=0; i<_sensorCount; i++) {
        DeviceAddress tmpAddr;
        _sensors->getAddress(tmpAddr, i);
        Serial.print("Found DS18B20 ");
        printSensorAddress(tmpAddr);   
        int index = findSensorIndex(tmpAddr);
        if (index < 0) {
            Serial.print(", but not exists in EPPROM");
            index = findFirstFreeSensorIndex();
            if (index >=0) {
                Serial.print(", adding at index #");
                Serial.println(index);
                saveSensorAddressAtIndex(tmpAddr, index);
                continue;
            }
        } else {
            Serial.print(", exists in EEPROM at index #");
            Serial.println(index);
            EEPROM.update(calculateMemStart(index), 1);
            continue;
        }
        Serial.println(" ERROR");
    }

    printAllSensorsData();
    uint8_t deviceCount = 0;

    for (uint8_t i=0; i<TEMP_DISCOVERY_MAX_ATTACHED_DS18B20; i++) {
        if (isActive(i)) {
            indicies[deviceCount++] = i;
        }
    }
    return deviceCount;
}

int TempSensorDiscovery::calculateMemStart(uint8_t index) {
    return TEMP_DISCOVERY_EEPROM_START + (index * 9);
}

bool TempSensorDiscovery::isActive(uint8_t index) {
    int startAddr = calculateMemStart(index);
    return EEPROM.read(startAddr);
}

void TempSensorDiscovery::loadSensorAddressByIndex(DeviceAddress addr, uint8_t index) {
    // +1 - skip first byte - it's a isConnected flag
    int startAddr = calculateMemStart(index) + 1;    
    for (uint8_t i=0; i<sizeof(DeviceAddress); i++) {
        addr[i] = EEPROM.read(startAddr + i);
    }
}

void TempSensorDiscovery::saveSensorAddressAtIndex(DeviceAddress addr, uint8_t index) {
    int startAddr = calculateMemStart(index);
    Serial.print("saving ");
    // save isConnected flag
    EEPROM.update(startAddr, 1);
    Serial.print(startAddr);
    Serial.print(": ");
    Serial.print(EEPROM.read(startAddr), HEX);
    Serial.print(" ");
    startAddr++;

    for (uint8_t i=0; i<sizeof(DeviceAddress); i++) {
   
        EEPROM.update(startAddr + i, addr[i]);

        Serial.print(startAddr + i);
        Serial.print(": ");
        Serial.print(EEPROM.read(startAddr + i), HEX);
        Serial.print(", ");
    }
    Serial.println();
}

int TempSensorDiscovery::findSensorIndex(DeviceAddress addr) {
    for (uint8_t i=0; i<TEMP_DISCOVERY_MAX_ATTACHED_DS18B20; i++) {
        // +1 - skip first byte - it's a isConnected flag
        int startAddr = calculateMemStart(i) + 1;
        bool found = true;
        for (uint8_t b=0; b<8; b++) {
            if (addr[b] != EEPROM.read(startAddr + b)) {
                found = false;
                break;
            }
        }
        if (found) {
            return i;
        }
    }
    return -1;
}

int TempSensorDiscovery::findFirstFreeSensorIndex() {
    for (uint8_t i=0; i<TEMP_DISCOVERY_MAX_ATTACHED_DS18B20; i++) {
        if (EEPROM.read(calculateMemStart(i)) == 0) {
            return i;
        }
    }

    return -1;
}

void TempSensorDiscovery::printAllSensorsData() {
    for (uint8_t i=0; i<TEMP_DISCOVERY_MAX_ATTACHED_DS18B20; i++) {
        
        printSensorData(i);           
    }
}

void TempSensorDiscovery::printSensorData(uint8_t index) {
    int startAddr = calculateMemStart(index);
    DeviceAddress addr;
    loadSensorAddressByIndex(addr, index);    

    Serial.print(index, DEC);
    Serial.print(": |");
    Serial.print(EEPROM.read(startAddr), HEX);
    Serial.print("|");    
    printSensorAddress(addr);
    Serial.println("|");
}

void TempSensorDiscovery::printSensorAddress(uint8_t addr[8]) {
    for (uint8_t i=0; i<8; i++) {
        Serial.print(addr[i], HEX);
    }
}
