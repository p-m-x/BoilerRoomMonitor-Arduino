/**
 * @def TEMP_DISCOVERY_EEPROM_START
 * @brief EEPROM start byte for Temperature sensor discovery logic
 *
 * Needs 9 * TEMP_DISCOVERY_MAX_ATTACHED_DS18B20 bytes of EEPROM
 */
#ifndef TEMP_DISCOVERY_EEPROM_START
#define TEMP_DISCOVERY_EEPROM_START 512
#endif

/**
 * @def TEMP_DISCOVERY_MAX_ATTACHED_DS18B20
 * @brief Max count of attached DS18B20 devices
 */
#ifndef TEMP_DISCOVERY_MAX_ATTACHED_DS18B20
#define TEMP_DISCOVERY_MAX_ATTACHED_DS18B20 16
#endif

#include <DallasTemperature.h>
#include <EEPROM.h>

class TempSensorDiscovery {
    public: 
        TempSensorDiscovery(DallasTemperature *sensors);
        uint8_t discover(uint8_t (&indicies)[TEMP_DISCOVERY_MAX_ATTACHED_DS18B20]);
        void loadSensorAddressByIndex(DeviceAddress addr, uint8_t index);
    private:
        DallasTemperature *_sensors;
        uint8_t _sensorCount = 0;
        int calculateMemStart(uint8_t index);
        int findSensorIndex(DeviceAddress addr);
        bool isActive(uint8_t index);
        int findFirstFreeSensorIndex();        
        void saveSensorAddressAtIndex(DeviceAddress addr, uint8_t index);        
        void printAllSensorsData();
        void printSensorData(uint8_t index);
        void printSensorAddress(DeviceAddress addr);

};
