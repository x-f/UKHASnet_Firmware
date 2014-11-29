//************* Define Node that you're compiling for ****************/
#define XF0

//************* Node-specific config ****************/

// geitvejs
#ifdef XF0
  char id[] = "XF0";
  #define LOCATION_STRING "12.348,45.636"
  byte num_repeats = '3'; //The number of hops the message will make in the network
  #define BEACON_INTERVAL 300 // Beacon Interval in seconds
  uint8_t rfm_power = 15; // dBmW
  #define SENSITIVE_RX // Enables TESTLNA_SENSITIVE
  //#define ENABLE_RFM_TEMPERATURE // Comment out to disable
  //#define ENABLE_BATTV_SENSOR // Comment out to disable, you must also disable zombie mode
  //#define BATTV_PIN 3 //ADC 0 - Battery Voltage, scaled to 1.1V
  //#define BATTV_FUDGE 0.935 // Battery Voltage ADC Calibration Factor
  // Power Saving
  //#define ENABLE_ZOMBIE_MODE // Comment this out to disable
  //#define ZOMBIE_THRESHOLD 3.65 // Lower Voltage Threshold
  //#define ENABLE_UART_OUTPUT // UART output of packets and rssi, used for gateways
  #define ENABLE_ETHERNET // Ethernet shield, UDP
  #define RFM_SSpin 9
  #define ENABLE_BMP085 // BMP085 pressure and temperature sensor
  //#define ENABLE_BMP085_TEMP // BMP085 temperature
  #define ENABLE_BMP085_PRESSURE // BMP085 pressure
  #define BMP085_PWR A3
  #define ENABLE_DHT22 // DHT22 humidity and temperature sensor
  #define ENABLE_DHT22_TEMP // DHT22 temperature
  #define ENABLE_DHT22_HUMIDITY // DHT22 humidity
  #define DHT22_PIN A2
#endif


#ifdef XF1
  char id[] = "XF1";
  #define LOCATION_STRING "12.348,45.64"
  byte num_repeats = '3'; //The number of hops the message will make in the network
  #define BEACON_INTERVAL 300 // Beacon Interval in seconds
  uint8_t rfm_power = 15; // dBmW
  #define SENSITIVE_RX // Enables TESTLNA_SENSITIVE
  #define ENABLE_BATTV_SENSOR // Comment out to disable, you must also disable zombie mode
  #define BATTV_PIN A3 //ADC 0 - Battery Voltage, scaled to 1.1V
  #define BATTV_R1 2180000.0
  #define BATTV_R2 990000.0
  #define BATTV_AREF 3.25 // analog reference value (~1.1V or ~3.3V)
  // Power Saving
  #define ENABLE_ZOMBIE_MODE // Comment this out to disable
  #define ZOMBIE_THRESHOLD 3.65 // Lower Voltage Threshold
  //#define ENABLE_UART_OUTPUT // UART output of packets and rssi, used for gateways
  #define RFM_SSpin 10
  #define ENABLE_BMP085 // BMP085 pressure and temperature sensor
  #define BMP085_PWR 9
  #define ENABLE_BMP085_TEMP // BMP085 temperature
  //#define ENABLE_BMP085_PRESSURE // BMP085 pressure
#endif

#ifdef XF2
  char id[] = "XF2";
  #define LOCATION_STRING "12.348,45.637"
  byte num_repeats = '3'; //The number of hops the message will make in the network
  #define BEACON_INTERVAL 300 // Beacon Interval in seconds
  uint8_t rfm_power = 2; // dBmW
  #define SENSITIVE_RX // Enables TESTLNA_SENSITIVE
  //#define ENABLE_RFM_TEMPERATURE // Comment out to disable
  #define ENABLE_BATTV_SENSOR // Comment out to disable, you must also disable zombie mode
  #define BATTV_PIN A5 //ADC 0 - Battery Voltage, scaled to 1.1V
  #define BATTV_R1 996000.0
  #define BATTV_R2 98300.0
  #define BATTV_INTERNALREF // use internal reference?
  #define BATTV_AREF 1.10 // analog reference value (~1.1V or ~3.3V)
  //#define BATTV_FUDGE 0.2 // Battery Voltage ADC Calibration Factor
  //#define SENSOR_MODE // just a transmitter
  // Power Saving
  #define ENABLE_ZOMBIE_MODE // Comment this out to disable
  // h4x!! act as sensor node
  #define ZOMBIE_THRESHOLD 93.70 // Lower Voltage Threshold
  //#define ADVERTISE_ZOMBIE_MODE // append Z if node is in Zombie mode
  //#define ENABLE_UART_OUTPUT // UART output of packets and rssi, used for gateways
  #define RFM_SSpin 10
  //#define ENABLE_BMP085 // BMP085 pressure and temperature sensor
  //#define BMP085_PWR 9
  //#define ENABLE_BMP085_TEMP // BMP085 temperature
  //#define ENABLE_BMP085_PRESSURE // BMP085 pressure
  #define ENABLE_DS18B20
  #define DS18B20_PIN 8
  #define DS18B20_PWR 9
  uint8_t ds_addr[] = { 0x28, 0xB7, 0x80, 0x8B, 0x02, 0x00, 0x00, 0x55 };
  #define ENABLE_DS18B20_2 // 2nd sensor
  uint8_t ds_addr_2[] = { 0x28, 0xF7, 0x15, 0x05, 0x05, 0x00, 0x00, 0xF3 };
  //#define ENABLE_GPS
#endif


#ifdef XF4
  char id[] = "XF4";
  #define LOCATION_STRING "12.349,45.639"
  byte num_repeats = '3'; //The number of hops the message will make in the network
  #define BEACON_INTERVAL 60 // Beacon Interval in seconds
  uint8_t rfm_power = 2; // dBmW
  #define SENSITIVE_RX // Enables TESTLNA_SENSITIVE
  #define ENABLE_BATTV_SENSOR // Comment out to disable, you must also disable zombie mode
  #define BATTV_PIN A3 //ADC 0 - Battery Voltage, scaled to 1.1V
  #define BATTV_R1 982000.0
  #define BATTV_R2 98500.0
  #define BATTV_INTERNALREF // use internal reference?
  #define BATTV_AREF 1.07 // analog reference value (~1.1V or ~3.3V)
  //#define BATTV_FUDGE 0.2 // Battery Voltage ADC Calibration Factor
  // Power Saving
  #define ENABLE_ZOMBIE_MODE // Comment this out to disable
  #define ZOMBIE_THRESHOLD 3.65 // Lower Voltage Threshold
  //#define ENABLE_UART_OUTPUT // UART output of packets and rssi, used for gateways
  #define RFM_SSpin 10
  //#define ENABLE_BMP085 // BMP085 pressure and temperature sensor
  //#define BMP085_PWR 9
  //#define ENABLE_BMP085_TEMP // BMP085 temperature
  //#define ENABLE_BMP085_PRESSURE // BMP085 pressure
  #define ENABLE_DS18B20
  #define DS18B20_PIN 8
  #define DS18B20_PWR 9
  //#define DS18B20_ADDRESS 28F71505050000F3
  uint8_t ds_addr[] = { 0x28, 0xF7, 0x15, 0x05, 0x05, 0x00, 0x00, 0xF3 };
  //#define ENABLE_GPS
#endif



#ifdef YYYY
  char id[] = "YYYY";
  #define LOCATION_STRING "50.4535,-1.35435"
  byte num_repeats = '3'; //The number of hops the message will make in the network
  #define BEACON_INTERVAL 300 // Beacon Interval in seconds
  uint8_t rfm_power = 20; // dBmW
  #define SENSITIVE_RX // Enables TESTLNA_SENSITIVE
  #define ENABLE_BATTV_SENSOR // Comment out to disable, you must also disable zombie mode
  #define BATTV_PIN 0 //ADC 0 - Battery Voltage, scaled to 1.1V
  #define BATTV_FUDGE 0.935 // Battery Voltage ADC Calibration Factor
  // Power Saving
  #define ENABLE_ZOMBIE_MODE // Comment this out to disable
  #define ZOMBIE_THRESHOLD 3.65 // Lower Voltage Threshold
  #define ENABLE_UART_OUTPUT // UART output of packets and rssi, used for gateways
#endif

#ifdef XXXX
  char id[] = "XXXX";
  #define LOCATION_STRING "50.2475,-1.36545"
  byte num_repeats = '3'; //The number of hops the message will make in the network
  #define BEACON_INTERVAL 300 // Beacon Interval in seconds
  uint8_t rfm_power = 20; // dBmW
  #define SENSITIVE_RX // Enables TESTLNA_SENSITIVE
  #define ENABLE_BATTV_SENSOR // Comment out to disable, you must also disable zombie mode
  #define BATTV_PIN 0 //ADC 0 - Battery Voltage, scaled to 1.1V
  #define BATTV_FUDGE 0.943 // Battery Voltage ADC Calibration Factor
  // Power Saving
  #define ENABLE_ZOMBIE_MODE // Comment this out to disable
  #define ZOMBIE_THRESHOLD 3.65 // Lower Voltage Threshold
#endif

//************* Other config ****************/
// RFM Temperature Sensor - Not very accurate and sometimes glitchy
#define RFM_TEMP_FUDGE 0 // Initial RFM Calibration
#define RX_TEMP_FUDGE 5.0 // Temperature offset when in RX due to self-heating

