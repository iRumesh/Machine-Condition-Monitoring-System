/*----------WiFi Credentials------------*/
// #define WIFI_SSID "DSI_IOT"
// #define WIFI_PASSWORD "dsi@#123"

#define WIFI_SSID "DIGITAL LAB"
#define WIFI_PASSWORD "#123@450#"

IPAddress local_IP(192, 168, 1, 102);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// MQTT Config
#define MQTT_SERVER "thingsboard.cloud"
#define MQTT_PORT 1883
#define MQTT_CLIENTID "DW1"
#define MQTT_USERNAME "D01"
#define MQTT_TELE_TOPIC "v1/devices/me/telemetry"

const unsigned long sendInterval = 5000;
