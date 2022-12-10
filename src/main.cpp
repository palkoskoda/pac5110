/*
 *  Simple sketch that shows the basic usage of the ABBAurora class.
 *  Example is from an ESP32 with a MAX485 as RS485 interface.
 */
#include <Arduino.h>
#include <pac5110.h>
#include <Wire.h>
#include <WiFi.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define RX2 16
#define TX2 17
#define INVERTER_ADDRESS 1
#define TX_CONTROL_GPIO 4

#define WIFI_SSID "wifidoma2"
#define WIFI_PASSWORD "asdfghjkl"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 1, 2)
// For a cloud MQTT broker, type the domain name
// #define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// Temperature MQTT Topics
#define MQTT_PREFIX "esp32/merak/"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

ABBAurora *inverter;

unsigned long previousMillis = 0; // Stores last time temperature was published
const long interval = 60000;      // Interval at which to publish 0 readings

void connectToWifi()
{
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent)
{
    Serial.println("Connected to MQTT.");
    Serial.print("Session present: ");
    Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    Serial.println("Disconnected from MQTT.");
    if (WiFi.isConnected())
    {
        xTimerStart(mqttReconnectTimer, 0);
    }
}

void onMqttPublish(uint16_t packetId)
{
    Serial.print("Publish acknowledged.");
    Serial.print("  packetId: ");
    Serial.println(packetId);
}

void setup()
{
    Serial.begin(115200);
    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

    WiFi.onEvent(WiFiEvent);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    // mqttClient.onSubscribe(onMqttSubscribe);
    // mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onPublish(onMqttPublish);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    // If your broker requires authentication (username and password), set them below
    // mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
    connectToWifi();
    ABBAurora::setup(Serial2, RX2, TX2, TX_CONTROL_GPIO);
    inverter = new ABBAurora(INVERTER_ADDRESS);
    Serial.println("Setup done");
}

void loop()
{
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
        // ak dlhsie neaktualizovalo vykon, pošle 0
        // uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_POWER_IN_1, 2, true, String(0).c_str());
        // uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_POWER_IN_2, 2, true, String(0).c_str());
        // uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_GRID_POWER, 2, true, String(0).c_str());
    }

    if (inverter->ReadValue(Phase_1_line_to_neutral_volts))
    {
        float tmp = inverter->Merak.Value;
        uint16_t packetIdPub1 = mqttClient.publish("esp32/merak/Phase_1_line_to_neutral_volts", 2, true, String(tmp).c_str());
        Serial.printf("Publishing on topic %s at QoS 2, packetId: %i", "esp32/merak/Phase_1_line_to_neutral_volts", packetIdPub1);
        Serial.printf("Message: %.2f \n", tmp);
    }

    if (inverter->ReadValue(Phase_2_line_to_neutral_volts))
    {
        float tmp = inverter->Merak.Value;
        uint16_t packetIdPub2 = mqttClient.publish("esp32/merak/Phase_2_line_to_neutral_volts", 2, true, String(tmp).c_str());
        Serial.printf("Publishing on topic %s at QoS 2, packetId: %i", "esp32/merak/Phase_2_line_to_neutral_volts", packetIdPub2);
        Serial.printf("Message: %.2f \n", tmp);
    }

    if (inverter->ReadValue(Phase_3_line_to_neutral_volts))
    {
        float tmp = inverter->Merak.Value;
        uint16_t packetIdPub3 = mqttClient.publish("esp32/merak/Phase_3_line_to_neutral_volts", 2, true, String(tmp).c_str());
        Serial.printf("Publishing on topic %s at QoS 2, packetId: %i", "esp32/merak/Phase_3_line_to_neutral_volts", packetIdPub3);
        Serial.printf("Message: %.2f \n", tmp);
    }
    
    if (inverter->ReadValue(Phase_1_active_power))
    {
        float tmp = inverter->Merak.Value;
        uint16_t packetIdPub4 = mqttClient.publish("esp32/merak/Phase_1_active_power", 2, true, String(tmp).c_str());
        Serial.printf("Publishing on topic %s at QoS 2, packetId: %i", "esp32/merak/Phase_1_active_power", packetIdPub4);
        Serial.printf("Message: %.2f \n", tmp);
    }
    
    if (inverter->ReadValue(Phase_2_active_power))
    {
        float tmp = inverter->Merak.Value;
        uint16_t packetIdPub5 = mqttClient.publish("esp32/merak/Phase_2_active_power", 2, true, String(tmp).c_str());
        Serial.printf("Publishing on topic %s at QoS 2, packetId: %i", "esp32/merak/Phase_2_active_power", packetIdPub5);
        Serial.printf("Message: %.2f \n", tmp);
    }
    
    if (inverter->ReadValue(Phase_3_active_power))
    {
        float tmp = inverter->Merak.Value;
        uint16_t packetIdPub6 = mqttClient.publish("esp32/merak/Phase_3_active_power", 2, true, String(tmp).c_str());
        Serial.printf("Publishing on topic %s at QoS 2, packetId: %i", "esp32/merak/Phase_3_active_power", packetIdPub6);
        Serial.printf("Message: %.2f \n", tmp);
    }
     
    if (inverter->ReadValue(Total_system_active_power))
    {
        float tmp = inverter->Merak.Value;
        uint16_t packetIdPub7 = mqttClient.publish("esp32/merak/Total_system_active_power", 2, true, String(tmp).c_str());
        Serial.printf("Publishing on topic %s at QoS 2, packetId: %i", "esp32/merak/Total_system_active_power", packetIdPub7);
        Serial.printf("Message: %.2f \n", tmp);
    }
         
    /*if (inverter->ReadValue(Current_2st_63st_Harmonic_L1))
    {

        uint16_t packetIdPub8 = mqttClient.publish("esp32/merak/Current_2st_63st_Harmonic_L1", 2, true, String(tmp).c_str());
        Serial.printf("Publishing on topic %s at QoS 2, packetId: %i", "esp32/merak/Current_2st_63st_Harmonic_L1", packetIdPub8);
        Serial.printf("Message: %.2f \n", packetIdPub8);
    }
         
    if (inverter->ReadValue(Current_2st_63st_Harmonic_L2))
    {

        uint16_t packetIdPub9 = mqttClient.publish("esp32/merak/Current_2st_63st_Harmonic_L2", 2, true, String(tmp).c_str());
        Serial.printf("Publishing on topic %s at QoS 2, packetId: %i", "esp32/merak/Current_2st_63st_Harmonic_L2", packetIdPub9);
        Serial.printf("Message: %.2f \n", packetIdPub9);
    }
    
    if (inverter->ReadValue(Current_2st_63st_Harmonic_L3))
    {

        uint16_t packetIdPub10 = mqttClient.publish("esp32/merak/Current_2st_63st_Harmonic_L3", 2, true, String(tmp).c_str());
        Serial.printf("Publishing on topic %s at QoS 2, packetId: %i", "esp32/merak/Current_2st_63st_Harmonic_L3", packetIdPub10);
        Serial.printf("Message: %.2f \n", packetIdPub10);
    }*/
}