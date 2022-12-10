/*
 *  Simple sketch that shows the basic usage of the ABBAurora class.
 *  Example is from an ESP32 with a MAX485 as RS485 interface.
 */
#include <Arduino.h>
#include <pac5110.h>

#define RX2 16
#define TX2 17
#define INVERTER_ADDRESS 1
#define TX_CONTROL_GPIO 4

ABBAurora *inverter;
void setup()
{
    Serial.begin(115200);
    ABBAurora::setup(Serial2, RX2, TX2, TX_CONTROL_GPIO);
    inverter = new ABBAurora(INVERTER_ADDRESS);
    Serial.println("Setup done");
}

void loop()
{
        if (inverter->ReadInput(1))
    {
        Serial.print("DI2 : ");
        Serial.print(inverter->Merak.Value);
    }

    if (inverter->ReadValue(Phase_2_line_to_neutral_volts))
    {
        Serial.print("   Pin1 : ");
        Serial.print(inverter->Merak.Value);
        Serial.println(" V");
    }


}