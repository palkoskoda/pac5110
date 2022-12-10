#ifndef pac5110_h
#define pac5110_h
#include <Arduino.h>
#include <pac5110enums.h>
#include <Wire.h>

//RS485 control
#define RS485Transmit HIGH
#define RS485Receive LOW

class ABBAurora
{
private:
    int MaxAttempt = 1;
    static byte TXPinControl;
    static HardwareSerial *serial;

    void clearData(byte *data, byte len);

    int MODBUS_CRC16_v1( const unsigned char *buf, unsigned int len );

    bool Send(byte address, byte param0, byte param1, byte param2, byte param3, byte param4, int ReceiveSize = 9);

    union {
        byte asBytes[4];
        float asFloat;
    } foo;

    union {
        byte asBytes[4];
        unsigned long asUlong;
    } ulo;

public:
    bool SendStatus = false;
    bool ReceiveStatus = false;
    int ByteCount = 0; 
    byte Address = 0;
    byte ReceiveData[9];

    static void setup(HardwareSerial &serial, byte RXGpioPin, byte TXGpioPin, byte TXControllPin);

    ABBAurora(byte address);

    void clearReceiveData();


    typedef struct
    {
        float Value;
        bool ReadState;
    } DataMerak;

    DataMerak Merak;

    bool ReadValue(merak_VALUE_TYPE type);    
    bool ReadInput(int type);
    bool ReadInput();

};

#endif
