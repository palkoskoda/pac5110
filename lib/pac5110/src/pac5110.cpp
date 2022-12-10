#include "pac5110.h"

byte ABBAurora::TXPinControl;
HardwareSerial *ABBAurora::serial;

ABBAurora::ABBAurora(byte address)
{
    Address = address;
    SendStatus = false;
    ReceiveStatus = false;
    clearReceiveData();
}
void ABBAurora::setup(HardwareSerial &hardwareSerial, byte RXGpioPin, byte TXGpioPin, byte TXControllPin)
{
    TXPinControl = TXControllPin;

    pinMode(TXPinControl, OUTPUT);
    digitalWrite(TXPinControl, LOW);

    serial = &hardwareSerial;
    serial->begin(19200, SERIAL_8N1, RXGpioPin, TXGpioPin, false, 500);
}

void ABBAurora::clearData(byte *data, byte len)
{
    for (int i = 0; i < len; i++)
    {
        data[i] = 0;
    }
}


int ABBAurora::MODBUS_CRC16_v1( const unsigned char *buf, unsigned int len )
{
	uint16_t crc = 0xFFFF;
	unsigned int i = 0;
	char bit = 0;

	for( i = 0; i < len; i++ )
	{
		crc ^= buf[i];

		for( bit = 0; bit < 8; bit++ )
		{
			if( crc & 0x0001 )
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
			{
				crc >>= 1;
			}
		}
	}

	return crc;
}

bool ABBAurora::Send(byte address, byte param0, byte param1, byte param2, byte param3, byte param4, int ReceiveSize)
{

    SendStatus = false;
    ReceiveStatus = false;

    byte SendData[8];
    SendData[0] = address;
    SendData[1] = param0;
    SendData[2] = param1;
    SendData[3] = param2;
    SendData[4] = param3;
    SendData[5] = param4;

    int crc = MODBUS_CRC16_v1(SendData, 6);
    SendData[6] = lowByte(crc);
    SendData[7] = highByte(crc);

    Serial.print(SendData[0], HEX); Serial.print(" ");
    Serial.print(SendData[1], HEX); Serial.print(" ");
    Serial.print(SendData[2], HEX); Serial.print(" ");
    Serial.print(SendData[3], HEX); Serial.print(" ");
    Serial.print(SendData[4], HEX); Serial.print(" ");
    Serial.print(SendData[5], HEX); Serial.print(" ");
    Serial.print(SendData[6], HEX); Serial.print(" ");
    Serial.println(SendData[7], HEX);
    clearReceiveData();

    for (int i = 0; i < this->MaxAttempt; i++)
    {
        //digitalWrite(TXPinControl, RS485Transmit);
        delay(50);

        if (serial->write(SendData, sizeof(SendData)) != 0)
        {
            serial->flush();
            SendStatus = true;

            //digitalWrite(TXPinControl, RS485Receive);

            if (serial->readBytes(ReceiveData, ReceiveSize) != 0)
            {
                Serial.print(ReceiveData[0], HEX); Serial.print(" ");
                Serial.print(ReceiveData[1], HEX); Serial.print(" ");
                Serial.print(ReceiveData[2], HEX); Serial.print(" ");
                Serial.print(ReceiveData[3], HEX); Serial.print(" ");
                Serial.print(ReceiveData[4], HEX); Serial.print(" ");
                Serial.print(ReceiveData[5], HEX); Serial.print(" ");
                Serial.print(ReceiveData[6], HEX); Serial.print(" ");
                Serial.print(ReceiveData[7], HEX); Serial.print(" ");
                Serial.println(ReceiveData[8], HEX);
                /*Serial.print("ReceiveSize: ");
                Serial.print(ReceiveSize);
                Serial.print("sizeof rd: ");
                Serial.print(sizeof(ReceiveData));
                Serial.print("   crc priate: ");
                Serial.print(ReceiveData[ReceiveSize-1], HEX); Serial.print(ReceiveData[ReceiveSize-2],HEX);
                Serial.print("  crc vypocitane: ");
                Serial.println(MODBUS_CRC16_v1(ReceiveData, ReceiveSize-2), HEX);*/
                if ((int)word(ReceiveData[ReceiveSize-1], ReceiveData[ReceiveSize-2]) == MODBUS_CRC16_v1(ReceiveData, ReceiveSize-2))
                {
                    //Serial.println("vyslo to");
                    ReceiveStatus = true;
                    break;
                }
            }
        }
    }
    return ReceiveStatus;
}

void ABBAurora::clearReceiveData()
{
    clearData(ReceiveData, 9);
}

/**
 * Reads a single value of the digital signal procesor.
 * Not all values are supported by all models. 
 * Read values are in following Units:
 * Voltage V
 * Current A 
 * Power W 
 * Temperature °C 
 * 
 **/
/*bool ABBAurora::ReadValue(merak_VALUE_TYPE type)
{
    Merak.ReadState = Send(this->Address, (byte)04, (byte)0, (byte)0, (byte)(highByte(type)), (byte)(lowByte(type)));

    foo.asBytes[0] = ReceiveData[6];
    foo.asBytes[1] = ReceiveData[5];
    foo.asBytes[2] = ReceiveData[4];
    foo.asBytes[3] = ReceiveData[3];

    Merak.Value = foo.asFloat;
    return Merak.ReadState;
}*/

bool ABBAurora::ReadValue(merak_VALUE_TYPE type)
{
    Merak.ReadState = Send(this->Address, (byte)04, (byte)(highByte(type)), (byte)(lowByte(type)), (byte)0, (byte)2);

    foo.asBytes[0] = ReceiveData[6];
    foo.asBytes[1] = ReceiveData[5];
    foo.asBytes[2] = ReceiveData[4];
    foo.asBytes[3] = ReceiveData[3];

    Merak.Value = foo.asFloat;
    return Merak.ReadState;
}

bool ABBAurora::ReadInput()
{
    Merak.ReadState = Send(this->Address, (byte)02, (byte)0, (byte)0, (byte)0, (byte)3);

    Merak.Value = ReceiveData[3];

    return Merak.ReadState;
}

bool ABBAurora::ReadInput(int type)
{
    Merak.ReadState = Send(this->Address, (byte)02, (byte)0, (byte)0, (byte)0, (byte)3, 6);

    Merak.Value = bitRead(ReceiveData[3], type);

    return Merak.ReadState;
}