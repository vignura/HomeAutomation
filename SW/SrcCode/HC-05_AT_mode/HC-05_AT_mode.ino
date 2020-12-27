#include <SoftwareSerial.h>

#define HC05_EN         PIN_A5
SoftwareSerial BTSerial(3, 4); // RX | TX


void setup()
{
//pinMode(HC05_EN, OUTPUT);
//digitalWrite(HC05_EN, LOW);

Serial.begin(9600);
Serial.println("Enter AT commands:");
BTSerial.begin(38400); // HC-05 default speed in AT command more
}

void loop()
{
if (BTSerial.available()) // read from HC-05 and send to Arduino Serial Monitor
Serial.write(BTSerial.read());

if (Serial.available()) // Keep reading from Arduino Serial Monitor and send to HC-05
BTSerial.write(Serial.read());
}
