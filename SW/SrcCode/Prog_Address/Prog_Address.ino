#include <EEPROM.h>
#include <utility.h>
#include <serial_com.h>

#define PRINT_DEBUG
#define DEBUG_BUAD_RATE             9600
#define MAX_CMD_STRING_SIZE         128
#define EEPROM_ADDRESS_LOCATION     512
/***********************************************************************************************/
/*! 
* \fn         :: setup()
* \author     :: Vignesh S
* \date       :: 27-DEC-2020
* \brief      :: This function performs the following initializations
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void setup() {
  
  char arrcCmd[32] = {0};
  int iReadBytes = 0;

  Serial.begin(DEBUG_BUAD_RATE);

  Serial.println("Program Address");
  Serial.println("Enter Address [eg 254.254]: ");
}

void loop() {
  
  int ret = 0;
  int iReadBytes = 0;
  uint8_t write_addr[2] = {0};
  uint8_t read_addr[2] = {0};
  char arrcCmd[MAX_CMD_STRING_SIZE] = {0};
  char arrcMsg[MAX_CMD_STRING_SIZE] = {0};
  
  iReadBytes = RecvCmd<HardwareSerial>(Serial, arrcCmd, MAX_CMD_STRING_SIZE); 
  if(iReadBytes > 0)
  {
    #ifdef PRINT_DEBUG
      sprintf(arrcMsg, "Received: [%d] %s", iReadBytes, arrcCmd);
      Serial.println(arrcMsg);
    #endif

    ret = sscanf(arrcCmd, "%d.%d", &write_addr[0], &write_addr[1]);
    if(ret != 2)
    {
      sprintf(arrcMsg, "[ret: %d]: enter a valid address", ret);
      Serial.println(arrcMsg);
    }
    else
    {
      sprintf(arrcMsg, "storing address %03d.%03d at EEPROM location %d", write_addr[0], write_addr[1], EEPROM_ADDRESS_LOCATION);
      Serial.println(arrcMsg);
      EEPROM.put(EEPROM_ADDRESS_LOCATION, write_addr);

      sprintf(arrcCmd, "reading back from EEPROM location %d", EEPROM_ADDRESS_LOCATION);
      Serial.println(arrcCmd);
      EEPROM.get(EEPROM_ADDRESS_LOCATION, read_addr);
      if((read_addr[0] == write_addr[0]) && (read_addr[1] == write_addr[1]))
      {
        sprintf(arrcMsg, "read back success", EEPROM_ADDRESS_LOCATION);
        Serial.println(arrcMsg);
      }
      else
      {
        sprintf(arrcMsg, "read back failed", EEPROM_ADDRESS_LOCATION);
        Serial.println(arrcMsg);  
      }
    }

    Serial.println("\nProgram Address");
    Serial.println("Enter Address [eg 254.254]: ");
  }
}