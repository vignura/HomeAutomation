// headers
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <utility.h>
#include <serial_com.h>
#include <CmdProcess.h>

/* Master pins */
#define MTX_PIN         2
#define MRX_PIN         3

/***********************************************************************************************/
/* comment the below macro to disable debug prints */
#define PRINT_DEBUG

#define MAX_DEBUG_MSG_SIZE                  128
#define MAX_CMD_STRING_SIZE                 32

#define MAX_RELAY_COUNT                     5
#define SELF_TEST_COUNT                     0x00
#define SS_BUAD_RATE                        9600
#define DEBUG_BUAD_RATE                     9600

#define ADDRESS_STRING_SIZE                8 // eg 001.001
/* bluetooth command Strings*/
#define CMD_STR_RELAY_ON                  "ON"
#define CMD_STR_RELAY_ON_TIMER            "TON " /* TON hh:mm:ss */
#define CMD_STR_RELAY_OFF                 "OFF"
#define CMD_STR_START_TEST                "TEST"
#define CMD_STR_RELAY_OFF_ALL             "AOFF"

/* bluetooth command IDs */
#define CMD_INVALID_CMD_ID                      -1

#define SERIAL_CMD_TIMEOUT_MS                   1000
/****************************************** globals ********************************************/
/* SoftwareSerial (RX, TX) */
SoftwareSerial SSerial(MRX_PIN, MTX_PIN);

#ifdef PRINT_DEBUG
  char g_arrcMsg[MAX_DEBUG_MSG_SIZE] = {0};
  char g_arrcBTMsg[MAX_CMD_STRING_SIZE] = {0};
#endif

uint8_t g_Slave_add[2];
unsigned long g_ulOnTimeSec = 0;
unsigned char g_ucRlySel = 0;
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
  
  // assign slave temporary address
  g_Slave_add[0] = 1;
  g_Slave_add[1] = 1;

  // Serial port initialization
  #ifdef PRINT_DEBUG
    Serial.begin(DEBUG_BUAD_RATE);
  #endif
  
  SSerial.begin(SS_BUAD_RATE);
}

/***********************************************************************************************/
/*! 
* \fn         :: loop()
* \author     :: Vignesh S
* \date       :: 02-DEC-2018
* \brief      :: This functions is called repeatedly from a infinite for loop in main().
*                It does the following functions
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void loop() {

  char arrcCmd[MAX_CMD_STRING_SIZE] = {0};
  int iReadBytes = 0;
  int iCmdID = 0;

  // read data from HC-05 if available
  iReadBytes = RecvCmd(Serial, arrcCmd, MAX_CMD_STRING_SIZE); 
  if(iReadBytes > 0)
  {
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg, "Received: [%d] %s", iReadBytes, arrcCmd);
      Serial.println(g_arrcMsg);
    #endif

    // validate the command
    if(isValidCmd(arrcCmd, iReadBytes, &iCmdID) == true)
    {
      // if valid command is received, process it
      CmdProcess(iCmdID);
    }
    else
    {
      // do nothing
    }
  }
}

/***********************************************************************************************/
/*! 
* \fn         :: isValidCmd()
* \author     :: Vignesh S
* \date       :: 05-DEC-2018
* \brief      :: This function validates the received command and return true if it a valid 
*                command else returns false, if a valid command is received, then the 
*                corresponding command ID is returned through out_iCmdID parameter, similarly
*                Invalid command ID error code is returned.
* \param[in]  :: parrcCmd, iCmdLen 
* \param[out] :: out_iCmdID
* \return     :: true or false
*/
/***********************************************************************************************/
bool isValidCmd(char *parrcCmd, int iCmdLen, int *out_iCmdID)
{
  unsigned long ulHour = 0;
  unsigned long ulMin = 0;
  unsigned long ulSec = 0;
  unsigned char ucRlySel = 0;
  uint8_t slave_address[2] = {0};
  int iRetVal = 0;
  int iPwd = 0;

  if((parrcCmd == NULL) || (out_iCmdID == NULL) || (iCmdLen <= 0))
  {
    return false;
  }

  // extract slave address
  iRetVal = sscanf(parrcCmd, "%d.%d", &slave_address[0], &slave_address[1]);
  if(iRetVal != 0x02)
  {
    // invalid slave adderess
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg,"Invalid Pramater: %s", parrcCmd);
      Serial.println(g_arrcMsg);
    #endif
    return false;
  }
  else
  {
    // copy to global
    parrcCmd += ADDRESS_STRING_SIZE; 
    g_Slave_add[0] = slave_address[0];
    g_Slave_add[1] = slave_address[1];
  }
  
  if (StrnCmp(parrcCmd, CMD_STR_RELAY_ON, strlen(CMD_STR_RELAY_ON)) == true)
  {
    iRetVal = sscanf(parrcCmd, CMD_STR_RELAY_ON "%d", &ucRlySel);
    if(iRetVal != 0x01)
    {
      // invalid command
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg,"Invalid Pramater: %s", parrcCmd);
        Serial.println(g_arrcMsg);
      #endif
      *out_iCmdID = CMD_INVALID_CMD_ID;
      return false;
    }
    else
    {
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg,"Cmd: %s\r\nRly Sel: %d\r\n", parrcCmd, ucRlySel);
        Serial.println(g_arrcMsg);
      #endif

      if((ucRlySel < 1) || (ucRlySel > MAX_RELAY_COUNT))
      {
        *out_iCmdID = CMD_INVALID_CMD_ID;
        return false;
      }

      g_ucRlySel = ucRlySel;
    }
    
    *out_iCmdID = CMD_RLY_ON_ID;
    return true;
  }
  else if (StrnCmp(parrcCmd, CMD_STR_RELAY_ON_TIMER, strlen(CMD_STR_RELAY_ON_TIMER)) == true)
  {
    iRetVal = sscanf(parrcCmd, CMD_STR_RELAY_ON_TIMER "%d %d:%d:%d", &ucRlySel, &ulHour, &ulMin, &ulSec);
    if(iRetVal != 0x04)
    {
        // invalid command
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg,"Invalid Pramater: %s", parrcCmd);
        Serial.println(g_arrcMsg);
      #endif
      *out_iCmdID = CMD_INVALID_CMD_ID;
      
      return false;
    }
    else
    {
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg,"Cmd: %s\r\nRly Sel: %d\r\nHour: %ld\r\nMin: %ld\r\nSec: %ld", 
                ucRlySel, parrcCmd, ulHour, ulMin, ulSec);
        Serial.println(g_arrcMsg);
      #endif

      if((ucRlySel < 1) || (ucRlySel > MAX_RELAY_COUNT))
      {
        *out_iCmdID = CMD_INVALID_CMD_ID;
        return false;
      }

      g_ucRlySel = ucRlySel;
      // set the global timer variable
      g_ulOnTimeSec = (ulHour * 3600UL) + (ulMin * 60UL) + ulSec;
    }
   
   *out_iCmdID = CMD_RLY_ON_TIMER_ID; 
    return true;
  }
  else if (StrnCmp(parrcCmd, CMD_STR_RELAY_OFF, strlen(CMD_STR_RELAY_OFF)) == true)
  {
    iRetVal = sscanf(parrcCmd, CMD_STR_RELAY_OFF "%d", &ucRlySel);
    if(iRetVal != 0x01)
    {
      // invalid command
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg,"Invalid Pramater: %s", parrcCmd);
        Serial.println(g_arrcMsg);
      #endif
      *out_iCmdID = CMD_INVALID_CMD_ID;
      return false;
    }
    else
    {
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg,"Cmd: %s\r\nRly Sel: %d", parrcCmd, ucRlySel);
        Serial.println(g_arrcMsg);
      #endif

      if((ucRlySel < 1) || (ucRlySel > MAX_RELAY_COUNT))
      {
        *out_iCmdID = CMD_INVALID_CMD_ID;
        return false;
      }

      g_ucRlySel = ucRlySel;
    }
    
    *out_iCmdID = CMD_RLY_OFF_ID;
    return true;
  }
  else if(StrnCmp(parrcCmd, CMD_STR_START_TEST, strlen(CMD_STR_START_TEST)) == true)
  {
    *out_iCmdID = CMD_TEST_ID;
    return true;
  }
  else if (StrnCmp(parrcCmd, CMD_STR_RELAY_OFF_ALL, strlen(CMD_STR_RELAY_OFF_ALL)) == true)
  {
    *out_iCmdID = CMD_RLY_OFF_ALL_ID;
    return true;
  }
  else
  {
    // invalid command
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg,"Invalid Cmd: %s", parrcCmd);
      Serial.println(g_arrcMsg);
    #endif
    *out_iCmdID = CMD_INVALID_CMD_ID;
  }

  return false;
}

/***********************************************************************************************/
/*! 
* \fn         :: CmdProcess()
* \author     :: Vignesh S
* \date       :: 05-DEC-2018
* \brief      :: This function processes the recceived command and preforms corresponding task
* \param[in]  :: iCmdID
* \param[out] :: ipResponse
* \return     :: None
*/
/***********************************************************************************************/
void CmdProcess(int iCmdID)
{ 
  int i = 0;
  int iRetVal = 0;
  int data_size = 0;

  serial_packet cmd_packet = {0};
  serial_packet res_packet = {0};
  data_packets Udata_packets = {0};

  Udata_packets.ptr = (uint16_t*)cmd_packet.data;

  // fill the command packet
  cmd_packet.header = SERIAL_CMD_HEADER;
  cmd_packet.address[0] = g_Slave_add[0];
  cmd_packet.address[1] = g_Slave_add[1];
  cmd_packet.cmdID = iCmdID;

  switch(iCmdID)
  {
    case CMD_TEST_ID:
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Running self test", g_ucRlySel);
        Serial.println(g_arrcMsg);
      #endif
      data_size = 0;
    break;

    case CMD_RLY_ON_ID:
      
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay %d ON", g_ucRlySel);
        Serial.println(g_arrcMsg);
      #endif
      Udata_packets.Prelay_on->relay_num = g_ucRlySel;
      data_size = sizeof(relay_on);
    break;

    case CMD_RLY_ON_TIMER_ID:
      
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay %d ON for %ld seconds", g_ucRlySel, g_ulOnTimeSec);
        Serial.println(g_arrcMsg);
      #endif
      Udata_packets.Prelay_on_timer->relay_num = g_ucRlySel;
      Udata_packets.Prelay_on_timer->on_time_sec = g_ulOnTimeSec;
      data_size = sizeof(relay_on_timer);
    break;

    case CMD_RLY_OFF_ID:

      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay %d OFF", g_ucRlySel);
        Serial.println(g_arrcMsg);
      #endif
      Udata_packets.Prelay_off->relay_num = g_ucRlySel;
      data_size = sizeof(relay_off);
    break;

    case CMD_RLY_OFF_ALL_ID:
      
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning all Relay OFF");
        Serial.println(g_arrcMsg);
      #endif
      data_size = 0;
    break;

    default:
      ;// do nothing 
  }

  cmd_packet.data_size = data_size;
  cmd_packet.checksum = compute_checksum(&cmd_packet);

  if(transmit_command(SSerial, &cmd_packet, &res_packet, SERIAL_CMD_TIMEOUT_MS) < SERIAL_HEADER_SIZE)
  {
    #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "transmit_command failed..!");
        Serial.println(g_arrcMsg);
    #endif    
  }
  else
  {
    #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "transmit_command success");
        Serial.println(g_arrcMsg);
    #endif     
  }
}