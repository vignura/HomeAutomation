/***********************************************************************************************/
/*
 *  \file       : CZ-02-HomeAutomation.ino
 *  \date       : 02-DEC-2020 
 *  \author     : Vignesh S 
 *  \email      : vignura@gmail.com
 *  \copyright  : All Rights are Reserved 
/*
/***********************************************************************************************/
// headers
#include <SoftwareSerial.h>
#include <Relay.h>
#include <EEPROM.h>
/*************************************** Pin Mappings ******************************************/
// HC-05 
#define HC05_ST         PIN_A1    /* status pin */
#define HC05_EN         PIN_A5    /* enable pin */

// Relay 
#define RELAY_01        7       /*DRV 16 - PD7 */
#define RELAY_02        8       /*DRV 17 - PB0 */
#define RELAY_03        9       /*DRV 18 - PB1 */
#define RELAY_04        10      /*DRV 19 - PB2 */
#define RELAY_05        PIN_A2  /*DRV 20 - PC2 / ADC2 */
#define MAX_RELAY_COUNT 5

/* bluetooth pins */
#define BTX_PIN         2
#define BRX_PIN         3

/***********************************************************************************************/
/* comment the below macro to disable debug prints */
#define PRINT_DEBUG

/* comment the below macro to disable Bluetooth Response messages */
#define SEND_BTRES

#define MAX_DEBUG_MSG_SIZE                  128
#define MAX_CMD_STRING_SIZE                 32

#define SELF_TEST_COUNT                     0x00
#define HC05_BUAD_RATE                      9600
#define DEBUG_BUAD_RATE                     9600
#define HC05_SERIAL_READ_DELAY_MS           0x02

/* bluetooth command Strings*/
#define CMD_STR_RELAY_ON                  "ON"
#define CMD_STR_RELAY_ON_TIMER            "TON " /* TON hh:mm:ss */
#define CMD_STR_RELAY_OFF                 "OFF"
#define CMD_STR_START_TEST                "TEST"
#define CMD_STR_CHANGE_PWD                "ChPwd" /*ChPwd 0000 */
#define CMD_STR_RELAY_OFF_ALL             "AOFF"

/* bluetooth response Strings*/
#define RES_RELAY_ON                      "ON"
#define RES_RELAY_OFF                     "OFF"
#define RES_START_TEST                    "TEST"

/* bluetooth command IDs */
#define CMD_INVALID_CMD_ID                      -1
#define CMD_STR_RELAY_ON_ID                     0x01
#define CMD_STR_RELAY_ON_TIMER_ID               0x02
#define CMD_STR_RELAY_OFF_ID                    0x03
#define CMD_STR_START_TEST_ID                   0x04
#define CMD_STR_CHANGE_PWD_ID                   0x05
#define CMD_STR_RELAY_OFF_ALL_ID                0x06


/****************************************** globals ********************************************/
/* SoftwareSerial (RX, TX) */
SoftwareSerial SS_Bluetooth(BRX_PIN, BTX_PIN);

/* Relay objects pointers */
Relay *Rly[MAX_RELAY_COUNT] = {0};
unsigned char g_ucRlyPin[MAX_RELAY_COUNT] = {RELAY_01, RELAY_02, RELAY_03, RELAY_04, RELAY_05};

#ifdef PRINT_DEBUG
  char g_arrcMsg[MAX_DEBUG_MSG_SIZE] = {0};
#endif

#ifdef SEND_BTRES
  char g_arrcBTMsg[MAX_CMD_STRING_SIZE] = {0};
#endif

unsigned long g_ulOnTimeSec = 0;
unsigned char g_ucRlySel = 0;
int g_iPwd = 0;

/***********************************************************************************************/
/*! 
* \fn         :: setup()
* \author     :: Vignesh S
* \date       :: 02-DEC-2018
* \brief      :: This function performs the following initializations
*                1) Sets the Relay pins as output and sets its state to low
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void setup() {
  
  // set the HC-05 Enable as output and set the state to LOW
  //pinMode(HC05_EN, OUTPUT);
  //digitalWrite(HC05_EN, LOW);

  Relay_init();

  // Serial port initialization
  #ifdef PRINT_DEBUG
    Serial.begin(DEBUG_BUAD_RATE);
  #endif
  
  SS_Bluetooth.begin(HC05_BUAD_RATE);

  // perform self test
  SelfTest(SELF_TEST_COUNT);

}

/***********************************************************************************************/
/*! 
* \fn         :: Relay_init()
* \author     :: Vignesh S
* \date       :: 21-DEC-2020
* \brief      :: This function initializes MAX_RELAY_COUNT relay instances with states stored
*                EEPROM if the EEPROM Checksum is valid, else initializes the relay instances
*                to default state (OFF)
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void Relay_init()
{
  uint8_t arrState[MAX_RELAY_COUNT] = {0};
  uint8_t ReadChecksum = 0;
  uint8_t checksum = 0;

  // read previously stored states from EEPROM
  for(int i = 0; i < MAX_RELAY_COUNT; i++)
  {
    EEPROM.get((i * sizeof(uint8_t)), arrState[i]);
    checksum ^= arrState[i];
  }

  // read checksum
  EEPROM.get(((MAX_RELAY_COUNT +1) * sizeof(uint8_t)), ReadChecksum);

  // verify checksum
  if(ReadChecksum == checksum)
  {
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg, "Restoring relay states form EEPROM");
      Serial.println(g_arrcMsg);
    #endif

    // Relay initialization
    for(int i = 0; i < MAX_RELAY_COUNT; i++)
    {
      Rly[i] = new Relay(g_ucRlyPin[i], arrState[i], false);
    }
  } 
  else
  {
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg, "EEPROM Checksum mismatch\n" "Setting all relay states to OFF");
      Serial.println(g_arrcMsg);
    #endif

    // Relay initialization
    for(int i = 0; i < MAX_RELAY_COUNT; i++)
    {
      Rly[i] = new Relay(g_ucRlyPin[i], false);
      Rly[i]->setState(RELAY_OFF);
    }
  } 

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
  int iEmgStopState = 0;

  // read data from HC-05 if available
  iReadBytes = RecvCmd(arrcCmd, MAX_CMD_STRING_SIZE); 
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
      CmdProcess(iCmdID, g_arrcBTMsg);

      #ifdef SEND_BTRES
        SS_Bluetooth.println(g_arrcBTMsg);
      #endif
    }
    else
    {
      // do nothing
    }
  }

  // run relay timer task
  for(int i = 0; i < MAX_RELAY_COUNT; i++)
  {
    Rly[i]->TimerTask();
  }

}

/***********************************************************************************************/
/*! 
* \fn         :: BuletoothTerminal()
* \author     :: Vignesh S
* \date       :: 03-DEC-2018
* \brief      :: This function perform bluethooth terminal emulation
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void BuletoothTerminal()
{
  // if data is available in bluetooth, send it to debug
  if(SS_Bluetooth.available())
  {
    Serial.write(SS_Bluetooth.read());
    // echo
    //SS_Bluetooth.write(SS_Bluetooth.read());
    //digitalWrite(LED_BUILTIN, HIGH);
  }

  // if data is available in debug, send it to bluetooth 
  if(Serial.available())
  {
    SS_Bluetooth.write(Serial.read());
    //digitalWrite(LED_BUILTIN, LOW);
  }
}

/***********************************************************************************************/
/*! 
* \fn         :: SelfTest()
* \author     :: Vignesh S
* \date       :: 02-DEC-2018
* \brief      :: This function perform selftest of onboard peripherals
* \param[in]  :: iTestCount (number of times to repeat the test)
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void SelfTest(int iTestCount)
{
  int iCount = 0;
  int i = 0;

  #ifdef PRINT_DEBUG
    sprintf(g_arrcMsg, "Performing Self Test..\r\nTest Count: %d", iTestCount);
    Serial.println(g_arrcMsg);
  #endif

  for(iCount = 0; iCount < iTestCount; iCount++)
  {
    // turn on Relay
    for (i = 0; i < MAX_RELAY_COUNT; i++)
    {
      Rly[i]->setState(RELAY_ON);
    }

    delay(5000);
    
    // turn off Relay
    for (i = 0; i < MAX_RELAY_COUNT; i++)
    {
      Rly[i]->setState(RELAY_OFF);
    }

    delay(100);   
  }
}

/***********************************************************************************************/
/*! 
* \fn         :: PrintBytes()
* \author     :: Vignesh S
* \date       :: 02-DEC-2018
* \brief      :: This function prints the buffer byte by byte 
* \param[in]  :: ucBuffer 
* \param[in]  :: iBuflen
* \return     :: None
*/
/***********************************************************************************************/
void PrintBytes(unsigned char *ucBuffer, int iBuflen)
{
  int iIndex = 0;

  if(ucBuffer == NULL)
  {
    return;
  }

  for(iIndex = 0; iIndex < iBuflen; iIndex++)
  {
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg,"BY%0d: %#X[%c]", ucBuffer[iIndex]);
      Serial.println(g_arrcMsg);
    #endif
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
  int iRetVal = 0;
  int iPwd = 0;

  if((parrcCmd == NULL) || (out_iCmdID == NULL) || (iCmdLen <= 0))
  {
    return false;
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
    
    *out_iCmdID = CMD_STR_RELAY_ON_ID;
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
   
   *out_iCmdID = CMD_STR_RELAY_ON_TIMER_ID; 
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
    
    *out_iCmdID = CMD_STR_RELAY_OFF_ID;
    return true;
  }
  else if(StrnCmp(parrcCmd, CMD_STR_START_TEST, iCmdLen) == true)
  {
    *out_iCmdID = CMD_STR_START_TEST_ID;
    return true;
  }
  else if(StrnCmp(parrcCmd, CMD_STR_CHANGE_PWD, iCmdLen) == true)
  {
    iRetVal = sscanf(parrcCmd, CMD_STR_CHANGE_PWD "%d", &iPwd);
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
        sprintf(g_arrcMsg,"Cmd: %s\r\nPwd: %d", parrcCmd, iPwd);
        Serial.println(g_arrcMsg);
      #endif

      *out_iCmdID = CMD_STR_CHANGE_PWD_ID;
      return true;
    }
  }
  else if (StrnCmp(parrcCmd, CMD_STR_RELAY_OFF_ALL, iCmdLen) == true)
  {
    *out_iCmdID = CMD_STR_RELAY_OFF_ALL_ID;
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
* \fn         :: StrnCmp()
* \author     :: Vignesh S
* \date       :: 05-DEC-2018
* \brief      :: This function compares two strings, returns true if they are identical else
*                false.  
* \param[in]  :: pString1, pString2, iLen 
* \return     :: true or false
*/
/***********************************************************************************************/
bool StrnCmp(char *pString1, char *pString2, int iLen)
{
  if((pString1 == NULL) || (pString2 == NULL) || (iLen <= 0))
  {
    return false;
  }

  for(int iIndex = 0; (iIndex < iLen); iIndex++)
  {
    if(pString1[iIndex] != pString2[iIndex])
    {
      return false;
    }
  }

  return true;
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
void CmdProcess(int iCmdID, char *pResponse)
{ 
  int iRetVal = 0;
  int i = 0;

  if(pResponse == NULL)
  {
    return;
  }

  switch(iCmdID)
  {
    case CMD_STR_RELAY_ON_ID:
      sprintf(pResponse, "%s %d", CMD_STR_RELAY_ON, g_ucRlySel);

      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay %d ON", g_ucRlySel);
        Serial.println(g_arrcMsg);
      #endif

      Rly[g_ucRlySel -1]->setState(RELAY_ON);
      storeRelayStates();
    break;

    case CMD_STR_RELAY_ON_TIMER_ID:
      
      sprintf(pResponse, "%s %d", CMD_STR_RELAY_ON_TIMER, g_ucRlySel);

      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay %d ON for %ld seconds", g_ucRlySel, g_ulOnTimeSec);
        Serial.println(g_arrcMsg);
      #endif
      
      Rly[g_ucRlySel -1]->setTimer(g_ulOnTimeSec);
      storeRelayStates();
    break;

    case CMD_STR_RELAY_OFF_ID:

      sprintf(pResponse, "%s %d", CMD_STR_RELAY_OFF, g_ucRlySel);

      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay %d OFF", g_ucRlySel);
        Serial.println(g_arrcMsg);
      #endif

      Rly[g_ucRlySel -1]->setState(RELAY_OFF);
    break;

    case CMD_STR_START_TEST_ID:

      sprintf(pResponse, "%s", CMD_STR_START_TEST);

      // perform self test
      SelfTest(0x01);
      storeRelayStates();
    break;

    case CMD_STR_CHANGE_PWD_ID:

      sprintf(pResponse, "%s", CMD_STR_CHANGE_PWD);
      // change Password
      iRetVal = HC05_ChangePwd(g_iPwd);
      if(iRetVal != 0)
      {
        #ifdef PRINT_DEBUG
          sprintf(g_arrcMsg,"ERROR: Failed to change Password..!");
          Serial.println(g_arrcMsg);
        #endif
      }
    break;

    case CMD_STR_RELAY_OFF_ALL_ID:
      sprintf(pResponse, "%s", CMD_STR_RELAY_OFF_ALL);

      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning all Relay OFF");
        Serial.println(g_arrcMsg);
      #endif

      for(i = 0; i < MAX_RELAY_COUNT; i++)
      {
        Rly[i]->setState(RELAY_OFF);
      }
      storeRelayStates();
    break;

    default:
      ;// do nothing 
  }
}

/***********************************************************************************************/
/*! 
* \fn         :: RecvCmd()
* \author     :: Vignesh S
* \date       :: 07-DEC-2018
* \brief      :: This function cheks HC-05 serial port for received data and if data is available
*                then reads it, the number of bytes read is returned.
* \param[in]  :: pBuff, iBuflen
* \return     :: iIndex
*/
/***********************************************************************************************/
int RecvCmd(char *pBuff, int iBuflen)
{
  int iIndex = 0;

  if(pBuff == NULL)
  {
    return -1;
  }
  
  while(iIndex < iBuflen)
  {
    delay(HC05_SERIAL_READ_DELAY_MS);
    
    if(SS_Bluetooth.available())
    {
      pBuff[iIndex] = SS_Bluetooth.read();
      iIndex++;
    }
    else
    {
      break;
    }
  }

  return iIndex;
}

/***********************************************************************************************/
/*! 
* \fn         :: HC05_ChangePwd()
* \author     :: Vignesh S
* \date       :: 16-SEP-2019
* \brief      :: This function change the bluetooth paring password of the HC-05 modlue by 
*                switching the modlue to AT mode.
* \param[in]  :: iPwd
* \return     :: iRetVal
*/
/***********************************************************************************************/
int HC05_ChangePwd(int iPwd)
{
  char arrcCmd[MAX_CMD_STRING_SIZE] = {0};
  int iRetVal = 0;
  int iReadBytes = 0;

  // validate the input 
  if((iPwd < 0) || (iPwd > 9999))
  {
    return -1;
  }

  // switch the bluetooth modlue into AT mode
  digitalWrite(HC05_EN, HIGH);
  delay(10);

  // send change password command
  sprintf(g_arrcBTMsg, "AT+PSWD=%04d\r\n", iPwd);

  #ifdef PRINT_DEBUG
    sprintf(g_arrcMsg, "Changing Password to: %d", iPwd);    
    Serial.println(g_arrcMsg);
  #endif

  SS_Bluetooth.write(g_arrcBTMsg);

  // verify response
  delay(10);
  iReadBytes = RecvCmd(arrcCmd, MAX_CMD_STRING_SIZE); 
  if(iReadBytes > 0)
  {
    if(StrnCmp(arrcCmd, "OK", iReadBytes) == true)
    {
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Changing Password Failed\nInvalid Response: %s", arrcCmd);    
        Serial.println(g_arrcMsg);
      #endif
    }
    else
    {
     #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Changing Password success");    
        Serial.println(g_arrcMsg);
      #endif 
    }
  }

  // switch the bluetooth modlue back to normal mode
  digitalWrite(HC05_EN, LOW);
  delay(10);

  return iRetVal;
}

/***********************************************************************************************/
/*! 
* \fn         :: storeRelayStates()
* \author     :: Vignesh S
* \date       :: 21-DEC-2020
* \brief      :: This function stores the present relay states to EEPROM starting from zeroth
*                location.
* \param[in]  :: none
* \return     :: none
*/
/***********************************************************************************************/
void storeRelayStates()
{
  uint8_t checksum = 0;
  uint8_t state = 0;

  for(int i = 0; i < MAX_RELAY_COUNT; i++)
  {
    state = Rly[i]->getState();
    EEPROM.put((i * sizeof(uint8_t)), state);
    checksum ^= state;
  }

  EEPROM.put(((MAX_RELAY_COUNT +1) * sizeof(uint8_t)), checksum);
}
