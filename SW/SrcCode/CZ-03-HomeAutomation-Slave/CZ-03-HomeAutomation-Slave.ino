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
#include <utility.h>
#include <serial_com.h>
#include <CmdProcess.h>

/*************************************** Pin Mappings ******************************************/
// Relay 
#define RELAY_01        7       /*DRV 16 - PD7 */
#define RELAY_02        8       /*DRV 17 - PB0 */
#define RELAY_03        9       /*DRV 18 - PB1 */
#define RELAY_04        10      /*DRV 19 - PB2 */
#define RELAY_05        PIN_A2  /*DRV 20 - PC2 / ADC2 */
#define MAX_RELAY_COUNT 5

/* Master pins */
#define MTX_PIN         2
#define MRX_PIN         3

/***********************************************************************************************/
/* comment the below macro to disable debug prints */
#define PRINT_DEBUG

#define MAX_DEBUG_MSG_SIZE                  128
#define MAX_CMD_STRING_SIZE                 32

#define SELF_TEST_COUNT                     0x00
#define MASTER_BUAD_RATE                    9600
#define DEBUG_BUAD_RATE                     9600

/****************************************** globals ********************************************/
/* SoftwareSerial (RX, TX) */
SoftwareSerial SS_Master(MRX_PIN, MTX_PIN);

/* Relay objects pointers */
Relay *Rly[MAX_RELAY_COUNT] = {0};
unsigned char g_ucRlyPin[MAX_RELAY_COUNT] = {RELAY_01, RELAY_02, RELAY_03, RELAY_04, RELAY_05};

#ifdef PRINT_DEBUG
  char g_arrcMsg[MAX_DEBUG_MSG_SIZE] = {0};
#endif

uint8_t g_address[2];
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

  // assign temporary address
  g_address[0] = 1;
  g_address[1] = 1;

  Relay_init();

  // Serial port initialization
  #ifdef PRINT_DEBUG
    Serial.begin(DEBUG_BUAD_RATE);
  #endif
  
  SS_Master.begin(MASTER_BUAD_RATE);

  // over ride TX pin to input state
  pinMode(MTX_PIN, INPUT);

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

  serial_packet cmd_packet = {0};
  serial_packet res_packet = {0};

  int iReadBytes = 0;

  // read data from serial port if available
  iReadBytes = recv_packet<SoftwareSerial>(SS_Master, &cmd_packet); 
  if(iReadBytes > 0)
  {
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg, "Received: [%d] bytes", iReadBytes);
      Serial.println(g_arrcMsg);
    #endif

    // validate the command
    if(is_valid_packet(&cmd_packet, g_address) == true)
    {
      // if valid command is received, process it
      CmdProcess(&cmd_packet, &res_packet);

      pinMode(MTX_PIN, OUTPUT);
      send_packet<SoftwareSerial>(SS_Master, &res_packet);
      pinMode(MTX_PIN, INPUT);
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
* \fn         :: CmdProcess()
* \author     :: Vignesh S
* \date       :: 05-DEC-2018
* \brief      :: This function processes the recceived command and preforms corresponding task
* \param[in]  :: iCmdID
* \param[out] :: ipResponse
* \return     :: None
*/
/***********************************************************************************************/
void CmdProcess(serial_packet *cmd_packet, serial_packet *res_packet)
{ 
  int i = 0;
  uint8_t data_size = 0;
  data_packets Udata_packets = {0};

  Udata_packets.ptr = (uint16_t*)cmd_packet->data;

  switch(cmd_packet->cmdID)
  {
    case CMD_TEST_ID:

      // perform self test
      SelfTest(0x01);
      storeRelayStates();
      data_size = 0;
    break;

    case CMD_RLY_ON_ID:

      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay %d ON", Udata_packets.Prelay_on->relay_num);
        Serial.println(g_arrcMsg);
      #endif

      Rly[Udata_packets.Prelay_on->relay_num -1]->setState(RELAY_ON);
      storeRelayStates();

      data_size = 0;
    break;

    case CMD_RLY_ON_TIMER_ID:
      
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay %d ON for %ld seconds", 
                Udata_packets.Prelay_on_timer->relay_num, Udata_packets.Prelay_on_timer->on_time_sec);
        Serial.println(g_arrcMsg);
      #endif
      
      Rly[Udata_packets.Prelay_on_timer->relay_num -1]->setTimer(Udata_packets.Prelay_on_timer->on_time_sec);
      storeRelayStates();

      data_size = 0;
    break;

    case CMD_RLY_OFF_ID:

      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay %d OFF", Udata_packets.Prelay_off->relay_num);
        Serial.println(g_arrcMsg);
      #endif

      Rly[Udata_packets.Prelay_off->relay_num -1]->setState(RELAY_OFF);

      data_size = 0;
    break;

    case CMD_RLY_OFF_ALL_ID:

      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning all Relays OFF");
        Serial.println(g_arrcMsg);
      #endif

      for(i = 0; i < MAX_RELAY_COUNT; i++)
      {
        Rly[i]->setState(RELAY_OFF);
      }
      
      storeRelayStates();
      data_size = 0;
    break;

    default:
      ;// do nothing 


    // fill the respose packet
    res_packet->header = SERIAL_RES_HEADER;
    res_packet->address[0] = cmd_packet->address[0];
    res_packet->address[1] = cmd_packet->address[1];
    res_packet->cmdID = cmd_packet->cmdID;
    res_packet->data_size = data_size;
    res_packet->checksum = compute_checksum(res_packet);
  }
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
  if(SS_Master.available())
  {
    Serial.write(SS_Master.read());
    // echo
    //SS_Master.write(SS_Master.read());
    //digitalWrite(LED_BUILTIN, HIGH);
  }

  // if data is available in debug, send it to bluetooth 
  if(Serial.available())
  {
    SS_Master.write(Serial.read());
    //digitalWrite(LED_BUILTIN, LOW);
  }
}