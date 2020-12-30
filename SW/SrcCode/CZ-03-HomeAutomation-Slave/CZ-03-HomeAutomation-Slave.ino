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
#define BUS_PIN         2

/***********************************************************************************************/
/* comment the below macro to disable debug prints */
#define PRINT_DEBUG

#define MAX_DEBUG_MSG_SIZE                  128
#define MAX_CMD_STRING_SIZE                 32

#define SELF_TEST_COUNT                     0x00
#define MASTER_BUAD_RATE                    9600
#define DEBUG_BUAD_RATE                     9600

#define SLAVE_ADDRESS                       "1.3"
/****************************************** globals ********************************************/
/* SoftwareSerial (RX, TX) */
SoftwareSerial SS_Master(BUS_PIN, BUS_PIN);

/* Relay objects pointers */
Relay *Rly[MAX_RELAY_COUNT] = {0};
unsigned char g_ucRlyPin[MAX_RELAY_COUNT] = {RELAY_01, RELAY_02, RELAY_03, RELAY_04, RELAY_05};

#ifdef PRINT_DEBUG
  char g_arrcMsg[MAX_DEBUG_MSG_SIZE] = {0};
#endif
uint8_t g_dev_addr[2];
uint8_t g_recv_addr[2];
uint8_t g_Test_Count;
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
  
  // Serial port initialization
  #ifdef PRINT_DEBUG
    Serial.begin(DEBUG_BUAD_RATE);
  #endif

  str_to_addr(SLAVE_ADDRESS, g_dev_addr);
  set_dev_addr(g_dev_addr);
  #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg, "Device Address: %03d.%03d", g_dev_addr[0], g_dev_addr[1]);
      Serial.println(g_arrcMsg);
  #endif

  Relay_init();

  SS_Master.begin(MASTER_BUAD_RATE);

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

  int iReadBytes = 0;
  int8_t ret = 0;
  cmd_res_pkt cmd_pkt = {0};
  cmd_res_pkt res_pkt = {0};

  // read data from serial port if available
  // we want to receive packets form any address
  str_to_addr(SERIAL_ADDR_ANY, g_recv_addr);
  iReadBytes = recvfrom<SoftwareSerial>(SS_Master, g_recv_addr, (uint8_t *)&cmd_pkt, sizeof(cmd_pkt), DEFAULT_RECV_TIMEOUT_MS); 
  if(iReadBytes > 0)
  {
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg, "Received: [%d] bytes from %03d.%03d", iReadBytes, g_recv_addr[0], g_recv_addr[1]);
      Serial.println(g_arrcMsg);
    #endif

    // validate the command
    if(cmd_pkt.cmdID <= MAX_CMD_ID)
    {
      // if valid command is received, process it
      CmdProcess(&cmd_pkt, &res_pkt);

      ret = sendto<SoftwareSerial>(SS_Master, g_recv_addr, (uint8_t *)&res_pkt, (res_pkt.data_size + CMD_DATA_HEADER_SIZE));
      if(ret != (res_pkt.data_size + CMD_DATA_HEADER_SIZE))
      {
        #ifdef PRINT_DEBUG
          sprintf(g_arrcMsg, "sendto failed..!");
          Serial.println(g_arrcMsg);
          get_err_str(ret, g_arrcMsg, sizeof(g_arrcMsg));
          Serial.println(g_arrcMsg);
        #endif
      }
    }
    else
    {
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "invalid command ID: %d", cmd_pkt.cmdID);
        Serial.println(g_arrcMsg);
      #endif
    }
  }
  else
  {
    // #ifdef PRINT_DEBUG
    //   sprintf(g_arrcMsg, "recvfrom failed..!");
    //   Serial.println(g_arrcMsg);
    //   get_err_str(iReadBytes, g_arrcMsg, sizeof(g_arrcMsg));
    //   Serial.println(g_arrcMsg);
    // #endif
  }

  // run relay timer task
  for(int i = 0; i < MAX_RELAY_COUNT; i++)
  {
    Rly[i]->TimerTask();
  }

  // run self test if test count is greater than 0
  if(g_Test_Count > 0)
  {
    SelfTest(g_Test_Count);
    g_Test_Count = 0;
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
void CmdProcess(cmd_res_pkt *cmd_packet, cmd_res_pkt *res_packet)
{ 
  int i = 0;
  uint8_t data_size = 0;
  cmd_res_data Udata = {0};

  Udata.ptr = (uint16_t*)cmd_packet->data;

  switch(cmd_packet->cmdID)
  {
    case CMD_TEST_ID:

      // perform self test
      //SelfTest(0x01);
      g_Test_Count = 1;
      storeRelayStates();
      data_size = 0;
    break;

    case CMD_RLY_ON_ID:

      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay %d ON", Udata.Prelay_on->relay_num);
        Serial.println(g_arrcMsg);
      #endif

      Rly[Udata.Prelay_on->relay_num -1]->setState(RELAY_ON);
      storeRelayStates();

      data_size = 0;
    break;

    case CMD_RLY_ON_TIMER_ID:
      
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay %d ON for %ld seconds", 
                Udata.Prelay_on_timer->relay_num, Udata.Prelay_on_timer->on_time_sec);
        Serial.println(g_arrcMsg);
      #endif
      
      Rly[Udata.Prelay_on_timer->relay_num -1]->setTimer(Udata.Prelay_on_timer->on_time_sec);
      storeRelayStates();

      data_size = 0;
    break;

    case CMD_RLY_OFF_ID:

      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay %d OFF", Udata.Prelay_off->relay_num);
        Serial.println(g_arrcMsg);
      #endif

      Rly[Udata.Prelay_off->relay_num -1]->setState(RELAY_OFF);

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
  }

  res_packet->cmdID = cmd_packet->cmdID;
  res_packet->data_size = data_size;
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
