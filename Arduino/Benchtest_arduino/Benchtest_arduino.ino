#include <OneWire.h>
#include <EEPROM.h>
#include <Wire.h>
#include "HX711.h"

#define DEFAULT_DEBUG_MODE 0
#define DEFAULT_PROTOCOL_MODE 1

#define DATA_PIN_NUM 10
#define LOADCELL_SCK_PIN_NUM 11
#define LOADCELL_DATA_PIN_NUM 12
#define LED_PIN_NUM 13

#define TEMP_MAX 65
#define CMD_BUF_SIZE  64
#define PARAM_BUF_SIZE 8
#define BAUD_RATE 115200
#define CMDS_COUNT 13
#define SENSORS_MAX 15
#define SCRATCHPAD_BYTES_COUNT 9
#define ROM_SIZE 8
#define CMD_CHARS_MAX 10
#define DESC_CHARS_MAX 32
#define READ_SENSORS_DELAY_MS 1000
#define START_BYTE 0xF0
#define ACK_BYTE 0xF1
#define READ_DATA_LENGTH 13
#define SEARCH_DATA_LENGTH 8
#define CMD_TYPE_PROTOCOL 0
#define CMD_TYPE_DEBUG 1
#define INPUT_BUF_SIZE 32
#define LCD_I2C_ADDR 0x21
#define LCD_FLOAT_BUFFER_SIZE 16
#define LOADCELL_MAX_WEIGHT 10000 //load cell max weight in grams
#define SCALE_AVERAGES_NUM 10

#define CHANNEL0_PWM_PIN 9
#define CHANNEL1_PWM_PIN 5
#define CHANNEL2_PWM_PIN 6
#define CHANNEL3_PWM_PIN 3
#define CHANNEL0_ADC_PIN 0
#define CHANNEL1_ADC_PIN 1
#define CHANNEL2_ADC_PIN 2
#define CHANNEL3_ADC_PIN 3
#define CHANNEL0_ADC_OFFSET -4
#define CHANNEL1_ADC_OFFSET 0
#define CHANNEL2_ADC_OFFSET -2
#define CHANNEL3_ADC_OFFSET -2

#define CURRENT_CAL_TIMEOUT_MS  20000
#define LOADCELL_CAL_TIMEOUT_MS 60000
#define CURRENT_UPDATE_DELAY_MS 250
#define CURRENT_CHANNELS_COUNT 4
#define CURRENT_AVG_NUM 30
#define CURRENT_THRESHOLD 0.05
#define CURRENT_KP_GAIN 0.1
#define CURRENT_PWM_FACTOR 51
#define CURRENT_SENSOR_MV_PER_AMP 185
#define PWM_MAX_VALUE 255
#define CAL_RESOLUTION 10
#define CURRENT_VAR_RATIO 100 //ratio by which current variables are multiplied to fit int types, ex: 3.5A = 350 if ratio is 100
#define CURRENT_MAX_SETPOINT 5.0 * CURRENT_VAR_RATIO
#define CURRENT_MAX_ABSOLUTE 5.5 * CURRENT_VAR_RATIO
#define CURRENT_MAX_TOTAL 14.0 * CURRENT_VAR_RATIO

#define FLOAT_BYTE_COUNT 4
#define INT_BYTE_COUNT 2
#define EEPROM_PTSX_BASE_ADDR 0
#define EEPROM_SLOPE_BASE_ADDR (EEPROM_PTSX_BASE_ADDR + (FLOAT_BYTE_COUNT * CURRENT_CHANNELS_COUNT * (CAL_RESOLUTION + 1))) //for a resolution of n there is (n+1) points
#define EEPROM_OFFSET_BASE_ADDR (EEPROM_SLOPE_BASE_ADDR + (FLOAT_BYTE_COUNT * CURRENT_CHANNELS_COUNT * CAL_RESOLUTION))
#define EEPROM_SCALE_RATIO_BASE_ADDR 985
#define EEPROM_SCALE_OFFSET_BASE_ADDR 990

OneWire  ds(DATA_PIN_NUM);  // on pin 10 (a 4.7K resistor is necessary)
HX711 scale(LOADCELL_DATA_PIN_NUM, LOADCELL_SCK_PIN_NUM);    // parameter "gain" is ommited; the default value 128 is used by the library

unsigned long lastTimeRead = 0;
unsigned long lastTimeCurrentUpdate = 0;
char inputCmd[INPUT_BUF_SIZE];
char inputParam1[INPUT_BUF_SIZE];
char inputParam2[INPUT_BUF_SIZE];
char *inputCmdPtr = inputCmd;
char *inputParam1Ptr = inputParam1;
char *inputParam2Ptr = inputParam2;
unsigned char byteCount;
unsigned char cmdLength;
unsigned char sensorsCount = 0;
bool readContinuous = 0;
bool readOnce = 0;
byte readState = 0;
bool debugMode = DEFAULT_DEBUG_MODE;
bool protocolMode = DEFAULT_PROTOCOL_MODE;
bool readReadyToSend = 0;
byte serialEventState = 0;
bool serialParserEnable = 1;
bool overTemp = 0;
bool overCurrent = 0;
float weight = 0;

struct currentChannel_t
{
  unsigned char pwmPin;
  unsigned char adcPin;
  char adcOffset;
  int readings[CURRENT_AVG_NUM];  // the readings from the analog input
  int readIndex;              // the index of the current reading
  int total;                  // the running total
  int average;                // the average
  int currentSetpoint;
  int pwm;
  union
  {
    int value;
    byte bytes[INT_BYTE_COUNT];
  }current;
};

struct currentChannel_t currentChannelList[CURRENT_CHANNELS_COUNT] = 
{
  { CHANNEL0_PWM_PIN, CHANNEL0_ADC_PIN, CHANNEL0_ADC_OFFSET, {0}, 0, 0, 0, 0, 0, 0 },
  { CHANNEL1_PWM_PIN, CHANNEL1_ADC_PIN, CHANNEL1_ADC_OFFSET, {0}, 0, 0, 0, 0, 0, 0 },
  { CHANNEL2_PWM_PIN, CHANNEL2_ADC_PIN, CHANNEL2_ADC_OFFSET, {0}, 0, 0, 0, 0, 0, 0 },
  { CHANNEL3_PWM_PIN, CHANNEL3_ADC_PIN, CHANNEL3_ADC_OFFSET, {0}, 0, 0, 0, 0, 0, 0 }
};

struct sensor {
  byte rom[ROM_SIZE];
  byte scratchPad[SCRATCHPAD_BYTES_COUNT];
  bool scratchPadCrcValid;
  union
  {
    float value;
    byte bytes[FLOAT_BYTE_COUNT];
  }temp;
};

struct sensor sensorsList[SENSORS_MAX];

enum programStateType { mainState, searchState, tempsState, infosState, optionsState };

struct cmdType {
  byte cmdId;
  const char *cmdString;
  const char *description;
};

enum cmdNum { 
  helpCmd, 
  searchCmd,
  readCmd,
  startCmd,
  stopCmd,
  resetCmd,
  modeCmd,
  listCmd,
  setCurrentCmd,
  getCurrentCmd,
  readScaleCmd,
  calScaleCmd,
  resetScaleCmd
};

struct cmdType cmdList[CMDS_COUNT]=
{
  { 0x00, "help",       "Lists available commands" },
  { 0x01, "search",     "Search available sensors" },
  { 0x02, "readTemp",   "Get temperature once" },
  { 0x03, "start",      "Get temperature continuously" },
  { 0x04, "stop",       "Stop continuous temperature" }, 
  { 0x05, "resetTemp",  "Reset sensor list" },
  { 0x06, "mode",       "0: protocol & debug, 1: protocol, 2: debug"},
  { 0x07, "list",       "Print sensors list" },
  { 0x08, "setCurrent", "Set the current of TEC #"},
  { 0x09, "getCurrent", "Get the current value of the TECs"},
  { 0x0a, "readScale",  "Get the scale weight value"},
  { 0x0b, "calScale",   "Calibrate the scale"},
  { 0x0c, "resetScale", "Reset the scale value to zero"}
};

programStateType programState = mainState;

void processCommand(bool cmdType);
void searchSensors(void);
void readSensors(void);
bool getReadDelay(void);
void resetReadDelay(void);
byte CRC8(const byte *data, byte len);
void sendTemp();
void sendList();
void sendData(byte cmd, byte *data, byte len);
void calCurrent(unsigned char channel);
void setCurrent(float current, unsigned char pin);
void blinkLed(int ms);
void updateCurrentParams(unsigned char channel, float *ptsx, float *slope, float *offset);
void printLcd(byte row, byte column, const char *str);
void clearLcd(void);
void printFloatLcd(byte row, byte column, const char *str, float number);
void initScale(void);
void resetScale(void);
float readScale(byte averages);
void calScale(void);
void checkOverCurrent(void);
void disableAllCurrent(void);
void readCurrents(void);
void updateCurrents(void);
bool getCurrentUpdateDelay(void);
void sendCurrent(void);

/*/////////////////////////////////////////////////////////
Setup
/////////////////////////////////////////////////////////*/
void setup()
{  
  Serial.begin(BAUD_RATE);
  pinMode(LED_PIN_NUM, OUTPUT);

  for(int i=0; i<CURRENT_CHANNELS_COUNT; i++)
  {
    setCurrent(0, i);
    analogWrite(currentChannelList[i].pwmPin, 0);
  }
  
  Wire.begin();
  clearLcd();
  printLcd(0, 1, ">Benchtest Oh Yeah<");

  Serial.println(F("Program started..."));
  searchSensors();
  readOnce = 1;
  initScale();
}

/*/////////////////////////////////////////////////////////
Main
/////////////////////////////////////////////////////////*/
void loop()
{
  readSensors();
  sendTemp();
  
//  float weight = readScale(30);
//  clearLcd();
//  printFloatLcd(7, 0, "Weight: ", weight);

  readCurrents();
  
  if(getCurrentUpdateDelay())
  {
//    weight = readScale(30);
    updateCurrents();

    printLcd(2, 0, "CH");
    printLcd(2, 4, "ADC");
    printLcd(2, 12, "Current");
    printLcd(3, 0, "0");
    printFloatLcd(3, 4, "", (float)currentChannelList[0].average);
    printFloatLcd(3, 12, "", (float)currentChannelList[0].current.value/CURRENT_VAR_RATIO);
    printLcd(4, 0, "1");
    printFloatLcd(4, 4, "", (float)currentChannelList[1].average);
    printFloatLcd(4, 12, "", (float)currentChannelList[1].current.value/CURRENT_VAR_RATIO);
    printLcd(5, 0, "2");
    printFloatLcd(5, 4, "", (float)currentChannelList[2].average);
    printFloatLcd(5, 12, "", (float)currentChannelList[2].current.value/CURRENT_VAR_RATIO);
    printLcd(6, 0, "3");
    printFloatLcd(6, 4, "", (float)currentChannelList[3].average);
    printFloatLcd(6, 12, "", (float)currentChannelList[3].current.value/CURRENT_VAR_RATIO);
//    printFloatLcd(7, 0, "Weight: ", weight);
  }
}

/*/////////////////////////////////////////////////////////
checkOverTemp
/////////////////////////////////////////////////////////*/
void checkOverTemp(void)
{
  bool overTempDetected = 0;
  for(int i=0; i<sensorsCount; i++)
  {
    if(sensorsList[i].temp.value > TEMP_MAX)
    {
      overTempDetected = 1;
      break;
    }
  }
  if(overTempDetected)
  {
    disableAllCurrent();
    overTemp = 1;
    Serial.println(F("CRITICAL: Over temperature detected!!"));
  }
  else
  {
    overTemp = 0;
  }
}

/*/////////////////////////////////////////////////////////
sendCurrent
/////////////////////////////////////////////////////////*/
void sendCurrent(void)
{
  byte dataBuf[ROM_SIZE];
  for(int i=0; i<CURRENT_CHANNELS_COUNT; i++)
  {
    for(int j=0; j<INT_BYTE_COUNT; j++)
    {
      dataBuf[j] = currentChannelList[i].current.bytes[j];
    }
    sendData(getCurrentCmd, dataBuf, sizeof(dataBuf));
  }
}

/*/////////////////////////////////////////////////////////
sendList
/////////////////////////////////////////////////////////*/
void sendList()
{
  byte dataBuf[ROM_SIZE];
  for(int i=0; i<sensorsCount; i++)
  {
    for(int j=0; j<ROM_SIZE; j++)
    {
      dataBuf[j] = sensorsList[i].rom[j];
    }
    sendData(searchCmd, dataBuf, sizeof(dataBuf));
  }
}

/*/////////////////////////////////////////////////////////
sendAck
/////////////////////////////////////////////////////////*/
void sendAck()
{
  Serial.write(ACK_BYTE);
}

/*/////////////////////////////////////////////////////////
sendTemp
/////////////////////////////////////////////////////////*/
void sendTemp()
{
  if(readReadyToSend)
  {
    byte dataBuf[READ_DATA_LENGTH];
    for(int i=0; i<sensorsCount; i++)
    {
      for(int j=0; j<ROM_SIZE; j++)
      {
        dataBuf[j] = sensorsList[i].rom[j];
      }
      for(int j=0; j<FLOAT_BYTE_COUNT; j++)
      {
        dataBuf[j+ROM_SIZE] = sensorsList[i].temp.bytes[j];
      }
      dataBuf[READ_DATA_LENGTH - 1] = sensorsList[i].scratchPadCrcValid;
      sendData(readCmd, dataBuf, sizeof(dataBuf));
    }
    readReadyToSend = 0;
  }
}

/*/////////////////////////////////////////////////////////
sendData
/////////////////////////////////////////////////////////*/
void sendData(byte cmd, byte *data, byte dataLength)
{
  if(protocolMode)
  {
    byte totalLength = dataLength + 1;
    byte buf[totalLength];
    buf[0] = cmd;
    for(int i=0; i<dataLength; i++)
    {
      buf[i+1] = data[i];
    }
    byte CRC = CRC8(buf, totalLength);
    Serial.write(START_BYTE);
    Serial.write(totalLength);
    for(int i=0; i<totalLength; i++)
    {
      Serial.write(buf[i]);
    }
    Serial.write(CRC);
    Serial.flush();
    Serial.print("\r\n");
  }
}

/*/////////////////////////////////////////////////////////
CRC8
/////////////////////////////////////////////////////////*/
//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
byte CRC8(const byte *data, byte len) 
{
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

/*/////////////////////////////////////////////////////////
getCurrentUpdateDelay
/////////////////////////////////////////////////////////*/
bool getCurrentUpdateDelay(void)
{
  unsigned long actualTime = millis();
  if((actualTime - lastTimeCurrentUpdate) >= CURRENT_UPDATE_DELAY_MS)
  {
    lastTimeCurrentUpdate = actualTime;
    return 1;
  }
  else
  {
    return 0;
  }
}

/*/////////////////////////////////////////////////////////
getReadDelay
/////////////////////////////////////////////////////////*/
bool getReadDelay(void)
{
  unsigned long actualTime = millis();
  if((actualTime - lastTimeRead) >= READ_SENSORS_DELAY_MS)
  {
    lastTimeRead = actualTime;
    return 1;
  }
  else
  {
    return 0;
  }
}

/*/////////////////////////////////////////////////////////
resetReadDelay
/////////////////////////////////////////////////////////*/
void resetReadDelay(void)
{
  lastTimeRead = millis();
}

/*/////////////////////////////////////////////////////////
searchSensors
/////////////////////////////////////////////////////////*/
void searchSensors()
{
  unsigned char initialSensorsCount = sensorsCount;
  byte addr[ROM_SIZE];
  bool knownAddr;
  
  if(sensorsCount < SENSORS_MAX)
  {
    Serial.print(F("\nSearching for sensors"));
    while(ds.search(addr))
    {
      knownAddr = 0;
      //check if the address found is already known
      for(byte i=0; i<sensorsCount; i++)
      {
        if(memcmp(&sensorsList[i].rom, addr, ROM_SIZE) == 0)
        {
          knownAddr = 1;
          break;
        }
      }
      if(!knownAddr)
      {
        if(sensorsCount < SENSORS_MAX)
        {
          if(OneWire::crc8(addr, (ROM_SIZE - 1)) == addr[(ROM_SIZE - 1)])   //verify the crc
          {
            Serial.print('.');
            memcpy(&sensorsList[sensorsCount].rom, addr, ROM_SIZE);
            sensorsCount++;
          }
          else
          {
            Serial.print(F("\nBad CRC on device \""));
            for(byte j=0; j<ROM_SIZE; j++)
            {
              Serial.print(addr[j], HEX);
              Serial.print(' ');
            }
            Serial.print(F("\", expected: "));
            Serial.print(OneWire::crc8(addr, (ROM_SIZE - 1)), HEX);
            Serial.print(F(", received: "));
            Serial.print(addr[(ROM_SIZE - 1)], HEX);
            Serial.print('\n');
          }
        }
        else
        {
          Serial.println(F("\nFound more new sensors, but max number of sensors already reached!"));
          break;
        }
      }
    }
    Serial.print('\n');
    if(sensorsCount - initialSensorsCount)
    {
      Serial.print(F("Found "));
      Serial.print(sensorsCount - initialSensorsCount);
      Serial.print(F(" new sensors:\n"));
      for(char i=initialSensorsCount; i<sensorsCount; i++)
      {
        for(char j=0; j<ROM_SIZE; j++)
        {
          Serial.print(sensorsList[i].rom[j], HEX);
          Serial.print(' ');
        }
        Serial.print('\n');
      }
    }
    else
      Serial.println(F("No new sensor"));
  }
  else
    Serial.println(F("\nMax number of sensors already reached, aborting search..."));
}

/*/////////////////////////////////////////////////////////
processCommand
/////////////////////////////////////////////////////////*/
void processCommand(bool cmdType)
{
  int cmd;
  bool cmdMatch = 0;
  unsigned char channel;
  int current;

  inputParam1[0] = 0;
  inputParam2[0] = 0;

  if(cmdType == CMD_TYPE_DEBUG)   //if received a debug command
  {
    const char *indexParam1;
    const char *indexParam2;
    indexParam1 = strchr(inputCmdPtr, ' '); //search for space character
    if(indexParam1 != NULL)
    {
      indexParam1++;
      indexParam2 = strchr(indexParam1, ' ');
      if(indexParam2 != NULL)
      {
        indexParam2++;
        strncpy(inputParam1Ptr, indexParam1, (indexParam2 - indexParam1 - 1));
        strncpy(inputParam2Ptr, indexParam2, INPUT_BUF_SIZE);
      }
      else
      {
        strncpy(inputParam1Ptr, indexParam1, INPUT_BUF_SIZE);
      }
      char *c = strchr(inputCmdPtr, ' ');
      *c = 0;
    }
    else  //clear parameter strings
    {
      inputParam1[0] = 0;
      inputParam2[0] = 0;
    }
    
    for(char i=0; i<CMDS_COUNT; i++)
    {
      const char *cmdPtr = cmdList[i].cmdString;
      if(strstr(inputCmdPtr, cmdPtr)) //compare received cmd with cmd list
      {
        cmd = i;
        cmdMatch = 1;
        break;
      }
    }
//    Serial.print(F("Param 1: \"")); Serial.print(inputParam1); Serial.print(F("\", param 2: \"")); Serial.print(inputParam2); Serial.println("\"");
  }
  else if(cmdType == CMD_TYPE_PROTOCOL)   //if received a protocol command
  {
    byte receivedCmd = inputCmd[0];  //get the received command number
    for(char i=0; i<CMDS_COUNT; i++)
    {
      if(receivedCmd == cmdList[i].cmdId)
      {
        cmd = i;
        cmdMatch = 1;
        break;
      }
    }
  }
  
  if(cmdMatch)
  {
    cmdMatch = 0;
    switch(cmd)
    {
      case helpCmd:
        Serial.println(F("\nList of available commands:"));
        for(char i=0; i<CMDS_COUNT; i++)
        {
          Serial.print(F(" - ")); Serial.print(cmdList[i].cmdId, HEX); Serial.print(F(" ")); Serial.print(cmdList[i].cmdString); Serial.print(F(" -> ")); Serial.println(cmdList[i].description);
        }
        Serial.println(F("End of list"));
//        sendAck();
      break;
      
      case searchCmd:
        programState = searchState;
        searchSensors();
//          sendList();
      break;

      case readCmd:
        readOnce = 1;
      break;

      case startCmd:
        readContinuous = 1;
        clearAndHome();
      break;

      case stopCmd:
        readContinuous = 0;
      break;

      case resetCmd:
        programState = searchState;
        ds.reset_search();
        sensorsCount = 0;
        Serial.println(F("\nSensors list was reset"));
//        sendAck();
      break;

      case modeCmd:
      {
        int mode = -1;
        if(cmdType == CMD_TYPE_DEBUG)
        {
            if(inputParam1[0] != 0)
            {
              mode = atoi(inputParam1Ptr);
            }
        }
        else if(cmdType == CMD_TYPE_PROTOCOL)
        {
            mode = inputCmd[1];
        }
        if(mode == 0)
        {
          debugMode = 1;
          protocolMode = 1;
          Serial.println(F("Debug and protocol activated"));
        }
        else if(mode == 1)
        {
          debugMode = 0;
          protocolMode = 1;
        }
        else if(mode == 2)
        {
          debugMode = 1;
          protocolMode = 0;
          Serial.println(F("Debug only activated"));
        }
        else
          Serial.println(F("Invalid mode received"));
      }
      break;

      case listCmd:
        if(sensorsCount == 0)
        {
          Serial.println(F("\n...Sensors list empty..."));
        }
        else
        {
          Serial.println(F("\nSensors list:"));
          for(char i=0; i<sensorsCount; i++)
          {
            for(char j=0; j<ROM_SIZE; j++)
            {
              Serial.print(sensorsList[i].rom[j], HEX);
              Serial.print(' ');
            }
            Serial.print('\n');
          }
        }
        sendList();
      break;

      case setCurrentCmd:
        switch(cmdType)
        {
          case CMD_TYPE_DEBUG:
            channel = atoi(inputParam1Ptr);
            current = (int)(atof(inputParam2Ptr) * CURRENT_VAR_RATIO);
            if(channel < 0 || channel >= CURRENT_CHANNELS_COUNT)
            {
              Serial.print(F("Channel selected out of range (0 - ")); Serial.print(CURRENT_CHANNELS_COUNT - 1); Serial.println(")");
              break;
            }
            setCurrent(current, channel);
          break;
          case CMD_TYPE_PROTOCOL:
            channel = inputCmd[1];
            current = (int)(inputCmd[2]) * (CURRENT_VAR_RATIO / 10);
            setCurrent(current, channel);
          break;
        }
      break;

      case getCurrentCmd:
        if(protocolMode)
        {
          sendCurrent();
        }
        if(debugMode)
        {
          for(int i=0; i<CURRENT_CHANNELS_COUNT; i++)
          {
            Serial.print(F("Channel "));
            Serial.print(i);
            Serial.print(F(" current: "));
            Serial.print((float)currentChannelList[i].current.value / CURRENT_VAR_RATIO);
            Serial.println("A");
          }
        }
      break;

      case readScaleCmd:
        union
        {
          float value;
          byte bytes[FLOAT_BYTE_COUNT];
        }weight;
        weight.value = readScale(SCALE_AVERAGES_NUM);
        byte dataBuf[FLOAT_BYTE_COUNT];
        for(int i=0; i<FLOAT_BYTE_COUNT; i++)
        {
          dataBuf[i] = weight.bytes[i];
        }
        sendData(readScaleCmd, dataBuf, sizeof(dataBuf));
      break;

      case calScaleCmd:
        calScale();
      break;

      case resetScaleCmd:
        resetScale();
      break;
      
      default:
        Serial.println(F("Unknown command! (default switch case)"));
      break;
    }
  }
  else
  {
    Serial.print(F("Received cmd: \"")); Serial.print(inputCmd); Serial.println(F("\" -> Unknown command!"));
  }
  inputCmd[0] = 0;
}

/*/////////////////////////////////////////////////////////
readSensors
/////////////////////////////////////////////////////////*/
void readSensors()
{
//  if(readOnce || readContinuous || readState)
//  {
    if(sensorsCount)
    {
      byte crc;
      char i;
      switch(readState)
      {
        case 0:
          if(readContinuous)
          {
            cursorHome();
          }
//          Serial.println(F("\nReading sensors..."));
          ds.reset();
          ds.skip();  //address all sensors
          ds.write(0x44, 1); // start conversion, with parasite power on at the end
          resetReadDelay();  //reset the counter
          readState = 1;
        break;

        case 1:
          if(getReadDelay())  //if READ_SENSORS_DELAY_MS is elapsed
          {
            readState = 2;
          }
        break;
        
        case 2:
          for(i=0; i<sensorsCount; i++)
          {
            ds.reset();
            ds.select(sensorsList[i].rom);  //address all sensors
            ds.write(0xBE);         // Read Scratchpad
            ds.read_bytes(sensorsList[i].scratchPad, SCRATCHPAD_BYTES_COUNT);
            crc = OneWire::crc8(sensorsList[i].scratchPad, (SCRATCHPAD_BYTES_COUNT - 1));
            if(crc == sensorsList[i].scratchPad[SCRATCHPAD_BYTES_COUNT - 1])
            {
              sensorsList[i].scratchPadCrcValid = 1;
            }
            else
            {
              sensorsList[i].scratchPadCrcValid = 0;
              Serial.print(F("Bad CRC for sensor #"));
              Serial.print(i, DEC);
              Serial.print(F(": expected "));
              Serial.print(sensorsList[i].scratchPad[SCRATCHPAD_BYTES_COUNT - 1], HEX);
              Serial.print(F(", received "));
              Serial.println(crc, HEX);
            }
            if(sensorsList[i].scratchPadCrcValid)
            {
              int16_t raw = (sensorsList[i].scratchPad[1] << 8) | sensorsList[i].scratchPad[0];
              byte cfg = (sensorsList[i].scratchPad[4] & 0x60);  // at lower res, the low bits are undefined, so let's zero them
              if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
              else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
              else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
              //// default is 12 bit resolution, 750 ms conversion time
              sensorsList[i].temp.value = (float)raw / 16.0;
            }
          }

          checkOverTemp();
                    
          if(readOnce || readContinuous)
          {
            readReadyToSend = 1;
            if(debugMode)
            {
              Serial.print("\n");
              for(i=0; i<sensorsCount; i++)
              {
                Serial.print(F("Temperature "));
                Serial.print(i, DEC);
                Serial.print(": ");
                Serial.println(sensorsList[i].temp.value);
              }
            }
          }
          readOnce = 0;
          readState = 0;
          break;
      }
    }
    else
    {
      readOnce = 0;
      Serial.println(F("\nSensor list is empty, execute \"search\" command first"));
    }
//  }
}

/*/////////////////////////////////////////////////////////
serialEvent
/////////////////////////////////////////////////////////*/
void serialEvent()
{
  unsigned char len;
  
  if(serialParserEnable)
  {
    char inChar;
    
    while (Serial.available())
    {
      inChar = (char)Serial.read();   //get new byte
  
      switch(serialEventState)
      {
        case 0:
          if((byte)inChar == START_BYTE)
          {
            serialEventState = 1;
          }
          else
          {
            inputCmd[0] = inChar;
            inputCmd[1] = 0;
            
            serialEventState = 3;
          }
          break;
  
        case 1:   //get the length byte
          cmdLength = (unsigned char)inChar;
//          Serial.print("Length = "); Serial.println(inChar, DEC);
          serialEventState = 2;
          byteCount = 0;
          break;
  
        case 2: 
          if(byteCount < cmdLength)
          {
            inputCmd[byteCount] = inChar;
            byteCount++;
//            Serial.print("Read byte: "); Serial.println(inChar, HEX);
          }
          else                                  //all bytes received, compare CRC
          {
//            Serial.print("Received CRC: "); Serial.println(inChar, HEX);
            if(byteCount != cmdLength)
            {
              Serial.println(F("Error: Command length mismatch!"));
            }
            
            byte buf[byteCount];
            for(int i=0; i<byteCount; i++)
            {
                buf[i] = (byte)inputCmd[i];
            }
            
            byte CRC = CRC8(buf, byteCount); //compute CRC
            byte receivedCRC = (byte)inChar;
            if(CRC == receivedCRC)  //compare CRC with the one received
            {
              blinkLed(100);
              sendAck();
              processCommand(CMD_TYPE_PROTOCOL);
            }
            else
            {
              Serial.print(F("Invalid CRC, expected: ")); Serial.print(CRC, HEX); Serial.print(F(", received: ")); Serial.println(receivedCRC, HEX);
            }
            serialEventState = 0;
          }
          break;
  
        case 3:
          if (inChar == '\n' || inChar == '\r')   //if newline, full command received
          {
            processCommand(CMD_TYPE_DEBUG);
            serialEventState = 0;
          }
          else
          {
            if((byte)inChar == START_BYTE)
            {
              inputCmd[0] = 0;
              serialEventState = 1;
            }
            else
            {
                len = strlen(inputCmdPtr); 
                inputCmd[len] = inChar; //add received char to String
                inputCmd[len+1] = 0;
            }
          }
          break;
      }
    }
  }
}

/*/////////////////////////////////////////////////////////
blink
/////////////////////////////////////////////////////////*/
void blinkLed(int ms)
{
  digitalWrite(LED_PIN_NUM, HIGH);
  delay(ms);
  digitalWrite(LED_PIN_NUM, LOW);
}

/*/////////////////////////////////////////////////////////
cursorHome
/////////////////////////////////////////////////////////*/
// send the cursor home
void cursorHome()
{
  Serial.write(27); // ESC
  Serial.print("[H"); // cursor to home
}

/*/////////////////////////////////////////////////////////
clearAndHome
/////////////////////////////////////////////////////////*/
// clear the terminal screen and send the cursor home
void clearAndHome()
{
  Serial.write(27); // ESC
  Serial.print("[2J"); // clear screen
  Serial.write(27); // ESC
  Serial.print("[H"); // cursor to home
}

