void readCurrents()
{
  int adc;
  for(int i=0; i<CURRENT_CHANNELS_COUNT; i++)
  {
    adc = analogRead(currentChannelList[i].adcPin) - currentChannelList[i].adcOffset;
    currentChannelList[i].total = currentChannelList[i].total - currentChannelList[i].readings[currentChannelList[i].readIndex];
    currentChannelList[i].readings[currentChannelList[i].readIndex] = adc;
    currentChannelList[i].total = currentChannelList[i].total + currentChannelList[i].readings[currentChannelList[i].readIndex];
    currentChannelList[i].readIndex = currentChannelList[i].readIndex + 1;
  
    if (currentChannelList[i].readIndex >= CURRENT_AVG_NUM) 
    {
      currentChannelList[i].readIndex = 0;
    }
  }
}

void getCurrentsAvg(void)
{
  for(int i=0; i<CURRENT_CHANNELS_COUNT; i++)
  {
    currentChannelList[i].average = currentChannelList[i].total / CURRENT_AVG_NUM;
    currentChannelList[i].current = ( ( ((currentChannelList[i].average / 1024.0) * 5000) - 2500) / CURRENT_SENSOR_MV_PER_AMP );
  }
}

void updateCurrents(void)
{
  float error;
  for(int i=0; i<CURRENT_CHANNELS_COUNT; i++)
  {
    error = currentChannelList[i].currentSetpoint - currentChannelList[i].current;
    currentChannelList[i].pwm += round(error * CURRENT_KP_GAIN * CURRENT_PWM_FACTOR);
    if(currentChannelList[i].pwm < 0)
    {
      currentChannelList[i].pwm = 0;
    }
    if(currentChannelList[i].pwm > PWM_MAX_VALUE)
    {
      currentChannelList[i].pwm = PWM_MAX_VALUE;
    }
    analogWrite(currentChannelList[i].pwmPin, currentChannelList[i].pwm);
//    Serial.print(F("error: "));
//    Serial.println(error);
//    Serial.print(F("pwm: "));
//    Serial.println(currentChannelList[i].pwm);
  }
}

void calCurrent(unsigned char channel)
{
//  char bufBytes[3];
//  float ptsY[CAL_RESOLUTION + 1];
//  float ptsX[CAL_RESOLUTION + 1];
//  float slope[CAL_RESOLUTION];
//  float offset[CAL_RESOLUTION];
//  clearAndHome();
//  serialParserEnable = 0;
//  int i;
//  unsigned char cmdPwm;
//  float measuredCurrent;
//
//  Serial.setTimeout(CURRENT_CAL_TIMEOUT_MS);
//
//  while(Serial.available()) //flush the input buffer
//  {
//    Serial.read();
//  }
//  
//  Serial.print(F("\n\nCurrent calibration\n\n"));
//  if(channel < 0 || channel > (CURRENT_CHANNELS_COUNT - 1))
//  {
//    Serial.print(F("Invalid channel selected, exiting...\n"));
//    exit;
//  }
//
//  Serial.print(F("Channel ")); Serial.print(channel, DEC); Serial.println(F(" selected"));
//  Serial.print(F("Begin calibration? (y/n)\n"));
//  Serial.readBytesUntil('\n', bufBytes, sizeof(bufBytes));
////  bufBytes.trim();
//  if(bufBytes[0] != 'y')
//  {
//    Serial.println(F("Exiting calibration..."));
//    serialParserEnable = 1;
//    return;
//  }
//  Serial.println(F("Starting calibration..."));
//
//  for(i=0; i<=CAL_RESOLUTION; i++)
//  {
//    cmdPwm = ((float)i/CAL_RESOLUTION) * PWM_MAX_VALUE;
//    analogWrite(currentControlPin[channel], cmdPwm);
//    Serial.print(F("Current PWM set to "));
//    Serial.println(cmdPwm);
//    Serial.println(F("Enter measured current:"));
//    measuredCurrent = Serial.parseFloat();
//    Serial.print(F("Received measured current of "));
//    Serial.println(measuredCurrent);
//    ptsY[i] = cmdPwm;
//    ptsX[i] = measuredCurrent;
//  }
//
//  for(i=0; i<CAL_RESOLUTION; i++)
//  {
//    slope[i] = (ptsY[i+1] - ptsY[i]) / (ptsX[i+1] - ptsX[i]);
//    offset[i] = ptsY[i] - (slope[i] * ptsX[i]);
//    Serial.print(i); Serial.print(F(" slope: ")); Serial.print(slope[i]); Serial.print(F(", offset: ")); Serial.println(offset[i]);
//  }
//
//  updateCurrentParams(channel, ptsX, slope, offset);
//  setCurrent(0, channel);
//
//  serialParserEnable = 1;
}
//
//void updateCurrentParams(unsigned char channel, float *ptsx, float *slope, float *offset)
//{
//  int i,j;
//
//  for(i=0; i<(CAL_RESOLUTION+1); i++)
//  {
//    EEPROM.put(EEPROM_PTSX_BASE_ADDR + (channel * FLOAT_BYTE_COUNT * (CAL_RESOLUTION+1)) + (i * FLOAT_BYTE_COUNT), ptsx[i]); //addr = base addr + channel addr + point addr
//  }
//  for(i=0; i<CAL_RESOLUTION; i++)
//  {
//    EEPROM.put(EEPROM_SLOPE_BASE_ADDR + (channel * FLOAT_BYTE_COUNT * CAL_RESOLUTION) + (i * FLOAT_BYTE_COUNT), slope[i]);
//    EEPROM.put(EEPROM_OFFSET_BASE_ADDR + (channel * FLOAT_BYTE_COUNT * CAL_RESOLUTION) + (i * FLOAT_BYTE_COUNT), offset[i]);
//  }
//
//  for(j=0; j<CURRENT_CHANNELS_COUNT; j++)
//  {
//    for(i=0; i<CAL_RESOLUTION; i++)
//    {
//      float s, o;
//      EEPROM.get(EEPROM_SLOPE_BASE_ADDR + (j * FLOAT_BYTE_COUNT * CAL_RESOLUTION) + (i * FLOAT_BYTE_COUNT), s);
//      EEPROM.get(EEPROM_OFFSET_BASE_ADDR + (j * FLOAT_BYTE_COUNT * CAL_RESOLUTION) + (i * FLOAT_BYTE_COUNT), o);
//      Serial.print(F("Channel ")); Serial.print(j); Serial.print(F(" range ")); Serial.print(i); Serial.print(F(": slope = ")); Serial.print(s);
//      Serial.print(F(", offset = ")); Serial.println(o); 
//    }
//  }
//}

void setCurrent(float current, unsigned char channel)
{
  int pwm;
  int i;
  int range;
  float ptX;
  float slope;
  float offset;
 
  if(current < 0)
  {
    Serial.println(F("Current specified out of range! Setting to min value (0A)"));
    current = 0;
  }
  else if(current > CURRENT_MAX)
  {
    Serial.print(F("Current specified out of range! Setting to max value (")); Serial.print(CURRENT_MAX); Serial.println(F(")"));
    current = CURRENT_MAX;
  }

  currentChannelList[channel].currentSetpoint = current;

//  for(i=0; i<CAL_RESOLUTION; i++)
//  {
//    EEPROM.get(EEPROM_PTSX_BASE_ADDR + (channel * FLOAT_BYTE_COUNT * (CAL_RESOLUTION+1)) + ((i+1) * FLOAT_BYTE_COUNT), ptX);
//    if(current <= ptX || i == (CAL_RESOLUTION - 1))
//    {
//      range = i;
//      break;
//    }
//  }
//  EEPROM.get(EEPROM_SLOPE_BASE_ADDR + (channel * FLOAT_BYTE_COUNT * CAL_RESOLUTION) + (range * FLOAT_BYTE_COUNT), slope);
//  EEPROM.get(EEPROM_OFFSET_BASE_ADDR + (channel * FLOAT_BYTE_COUNT * CAL_RESOLUTION) + (range * FLOAT_BYTE_COUNT), offset);
//  pwm = round((slope * current) + offset);
//  if(pwm < 0)
//  {
//    pwm = 0;
//  }
//  if(pwm > 255)
//  {
//    pwm = 255;
//  }
//  analogWrite(currentControlPin[channel], pwm);
  Serial.print(F("Channel ")); Serial.print(channel); Serial.print(F(" set to ")); Serial.print(current); Serial.print(F("A, ")); Serial.println(pwm);
}
