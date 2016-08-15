/*/////////////////////////////////////////////////////////
checkOverCurrent
/////////////////////////////////////////////////////////*/
void checkOverCurrent(void)
{
  bool overCurrentDetected = 0;
  for(int i=0; i<CURRENT_CHANNELS_COUNT; i++)
  {
    if(sensorsList[i].temp.value > TEMP_MAX)
    {
      overCurrentDetected = 1;
      break;
    }
  }
  if(overCurrentDetected)
  {
    disableAllCurrent();
    overCurrent = 1;
    Serial.println(F("Over temperature detected!!"));
  }
  else
  {
    overCurrent = 0;
  }
}

void disableAllCurrent(void)
{
  for(int i=0; i<CURRENT_CHANNELS_COUNT; i++)
  {
    analogWrite(currentChannelList[i].pwmPin, 0);
    currentChannelList[i].currentSetpoint = 0;
    currentChannelList[i].pwm = 0;
  }
}

/*/////////////////////////////////////////////////////////
readCurrents
/////////////////////////////////////////////////////////*/
void readCurrents(void)
{
  int adc;
  int i;
  int totalCurrent = 0;
  bool overCurrentDetected = 0;
  for(i=0; i<CURRENT_CHANNELS_COUNT; i++)
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

    currentChannelList[i].average = currentChannelList[i].total / CURRENT_AVG_NUM;
    currentChannelList[i].current.value = ( ( (( (float)currentChannelList[i].average / 1024.0) * 5000) - 2500) / CURRENT_SENSOR_MV_PER_AMP ) * CURRENT_VAR_RATIO;
    totalCurrent += currentChannelList[i].current.value;
    if(currentChannelList[i].current.value > CURRENT_MAX_ABSOLUTE || totalCurrent > CURRENT_MAX_TOTAL)
    {
      overCurrentDetected = 1;
      break;
    }
  }

  if(overCurrentDetected)
  {
    disableAllCurrent();
    overCurrent = 1;

    Serial.print(F("CRITICAL: "));
    if(totalCurrent > CURRENT_MAX_TOTAL)
    {
      Serial.println(F("Total current above limit!!"));
    }
    else
    {
      Serial.print(F("Over current detected on channel "));
      Serial.print(i);
      Serial.println("!!");
    }
  }
  else
  {
    overCurrent = 0;
  }
}

/*/////////////////////////////////////////////////////////
updateCurrents
/////////////////////////////////////////////////////////*/
void updateCurrents(void)
{
  if(!overTemp && !overCurrent)
  {
    int error;
    for(int i=0; i<CURRENT_CHANNELS_COUNT; i++)
    {
      error = currentChannelList[i].currentSetpoint - currentChannelList[i].current.value;
      currentChannelList[i].pwm += round((float)(error * CURRENT_KP_GAIN * CURRENT_PWM_FACTOR) / CURRENT_VAR_RATIO);
      if(currentChannelList[i].pwm < 0)
      {
        currentChannelList[i].pwm = 0;
      }
      if(currentChannelList[i].pwm > PWM_MAX_VALUE)
      {
        currentChannelList[i].pwm = PWM_MAX_VALUE;
      }
      analogWrite(currentChannelList[i].pwmPin, currentChannelList[i].pwm);
    }
  }
}

/*/////////////////////////////////////////////////////////
setCurrent
/////////////////////////////////////////////////////////*/
void setCurrent(int current, unsigned char channel)
{ 
  if(current < 0)
  {
    Serial.println(F("Current specified out of range! Setting to min value (0A)"));
    current = 0;
  }
  else if(current > CURRENT_MAX_SETPOINT)
  {
    Serial.print(F("Current specified out of range! Setting to max value (")); Serial.print(CURRENT_MAX_SETPOINT); Serial.println(F(")"));
    current = CURRENT_MAX_SETPOINT;
  }

  if(channel == 255)
  {
    for(int i; i<CURRENT_CHANNELS_COUNT; i++)
    {
      currentChannelList[i].currentSetpoint = current;
      Serial.print(F("Channel ")); Serial.print(i); Serial.print(F(" set to ")); Serial.print((float)current / CURRENT_VAR_RATIO); Serial.print(F("A\n"));
    }
  }
  else if(channel > (CURRENT_CHANNELS_COUNT-1))
  {
    Serial.println(F("Channel specified out of range!"));
    return;
  }
  else
  {
    currentChannelList[channel].currentSetpoint = current;
    Serial.print(F("Channel ")); Serial.print(channel); Serial.print(F(" set to ")); Serial.print((float)current / CURRENT_VAR_RATIO); Serial.print(F("A\n"));
  }
}
