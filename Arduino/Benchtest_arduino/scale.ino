void initScale(void)
{
  float ratio;
  long offset;
  EEPROM.get(EEPROM_SCALE_RATIO_BASE_ADDR, ratio);
  EEPROM.get(EEPROM_SCALE_OFFSET_BASE_ADDR, offset);
  Serial.print(F("weight ratio: "));
  Serial.println(ratio);
  Serial.print(F("weight offset: "));
  Serial.println(offset);
  scale.set_scale(ratio);
  scale.set_offset(offset);
}

void resetScale(void)
{
  scale.tare();  
  Serial.println(F("Scale was reset"));
}

float readScale(byte averages)
{
  return scale.get_units(averages);
}

/* Calibration procedure taken from HX711 library readme */
void calScale(void)
{
  char bufBytes[3];
  int calWeight;
  float getUnitsValue;
  union
  {
    float value;
    byte bytes[4];
  }scaleRatio;
  union
  {
    long value;
    byte bytes[4];
  }scaleOffset;
  
  Serial.setTimeout(LOADCELL_CAL_TIMEOUT_MS);

  while(Serial.available()) //flush the input buffer
  {
    Serial.read();
  }
  
  Serial.print(F("\n\nLoad cell calibration\n\nRemove any weight on the scale, then press enter\n"));
  Serial.readBytes(bufBytes, 1);

  scale.set_scale();
  scale.tare();
  Serial.println(F("\n\nFirst place the calibration weight on the scale, then enter it's weight in grams"));
  calWeight = Serial.parseInt();
  if(calWeight <= 0 || calWeight > LOADCELL_MAX_WEIGHT)
  {
    Serial.print(F("Weight value \""));
    Serial.print(calWeight);
    Serial.print(F("\" is out of range! (1 - "));
    Serial.print(LOADCELL_MAX_WEIGHT);
    Serial.print(F(")\nExiting...\n"));
    return;
  }
  getUnitsValue = scale.get_units(10);
  scaleRatio.value = getUnitsValue / calWeight;
  Serial.print(F("Param: "));
  Serial.println(scaleRatio.value);
  for(byte i=0; i<4; i++)
  {
    EEPROM.put((EEPROM_SCALE_RATIO_BASE_ADDR + i), scaleRatio.bytes[i]);
  }
  scale.set_scale(scaleRatio.value);
  
  scaleOffset.value = scale.get_offset();
  Serial.print(F("Off: "));
  Serial.println(scaleOffset.value);
  for(byte i=0; i<4; i++)
  {
    EEPROM.put((EEPROM_SCALE_OFFSET_BASE_ADDR + i), scaleOffset.bytes[i]);
  }
  while(Serial.available()) //flush the input buffer
  {
    Serial.read();
  }
}

