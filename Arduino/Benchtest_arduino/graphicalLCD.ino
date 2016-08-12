void printLcd(byte row, byte column, const char *str)
{
  unsigned int len, i;
  
  if( row > 7 || column > 20 || str == NULL)
  {
    Serial.println(F("printLcd error: Invalid parameter"));
    return;
  }
  Wire.beginTransmission(LCD_I2C_ADDR);
  Wire.write(0x25);
  Wire.write(row);
  Wire.write(column);
  Wire.endTransmission();
  
  Wire.beginTransmission(LCD_I2C_ADDR);
  Wire.write(0x20);
  len = strlen(str);
  for(i=0; i<len; i++)
  {
    Wire.write(str[i]);
  }
  Wire.endTransmission();
}

void clearLcd(void)
{  
  Wire.beginTransmission(LCD_I2C_ADDR);
  Wire.write(0x05);
  Wire.endTransmission();
}

void printFloatLcd(byte row, byte column, const char *str, float number)
{
  unsigned int len, i;
  char numberStr[LCD_FLOAT_BUFFER_SIZE];
  
  if( row > 7 || column > 20)
  {
    Serial.println(F("printLcd error: Invalid parameter"));
    return;
  }

  //set cursor to position
  Wire.beginTransmission(LCD_I2C_ADDR);
  Wire.write(0x25);
  Wire.write(row);
  Wire.write(column);
  Wire.endTransmission();

  //convert passed number to a string
  dtostrf(number, 1, 2, numberStr);
  if(strlen(numberStr) >=  LCD_FLOAT_BUFFER_SIZE)
  {
    Serial.println(F("printLcd error: buffer too small for float string"));
  }

  Wire.beginTransmission(LCD_I2C_ADDR);
  Wire.write(0x20);
  if(str != NULL)   //print header string if one was passed
  {
    len = strlen(str);
    for(i=0; i<len; i++)
    {
      Wire.write(str[i]);
    }
  }
  len = strlen(numberStr);
  for(i=0; i<len; i++)
  {
    Wire.write(numberStr[i]);
  }
  Wire.endTransmission();
}

