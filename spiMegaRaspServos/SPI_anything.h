#include <Arduino.h>

template <typename T> unsigned int SPI_writeAnything (const T& value)
  {
    const byte * p = (const byte*) &value;
    unsigned int i;
    for (i = 0; i < sizeof value; i++)
          SPI.transfer(*p++);
    return i;
  }  // end of SPI_writeAnything

bool SPI_readAnything(char* value, int tam)
{
    if(SPI.transfer(0) != 127) //127 indica inicio de um bloco
      return false;
    char *p = value;
    unsigned int i;
    for (i = 0; i < tam; i++){
    	//delayMicroseconds(20);
    	p[i] = SPI.transfer(0);
    }
    char var = SPI.transfer(0);
    if(var != -127) //127 indica fim de um bloco
      return false;
    else
      return true;
   	//delayMicroseconds(20);
}  // end of SPI_readAnything
  
  
template <typename T> unsigned int SPI_readAnything_ISR(T& value)
  {
    byte * p = (byte*) &value;
    unsigned int i;
    *p++ = SPDR;  // get first byte
    for (i = 1; i < sizeof value; i++)
          *p++ = SPI.transfer (0);
    return i;
  }  // end of SPI_readAnything_ISR
