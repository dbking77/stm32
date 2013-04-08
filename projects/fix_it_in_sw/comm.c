#include "comm.h"

const char* hex2str(uint32_t value)
{
  static char buf[9];
  for (int i=0; i<8; ++i)
  {
    char x = (value>>28) & 0xf;
    buf[i] = (x>=10) ? ('A'+x-10) : ('0'+x);
    value <<= 4;
  }
  buf[8] = '\0';
  return buf;
}

const char* dec2str(int32_t value)
{
  uint8_t neg = 0;
  if (value < 0)
  {
    neg = 1;
    value = -value;
  }
  static char buf[12];
  char* p=&buf[sizeof(buf)-1];
  *p = '\0';
  do 
  {
    unsigned rem = value%10;
    value /= 10;
    *(--p) = '0' + rem;
  } while (value > 0);
  if (neg)
  {
    *(--p) = '-';
  }
  return p;
}


const char* zdec2str(uint32_t value, int zero_fill)
{  
  if (zero_fill > 10) 
    zero_fill = 10;

  static char buf[11];
  char* p=&buf[sizeof(buf)-1];
  *p = '\0';
  do 
  {
    unsigned rem = value%10;
    value /= 10;
    *(--p) = '0' + rem;
    --zero_fill;
  } while (value > 0);
  
  while(zero_fill>0)
  {
    *(--p) = '0';
    --zero_fill;
  }

  return p;
}


