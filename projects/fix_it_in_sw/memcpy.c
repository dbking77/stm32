#include <string.h>
#include <stdint.h>

// Replacement for the C-library memcpy and memset functions

void *memset(void *str, int c, size_t n)
{
  while (n)
  {
    ((char*)str)[--n] = c;
  }
  return str;
}

void *memcpy(void *str1, const void *str2, size_t n)
{
  while (n)
  {
    --n;
    ((char*)str1)[n] = ((char*)str2)[n];
  }
  return str1;
}

