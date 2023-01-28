#include "common_utils.h"



void memcpy(void *dest, void *src, uint32_t n) 
{ 
    char *csrc  = (char *)src; 
    char *cdest = (char *)dest; 
  
    for (uint32_t i = 0; i < n; i++) 
    {
        cdest[i] = csrc[i]; 
    }
} 


void memcpy(const void *dest, const void *src, uint32_t n) 
{ 
    char *csrc  = (char *)src; 
    char *cdest = (char *)dest; 
  
    for (uint32_t i = 0; i < n; i++) 
    {
        cdest[i] = csrc[i]; 
    }
} 


void *memset(void *buffer, int c, int len)
{
  unsigned char *b_tmp = (unsigned char*)buffer;

  for (uint32_t i = 0; i < len; i++)
  {
    b_tmp[i] = c;
  }
  
  return buffer;
}


/*
void *memset(const void *buffer, int c, int len)
{
  unsigned char *b_tmp = (unsigned char*)buffer;

  for (uint32_t i = 0; i < len; i++)
  {
    b_tmp[i] = c;
  }
  
  return (void*)buffer;
}
*/