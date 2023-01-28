#ifndef _COMMON_UTILS_H_
#define _COMMON_UTILS_H_

#include <stdint.h>


void memcpy(void *dest, void *src, uint32_t n);
void memcpy(const void *dest, const void *src, uint32_t n);

void *memset(void *buffer, int c, int len);
//void *memset(const void *buffer, int c, int len);

#endif 
