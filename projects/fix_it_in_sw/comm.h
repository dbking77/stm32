#ifndef COMM_H_INCLUDE_GUARD
#define COMM_H_INCLUDE_GUARD

#include <stdint.h>

// serial streaming functions
void send_string(const char* data);
void send_decimal(uint32_t value, int multiplier);
const char* hex2str(uint32_t value);
const char* dec2str(int32_t value);


#endif //COMM_H_INCLUDE_GUARD
