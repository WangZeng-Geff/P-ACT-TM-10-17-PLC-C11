#ifndef _CRC32_H_
#define _CRC32_H_

#include <stdint.h>

uint32_t get_crc32_s(uint32_t crc, const uint8_t *ptr, uint32_t data_len);

#endif
