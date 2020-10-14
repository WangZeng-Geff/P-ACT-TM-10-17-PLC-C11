#ifndef _COMFUNC_H_
#define _COMFUNC_H_

//#include "estype.h"
#include <stddef.h>
#include <stdint.h>

#define SIZE_1K 1024L
#define SIZE_1M (SIZE_1K*SIZE_1K)

#define set_bit(x, bit) ((x) |= 1 << (bit))
#define clr_bit(x, bit) ((x) &= ~(1 << (bit)))
#define tst_bit(x, bit) ((x) & (1 << (bit)))
#define get_bits(val,x1,x2) (((val)>>(x1))&((1<<((x2)-(x1)+1))-1))

#define min(a, b) ((a)<(b) ? (a):(b))
#define max(a, b) ((a)>(b) ? (a):(b))

#ifndef time_after
#define time_after(a,b)		((long)(b) - (long)(a) < 0)
#endif

#ifndef time_diff
#define time_diff(a,b)		(unsigned long)((long)(a) - (long)(b))
#endif

#ifndef time_after_eq
#define time_after_eq(a,b)	((long)(a) - (long)(b) >= 0)
#endif

#define array_size(array) (sizeof(array)/sizeof(*array))

#define FIELD_SIZEOF(t, f) (sizeof(((t*)0)->f))
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

/*
 * This looks more complex than it should be. But we need to
 * get the type for the ~ right in round_down (it needs to be
 * as wide as the result!), and we want to evaluate the macro
 * arguments just once each.
 */
#define __round_mask(x, y) ((__typeof__(x))((y)-1))
#define round_up(x, y) ((((x)-1) | __round_mask(x, y))+1)
#define round_down(x, y) ((x) & ~__round_mask(x, y))

#define _swab16(x) ((uint16_t)(				\
    (((uint16_t)(x) & (uint16_t)0x00ffU) << 8) |			\
    (((uint16_t)(x) & (uint16_t)0xff00U) >> 8)))

#define _swab32(x) ((uint32_t)(				\
    (((uint32_t)(x) & (uint32_t)0x000000ffUL) << 24) |		\
    (((uint32_t)(x) & (uint32_t)0x0000ff00UL) <<  8) |		\
    (((uint32_t)(x) & (uint32_t)0x00ff0000UL) >>  8) |		\
    (((uint32_t)(x) & (uint32_t)0xff000000UL) >> 24)))

#ifdef BIG_ENDIAN
#define cpu_to_le32(x) ((uint32_t)_swab32((x)))
#define le32_to_cpu(x) ((uint32_t)_swab32((uint32_t)(x)))
#define cpu_to_le16(x) ((uint16_t)_swab16((x)))
#define le16_to_cpu(x) ((uint16_t)_swab16((uint16_t)(x)))
#define cpu_to_be32(x) ((uint32_t)(x))
#define be32_to_cpu(x) ((uint32_t)(x))
#define cpu_to_be16(x) ((uint16_t)(x))
#define be16_to_cpu(x) ((uint16_t)(x))
#else
#define cpu_to_le32(x) ((uint32_t)(x))
#define le32_to_cpu(x) ((uint32_t)(x))
#define cpu_to_le16(x) ((uint16_t)(x))
#define le16_to_cpu(x) ((uint16_t)(x))
#define cpu_to_be32(x) ((uint32_t)_swab32((x)))
#define be32_to_cpu(x) ((uint32_t)_swab32((uint32_t)(x)))
#define cpu_to_be16(x) ((uint16_t)_swab16((x)))
#define be16_to_cpu(x) ((uint16_t)_swab16((uint16_t)(x)))
#endif

#define bcd2bin(val) (((val) & 0x0f) + ((val) >> 4) * 10)
#define bin2bcd(val) ((((val) / 10) << 4) + (val) % 10)

#define swap(a, b) do {a ^= b; b ^= a; a ^= b;} while (0)

uint8_t checksum(const void *data, int len);

/* byte order */
uint32_t get_le_val(const uint8_t * p, int bytes);
uint32_t get_be_val(const uint8_t * p, int bytes);
void put_le_val(uint32_t val, uint8_t * p, int bytes);
void put_be_val(uint32_t val, uint8_t * p, int bytes);

int split(char *str, char **arr, const char *del);
int is_all_xx(const void *s1, uint8_t val, int n);

const char *arr2str(const void *arr, int len, void *dest, int maxlen);
int str2arr(const char *str, void *out, int maxlen);
char *i2str(uint8_t val, char *destStr);
void print_debug_array(const char *lvl, const void *arr, int len);

void reverse(void *buff, size_t len);

uint16_t calc_crc16(uint16_t crc, const void *buf, int size);
uint16_t add_crc16(uint8_t *buffer, uint8_t data_length);
uint8_t XorCheack(uint8_t *buffer, uint8_t data_length);
uint8_t get_1byte_bit1_number(uint8_t data, uint8_t pos);

uint8_t memcmp_my(const void *s1, const void *s2, uint8_t n);
void mymemcpy(void *dst,void *src,uint8_t len);
void memset_my(void *s1, uint8_t value, uint8_t n);

#endif
