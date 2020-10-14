#ifndef _UPDATE_H_
#define _UPDATE_H_

#include <stdint.h>
#include <stddef.h>
#include "protocol.h"

#define UPDATE_FINISHED 0xFFFF

#pragma pack(1)
struct Update
{
    uint8_t seq[2];
    uint8_t ack;
    uint8_t crc[2];
    uint8_t len;
    uint8_t data[0];
};
#define UPDATE_HEAD sizeof(struct Update)
struct UpdateFile
{
    uint8_t size[4];
    uint8_t crc[2];
    uint8_t block_size;
    uint8_t type[8];
    uint8_t data[0];
};
#define UPDATE_FILE_HEAD sizeof(struct UpdateFile)
#pragma pack()
struct UpdateCache
{
    uint32_t file_size;
    uint32_t max_blk_cnt;
    uint32_t block_no;
    uint32_t block_size;
    uint16_t file_crc;
    uint32_t file_type;
    int waiting_for_reset;
    uint8_t ver[0x40];
};
extern struct UpdateCache update_cache;
void update_init(void);
int do_update(const uint8_t *in, int len);
#endif
