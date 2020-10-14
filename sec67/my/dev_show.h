#ifndef _DEV_SHOW_H_
#define _DEV_SHOW_H_

#include "include.h"
#include "repeater.h"
#include "protocol.h"

#define TIME_LEN                 0x02
#define TYPE_LEN                 0x04

int search_frame_opt(const sdk_evt_rx_plc_t *info, struct SmartFrame *pframe);

#endif
