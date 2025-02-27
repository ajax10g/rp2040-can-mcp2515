#pragma once

#define BOARD_DEVICE_RHPORT_NUM 0
#define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_FULL_SPEED
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)

#define CFG_TUD_VENDOR 1

#define CFG_TUD_VENDOR_EPSIZE     32
#define CFG_TUD_VENDOR_EP_BUFSIZE 512
#define CFG_TUD_VENDOR_RX_BUFSIZE 512
#define CFG_TUD_VENDOR_TX_BUFSIZE 512

#define CFG_TUD_ENDPOINT0_SIZE 64

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

#define CFG_TUSB_OS OPT_OS_PICO /*clpham: was OPT_OS_NONE*/
