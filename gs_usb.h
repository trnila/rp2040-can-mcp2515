#pragma once
#include <stdint.h>

enum gs_usb_breq {
  GS_USB_BREQ_HOST_FORMAT = 0,
  GS_USB_BREQ_BITTIMING,
  GS_USB_BREQ_MODE,
  GS_USB_BREQ_BERR,
  GS_USB_BREQ_BT_CONST,
  GS_USB_BREQ_DEVICE_CONFIG,
  GS_USB_BREQ_TIMESTAMP,
  GS_USB_BREQ_IDENTIFY,
  GS_USB_BREQ_GET_USER_ID,
  GS_USB_BREQ_QUIRK_CANTACT_PRO_DATA_BITTIMING = GS_USB_BREQ_GET_USER_ID,
  GS_USB_BREQ_SET_USER_ID,
  GS_USB_BREQ_DATA_BITTIMING,
  GS_USB_BREQ_BT_CONST_EXT,
  GS_USB_BREQ_SET_TERMINATION,
  GS_USB_BREQ_GET_TERMINATION,
  GS_USB_BREQ_GET_STATE,
};

struct gs_device_config {
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t icount;
  uint32_t sw_version;
  uint32_t hw_version;
} __attribute__((packed));

struct gs_device_bt_const {
  uint32_t feature;
  uint32_t fclk_can;
  uint32_t tseg1_min;
  uint32_t tseg1_max;
  uint32_t tseg2_min;
  uint32_t tseg2_max;
  uint32_t sjw_max;
  uint32_t brp_min;
  uint32_t brp_max;
  uint32_t brp_inc;
} __attribute__((packed));

struct gs_host_frame {
  uint32_t echo_id;
  uint32_t can_id;

  uint8_t can_dlc;
  uint8_t channel;
  uint8_t flags;
  uint8_t reserved;

  uint8_t data[8];
} __attribute__((packed));

struct gs_device_bittiming {
  uint32_t prop_seg;
  uint32_t phase_seg1;
  uint32_t phase_seg2;
  uint32_t sjw;
  uint32_t brp;
} __attribute__((packed));

struct gs_device_mode {
  uint32_t mode;
  uint32_t flags;
} __attribute__((packed));
