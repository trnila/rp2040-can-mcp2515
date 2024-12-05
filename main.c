#include "gs_usb.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include <assert.h>

#if defined(BOARD_PICO_MCP2515_MODULE_20MHZ) ||                                \
    defined(BOARD_PICO_MCP2515_MODULE_8MHZ)
#if defined(BOARD_PICO_MCP2515_MODULE_20MHZ)
const static uint32_t MCP2515_OSC_FREQ = 20000000;
#else
const static uint32_t MCP2515_OSC_FREQ = 8000000;
#endif
const static uint MCP2515_IRQ_GPIO = 20;
const static spi_inst_t *MCP2515_SPI = spi_default;
const static uint MCP2515_SPI_CSN_GPIO = PICO_DEFAULT_SPI_CSN_PIN;
const static uint MCP2515_SPI_RX_GPIO = PICO_DEFAULT_SPI_RX_PIN;
const static uint MCP2515_SPI_SCK_GPIO = PICO_DEFAULT_SPI_SCK_PIN;
const static uint MCP2515_SPI_TX_GPIO = PICO_DEFAULT_SPI_TX_PIN;
#elif defined(BOARD_ADAFRUIT_CAN_FEATHER)
const static uint32_t MCP2515_OSC_FREQ = 16000000;
const static uint MCP2515_IRQ_GPIO = 22;
const static spi_inst_t *MCP2515_SPI = spi1;
const static uint MCP2515_SPI_CSN_GPIO = 19;
const static uint MCP2515_SPI_RX_GPIO = 8;
const static uint MCP2515_SPI_SCK_GPIO = 14;
const static uint MCP2515_SPI_TX_GPIO = 15;
#else
#error Invalid board selected
#endif

struct usb_control_out_t {
  uint8_t bRequest;
  void *buffer;
  uint16_t wLength;
};

enum mcp2515_mode_t {
  MCP2515_MODE_NORMAL,
  MCP2515_MODE_SLEEP,
  MCP2515_MODE_LOOPBACK,
  MCP2515_MODE_LISTENONLY,
  MCP2515_MODE_CONFIG,
};

#define MCP2515_RX_BUFS 2
#define MCP2515_TX_BUFS 3

const static uint16_t MCP2515_CMD_RESET = 0b11000000;
const static uint16_t MCP2515_CMD_WRITE = 0b00000010;
const static uint16_t MCP2515_CMD_READ = 0b00000011;
const static uint16_t MCP2515_CMD_BIT_MODIFY = 0b00000101;
const static uint16_t MCP2515_CMD_READ_STATUS = 0b10100000;
inline static uint16_t MCP2515_CMD_READ_RX_BUFFER(size_t n) {
  return 0b10010000 | ((n & 1U) << 2U);
}

const static uint16_t MCP2515_CANSTAT = 0x0E;
const static uint16_t MCP2515_CANCTRL = 0x0F;
const static uint16_t MCP2515_CNF3 = 0x28;
const static uint16_t MCP2515_CNF2 = 0x29;
const static uint16_t MCP2515_CNF1 = 0x2A;
const static uint16_t MCP2515_CANINTE = 0x2B;
const static uint16_t MCP2515_CANINTF = 0x2C;
const static uint16_t MCP2515_RXB0CTRL = 0x60;

// Rollover enable bit (use RX1 if RX0 is full)
const static uint8_t MCP2515_RXB0CTRL_BUKT = 1 << 2;

const uint32_t CAN_STDMSGID_MAX = 0x7FF;
const uint8_t SIDL_EXTENDED_MSGID = 1U << 3U;

volatile struct gs_host_frame tx[MCP2515_TX_BUFS];

static uint32_t byte_order = 0;
static struct gs_device_bittiming device_bittiming;
static struct gs_device_mode device_mode;
struct usb_control_out_t usb_control_out[] = {
    {GS_USB_BREQ_HOST_FORMAT, &byte_order, sizeof(byte_order)},
    {GS_USB_BREQ_BITTIMING, &device_bittiming, sizeof(device_bittiming)},
    {GS_USB_BREQ_MODE, &device_mode, sizeof(device_mode)},
};

void spi_transmit(uint8_t *tx, uint8_t *rx, size_t len) {
  asm volatile("nop \n nop \n nop");
  gpio_put(MCP2515_SPI_CSN_GPIO, 0);
  asm volatile("nop \n nop \n nop");
  if (rx) {
    spi_write_read_blocking(MCP2515_SPI, tx, rx, len);
  } else {
    spi_write_blocking(MCP2515_SPI, tx, len);
  }
  asm volatile("nop \n nop \n nop");
  gpio_put(MCP2515_SPI_CSN_GPIO, 1);
  asm volatile("nop \n nop \n nop");
}

void mcp2515_reset() {
  uint8_t tx = MCP2515_CMD_RESET;
  spi_transmit(&tx, NULL, 1);
}

void mcp2515_write(uint8_t addr, uint8_t value) {
  uint8_t tx[] = {MCP2515_CMD_WRITE, addr, value};
  spi_transmit(tx, NULL, sizeof(tx));
}

uint8_t mcp2515_read(uint8_t addr) {
  uint8_t tx[] = {
      MCP2515_CMD_READ, addr, 0 /* dummy for response */
  };
  uint8_t rx[sizeof(tx)];
  spi_transmit(tx, rx, sizeof(tx));
  return rx[2];
}

uint8_t mcp2515_read_status() {
  uint8_t tx[] = {
      MCP2515_CMD_READ_STATUS, 0 /* dummy for response */
  };
  uint8_t rx[sizeof(tx)];
  spi_transmit(tx, rx, sizeof(tx));
  return rx[1];
}

void mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t val) {
  uint8_t tx[] = {MCP2515_CMD_BIT_MODIFY, reg, mask, val};
  spi_transmit(tx, NULL, sizeof(tx));
}

uint8_t mcp2515_canstat_to_irqs(uint8_t canstat) {
  return (canstat >> 1) & 0b111;
}

ssize_t mcp2515_get_free_tx() {
  for (size_t i = 0; i < sizeof(tx) / sizeof(*tx); i++) {
    if (tx[i].echo_id == -1) {
      return i;
    }
  }

  return -1;
}

void mcp2515_set_mode(enum mcp2515_mode_t mode) {
  mcp2515_write(MCP2515_CANCTRL, (mode << 5U));

  // wait until mode is switched
  while ((mcp2515_read(MCP2515_CANSTAT) >> 5) != mode) {
  }
}

void handle_rx(uint8_t rxn) {
  // read RXBnSIDH ... RXBnD0 ... RXBnD7
  // and automatically clear pending RX
  uint8_t tx[14] = {
      MCP2515_CMD_READ_RX_BUFFER(rxn),
  };
  uint8_t rx[sizeof(tx)] = {0};
  spi_transmit(tx, rx, sizeof(tx));

  struct gs_host_frame rxf = {0};
  rxf.echo_id = -1;
  rxf.can_dlc = rx[5] & 0b1111;
  rxf.flags = 0;
  rxf.channel = 0;

  if (rx[2] & SIDL_EXTENDED_MSGID) {
    rxf.can_id = (rx[1] << 21U) | ((rx[2] >> 5U) << 18U) |
                 ((rx[2] & 0b11) << 16U) | (rx[3] << 8U) | rx[4] |
                 (1 << 31U); // extended frame, see linux/can.h
  } else {
    rxf.can_id = (rx[1] << 3U) | (rx[2] >> 5U);
  }
  memcpy(rxf.data, &rx[6], rxf.can_dlc);

  tud_vendor_write(&rxf, sizeof(rxf));
  tud_vendor_write_flush();
}

int main() {
  tusb_init();

  for (size_t i = 0; i < sizeof(tx) / sizeof(*tx); i++) {
    tx[i].echo_id = -1;
  }

  spi_init(MCP2515_SPI, 1000 * 1000);
  gpio_set_function(MCP2515_SPI_RX_GPIO, GPIO_FUNC_SPI);
  gpio_set_function(MCP2515_SPI_SCK_GPIO, GPIO_FUNC_SPI);
  gpio_set_function(MCP2515_SPI_TX_GPIO, GPIO_FUNC_SPI);
  gpio_init(MCP2515_SPI_CSN_GPIO);
  gpio_set_dir(MCP2515_SPI_CSN_GPIO, GPIO_OUT);

  gpio_init(MCP2515_IRQ_GPIO);
  gpio_pull_up(MCP2515_IRQ_GPIO);

  mcp2515_reset();
  sleep_ms(250);

  // enable interrupts on rxs and txs
  mcp2515_write(MCP2515_CANINTE, 0b11111);

  // use RX1 if RX0 is full
  mcp2515_write(MCP2515_RXB0CTRL, MCP2515_RXB0CTRL_BUKT);

  // transition from config to normal mode
  // mcp2515_set_mode(MCP2515_MODE_SLEEP);

  for (;;) {
    while (gpio_get(MCP2515_IRQ_GPIO) == 0) {
      uint8_t status = mcp2515_read_status();
      for (size_t rxn = 0; rxn < MCP2515_RX_BUFS; rxn++) {
        if (status & (1 << rxn)) {
          handle_rx(rxn);
        }
      }

      for (size_t txn = 0; txn < MCP2515_TX_BUFS; txn++) {
        if (status & (1U << (3 + txn * 2))) {
          assert(tx[txn].echo_id != -1);

          tud_vendor_write(&tx[txn], sizeof(tx[txn]));
          tud_vendor_write_flush();
          tx[txn].echo_id = -1;

          // ack irq
          mcp2515_bit_modify(MCP2515_CANINTF, 1 << (2 + txn), 0);
        }
      }
    }

    tud_task();

    if (tud_vendor_available()) {
      ssize_t txn = mcp2515_get_free_tx();
      if (txn >= 0) {
        struct gs_host_frame *frame = &tx[txn];
        uint32_t count = tud_vendor_read(frame, sizeof(*frame));
        if (count != sizeof(*frame)) {
          for (;;)
            ;
        }

        size_t hdr_size = 6;
        uint8_t tx[hdr_size + sizeof(frame->data)];
        memset(tx, 0, sizeof(tx));
        tx[0] = 0b01000000 | (txn == 0 ? 0 : (1 << txn));
        if (frame->can_id <= CAN_STDMSGID_MAX) {
          tx[1] = frame->can_id >> 3U;           // SIDH
          tx[2] = (frame->can_id & 0b111) << 5U; // SIDL
        } else {
          // msgid 27..20
          tx[1] = frame->can_id >> 21U;
          // msgid 19..18, msgid 16,17, enable extended frame
          tx[2] = (((frame->can_id >> 18U) & 0b111U) << 5U) |
                  ((frame->can_id >> 16U) & 0b11U) | (1U << 3U);
          // msgid 15..8
          tx[3] = frame->can_id >> 8U;
          // msgid 7..0
          tx[4] = frame->can_id;
        }
        tx[5] = frame->can_dlc; // DLC
        memcpy(&tx[6], frame->data, frame->can_dlc);

        spi_transmit(tx, NULL, hdr_size + frame->can_dlc);

        // request to send
        tx[0] = 0b10000000U | (1U << txn);
        spi_transmit(tx, NULL, 1);
      }
    }
  }
}

bool usb_handle_control_out(uint8_t req) {
  if (req == GS_USB_BREQ_HOST_FORMAT) {
    return byte_order == 0xbeef;
  } else if (req == GS_USB_BREQ_MODE) {
    mcp2515_set_mode(device_mode.mode ? MCP2515_MODE_NORMAL
                                      : MCP2515_MODE_CONFIG);
    return true;
  } else if (req == GS_USB_BREQ_BITTIMING) {
    mcp2515_write(MCP2515_CNF1,
                  (((device_bittiming.sjw - 1) & 0b11U) << 6U) |
                      ((device_bittiming.brp / 2 - 1) & 0b111111U));
    mcp2515_write(MCP2515_CNF2,
                  ((device_bittiming.prop_seg - 1) & 0b111U) |
                      (((device_bittiming.phase_seg1 - 1) & 0b111U) << 3) |
                      (1U << 7U));
    mcp2515_write(MCP2515_CNF3, (((device_bittiming.phase_seg2 - 1) & 0b111U)));
    return true;
  }
  return false;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                const tusb_control_request_t *request) {
  if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR ||
      request->wIndex != 0) {
    return false;
  }

  if (request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
    for (size_t i = 0; i < sizeof(usb_control_out) / sizeof(*usb_control_out);
         i++) {
      if (usb_control_out[i].bRequest == request->bRequest) {
        if (stage == CONTROL_STAGE_SETUP) {
          if (usb_control_out[i].wLength == request->wLength) {
            return tud_control_xfer(rhport, request, usb_control_out[i].buffer,
                                    usb_control_out[i].wLength);
          }
        } else if (stage == CONTROL_STAGE_DATA) {
          return usb_handle_control_out(request->bRequest);
        } else if (stage == CONTROL_STAGE_ACK) {
          return true;
        }
      }
    }
  } else if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
    if (request->bRequest == GS_USB_BREQ_DEVICE_CONFIG) {
      if (stage == CONTROL_STAGE_SETUP) {
        struct gs_device_config res;
        res.icount = 0;
        res.sw_version = 18;
        res.hw_version = 11;
        return tud_control_xfer(rhport, request, (void *)&res, sizeof(res));
      } else {
        return true;
      }
    } else if (request->bRequest == GS_USB_BREQ_BT_CONST) {
      if (stage == CONTROL_STAGE_SETUP) {
        struct gs_device_bt_const res = {
            0, MCP2515_OSC_FREQ,
            // tseg1 1..8 (3 bits)
            1, 8,
            // tseg2 1..8 (3 bits)
            1, 8,
            // sjw 0..3 (2 bits)
            4,
            // brp 2..64 with increment of 2 (Tq = 2 * (BRP + 1) / Fosc)
            2, 64, 2};
        return tud_control_xfer(rhport, request, (void *)&res, sizeof(res));
      } else {
        return true;
      }
    }
  }

  return false;
}
