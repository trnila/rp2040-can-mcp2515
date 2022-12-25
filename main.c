#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "tusb.h"

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

const static uint16_t MCP2515_CMD_RESET = 0b11000000;
const static uint16_t MCP2515_CMD_WRITE = 0b00000010;
const static uint16_t MCP2515_CMD_READ  = 0b00000011;

const static uint16_t MCP2515_CANSTAT = 0x0E;
const static uint16_t MCP2515_CANCTRL = 0x0F;
const static uint16_t MCP2515_CNF3 = 0x28;
const static uint16_t MCP2515_CNF2 = 0x29;
const static uint16_t MCP2515_CNF1 = 0x2A;
const static uint16_t MCP2515_CANINTE = 0x2B;

const static uint MCP2515_IRQ_GPIO = 20;

volatile bool rx_pending = false;

void spi_transmit(uint8_t *tx, uint8_t* rx, size_t len) {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    if(rx) {
        spi_write_read_blocking(spi_default, tx, rx, len);
    } else {
        spi_write_blocking(spi_default, tx, len);
    }
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

void mcp2515_reset() {
    uint8_t tx = MCP2515_CMD_RESET;
    spi_transmit(&tx, NULL, 1);
}

void mcp2515_write(uint8_t addr, uint8_t value) {
    uint8_t tx[] = {
        MCP2515_CMD_WRITE,
        addr,
        value
    };
    spi_transmit(tx, NULL, sizeof(tx));
}

uint8_t mcp2515_read(uint8_t addr) {
    uint8_t tx[] = {
        MCP2515_CMD_READ,
        addr,
        0 /* dummy for response */
    };
    uint8_t rx[sizeof(tx)];
    spi_transmit(tx, rx, sizeof(tx));
    return rx[2];
}

void mcp2515_isr(uint gpio, uint32_t event_mask) {
    rx_pending = true;
}

int main() {
    tusb_init();

    stdio_init_all();

    spi_init(spi_default, 1000 * 1000);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);

    gpio_init(MCP2515_IRQ_GPIO);
    gpio_set_irq_enabled_with_callback(MCP2515_IRQ_GPIO, GPIO_IRQ_LEVEL_LOW, true, mcp2515_isr);

    mcp2515_reset();
    sleep_ms(250);

    mcp2515_write(MCP2515_CNF1, 0x01);
    mcp2515_write(MCP2515_CNF2, 0xb5);
    //mcp2515_write(0x28, 0x82);
    uint8_t tx[] = {0x05, MCP2515_CNF3, 0x07, 0x01};
    spi_transmit(tx, NULL, sizeof(tx));

    // enable interrupts on rxs
    mcp2515_write(MCP2515_CANINTE, 3);

    // transition from config to normal mode
    mcp2515_write(MCP2515_CANCTRL, 0x07 | (1 << 3));
      
    for(;;) {
        if(rx_pending) {
            rx_pending = false;
            struct gs_host_frame rxf;
            volatile uint8_t canstat = mcp2515_read(MCP2515_CANSTAT);
            uint8_t isr = (canstat >> 1) & 0b111;

            if(isr == 0b110) {
                uint8_t tx[14] = {
                    0b10010000
                };
                uint8_t rx[14] = {0};

                spi_transmit(tx, rx, sizeof(tx));

                printf("rx %d\n", rx[5] & 0b1111);

                rxf.echo_id = -1;
                rxf.can_dlc = rx[5] & 0b1111;
                rxf.flags = 0;
                rxf.channel = 0;
                rxf.can_id = (rx[1] << 3) | (rx[2] >> 5);
                memcpy(rxf.data, &rx[6], rxf.can_dlc);

                tud_vendor_write(&rxf, sizeof(rxf));
            }
        }
        
        tud_task();
        if ( tud_vendor_available() ) {
            uint8_t buf[64];
            uint32_t count = tud_vendor_read(buf, sizeof(buf));

            struct gs_host_frame *frame = buf;

            uint8_t tx[24];
            tx[0] = 0b01000000;
            tx[1] = 0xFF; //SIDH
            tx[2] = 0x00; //SIDL
            tx[3] = 0x00; //EID8
            tx[4] = 0x00; // IED0
            tx[5] = frame->can_dlc; // DLC
            memcpy(&tx[6], frame->data, frame->can_dlc);

            spi_transmit(tx, NULL, 6 + frame->can_dlc);

            tx[0] = 0b10000001;
            spi_transmit(tx, NULL, 1);

            tud_vendor_write(buf, count);
        }
    }
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
    // nothing to with DATA & ACK stage
    if (stage != CONTROL_STAGE_SETUP) return true;

    if(request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR) {
        if(request->bRequest == 0 || request->bRequest == 1 || request->bRequest == 2) {
            return tud_control_xfer(rhport, request, request, request->wLength);
        } else if(request->bRequest == 5) {
            struct gs_device_config res;
            res.icount = 0;
            res.sw_version = 18;
            res.hw_version = 11;
            return tud_control_xfer(rhport, request, (void*) &res, sizeof(res));
        } else if(request->bRequest == 4) {
            struct gs_device_bt_const res = {
                0,
                8000000, // can timing base clock
                1, // tseg1 min
                16, // tseg1 max
                1, // tseg2 min
                8, // tseg2 max
                4, // sjw max
                1, // brp min
                1024, //brp_max
                1, // brp increment;
            };
            return tud_control_xfer(rhport, request, (void*) &res, sizeof(res));
        }
    }

    for(;;);
    return false;
}
