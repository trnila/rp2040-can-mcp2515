#include "pico/stdlib.h"
#include "hardware/spi.h"


const uint LED_PIN = 25;

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

#define MCP2515_CMD_RESET 0b11000000
#define MCP2515_CMD_WRITE 0b00000010
#define MCP2515_CMD_READ  0b00000011

void mcp2515_reset() {
    uint8_t tx = MCP2515_CMD_RESET;
    spi_transmit(&tx, NULL, 1);
}

void mcp2515_write(uint8_t addr, uint8_t value) {
    uint8_t tx[3];
    tx[0] = MCP2515_CMD_WRITE;
    tx[1] = addr;
    tx[2] = value;
    spi_transmit(tx, NULL, sizeof(tx));
}

uint8_t mcp2515_read(uint8_t addr) {
    uint8_t tx[3];
    uint8_t rx[3];
    tx[0] = MCP2515_CMD_READ;
    tx[1] = addr;
    spi_transmit(tx, rx, sizeof(tx));
    return rx[2];
}

void mcp2515_isr(uint gpio, uint32_t event_mask) {
    volatile uint8_t canstat = mcp2515_read(0x0e);
    uint8_t isr = (canstat >> 1) & 0b111;

    if(isr == 0b110) {
        uint8_t tx[14] = {
            0b10010000
        };
        uint8_t rx[14] = {0};

        spi_transmit(tx, rx, sizeof(tx));

        printf("rx %d\n", rx[5] & 0b1111);
    }
}

int main() {
    stdio_init_all();

    spi_init(spi_default, 1000 * 1000);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);

    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);


    // GP20
    gpio_init(20);
    gpio_set_irq_enabled_with_callback(20, GPIO_IRQ_LEVEL_LOW, true, mcp2515_isr);


    mcp2515_reset();
    sleep_ms(250);


    volatile uint8_t canstat = mcp2515_read(0x0e);
    volatile uint8_t canctrl = mcp2515_read(0x0f);


    mcp2515_write(0x2A, 0x01);
    mcp2515_write(0x29, 0xb5);
    //mcp2515_write(0x28, 0x82);
    uint8_t tx[] = {0x05, 0x28, 0x07, 0x01};
    spi_transmit(tx, NULL, sizeof(tx));

    // interrupt on rxs
    mcp2515_write(0x2B, 3);

    mcp2515_write(0x0F, 0x07 | (1 << 3));
      

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    for(;;) {
        volatile uint8_t status = mcp2515_read(0x0F);


        uint8_t tx[24];
        tx[0] = 0b01000000;
        tx[1] = 0xFF; //SIDH
        tx[2] = 0x00; //SIDL
        tx[3] = 0x00; //EID8
        tx[4] = 0x00; // IED0
        tx[5] = 1; // DLC
        tx[6] = 0x42;
        spi_transmit(tx, NULL, 7);


        tx[0] = 0b10000001;
        spi_transmit(tx, NULL, 1);


        gpio_put(LED_PIN, 0);
        sleep_ms(250);
        gpio_put(LED_PIN, 1);
        puts("Hello World\n");
        sleep_ms(1000);
    }
}