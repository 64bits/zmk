#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <dt-bindings/zmk/mouse.h>

#define TP_CLK_PIN 6
#define TP_DAT_PIN 7
#define HIGH 1
#define LOW 0

struct DataReport {
    uint8_t state;
    int8_t x;
    int8_t y;
} dataReport;

const struct device *dev;
uint8_t result = 0;

static const struct device *get_trackpoint_device(void)
{
    const struct device *const dev = device_get_binding("GPIO_0");

    if (dev == NULL) {
        /* No such node, or the node does not have status "okay". */
        printk("\nError: GPIO not found.\n");
        return NULL;
    }

    if (!device_is_ready(dev)) {
        printk("\nError: GPIO \"%s\" is not ready; "
               "check the driver initialization logs for errors.\n",
               dev->name);
        return NULL;
    }

    printk("Found device \"%s\", getting sensor data\n", dev->name);
    return dev;
}

void write(uint8_t data) {
    uint8_t i;
    uint8_t parity = 1;

    gohi(TP_DAT_PIN);
    gohi(TP_CLK_PIN);
    k_sleep(K_USEC(300));
    golo(TP_CLK_PIN);
    k_sleep(K_USEC(300));
    golo(TP_DAT_PIN);
    k_sleep(K_USEC(10));
    gohi(TP_CLK_PIN);	// start bit
    /* wait for device to take control of clock */
    while (gpio_pin_get(dev, TP_CLK_PIN) == HIGH)
        ;	// this loop intentionally left blank
    // clear to send data
    for (i=0; i < 8; i++)
    {
        if (data & 0x01)
        {
            gohi(TP_DAT_PIN);
        } else {
            golo(TP_DAT_PIN);
        }
        // wait for clock
        while (gpio_pin_get(dev, TP_CLK_PIN) == LOW)
            ;
        while (gpio_pin_get(dev, TP_CLK_PIN) == HIGH)
            ;
        parity = parity ^ (data & 0x01);
        data = data >> 1;
    }
    // parity bit
    if (parity)
    {
        gohi(TP_DAT_PIN);
    } else {
        golo(TP_DAT_PIN);
    }
    // clock cycle - like ack.
    while (gpio_pin_get(dev, TP_CLK_PIN) == LOW)
        ;
    while (gpio_pin_get(dev, TP_CLK_PIN) == HIGH)
        ;
    // stop bit
    gohi(TP_DAT_PIN);
    k_sleep(K_USEC(50));
    while (gpio_pin_get(dev, TP_CLK_PIN) == HIGH)
        ;
    // mode switch
    while ((gpio_pin_get(dev, TP_CLK_PIN) == LOW) || (gpio_pin_get(dev, TP_DAT_PIN) == LOW))
        ;
    // hold up incoming data
    golo(TP_CLK_PIN);
}

uint8_t read(void) {
    uint8_t data = 0x00;
    uint8_t i;
    uint8_t bit = 0x01;

    // start clock
    gohi(TP_CLK_PIN);
    gohi(TP_DAT_PIN);
    k_sleep(K_USEC(50));
    while (gpio_pin_get(dev, TP_CLK_PIN) == HIGH)
        ;
    k_sleep(K_USEC(5));  // not sure why.
    while (gpio_pin_get(dev, TP_CLK_PIN) == LOW)
        ;  // eat start bit
    for (i = 0; i < 8; i++) {
        while (gpio_pin_get(dev, TP_CLK_PIN) == HIGH)
            ;
        if (gpio_pin_get(dev, TP_DAT_PIN) == HIGH) {
            data = data | bit;
        }
        while (gpio_pin_get(dev, TP_CLK_PIN) == LOW)
            ;
        bit = bit << 1;
    }
    // eat parity bit, ignore it.
    while (gpio_pin_get(dev, TP_CLK_PIN) == HIGH)
        ;
    while (gpio_pin_get(dev, TP_CLK_PIN) == LOW)
        ;
    // eat stop bit
    while (gpio_pin_get(dev, TP_CLK_PIN) == HIGH)
        ;
    while (gpio_pin_get(dev, TP_CLK_PIN) == LOW)
        ;
    golo(TP_CLK_PIN);  // hold incoming data

    return data;
}


void main(void)
{
    dev = get_trackpoint_device();

    if (dev == NULL) {
        return;
    }

    // Basic setup
    gohi(TP_CLK_PIN);
    gohi(TP_DAT_PIN);

    while (1) {
        k_sleep(K_MSEC(50));
        write(0xeb);
        result = read();
        if(250 != result) {
            printk("trackpoint read failed\n");
            continue;
        }
        dataReport.state = read();
        dataReport.x = read();
        dataReport.y = read();
        printk("reading trackpoint...%d, -%d!\n", dataReport.x, dataReport.y);
        zmk_hid_mouse_movement_set(0, 0);
        zmk_hid_mouse_scroll_set(0, 0);
        zmk_hid_mouse_movement_update(CLAMP(dataReport.x, INT8_MIN, INT8_MAX), CLAMP(-dataReport.y, INT8_MIN, INT8_MAX));
        zmk_endpoints_send_mouse_report();
    }
}

void gohi(uint8_t pin) {
    gpio_pin_configure(dev, pin, GPIO_INPUT | GPIO_PULL_UP );
    gpio_pin_set(dev, pin, 1);
}

void golo(uint8_t pin) {
    gpio_pin_configure(dev, pin, GPIO_OUTPUT | GPIO_PULL_UP);
    gpio_pin_set(dev, pin, 0);
}
