/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <kernel.h>
#include <zmk/trackpoint.h>
#include <drivers/gpio.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <inttypes.h>

#define TP_CLK_PIN 13 // 8
#define TP_DAT_PIN 14 // 9
#define TP_RST_PIN 15 // 10
#define HIGH 1
#define LOW 0

uint8_t bitsToRead;
uint64_t currentBytes;
uint8_t bit = 0x01;

const struct device *gpiodev;
static struct k_work initialize_trackpoint;
static struct k_work read_result;

static struct gpio_callback gpio_clk_ctx;
static struct gpio_callback gpio_dat_ctx;

K_THREAD_STACK_DEFINE(trackpoint_work_stack_area, 2048);
static struct k_work_q trackpoint_work_q;

struct k_work_q *zmk_trackpoint_work_q() {
    return &trackpoint_work_q;
}

static void handle_clk_int(const struct device *gpio,
                           struct gpio_callback *cb, uint32_t pins)
{
    currentBytes = currentBytes | bit;
    currentBytes = currentBytes << 1;
}

static void handle_dat_int(const struct device *gpio,
                           struct gpio_callback *cb, uint32_t pins)
{
    if (bit) {
        bit = 0;
        gpio_pin_interrupt_configure(gpiodev, TP_DAT_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    } else {
        bit = 1;
        gpio_pin_interrupt_configure(gpiodev, TP_DAT_PIN, GPIO_INT_EDGE_TO_INACTIVE);
    }
}

void gohi(uint8_t pin) {
    gpio_pin_set_raw(gpiodev, pin, 1);
}

void golo(uint8_t pin) {
    gpio_pin_set_raw(gpiodev, pin, 0);
}

int read(uint8_t pin) {
    return gpio_pin_get_raw(gpiodev, pin);
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
    while (read(TP_CLK_PIN) == HIGH)
        ;	// this loop intentionally left blank
    // clear to send data
    for (i=0; i < 8; i++)
    {
        // wait for clock
        if (data & 0x01)
        {
            gohi(TP_DAT_PIN);
        } else {
            golo(TP_DAT_PIN);
        }
        while (read(TP_CLK_PIN) == LOW)
            ;
        while (read(TP_CLK_PIN) == HIGH)
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
    while (read(TP_CLK_PIN) == LOW)
        ;
    while (read(TP_CLK_PIN) == HIGH)
        ;
    // stop bit
    gohi(TP_DAT_PIN);
    k_sleep(K_USEC(30));
    while (read(TP_CLK_PIN) == HIGH)
        ;
    golo(TP_CLK_PIN); return;
    // mode switch
    while ((read(TP_CLK_PIN) == LOW) || (read(TP_DAT_PIN) == LOW))
        ;
    // hold up incoming data
    golo(TP_CLK_PIN);
}

static void initialize_trackpoint_fn(struct k_work *work)
{
    // Continue
    k_sleep(K_MSEC(2000));
    gpio_pin_set(gpiodev, TP_RST_PIN, LOW);
    bitsToRead = 22;
    gohi(TP_CLK_PIN);
    gohi(TP_DAT_PIN);
    k_sleep(K_MSEC(1000));
}

uint8_t bitReverse(uint8_t num) {
    return ((num & 0x01) << 7)
    | ((num & 0x02) << 5)
    | ((num & 0x04) << 3)
    | ((num & 0x08) << 1)
    | ((num & 0x10) >> 1)
    | ((num & 0x20) >> 3)
    | ((num & 0x40) >> 5)
    | ((num & 0x80) >> 7);
}

void printAllBytes()
{
    uint8_t firstByte = bitReverse((currentBytes >> 3) & 0xFF);
    uint8_t secondByte = bitReverse((currentBytes >> 14) & 0xFF);
    uint8_t thirdByte = bitReverse((currentBytes >> 25) & 0xFF);
    printk("binary = 0x%" PRIx64 "\n", currentBytes);
    printk("first byte = 0x%x", firstByte);
    printk("second byte = 0x%x", secondByte);
    printk("third byte = 0x%x", thirdByte);
}

int zmk_trackpoint_init() {
    gpiodev = device_get_binding("GPIO_1");
    gpio_pin_configure(gpiodev, TP_CLK_PIN, GPIO_INPUT | GPIO_OUTPUT_HIGH );
    gpio_pin_configure(gpiodev, TP_DAT_PIN, GPIO_INPUT | GPIO_OUTPUT_HIGH );
    gpio_pin_configure(gpiodev, TP_RST_PIN, GPIO_OUTPUT);
    gpio_pin_set(gpiodev, TP_RST_PIN, HIGH);
    k_work_queue_start(&trackpoint_work_q, trackpoint_work_stack_area,
                       K_THREAD_STACK_SIZEOF(trackpoint_work_stack_area),
                       CONFIG_SYSTEM_WORKQUEUE_PRIORITY,
                       NULL);
    // Initialize gpio callbacks
    gpio_init_callback(&gpio_dat_ctx, handle_dat_int, BIT(TP_DAT_PIN));
    gpio_init_callback(&gpio_clk_ctx, handle_clk_int, BIT(TP_CLK_PIN));
    gpio_add_callback(gpiodev, &gpio_dat_ctx);
    gpio_add_callback(gpiodev, &gpio_clk_ctx);
    gpio_pin_interrupt_configure(gpiodev, TP_CLK_PIN, GPIO_INT_EDGE_TO_INACTIVE);
    gpio_pin_interrupt_configure(gpiodev, TP_DAT_PIN, GPIO_INT_EDGE_TO_INACTIVE);
    // Start Trackpoint work
    k_sleep(K_MSEC(2000));
    gpio_pin_set(gpiodev, TP_RST_PIN, LOW);
    k_sleep(K_MSEC(1000));
    write(0xff);
    printk("{INIT}");
    k_sleep(K_MSEC(1)); // Post-write wait?
    currentBytes = 0;
    gpio_pin_set(gpiodev, TP_CLK_PIN, HIGH);
    gpio_pin_set(gpiodev, TP_DAT_PIN, HIGH);
    k_sleep(K_MSEC(60));
    //currentBytes = currentBytes >> 5;
    printAllBytes();
//    k_work_init_delayable(&initialize_trackpoint, initialize_trackpoint_fn);
//    k_work_reschedule_for_queue(&trackpoint_work_q, &initialize_trackpoint, K_MSEC(2000));
    return 0;
}