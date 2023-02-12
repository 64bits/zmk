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

#define TP_CLK_PIN 13 // 8
#define TP_DAT_PIN 14 // 9
#define TP_RST_PIN 15 // 10
#define HIGH 1
#define LOW 0

int bitsToRead;
uint16_t lastByte;
uint8_t bit = 0x01;

const struct device *gpiodev;
static struct k_work_delayable initialize_trackpoint;

static struct gpio_callback gpio_clk_ctx;

K_THREAD_STACK_DEFINE(trackpoint_work_stack_area, 2048);
static struct k_work_q trackpoint_work_q;

struct k_work_q *zmk_trackpoint_work_q() {
    return &trackpoint_work_q;
}

static void handle_clk_int(const struct device *gpio,
                           struct gpio_callback *cb, uint32_t pins)
{
    if (bitsToRead != 0) {
        lastByte = lastByte << 1;
        if (gpio_pin_get_raw(gpio, TP_DAT_PIN) == HIGH) {
            lastByte = lastByte | bit;
        }
        bitsToRead--;
    } else {
        gpio_pin_set_raw(gpio, TP_CLK_PIN, 0);
        printk("{%u}", (unsigned int)lastByte);
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

static void read_result_fn(struct k_work *work)
{
    printk("\n[TP] The last read byte was: 0x%x\n", lastByte);
}

static void initialize_trackpoint_fn(struct k_work *work)
{
    gpio_pin_interrupt_configure(gpiodev, TP_CLK_PIN, GPIO_INT_EDGE_TO_INACTIVE);
    // Sleep for a little while
    k_sleep(K_MSEC(50));
    // Continue
    gpio_pin_set(gpiodev, TP_RST_PIN, LOW);
    bitsToRead = 11;
    gohi(TP_CLK_PIN);
    gohi(TP_DAT_PIN);
    // My expectation is that the TP will now send me data...but it doesn't work
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
    gpio_init_callback(&gpio_clk_ctx, handle_clk_int, BIT(TP_CLK_PIN));
    gpio_add_callback(gpiodev, &gpio_clk_ctx);
    // Start Trackpoint work
    k_work_init_delayable(&initialize_trackpoint, initialize_trackpoint_fn);
    k_work_reschedule_for_queue(&trackpoint_work_q, &initialize_trackpoint, K_MSEC(2000));
    return 0;
}