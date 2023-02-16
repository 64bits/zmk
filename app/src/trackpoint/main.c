/*
 * Trackpoint driver
 * ---
 * CLK and DAT lines are held high when idle, always allowing TP to send
 * in data, though it should only do that in response to commands
 */
#include <devicetree.h>
#include <kernel.h>
#include <drivers/gpio.h>
#include <zmk/trackpoint.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <inttypes.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(trackpoint);

#define POLL_TTL 750

#define READ 0
#define WRITE 1
#define SLEEP 2
#define HIGH 1
#define LOW 0

// TODO: Use something better than buttons (custom device?)
static struct gpio_dt_spec tp_dat = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(button0), gpios, 0);
static struct gpio_dt_spec tp_clk = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(button0), gpios, 1);
static struct gpio_dt_spec tp_rst = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(button0), gpios, 2);

K_TIMER_DEFINE(poll_timer, NULL, NULL);

uint8_t bit = 0x01; // TODO: Remove?
int8_t to_read;
uint64_t current_bytes;
uint16_t transmit;

static struct k_work_delayable poll_trackpoint;
static struct k_work count_read_bytes;
static struct k_work wake_trackpoint;

K_MUTEX_DEFINE(transmission);
K_CONDVAR_DEFINE(transmission_end);

static struct gpio_callback gpio_read_clk_ctx;
static struct gpio_callback gpio_read_dat_ctx;
static struct gpio_callback gpio_write_clk_ctx;
static struct gpio_callback gpio_sleep_clk_ctx;

K_THREAD_STACK_DEFINE(trackpoint_work_stack_area, 2048);
static struct k_work_q trackpoint_work_q;

struct k_work_q *zmk_trackpoint_work_q() {
    return &trackpoint_work_q;
}

static void handle_clk_lo_read_int(const struct device *gpio,
                           struct gpio_callback *cb, uint32_t pins)
{
    current_bytes = current_bytes | bit;
    current_bytes = current_bytes << 1;
    k_work_submit_to_queue(&trackpoint_work_q, &count_read_bytes);
}

static void handle_clk_lo_write_int(const struct device *gpio,
                              struct gpio_callback *cb, uint32_t pins)
{
    if(transmit > 0) {
        gpio_pin_set_dt(&tp_dat, (transmit & 1) ? 1 : 0);
        transmit = transmit >> 1;
    } else {
        k_mutex_lock(&transmission, K_FOREVER);
        k_condvar_signal(&transmission_end);
        k_mutex_unlock(&transmission);
    }
}

static void handle_dat_int(const struct device *gpio,
                           struct gpio_callback *cb, uint32_t pins)
{
    if (bit) {
        bit = 0;
        gpio_pin_interrupt_configure_dt(&tp_dat, GPIO_INT_EDGE_TO_ACTIVE);
    } else {
        bit = 1;
        gpio_pin_interrupt_configure_dt(&tp_dat, GPIO_INT_EDGE_TO_INACTIVE);
    }
}


static void handle_clk_lo_sleep_int(const struct device *gpio,
                                    struct gpio_callback *cb, uint32_t pins)
{
    gpio_remove_callback(tp_clk.port, &gpio_sleep_clk_ctx);
    k_work_submit_to_queue(&trackpoint_work_q, &wake_trackpoint);
}

void set_gpio_mode(uint8_t mode) {
    gpio_remove_callback(tp_clk.port, &gpio_write_clk_ctx);
    gpio_remove_callback(tp_dat.port, &gpio_read_dat_ctx);
    gpio_remove_callback(tp_clk.port, &gpio_read_clk_ctx);
    gpio_remove_callback(tp_clk.port, &gpio_sleep_clk_ctx);

    if(mode == WRITE) {
        gpio_add_callback(tp_clk.port, &gpio_write_clk_ctx);
        gpio_pin_interrupt_configure_dt(&tp_clk, GPIO_INT_EDGE_TO_INACTIVE);
        gpio_pin_interrupt_configure_dt(&tp_dat, GPIO_INT_DISABLE);
    } else if(mode == READ) {
        gpio_add_callback(tp_dat.port, &gpio_read_dat_ctx);
        gpio_add_callback(tp_clk.port, &gpio_read_clk_ctx);
        gpio_pin_interrupt_configure_dt(&tp_clk, GPIO_INT_EDGE_TO_INACTIVE);
        gpio_pin_interrupt_configure_dt(&tp_dat, GPIO_INT_EDGE_TO_INACTIVE);
    } else {
        gpio_add_callback(tp_clk.port, &gpio_sleep_clk_ctx);
        gpio_pin_interrupt_configure_dt(&tp_clk, GPIO_INT_EDGE_TO_INACTIVE);
        gpio_pin_interrupt_configure_dt(&tp_dat, GPIO_INT_DISABLE);
    }
}

void go_hi(const struct gpio_dt_spec* spec) {
    gpio_pin_set_dt(spec, 1);
}

void go_lo(const struct gpio_dt_spec* spec) {
    gpio_pin_set_dt(spec, 0);
}

int read(const struct gpio_dt_spec* spec) {
    return gpio_pin_get_dt(spec);
}

static void count_read_bytes_fn(struct k_work *work) {
    LOG_INF("R");
    if(--to_read <= 0) {
        k_mutex_lock(&transmission, K_FOREVER);
        gpio_remove_callback(tp_clk.port, &gpio_read_clk_ctx);
        k_condvar_signal(&transmission_end);
        k_mutex_unlock(&transmission);
    }
}

int8_t bit_reverse(int8_t num) {
    return ((num & 0x01) << 7)
           | ((num & 0x02) << 5)
           | ((num & 0x04) << 3)
           | ((num & 0x08) << 1)
           | ((num & 0x10) >> 1)
           | ((num & 0x20) >> 3)
           | ((num & 0x40) >> 5)
           | ((num & 0x80) >> 7);
}

int8_t byte_at_idx(uint64_t list, int8_t index) {
    return bit_reverse((list >> (3+index*11)) & 0xFF);
}


uint64_t write(uint8_t data, uint8_t num_response_bits) {
    uint64_t result;
    // Prepare the bits to be sent
    uint16_t bits;
    uint8_t parity = 1;
    uint8_t mask = 0x01;
    bits = data;
    // Calculate parity
    for (int i=0; i < 8; i++)
    {
        parity = parity ^ (data & mask);
        data = data >> 1;
    }
    bits = (parity << 8) | bits;
    bits = (0x01 << 9) | bits; // Stop bit
    transmit = bits;

    // Switch modes
    k_mutex_lock(&transmission, K_FOREVER);
    go_lo(&tp_dat);
    go_hi(&tp_clk);
    // Block on condition of transmission end
    k_condvar_wait(&transmission_end, &transmission, K_FOREVER);

    // Read the response
    to_read = num_response_bits;
    set_gpio_mode(READ);
    LOG_INF("\nWrote, reading\n");
    k_condvar_wait(&transmission_end, &transmission, K_FOREVER);
    go_lo(&tp_clk); // Important that this happens as soon as transmission ends
    current_bytes = current_bytes >> 1;
    LOG_INF("\nRead %d bits\n", num_response_bits - to_read);
    print_all_bytes();
    k_mutex_unlock(&transmission);
    return result;
}


static void wake_trackpoint_fn(struct k_work *work) {
    LOG_INF("Woke up");
    // Writing at this point seems to work, but reading doesn't
    // I think the write is not _really_ working, I'm just minsinterpreting the clock
    go_lo(&tp_clk); // We no longer accept data from the TP
    go_hi(&tp_dat); // Just to be sure
    k_sleep(K_USEC(2500)); // Wait a little
    // All that didn't help, still unable to read...perhaps it has to do with mode change
    write(0xf5, 11); // Disable stream mode
    k_timer_start(&poll_timer, K_MSEC(POLL_TTL), K_NO_WAIT);
    k_work_reschedule_for_queue(&trackpoint_work_q, &poll_trackpoint, K_MSEC(50));
}

void set_sensitivity(uint8_t sens) {
    // Set sensitivity
    write(0xe2, 11);
    write(0x81, 11);
    write(0x4a, 11);
    write(sens, 11);
}

void print_all_bytes()
{
    uint8_t firstByte = bit_reverse((current_bytes >> 3) & 0xFF);
    uint8_t secondByte = bit_reverse((current_bytes >> 14) & 0xFF);
    uint8_t thirdByte = bit_reverse((current_bytes >> 25) & 0xFF);
    LOG_INF("binary = 0x%" PRIx64 "\n", current_bytes);
    LOG_INF("first byte = 0x%x\n", firstByte);
    LOG_INF("second byte = 0x%x\n", secondByte);
    LOG_INF("third byte = 0x%x\n\n", thirdByte);
}

void enter_sleep_mode()
{
    write(0xf4, 11); // Enable stream mode
    // set_gpio_mode(SLEEP); // Turn on when fixed
    go_hi(&tp_clk);
    k_sleep(K_SECONDS(1)); // Required to really get the test testin
    LOG_INF("DOWIT");
    // These are not _really_ writing...
    // This is supposed to say 'wait, no IO allowed'
    go_lo(&tp_clk);
    go_hi(&tp_dat);
    k_sleep(K_MSEC(250)); // Doesn't seem to matter if I wait longer?
    // If this can read & write, it's fixed
    write(0xf5, 11);

}

uint64_t res;
int16_t x_delta;
int16_t y_delta;
static void poll_trackpoint_fn(struct k_work *work)
{
    res = write(0xeb, 44);
    x_delta = byte_at_idx(res, 1);
    y_delta = -byte_at_idx(res, 0);
    // If either value is exactly zero, it's probably drift
    if(x_delta !=0 && y_delta !=0) {
        zmk_hid_mouse_movement_set(0, 0);
        zmk_hid_mouse_scroll_set(0, 0);
        zmk_hid_mouse_movement_update(CLAMP(x_delta, INT8_MIN, INT8_MAX),
                                      CLAMP(y_delta, INT8_MIN, INT8_MAX));
        zmk_endpoints_send_mouse_report();
        // If the mouse was moved, we restart the timer
        k_timer_start(&poll_timer, K_MSEC(POLL_TTL), K_NO_WAIT);
    }
    if(k_timer_status_get(&poll_timer) < 0) {
        // If the timer hasn't expired, continue polling
        k_work_reschedule_for_queue(&trackpoint_work_q, work, K_MSEC(50));
    } else {
        enter_sleep_mode();
    }
}

int zmk_trackpoint_init() {
    LOG_INF("\nTrackpoint called\n");
    LOG_INF("\nThe pins are %d, %d, and %d\n", tp_dat.pin, tp_clk.pin, tp_rst.pin);
    gpio_pin_configure_dt(&tp_clk, GPIO_INPUT | GPIO_OUTPUT_LOW );
    gpio_pin_configure_dt(&tp_dat, GPIO_INPUT | GPIO_OUTPUT_HIGH );
    gpio_pin_configure_dt(&tp_rst, GPIO_OUTPUT);
    gpio_pin_set_dt(&tp_rst, HIGH);
    k_work_queue_start(&trackpoint_work_q, trackpoint_work_stack_area,
                       K_THREAD_STACK_SIZEOF(trackpoint_work_stack_area),
                       CONFIG_SYSTEM_WORKQUEUE_PRIORITY,
                       NULL);
    // Initialize gpio callbacks
    gpio_init_callback(&gpio_read_dat_ctx, handle_dat_int, BIT(tp_dat.pin));
    gpio_init_callback(&gpio_read_clk_ctx, handle_clk_lo_read_int, BIT(tp_clk.pin));
    gpio_init_callback(&gpio_write_clk_ctx, handle_clk_lo_write_int, BIT(tp_clk.pin));
    gpio_init_callback(&gpio_sleep_clk_ctx, handle_clk_lo_sleep_int, BIT(tp_clk.pin));
    k_work_init(&count_read_bytes, count_read_bytes_fn);
    k_work_init(&wake_trackpoint, wake_trackpoint_fn);
    k_work_init_delayable(&poll_trackpoint, poll_trackpoint_fn);
    // Start Trackpoint work
    k_sleep(K_MSEC(500));
    gpio_pin_set_dt(&tp_rst, LOW);
    // Set sensitivity
    set_sensitivity(0xc0);
    // Let's go
    enter_sleep_mode();
    return 0;
}