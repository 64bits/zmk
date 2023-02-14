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

#define READ 0
#define WRITE 1
#define HIGH 1
#define LOW 0

// TODO: Use something better than buttons (custom device?)
static struct gpio_dt_spec tp_dat = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(button0), gpios, 0);
static struct gpio_dt_spec tp_clk = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(button0), gpios, 1);
static struct gpio_dt_spec tp_rst = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(button0), gpios, 2);

uint8_t bit = 0x01; // TODO: Remove?
uint64_t current_bytes;
uint16_t transmit;

static struct k_work_delayable initialize_trackpoint;
static struct k_work_delayable poll_trackpoint;

K_MUTEX_DEFINE(transmission);
K_CONDVAR_DEFINE(transmission_end);

static struct gpio_callback gpio_read_clk_ctx;
static struct gpio_callback gpio_read_dat_ctx;
static struct gpio_callback gpio_write_clk_ctx;

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
}

// TODO: Fix the name, it's not correct, we write on clock LOW
static void handle_clk_lo_write_int(const struct device *gpio,
                              struct gpio_callback *cb, uint32_t pins)
{
    k_mutex_lock(&transmission, K_FOREVER);
    if(transmit > 0) {
        gpio_pin_set_dt(&tp_dat, (transmit & 1) ? 1 : 0);
        transmit = transmit >> 1;
    } else {
        k_condvar_signal(&transmission_end);
    }
    k_mutex_unlock(&transmission);
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

void set_gpio_mode(uint8_t mode) {
    if(mode == WRITE) {
        gpio_remove_callback(tp_dat.port, &gpio_read_dat_ctx);
        gpio_remove_callback(tp_clk.port, &gpio_read_clk_ctx);
        gpio_add_callback(tp_clk.port, &gpio_write_clk_ctx);
        gpio_pin_interrupt_configure_dt(&tp_clk, GPIO_INT_EDGE_TO_INACTIVE);
        gpio_pin_interrupt_configure_dt(&tp_dat, GPIO_INT_DISABLE);
    } else {
        gpio_remove_callback(tp_clk.port, &gpio_write_clk_ctx);
        gpio_add_callback(tp_dat.port, &gpio_read_dat_ctx);
        gpio_add_callback(tp_clk.port, &gpio_read_clk_ctx);
        gpio_pin_interrupt_configure_dt(&tp_clk, GPIO_INT_EDGE_TO_INACTIVE);
        gpio_pin_interrupt_configure_dt(&tp_dat, GPIO_INT_EDGE_TO_INACTIVE);
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

void write(uint8_t data) {
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
    set_gpio_mode(WRITE);
    go_lo(&tp_dat);
    // Block on condition of transmission end
    k_condvar_wait(&transmission_end, &transmission, K_FOREVER);
    set_gpio_mode(READ);
    // Return clock and data to the high position
    go_hi(&tp_clk);
    go_hi(&tp_dat);
    k_mutex_unlock(&transmission);
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

int8_t read_byte_at_idx(int8_t index) {
    return bit_reverse((current_bytes >> (3+index*11)) & 0xFF);
}

void set_sensitivity(uint8_t sens) {
    // Set sensitivity
    write(0xe2);
    k_sleep(K_MSEC(5));
    write(0x81);
    k_sleep(K_MSEC(5));
    write(0x4a);
    k_sleep(K_MSEC(5));
    write(sens);
    k_sleep(K_MSEC(5));
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

static void poll_trackpoint_fn(struct k_work *work)
{
    // TODO: Don't update position when either one is 0 (help with noise)
    write(0xeb);
    k_sleep(K_MSEC(1)); // Post-write wait?
    current_bytes = 0;
    k_sleep(K_MSEC(5)); // Reset takes ~60, but poll is fast
    zmk_hid_mouse_movement_set(0, 0);
    zmk_hid_mouse_scroll_set(0, 0);
    zmk_hid_mouse_movement_update(CLAMP(read_byte_at_idx(1), INT8_MIN, INT8_MAX),
                               CLAMP(-read_byte_at_idx(0), INT8_MIN, INT8_MAX));
    zmk_endpoints_send_mouse_report();
    print_all_bytes();
    k_work_reschedule_for_queue(&trackpoint_work_q, work, K_MSEC(50));
}

int zmk_trackpoint_init() {
    LOG_INF("\nTrackpoint called\n");
    LOG_INF("\nThe pins are %d, %d, and %d\n", tp_dat.pin, tp_clk.pin, tp_rst.pin);
    gpio_pin_configure_dt(&tp_clk, GPIO_INPUT | GPIO_OUTPUT_HIGH );
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
    set_gpio_mode(READ);
    // Start Trackpoint work
    k_sleep(K_MSEC(500));
    gpio_pin_set_dt(&tp_rst, LOW);
    // Set sensitivity
    set_sensitivity(0xc0);
    // Let's go
    k_work_init_delayable(&poll_trackpoint, poll_trackpoint_fn);
    k_work_reschedule_for_queue(&trackpoint_work_q, &poll_trackpoint, K_MSEC(50));
    // In sleep mode, interrupts are used to activate the polling function until
    // such a time as when the timer used therein (which is reset upon activity)
    // expires -- this should help save on battery
    // set_gpio_mode(SLEEP);
    return 0;
}