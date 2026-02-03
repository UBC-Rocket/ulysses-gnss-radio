/*
Purpose: GPS Driver
Functionalities:
    gps_init()
    -> save UART6 (GPS) + UART1 (output)
    -> clear buffers/state
    -> start UART6 receive interrupt for 1 byte

    GPS sends bytes on UART6
    -> 1 byte arrives
    -> interrupt happens
    -> gps_uart_rx_cplt_callback() runs
    -> push byte into circular buffer
    -> re-arm receive for next 1 byte

    Main loop keeps calling gps_process()
    -> pop bytes from circular buffer
    -> feed bytes into NMEA builder

    NMEA builder
    -> wait for $ (start)
    -> keep appending bytes
    -> when \n arrives (end)
    -> send the whole sentence out UART1 (non-blocking)
    -> reset and wait for next $

    If UART error happens
    -> gps_uart_error_callback()
    -> re-arm receive again

Author: Ernie Han
*/

#include "gps.h"
#include "gps_queue.h"
#include <stdbool.h>
#include <string.h>

#define GPS_RX_RING_SZ 512u // power of two
#define GPS_RX_MASK (GPS_RX_RING_SZ - 1u)
#define GPS_NMEA_MAX 128u // NMEA typical <= 82 chars; 128 is safe-ish

#if (GPS_RX_RING_SZ & (GPS_RX_RING_SZ - 1u))
#error "GPS_RX_RING_SZ must be power of two"
#endif

static UART_HandleTypeDef *s_gps = NULL;
static UART_HandleTypeDef *s_out = NULL;

static uint8_t s_rx_byte;

static uint8_t s_ring[GPS_RX_RING_SZ];
static volatile uint16_t s_head = 0;
static volatile uint16_t s_tail = 0;

// buffer to holld NMEA data as it's being built
static uint8_t s_line[GPS_NMEA_MAX];
static uint16_t s_line_len = 0;
static bool s_in_sentence = false;

// queue variable to use to send through SPI when gps data is ready
static gps_sample_queue_t s_gps_queue;

// to keep track of ongoing UART1 transmission
static volatile bool s_out_tx_busy = false;

// buffer to hold the complete NMEA sentence + tracks the length of it
// s_line is reset as soon as NMEA sentence is sent out, so we need another
// buffer while UART1 is transmitting
static uint8_t s_out_tx_buf[GPS_NMEA_MAX];
static uint16_t s_out_tx_len = 0;

static inline bool ring_empty(void) { return s_head == s_tail; }
static inline bool ring_full(uint16_t head, uint16_t tail) {
  return ((head + 1u) & GPS_RX_MASK) == tail;
}

// pushing one byte to the circular buffer
// if overflow, drop new byte (TODO: implement drop old bytes when overflow)
static void ring_push_isr(uint8_t b) {
  uint16_t head = s_head;
  uint16_t tail = s_tail;

  if (ring_full(head, tail)) {
    // overflow: drop byte (simple policy)
    return;
  }
  s_ring[head] = b;
  s_head = (uint16_t)((head + 1u) & GPS_RX_MASK);
}

// pop each character from the tail pointer
static bool ring_pop_main(uint8_t *out) {
  if (ring_empty())
    return false;
  uint16_t tail = s_tail;
  *out = s_ring[tail];
  s_tail = (uint16_t)((tail + 1u) & GPS_RX_MASK);
  return true;
}

// resets the flag that indiciates if the NMEA sentence is being built or not
static void nmea_reset(void) {
  s_in_sentence = false;
  s_line_len = 0;
}

// helper function to send data through UART1 (non-blocking)
static void out_try_start_tx(void) {
  // check if UART1 is set && UART1 is not busy && there's data to send
  if (!s_out || s_out_tx_busy || s_out_tx_len == 0)
    return;

  // mark UART1 as busy
  s_out_tx_busy = true;

  // Start non-blocking transmit
  if (HAL_UART_Transmit_IT(s_out, s_out_tx_buf, s_out_tx_len) != HAL_OK) {
    // if transfer failed, reset busy flag to false
    s_out_tx_busy = false;
  }
}

// parse bytes in the circular buffer, build complete NMEA sentence and
static void build_complete_NMEA_sentence(uint8_t b) {
  // wait for the next '$' to start building the NMEA sentence
  if (!s_in_sentence) {
    if (b == '$') {
      s_in_sentence = true;
      s_line_len = 0;
      s_line[s_line_len++] = b;
    }
    return;
  }

  // currently in the process of building the NMEA sentence
  if (s_line_len >= (GPS_NMEA_MAX - 1u)) {
    // too long: resync
    nmea_reset();
    return;
  }

  s_line[s_line_len++] = b;

  // sentence built, reset the flag + enqueue the NMEA sentence to the queue to
  // be sent through SPI
  if (b == '\n') {
    if (s_out) {
      // UART1 not busy, copy complete NMEA to s_out_tx_buf and start
      // non-blocking transmit
      if (!s_out_tx_busy) {
        memcpy(s_out_tx_buf, s_line, s_line_len);
        s_out_tx_len = s_line_len;
        out_try_start_tx();
      }

      // this will be commented out for testing purposes
      // TODO: once testing is done, uncomment this line
      // gps_sample_enqueue(s_line, &s_gps_queue);
    }
    nmea_reset();
  }
}

// starts listening to uart6 for GPS data
void gps_init(UART_HandleTypeDef *gps_uart, UART_HandleTypeDef *out_uart) {
  s_gps = gps_uart;
  s_out = out_uart;

  s_head = s_tail = 0;
  nmea_reset();

  // initialize queue
  gps_sample_queue_init(&s_gps_queue);

  if (s_gps) {
    (void)HAL_UART_Receive_IT(s_gps, &s_rx_byte, 1);
  }
}

// "main loop" that will process the circular buffer and build full NMEA
// sentence and send to UART1
void gps_process(void) {
  uint8_t b;
  while (ring_pop_main(&b)) {
    build_complete_NMEA_sentence(b);
  }
}

// gets called when HAL_UART_Receive_IT() has received 1 byte -> when data
// received, push to ring bufffer and rearm
void gps_uart_rx_cplt_callback(UART_HandleTypeDef *huart) {
  if (!s_gps || huart != s_gps)
    return;

  // push the received byte to the ring buffer
  ring_push_isr(s_rx_byte);

  // rearm
  (void)HAL_UART_Receive_IT(s_gps, &s_rx_byte, 1);
}

// gets called when HAL_UART_Transmit_IT() has finished sending data
// marks UART1 as free and resets the buffer length to 0
void gps_uart_tx_cplt_callback(UART_HandleTypeDef *huart) {
  if (!s_out || huart != s_out)
    return;

  s_out_tx_busy = false;
  s_out_tx_len = 0;
}

// error handling
void gps_uart_error_callback(UART_HandleTypeDef *huart) {
  if (!s_gps || huart != s_gps)
    return;

  // if something goes wrong, just try to rearm
  (void)HAL_UART_Receive_IT(s_gps, &s_rx_byte, 1);
}
