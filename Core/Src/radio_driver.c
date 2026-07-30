/**
 * @file radio_driver.c
 * @brief Radio transceiver driver using DMA circular + Character Match
 *
 * Receives null-terminated messages from radio via UART5 using DMA
 * in circular mode with Character Match on 0x00 as the sole interrupt
 * trigger. DMA never stops — no restart gaps, no data loss.
 *
 * TX: Messages are sent followed by null terminator (0x00).
 * RX: Bytes accumulate until 0x00 delimiter, then message is enqueued.
 */

#include "radio_driver.h"
#include "stm32g0xx_hal.h"
#include <string.h>
#ifdef DEBUG
#include "debug_uart.h"
#endif

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define RADIO_DMA_BUF_SIZE    512u   /**< DMA circular buffer size (power of 2) */

#if (RADIO_DMA_BUF_SIZE & (RADIO_DMA_BUF_SIZE - 1u))
#error "RADIO_DMA_BUF_SIZE must be power of two"
#endif

/* ============================================================================
 * External References
 * ============================================================================ */

extern UART_HandleTypeDef huart5;  /* Radio UART (defined in main.c) */

/* ============================================================================
 * Private State
 * ============================================================================ */

static radio_message_queue_t *rx_queue = NULL;

/** DMA circular receive buffer */
static uint8_t s_dma_buf[RADIO_DMA_BUF_SIZE];

/** Last processed position in DMA buffer */
static volatile uint16_t s_last_pos = 0;

/** Message accumulation buffer */
static uint8_t s_msg_buf[RADIO_MAX_MESSAGE_LEN];
static uint16_t s_msg_len = 0;

/** Driver initialized flag */
static bool s_initialized = false;

/* ── AT passthrough session state ── */

/** Raw ring sizes (power of two; masked indexing) */
#define AT_RX_RING_SIZE   256u
#define AT_TX_RING_SIZE   256u

/**
 * Watchdog: end the session if the master stops issuing AT opcodes.
 *
 * The master polls for replies every few milliseconds throughout a session,
 * so any gap this long means it is gone (reset, brownout, SPI fault). An
 * abandoned session would leave the modem serial in raw mode with radio TX
 * parked -- a silent, permanent loss of downlink. Recovering on our own is
 * far better than waiting for a master that may never come back.
 */
#define RADIO_AT_SESSION_TIMEOUT_MS  10000u

/** True while an AT session is open; the CM handler stands down */
static volatile bool s_at_active = false;

/** HAL tick of the last AT opcode, for the watchdog above */
static volatile uint32_t s_at_last_activity = 0;

/**
 * Set when the message parser must discard bytes until the next 0x00.
 *
 * Ending a session drops residue by snapshotting the DMA write position,
 * but the modem keeps talking past that instant -- the echo of ATZ, a
 * trailing OK, reboot chatter. That tail is ASCII, so it carries no 0x00
 * and feed_byte() will not enqueue it; it just accumulates and gets
 * prepended to the next genuine uplink frame, which then fails to decode.
 * Resyncing on a real terminator is the only way to land on a boundary.
 */
static volatile bool s_resync = false;

/**
 * Discard a partial message that has stalled without its terminator.
 *
 * The parser splits on 0x00 and waits indefinitely otherwise, so a stray
 * byte with no terminator behind it is not dropped -- it sits in the
 * accumulator and is prepended to the next genuine frame, whenever that
 * arrives. Observed 2026-07-30: "++++++" leaked onto the air from a ground
 * modem still in transparent mode, waited ten minutes, then fused with an
 * uplink command and broke its COBS decode.
 *
 * A real frame arrives as one burst; anything still incomplete after this
 * long is debris. Generous enough not to truncate a slow frame at low air
 * rates (a 256-byte message is ~500 ms even at 4 kbps).
 */
#define RADIO_MSG_IDLE_TIMEOUT_MS  1000u

/** HAL tick of the last byte accumulated into s_msg_buf */
static volatile uint32_t s_msg_last_tick = 0;

/** Raw bytes received from the modem, verbatim (no 0x00 splitting) */
static uint8_t s_at_rx_ring[AT_RX_RING_SIZE];
static volatile uint16_t s_at_rx_head = 0;  /* written by radio_at_poll */
static volatile uint16_t s_at_rx_tail = 0;  /* read by radio_at_peek/consume */

/** Raw bytes staged for the modem, written out by radio_at_flush */
static uint8_t s_at_tx_ring[AT_TX_RING_SIZE];
static volatile uint16_t s_at_tx_head = 0;  /* written by radio_at_write (ISR) */
static volatile uint16_t s_at_tx_tail = 0;  /* read by radio_at_flush */

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static void process_dma_data(uint16_t new_pos);
static void feed_byte(uint8_t b);
static uint16_t dma_write_pos(void);

/* ============================================================================
 * Public API
 * ============================================================================ */

void radio_init(radio_message_queue_t *queue)
{
    rx_queue = queue;
    radio_message_queue_init(rx_queue);

    /* Reset state */
    s_last_pos = 0;
    s_msg_len = 0;
    s_resync = false;
    memset(s_dma_buf, 0, sizeof(s_dma_buf));
    memset(s_msg_buf, 0, sizeof(s_msg_buf));

    /* ADD[7:0] can only be written when UE=0 or RE=0 (RM0444) */
    __HAL_UART_DISABLE(&huart5);
    MODIFY_REG(huart5.Instance->CR2, USART_CR2_ADD,
               ((uint32_t)0x00 << USART_CR2_ADD_Pos));
    __HAL_UART_ENABLE(&huart5);

    /* Start DMA circular reception (no IDLE — CM is the only trigger) */
    HAL_UART_Receive_DMA(&huart5, s_dma_buf, RADIO_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_CM);

    s_initialized = true;
}

bool radio_send(const uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0 || len > 254) {
        return false;
    }

    /* Send data bytes */
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart5, (uint8_t *)data, len, 1000);
    if (status != HAL_OK) {
        return false;
    }

    /* Send null terminator */
    uint8_t null_byte = 0x00;
    status = HAL_UART_Transmit(&huart5, &null_byte, 1, 100);

    return (status == HAL_OK);
}

bool radio_available(void)
{
    if (rx_queue == NULL) {
        return false;
    }
    return !radio_message_queue_empty(rx_queue);
}

uint8_t radio_read(uint8_t *buffer)
{
    if (rx_queue == NULL || buffer == NULL) {
        return 0;
    }

    if (radio_message_queue_empty(rx_queue)) {
        return 0;
    }

    /* Dequeue the oldest message */
    radio_message_dequeue(rx_queue, buffer);

    /* Find the actual length (messages are null-padded) */
    uint8_t len = 0;
    while (len < RADIO_MAX_MESSAGE_LEN && buffer[len] != 0) {
        len++;
    }

    return len;
}

uint8_t radio_rx_count(void)
{
    if (rx_queue == NULL) {
        return 0;
    }

    if (radio_message_queue_empty(rx_queue)) {
        return 0;
    }

    return (rx_queue->head - rx_queue->tail + RADIO_MESSAGE_QUEUE_LEN) % RADIO_MESSAGE_QUEUE_LEN;
}

radio_message_queue_t *radio_get_rx_queue(void)
{
    return rx_queue;
}

/* ============================================================================
 * UART Callbacks (called from main.c)
 * ============================================================================ */

/**
 * @brief Called when IDLE line detected OR DMA transfer complete
 *
 * This is the main receive callback for DMA + IDLE mode.
 *
 * @param huart UART handle
 * @param Size Current position in DMA buffer
 */
void radio_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART5 || !s_initialized) {
        return;
    }

    /* During an AT session radio_at_poll() owns s_last_pos and the DMA
     * buffer. A stray 0x00 in the modem's reply must not let the message
     * parser consume bytes out from under it. */
    if (s_at_active) {
        return;
    }

    process_dma_data(Size);
}

/**
 * @brief Called on UART error - restarts circular DMA
 */
void radio_uart_error_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART5 || !s_initialized) {
        return;
    }

    /* Reset and restart circular DMA + CM */
    s_last_pos = 0;
    s_msg_len = 0;

    /* A framing/overrun error lands mid-message: clearing s_msg_len drops
     * the part already seen, but the remainder still arrives and would be
     * enqueued as a truncated frame. Resync to the next terminator so the
     * master never sees the fragment. */
    s_resync = true;

    HAL_UART_Receive_DMA(&huart5, s_dma_buf, RADIO_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_CM);
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Process new data in DMA buffer
 *
 * Handles circular buffer wrap-around and feeds bytes to message parser.
 *
 * @param new_pos Current DMA buffer position
 */
static void process_dma_data(uint16_t new_pos)
{
    uint16_t last = s_last_pos;
    uint16_t current = new_pos;

    if (current == last) {
        /* No new data */
        return;
    }

    if (current > last) {
        /* Simple case: no wrap-around */
        for (uint16_t i = last; i < current; i++) {
            feed_byte(s_dma_buf[i]);
        }
    } else {
        /* Wrap-around: process end of buffer, then start */
        for (uint16_t i = last; i < RADIO_DMA_BUF_SIZE; i++) {
            feed_byte(s_dma_buf[i]);
        }
        for (uint16_t i = 0; i < current; i++) {
            feed_byte(s_dma_buf[i]);
        }
    }

    s_last_pos = current;
}

/**
 * @brief Feed one byte to message accumulator
 *
 * Accumulates bytes until null terminator, then enqueues complete message.
 *
 * @param b Received byte
 */
static void feed_byte(uint8_t b)
{
    if (rx_queue == NULL) {
        return;
    }

    /* Post-session resync: throw away the modem's AT tail up to and
     * including the next terminator, so the first message we enqueue after
     * a session begins at a true frame boundary. */
    if (s_resync) {
        if (b == 0x00) {
            s_resync = false;
            s_msg_len = 0;
        }
        return;
    }

    if (b == 0x00) {
        /* Null terminator - end of message */
        if (s_msg_len > 0) {
            /* Enqueue the accumulated message */
            radio_message_enqueue(s_msg_len, s_msg_buf, rx_queue);

#ifdef DEBUG
            /* Log to debug console */
            debug_uart_log_radio(s_msg_buf, s_msg_len);
#endif

            /* Reset for next message */
            s_msg_len = 0;
            memset(s_msg_buf, 0, sizeof(s_msg_buf));
        }
    } else {
        /* Accumulate byte */
        if (s_msg_len < RADIO_MAX_MESSAGE_LEN) {
            s_msg_buf[s_msg_len] = b;
            s_msg_len++;
            s_msg_last_tick = HAL_GetTick();
        } else {
            /* Buffer overflow - discard and start over */
            s_msg_len = 0;
            memset(s_msg_buf, 0, sizeof(s_msg_buf));
        }
    }
}

/**
 * @brief Current DMA write position in the circular RX buffer
 *
 * CNDTR counts down from RADIO_DMA_BUF_SIZE as bytes land, so the write
 * index is the buffer size minus what remains.
 */
static uint16_t dma_write_pos(void)
{
    return (uint16_t)(RADIO_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart5.hdmarx));
}

/* ============================================================================
 * AT Passthrough Session
 * ============================================================================ */

void radio_rx_idle_check(void)
{
    if (!s_initialized || s_at_active || s_msg_len == 0) {
        return;
    }

    /* Unsigned subtraction, so this stays correct across HAL tick rollover */
    if ((HAL_GetTick() - s_msg_last_tick) > RADIO_MSG_IDLE_TIMEOUT_MS) {
#ifdef DEBUG
        debug_uart_log_rx_discard(s_msg_buf, s_msg_len);
#endif
        s_msg_len = 0;
        memset(s_msg_buf, 0, sizeof(s_msg_buf));
    }
}

void radio_at_session_begin(void)
{
    if (!s_initialized) {
        return;
    }

    /* Skip past anything already in the DMA buffer: bytes that arrived
     * before the session belong to normal radio traffic, not to the AT
     * exchange, and must not be parsed as a reply. */
    s_last_pos = dma_write_pos();
    s_msg_len = 0;

    s_at_rx_head = 0;
    s_at_rx_tail = 0;
    s_at_tx_head = 0;
    s_at_tx_tail = 0;

    s_at_last_activity = HAL_GetTick();
    s_at_active = true;

#ifdef DEBUG
    debug_uart_log_at_session(true);
#endif
}

void radio_at_touch(void)
{
    s_at_last_activity = HAL_GetTick();
}

void radio_at_session_end(void)
{
    if (!s_initialized) {
        return;
    }

    s_at_active = false;

    /* Drop AT residue already in the DMA buffer... */
    s_last_pos = dma_write_pos();
    s_msg_len = 0;

    /* ...and the tail still to arrive. The modem is mid-sentence (ATZ echo,
     * a trailing OK, reboot chatter) and none of it contains a 0x00, so
     * without this it would be prepended to the next real uplink frame and
     * break its COBS decode. Costs at most one message: the discarded one
     * is AT residue, not telemetry. */
    s_resync = true;

#ifdef DEBUG
    debug_uart_log_at_session(false);
#endif
}

bool radio_at_session_active(void)
{
    return s_at_active;
}

void radio_at_poll(void)
{
    if (!s_initialized || !s_at_active) {
        return;
    }

    /* Unsigned subtraction, so this stays correct across HAL tick rollover */
    if ((HAL_GetTick() - s_at_last_activity) > RADIO_AT_SESSION_TIMEOUT_MS) {
        radio_at_session_end();
        return;
    }

    uint16_t current = dma_write_pos();
    uint16_t last = s_last_pos;

#ifdef DEBUG
    uint8_t seen[64];
    uint16_t seen_n = 0;
#endif

    while (last != current) {
        uint16_t next_head = (uint16_t)((s_at_rx_head + 1u) & (AT_RX_RING_SIZE - 1u));

        if (next_head == s_at_rx_tail) {
            /* Ring full: the master is not draining fast enough. Drop the
             * new byte rather than the older ones -- the AT engine is
             * matching a reply prefix, so the head of the stream is what
             * carries meaning. */
            break;
        }

        s_at_rx_ring[s_at_rx_head] = s_dma_buf[last];
        s_at_rx_head = next_head;

#ifdef DEBUG
        if (seen_n < sizeof(seen)) {
            seen[seen_n++] = s_dma_buf[last];
        }
#endif

        last = (uint16_t)((last + 1u) & (RADIO_DMA_BUF_SIZE - 1u));
    }

    s_last_pos = last;

#ifdef DEBUG
    if (seen_n > 0) {
        debug_uart_log_at_rx(seen, seen_n);
    }
#endif
}

uint8_t radio_at_peek(uint8_t *dst, uint8_t max)
{
    if (dst == NULL) {
        return 0;
    }

    uint16_t tail = s_at_rx_tail;
    uint8_t n = 0;

    while (n < max && tail != s_at_rx_head) {
        dst[n++] = s_at_rx_ring[tail];
        tail = (uint16_t)((tail + 1u) & (AT_RX_RING_SIZE - 1u));
    }

    return n;
}

void radio_at_consume(uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        if (s_at_rx_tail == s_at_rx_head) {
            return;
        }
        s_at_rx_tail = (uint16_t)((s_at_rx_tail + 1u) & (AT_RX_RING_SIZE - 1u));
    }
}

bool radio_at_write(const uint8_t *data, uint8_t len)
{
    if (data == NULL) {
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        uint16_t next_head = (uint16_t)((s_at_tx_head + 1u) & (AT_TX_RING_SIZE - 1u));

        if (next_head == s_at_tx_tail) {
            return false; /* ring full; caller sees a short write */
        }

        s_at_tx_ring[s_at_tx_head] = data[i];
        s_at_tx_head = next_head;
    }

    return true;
}

void radio_at_flush(void)
{
    if (!s_initialized || !s_at_active) {
        return;
    }

    /* Snapshot head once: radio_at_write() may append from the SPI ISR
     * while this loop runs, and those bytes simply go out next pass. */
    uint16_t head = s_at_tx_head;
    uint8_t burst[AT_TX_RING_SIZE];
    uint16_t n = 0;

    while (s_at_tx_tail != head && n < sizeof(burst)) {
        burst[n++] = s_at_tx_ring[s_at_tx_tail];
        s_at_tx_tail = (uint16_t)((s_at_tx_tail + 1u) & (AT_TX_RING_SIZE - 1u));
    }

    if (n == 0) {
        return;
    }

    /* One contiguous write. "+++" has to reach the modem as an unbroken
     * burst -- three separate transmits would put gaps between the
     * characters, and no terminator or framing may be added around it. */
    HAL_UART_Transmit(&huart5, burst, n, 100);

#ifdef DEBUG
    debug_uart_log_at_tx(burst, n);
#endif
}
