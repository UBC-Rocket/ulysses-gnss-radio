#ifndef GPS_FIX_QUEUE_H
#define GPS_FIX_QUEUE_H

#include "protocol_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define GPS_FIX_QUEUE_LEN 10  // Match existing GPS_SAMPLE_QUEUE_LEN

typedef struct {
    gps_fix_t fixes[GPS_FIX_QUEUE_LEN];
    volatile uint8_t head;
    volatile uint8_t tail;
} gps_fix_queue_t;

static inline void gps_fix_queue_init(gps_fix_queue_t *q) {
    q->head = 0;
    q->tail = 0;
}

static inline bool gps_fix_queue_empty(const gps_fix_queue_t *q) {
    return q->head == q->tail;
}

static inline bool gps_fix_queue_full(const gps_fix_queue_t *q) {
    return ((q->head + 1) % GPS_FIX_QUEUE_LEN) == q->tail;
}

static inline bool gps_fix_enqueue(const gps_fix_t *fix, gps_fix_queue_t *q) {
    if (gps_fix_queue_full(q)) return false;

    memcpy((void*)&q->fixes[q->head], (const void*)fix, sizeof(gps_fix_t));

    q->head = (q->head + 1) % GPS_FIX_QUEUE_LEN;

    return true;
}

static inline bool gps_fix_dequeue(gps_fix_t *fix, gps_fix_queue_t *q) {
    if (gps_fix_queue_empty(q)) return false;

    memcpy((void*)fix, (const void*)&q->fixes[q->tail], sizeof(gps_fix_t));

    q->tail = (q->tail + 1) % GPS_FIX_QUEUE_LEN;

    return true;
}

static inline bool gps_fix_peek(gps_fix_t *fix, const gps_fix_queue_t *q) {
    if (gps_fix_queue_empty(q)) return false;

    memcpy((void*)fix, (const void*)&q->fixes[q->tail], sizeof(gps_fix_t));

    return true;
}

static inline bool gps_fix_queue_tail_pointer(const gps_fix_queue_t *q, const gps_fix_t **tail_pointer) {
    if (gps_fix_queue_empty(q)) return false;

    *tail_pointer = &q->fixes[q->tail];

    return true;
}

static inline void gps_fix_queue_pop(gps_fix_queue_t *q) {
    if (gps_fix_queue_empty(q)) return;

    q->tail = (q->tail + 1) % GPS_FIX_QUEUE_LEN;
}

static inline uint8_t gps_fix_queue_length(const gps_fix_queue_t *q) {
    if (q->head >= q->tail) {
        return q->head - q->tail;
    } else {
        return GPS_FIX_QUEUE_LEN - q->tail + q->head;
    }
}

#endif // GPS_FIX_QUEUE_H
