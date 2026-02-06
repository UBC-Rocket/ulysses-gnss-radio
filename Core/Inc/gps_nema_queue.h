#ifndef GPS_QUEUE_H
#define GPS_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define GPS_SAMPLE_SIZE 87  // Max NMEA sentence length (82 chars + 5 byte margin)
#define GPS_SAMPLE_QUEUE_LEN 10

typedef struct {
    uint8_t samples[GPS_SAMPLE_QUEUE_LEN][GPS_SAMPLE_SIZE];

    volatile uint8_t head;
    volatile uint8_t tail;
} gps_sample_queue_t;

static inline void gps_sample_queue_init(gps_sample_queue_t *q) {
    q->head = 0;
    q->tail = 0;
}

static inline bool gps_sample_queue_empty(gps_sample_queue_t *q) {
    return q->head == q->tail;
}

static inline bool gps_sample_queue_full(gps_sample_queue_t *q) {
    return ((q->head + 1) % GPS_SAMPLE_QUEUE_LEN) == q->tail;
}

static inline bool gps_sample_enqueue(uint8_t *data, gps_sample_queue_t *q) {
    if (gps_sample_queue_full(q)) return false;

    memcpy((void*)q->samples[q->head], (void*)data, GPS_SAMPLE_SIZE);

    q->head = (q->head + 1) % GPS_SAMPLE_QUEUE_LEN;

    return true;
}

static inline bool gps_sample_dequeue(gps_sample_queue_t *q, uint8_t *data) {
    if (gps_sample_queue_empty(q)) return false;

    memcpy((void*)data, (void*)q->samples[q->tail], GPS_SAMPLE_SIZE);

    q->tail = (q->tail + 1) % GPS_SAMPLE_QUEUE_LEN;

    return true;
}

static inline bool gps_sample_queue_tail_pointer(gps_sample_queue_t *q, uint8_t **tail_pointer) {
    if (gps_sample_queue_empty(q)) return false;

    *tail_pointer = q->samples[q->tail];

    return true;
}

static inline void gps_sample_queue_pop(gps_sample_queue_t *q) {
    if (gps_sample_queue_empty(q)) return;

    q->tail = (q->tail + 1) % GPS_SAMPLE_QUEUE_LEN;
}

#endif
