#ifndef RADIO_DRIVER_H
#define RADIO_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

// Initialize radio driver (call once at startup)
void radio_init(void);

// Send data to ground station
// Returns: true if sent, false if invalid length
bool radio_send(uint8_t *data, uint8_t len);

// Check if message available
bool radio_available(void);

// Read received message
// Returns: message length (0 if none available)
uint8_t radio_read(uint8_t *buffer);

#endif