#ifndef TIMING_UTILS_H
#define TIMING_UTILS_H

#include <stdint.h>

/**
 * @brief Initialize microsecond timing subsystem
 *
 * Configures a hardware timer (TIM) to provide microsecond resolution timing.
 * Must be called once during system initialization before using timing functions.
 */
void timing_init(void);

/**
 * @brief Get current time in microseconds
 *
 * @return uint32_t Current timestamp in microseconds (wraps around at ~71.5 minutes)
 *
 * @note Uses a 32-bit counter, so timestamps will wrap. For collision detection
 *       (t_race ~15us), this is not an issue.
 */
uint32_t get_time_us(void);

/**
 * @brief Calculate elapsed time in microseconds between two timestamps
 *
 * @param start_us Start timestamp (from get_time_us())
 * @param end_us End timestamp (from get_time_us())
 * @return uint32_t Elapsed time in microseconds
 *
 * @note Handles wraparound correctly for typical use cases
 */
static inline uint32_t elapsed_us(uint32_t start_us, uint32_t end_us) {
    return end_us - start_us;  // Unsigned math handles wraparound
}

/**
 * @brief Delay for a specified number of microseconds (blocking)
 *
 * @param delay_us Delay duration in microseconds
 *
 * @warning This is a BLOCKING delay. Use sparingly, especially in ISR context.
 */
void delay_us(uint32_t delay_us);

#endif // TIMING_UTILS_H
