/**
 * @file Curves.hpp
 * @author Planeson, Red Bird Racing
 * @brief Definition of throttle and brake mapping tables
 * @version 1.2
 * @date 2026-02-07
 * @see Interp.hpp, Pedal
 */

#ifndef CURVES_HPP
#define CURVES_HPP
#include "Interp.hpp"
#include <stdint.h>

// === APPS Limits ===

constexpr uint16_t APPS_5V_MIN = 250;  /**< value below which apps_5v is considered shorted to ground */
constexpr uint16_t APPS_5V_MAX = 700; /**< value above which apps_5v is considered shorted to rail */

constexpr uint16_t APPS_3V3_MIN = 100;  /**< value below which apps_3v3 is considered shorted to ground */
constexpr uint16_t APPS_3V3_MAX = 500; /**< value above which apps_3v3 is considered shorted to rail */
constexpr uint16_t APPS_FINAL_MIN = APPS_5V_MIN; /**< final apps minimum value */
constexpr uint16_t APPS_FINAL_MAX = APPS_5V_MAX; /**< final apps maximum value */

/**
 * @brief Ratio between 5V APPS and 3.3V APPS, use integer math to avoid float operations.
 * Expanded to apps_scaled = apps_3v3 * APPS_RATIO
 * @note max 64 for multiply to prevent overflow
 */
#define APPS_RATIO 50 / 33

/**
 * @brief Throttle mapping table
 */
constexpr TablePoint<uint16_t, int16_t> THROTTLE_TABLE[5] = {
    {320, 0},
    {405, 2000},
    {490, 10000},
    {570, 25000},
    {650, 32500}}; // make sure this point doesn't exceed +-32767

// === Brake Limits ===

constexpr uint16_t brake_min = 80;  /**< value below which brake is considered shorted to ground */
constexpr uint16_t brake_max = 340; /**< value above which brake is considered shorted to rail */

/**
 * @brief Brake mapping table, negative values for regen
 */
constexpr TablePoint<uint16_t, int16_t> BRAKE_TABLE[5] = {
    {120, 0},
    {150, -15000},
    {180, -26000},
    {210, -31000},
    {240, -32500}}; // make sure this point doesn't exceed +-32767
#endif              // CURVES_HPP