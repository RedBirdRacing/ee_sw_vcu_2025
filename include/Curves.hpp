/**
 * @file Curves.hpp
 * @author Planeson, Red Bird Racing
 * @brief Definition of throttle and brake mapping tables
 * @version 1.3
 * @date 2026-02-04
 * @see Interp.hpp, Pedal
 */

#ifndef CURVES_HPP
#define CURVES_HPP
#include "Interp.hpp"
#include <stdint.h>

// === APPS Limits ===

constexpr uint16_t APPS_5V_MIN = 250; /**< value below which apps_5v is considered shorted to ground */
constexpr uint16_t APPS_5V_MAX = 700; /**< value above which apps_5v is considered shorted to rail */

constexpr uint16_t APPS_3V3_MIN = 135;           /**< value below which apps_3v3 is considered shorted to ground */
constexpr uint16_t APPS_3V3_MAX = 500;           /**< value above which apps_3v3 is considered shorted to rail */
constexpr uint16_t APPS_FINAL_MIN = APPS_5V_MIN; /**< final apps minimum value */
constexpr uint16_t APPS_FINAL_MAX = APPS_5V_MAX; /**< final apps maximum value */

/**
 * @brief Ratio between 5V APPS and 3.3V APPS, use integer math to avoid float operations.
 * Expanded to apps_scaled = apps_3v3 * APPS_RATIO
 * @note max 64 for multiply to prevent overflow
 */
#define APPS_RATIO 53 / 34

/**
 * @brief Throttle mapping table
 */
const TablePoint<uint16_t, int16_t> THROTTLE_TABLE[5] = {
    {320, 0},
    {405, 2000},
    {490, 10000},
    {570, 25000},
    {650, 32500}}; // make sure this point doesn't exceed +-32767

// === Brake Limits ===

constexpr uint16_t brake_min = 30;  /**< value below which brake is considered shorted to ground */
constexpr uint16_t brake_max = 950; /**< value above which brake is considered shorted to rail */

/**
 * @brief Brake mapping table, negative values for regen
 */
const TablePoint<uint16_t, int16_t> BRAKE_TABLE[5] = {
    {120, 0},
    {250, -15000},
    {500, -26000},
    {750, -31000},
    {900, -32500}}; // make sure this point doesn't exceed +-32767

const TablePoint<uint16_t, int16_t> APPS_TABLE[5] = {
    {100, 200},
    {220, 300},
    {300, 420},
    {350, 500},
    {500, 750}};

#endif // CURVES_HPP