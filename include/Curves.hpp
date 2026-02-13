/**
 * @file Curves.hpp
 * @author Planeson, Red Bird Racing
 * @brief Definition of throttle and brake mapping tables
 * @version 1.5
 * @date 2026-02-09
 * @see Interp.hpp, Pedal
 */

#ifndef CURVES_HPP
#define CURVES_HPP
#include "Interp.hpp"
#include <stdint.h>

// === APPS Limits ===

constexpr uint16_t APPS_5V_MIN = 50;  /**< value below which apps_5v is considered shorted to ground */
constexpr uint16_t APPS_5V_MAX = 950; /**< value above which apps_5v is considered shorted to rail */

constexpr uint16_t APPS_3V3_MIN = 50;            /**< value below which apps_3v3 is considered shorted to ground */
constexpr uint16_t APPS_3V3_MAX = 950;           /**< value above which apps_3v3 is considered shorted to rail */

/**
 * @brief Ratio between 5V APPS and 3.3V APPS, use integer math to avoid float operations.
 * Expanded to apps_scaled = apps_3v3 * APPS_RATIO
 * @note max 64 for multiply to prevent overflow
 */
#define APPS_RATIO 53 / 34

/**
 * @brief Throttle mapping table
 */
constexpr TablePoint<uint16_t, int16_t> CURVE_TABLE[5] = {
    {0, 0},
    {15000, 2000},
    {30000, 10000},
    {45000, 25000},
    {60000, 32500}}; // make sure this point doesn't exceed +-32767

// === Brake Limits ===

constexpr uint16_t brake_min = 50;  /**< value below which brake is considered shorted to ground */
constexpr uint16_t brake_max = 950; /**< value above which brake is considered shorted to rail */

/**
 * @brief Brake mapping table, negative values for regen
 */
constexpr TablePoint<uint16_t, int16_t> BRAKE_TABLE[5] = {
    {120, 0},
    {150, -15000},
    {180, -26000},
    {210, -31000},
    {240, -32500}}; // make sure this point doesn't exceed +-32767

/**
 * @brief APPS_3V3 mapping table, maps 3V3 readings to 5V readings
 */
constexpr TablePoint<uint16_t, uint16_t> APPS_3V3_SCALE_TABLE[2] = {
    {220, 325},
    {410, 621}};

/**
 * @brief APPS_5V to percent mapping table, maps 5V readings to percent throttle (0-60000) 
 */
constexpr TablePoint<uint16_t, uint16_t> APPS_5V_PERCENT_TABLE[2] = {
    {325, 0},
    {621, 60000}};

// === calculated tables
/**
 * @brief APPS percent mapping table, maps APPS percentage to 5V readings
 * @see THROTTLE_TABLE, APPS_5V_TABLE_INVERTED_MAP
 */
constexpr TablePoint<uint16_t, uint16_t> APPS_5V_TABLE_INVERTED[2] = {
    {APPS_5V_PERCENT_TABLE[0].out, APPS_5V_PERCENT_TABLE[0].in},
    {APPS_5V_PERCENT_TABLE[1].out, APPS_5V_PERCENT_TABLE[1].in}};

/**
 * @brief LinearInterp for APPS_5V_TABLE_INVERTED, used to get APPS reading from a desired percentage
 * @see THROTTLE_TABLE, APPS_5V_TABLE_INVERTED
 */
constexpr LinearInterp<uint16_t, uint16_t, uint32_t, 2> APPS_5V_TABLE_INVERTED_MAP{APPS_5V_TABLE_INVERTED};

/**
 * @brief Throttle mapping table (calculated), maps APPS_5V readings to torque values
 * @see CURVE_TABLE, APPS_5V_PERCENT_TABLE
 */
constexpr TablePoint<uint16_t, int16_t> THROTTLE_TABLE[5] = {
    {APPS_5V_TABLE_INVERTED_MAP.interp(CURVE_TABLE[0].in), CURVE_TABLE[0].out},
    {APPS_5V_TABLE_INVERTED_MAP.interp(CURVE_TABLE[1].in), CURVE_TABLE[1].out},
    {APPS_5V_TABLE_INVERTED_MAP.interp(CURVE_TABLE[2].in), CURVE_TABLE[2].out},
    {APPS_5V_TABLE_INVERTED_MAP.interp(CURVE_TABLE[3].in), CURVE_TABLE[3].out},
    {APPS_5V_TABLE_INVERTED_MAP.interp(CURVE_TABLE[4].in), CURVE_TABLE[4].out}};

#endif // CURVES_HPP