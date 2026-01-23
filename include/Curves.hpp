/**
 * @file Curves.hpp
 * @author Planeson, Red Bird Racing
 * @brief Definition of throttle and brake mapping tables
 * @version 1.1
 * @date 2026-01-15
 * @see Interp.hpp, Pedal
 */

#ifndef CURVES_HPP
#define CURVES_HPP
#include "Interp.hpp"
#include <stdint.h>

// === APPS Limits ===

constexpr uint16_t apps_5v_min = 50;  /**< value below which apps_5v is considered shorted to ground */
constexpr uint16_t apps_5v_max = 950; /**< value above which apps_5v is considered shorted to rail */

constexpr uint16_t apps_3v3_min = 30;  /**< value below which apps_3v3 is considered shorted to ground */
constexpr uint16_t apps_3v3_max = 600; /**< value above which apps_3v3 is considered shorted to rail */

constexpr uint16_t apps_final_min = apps_5v_min;
constexpr uint16_t apps_final_max = apps_5v_max;
/**
 * @brief Throttle mapping table
 */
const TablePoint<uint16_t, int16_t> throttle_table[5] = {
    {60, 0},
    {200, 2000},
    {450, 10000},
    {700, 25000},
    {900, 32500}}; // make sure this point doesn't exceed +-32767

// === Brake Limits ===

constexpr uint16_t brake_min = 30;  /**< value below which brake is considered shorted to ground */
constexpr uint16_t brake_max = 950; /**< value above which brake is considered shorted to rail */

/**
 * @brief Brake mapping table, negative values for regen
 */
const TablePoint<uint16_t, int16_t> brake_table[5] = {
    {60, 0},
    {250, -15000},
    {500, -26000},
    {750, -31000},
    {900, -32500}}; // make sure this point doesn't exceed +-32767
#endif // CURVES_HPP