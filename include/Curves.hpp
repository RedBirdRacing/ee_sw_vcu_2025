/**
 * @file Curves.hpp
 * @author Planeson, Red Bird Racing
 * @brief Definition of throttle and brake mapping tables
 * @version 1.0
 * @date 2026-01-15
 * @see Interp.hpp, Pedal
 */

#ifndef CURVES_HPP
#define CURVES_HPP
#include "Interp.hpp"
#include <stdint.h>

// === APPS Limits ===

constexpr uint16_t apps_min = 30;  /**< value below which apps is considered shorted to ground */
constexpr uint16_t apps_max = 950; /**< value above which apps is considered shorted to rail */
/** @brief Throttle mapping table */
const TablePoint<uint16_t, int16_t> throttleTable[5] = {
    {60, 0},
    {200, 2000},
    {450, 10000},
    {700, 25000},
    {900, 32500}};

// === Brake Limits ===

constexpr uint16_t brake_min = 30;  /**< value below which brake is considered shorted to ground */
constexpr uint16_t brake_max = 950; /**< value above which brake is considered shorted to rail */
/** @brief Brake mapping table, negative for regen */
const TablePoint<uint16_t, int16_t> brakeTable[5] = {
    {60, 0},
    {250, -15000},
    {500, -26000},
    {750, -31000},
    {900, -32500}};
#endif // CURVES_HPP