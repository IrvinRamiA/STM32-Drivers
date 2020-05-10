/*!
 * @file utils.h
 * @brief
 * @date April 18, 2020
 */

#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

void SetBit(volatile uint32_t *reg, uint8_t bitNumber);
void ResetBit(volatile uint32_t *reg, uint8_t bitNumber);

#endif
