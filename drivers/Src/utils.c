/*!
 * @file utils.c
 * @brief
 * @date April 18, 2020
 */

#include "utils.h"

void SetBit(volatile uint32_t *reg, uint8_t bitNumber)
{
	*reg |= (1 << bitNumber);
}

void ResetBit(volatile uint32_t *reg, uint8_t bitNumber)
{
	*reg &= ~(1 << bitNumber);
}
