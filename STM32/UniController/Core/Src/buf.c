/*
 * buf.c
 *
 *  Created on: Mar 8, 2023
 *      Author: jonathanloov
 */

#include <stdint.h>

char *write_hex(char *dest, uint8_t val) {
	uint8_t lo = val & 0xf;
	uint8_t hi = val >> 4;
	if (hi < 10) {
		dest[0] = hi + '0';
	} else {
		dest[0] = hi - 10 + 'a';
	}

	if (lo < 10) {
		dest[1] = lo + '0';
	} else {
		dest[1] = lo - 10 + 'a';
	}

	return dest + 2;
}
