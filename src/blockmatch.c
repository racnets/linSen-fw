#include <stdint.h>
#include <stdlib.h>
#include <math.h> // pow

#include "main.h"
#include "blockmatch.h"
#include "usart1.h"
#include "tsl1401.h"

#define INT_MAX	2147483647

void blockMatch(volatile uint16_t *now, volatile uint16_t *last, int size, int search_radius, int block_size, result_t *result) {
	/* tmp */
	int min, diff;
	int s, x;
	int b = 0;
	int i = 0;
	/* output */
	int aktShift;
	int gshift_a[2*MAX_SEARCH_RADIUS+1] = {};
	int* gshift = &gshift_a[MAX_SEARCH_RADIUS];

	/* calculate sector shift */
	while (b < size) {
		aktShift = 0;
		min = INT_MAX;
		for (s = -search_radius; s <= search_radius; ++s) {
			if ((b + s >= 0) && (b + block_size + s <= size)) {
				diff = 0;
				for (x = b; x < b + block_size; ++x) {
					diff += pow(now[x] - last[x + s], 2);
				}
				if (config.s.oflow_algo & DEBUG) {
					printInt32(diff);
					printChar('\t');
				}
				if (diff < min)	{
					min = diff;
					aktShift = s;
				}
				gshift[s] += diff;
			}
		}
		result->vector[i++] = aktShift;
		b += block_size;
	}
	
	/* calculate global shift */	
	min = INT_MAX;
	for (s = -search_radius; s <= search_radius; ++s) {
		if (gshift[s] < min) {
			min = gshift[s];
			aktShift = s;
		}
	}
	
	result->global = aktShift;// / size;
	result->size = size;
}

int crossSection(volatile uint16_t *data, int size) {
	int sum = 0;
	int i;
	for (i = 0; i < size; i++) {
		sum += data[i];
	}

	return sum / size;
}

void makeBinary(volatile uint16_t *data, int size, int average_brightness) {
	int i;
	for (i = 0; i < size; i++) {
		data[i] = (data[i] > average_brightness)? 1 : 0;
	}
}

void printBuffer(volatile uint16_t *data, int start, int size) {
	int i;
	for (i = start; i < start + size; i++) {
		printChar('\t');
		printInt32(data[i]);
	}					
	printChar('\n');		
}
