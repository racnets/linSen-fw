#include <stdint.h>
#include <stdlib.h>

#include "main.h"
#include "blockmatch.h"
#include "usart1.h"
#include "tsl1401.h"

#define INT_MAX	2147483647

void blockMatch(volatile uint16_t *now, volatile uint16_t *last, int size, int search_radius, int sectors, result_t *result) {
	/* tmp */
	int min = INT_MAX;
	int diff = 0;
	int s, i, x;
	/* output */
	static int aktShift = 0;
	
	//int pixel_per_sector = size / sectors;

//	for (s = 0; s < sectors; ++s) {
		for (i = -search_radius; i <= search_radius; i++) {
			diff = 0;
			for (x = search_radius; x < (size - search_radius); x++)	{
				diff += abs(now[x] - last[x + i]);
			}
			if (config.s.oflow_algo & DEBUG) {
				printInt32(diff);
				printChar('\t');
			}
			if (diff < min)	{
				min = diff;
				aktShift = i;
			}
		}
//	}
	result->size = 1;
	result->vector[0] = aktShift;
	result->global = aktShift;
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
