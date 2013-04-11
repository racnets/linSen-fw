#ifndef __BLOCKMATCH_H
#define __BLOCKMATCH_H

#define MAX_SEARCH_RADIUS	32
void blockMatch(volatile uint16_t *now, volatile uint16_t *last, int size, int block_size, int sectors, result_t *result);
int crossSection(volatile uint16_t *data, int size);
void makeBinary(volatile uint16_t *data, int size, int average_brightness);
void printBuffer(volatile uint16_t *data, int start, int size);

#endif /* __BLOCKMATCH_H */
