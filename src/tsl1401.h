#ifndef __TSL1401_H
#define __TSL1401_H

typedef struct {
	int max_exposure;
	int min_exposure;
} exposure_range_t;
extern exposure_range_t exposure_range;

/* datasets */
#define TSL1401PIXELCOUNT 128
extern volatile uint16_t *tsl1401_dataset;
extern volatile uint16_t *tsl1401_dataset_old;

/* flags */
extern volatile int tsl1401_data_transfer_complete;

/* functions */
void tsl1401_init(int exposure, int pixel_clock);
void tsl1401_toggle_buffer(void);
int getExposureValue(void);
void setExposureValue(int value);
void setPixelClock(int value);
int getPixelClock();

#endif /* __TSL1401_H */
