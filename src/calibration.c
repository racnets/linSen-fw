#include "misc.h"
#include "calibration.h"

#include "main.h"
#include "tsl1401.h"
#include "usart1.h"
#include "blockmatch.h"
#include "led.h"

/* auto calibration values */
#define RUNS 8

exposure_calib_param_t exposure_calib_param = {
	2,
	2000,
	200,
	500,
	10
};

/* 
 * auto exposure calibration 
 * try to achive the value given with set_exposure_calibration_point
 * return RUNNING if not yet ready
 * return DONE if succeed
 */
void exposure_calibration() {
	static int count = RUNS;
	static int calib_value = 0;
	static int equal_count = 0;

	if ((config.s.average_brightness < exposure_calib_param.min) || (config.s.average_brightness > exposure_calib_param.max) || (config.s.exposure_calib == ONCE)) {
		/* first dataset is mostly dirty because of exposure change */
		/* low pass the average */
		/* caution low pass works as p factor too */
		if (count != RUNS) calib_value += exposure_calib_param.setpoint - config.s.average_brightness;
		
		if (!(count--)) {
			/* get current exposure value */
			int _exposure = getExposureValue();
			
			/* calculate new exposure using a P filter */
			if ((calib_value > exposure_calib_param.hysterese) || (calib_value < -exposure_calib_param.hysterese)) _exposure += exposure_calib_param.p * calib_value;
			else equal_count++;
			
			/* limit exposure to constraints */
			if (_exposure > exposure_range.max_exposure) {
				_exposure = exposure_range.max_exposure;
				equal_count++;
			}
			if (_exposure < exposure_range.min_exposure) {
				_exposure = exposure_range.min_exposure;
				equal_count++;
			}
			/* set new exposure value */
			setExposureValue(_exposure);

			count = RUNS;
			calib_value = 0;
			/* check if enough equal conditions occured */
			if (equal_count > RUNS) {
				equal_count = 0;
				/* check running modus */
				if (config.s.exposure_calib == ONCE) config.s.exposure_calib = OFF;
				config.s.exposure_calib_status = DONE;
			} else config.s.exposure_calib_status = RUNNING;
		}
	}
	
}
