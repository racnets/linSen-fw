#ifndef __CALIBRATION_H
#define __CALIBRATION_H

typedef struct {
	uint8_t p; 			/* p-factor */
	uint16_t max;		/* max average brightness setpoint */
	uint16_t min;		/* min average brightness setpoint */
	uint16_t setpoint;	/* average brightness setpoint */
	uint8_t hysterese;	/* tolerance for setpoint */
} exposure_calib_param_t;

extern exposure_calib_param_t exposure_calib_param;

void exposure_calibration(void);
void set_exposure_calibration_point(int value);
#endif /* __CALIBRATION_H */
