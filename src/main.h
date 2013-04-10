#ifndef __MAIN_H
#define __MAIN_H

#define LIN_SEN_SYSID 	69
#define LIN_SEN_ID		1

/* exposure calibration */
#define ONCE	1
#define FOREVER	2

/* exposure calibration status */
#define RUNNING 1 
#define DONE	2

/* flags */
#define OFF				0x00
#define OPEN			0x01
#define MAVLINK			0x02
#define MAVLINK_ONCE	0x04
#define TERMINAL		0x08
#define LEDS			0x20
#define I2C				0x40
#define I2C_REQUEST		0x80
	
/* alog flag */
#define BINARY		0x80
#define DEBUG		0x40
#define BM_1		0x01

typedef struct {
	/* system identifier */
	uint8_t sys_id;
	/* unique identifier */
	uint8_t id;
	/* optical flow algorithm */
	/* OFF		0x00 */
	/* BM_1		0x01 */
	/* DEBUG 	0x40 */
	/* BINARY 	0x80 */
	uint8_t oflow_algo;
	/* parameter for optical flow algorithm */
	uint8_t oflow_algo_param;
	/* exposure calibration mode */
	/* OFF		0 */
	/* ONCE		1 */
	/* FOREVER	2 */
	uint8_t exposure_calib;
	/* exposure calibration status */
	/* OFF		0 */
	/* RUNNING	1 */
	/* DONE		2 */
	uint8_t exposure_calib_status;
	/* average brightness value and setpoint*/
	uint16_t average_brightness;
	/* average brightness calculation mode */
	/* OFF		0 */
	/* ONCE		1 */
	/* FOREVER	2 */
	uint8_t calc_average_brightness;
	/* USART communication mode */
	/* OFF		0x00 */
	/* OPEN		0x01 */
	/* MAVLINK	0x02 */
	/* TERMINAL	0x08 */
	uint8_t usart_modus;
	/* debug modus */
	/* OFF		0x00 */
	/* MAVLINK 	0x02 */
	uint8_t debug_modus;
	/* free to use debug value */
	uint16_t debug;
	/* result output mode */
	/* OFF		0x00 */
	/* MAVLINK	0x02 */
	/* LEDS		0x20 */
	/* I2C		0x40 */
	uint8_t result_output;
} config_t;

typedef union {
    config_t s;
    uint8_t a[sizeof(config_t)];
} config_u_t;    

extern config_u_t config;

/* result defines */
#define MAX_RESULT_VECTORS	16

typedef struct {
	uint8_t size;
	int16_t global;
	int16_t vector[MAX_RESULT_VECTORS];
	uint8_t overflow;
} result_t;

extern result_t linSen_result;

#endif /* __MAIN_H */
