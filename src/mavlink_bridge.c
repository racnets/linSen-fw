#include <huch/mavlink.h>

#include "mavlink_bridge.h"
#include "main.h"
#include "calibration.h"
#include "tsl1401.h"
#include "usart1.h"
#include "led.h"
#include "time.h"

#define LIN_SEN_COMPID	0

volatile int mavlinkCtrl = 0;
volatile int mavlinkArg = 0;

mavlink_message_t mav_msg_tx;

mavlink_status_t mav_status;

/* function to send a mavlink control message over USART1 using DMA */
int mavlinkSendCtrl(uint8_t cmd, uint16_t arg) {
	if (USART1DMAReady()) {
		
		/* setup message */
		mavlink_msg_huch_lin_sen_ctrl_pack(
			LIN_SEN_SYSID,
			LIN_SEN_COMPID,
			&mav_msg_tx,
			get_time(),
			cmd,
			arg
		);
		
		/* send over usart1 with dma */
		sendUSART1DMA((uint32_t)&(mav_msg_tx.magic), MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)mav_msg_tx.len);
		
		return 1;
	} else return 0;
}

/* function to send a mavlink linSen raw data message over USART1 using DMA */
int mavlinkSendLinRaw(volatile uint16_t *data) {
	uint8_t *mav_msg_raw;
#if MAVLINK_CRC_EXTRA
	static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;
#endif
	
	if (USART1DMAReady()) {
		int i = 4;
		int k = 0;
		
		// timestamp in microseconds
		_mav_put_uint32_t(mav_msg_tx.payload64, 0, get_time()); 
		/* add raw data to message payload */
		/* and pack 4 12bit values into 3 16bit */
		/* and send these little endian as 3 x 2 8bit */
		mav_msg_raw = (uint8_t *)mav_msg_tx.payload64;
		for (; k < MAVLINK_MSG_HUCH_LIN_SEN_RAW_FIELD_DATA_LEN * 4 / 3;) {
			mav_msg_raw[i++] = ((data[k] << 4 | data[k+1] >> 8)) & 0xff;
			mav_msg_raw[i++] = (data[k] >> 4) & 0xff;
			mav_msg_raw[i++] = ((data[k+1] << 8 | data[k+2] >> 4)) & 0xff;
			mav_msg_raw[i++] = (data[k+1] ) & 0xff;
			mav_msg_raw[i++] = ((data[k+2] << 12 | data[k+3])) & 0xff;
			mav_msg_raw[i++] = (data[k+2] << 4 | data[k+3] >> 8) & 0xff;

			k += 4;
		}
		
		mav_msg_tx.msgid = MAVLINK_MSG_ID_HUCH_LIN_SEN_RAW;

		mavlink_finalize_message(&mav_msg_tx, LIN_SEN_SYSID, LIN_SEN_COMPID, MAVLINK_MSG_ID_HUCH_LIN_SEN_RAW_LEN, mavlink_message_crcs[MAVLINK_MSG_ID_HUCH_LIN_SEN_RAW]);

		/* send over usart1 with dma */
		sendUSART1DMA((uint32_t)&(mav_msg_tx.magic), MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)mav_msg_tx.len);

		return 1;
	} else return 0;
}

/* function to send a mavlink quadPix raw data message over USART1 using DMA */
int mavlinkSendQuadRaw(volatile uint32_t *data) {
	uint32_t *mav_msg_raw;
#if MAVLINK_CRC_EXTRA
	static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;
#endif

	if (USART1DMAReady()) {
		int i = 1;
		int k = 0;

		// timestamp in microseconds
		_mav_put_uint32_t(mav_msg_tx.payload64, 0, get_time());
		mav_msg_raw = (uint32_t *)mav_msg_tx.payload64;
		/* add raw data to message payload */
		for (; k < 10;) {
			mav_msg_raw[i++] = data[k++];
		}

		mav_msg_tx.msgid = MAVLINK_MSG_ID_HUCH_QUAD_PIX_RAW;

		mavlink_finalize_message(&mav_msg_tx, LIN_SEN_SYSID, LIN_SEN_COMPID, MAVLINK_MSG_ID_HUCH_QUAD_PIX_RAW_LEN, mavlink_message_crcs[MAVLINK_MSG_ID_HUCH_QUAD_PIX_RAW]);

		/* send over usart1 with dma */
		sendUSART1DMA((uint32_t)&(mav_msg_tx.magic), MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)mav_msg_tx.len);

		return 1;
	} else return 0;
}

/* function to send a mavlink optical flow data message over USART1 using DMA */
int mavlinkSendOpticalFlow(int16_t flow_x, int16_t flow_y, uint8_t quality) {
	if (USART1DMAReady()) {
		
		/* setup message */
		mavlink_msg_optical_flow_pack(
			LIN_SEN_SYSID,
			LIN_SEN_COMPID,
			&mav_msg_tx,
			get_time(),
			LIN_SEN_ID,
			flow_x,
			flow_y,
			0,
			0,
			quality,
			-1
		);
		
		/* send over usart1 with dma */
		sendUSART1DMA((uint32_t)&(mav_msg_tx.magic), MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)mav_msg_tx.len);
		
		return 1;
	} else return 0;
}

/* process incoming mavlink message */
void processMavlink() {
	static mavlink_message_t mav_msg_rx;
	uint8_t rx_byte;
	
	while (usart1_rx_get_data_available(&rx_byte)) {
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &mav_msg_rx, &mav_status)) {
			// set usart to exclusiv mavlink mode
			if (config.s.usart_modus == OPEN) config.s.usart_modus = MAVLINK;
			
			// Handle message
	 		switch(mav_msg_rx.msgid) {
				case MAVLINK_MSG_ID_HUCH_LIN_SEN_CTRL: {
					mavlink_huch_lin_sen_ctrl_t ctrl;
					mavlink_msg_huch_lin_sen_ctrl_decode(&mav_msg_rx, &ctrl);

					if (!mavlinkCtrl) {
						mavlinkCtrl = ctrl.cmd;
						mavlinkArg = ctrl.arg;
					}
					break;
				}
				default:
					//Do nothing
					break;
			}
		}
	}
}

void handleMavlink() {
	/* mavlink control handling */
	if (mavlinkCtrl) {
		switch (mavlinkCtrl) {
			case LIN_SEN_DEBUG_RAW_LINE: {
				if (mavlinkArg) config.s.debug_modus |= MAVLINK;
				else config.s.debug_modus &= ~MAVLINK;
				mavlinkCtrl = 0;
			    break;
			}
			case LIN_SEN_LED_BLUE: {
				if (mavlinkArg) ledBlueOn();
				else ledBlueOff();
				mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_LED_GREEN: {
				if (mavlinkArg) ledGreenOn();
				else ledGreenOff();
				mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_AVG_GET: {
				config.s.calc_average_brightness |= ONCE;
				if (mavlinkSendCtrl(LIN_SEN_AVG_GET, config.s.average_brightness)) mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_AVG_SET: {
				if (config.s.exposure_calib == OFF) config.s.exposure_calib = ONCE;
				exposure_calib_param.setpoint = mavlinkArg;
				mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_EXPOSURE_GET: {
				if (mavlinkSendCtrl(LIN_SEN_EXPOSURE_GET, getExposureValue())) mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_EXPOSURE_SET: {
				setExposureValue(mavlinkArg);
				mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_PIX_CLK_GET: {
				if (mavlinkSendCtrl(LIN_SEN_PIX_CLK_GET, getPixelClock())) mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_PIX_CLK_SET: {
				setPixelClock(mavlinkArg);
				mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_GAIN_GET: {
				//TODO for later use
				if (mavlinkSendCtrl(LIN_SEN_GAIN_GET, 0)) mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_GAIN_SET: {
				//TODO for later use
				config.s.debug = mavlinkArg;
				mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_ALGO_GET: {
				if (mavlinkSendCtrl(LIN_SEN_ALGO_GET, config.s.oflow_algo)) mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_ALGO_SET: {
				config.s.oflow_algo = mavlinkArg;
				mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_DEBUG: {
				if (mavlinkSendCtrl(LIN_SEN_DEBUG, config.s.debug)) mavlinkCtrl = 0;
				break;
			}
			case LIN_SEN_RESULT: {
				if (mavlinkArg)	config.s.result_output |= MAVLINK;
				else config.s.result_output &= ~MAVLINK;				
				mavlinkCtrl = 0;
				break;
			}
			default:
				mavlinkCtrl = 0;
			}
	}
}
