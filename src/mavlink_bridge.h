#ifndef MAVLINK_BRIDGE_H_
#define MAVLINK_BRIDGE_H_

#include <huch/mavlink.h>
extern mavlink_system_t mavlink_system;

void processMavlink(void);
void handleMavlink(void);

int mavlinkSendRaw(volatile uint16_t *data);
int mavlinkSendCtrl(uint8_t cmd, uint16_t arg);
int mavlinkSendOpticalFlow(int16_t flow_x, int16_t flow_y, uint8_t quality);

#endif //MAVLINK_BRIDGE_H
