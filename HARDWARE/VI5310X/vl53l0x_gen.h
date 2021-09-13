#ifndef __VL53L0X_GEN_H
#define __VL53L0X_GEN_H

#include "vl53l0x.h"
#include "main.h"

#if VL53L0X==1
extern VL53L0X_RangingMeasurementData_t vl53l0x_data;

extern vu16 Distance_data;//±£´æ²â¾àÊý¾Ý

VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode);
void vl53l0x_general_test(VL53L0X_Dev_t *dev);
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata,char *buf);

#endif

#endif


