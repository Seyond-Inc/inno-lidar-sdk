/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_COMMON_INNO_FAULTS_ROBINW_H_
#define SDK_COMMON_INNO_FAULTS_ROBINW_H_

/*RobinW fault list*/
enum InnoLidarRobinWInFault {
  INNO_LIDAR_ROBINW_IN_FAULT_OTHER = 0,
  INNO_LIDAR_ROBINW_IN_FAULT_POWER_LOW = 1,
  INNO_LIDAR_ROBINW_IN_FAULT_POWER_HIGH = 2,
  INNO_LIDAR_ROBINW_IN_FAULT_MAIN5V_OC = 3,
  INNO_LIDAR_ROBINW_IN_FAULT_POWSUP2 = 4,
  INNO_LIDAR_ROBINW_IN_FAULT_LASER_POWSUP = 5,
  INNO_LIDAR_ROBINW_IN_FAULT_WINDOW_BLOCKAGE1 = 6,
  INNO_LIDAR_ROBINW_IN_FAULT_WINDOW_BLOCKAGE2 = 7,
  INNO_LIDAR_ROBINW_IN_FAULT_LASER_INTERLOCK = 8,
  INNO_LIDAR_ROBINW_IN_FAULT_COVEROPEN = 9,
  INNO_LIDAR_ROBINW_IN_FAULT_POLYGON_SPEEDLOW = 10,
  INNO_LIDAR_ROBINW_IN_FAULT_LASER_STRONGCHARGE = 11,
  INNO_LIDAR_ROBINW_IN_FAULT_LASER_DISCHARGE = 12,
  INNO_LIDAR_ROBINW_IN_FAULT_CHANNEL_CONSISTENCY = 13,
  INNO_LIDAR_ROBINW_IN_FAULT_GROUP_CONSISTENCY = 14,
  INNO_LIDAR_ROBINW_IN_FAULT_COM_PMIC_TIMEOUT = 15,
  INNO_LIDAR_ROBINW_IN_FAULT_COM_PMIC_CRC = 16,
  INNO_LIDAR_ROBINW_IN_FAULT_LASER_SWITCH_STUCK_CLOSE = 17,
  INNO_LIDAR_ROBINW_IN_FAULT_POLYGON_SENSOR_PLAUSIBILITY = 18,
  INNO_LIDAR_ROBINW_IN_FAULT_POLYGON_HALL = 19,
  INNO_LIDAR_ROBINW_IN_FAULT_POLYGON_CONTROL = 20,
  INNO_LIDAR_ROBINW_IN_FAULT_POLYGON_STUCK = 21,
  INNO_LIDAR_ROBINW_IN_FAULT_POLYGON_PREDRIVE = 22,
  INNO_LIDAR_ROBINW_IN_FAULT_POLYGON_OC = 23,
  INNO_LIDAR_ROBINW_IN_FAULT_POLYGON_OVERDRIVE = 24,
  INNO_LIDAR_ROBINW_IN_FAULT_VBIAS_HIGH = 25,
  INNO_LIDAR_ROBINW_IN_FAULT_VBIAS_LOW = 26,
  INNO_LIDAR_ROBINW_IN_FAULT_EPHY = 27,
  INNO_LIDAR_ROBINW_IN_FAULT_DBTEMP = 28,
  INNO_LIDAR_ROBINW_IN_FAULT_LPDDR4 = 29,
  INNO_LIDAR_ROBINW_IN_FAULT_FLASH_FILE = 30,
  INNO_LIDAR_ROBINW_IN_FAULT_PTP_TIMEOUT = 31,
  INNO_LIDAR_ROBINW_IN_FAULT_PTPSYN = 32,
  INNO_LIDAR_ROBINW_IN_FAULT_PTPOFFSET = 33,
  INNO_LIDAR_ROBINW_IN_FAULT_MOTORSYN_FAULT = 34,
  INNO_LIDAR_ROBINW_IN_FAULT_SOC_EXTWD = 35,
  INNO_LIDAR_ROBINW_IN_FAULT_SOCPS_CLOCK = 36,
  INNO_LIDAR_ROBINW_IN_FAULT_SOCPL_CLOCK = 37,
  INNO_LIDAR_ROBINW_IN_FAULT_TRIGGER_RATE_HIGH = 38,
  INNO_LIDAR_ROBINW_IN_FAULT_TRIGGER_RATE_LOW = 39,
  INNO_LIDAR_ROBINW_IN_FAULT_SCATTER_NUMBER = 40,
  INNO_LIDAR_ROBINW_IN_FAULT_REF_DELAY = 41,
  INNO_LIDAR_ROBINW_IN_FAULT_REF_INTENSITY_LOW = 42,
  INNO_LIDAR_ROBINW_IN_FAULT_REF_INTENSITY_HIGH = 43,
  INNO_LIDAR_ROBINW_IN_FAULT_ETHERNET1 = 44,
  INNO_LIDAR_ROBINW_IN_FAULT_LASER_OVERHEAT1 = 45,
  INNO_LIDAR_ROBINW_IN_FAULT_LASER_OVERHEAT2 = 46,
  INNO_LIDAR_ROBINW_IN_FAULT_SOCTEMP1 = 47,
  INNO_LIDAR_ROBINW_IN_FAULT_PSSYSMON = 48,
  INNO_LIDAR_ROBINW_IN_FAULT_PMICTEMP1 = 49,
  INNO_LIDAR_ROBINW_IN_FAULT_PMICTEMP2 = 50,
  INNO_LIDAR_ROBINW_IN_FAULT_ASSERT_FAILURE = 51,
  INNO_LIDAR_ROBINW_IN_FAULT_CPULOAD_HIGH = 52,
  INNO_LIDAR_ROBINW_IN_FAULT_LATENCY_LONG = 53,
  INNO_LIDAR_ROBINW_IN_FAULT_DATA_DROP1 = 54,
  INNO_LIDAR_ROBINW_IN_FAULT_DATA_DROP2 = 55,
  INNO_LIDAR_ROBINW_IN_FAULT_RAWDATA_TO = 56,
  INNO_LIDAR_ROBINW_IN_FAULT_EXCESSIVE_NOISE = 57,
  INNO_LIDAR_ROBINW_IN_FAULT_SOCCHIP = 58,
  INNO_LIDAR_ROBINW_IN_FAULT_MODE_ENTER = 59,
  INNO_LIDAR_ROBINW_IN_FAULT_DATA_DROP3 = 60,
  INNO_LIDAR_ROBINW_IN_FAULT_DATA_DROP4 = 61,
  INNO_LIDAR_ROBINW_IN_FAULT_DATA_DROP5 = 62,
  INNO_LIDAR_ROBINW_IN_FAULT_CANBUSOFF = 63,
  INNO_LIDAR_ROBINW_IN_FAULT_CANCOMLOSS = 64,
  INNO_LIDAR_ROBINW_IN_FAULT_DMA_ERROR = 65,
  INNO_LIDAR_ROBINW_IN_FAULT_RPU = 66,
  INNO_LIDAR_ROBINW_IN_FAULT_PL_PCAP_ERROR = 67,
  INNO_LIDAR_ROBINW_IN_FAULT_RAWDATA_CRC = 68,
  INNO_LIDAR_ROBINW_IN_FAULT_REFNUM = 69,
  INNO_LIDAR_ROBINW_IN_FAULT_REFLASER = 70,
  INNO_LIDAR_ROBINW_IN_FAULT_ETH_SQI = 71,
  INNO_LIDAR_ROBINW_IN_FAULT_ETH_FRAMEDROP = 72,
  INNO_LIDAR_ROBINW_IN_FAULT_ETH_FCSERROR = 73,
  INNO_LIDAR_ROBINW_IN_FAULT_ADCSTUCK = 74,
  INNO_LIDAR_ROBINW_IN_FAULT_ADCCOM = 75,
  INNO_LIDAR_ROBINW_IN_FAULT_DBSENSOR = 76,
  INNO_LIDAR_ROBINW_IN_FAULT_ENCODER = 77,
  INNO_LIDAR_ROBINW_IN_FAULT_RESERVED78 = 78,
  INNO_LIDAR_ROBINW_IN_FAULT_RESERVED79 = 79,
  INNO_LIDAR_ROBINW_IN_FAULT_RESERVED80 = 80,
  INNO_LIDAR_ROBINW_IN_FAULT_RESERVED81 = 81,
  INNO_LIDAR_ROBINW_IN_FAULT_SECBOOT = 82,
  INNO_LIDAR_ROBINW_IN_FAULT_MAX = 83,
};

#endif  // SDK_COMMON_INNO_FAULTS_ROBINW_H_
