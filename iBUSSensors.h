/*
 * iBUSTelemetry.cpp - adis1313
 *
 * Libraries or parts of the code used in this project:
 *
 * SoftwareSerialWithHalfDuplex
 * https://github.com/nickstedman/SoftwareSerialWithHalfDuplex
 *
 * iBUStelemetry
 * https://github.com/Hrastovc/iBUStelemetry
 *
 * FlySkyI6
 * https://github.com/qba667/FlySkyI6/blob/master/source/source/ibustelemetry.h
 *
 * Big thanks to the authors!
 */

#ifndef iBUSSensors_h
#define iBUSSensors_h

#define MAX_SENS_COUNT 15

#define STAB           0
#define ACRO           1
#define AHOLD          2
#define AUTO           3
#define GUIDED         4
#define LOITER         5
#define RTL            6
#define CIRCLE         7
#define PHOLD          8
#define LAND           9

#define UNARMED        0
#define ARMED          1

// 2 byte sensors

#define IBUS_MEAS_TYPE_TEM            0x01 // Temperature
#define IBUS_MEAS_TYPE_EXTV           0x03 // External voltage
#define IBUS_MEAS_TYPE_CELL           0x04 // Avg cell voltage
#define IBUS_MEAS_TYPE_BAT_CURR       0x05 // Battery current
#define IBUS_MEAS_TYPE_FUEL           0x06 // Remaining battery percentage
#define IBUS_MEAS_TYPE_RPM            0x07 // Throttle value / battery capacity
#define IBUS_MEAS_TYPE_CMP_HEAD       0x08 // Heading
#define IBUS_MEAS_TYPE_CLIMB_RATE     0x09 // Climb rate
#define IBUS_MEAS_TYPE_COG            0x0a // Course over ground
#define IBUS_MEAS_TYPE_GPS_STATUS     0x0b // GPS status (2 values)
#define IBUS_MEAS_TYPE_ACC_X          0x0c // Acc X
#define IBUS_MEAS_TYPE_ACC_Y          0x0d // Acc Y
#define IBUS_MEAS_TYPE_ACC_Z          0x0e // Acc Z
#define IBUS_MEAS_TYPE_ROLL           0x0f // Roll
#define IBUS_MEAS_TYPE_PITCH          0x10 // Pitch
#define IBUS_MEAS_TYPE_YAW            0x11 // Yaw
#define IBUS_MEAS_TYPE_VERTICAL_SPEED 0x12 // Vertical speed
#define IBUS_MEAS_TYPE_GROUND_SPEED   0x13 // Speed m/s
#define IBUS_MEAS_TYPE_GPS_DIST       0x14 // Distance from home
#define IBUS_MEAS_TYPE_ARMED          0x15 // Armed / unarmed
#define IBUS_MEAS_TYPE_FLIGHT_MODE    0x16 // Flight mode

#define IBUS_MEAS_TYPE_PRES           0x41 // Pressure
#define IBUS_MEAS_TYPE_SPE            0x7e // Speed km/h

// 4 byte sensors

#define IBUS_MEAS_TYPE_GPS_LAT 0x80 // WGS84 in degrees * 1E7
#define IBUS_MEAS_TYPE_GPS_LON 0x81 // WGS84 in degrees * 1E7
#define IBUS_MEAS_TYPE_GPS_ALT 0x82 // GPS altitude
#define IBUS_MEAS_TYPE_ALT     0x83 // Altitude
#define IBUS_MEAS_TYPE_ALT_MAX 0x84 // Max altitude
#define IBUS_MEAS_TYPE_S85     0x85
#define IBUS_MEAS_TYPE_S86     0x86
#define IBUS_MEAS_TYPE_S87     0x87
#define IBUS_MEAS_TYPE_S88     0x88
#define IBUS_MEAS_TYPE_S89     0x89
#define IBUS_MEAS_TYPE_S8a     0x8a

#endif // ifndef iBUSSensors_h