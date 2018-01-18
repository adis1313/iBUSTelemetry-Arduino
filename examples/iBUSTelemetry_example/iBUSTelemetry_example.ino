#include <iBUSTelemetry.h>

#define UPDATE_INTERVAL 500

iBUSTelemetry telemetry(11); // I use only PCINT0 interrupt vector, so you can use D8 to D13 pins.

uint32_t prevMillis = 0; // Necessary to updateValues() method. Look below.
float i = 0;

void setup()
{
    telemetry.begin(); // Let's start having fun!

    telemetry.addSensor(0x01); // You can use sensors definitions from iBUSSensors.h instead of numbers.
                               // Ex.: telemetry.addSensor(IBUS_MEAS_TYPE_TEM);

    telemetry.addSensor(IBUS_MEAS_TYPE_GPS_STATUS);
    telemetry.addSensor(IBUS_MEAS_TYPE_SPE);
    telemetry.addSensor(IBUS_MEAS_TYPE_GPS_LAT);
    telemetry.addSensor(IBUS_MEAS_TYPE_ARMED);
    telemetry.addSensor(IBUS_MEAS_TYPE_FLIGHT_MODE);
    telemetry.addSensor(IBUS_MEAS_TYPE_ALT);
}

void loop()
{
    updateValues(); // Very important! iBUS protocol is very sensitive to timings.
                    // DO NOT USE ANY delay()! Look at updateValues() method.
                    // It's an example of how to use intervals without delays.

    telemetry.run(); //It must be here. Period.
}

void updateValues()
{
    uint32_t currMillis = millis();

    if (currMillis - prevMillis >= UPDATE_INTERVAL) { // Code in the middle of these brackets will be performed every 500ms.
        prevMillis = currMillis;

        telemetry.setSensorValueFP(1, i); // Now, you have two ways to set sensors values. Using floating point variables
                                          // or directly in 32bit integer, but then you have to format variable by yourself.
                                          // Ex.: telemetry.setSensorValueFP(1, 24.45); is equivalent to telemetry.setSensorValue(1, 2445);
                                          // The values differ for different sensors.

        telemetry.setSensorValue(2, telemetry.gpsStateValues(3, 8)); // As GPS status consists of two values,
                                                                     // use gpsStateValues(firstVal, secondVal) to set it properly.

        telemetry.setSensorValue(3, 123 * 10);

        telemetry.setSensorValue(4, 179583647); // You can set LAT / LON using FP metohod, but due to fact floats have only 6 digits precision,
                                                // your values on tx may be corrupted in some cases. It's better to use setSensorValue().

        telemetry.setSensorValue(5, UNARMED); // ARMED / UNARMED or 1 / 0 could be used.

        telemetry.setSensorValue(6, LOITER); // Available flight modes:	   STAB   0
																		// ACRO   1
																		// AHOLD  2
																		// AUTO   3
																		// GUIDED 4
																		// LOITER 5
																		// RTL    6
																		// CIRCLE 7
																		// PHOLD  8
																		// LAND   9

        telemetry.setSensorValueFP(7, 54.87);

        i += 0.1;
        if (i > 50)
            i = 0;

        // These were the most difficult sensors to use. I hope that this library will be useful for you and will make your work easier. :)
    }
} /* updateValues */