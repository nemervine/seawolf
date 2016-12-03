#include "seawolf.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

const char* app_name = "Serial : Flowsensor";
void manage(SerialPort sp);
void update_pwm(SerialPort sp, float value);

int main(int argc, char** argv) {
    /* Configuration */
    Seawolf_loadConfig("../conf/seawolf.conf");

    /* Init libseawolf */
    Seawolf_init(app_name);

    /* Check arguments */
    if(argc != 2) {
        Logging_log(ERROR, Util_format("%s spawned with invalid argument count of %d", argv[0], argc));
        exit(1);
    }

    /* Attempt to open serial device */
    char* device = argv[1];
    SerialPort sp = Serial_open(device);

    /* Error opening device */
    if(sp == 0) {
        Logging_log(ERROR, Util_format("%s could not open device %s", argv[0], device));
        exit(1);
    }

    /* Set baud rate */
    Serial_setBaud(sp, 19200);

    /* Error checking done */
    Logging_log(INFO, Util_format("%s running successfully with device %s", argv[0], device));

    /* Start app specific code */
    manage(sp);

    /* Close serial device and exit */
    Serial_closePort(sp);
    Seawolf_close();
    return 0;
}

void update_pwm(SerialPort sp, float value) {
    unsigned char command[1];
    if(value < 0 || value > 255) {
        return;
    }
    command[0] = value;
    Serial_send(sp, command, 1);
}

void manage(SerialPort sp) {
    Var_subscribe("PumpPWM");
	unsigned char data[2];
	unsigned short pulses;
	
	Serial_setBlocking(sp);

    while(true) {
        Var_sync();
		
		while(Serial_getByte(sp) != 0x01);

        Serial_get(sp, data, 2);
		
		pulses = (data[0] * 256) + data[1];
		
		if(pulses >= 0) {
			Var_set("FlowPulses", pulses);
		}

        if(Var_poked("PumpPWM")) {
            update_pwm(sp, Var_get("PumpPWM"));
        }
        
    }

}
