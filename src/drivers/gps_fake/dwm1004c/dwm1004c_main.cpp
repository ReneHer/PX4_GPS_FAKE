/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include "DWM1004C.hpp"


void DWM1004C::print_usage()
{
	PX4_INFO_RAW("\n");

	PRINT_MODULE_USAGE_NAME("dwm1004c", "driver");
	// PRINT_MODULE_USAGE_SUBCATEGORY("gps");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x28);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	PRINT_MODULE_USAGE_COMMAND_DESCR("print", "print choosen parameters of the next <val> (default=1) loops");
	PRINT_MODULE_USAGE_ARG("received_data <val>", "    received data of the I2C-communication", true);
	PRINT_MODULE_USAGE_ARG("deviations <val>", "       deviation of the meassured to the target distances between the fix point and the used anchors", true);
	PRINT_MODULE_USAGE_ARG("sum_deviations <val>", "   sum of the deviations of the meassured to the target distances to the programmed fix point", true);
	PRINT_MODULE_USAGE_ARG("local_coordinates <val>", "calculated local coordinates", true);
	PRINT_MODULE_USAGE_ARG("local_velocities <val>", " calculated local velocities", true);
	PRINT_MODULE_USAGE_ARG("found_errors <val>", "     number of found errors", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("anchor", "usage of anchor <val> (default=on) in the calculations");
	PRINT_MODULE_USAGE_ARG("<val> on", "set 'anchors_used[<val>] = 1'", true);
	PRINT_MODULE_USAGE_ARG("<val> off", "set 'anchors_used[<val>] = 0'", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("ant_offset", "change of the LOCODECK_ANTENNA_OFFSET");
	PRINT_MODULE_USAGE_ARG("up", "increase the offset by 0.01, if the sum of deviations of meassured to target distances of fix point > 0.0", true);
	PRINT_MODULE_USAGE_ARG("down", "decrease the offset by 0.01, if the sum of deviations of meassured to target distances of fix point < 0.0", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("ema_alpha", "change of alpha value of the Exponential Moving Average/Low Pass Filter");
	PRINT_MODULE_USAGE_ARG("<val>", "value of alpha between 0.0 (vel_0) ... 0.3 (default) ... 1.0 (no filter)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("vel_ned_valid", "validity of the calculated local velocities in the NED-frame");
	PRINT_MODULE_USAGE_ARG("true", "set 'sensor_gps.vel_ned_valid = true'", true);
	PRINT_MODULE_USAGE_ARG("false", "set 'sensor_gps.vel_ned_valid = false'", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("dwm_reset", "reset the DWM1004C module");

	PX4_INFO_RAW("\n");
}


extern "C" int dwm1004c_main(int argc, char *argv[])
{
	using ThisDriver = DWM1004C;

	BusCLIArguments cli{true, false};
	cli.i2c_address = I2C_ADDRESS_DEFAULT;
	cli.default_i2c_frequency = I2C_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb)
	{
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_GPS_DEVTYPE_DWM1004C);

	if (!strcmp(verb, "start"))
	{
		return ThisDriver::module_start(cli, iterator);

	}
	else if (!strcmp(verb, "stop"))
	{
		return ThisDriver::module_stop(iterator);

	}
	else if (!strcmp(verb, "status"))
	{
		return ThisDriver::module_status(iterator);
	}
	else if (!strcmp(verb, "print"))
	{
		if (argc >= 3)
		{
			if (!strcmp(argv[2], "received_data"))
			{
				cli.custom1 = 10;
				cli.custom2 = 1;
				if (argc >= 4)
				{
					cli.custom2 = atoi(argv[3]);
				}
				PX4_INFO_RAW("The received data will be printed for the next %d I2C-communications. -> dmesg\n", cli.custom2);
				return ThisDriver::module_custom_method(cli, iterator);
			}
			else if (!strcmp(argv[2], "deviations"))
			{
				cli.custom1 = 11;
				cli.custom2 = 1;
				if (argc >= 4)
				{
					cli.custom2 = atoi(argv[3]);
				}
				PX4_INFO_RAW("The deviation of the meassured to the target distances between the programmed fix point\n");
				PX4_INFO_RAW("and the used anchors will be printed for the next %d loops. -> dmesg\n", cli.custom2);
				return ThisDriver::module_custom_method(cli, iterator);
			}
			else if (!strcmp(argv[2], "sum_deviations"))
			{
				cli.custom1 = 12;
				cli.custom2 = 1;
				if (argc >= 4)
				{
					cli.custom2 = atoi(argv[3]);
				}
				PX4_INFO_RAW("The sum of the deviations of the meassured to the target distances between\n");
				PX4_INFO_RAW("and the used anchors will be printed for the next %d loops. -> dmesg\n", cli.custom2);
				return ThisDriver::module_custom_method(cli, iterator);
			}
			else if (!strcmp(argv[2], "local_coordinates"))
			{
				cli.custom1 = 13;
				cli.custom2 = 1;
				if (argc >= 4)
				{
					cli.custom2 = atoi(argv[3]);
				}
				PX4_INFO_RAW("The calculated local coordinates will be printed for the next %d loops. -> dmesg\n", cli.custom2);
				return ThisDriver::module_custom_method(cli, iterator);
			}
			else if (!strcmp(argv[2], "local_velocities"))
			{
				cli.custom1 = 14;
				cli.custom2 = 1;
				if (argc >= 4)
				{
					cli.custom2 = atoi(argv[3]);
				}
				PX4_INFO_RAW("The calculated local velocities will be printed for the next %d loops. -> dmesg\n", cli.custom2);
				return ThisDriver::module_custom_method(cli, iterator);
			}
			else if (!strcmp(argv[2], "found_errors"))
			{
				cli.custom1 = 15;
				cli.custom2 = 1;
				if (argc >= 4)
				{
					cli.custom2 = atoi(argv[3]);
				}
				PX4_INFO_RAW("The number of found errors will be printed for the next %d loops. -> dmesg\n", cli.custom2);
				return ThisDriver::module_custom_method(cli, iterator);
			}
		}
	}
	else if (!strcmp(verb, "anchor"))
	{
		if (argc >= 4)
		{
			cli.custom2 = atoi(argv[2]);
			if (!strcmp(argv[3], "on"))
			{
				cli.custom1 = 20;
				PX4_INFO_RAW("Will set 'anchors_used[%d] = 1'.\n", cli.custom2);
				return ThisDriver::module_custom_method(cli, iterator);
			}
			else if (!strcmp(argv[3], "off"))
			{
				cli.custom1 = 21;
				PX4_INFO_RAW("Will set 'anchors_used[%d] = 0'.\n", cli.custom2);
				return ThisDriver::module_custom_method(cli, iterator);
			}
		}
	}
	else if (!strcmp(verb, "ant_offset"))
	{
		if (argc >= 3)
		{
			if (!strcmp(argv[2], "up"))
			{
				cli.custom1 = 30;
				PX4_INFO_RAW("The LOCODECK_ANTENNA_OFFSET will be increased by 0.01.\n");
				return ThisDriver::module_custom_method(cli, iterator);
			}
			else if (!strcmp(argv[2], "down"))
			{
				cli.custom1 = 31;
				PX4_INFO_RAW("The LOCODECK_ANTENNA_OFFSET will be decreased by 0.01.\n");
				return ThisDriver::module_custom_method(cli, iterator);
			}
		}
	}
	else if (!strcmp(verb, "ema_alpha"))
	{
		cli.custom1 = 40;
		double alpha = 0.3;
		cli.custom_data = &alpha;
		if (argc >= 3)
		{
			alpha = atof(argv[2]);
		}
		// double x = *((float *)cli.custom_data);
		PX4_INFO_RAW("Will set 'ema_alpha = %f'.\n", *((double *)cli.custom_data)); // TODO: check if this works?
		return ThisDriver::module_custom_method(cli, iterator);
	}
	else if (!strcmp(verb, "vel_ned_valid"))
	{
		if (argc >= 3)
		{
			if (!strcmp(argv[2], "true"))
			{
				cli.custom1 = 50;
				PX4_INFO_RAW("Will set 'sensor_gps.vel_ned_valid = true'.\n");
				return ThisDriver::module_custom_method(cli, iterator);
			}
			else if (!strcmp(argv[2], "false"))
			{
				cli.custom1 = 51;
				PX4_INFO_RAW("Will set 'sensor_gps.vel_ned_valid = false'.\n");
				return ThisDriver::module_custom_method(cli, iterator);
			}
		}
	}
	else if (!strcmp(verb, "dwm_reset"))
	{
		cli.custom1 = 90;
		PX4_INFO_RAW("The DWM1004C will be reseted.\n");
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();

	return -1;
}
