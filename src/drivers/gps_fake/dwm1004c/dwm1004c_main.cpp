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

	PRINT_MODULE_USAGE_COMMAND_DESCR("print", "print choosen parameters of the next <val> (default = 1) loops");
	PRINT_MODULE_USAGE_ARG("received_data <val>", "    received data of the I2C-communication", true);
	PRINT_MODULE_USAGE_ARG("deviations <val>", "       deviation of the meassured to the target distances between the fix point and the used anchors", true);
	PRINT_MODULE_USAGE_ARG("sum_deviations <val>", "   sum of the deviations of the meassured to the target distances to the programmed fix point", true);
	PRINT_MODULE_USAGE_ARG("local_coordinates <val>", "calculated local coordinates", true);
	PRINT_MODULE_USAGE_ARG("local_velocities <val>", " calculated local velocities", true);
	PRINT_MODULE_USAGE_ARG("found_errors <val>", "     number of found errors", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("anchor", "usage of anchor <val> in the calculations");
	PRINT_MODULE_USAGE_ARG("<val> on", "set 'anchors_used[<val>] = 1' (default) ", true);
	PRINT_MODULE_USAGE_ARG("<val> off", "set 'anchors_used[<val>] = 0'", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("ant_offset", "change of the LOCODECK_ANTENNA_OFFSET (default = 153.92)");
	PRINT_MODULE_USAGE_ARG("up", "increase the offset by 0.01, if the sum of deviations of meassured to target distances of fix point > 0.0", true);
	PRINT_MODULE_USAGE_ARG("down", "decrease the offset by 0.01, if the sum of deviations of meassured to target distances of fix point < 0.0", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("ema_alpha", "change of alpha value of the Exponential Moving Average/Low Pass Filter");
	PRINT_MODULE_USAGE_ARG("<val>", "value of alpha between 0.0 (vel_0) ... 0.3 (default) ... 1.0 (no filter)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("vel_ned_valid", "validity of the calculated local velocities in the NED-frame");
	PRINT_MODULE_USAGE_ARG("true", "set 'sensor_gps.vel_ned_valid = true' (default) ", true);
	PRINT_MODULE_USAGE_ARG("false", "set 'sensor_gps.vel_ned_valid = false'", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("driver_freq", "working frequency of this driver");
	PRINT_MODULE_USAGE_ARG("5_Hz", "set working frequency to 5 Hz (default)", true);
	PRINT_MODULE_USAGE_ARG("10_Hz", "set working frequency to 10 Hz", true);
	PRINT_MODULE_USAGE_ARG("15_Hz", "set working frequency to 15 Hz", true);
	PRINT_MODULE_USAGE_ARG("20_Hz", "set working frequency to 20 Hz", true);
	// PRINT_MODULE_USAGE_COMMAND_DESCR("anchor_position", "change the anchor positions in local xyz-coordinates");
	// PRINT_MODULE_USAGE_ARG("0 <val> <val> <val>", "set new position for anchor 0 in local [x y z]-coordinates separated by spaces", true);
	// PRINT_MODULE_USAGE_ARG("1 <val> <val> <val>", "set new position for anchor 1 in local [x y z]-coordinates separated by spaces", true);
	// PRINT_MODULE_USAGE_ARG("2 <val> <val> <val>", "set new position for anchor 2 in local [x y z]-coordinates separated by spaces", true);
	// PRINT_MODULE_USAGE_ARG("3 <val> <val> <val>", "set new position for anchor 3 in local [x y z]-coordinates separated by spaces", true);
	// PRINT_MODULE_USAGE_ARG("3 <val> <val> <val>", "set new position for anchor 3 in local [x y z]-coordinates separated by spaces", true);
	// PRINT_MODULE_USAGE_ARG("5 <val> <val> <val>", "set new position for anchor 5 in local [x y z]-coordinates separated by spaces", true);
	// PRINT_MODULE_USAGE_ARG("6 <val> <val> <val>", "set new position for anchor 6 in local [x y z]-coordinates separated by spaces", true);
	// PRINT_MODULE_USAGE_ARG("7 <val> <val> <val>", "set new position for anchor 7 in local [x y z]-coordinates separated by spaces", true);
	// PRINT_MODULE_USAGE_COMMAND_DESCR("target_distance", "change the target distance from the fix-point to an anchor");
	// PRINT_MODULE_USAGE_ARG("0 <val>", "set new target distance <val> from the fix-point to anchor 0", true);
	// PRINT_MODULE_USAGE_ARG("1 <val>", "set new target distance <val> from the fix-point to anchor 1", true);
	// PRINT_MODULE_USAGE_ARG("2 <val>", "set new target distance <val> from the fix-point to anchor 2", true);
	// PRINT_MODULE_USAGE_ARG("3 <val>", "set new target distance <val> from the fix-point to anchor 3", true);
	// PRINT_MODULE_USAGE_ARG("3 <val>", "set new target distance <val> from the fix-point to anchor 4", true);
	// PRINT_MODULE_USAGE_ARG("5 <val>", "set new target distance <val> from the fix-point to anchor 5", true);
	// PRINT_MODULE_USAGE_ARG("6 <val>", "set new target distance <val> from the fix-point to anchor 6", true);
	// PRINT_MODULE_USAGE_ARG("7 <val>", "set new target distance <val> from the fix-point to anchor 7", true);
	// PRINT_MODULE_USAGE_COMMAND_DESCR("fix_position", "change the position for the fix-point, used for calibration, in local xyz-coordinates");
	// PRINT_MODULE_USAGE_ARG("<val> <val> <val>", "set new local [x y z] for the fix-point separated by spaces (default = [2.000 3.000 1.080])", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("ref_position", "change the reference position in LLA-coordinates on earth");
	PRINT_MODULE_USAGE_ARG("<val> <val> <val>", "set new [latitude longitude altitude] separated by spaces (default = [52.51539, 13.32394, 60.00000])", true);
	// PRINT_MODULE_USAGE_COMMAND_DESCR("uORB_message", "custom uORB message with the calculated x_N and vel_N values for logging");
	// PRINT_MODULE_USAGE_ARG("start", "start the custom uORB message", true);
	// PRINT_MODULE_USAGE_ARG("stop", "stop the custom uORB message", true);
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
				PX4_INFO_RAW("Will set 'anchors_used(%d) = 1'.\n", cli.custom2);
				return ThisDriver::module_custom_method(cli, iterator);
			}
			else if (!strcmp(argv[3], "off"))
			{
				cli.custom1 = 21;
				PX4_INFO_RAW("Will set 'anchors_used(%d) = 0'.\n", cli.custom2);
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
		PX4_INFO_RAW("Will set 'ema_alpha = %f'.\n", *((double *)cli.custom_data));
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
	else if (!strcmp(verb, "driver_freq"))
	{
		if (argc >= 3)
		{
			if (!strcmp(argv[2], "5_Hz"))
			{
				cli.custom1 = 60;
				PX4_INFO_RAW("Will set the working frequency of the driver to 5 Hz.\n");
				return ThisDriver::module_custom_method(cli, iterator);
			}
			else if (!strcmp(argv[2], "10_Hz"))
			{
				cli.custom1 = 61;
				PX4_INFO_RAW("Will set the working frequency of the driver to 10 Hz.\n");
				return ThisDriver::module_custom_method(cli, iterator);
			}
			else if (!strcmp(argv[2], "15_Hz"))
			{
				cli.custom1 = 62;
				PX4_INFO_RAW("Will set the working frequency of the driver to 15 Hz.\n");
				return ThisDriver::module_custom_method(cli, iterator);
			}
			else if (!strcmp(argv[2], "20_Hz"))
			{
				cli.custom1 = 63;
				PX4_INFO_RAW("Will set the working frequency of the driver to 20 Hz.\n");
				return ThisDriver::module_custom_method(cli, iterator);
			}
		}
	}
	// else if (!strcmp(verb, "anchor_position"))
	// {
	// 	if (argc >= 3)
	// 	{
	// 		if (!strcmp(argv[2], "0"))
	// 		{
	// 			cli.custom1 = 70;
	// 			double anchorPosition[3] = {3.1900, 0.0000, 2.0500};
	// 			cli.custom_data = &anchorPosition;
	// 			if (argc >= 6)
	// 			{
	// 				anchorPosition[0] = atof(argv[3]);
	// 				anchorPosition[1] = atof(argv[4]);
	// 				anchorPosition[2] = atof(argv[5]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'anchorPosition(0) = [%f, %f, %f]'.\n", *((double *)cli.custom_data), *(((double *)cli.custom_data)+1), *(((double *)cli.custom_data)+2));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "1"))
	// 		{
	// 			cli.custom1 = 71;
	// 			double anchorPosition[3] = {3.1900, 0.0000, 0.1200};
	// 			cli.custom_data = &anchorPosition;
	// 			if (argc >= 6)
	// 			{
	// 				anchorPosition[0] = atof(argv[3]);
	// 				anchorPosition[1] = atof(argv[4]);
	// 				anchorPosition[2] = atof(argv[5]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'anchorPosition(1) = [%f, %f, %f]'.\n", *((double *)cli.custom_data), *(((double *)cli.custom_data)+1), *(((double *)cli.custom_data)+2));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "2"))
	// 		{
	// 			cli.custom1 = 72;
	// 			double anchorPosition[3] = {2.7300, 6.6550, 1.7850};
	// 			cli.custom_data = &anchorPosition;
	// 			if (argc >= 6)
	// 			{
	// 				anchorPosition[0] = atof(argv[3]);
	// 				anchorPosition[1] = atof(argv[4]);
	// 				anchorPosition[2] = atof(argv[5]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'anchorPosition(2) = [%f, %f, %f]'.\n", *((double *)cli.custom_data), *(((double *)cli.custom_data)+1), *(((double *)cli.custom_data)+2));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "3"))
	// 		{
	// 			cli.custom1 = 73;
	// 			double anchorPosition[3] = {2.7400, 6.6550, 0.1750};
	// 			cli.custom_data = &anchorPosition;
	// 			if (argc >= 6)
	// 			{
	// 				anchorPosition[0] = atof(argv[3]);
	// 				anchorPosition[1] = atof(argv[4]);
	// 				anchorPosition[2] = atof(argv[5]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'anchorPosition(3) = [%f, %f, %f]'.\n", *((double *)cli.custom_data), *(((double *)cli.custom_data)+1), *(((double *)cli.custom_data)+2));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "4"))
	// 		{
	// 			cli.custom1 = 74;
	// 			double anchorPosition[3] = {0.3000, 6.6300, 1.7750};
	// 			cli.custom_data = &anchorPosition;
	// 			if (argc >= 6)
	// 			{
	// 				anchorPosition[0] = atof(argv[3]);
	// 				anchorPosition[1] = atof(argv[4]);
	// 				anchorPosition[2] = atof(argv[5]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'anchorPosition(4) = [%f, %f, %f]'.\n", *((double *)cli.custom_data), *(((double *)cli.custom_data)+1), *(((double *)cli.custom_data)+2));
	// 		}
	// 		else if (!strcmp(argv[2], "5"))
	// 		{
	// 			cli.custom1 = 75;
	// 			double anchorPosition[3] = {0.5100, 6.5000, 0.1600};
	// 			cli.custom_data = &anchorPosition;
	// 			if (argc >= 6)
	// 			{
	// 				anchorPosition[0] = atof(argv[3]);
	// 				anchorPosition[1] = atof(argv[4]);
	// 				anchorPosition[2] = atof(argv[5]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'anchorPosition(5) = [%f, %f, %f]'.\n", *((double *)cli.custom_data), *(((double *)cli.custom_data)+1), *(((double *)cli.custom_data)+2));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "6"))
	// 		{
	// 			cli.custom1 = 76;
	// 			double anchorPosition[3] = {0.9100, 0.8750, 2.0450};
	// 			cli.custom_data = &anchorPosition;
	// 			if (argc >= 6)
	// 			{
	// 				anchorPosition[0] = atof(argv[3]);
	// 				anchorPosition[1] = atof(argv[4]);
	// 				anchorPosition[2] = atof(argv[5]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'anchorPosition(6) = [%f, %f, %f]'.\n", *((double *)cli.custom_data), *(((double *)cli.custom_data)+1), *(((double *)cli.custom_data)+2));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "7"))
	// 		{
	// 			cli.custom1 = 77;
	// 			double anchorPosition[3] = {0.9050, 0.8750, 0.1600};
	// 			cli.custom_data = &anchorPosition;
	// 			if (argc >= 6)
	// 			{
	// 				anchorPosition[0] = atof(argv[3]);
	// 				anchorPosition[1] = atof(argv[4]);
	// 				anchorPosition[2] = atof(argv[5]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'anchorPosition(7) = [%f, %f, %f]'.\n", *((double *)cli.custom_data), *(((double *)cli.custom_data)+1), *(((double *)cli.custom_data)+2));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 	}
	// }
	// else if (!strcmp(verb, "target_distance"))
	// {
	// 	if (argc >= 3)
	// 	{
	// 		if (!strcmp(argv[2], "0"))
	// 		{
	// 			cli.custom1 = 80;
	// 			double target_distance = 3.370014836762592;
	// 			cli.custom_data = &target_distance;
	// 			if (argc >= 4)
	// 			{
	// 				target_distance = atof(argv[3]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'target_distance(0) = %f'.\n", *((double *)cli.custom_data));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "1"))
	// 		{
	// 			cli.custom1 = 81;
	// 			double target_distance = 3.367150130303073;
	// 			cli.custom_data = &target_distance;
	// 			if (argc >= 4)
	// 			{
	// 				target_distance = atof(argv[3]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'target_distance(1) = %f'.\n", *((double *)cli.custom_data));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "2"))
	// 		{
	// 			cli.custom1 = 82;
	// 			double target_distance = 3.79327694744267;
	// 			cli.custom_data = &target_distance;
	// 			if (argc >= 4)
	// 			{
	// 				target_distance = atof(argv[3]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'target_distance(2) = %f'.\n", *((double *)cli.custom_data));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "3"))
	// 		{
	// 			cli.custom1 = 83;
	// 			double target_distance = 3.83740146453300;
	// 			cli.custom_data = &target_distance;
	// 			if (argc >= 4)
	// 			{
	// 				target_distance = atof(argv[3]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'target_distance(3) = %f'.\n", *((double *)cli.custom_data));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "4"))
	// 		{
	// 			cli.custom1 = 84;
	// 			double target_distance = 4.06815990344529;
	// 			cli.custom_data = &target_distance;
	// 			if (argc >= 4)
	// 			{
	// 				target_distance = atof(argv[3]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'target_distance(4) = %f'.\n", *((double *)cli.custom_data));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "5"))
	// 		{
	// 			cli.custom1 = 85;
	// 			double target_distance = 3.91363002850295;
	// 			cli.custom_data = &target_distance;
	// 			if (argc >= 4)
	// 			{
	// 				target_distance = atof(argv[3]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'target_distance(5) = %f'.\n", *((double *)cli.custom_data));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "6"))
	// 		{
	// 			cli.custom1 = 86;
	// 			double target_distance = 2.57583966892351;
	// 			cli.custom_data = &target_distance;
	// 			if (argc >= 4)
	// 			{
	// 				target_distance = atof(argv[3]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'target_distance(6) = %f'.\n", *((double *)cli.custom_data));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "7"))
	// 		{
	// 			cli.custom1 = 87;
	// 			double target_distance = 2.56145466483403;
	// 			cli.custom_data = &target_distance;
	// 			if (argc >= 4)
	// 			{
	// 				target_distance = atof(argv[3]);
	// 			}
	// 			PX4_INFO_RAW("Will set 'target_distance(7) = %f'.\n", *((double *)cli.custom_data));
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 	}
	// }
	// else if (!strcmp(verb, "fix_position"))
	// {
	// 	if (argc >= 2)
	// 	{
	// 		cli.custom1 = 90;
	// 		double fix_position[3] = {2.000, 3.000, 1.080};
	// 		cli.custom_data = &fix_position;
	// 		if (argc >= 5)
	// 		{
	// 			fix_position[0] = atof(argv[2]);
	// 			fix_position[1] = atof(argv[3]);
	// 			fix_position[2] = atof(argv[4]);
	// 		}
	// 		PX4_INFO_RAW("Will set 'fix_position = [%f, %f, %f]'.\n", *((double *)cli.custom_data), *(((double *)cli.custom_data)+1), *(((double *)cli.custom_data)+2));
	// 		return ThisDriver::module_custom_method(cli, iterator);
	// 	}
	// }
	else if (!strcmp(verb, "ref_position"))
	{
		if (argc >= 2)
		{
			cli.custom1 = 100;
			double ref_position[3] = {52.515391059955704, 13.323944815389625, 60.000000000000000};
			cli.custom_data = &ref_position;
			if (argc >= 5)
			{
				ref_position[0] = atof(argv[2]);
				ref_position[1] = atof(argv[3]);
				ref_position[2] = atof(argv[4]);
			}
			PX4_INFO_RAW("Will set 'ref_position = [%f, %f, %f]'.\n", *((double *)cli.custom_data), *(((double *)cli.custom_data)+1), *(((double *)cli.custom_data)+2));
			return ThisDriver::module_custom_method(cli, iterator);
		}
	}
	// else if (!strcmp(verb, "uORB_message"))
	// {
	// 	if (argc >= 3)
	// 	{
	// 		if (!strcmp(argv[2], "start"))
	// 		{
	// 			cli.custom1 = 70;
	// 			PX4_INFO_RAW("Will start the publishing off the custom uORB message.\n");
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 		else if (!strcmp(argv[2], "stop"))
	// 		{
	// 			cli.custom1 = 71;
	// 			PX4_INFO_RAW("Will stop the publishing off the custom uORB message.\n");
	// 			return ThisDriver::module_custom_method(cli, iterator);
	// 		}
	// 	}
	// }
	else if (!strcmp(verb, "dwm_reset"))
	{
		cli.custom1 = 120;
		PX4_INFO_RAW("The DWM1004C will be reseted.\n");
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();

	return -1;
}
