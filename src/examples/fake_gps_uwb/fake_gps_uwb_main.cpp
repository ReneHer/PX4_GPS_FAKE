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

#include "FakeGpsUwb.hpp"

//#include <px4_platform_common/getopt.h>
//#include <px4_platform_common/module.h>

// int FakeGpsUwb::print_usage(const char *reason)
// {
// 	if (reason)
// 	{
// 		PX4_WARN("%s\n", reason);
// 	}

// 	PRINT_MODULE_DESCRIPTION(
// 	R"DESCR_STR(
// Description: Faking the GPS-Signal with position data provided by an UWB-Tag connected via I2C.)DESCR_STR");
// 	PRINT_MODULE_USAGE_NAME("fake_gps_uwb", "driver");
// 	PRINT_MODULE_USAGE_SUBCATEGORY("gps");
// 	PRINT_MODULE_USAGE_COMMAND("start");
// 	//PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
// 	//PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x28);
// 	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
// 	PX4_INFO_RAW("\n");

// 	return 0;
// }

extern "C" __EXPORT int fake_gps_uwb_main(int argc, char *argv[])
{
	//using ThisDriver = FakeGpsUwb;
	BusCLIArguments cli{true, false};
	//cli.i2c_address = I2C_ADDRESS_DEFAULT;
	//cli.default_i2c_frequency = I2C_SPEED;

	//const char *verb = cli.parseDefaultArguments(argc, argv);

	return FakeGpsUwb::main(argc, argv);
}
