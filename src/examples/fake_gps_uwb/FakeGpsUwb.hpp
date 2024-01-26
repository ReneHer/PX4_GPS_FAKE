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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/i2c_spi_buses.h>

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
// #include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
// #include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
// #include <uORB/topics/sensor_accel.h>
// #include <uORB/topics/vehicle_status.h>

#include <matrix/math.hpp>
#include <geo/geo.h>

#include <systemcmds/topic_listener/topic_listener.hpp>


#define LOCODECK_NR_OF_TWR_ANCHORS  8
#define DATA_RX_LENGTH				1 + 1 + LOCODECK_NR_OF_TWR_ANCHORS * sizeof(float) + 2
#define data_length 				920
#define DW_RESET 					0xFF
#define MAX_ERRORS 					10


using namespace time_literals;
using namespace matrix;


static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x28;


class FakeGpsUwb : /*public device::I2C, public I2CSPIDriver<FakeGpsUwb>,*/ public ModuleBase<FakeGpsUwb>, public ModuleParams, public px4::ScheduledWorkItem
{
	public:

		FakeGpsUwb();
		~FakeGpsUwb() override;

		/** @see ModuleBase */
		static int task_spawn(int argc, char *argv[]);

		/** @see ModuleBase */
		static int custom_command(int argc, char *argv[]);

		/** @see ModuleBase */
		static int print_usage(const char *reason = nullptr);

		bool init();

		bool check_algorithm();

		int print_status() override;

	private:
	
		void Run() override;

		Vector3f dme_least_squares(const Vector3f &x_ccf_i, const Matrix<float, 3, LOCODECK_NR_OF_TWR_ANCHORS> &anchorPosition, const Vector<float, LOCODECK_NR_OF_TWR_ANCHORS> &z, const Vector<uint8_t, LOCODECK_NR_OF_TWR_ANCHORS> &anchorUsed);

		void flat_earth_to_lla(double lat_ref, double lon_ref, double alt_ref, double x_fe, double y_fe, double z_fe, double Psi, double &lat, double &lon, double &alt);

		enum class STATE : uint8_t
		{
			MEASURE,
			READ,
		} _state{STATE::MEASURE};

		enum class STATUS : uint8_t
		{
			Normal_Operation = 0b00, // 0: Normal Operation. Good Data Packet
			Reserved         = 0b01, // 1: Reserved
			Stale_Data       = 0b10, // 2: Stale Data. Data has been fetched since last measurement cycle.
			Fault_Detected   = 0b11, // 3: Fault Detected
		};

		// hrt_abstime _timestamp_sample{0};

		// Publications
		uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};

		// Subscriptions
		uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates

		// Performance (perf) counters
		perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
		perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};
		perf_counter_t _fault_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": fault detected")};

		perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
		perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

		// Parameters
		DEFINE_PARAMETERS(
			(ParamInt<px4::params::SIM_GPS_USED>) _sim_gps_used
		)

		// static constexpr uint32_t SENSOR_INTERVAL_US{1000000 / 5}; // 5 Hz
		static constexpr uint32_t SENSOR_INTERVAL_US{9000000 / 1};

		float anchorPosition_data[LOCODECK_NR_OF_TWR_ANCHORS][3] =
		{
			{0.330000, 1.490000, 0.160000},
			{0.370000, 6.500000, 2.250000},
			{4.470000, 6.490000, 0.160000},
			{4.470000, 1.510000, 2.100000},
			{0.330000, 1.490000, 2.220000},
			{0.370000, 6.500000, 0.160000},
			{4.470000, 6.510000, 2.250000},
			{4.470000, 1.510000, 0.160000}
		};

		const float x_0_data[3] = {2.45, 4.10, 1.0};
		const float vel_N_data[3] = {0.0, 0.0, 0.0};

		const double lla_0_data[3] = {47.397742, 8.545594, 488.003000};
		// const double lla_0_data[3] = {44.532, -72.782, 1699};

		Vector3f x_N;
		Vector3f vel_N;
		Vector3d lla_0;

		bool check_data = false;
		bool check_inverse = false;
		int measurements_good = 0;

		FILE *fp_distances;
		FILE *fp_hits;
};
