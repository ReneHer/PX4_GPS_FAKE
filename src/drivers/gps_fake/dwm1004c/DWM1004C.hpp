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


#include <string.h>
#include <drivers/drv_hrt.h>
#include <geo/geo.h>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_gps.h>

// #include <systemcmds/topic_listener/topic_listener.hpp>


#define DEBUG_MODE_ON

#define SENSOR_INTERVAL_HZ			5
#define GPS_HOR_SPEED_DRIFT_DELAY 	2*SENSOR_INTERVAL_HZ

#define LOCODECK_NR_OF_TWR_ANCHORS	8

#define DWM1004C_PROBE 				0x00
#define DWM1004C_TRANSMIT 			0x99
#define DWM1004C_OFFSET_UP 			0xA3
#define DWM1004C_OFFSET_DOWN		0x8F
#define DWM1004C_RESET 				0xFF
#define DWM1004C_DATA_LENGTH		1 + 1 + LOCODECK_NR_OF_TWR_ANCHORS * sizeof(double) + 2

#define RMA_WINDOW_SIZE				3
#define EMA_ALPHA					0.3  // Glättungsfaktor. 0.7 ist schon zu viel, es wird instabil

#define MAX_ERRORS					10


using namespace time_literals;
using namespace matrix;


static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x5A; //0x28;


class DWM1004C : public device::I2C, public I2CSPIDriver<DWM1004C>
{
	public:

		DWM1004C(const I2CSPIDriverConfig &config);
		~DWM1004C() override;

		static void print_usage();

		int init() override;

		void RunImpl();

		bool check_algorithm();

		void print_status() override;

		void custom_method(const BusCLIArguments &cli) override;

	private:

		int probe() override;

		Vector3d dme_least_squares(const Vector3d &x_ccf_i, const Matrix<double, 3, LOCODECK_NR_OF_TWR_ANCHORS> &anchorPosition, const Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> &z, const Vector<uint8_t, LOCODECK_NR_OF_TWR_ANCHORS> &anchorUsed);

		Vector3d rolling_moving_average(Vector3d x);

		void flat_earth_to_lla(double lat_ref, double lon_ref, double alt_ref, double x_fe, double y_fe, double z_fe, double Psi, double &lat, double &lon, double &alt);

		enum class STATE : uint8_t
		{
			MEASURE,
			READ,
			PUBLISH,
		} _state{STATE::MEASURE};

		enum class STATUS : uint8_t
		{
			Normal_Operation = 0b00, // 0: Normal Operation. Good Data Packet
			Reserved         = 0b01, // 1: Reserved
			Stale_Data       = 0b10, // 2: Stale Data. Data has been fetched since last measurement cycle.
			Fault_Detected   = 0b11, // 3: Fault Detected
		};

		hrt_abstime _timestamp_sample{0};
		hrt_abstime _timestamp_velocity{0};

		// Publications
		uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};

		// Performance (perf) counters
		perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
		perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};
		perf_counter_t _fault_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": fault detected")};

		perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
		perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

		static constexpr uint32_t SENSOR_INTERVAL_US{1000000 / SENSOR_INTERVAL_HZ}; // 5 Hz

		/*static constexpr*/ const double anchorPosition_data[LOCODECK_NR_OF_TWR_ANCHORS][3] =
		{
			{3.1900, 0.0000, 2.0500},
			{3.1900, 0.0000, 0.1200},
			{2.7300, 6.6550, 1.7850},
			{2.7400, 6.6550, 0.1750},
			{0.3000, 6.6300, 1.7750},
			{0.5100, 6.5000, 0.1600},
			{0.9100, 0.8750, 2.0450},
			{0.9050, 0.8750, 0.1600},
			// {0.330000, 1.490000, 0.160000},
			// {0.370000, 6.500000, 2.250000},
			// {4.470000, 6.490000, 0.160000},
			// {4.470000, 1.510000, 2.100000},
			// {0.330000, 1.490000, 2.220000},
			// {0.370000, 6.500000, 0.160000},
			// {4.470000, 6.510000, 2.250000},
			// {4.470000, 1.510000, 0.160000},
		};

		const double target_distance_fix_point_data[LOCODECK_NR_OF_TWR_ANCHORS] =	// IFSYS_Center
		{
			{4.53292400112775},
			{4.37916658737710},
			{2.79663190284313},
			{2.65157500365349},
			{3.16758977773322},
			{2.82380948365856},
			{3.71138114453366},
			{3.51865741441249},
		};
		Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> target_distance_fix_point;
		Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> sum_actual_distances_fix_point;
		Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> deviation_distances_fix_point;
		Vector<uint16_t, LOCODECK_NR_OF_TWR_ANCHORS> counter_measurements_fix_point;
		double sum_deviation_distances_fix_point = 0.0;

		/*static constexpr*/ const double x_0_data[3] = {1.0, 1.0, 1.0}; // {2.45*10, 4.10*10, 1.0};
		/*static constexpr*/ const double vel_N_data[3] = {0.0, 0.0, 0.0};

		/*static constexpr*/ const double lla_0_data[3] = {47.397742, 8.545594, 488.003000};

		// Matrix<double, LOCODECK_NR_OF_TWR_ANCHORS, 3> anchorPosition;
		Vector3d x_N;
		Vector3d vel_N;
		Vector3d lla_0;

		SquareMatrix<double, RMA_WINDOW_SIZE> window_rma;
		Vector3d sum_rma;
		Vector3d x_N_rma_minus_1;

		Vector3d x_N_ema_minus_1;

		double max_distance = 0.0;
		int8_t anchors_used_sum = 0;
		bool check_data = false;
		bool check_inverse = false;
		int measurements_good = 0;
		uint8_t dwm_errors = 0;
		uint16_t Schedule_Counter = 0;

		struct custom_method_data_t
		{
			uint16_t	received_data_print;
			uint16_t	received_data_counter;
			uint16_t	deviations_print;
			uint16_t 	deviations_counter;
			uint16_t 	sum_deviations_print;
			uint16_t 	sum_deviations_counter;
			uint16_t 	local_coordinates_print;
			uint16_t 	local_coordinates_counter;
			uint16_t 	local_velocities_print;
			uint16_t 	local_velocities_counter;
			uint16_t 	found_errors_print;
			uint16_t 	found_errors_counter;
			double		ema_alpha;							// 0.3 Glättungsfaktor. 0.7 ist schon zu viel, es wird instabil
			bool		vel_ned_valid;
		} custom_method_data = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.3, true};
};
