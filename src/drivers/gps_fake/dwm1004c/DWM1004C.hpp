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

		// Vector3d dme_least_squares(const Vector3d &x_ccf_i, const Matrix<double, 3, LOCODECK_NR_OF_TWR_ANCHORS> &anchorPosition, const Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> &z, const Vector<uint8_t, LOCODECK_NR_OF_TWR_ANCHORS> &anchorUsed);
		Vector3d dme_least_squares(const Vector3d &x_ccf_i, const Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> &z, const Vector<uint8_t, LOCODECK_NR_OF_TWR_ANCHORS> &anchorUsed);

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

		uint8_t anchors_used_sum = 0;
		uint8_t dwm_errors = 0;

		double max_distance = 0.0;

		/*static constexpr*/ const double anchorPosition_data[LOCODECK_NR_OF_TWR_ANCHORS][3] =
		{
			// Cage
			// Standard [x, y, z]-Koodinaten
			{0.0000, 6.1030, 2.2300},
			{0.2600, 0.1030, 0.1600},
			{0.1300, 0.0000, 2.2580},
			{6.7400, 0.1030, 0.1600},
			{6.8600, 6.1880, 2.4800},
			{6.7400, 6.1030, 0.1600},
			{7.0000, 0.0050, 2.2400},
			{0.2600, 6.1030, 0.1600},
			// IFSYS Raum
			// Standard [x, y, z]-Koodinaten
			// {3.1900, 0.0050, 2.0500},
			// {3.1900, 0.0150, 0.1100},
			// {2.7300, 6.6500, 1.7800},
			// {2.7300, 6.6500, 0.1700},
			// {0.3000, 6.6200, 1.7700},
			// {0.5100, 6.5000, 0.1600},
			// {0.9100, 0.8900, 2.0300},
			// {0.9000, 0.8800, 0.1600},
			// [N, E, D]-Koodinaten
			// {3.1930, 7.1490, 0.4160},
			// {3.1900, 7.1550, 2.3500},
			// {2.7260, 0.5190, 0.6810},
			// {2.7290, 0.5110, 2.2920},
			// {0.3000, 0.5440, 0.6950},
			// {0.5120, 0.6670, 2.3070},
			// {0.9120, 6.2900, 0.4370},
			// {0.9030, 6.2900, 2.3030},
			// Keller Flugführer
			// Standard [x, y, z]-Koodinaten
			// {0.330000, 1.490000, 0.160000},
			// {0.370000, 6.500000, 2.250000},
			// {4.470000, 6.490000, 0.160000},
			// {4.470000, 1.510000, 2.100000},
			// {0.330000, 1.490000, 2.220000},
			// {0.370000, 6.500000, 0.160000},
			// {4.470000, 6.510000, 2.250000},
			// {4.470000, 1.510000, 0.160000},
		};
		Matrix<double, LOCODECK_NR_OF_TWR_ANCHORS, 3> anchorPosition;   // TODO: global?
		Matrix<double, 3, LOCODECK_NR_OF_TWR_ANCHORS> anchorPosition_transposed;   // TODO: global?

		const double target_distance_fix_point_data[LOCODECK_NR_OF_TWR_ANCHORS] =
		{
			// IFSYS_Center Tisch {2.000, 4.170, 0.735};
			// {4.53292400112775},
			// {4.37916658737710},
			// {2.79663190284313},
			// {2.65157500365349},
			// {3.16758977773322},
			// {2.82380948365856},
			// {3.71138114453366},
			// {3.51865741441249},
			// IFSYS_Center Rollwagen DWM Position Z1 {2.000, 3.000, 0.730};
			// {3.486904070948898},
			// {3.284539541549165},
			// {3.873622335747253},
			// {3.770232088346817},
			// {4.142333279686703},
			// {3.846426913383381},
			// {2.726343705404731},
			// {2.457549592582009},
			// IFSYS_Center Rollwagen DWM Position Z2 {2.000, 3.000, 0.815};
			// {3.455622230510737},
			// {3.301382286255259},
			// {3.851340675660880},
			// {3.783678765434508},
			// {4.121710809845834},
			// {3.859938471012200},
			// {2.686377672629074},
			// {2.478643782393912},
			// IFSYS_Center Rollwagen DWM Position Z3 {2.000, 3.000, 0.775};
			// {3.470118874044519},
			// {3.293193738606947},
			// {3.861609120561013},
			// {3.777118610793154},
			// {4.131210476361620},
			// {3.853352436515508},
			// {2.704926061836072},
			// {2.468374971514660},
			// IFSYS_Center Stehle DWM Position I1 {2.000, 3.000, 0.950};
			// {3.409706732257190},
			// {3.332416540590327},
			// {3.819574583641482},
			// {3.808838405603472},
			// {4.092374005391003},
			// {3.885125480599050},
			// {2.627308508721425},
			// {2.517687430957227},
			// IFSYS_Center Stehle DWM Position I2 {2.000, 3.000, 0.960};
			// {3.406493798614640},
			// {3.334921288426460},
			// {3.817400948289294},
			// {3.810885723818021},
			// {4.090369787684238},
			// {3.887171207960874},
			// {2.623156495522141},
			// {2.520843112928688},
			// IFSYS_Center Stehle DWM Position I3 {2.000, 3.000, 0.920};
			// {3.419502887847882},
			// {3.325071427804221},
			// {3.826244895455595},
			// {3.802847617246845},
			// {4.098527174485976},
			// {3.879136501851926},
			// {2.639952651090546},
			// {2.508435767565118},
			// IFSYS_Center Rollwagen mit Stehle DWM Position I1 {2.000, 3.000, 1.080};
			{3.370014836762592},
			{3.367150130303073},
			{3.79327694744267},
			{3.83740146453300},
			{4.06815990344529},
			{3.91363002850295},
			{2.57583966892351},
			{2.56145466483403},
		};
		Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> target_distance_fix_point;
		Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> sum_actual_distances_fix_point;
		Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> deviation_distances_fix_point;
		Vector<uint16_t, LOCODECK_NR_OF_TWR_ANCHORS> counter_measurements_fix_point;
		double sum_deviation_distances_fix_point = 0.0;
		double sum_abs_deviation_distances_fix_point = 0.0;

		int measurements_good = 0;

		/*static constexpr*/ const double x_0_data[3] = {3.500, 3.000, 0.160}; // Cage // {2.000, 3.000, 1.242}; // doppelter Rollwagen // {1.0, 1.0, 1.0}; // {2.45*10, 4.10*10, 1.0}; // == calibration point
		Vector3d x_N;
		/*static constexpr*/ const double vel_N_data[3] = {0.0, 0.0, 0.0};
		Vector3d vel_N;

		SquareMatrix<double, RMA_WINDOW_SIZE> window_rma;
		Vector3d sum_rma;
		Vector3d x_N_rma_minus_1;

		Vector3d x_N_ema_minus_1;

		uint8_t gps_hor_speed_drift_delay = 2*SENSOR_INTERVAL_HZ;
		uint16_t Schedule_Counter = 0;

		/*static constexpr*/ const double lla_0_data[3] = {52.515391059955704, 13.323944815389625, 60.000000000000000}; // Drone-Cage ILR // {47.397742, 8.545594, 488.003000}; // Zürich
		Vector3d lla_0;

		bool check_data = false;
		bool check_inverse = false;

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
			uint8_t		anchors_used[LOCODECK_NR_OF_TWR_ANCHORS];
			double		ema_alpha;							// 0.3 Glättungsfaktor. 0.7 ist schon zu viel, es wird instabil
			bool		vel_ned_valid;
			uint32_t 	schedule_delayed;
		} custom_method_data = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, {1, 1, 1, 1, 1, 1, 1, 1}, 0.3, true, 183_ms};
};
