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


FakeGpsUwb::FakeGpsUwb() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
	// ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{

}


FakeGpsUwb::~FakeGpsUwb()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_fault_perf);

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);

    fclose(fp_distances);
    fclose(fp_hits);
}


int FakeGpsUwb::task_spawn(int argc, char *argv[])
{
	FakeGpsUwb *instance = new FakeGpsUwb();

	if (instance)
	{
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init())
		{
			return PX4_OK;
		}

	}
	else
	{
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}


int FakeGpsUwb::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int FakeGpsUwb::print_usage(const char *reason)
{
	PX4_INFO_RAW("\n");

	if (reason)
	{
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
	R"DESCR_STR(Description: Faking the GPS-Signal with position data provided by an UWB-Tag connected via I2C.)DESCR_STR"
	);
	PRINT_MODULE_USAGE_NAME("fake_gps_uwb", "driver");
	// PRINT_MODULE_USAGE_SUBCATEGORY("gps");
	PRINT_MODULE_USAGE_COMMAND("start");
	//PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	//PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x28);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	PX4_INFO_RAW("\n");

	return 0;
}

/*
extern "C" __EXPORT int fake_gps_uwb_main(int argc, char *argv[])
{
	return FakeGpsUwb::main(argc, argv);
}
*/

bool FakeGpsUwb::init()
{
	// int ret = I2C::init();

	// if (ret != PX4_OK) {
	// 	DEVICE_DEBUG("I2C::init failed (%i)", ret);
	// 	return ret;
	// }

	// if (ret == PX4_OK) {
	// 	ScheduleNow();
	// }

	// return ret;

	// execute Run() on every sensor_accel publication
	// if (!_sensor_accel_sub.registerCallback()) {
	// 	PX4_ERR("callback registration failed");
	// 	return false;
	// }

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate
	ScheduleOnInterval(SENSOR_INTERVAL_US);
	// ScheduleOnInterval(125_ms); // 8 Hz

	x_N = Vector3f(x_0_data);
	vel_N = Vector3f(vel_N_data);
	lla_0 = Vector3d(lla_0_data);

	measurements_good = 0;

    fp_distances = fopen("/home/rene/PX4-Autopilot/src/examples/fake_gps_uwb/distances_circle.txt", "r");
    if (fp_distances == NULL)
    {
        PX4_ERR("Error opening file: distances.txt\n");
        return false;
    }
    fp_hits = fopen("/home/rene/PX4-Autopilot/src/examples/fake_gps_uwb/hits_circle.bin", "rb");
    if (fp_hits == NULL)
    {
        PX4_ERR("Error opening file: hits.bin\n");
        return false;
    }

	return true;
}

void FakeGpsUwb::Run()
{
	if (should_exit())
	{
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	uint8_t anchors_used_sum = 0;
	check_data = false;

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

    uint8_t buffer_read_hits[LOCODECK_NR_OF_TWR_ANCHORS];
    int count_read_hits;
	int count_read_distances;

	Vector<float, LOCODECK_NR_OF_TWR_ANCHORS> measurement;

	perf_begin(_sample_perf);

	count_read_hits = fread(buffer_read_hits, sizeof(uint8_t), LOCODECK_NR_OF_TWR_ANCHORS, fp_hits);

	if (count_read_hits != LOCODECK_NR_OF_TWR_ANCHORS)
	{
		PX4_ERR("Error reading file: hits.bin\n");
		check_data = false;
		bool check = check_algorithm();
		PX4_INFO("check: %d", check);
		perf_end(_sample_perf);
		perf_count(_comms_errors);
	}
	else
	{
		Vector<uint8_t, LOCODECK_NR_OF_TWR_ANCHORS> anchors_used(buffer_read_hits);

		for (int i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++)
		{
			count_read_distances = fscanf(fp_distances, "%f", &measurement(i));
		}

		perf_end(_sample_perf);

		if (count_read_distances != 1)
		{
			PX4_ERR("Error reading file: distances.txt\n");
			check_data = false;
			bool check = check_algorithm();
			PX4_INFO("check: %d", check);
			perf_count(_comms_errors);
		}
		else
		{
			Matrix<float, LOCODECK_NR_OF_TWR_ANCHORS, 3> anchorPosition(anchorPosition_data);

			anchors_used_sum = anchors_used.dot(anchors_used);

			if (anchors_used_sum > 2) // >= 4
			{
				measurements_good++;

				// x_N = x_0;

				double delta_i = 10.0;
				uint8_t i = 0;
				Vector3f x_ccf_i_plus_1(x_N);

				while (delta_i > 0.001 && i < 100)
				{
					Vector3f x_ccf_i(x_ccf_i_plus_1);
					x_ccf_i_plus_1 = dme_least_squares(x_ccf_i, anchorPosition.transpose(), measurement, anchors_used);
					delta_i = Vector3f (x_ccf_i_plus_1 - x_ccf_i).norm();
					i++;
				}

				if (i == 100)
				{
					PX4_ERR("Error: No convergence of DME-Least-Squares-Algorithm\n");
					check_data = false;
					// bool check = check_algorithm();
					// PX4_INFO("check: %d", check);
					perf_count(_fault_perf);
				}
				else
				{
					check_data = true;
					vel_N = (x_ccf_i_plus_1 - x_N) / ((double)SENSOR_INTERVAL_US * 1e-6);
					x_N = x_ccf_i_plus_1;
					// PX4_INFO("x = %f, y = %f, z = %f", (double)x_N(0), (double)x_N(1), (double)x_N(2));
					// PX4_INFO("vel_N = %f, %f, %f", (double)vel_N(0), (double)vel_N(1), (double)vel_N(2));
					// bool check = check_algorithm();
					// PX4_INFO("check: %d", check);
					// PX4_INFO("measurements_good: %d", measurements_good);

				}
			}
			else
			{
				// perf_end(_sample_perf);
				check_data = false;
				// PX4_INFO("x = %f, y = %f, z = %f", (double)x_N(0), (double)x_N(1), (double)x_N(2));
				// PX4_INFO("vel_N = %f, %f, %f", (double)vel_N(0), (double)vel_N(1), (double)vel_N(2));
				// bool check = check_algorithm();
				// PX4_INFO("check: %d", check);
				perf_count(_fault_perf);
			}
		}
	}

	// Filterung
	// positionsausreisser filtern, wenn die position zu weit von der letzten position entfernt ist, mit gleitendem mittelwert oder tiefpass filtern
	// geschwindigkeitsausreisser filtern, wenn die geschwindigkeit zu weit von der letzten geschwindigkeit entfernt ist, mit gleitendem mittelwert oder tiefpass filtern

	double latitude = 0.0;
	double longitude = 0.0;
	double altitude = 0.0;

	double x_fe = (double)x_N(0);
	double y_fe = (double)-x_N(1);
	double z_fe = (double)-x_N(2);

	// double x_n = (double)x_N(1);
	// double x_e = (double)x_N(0);
	// double x_d = (double)-x_N(2);

	// add_vector_to_global_position(lla_0(0), lla_0(1), x_n, x_e, &latitude, &longitude);
	// altitude = lla_0(2) - x_d;

	double Psi = 0.0;
	flat_earth_to_lla(lla_0(0), lla_0(1), lla_0(2), x_fe, y_fe, z_fe, Psi, latitude, longitude, altitude);
	// flat_earth_to_lla(5.3, 5.3, 5.3, lla_0(0), lla_0(1), lla_0(2), Psi, latitude, longitude, altitude);
	// PX4_INFO("lat = %.15lf, lon = %.15lf, alt = %.15lf", latitude, longitude, altitude);

	Vector3f gps_vel = Vector3f(0.0, 0.0, 0.0);
	gps_vel(0) =  vel_N(0);
	gps_vel(1) = -vel_N(1);
	gps_vel(2) = -vel_N(2);

	// device id
	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	device_id.devid_s.bus = 0;
	device_id.devid_s.address = 0;
	device_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;

	sensor_gps_s sensor_gps{};

	if (_sim_gps_used.get() >= 4)
	{
		// fix
		sensor_gps.s_variance_m_s = 0.4f;
		sensor_gps.c_variance_rad = 0.1f;
		sensor_gps.eph = 0.9f;
		sensor_gps.epv = 1.78f;
		sensor_gps.hdop = 0.7f;
		sensor_gps.vdop = 1.1f;

		sensor_gps.fix_type = 3; // 3D fix
	}
	else
	{
		// no fix
		sensor_gps.s_variance_m_s = 100.f;
		sensor_gps.c_variance_rad = 100.f;
		sensor_gps.eph = 100.f;
		sensor_gps.epv = 100.f;
		sensor_gps.hdop = 100.f;
		sensor_gps.vdop = 100.f;

		sensor_gps.fix_type = 0; // No fix
	}

	//
	sensor_gps.timestamp_sample = 0; // gpos.timestamp_sample;
	sensor_gps.latitude_deg = latitude; // Latitude in degrees
	sensor_gps.longitude_deg = longitude; // Longitude in degrees
	sensor_gps.altitude_msl_m = altitude; // Altitude in meters above MSL
	sensor_gps.altitude_ellipsoid_m = altitude;
	sensor_gps.time_utc_usec = hrt_absolute_time(); // hrt_absolute_time() + 1613692609599954; // 0;
	sensor_gps.device_id = device_id.devid;
	/*
	...
	*/
	sensor_gps.noise_per_ms = 0;
	sensor_gps.jamming_indicator = 0;
	sensor_gps.vel_m_s = sqrtf(gps_vel(0) * gps_vel(0) + gps_vel(1) * gps_vel(1)); // GPS ground speed, (metres/sec)
	sensor_gps.vel_n_m_s = gps_vel(0);
	sensor_gps.vel_e_m_s = gps_vel(1);
	sensor_gps.vel_d_m_s = gps_vel(2);
	sensor_gps.cog_rad = atan2(gps_vel(1), gps_vel(0)); // Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
	sensor_gps.timestamp_time_relative = 0;
	sensor_gps.heading = NAN;
	sensor_gps.heading_offset = NAN;
	sensor_gps.heading_accuracy = 0;
	//
	sensor_gps.automatic_gain_control = 0;
	//
	sensor_gps.jamming_state = 0;
	sensor_gps.spoofing_state = 0;
	sensor_gps.vel_ned_valid = true;
	sensor_gps.satellites_used = _sim_gps_used.get();
	/*
	...
	*/
	sensor_gps.timestamp = hrt_absolute_time();

	_sensor_gps_pub.publish(sensor_gps);

	// listener(ORB_ID(sensor_gps), 1 , -1, 0);

	perf_end(_loop_perf);
}

Vector3f FakeGpsUwb::dme_least_squares(const Vector3f &x_ccf_i, const Matrix<float, 3, LOCODECK_NR_OF_TWR_ANCHORS> &anchorPosition, const Vector<float, LOCODECK_NR_OF_TWR_ANCHORS> &z, const Vector<uint8_t, LOCODECK_NR_OF_TWR_ANCHORS> &anchorUsed)
{
	Vector<float, LOCODECK_NR_OF_TWR_ANCHORS> rho_ccf;
    Matrix<float, LOCODECK_NR_OF_TWR_ANCHORS, 3> H;

    for (uint8_t i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++)
    {
        if (anchorUsed(i) == 1)
        {
            rho_ccf(i) = Vector3f (x_ccf_i - anchorPosition.col(i)).norm();

            H(i, 0) = (x_ccf_i(0) - anchorPosition(0, i)) / (x_ccf_i - anchorPosition.col(i)).norm() ;
            H(i, 1) = (x_ccf_i(1) - anchorPosition(1, i)) / (x_ccf_i - anchorPosition.col(i)).norm() ;
            H(i, 2) = (x_ccf_i(2) - anchorPosition(2, i)) / (x_ccf_i - anchorPosition.col(i)).norm() ;
        }
        else
        {
            rho_ccf(i) = 0.0;

            H(i, 0) = 0.0;
            H(i, 1) = 0.0;
            H(i, 2) = 0.0;
        }

    }

    Vector<float, LOCODECK_NR_OF_TWR_ANCHORS> Delta_rho_ccf = z - rho_ccf;

    SquareMatrix<float, 3> inv_H_T_H;
    check_inverse = false;
    check_inverse = inv(SquareMatrix<float, 3> (H.transpose() * H), inv_H_T_H);

    if (check_inverse == false)
    {
        return x_ccf_i;
    }

    Vector3f Delta_x_ccf = (inv_H_T_H * H.transpose()) * Delta_rho_ccf;

    Vector3f x_ccf_i_plus_1(x_ccf_i + Delta_x_ccf);

    return x_ccf_i_plus_1;
}


void FakeGpsUwb::flat_earth_to_lla(double lat_ref, double lon_ref, double alt_ref, double x_fe, double y_fe, double z_fe, double Psi, double &lat, double &lon, double &alt)
{
	// Nach Matlab Block "Flat Earth to LLA"

	double transformation[] = {cos(math::radians(Psi)), -sin(math::radians(Psi)), sin(math::radians(Psi)), cos(math::radians(Psi))};
	Vector2d N_and_E = SquareMatrix<double, 2>(transformation) * Vector2d(x_fe, y_fe);

	double R = 6378137.0;
	double f = 1.0 / 298.257223563;
	double e2 = 2.0 * f - f * f;
	double R_N = R / sqrt(1.0 - e2 * sin(math::radians(lat_ref)) * sin(math::radians(lat_ref)));
	double R_M = R_N * (1.0 - e2) / (1.0 - e2 * sin(math::radians(lat_ref)) * sin(math::radians(lat_ref)));

	double dN = N_and_E(0);
	double dE = N_and_E(1);
	double dlat = math::degrees( atan2(1, R_M) * dN );
	double dlon = math::degrees( atan2(1, (R_N * cos(math::radians(lat_ref)))) * dE );

	lat = lat_ref + dlat;
	lon = lon_ref + dlon;

	alt = alt_ref - z_fe;
}


bool FakeGpsUwb::check_algorithm()
{
	if (check_data == true && check_inverse == true)
	{
		return true;
	}
	else
	{
		return false;
	}

	// uint64_t x = perf_event_count(_comms_errors);
}


int FakeGpsUwb::print_status()
{
	// I2CSPIDriverBase::print_status();

	PX4_INFO_RAW("\nDriver Counters:\n");
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_fault_perf);

	PX4_INFO_RAW("\nModule Counters:\n");
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	PX4_INFO_RAW("\n");

	return 0;
}

