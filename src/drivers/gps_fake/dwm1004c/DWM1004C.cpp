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


DWM1004C::DWM1004C(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{

}


DWM1004C::~DWM1004C()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_fault_perf);

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}


int DWM1004C::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK)
	{
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK)
	{
		ScheduleNow();
	}

	x_N = Vector3f(x_0_data);
	vel_N = Vector3f(vel_N_data);
	lla_0 = Vector3d(lla_0_data);

	anchors_used_sum = 0;
	measurements_good = 0;
	dwm_errors = 0;

	return ret;
}

void DWM1004C::RunImpl()
{
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// uint8_t anchors_used_sum = 0;

	// Nach PX4 "MS4525DO.cpp"

	switch (_state)
	{
		case STATE::MEASURE:
			{
				// Send the command to begin a measurement (Read_MR)
				uint8_t cmd = ADDR_READ_MR;

				if (dwm_errors == MAX_ERRORS)
				{
					cmd = DW_RESET;
					transfer(&cmd, 1, nullptr, 0);
					dwm_errors = 0;
					ScheduleDelayed(10_ms); // try again in 10 ms
					cmd = ADDR_READ_MR;
				}

				if (transfer(&cmd, 1, nullptr, 0) == OK)
				{
					_timestamp_sample = hrt_absolute_time();
					_state = STATE::READ;
					ScheduleDelayed(2_ms);

				}
				else
				{
					perf_count(_comms_errors);
					dwm_errors++;
					_state = STATE::MEASURE;
					ScheduleDelayed(10_ms); // try again in 10 ms
					anchors_used_sum = 0;
					// _state = STATE::PUBLISH;
					// ScheduleDelayed(2_ms);
				}
			}

			break;

		case STATE::READ:
			{
				// perform 2 x Read_DF4 (Data Fetch 4 Bytes)
				//  1st read: require status = Normal Operation. Good Data Packet
				//  2nd read: require status = Stale Data, data should match first read
				perf_begin(_sample_perf);
				uint8_t data_1[DATA_RX_LENGTH] {};
				int ret1 = transfer(nullptr, 0, &data_1[0], sizeof(data_1));

				uint8_t data_2[DATA_RX_LENGTH] {};
				int ret2 = transfer(nullptr, 0, &data_2[0], sizeof(data_2));
				perf_end(_sample_perf);

				if (ret1 != PX4_OK || ret2 != PX4_OK)
				{
					perf_count(_comms_errors);
					dwm_errors++;
				}
				else
				{
					// Status bits
					const uint8_t status_1 = data_1[0];
					const uint8_t status_2 = data_2[0];

					// Anchors used bits
					const uint8_t anchors_used_data_1 = data_1[1];
					const uint8_t anchors_used_data_2 = data_2[1];

					Vector<uint8_t, LOCODECK_NR_OF_TWR_ANCHORS> anchors_used;
					for (int8_t i = LOCODECK_NR_OF_TWR_ANCHORS-1; i >= 0; i--) // Vergleich mit Code bei DWM1004C: Byte wird von vorn order hinten befüllt??
					{
						anchors_used(i) = (anchors_used_data_1 >> i) & 0x01;
					}

					// 8 x 4 Byte Distance
					// Distance_1  data[2:5],  Distance_2 data [6:9],  Distance_3 data[10:13], Distance_4 data[14:17],
					// Distance_5 data[18:21], Distance_6 data[22:25], Distance_7 data[26:29], Distance_8 data[30:33]
					Vector<float, LOCODECK_NR_OF_TWR_ANCHORS> measurement_1;
					Vector<float, LOCODECK_NR_OF_TWR_ANCHORS> measurement_2;
					for (uint8_t i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++)
					{
						memcpy(&measurement_1(i), data_1+(2+i*sizeof(float)), sizeof(float));
						memcpy(&measurement_2(i), data_2+(2+i*sizeof(float)), sizeof(float));
					}

					// Checksum
					const uint16_t checksum_1 = (data_1[DATA_RX_LENGTH-1] << 8) + data_1[DATA_RX_LENGTH-2];
					const uint16_t checksum_2 = (data_2[DATA_RX_LENGTH-1] << 8) + data_2[DATA_RX_LENGTH-2];
					// const uint8_t checksum_1_msb = data_1[DATA_RX_LENGTH-2];
					// const uint8_t checksum_2_msb = data_2[DATA_RX_LENGTH-2];

					// const uint8_t checksum_1_lsb = data_1[DATA_RX_LENGTH-1];
					// const uint8_t checksum_2_lsb = data_2[DATA_RX_LENGTH-1];
					// uint16_t checksum_1 = (checksum_1_msb << 8) + checksum_1_lsb;
					// uint16_t checksum_2 = (checksum_2_msb << 8) + checksum_2_lsb;

					uint16_t checksum_RX_1 = 0;
					uint16_t checksum_RX_2 = 0;
					for (uint8_t i = 1; i < DATA_RX_LENGTH-2; i++)
					{
						checksum_RX_1 += data_1[i];
						checksum_RX_2 += data_2[i];
					}

					// uint32_t measurement_end_1 = measurement_1(LOCODECK_NR_OF_TWR_ANCHORS);
					// uint32_t measurement_end_2 = measurement_2(LOCODECK_NR_OF_TWR_ANCHORS);

					PX4_INFO_RAW("status:%X|%X, anchors_used:%X|%X, checksum:%X|%X, checksum_RX:%X|%X\n", status_1, status_2, anchors_used_data_1, anchors_used_data_2, checksum_1, checksum_2, checksum_RX_1, checksum_RX_2); // LOCODECK_NR_OF_TWR_ANCHORS,
					for (uint8_t i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++)
					{
						PX4_INFO_RAW("anchors_used[%d] = %d\n", i, anchors_used(i));
					}
					for (uint8_t i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++)
					{
						PX4_INFO_RAW("measurement_1[%d] = %f\n", i, (double)measurement_1(i));
					}

					if ((status_1 == (uint8_t)STATUS::Fault_Detected) || (status_2 == (uint8_t)STATUS::Fault_Detected))
					{
						// Fault Detected
						perf_count(_fault_perf);
						dwm_errors++;

						PX4_INFO_RAW("Fault Detected\n");
					}
					else if ((status_1 == (uint8_t)STATUS::Normal_Operation) && (status_2 == (uint8_t)STATUS::Stale_Data)
						&& (anchors_used_data_1 == anchors_used_data_2) && (checksum_1 == checksum_RX_2)) // && (measurement_end_1 == measurement_end_2)
					{
						check_data = false;

						// PX4_INFO_RAW("checksum: %X|%X, checksum_RX: %X|%X\n", checksum_1, checksum_2, checksum_RX_1, checksum_RX_2); // LOCODECK_NR_OF_TWR_ANCHORS,

						Matrix<float, LOCODECK_NR_OF_TWR_ANCHORS, 3> anchorPosition(anchorPosition_data);

						anchors_used_sum = anchors_used.dot(anchors_used);

						if (anchors_used_sum > 2) // >= 4
						{
							measurements_good++;

							PX4_INFO_RAW("anchors_used_sum = %d\n", anchors_used_sum);

							double delta_i = 10.0;
							uint8_t i = 0;
							Vector3f x_ccf_i_plus_1(x_N);

							while (delta_i > 0.001 && i < 100)
							{
								Vector3f x_ccf_i(x_ccf_i_plus_1);
								x_ccf_i_plus_1 = dme_least_squares(x_ccf_i, anchorPosition.transpose(), measurement_1, anchors_used);
								delta_i = Vector3f (x_ccf_i_plus_1 - x_ccf_i).norm();
								i++;
							}

							if (i == 100)
							{
								// PX4_ERR("Error: No convergence of DME-Least-Squares-Algorithm\n");
								PX4_INFO_RAW("Error: No convergence of DME-Least-Squares-Algorithm\n");
								check_data = false;
								perf_count(_fault_perf);
								dwm_errors++;
							}
							else
							{
								check_data = true;
								vel_N = (x_ccf_i_plus_1 - x_N) / ((double)SENSOR_INTERVAL_US * 1e-6);
								x_N = x_ccf_i_plus_1;
								dwm_errors = 0;
							}
						}
						else
						{
							check_data = false;
							perf_count(_fault_perf);
							dwm_errors++;

							PX4_INFO_RAW("Error: Not enough anchors used\n");
							// PX4_INFO_RAW("dwm_errors = %d\n", dwm_errors);
						}
					}
					else
					{
						PX4_DEBUG("status:%X|%X, anchors_used:%X|%X, checksum:%X|%X, checksum_RX:%X|%X", status_1, status_2,
							anchors_used_data_1, anchors_used_data_2, checksum_1, checksum_2, checksum_RX_1, checksum_RX_2); // LOCODECK_NR_OF_TWR_ANCHORS,
							//(double)measurement_1(LOCODECK_NR_OF_TWR_ANCHORS), (double)measurement_2(LOCODECK_NR_OF_TWR_ANCHORS));
						dwm_errors++;

						PX4_INFO_RAW("Error: Status, Anchors, Checksum or Measurement not OK\n");
					}
				}

				_state = STATE::PUBLISH;
				ScheduleDelayed(2_ms);

				break;
			}
		case STATE::PUBLISH:
			{
				double latitude = 0.0;
				double longitude = 0.0;
				double altitude = 0.0;

				double x_fe = (double)x_N(0);
				double y_fe = (double)-x_N(1);
				double z_fe = (double)-x_N(2);

				double Psi = 0.0;
				flat_earth_to_lla(lla_0(0), lla_0(1), lla_0(2), x_fe, y_fe, z_fe, Psi, latitude, longitude, altitude);

				Vector3f gps_vel = Vector3f(0.0, 0.0, 0.0);
				gps_vel(0) =  vel_N(0);
				gps_vel(1) = -vel_N(1);
				gps_vel(2) = -vel_N(2);

				// device id
				device::Device::DeviceId device_id;
				device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_I2C; // DeviceBusType_UNKNOWN
				device_id.devid_s.bus = 0;
				device_id.devid_s.address = 0;
				device_id.devid_s.devtype = DRV_GPS_DEVTYPE_DWM1004C;

				sensor_gps_s sensor_gps{};

				if (anchors_used_sum >= 4)
				{
					// fix
					sensor_gps.s_variance_m_s = 0.4f;
					sensor_gps.c_variance_rad = 0.1f;
					sensor_gps.eph = 0.9f;				//0.3f
					sensor_gps.epv = 1.78f;				//0.3f
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
				sensor_gps.timestamp_sample = _timestamp_sample; // = 0 // gpos.timestamp_sample;
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
				sensor_gps.satellites_used = anchors_used_sum;
				/*
				...
				*/
				sensor_gps.timestamp = hrt_absolute_time();

				// if (hrt_elapsed_time(&_timestamp_sample) < 20_ms)
				// {
				//
				// }
				// _sensor_gps_pub.publish(sensor_gps);

				PX4_INFO_RAW("sensor_gps.satellites_used = %d\n", sensor_gps.satellites_used);

				_state = STATE::MEASURE;
				// ScheduleDelayed(10_ms);

				ScheduleDelayed(5000_ms);

				break;
			}
	}


	// // Reset von DWM1004C ermöglichen
	// // Was passiert, wenn der GPS-Signal kacke ist oder keins vorhanden ist?
	// // Standardabweichung der Messung, Rauschen, Messunsicherheit
	// // Filterung
	// // positionsausreisser filtern, wenn die position zu weit von der letzten position entfernt ist, mit gleitendem mittelwert oder tiefpass filtern
	// // geschwindigkeitsausreisser filtern, wenn die geschwindigkeit zu weit von der letzten geschwindigkeit entfernt ist, mit gleitendem mittelwert oder tiefpass filtern


	// double latitude = 0.0;
	// double longitude = 0.0;
	// double altitude = 0.0;

	// double x_fe = (double)x_N(0);
	// double y_fe = (double)-x_N(1);
	// double z_fe = (double)-x_N(2);

	// double Psi = 0.0;
	// flat_earth_to_lla(lla_0(0), lla_0(1), lla_0(2), x_fe, y_fe, z_fe, Psi, latitude, longitude, altitude);

	// Vector3f gps_vel = Vector3f(0.0, 0.0, 0.0);
	// gps_vel(0) =  vel_N(0);
	// gps_vel(1) = -vel_N(1);
	// gps_vel(2) = -vel_N(2);

	// // device id
	// device::Device::DeviceId device_id;
	// device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_I2C; // DeviceBusType_UNKNOWN
	// device_id.devid_s.bus = 0;
	// device_id.devid_s.address = 0;
	// device_id.devid_s.devtype = DRV_GPS_DEVTYPE_DWM1004C;

	// sensor_gps_s sensor_gps{};

	// if (anchors_used_sum >= 4)
	// {
	// 	// fix
	// 	sensor_gps.s_variance_m_s = 0.4f;
	// 	sensor_gps.c_variance_rad = 0.1f;
	// 	sensor_gps.eph = 0.9f;				//0.3f
	// 	sensor_gps.epv = 1.78f;				//0.3f
	// 	sensor_gps.hdop = 0.7f;
	// 	sensor_gps.vdop = 1.1f;

	// 	sensor_gps.fix_type = 3; // 3D fix
	// }
	// else
	// {
	// 	// no fix
	// 	sensor_gps.s_variance_m_s = 100.f;
	// 	sensor_gps.c_variance_rad = 100.f;
	// 	sensor_gps.eph = 100.f;
	// 	sensor_gps.epv = 100.f;
	// 	sensor_gps.hdop = 100.f;
	// 	sensor_gps.vdop = 100.f;

	// 	sensor_gps.fix_type = 0; // No fix
	// }

	// //
	// sensor_gps.timestamp_sample = _timestamp_sample; // = 0 // gpos.timestamp_sample;
	// sensor_gps.latitude_deg = latitude; // Latitude in degrees
	// sensor_gps.longitude_deg = longitude; // Longitude in degrees
	// sensor_gps.altitude_msl_m = altitude; // Altitude in meters above MSL
	// sensor_gps.altitude_ellipsoid_m = altitude;
	// sensor_gps.time_utc_usec = hrt_absolute_time(); // hrt_absolute_time() + 1613692609599954; // 0;
	// sensor_gps.device_id = device_id.devid;
	// /*
	// ...
	// */
	// sensor_gps.noise_per_ms = 0;
	// sensor_gps.jamming_indicator = 0;
	// sensor_gps.vel_m_s = sqrtf(gps_vel(0) * gps_vel(0) + gps_vel(1) * gps_vel(1)); // GPS ground speed, (metres/sec)
	// sensor_gps.vel_n_m_s = gps_vel(0);
	// sensor_gps.vel_e_m_s = gps_vel(1);
	// sensor_gps.vel_d_m_s = gps_vel(2);
	// sensor_gps.cog_rad = atan2(gps_vel(1), gps_vel(0)); // Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
	// sensor_gps.timestamp_time_relative = 0;
	// sensor_gps.heading = NAN;
	// sensor_gps.heading_offset = NAN;
	// sensor_gps.heading_accuracy = 0;
	// //
	// sensor_gps.automatic_gain_control = 0;
	// //
	// sensor_gps.jamming_state = 0;
	// sensor_gps.spoofing_state = 0;
	// sensor_gps.vel_ned_valid = true;
	// sensor_gps.satellites_used = anchors_used_sum;
	// /*
	// ...
	// */
	// sensor_gps.timestamp = hrt_absolute_time();

	// // if (hrt_elapsed_time(&_timestamp_sample) < 20_ms)
	// // {
	// //
	// // }
	// // _sensor_gps_pub.publish(sensor_gps);

	// PX4_INFO_RAW("sensor_gps.satellites_used = %d\n", sensor_gps.satellites_used);

	perf_end(_loop_perf);
}

Vector3f DWM1004C::dme_least_squares(const Vector3f &x_ccf_i, const Matrix<float, 3, LOCODECK_NR_OF_TWR_ANCHORS> &anchorPosition, const Vector<float, LOCODECK_NR_OF_TWR_ANCHORS> &z, const Vector<uint8_t, LOCODECK_NR_OF_TWR_ANCHORS> &anchorUsed)
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

void DWM1004C::flat_earth_to_lla(double lat_ref, double lon_ref, double alt_ref, double x_fe, double y_fe, double z_fe, double Psi, double &lat, double &lon, double &alt)
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

bool DWM1004C::check_algorithm()
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

void DWM1004C::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO_RAW("\nDriver Counters:\n");
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_fault_perf);

	PX4_INFO_RAW("\nModule Counters:\n");
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	PX4_INFO_RAW("\n");
}


int DWM1004C::probe()
{
	// Nach PX4 "MS4525DO.cpp"

	_retries = 1;

	for (int i = 0; i < 10; i++)
	{
		// perform 2 x Read_DF3 (Data Fetch 3 Bytes)
		//  1st read: require status = Normal Operation. Good Data Packet
		//  2nd read: require status = Stale Data, data should match first read
		uint8_t data_1[3] {};
		int ret1 = transfer(nullptr, 0, &data_1[0], sizeof(data_1));

		uint8_t data_2[3] {};
		int ret2 = transfer(nullptr, 0, &data_2[0], sizeof(data_2));

		if (ret1 == PX4_OK && ret2 == PX4_OK)
		{
			// Status bits
			const uint8_t status_1 = data_1[0];
			const uint8_t status_2 = data_2[0];

			if ((status_1 == (uint8_t)STATUS::Normal_Operation)
			    && (status_2 == (uint8_t)STATUS::Stale_Data)
			    && (data_1[2] == data_1[2]))
			{
				_retries = 1; // enable retries during operation
				return PX4_OK;
			}
			else
			{
				PX4_ERR("status: %X status: %X", status_1, status_2);
			}
		}
		else
		{
			px4_usleep(1000); // TODO
		}
	}

	return PX4_ERROR;
}
