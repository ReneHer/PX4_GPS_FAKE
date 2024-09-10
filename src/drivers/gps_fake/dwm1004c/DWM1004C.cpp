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


// void DWM1004C::print_usage()
// {
// 	PX4_INFO_RAW("\n");

// 	PRINT_MODULE_USAGE_NAME("dwm1004c", "driver");
// 	// PRINT_MODULE_USAGE_SUBCATEGORY("gps");
// 	PRINT_MODULE_USAGE_COMMAND("start");
// 	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
// 	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(I2C_ADDRESS_DEFAULT); // 0x28);
// 	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

// 	PX4_INFO_RAW("\n");
// }


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
		// alternatively, Run on fixed interval
		// ScheduleOnInterval(SENSOR_INTERVAL_US/3);//, SENSOR_INTERVAL_US/3);		// TODO: checken
	}

	// anchorPosition = Matrix<double, LOCODECK_NR_OF_TWR_ANCHORS, 3>(anchorPosition_data);
	x_N = Vector3d(x_0_data);
	vel_N = Vector3d(vel_N_data);
	lla_0 = Vector3d(lla_0_data);

	anchors_used_sum = 0;
	dwm_errors = 0;
	measurements_good = 0;
	max_distance = 0.0;

	Matrix<double, LOCODECK_NR_OF_TWR_ANCHORS, 3> anchorPosition(anchorPosition_data);
	for (uint8_t i = 1; i < LOCODECK_NR_OF_TWR_ANCHORS; i++)
	{
        Vector3d anchorPosition_1 = anchorPosition.row(0);
		Vector3d anchorPosition_2 = anchorPosition.row(i);
		if ((anchorPosition_1 - anchorPosition_2).norm() > max_distance)
		{
			max_distance = (anchorPosition_1 - anchorPosition_2).norm();
		}
	}

	for (uint8_t i = 0; i < RMA_WINDOW_SIZE; i++)
	{
		window_rma.col(i) = x_N;
	}
	sum_rma.setZero();
	x_N_rma_minus_1 = x_N;

	x_N_ema_minus_1 = x_N;

	Schedule_Counter = 0;

	#ifdef DEBUG_MODE_ON
	target_distance_fix_point = Vector<double, LOCODECK_NR_OF_TWR_ANCHORS>(target_distance_fix_point_data);
	sum_actual_distances_fix_point.setZero();
	deviation_distances_fix_point.setZero();
	counter_measurements_fix_point.setZero();
	sum_deviation_distances_fix_point = 0.0;
	#endif

	return ret;
}

void DWM1004C::RunImpl()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	// perf_count(_loop_interval_perf);

	// uint8_t anchors_used_sum = 0;

	// Nach PX4 "MS4525DO.cpp"

	switch (_state)
	{
		case STATE::MEASURE:
			{
				// Send the command to begin a measurement (Read_MR)
				uint8_t cmd = DWM1004C_TRANSMIT;

				anchors_used_sum = 0;

				if (dwm_errors == MAX_ERRORS)
				{
					PX4_WARN("DWM1004C_RESET");

					cmd = DWM1004C_RESET;
					transfer(&cmd, 1, nullptr, 0);
					dwm_errors = 0;
					ScheduleDelayed(10_ms); // try again in 10 ms
					cmd = DWM1004C_TRANSMIT;
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
					// _state = STATE::MEASURE;
					// ScheduleDelayed(10_ms); // try again in 10 ms
					_state = STATE::PUBLISH;
					ScheduleDelayed(2_ms);
				}
			}

			break;

		case STATE::READ:
			{
				// PX4_INFO_RAW("STATE == READ\n");

				// perform 2 x Read_DF4 (Data Fetch 4 Bytes)
				//  1st read: require status = Normal Operation. Good Data Packet
				//  2nd read: require status = Stale Data, data should match first read
				perf_begin(_sample_perf);
				uint8_t data_1[DWM1004C_DATA_LENGTH] {};
				int ret1 = transfer(nullptr, 0, &data_1[0], sizeof(data_1));

				uint8_t data_2[DWM1004C_DATA_LENGTH] {};
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
					for (int8_t i = LOCODECK_NR_OF_TWR_ANCHORS-1; i >= 0; i--) // TODO: Vergleich mit Code bei DWM1004C: Byte wird von vorn order hinten befüllt??
					{
						anchors_used(i) = (anchors_used_data_1 >> i) & 0x01;
					}

					// 8 x 4 Byte Distance
					// Distance_1  data[2:5],  Distance_2 data [6:9],  Distance_3 data[10:13], Distance_4 data[14:17],
					// Distance_5 data[18:21], Distance_6 data[22:25], Distance_7 data[26:29], Distance_8 data[30:33]
					Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> measurement_1;
					Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> measurement_2;
					for (uint8_t i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++)
					{
						memcpy(&measurement_1(i), data_1+(2+i*sizeof(double)), sizeof(double));
						memcpy(&measurement_2(i), data_2+(2+i*sizeof(double)), sizeof(double));
					}

					// Checksum
					const uint16_t checksum_1 = (data_1[DWM1004C_DATA_LENGTH-1] << 8) + data_1[DWM1004C_DATA_LENGTH-2];
					const uint16_t checksum_2 = (data_2[DWM1004C_DATA_LENGTH-1] << 8) + data_2[DWM1004C_DATA_LENGTH-2];
					// const uint8_t checksum_1_msb = data_1[DWM1004C_DATA_LENGTH-2];
					// const uint8_t checksum_2_msb = data_2[DWM1004C_DATA_LENGTH-2];

					// const uint8_t checksum_1_lsb = data_1[DWM1004C_DATA_LENGTH-1];
					// const uint8_t checksum_2_lsb = data_2[DWM1004C_DATA_LENGTH-1];
					// uint16_t checksum_1 = (checksum_1_msb << 8) + checksum_1_lsb;
					// uint16_t checksum_2 = (checksum_2_msb << 8) + checksum_2_lsb;

					uint16_t checksum_RX_1 = 0;
					uint16_t checksum_RX_2 = 0;
					for (uint8_t i = 1; i < DWM1004C_DATA_LENGTH-2; i++)
					{
						checksum_RX_1 += data_1[i];
						checksum_RX_2 += data_2[i];
					}

					// uint32_t measurement_end_1 = measurement_1(LOCODECK_NR_OF_TWR_ANCHORS);
					// uint32_t measurement_end_2 = measurement_2(LOCODECK_NR_OF_TWR_ANCHORS);

					if (custom_method_data.received_data_counter < custom_method_data.received_data_print)
					{
						PX4_INFO("status:%X|%X, anchors_used:%X|%X, checksum:%X|%X, checksum_RX:%X|%X",
								status_1, status_2, anchors_used_data_1, anchors_used_data_2, checksum_1, checksum_2, checksum_RX_1, checksum_RX_2);
						for (uint8_t i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++)
						{
							// PX4_INFO("anchors_used[%d] = %d\n", i, anchors_used(i));
							PX4_INFO("measurement_1[%d] = %f", i, (double)measurement_1(i));
						}
						custom_method_data.received_data_counter++;
					}

					sum_deviation_distances_fix_point = 0.0;

					for (uint8_t i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++)
					{
						if (measurement_1(i) < 0.0 || measurement_1(i) > max_distance)
						{
							anchors_used(i) = 0;
							measurement_1(i) = 0.0;
							// Von der vorherigen Messung übernehmen oder 0.0 setzen? Ersteres lieferte schlechtere Ergebnisse bei einem Desktoptest.
						}

						if (anchors_used(i) == 1)
						{
							counter_measurements_fix_point(i)++;
							sum_actual_distances_fix_point(i) = sum_actual_distances_fix_point(i) + measurement_1(i);
						// }
						deviation_distances_fix_point(i) = ( sum_actual_distances_fix_point(i) / counter_measurements_fix_point(i) ) - target_distance_fix_point(i);
						sum_deviation_distances_fix_point = sum_deviation_distances_fix_point + deviation_distances_fix_point(i);
						}
					}

					if (custom_method_data.deviations_counter < custom_method_data.deviations_print)
					{
						// PX4_INFO("max_distance = %f\n", max_distance);
						for (uint8_t i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++)
						{
							// PX4_INFO("anchors_used[%d] = %d\n", i, anchors_used(i));
							// PX4_INFO("measurement_1[%d] = %f\n", i, (double)measurement_1(i));
							PX4_INFO("deviation_distances_fix_point[%d] = %f", i, (double)deviation_distances_fix_point(i));
						}
						custom_method_data.deviations_counter++;
					}

					if (custom_method_data.sum_deviations_counter < custom_method_data.sum_deviations_print)
					{
						PX4_INFO("sum_deviation_distances_fix_point = %f", sum_deviation_distances_fix_point);
						custom_method_data.sum_deviations_counter++;
					}

					if ((status_1 == (uint8_t)STATUS::Fault_Detected) || (status_2 == (uint8_t)STATUS::Fault_Detected))
					{
						// Fault Detected
						perf_count(_fault_perf);
						dwm_errors++;

						PX4_INFO("Fault Detected");
					}
					else if ((status_1 == (uint8_t)STATUS::Normal_Operation) && (status_2 == (uint8_t)STATUS::Stale_Data)
						&& (anchors_used_data_1 == anchors_used_data_2) && (checksum_1 == checksum_RX_2)) // && (measurement_end_1 == measurement_end_2)
					{
						// check_data = false;

						// PX4_INFO_RAW("checksum: %X|%X, checksum_RX: %X|%X\n", checksum_1, checksum_2, checksum_RX_1, checksum_RX_2);

						Matrix<double, LOCODECK_NR_OF_TWR_ANCHORS, 3> anchorPosition(anchorPosition_data);

						anchors_used_sum = anchors_used.dot(anchors_used);

						if (anchors_used_sum > 2) // >= 4
						{
							measurements_good++;

							// PX4_INFO_RAW("anchors_used_sum = %d\n", anchors_used_sum);

							double delta_i = 10.0;
							uint8_t i = 0;
							Vector3d x_ccf_i_plus_1(x_N);

							// measurement_1 = target_distance_fix_point;
							// PX4_INFO_RAW("x_ccf_i_plus_1 = %f, %f, %f\n", (double)x_ccf_i_plus_1(0), (double)x_ccf_i_plus_1(1), (double)x_ccf_i_plus_1(2));
							// PX4_INFO_RAW("anchors_used = %d, %d, %d, %d, %d, %d, %d, %d\n", anchors_used(0), anchors_used(1), anchors_used(2), anchors_used(3), anchors_used(4), anchors_used(5), anchors_used(6), anchors_used(7));
							// PX4_INFO_RAW("measurement_1 = %f, %f, %f, %f, %f, %f, %f, %f\n", (double)measurement_1(0), (double)measurement_1(1), (double)measurement_1(2), (double)measurement_1(3), (double)measurement_1(4), (double)measurement_1(5), (double)measurement_1(6), (double)measurement_1(7));
							// PX4_INFO_RAW("anchorPosition = %f, %f, %f, %f, %f, %f, %f, %f\n", (double)anchorPosition(0, 0), (double)anchorPosition(0, 1), (double)anchorPosition(0, 2), (double)anchorPosition(1, 0), (double)anchorPosition(1, 1), (double)anchorPosition(1, 2), (double)anchorPosition(2, 0), (double)anchorPosition(2, 1));

							while (delta_i > 0.001 && i < 100)
							{
								Vector3d x_ccf_i(x_ccf_i_plus_1);
								x_ccf_i_plus_1 = dme_least_squares(x_ccf_i, anchorPosition.transpose(), measurement_1, anchors_used);
								delta_i = Vector3d (x_ccf_i_plus_1 - x_ccf_i).norm();
								i++;
							}

							if (i < 100) // ((i < 100) || (check_inverse == false)) // TODO: checken
							{
								double Delta_x = (x_ccf_i_plus_1 - x_N).norm();
								if (x_ccf_i_plus_1(0) > 0.0 && x_ccf_i_plus_1(1) > 0.0 && x_ccf_i_plus_1(2) > 0.0 && Delta_x < max_distance)
								{
									// check_data = true;

									hrt_abstime _time_velocity_loop = hrt_elapsed_time(&_timestamp_velocity);
									_timestamp_velocity = hrt_absolute_time();

									// No Filter
									// vel_N.setZero();
									// vel_N = (x_ccf_i_plus_1 - x_N) / ((double)_time_velocity_loop * 1e-6); // TODO: Filtern: max 0.1 m/s??
									// vel_N = (x_ccf_i_plus_1 - x_N) / ((double)SENSOR_INTERVAL_US * 1e-6);

									// Rolling Moving Average
									// Vector3d x_N_rma = rolling_moving_average(x_ccf_i_plus_1);
									// Vector3d vel_N_rma = (x_N_rma - x_N_rma_minus_1) / ((double)_time_velocity_loop * 1e-6);
									// x_N_rma_minus_1 = x_N_rma;
									// vel_N = vel_N_rma;

									// Exponential Moving Average/Low Pass Filter
									// Vector3d x_N_ema = EMA_ALPHA * x_ccf_i_plus_1 + (1 - EMA_ALPHA) * x_N_ema_minus_1;
									Vector3d x_N_ema = custom_method_data.ema_alpha * x_ccf_i_plus_1 + (1 - custom_method_data.ema_alpha) * x_N_ema_minus_1;
									Vector3d vel_N_ema = (x_N_ema - x_N_ema_minus_1) / ((double)_time_velocity_loop * 1e-6);
									x_N_ema_minus_1 = x_N_ema;
									if (Schedule_Counter < GPS_HOR_SPEED_DRIFT_DELAY)
									{
										Schedule_Counter++;
									}
									else
									{
										vel_N = vel_N_ema;
									}

									x_N = x_ccf_i_plus_1;
									dwm_errors = 0;

									if (custom_method_data.local_coordinates_counter < custom_method_data.local_coordinates_print)
									{
										PX4_INFO("x_N = %f, %f, %f", x_N(0), x_N(1), x_N(2));
										custom_method_data.local_coordinates_counter++;
									}

									if (custom_method_data.local_velocities_counter < custom_method_data.local_velocities_print)
									{
										PX4_INFO("vel_N = %f, %f, %f", vel_N(0), vel_N(1), vel_N(2));
										custom_method_data.local_velocities_counter++;
									}

									// PX4_INFO_RAW("Iterations: %d\n", i);
									// PX4_INFO_RAW("vel_N_rma = %f, %f, %f\n", vel_N_rma(0), vel_N_rma(1), vel_N_rma(2));
									// PX4_INFO_RAW("vel_N_ema = %f, %f, %f\n", vel_N_ema(0), vel_N_ema(1), vel_N_ema(2));
									// PX4_INFO_RAW("Time Velocity Loop: %ld us\n", SENSOR_INTERVAL_US);
									// PX4_INFO_RAW("Time Velocity Loop: %lld us\n", _time_velocity_loop);
								}
								else
								{
									// check_data = false;
									perf_count(_fault_perf);
									dwm_errors++;

									PX4_INFO("Error: Measurement failure");
								}
							}
							else
							{
								// check_data = false;
								perf_count(_fault_perf);
								dwm_errors++;

								PX4_INFO("Error: No convergence of DME-Least-Squares-Algorithm");
							}
						}
						else
						{
							// check_data = false;
							perf_count(_fault_perf);
							dwm_errors++;

							PX4_INFO("Error: Not enough anchors used");
						}
					}
					else
					{
						PX4_DEBUG("status:%X|%X, anchors_used:%X|%X, checksum:%X|%X, checksum_RX:%X|%X",
								status_1, status_2, anchors_used_data_1, anchors_used_data_2, checksum_1, checksum_2, checksum_RX_1, checksum_RX_2);

						perf_count(_comms_errors);
						dwm_errors++;

						PX4_INFO("Error: Status, Anchors, Checksum or Measurement not OK");
						// PX4_INFO_RAW("status:%X|%X, anchors_used:%X|%X, checksum:%X|%X, checksum_RX:%X|%X",
						// 		status_1, status_2, anchors_used_data_1, anchors_used_data_2, checksum_1, checksum_2, checksum_RX_1, checksum_RX_2);
					}
				}

				if (custom_method_data.found_errors_counter < custom_method_data.found_errors_print)
				{
					PX4_INFO("dwm_errors = %d", dwm_errors);
					custom_method_data.found_errors_counter++;
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

				double x_fe = x_N(0);
				double y_fe = -x_N(1);
				double z_fe = -x_N(2);

				double Psi = 0.0;
				flat_earth_to_lla(lla_0(0), lla_0(1), lla_0(2), x_fe, y_fe, z_fe, Psi, latitude, longitude, altitude);

				Vector3f gps_vel = Vector3f(0.0, 0.0, 0.0);
				gps_vel(0) = (float)vel_N(0);
				gps_vel(1) = (float)-vel_N(1);
				gps_vel(2) = (float)-vel_N(2);

				// device id
				device::Device::DeviceId device_id;
				device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_I2C; // DeviceBusType_UNKNOWN
				device_id.devid_s.bus = 2; // 0;
				device_id.devid_s.address = I2C_ADDRESS_DEFAULT; // 0;
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
				sensor_gps.time_utc_usec = hrt_absolute_time() + 1725148800000000; // hrt_absolute_time(); //0;
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
				sensor_gps.timestamp_time_relative = 0; // TODO: nötig?
				sensor_gps.heading = NAN;
				sensor_gps.heading_offset = NAN;
				sensor_gps.heading_accuracy = 0;
				//
				sensor_gps.automatic_gain_control = 0;
				//
				sensor_gps.jamming_state = 0;
				sensor_gps.spoofing_state = 0;
				sensor_gps.vel_ned_valid = custom_method_data.vel_ned_valid; // TODO: Anstatt Tiefpass-Filter?
				sensor_gps.satellites_used = anchors_used_sum;
				/*
				...
				*/
				sensor_gps.timestamp = hrt_absolute_time();

				//if (hrt_elapsed_time(&_timestamp_sample) < 20_ms)
				//{
					_sensor_gps_pub.publish(sensor_gps);
					// PX4_INFO_RAW("elsapsed time = %lld\n", hrt_elapsed_time(&_timestamp_sample));	// TODO: Zeitmessung
					perf_count(_loop_interval_perf);
				//}

				// PX4_INFO_RAW("sensor_gps.satellites_used = %d\n", sensor_gps.satellites_used);

				_state = STATE::MEASURE;
				// ScheduleDelayed(10_ms);

				// listener(ORB_ID(sensor_gps), 1 , -1, 0);
				ScheduleDelayed(182_ms); // 183_ms // 5 Hz					// TODO: checken

				break;
			}
	}

	// Was passiert, wenn der GPS-Signal kacke ist oder keins vorhanden ist?
	// Standardabweichung der Messung, Rauschen, Messunsicherheit
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate
	// platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp
	// -> Zeile 63: "wq:I2C2" -> stacksizes -> 2336 -> 2656 (min. 300 bytes mehr) gegen "WARN  [load_mon] init low on stack!"

	// PX4_INFO_RAW("sensor_gps.satellites_used = %d\n", sensor_gps.satellites_used);

	// ScheduleDelayed(182_ms); // 183_ms // 5 Hz					// TODO: checken

	perf_end(_loop_perf);
}

Vector3d DWM1004C::dme_least_squares(const Vector3d &x_ccf_i, const Matrix<double, 3, LOCODECK_NR_OF_TWR_ANCHORS> &anchorPosition, const Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> &z, const Vector<uint8_t, LOCODECK_NR_OF_TWR_ANCHORS> &anchorUsed)
{
	Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> rho_ccf;
    Matrix<double, LOCODECK_NR_OF_TWR_ANCHORS, 3> H;

    for (uint8_t i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++)
    {
        if (anchorUsed(i) == 1)
        {
            rho_ccf(i) = Vector3d (x_ccf_i - anchorPosition.col(i)).norm();

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

    Vector<double, LOCODECK_NR_OF_TWR_ANCHORS> Delta_rho_ccf = z - rho_ccf;

    SquareMatrix<double, 3> inv_H_T_H;
    check_inverse = false;
    check_inverse = inv(SquareMatrix<double, 3> (H.transpose() * H), inv_H_T_H);

    if (check_inverse == false)
    {
		dwm_errors++;
		PX4_INFO("Error: No inverse matrix in least squares algorithm");
        return x_ccf_i;
    }

    Vector3d Delta_x_ccf = (inv_H_T_H * H.transpose()) * Delta_rho_ccf;

    Vector3d x_ccf_i_plus_1(x_ccf_i + Delta_x_ccf);

    return x_ccf_i_plus_1;
}

Vector3d DWM1004C::rolling_moving_average(Vector3d x)
{
	// Nach ChatGPT "Rolling Moving Average in C"

    static uint8_t i = 0;

    // Initialisiere die Summe für das erste Fenster
    if (i < RMA_WINDOW_SIZE)
	{
		window_rma.col(i) = x;
        sum_rma += x;

		return sum_rma / (double)(++i);
    }

    // Berechne den gleitenden Mittelwert für die restlichen Daten
	sum_rma += x - window_rma.col(0);

	for (uint8_t j = 0; j < RMA_WINDOW_SIZE-1; j++)
	{
		window_rma.col(j) = Vector3d(window_rma.col(j+1));
	}
	window_rma.col(RMA_WINDOW_SIZE-1) = x;

	return sum_rma / (double)RMA_WINDOW_SIZE;
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

void DWM1004C::custom_method(const BusCLIArguments &cli)
{
	uint8_t custom_method_cmd = DWM1004C_TRANSMIT;

	switch(cli.custom1)
	{
		case 10:
			custom_method_data.received_data_print = cli.custom2;
			custom_method_data.received_data_counter = 0;
			break;
		case 11:
			custom_method_data.deviations_print = cli.custom2;
			custom_method_data.deviations_counter = 0;
			break;
		case 12:
			custom_method_data.sum_deviations_print = cli.custom2;
			custom_method_data.sum_deviations_counter = 0;
			break;
		case 13:
			custom_method_data.local_coordinates_print = cli.custom2;
			custom_method_data.local_coordinates_counter = 0;
			break;
		case 14:
			custom_method_data.local_velocities_print = cli.custom2;
			custom_method_data.local_velocities_counter = 0;
			break;
		case 15:
			custom_method_data.found_errors_print = cli.custom2;
			custom_method_data.found_errors_counter = 0;
			break;
		case 20:
			custom_method_cmd = DWM1004C_OFFSET_UP;
			transfer(&custom_method_cmd, 1, nullptr, 0);
			counter_measurements_fix_point.setZero();
			sum_actual_distances_fix_point.setZero();
			PX4_INFO("The LOCODECK_ANTENNA_OFFSET will be increased by 0.01.");
			break;
		case 21:
			custom_method_cmd = DWM1004C_OFFSET_DOWN;
			transfer(&custom_method_cmd, 1, nullptr, 0);
			counter_measurements_fix_point.setZero();
			sum_actual_distances_fix_point.setZero();
			PX4_INFO("The LOCODECK_ANTENNA_OFFSET will be decreased by 0.01.");
			break;
		case 30:
			custom_method_data.ema_alpha = *((double *)cli.custom_data);
			PX4_INFO("ema_alpha = %f", custom_method_data.ema_alpha);
			break;
		case 40:
			custom_method_data.vel_ned_valid = true;
			PX4_INFO("vel_ned_valid = %s", custom_method_data.vel_ned_valid ? "true" : "false");
			break;
		case 41:
			custom_method_data.vel_ned_valid = false;
			PX4_INFO("vel_ned_valid = %s", custom_method_data.vel_ned_valid ? "true" : "false");
			break;
		case 50:
			custom_method_cmd = DWM1004C_RESET;
			transfer(&custom_method_cmd, 1, nullptr, 0);
			PX4_INFO("The DWM1004C will be reseted.");
			break;
	}
}

void DWM1004C::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO_RAW("\nDriver Counters:\n");
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_fault_perf);

	PX4_INFO_RAW("\nModule Counters:\n");
	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_loop_perf);

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

		#ifdef DEBUG_MODE_ON
		// PX4_INFO_RAW("ret1 = %d, ret2 = %d\n", ret1, ret2);
		// PX4_INFO_RAW("data_1[0] = %X, data_2[0] = %X\n", data_1[0], data_2[0]);
		// PX4_INFO_RAW("data_1[1] = %X, data_2[1] = %X\n", data_1[1], data_2[1]);
		// PX4_INFO_RAW("data_1[2] = %X, data_2[2] = %X\n", data_1[2], data_2[2]);
		#endif

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
