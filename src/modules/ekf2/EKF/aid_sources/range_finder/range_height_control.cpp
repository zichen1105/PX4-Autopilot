/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

/**
 * @file range_height_control.cpp
 * Control functions for ekf range finder height fusion
 */

#include "ekf.h"
#include "ekf_derivation/generated/compute_hagl_innov_var.h"

void Ekf::controlRangeHeightFusion()
{
	static constexpr const char *HGT_SRC_NAME = "RNG";

	bool rng_data_ready = false;

	if (_range_buffer) {
		// Get range data from buffer and check validity
		rng_data_ready = _range_buffer->pop_first_older_than(_time_delayed_us, _range_sensor.getSampleAddress());
		_range_sensor.setDataReadiness(rng_data_ready);

		// update range sensor angle parameters in case they have changed
		_range_sensor.setPitchOffset(_params.rng_sens_pitch);
		_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
		_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

		_range_sensor.runChecks(_time_delayed_us, _R_to_earth);

		if (_range_sensor.isDataHealthy()) {
			// correct the range data for position offset relative to the IMU
			const Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_range_sensor.setRange(_range_sensor.getRange() + pos_offset_earth(2) / _range_sensor.getCosTilt());

			if (_control_status.flags.in_air) {
				const bool horizontal_motion = _control_status.flags.fixed_wing
							       || (sq(_state.vel(0)) + sq(_state.vel(1)) > fmaxf(P.trace<2>(State::vel.idx), 0.1f));

				const float dist_dependant_var = sq(_params.range_noise_scaler * _range_sensor.getDistBottom());
				const float var = sq(_params.range_noise) + dist_dependant_var;

				_rng_consistency_check.setGate(_params.range_kin_consistency_gate);
				_rng_consistency_check.update(_range_sensor.getDistBottom(), math::max(var, 0.001f), _state.vel(2),
							      P(State::vel.idx + 2, State::vel.idx + 2), horizontal_motion, _time_delayed_us);
			}

		} else {
			// If we are supposed to be using range finder data as the primary height sensor, have bad range measurements
			// and are on the ground, then synthesise a measurement at the expected on ground value
			if (!_control_status.flags.in_air
			    && _range_sensor.isRegularlySendingData()
			    && _range_sensor.isDataReady()) {

				_range_sensor.setRange(_params.rng_gnd_clearance);
				_range_sensor.setValidity(true); // bypass the checks
			}
		}

		_control_status.flags.rng_kin_consistent = _rng_consistency_check.isKinematicallyConsistent();

	} else {
		return;
	}

	auto &aid_src = _aid_src_rng_hgt;

	if (rng_data_ready && _range_sensor.getSampleAddress()) {

		updateRangeHeight(aid_src);
		const bool measurement_valid = PX4_ISFINITE(aid_src.observation) && PX4_ISFINITE(aid_src.observation_variance);

		const bool continuing_conditions_passing = ((_params.rng_ctrl == static_cast<int32_t>(RngCtrl::ENABLED))
							    || (_params.rng_ctrl == static_cast<int32_t>(RngCtrl::CONDITIONAL)))
							   && measurement_valid
							   && _range_sensor.isDataHealthy()
						           && _rng_consistency_check.isKinematicallyConsistent();

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_range_buffer_push, 2 * estimator::sensor::RNG_MAX_INTERVAL)
				&& _range_sensor.isRegularlySendingData();


		const bool do_conditional_range_aid = (_hagl_sensor_status.flags.range_finder || _control_status.flags.rng_hgt)
						      && (_params.rng_ctrl == static_cast<int32_t>(RngCtrl::CONDITIONAL))
						      && isConditionalRangeAidSuitable();

		const bool do_range_aid = (_hagl_sensor_status.flags.range_finder || _control_status.flags.rng_hgt)
					  && (_params.rng_ctrl == static_cast<int32_t>(RngCtrl::ENABLED));

		if (_control_status.flags.rng_hgt) {
			if (!(do_conditional_range_aid || do_range_aid)) {
				ECL_INFO("stopping %s fusion", HGT_SRC_NAME);
				stopRngHgtFusion();
			}

		} else {
			if (_params.height_sensor_ref == static_cast<int32_t>(HeightSensor::RANGE)) {
				if (do_conditional_range_aid) {
					// Range finder is used while hovering to stabilize the height estimate. Don't reset but use it as height reference.
					ECL_INFO("starting conditional %s height fusion", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::RANGE;

					_control_status.flags.rng_hgt = true;
					stopRngTerrFusion();

					if (!_hagl_sensor_status.flags.flow && aid_src.innovation_rejected) {
						resetHaglRng(aid_src);
					}

				} else if (do_range_aid) {
					// Range finder is the primary height source, the ground is now the datum used
					// to compute the local vertical position
					ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::RANGE;

					_information_events.flags.reset_hgt_to_rng = true;
					resetVerticalPositionTo(-aid_src.observation, aid_src.observation_variance);
					_state.terrain = 0.f;
					_control_status.flags.rng_hgt = true;
					stopRngTerrFusion();

					aid_src.time_last_fuse = _time_delayed_us;
				}

			} else {
				if (do_conditional_range_aid || do_range_aid) {
					ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
					_control_status.flags.rng_hgt = true;

					if (!_hagl_sensor_status.flags.flow && aid_src.innovation_rejected) {
						resetHaglRng(aid_src);
					}
				}
			}
		}

		if (_control_status.flags.rng_hgt || _hagl_sensor_status.flags.range_finder) {
			if (continuing_conditions_passing) {

				fuseHaglRng(aid_src, _control_status.flags.rng_hgt, _hagl_sensor_status.flags.range_finder);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired() && _control_status.flags.rng_hgt) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_rng = true;
					resetVerticalPositionTo(-(aid_src.observation - _state.terrain));

					// reset vertical velocity
					resetVerticalVelocityToZero();

					aid_src.time_last_fuse = _time_delayed_us;

				} else if (is_fusion_failing) {
					// Some other height source is still working
					if (_hagl_sensor_status.flags.flow) {
						ECL_WARN("stopping %s fusion, fusion failing", HGT_SRC_NAME);
						stopRngHgtFusion();
						stopRngTerrFusion();

					} else {
						resetHaglRng(aid_src);
					}
				}

			} else {
				ECL_WARN("stopping %s fusion, continuing conditions failing", HGT_SRC_NAME);
				stopRngHgtFusion();
				stopRngTerrFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_hagl_sensor_status.flags.flow) {
					if (!aid_src.innovation_rejected) {
						_hagl_sensor_status.flags.range_finder = true;
						fuseHaglRng(aid_src, _control_status.flags.rng_hgt, _hagl_sensor_status.flags.range_finder);
					}

				} else {
					if (aid_src.innovation_rejected) {
						resetHaglRng(aid_src);
					}

					_hagl_sensor_status.flags.range_finder = true;
				}
			}
		}

	} else if ((_control_status.flags.rng_hgt || _hagl_sensor_status.flags.range_finder)
		   && !isNewestSampleRecent(_time_last_range_buffer_push, 2 * estimator::sensor::RNG_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s fusion, no data", HGT_SRC_NAME);
		stopRngHgtFusion();
		stopRngTerrFusion();
	}
}

void Ekf::updateRangeHeight(estimator_aid_source1d_s &aid_src)
{
	resetEstimatorAidStatus(aid_src);

	aid_src.observation = math::max(_range_sensor.getDistBottom(), _params.rng_gnd_clearance);
	aid_src.innovation = getHagl() - aid_src.observation;

	aid_src.observation_variance = getRngVar();

	sym::ComputeHaglInnovVar(P, aid_src.observation_variance, &aid_src.innovation_variance);

	const float innov_gate = math::max(_params.range_innov_gate, 1.f);
	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	// z special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innov_gate * sqrtf(aid_src.innovation_variance);
		aid_src.innovation = math::constrain(aid_src.innovation, -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}

	aid_src.timestamp_sample = _range_sensor.getSampleAddress()->time_us;
}

float Ekf::getRngVar() const
{
	return fmaxf(
		     P(State::pos.idx + 2, State::pos.idx + 2)
		     + sq(_params.range_noise)
		     + sq(_params.range_noise_scaler * _range_sensor.getRange()),
	       0.f);
}

void Ekf::resetHaglRng(estimator_aid_source1d_s &aid_src)
{
	_state.terrain = _state.pos(2) + aid_src.observation;
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, aid_src.observation_variance);
	_terrain_vpos_reset_counter++;

	aid_src.time_last_fuse = _time_delayed_us;
}

bool Ekf::isConditionalRangeAidSuitable()
{
#if defined(CONFIG_EKF2_TERRAIN)

	if (_control_status.flags.in_air) {
		// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
		// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
		float range_hagl_max = _params.max_hagl_for_range_aid;
		float max_vel_xy = _params.max_vel_for_range_aid;

		const float hagl_test_ratio = _aid_src_rng_hgt.test_ratio;

		bool is_hagl_stable = (hagl_test_ratio < 1.f);

		if (!_control_status.flags.rng_hgt) {
			range_hagl_max = 0.7f * _params.max_hagl_for_range_aid;
			max_vel_xy = 0.7f * _params.max_vel_for_range_aid;
			is_hagl_stable = (hagl_test_ratio < 0.01f);
		}

		const bool is_in_range = (getHagl() < range_hagl_max);

		bool is_below_max_speed = true;

		if (isHorizontalAidingActive()) {
			is_below_max_speed = !_state.vel.xy().longerThan(max_vel_xy);
		}

		return is_in_range && is_hagl_stable && is_below_max_speed;
	}

#endif // CONFIG_EKF2_TERRAIN

	return false;
}

void Ekf::stopRngHgtFusion()
{
	if (_control_status.flags.rng_hgt) {

		if (_height_sensor_ref == HeightSensor::RANGE) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_control_status.flags.rng_hgt = false;
	}
}

void Ekf::stopRngTerrFusion()
{
	_hagl_sensor_status.flags.range_finder = false;
}
