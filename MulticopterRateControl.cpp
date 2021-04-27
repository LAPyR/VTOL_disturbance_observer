/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"

#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <mathlib/mathlib.h>



#include <lib/drivers/device/Device.hpp>
//debugging
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rc_channels.h>

//---->

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterRateControl::MulticopterRateControl(bool vtol) :
        ModuleParams(nullptr),
        WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
        _actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
        _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
        _vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

        parameters_updated();
}
/*
//Vector<float, 1> MulticopterRateControl::dynamics(
float MulticopterRateControl::dynamics(
        float t,
        //const Vector<float, 1> &x,
        //const Vector<float, 1> &u)
        float x,
        float &u)
{
        return (1.0f * u);
}
*/
MulticopterRateControl::~MulticopterRateControl()
{
        perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
        if (!_vehicle_angular_velocity_sub.registerCallback()) {
                PX4_ERR("vehicle_angular_velocity callback registration failed!");
                return false;
        }

        return true;
}

void
MulticopterRateControl::parameters_updated()
{
        // rate control parameters
        // The controller gain K is used to convert the parallel (P + I/s + sD) form
        // to the ideal (K * [1 + 1/sTi + sTd]) form


 // Modificar las ganancias con las perillas

    uORB::SubscriptionData<manual_control_setpoint_s> manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
        manual_control_setpoint_sub.update();
        const manual_control_setpoint_s &manual_control_setpoint = manual_control_setpoint_sub.get();

    if(manual_control_setpoint.aux1 < 0.0f){
        if (manual_control_setpoint.aux3 <= -0.5f ){
            // L1 originalmente 100
            if(fabsf(manual_control_setpoint.aux6) > 0.15f){
                _pitchrate_k = _pitchrate_k + manual_control_setpoint.aux6 * 0.0003f;
            }
           if (_pitchrate_k < 0.01f){
               _pitchrate_k = 0.01f;
           }
           if (_pitchrate_k > 5.0f){
               _pitchrate_k = 5.0f;
           }
           _mostrar1 = _pitchrate_k;
        }
        else if (manual_control_setpoint.aux3 > -0.5f && manual_control_setpoint.aux3 <= 0.5f) {
            // L1 originalmente 100
            if(fabsf(manual_control_setpoint.aux6) > 0.15f){
                _pitchrate_p = _pitchrate_p + manual_control_setpoint.aux6 * 0.0003f;
            }
           if (_pitchrate_p < 0.01f){
               _pitchrate_p = 0.01f;
           }
           if (_pitchrate_p > 0.6f){
               _pitchrate_p = 0.6f;
           }
           _mostrar1 = _pitchrate_p;
        }
        else{
            // L1 originalmente 100
            if(fabsf(manual_control_setpoint.aux6) > 0.15f){
                _pitchrate_d = _pitchrate_d + manual_control_setpoint.aux6 * 0.00001f;
            }
           if (_pitchrate_d < 0.0f){
               _pitchrate_d = 0.0f;
           }
           if (_pitchrate_d > 0.05f){
               _pitchrate_d = 0.05f;
           }
           _mostrar1 = _pitchrate_d;
        }
    }
    else{

        _pitchrate_k = _param_mc_pitchrate_k.get();
        _pitchrate_p = _param_mc_pitchrate_p.get();
        _pitchrate_d = _param_mc_pitchrate_d.get();

    }

//    _pitchrate_k = _param_mc_pitchrate_k.get();
//    _pitchrate_p = _param_mc_pitchrate_p.get();
//    _pitchrate_d = _param_mc_pitchrate_d.get();

        const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _pitchrate_k, _param_mc_yawrate_k.get());

        _rate_control.setGains(
                rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _pitchrate_p, _param_mc_yawrate_p.get())),
                rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
                rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _pitchrate_d, _param_mc_yawrate_d.get())));

        _rate_control.setIntegratorLimit(
                Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

        _rate_control.setFeedForwardGain(
                Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


        // manual rate control acro mode rate limits
        _acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
                                  radians(_param_mc_acro_y_max.get()));

        _actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
}

float
MulticopterRateControl::get_landing_gear_state()
{
        // Only switch the landing gear up if we are not landed and if
        // the user switched from gear down to gear up.
        // If the user had the switch in the gear up position and took off ignore it
        // until he toggles the switch to avoid retracting the gear immediately on takeoff.
        if (_landed) {
                _gear_state_initialized = false;
        }

        float landing_gear = landing_gear_s::GEAR_DOWN; // default to down

        if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized) {
                landing_gear = landing_gear_s::GEAR_UP;

        } else if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
                // Switching the gear off does put it into a safe defined state
                _gear_state_initialized = true;
        }

        return landing_gear;
}

void
MulticopterRateControl::Run()
{

        uint64_t newTimeStamp = hrt_absolute_time();
        //float h = (newTimeStamp - _timeStamp) / 1.0e7f;
        _timeStamp = newTimeStamp;

    /* Yo los defini */
        /* advertise named debug value */
        struct debug_key_value_s dbg_key;
        strncpy(dbg_key.key, "rate_1", 10);

    /* Yo los defini */

        if (should_exit()) {
                _vehicle_angular_velocity_sub.unregisterCallback();
                exit_and_cleanup();
                return;
        }

        perf_begin(_loop_perf);

        // Check if parameters have changed
        if (_parameter_update_sub.updated()) {
                // clear update
                parameter_update_s param_update;
                _parameter_update_sub.copy(&param_update);

                updateParams();
                parameters_updated();
        }
        parameters_updated();

        /* run controller on gyro changes */
        vehicle_angular_velocity_s angular_velocity;

        if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

                // grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
                vehicle_angular_acceleration_s v_angular_acceleration{};
                _vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

                const hrt_abstime now = hrt_absolute_time();

                // Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
                const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
                _last_run = now;

                const Vector3f angular_accel{v_angular_acceleration.xyz};
                const Vector3f rates{angular_velocity.xyz};


                /* check for updates in other topics */
                _v_control_mode_sub.update(&_v_control_mode);

                if (_vehicle_land_detected_sub.updated()) {
                        vehicle_land_detected_s vehicle_land_detected;

                        if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
                                _landed = vehicle_land_detected.landed;
                                _maybe_landed = vehicle_land_detected.maybe_landed;
                        }
                }

                _vehicle_status_sub.update(&_vehicle_status);

                const bool manual_control_updated = _manual_control_sp_sub.update(&_manual_control_sp);

                // generate the rate setpoint from sticks?
                bool manual_rate_sp = false;

                if (_v_control_mode.flag_control_manual_enabled &&
                    !_v_control_mode.flag_control_altitude_enabled &&
                    !_v_control_mode.flag_control_velocity_enabled &&
                    !_v_control_mode.flag_control_position_enabled) {

                        // landing gear controlled from stick inputs if we are in Manual/Stabilized mode
                        //  limit landing gear update rate to 50 Hz
                        if (hrt_elapsed_time(&_landing_gear.timestamp) > 20_ms) {
                                _landing_gear.landing_gear = get_landing_gear_state();
                                _landing_gear.timestamp = hrt_absolute_time();
                                _landing_gear_pub.publish(_landing_gear);
                        }

                        if (!_v_control_mode.flag_control_attitude_enabled) {
                                manual_rate_sp = true;
                        }

                        // Check if we are in rattitude mode and the pilot is within the center threshold on pitch and roll
                        //  if true then use published rate setpoint, otherwise generate from manual_control_setpoint (like acro)
                        if (_v_control_mode.flag_control_rattitude_enabled) {
                                manual_rate_sp =
                                        (fabsf(_manual_control_sp.y) > _param_mc_ratt_th.get()) ||
                                        (fabsf(_manual_control_sp.x) > _param_mc_ratt_th.get());
                        }

                } else {
                        _landing_gear_sub.update(&_landing_gear);
                }

                if (manual_rate_sp) {
                        if (manual_control_updated) {

                                // manual rates control - ACRO mode
                                const Vector3f man_rate_sp{
                                        math::superexpo(_manual_control_sp.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
                                        math::superexpo(-_manual_control_sp.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
                                        math::superexpo(_manual_control_sp.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

                                _rates_sp = man_rate_sp.emult(_acro_rate_max);
                                _thrust_sp = _manual_control_sp.z;

                                // publish rate setpoint
                                vehicle_rates_setpoint_s v_rates_sp{};
                                v_rates_sp.roll = _rates_sp(0);
                                v_rates_sp.pitch = _rates_sp(1);
                                v_rates_sp.yaw = _rates_sp(2);
                                v_rates_sp.thrust_body[0] = 0.0f;
                                v_rates_sp.thrust_body[1] = 0.0f;
                                v_rates_sp.thrust_body[2] = -_thrust_sp;
                                v_rates_sp.timestamp = hrt_absolute_time();

                                _v_rates_sp_pub.publish(v_rates_sp);
                        }

                } else {
                        // use rates setpoint topic
                        vehicle_rates_setpoint_s v_rates_sp;

                        if (_v_rates_sp_sub.update(&v_rates_sp)) {
                                _rates_sp(0) = v_rates_sp.roll;
                                _rates_sp(1) = v_rates_sp.pitch;
                                _rates_sp(2) = v_rates_sp.yaw;
                                _thrust_sp = -v_rates_sp.thrust_body[2];
                        }
                }

                // run the rate controller
                if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled) {

                        // reset integral if disarmed
                        if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
                                _rate_control.resetIntegral();
                        }

                        // update saturation status from mixer feedback
                        if (_motor_limits_sub.updated()) {
                                multirotor_motor_limits_s motor_limits;

                                if (_motor_limits_sub.copy(&motor_limits)) {
                                        MultirotorMixer::saturation_status saturation_status;
                                        saturation_status.value = motor_limits.saturation_status;

                                        _rate_control.setSaturationStatus(saturation_status);
                                }
                        }



                        uORB::SubscriptionData<manual_control_setpoint_s> manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
                            manual_control_setpoint_sub.update();
                            const manual_control_setpoint_s &manual_control_setpoint = manual_control_setpoint_sub.get();

                        //uORB::SubscriptionData<rc_channels_s> rc_ch{ORB_ID(rc_channels)};
                        //rc_ch.update();
                        //const rc_channels_s &chann = rc_ch.get();

                            struct debug_vect_s dbg_vect;
                            strncpy(dbg_vect.name, "vec", 10);

                        //Condición para poder aplicar o _delta_hat_psi al control
                        float frs = _delta_hat_psi;
                        if (manual_control_setpoint.aux2 > 0.0f){
                             frs = _delta_hat_psi;
                            _delta_hat_psi = 0.0f;
                        }



                        // run rate controller
                        // se añade otra entrada a la función "_delta_hat_psi" del tipo flotante (se modifica el archivo RateControl.cpp y .hpp)
                        const Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, _delta_hat_psi, dt, _maybe_landed || _landed);
                        //const Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, 0.0f, dt, _maybe_landed || _landed);




                        _delta_hat_psi = frs;

                        // Observadores
                        // Observafor Luemberger
                        /* Se añaden las siguientes lineas al archivo MulticopterRateControl.hpp
                         *
                                  uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

                                  struct vehicle_attitude_s            _v_att {};
                                  float _contador = 0.0f;
                                  matrix::Vector3f _att_ob{0.0f,0.0f,0.0f};
                                  matrix::Vector3f _vel_ob{0.0f,0.0f,0.0f};
                                  float _delta_hat_psi = 0.0f;
                       */



                        _v_att.q[0] = 1.f;  // inicializamos para no tener errores
                        _vehicle_attitude_sub.update(&_v_att);


                        //Filtro básico para ruido en el angulo pitch
                        Vector3f ang_euler = Eulerf(Quatf(_v_att.q)); // Orientación actual en angulos de euler
                        _med_pitch1(2) = _med_pitch1(1);
                        _med_pitch1(1) = _med_pitch1(0);
                        _med_pitch1(0) = _med_pitch(2);
                        _med_pitch(2) = _med_pitch(1);
                        _med_pitch(1) = _med_pitch(0);
                        _med_pitch(0) = ang_euler(1);
                        ang_euler(1) = (_med_pitch(0) + _med_pitch(1) + _med_pitch(2) + _med_pitch1(0) + _med_pitch1(1) + _med_pitch1(2)) / 6.0f;

                        Vector3f err_ob = _att_ob - ang_euler; // attitude error                    

                        // condicion añadida para evitar problemas de calculo de error cuando el UAV pasa de -pi a pi en yaw
                        if (fabsf(err_ob(2)) > 3.1416f){
                            const int si = math::signNoZero(err_ob(2));
                            err_ob(2) = -((2.0f * 3.1416f) - fabsf(err_ob(2))) * (si * 1.f);
                        }


                        // Ganancias
                            if (manual_control_setpoint.aux4 < -0.95f ){
                                // L1 originalmente 100
                                if(fabsf(manual_control_setpoint.aux5) > 0.15f){
                                    _L1 = _L1 + manual_control_setpoint.aux5 * 0.1f;
                                }
                               if (_L1 < 0.0f){
                                   _L1 = 0.0f;
                               }
                               strncpy(dbg_vect.name, "L1", 10);
                               _mostrar = _L1;
                            }
                            else if (manual_control_setpoint.aux4 > -0.95f && manual_control_setpoint.aux4 < -0.65f) {
                                // L2 originalmente 200
                                if(fabsf(manual_control_setpoint.aux5) > 0.15f){
                                    _L2 = _L2 + manual_control_setpoint.aux5 * 0.1f;
                                }
                                if (_L2 < 0.0f){
                                    _L2 = 0.0f;
                                }
                                strncpy(dbg_vect.name, "L2", 10);
                                _mostrar = _L2;
                            }
                            else if (manual_control_setpoint.aux4 > -0.65f && manual_control_setpoint.aux4 < -0.35f) {
                                // k1 originalmente 100
                                if(fabsf(manual_control_setpoint.aux5) > 0.15f){
                                    _k1 = _k1 + manual_control_setpoint.aux5 * 0.1f;
                                }
                                if (_k1 < 0.0f){
                                    _k1 = 0.0f;
                                }
                                strncpy(dbg_vect.name, "k1", 10);
                                _mostrar = _k1;
                            }
                            else if (manual_control_setpoint.aux4 > -0.35f && manual_control_setpoint.aux4 < -0.25f) {
                                // k2 originalmente 0.5
                                if(fabsf(manual_control_setpoint.aux5) > 0.15f){
                                    _k2 = _k2 + manual_control_setpoint.aux5 * 0.001f;
                                }
                                if (_k2 < 0.001f){
                                    _k2 = 0.001f;
                                }
                                strncpy(dbg_vect.name, "k2", 10);
                                _mostrar = _k2;
                            }
                            else if (manual_control_setpoint.aux4 > -0.25f && manual_control_setpoint.aux4 < 0.2f) {
                                // k2 originalmente 0.5
                                if(fabsf(manual_control_setpoint.aux5) > 0.15f){
                                    _eps = _eps + manual_control_setpoint.aux5 * 0.000001f;
                                }
                                if (_eps < 0.00001f){
                                    _eps = 0.00001f;
                                }
                                strncpy(dbg_vect.name, "eps", 10);
                                _mostrar = _eps;
                            }
                            else if (manual_control_setpoint.aux4 > 0.2f && manual_control_setpoint.aux4 < 0.3f) {
                                // k3 originalmente 0.2
                                if(fabsf(manual_control_setpoint.aux5) > 0.15f){
                                    _k3 = _k3 + manual_control_setpoint.aux5 * 0.0001f;
                                }
                                if (_k3 < 0.0f){
                                    _k3 = 0.0f;
                                }
                                strncpy(dbg_vect.name, "k3", 10);
                                _mostrar = _k3;
                            }
                            else if (manual_control_setpoint.aux4 > 0.3f && manual_control_setpoint.aux4 < 0.6f) {
                                // l1 originalmente 0.355
                                if(fabsf(manual_control_setpoint.aux5) > 0.15f){
                                    _l1 = _l1 + manual_control_setpoint.aux5 * 0.0001f;
                                }
                                if (_l1 < 0.001f){
                                    _l1 = 0.001f;
                                }
                                strncpy(dbg_vect.name, "l1", 10);
                                _mostrar = _l1;
                            }
                            else{
                            }


                        //inicializado de la variable del observador de perturbación
                        const Vector3f delta_hat{0.0f, _delta_hat_psi,0.0f};

                        // integrador acceleracion angular -> velocidad angular
                        //_vel_ob += ((-k1 * _vel_ob) + ((att_control + delta_hat) / k2) - (L2 * err_ob)) * dt;   //sin delta_hat
                        _vel_ob += ((-_k1 * _vel_ob) + ((att_control) / _k2) - (_L2 * err_ob)) * dt;                 //con delta_hat

                         // integrador velocidad angular -> posicion angular
                        _att_ob += (_vel_ob - _L1*err_ob) * dt;

                        // wrap del integrador en la orientacion de yaw -pi, pi
                            if (_att_ob(2) > 3.1416f){
                                _att_ob(2) = _att_ob(2) - (2.f * 3.1416f);
                            }
                            if (_att_ob(2) < -3.1416f){
                                _att_ob(2) = (2.f * 3.1416f) + _att_ob(2);
                            }

                            _delta_hat_psi += (-((_l1 * err_ob(1)) / (fabsf(err_ob(1)) + _eps)) - (_k3 * _delta_hat_psi)) * dt;



                            dbg_vect.x = _mostrar;
                            dbg_vect.y = _mostrar1;
                            dbg_vect.z = _delta_hat_psi;
                            orb_advert_t pub_dbg_vect = orb_advertise(ORB_ID(debug_vect), &dbg_vect);
                            uint64_t timestamp_us = hrt_absolute_time();
                            dbg_vect.timestamp = timestamp_us;
                            orb_publish(ORB_ID(debug_vect), pub_dbg_vect, &dbg_vect);



                        // publish rate controller status
                        rate_ctrl_status_s rate_ctrl_status{};
                        _rate_control.getRateControlStatus(rate_ctrl_status);
                        rate_ctrl_status.timestamp = hrt_absolute_time();
                        _controller_status_pub.publish(rate_ctrl_status);

                        // publish actuator controls
                        actuator_controls_s actuators{};
                        actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
                        actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
                        actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
                        actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
                        actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = (float)_landing_gear.landing_gear;
                        actuators.timestamp_sample = angular_velocity.timestamp_sample;

                        // scale effort by battery status if enabled
                        if (_param_mc_bat_scale_en.get()) {
                                if (_battery_status_sub.updated()) {
                                        battery_status_s battery_status;

                                        if (_battery_status_sub.copy(&battery_status)) {
                                                _battery_status_scale = battery_status.scale;
                                        }
                                }

                                if (_battery_status_scale > 0.0f) {
                                        for (int i = 0; i < 4; i++) {
                                                actuators.control[i] *= _battery_status_scale;
                                        }
                                }
                        }

                        actuators.timestamp = hrt_absolute_time();
                        _actuators_0_pub.publish(actuators);

                } else if (_v_control_mode.flag_control_termination_enabled) {
                        if (!_vehicle_status.is_vtol) {
                                // publish actuator controls
                                actuator_controls_s actuators{};
                                actuators.timestamp = hrt_absolute_time();
                                _actuators_0_pub.publish(actuators);
                        }
                }
        }

        perf_end(_loop_perf);
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
        bool vtol = false;

        if (argc > 1) {
                if (strcmp(argv[1], "vtol") == 0) {
                        vtol = true;
                }
        }

        MulticopterRateControl *instance = new MulticopterRateControl(vtol);

        if (instance) {
                _object.store(instance);
                _task_id = task_id_is_work_queue;

                if (instance->init()) {
                        return PX4_OK;
                }

        } else {
                PX4_ERR("alloc failed");
        }

        delete instance;
        _object.store(nullptr);
        _task_id = -1;

        return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
        return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
        if (reason) {
                PX4_WARN("%s\n", reason);
        }

        PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

        return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
        return MulticopterRateControl::main(argc, argv);
}
