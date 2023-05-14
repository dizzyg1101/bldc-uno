/*
	Copyright 2019 Mitch Lustig

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */



   // Notes need to add back temp limit tilt back, verify tiltbacks work still, and maybe change output from UTILS expo to have same gains as non- expo use. 
   // Need to clean up code/trim to run fast. Ideally, we would filter IMU data how we want and then down-sample in this code
   // Right now it seems we are downsampling, and then filtering, which can have aliasing issues.
   // Biquad filter as implemented seems to be "notchy" feeling in the derriviative term
   


#include "conf_general.h"

#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "commands.h"
#include "imu/imu.h"
#include "imu/ahrs.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "datatypes.h"
#include "comm_can.h"
#include "terminal.h"
#include "digital_filter.h"

//Alex Changes. 
#include "app.h"



#include <math.h>
#include <stdio.h>

// Can
#define MAX_CAN_AGE 0.1

// Data type (Value 5 was removed, and can be reused at a later date, but i wanted to preserve the current value's numbers for UIs)
typedef enum {
	STARTUP = 0,
	RUNNING = 1,
	RUNNING_TILTBACK_DUTY = 2,
	RUNNING_TILTBACK_HIGH_VOLTAGE = 3,
	RUNNING_TILTBACK_LOW_VOLTAGE = 4,
	//Alex Changes. Mitch left space in value 5. Using for high temp tiltback.
	RUNNING_TILTBACK_HIGHTEMP = 5,
	FAULT_ANGLE_PITCH = 6,
	FAULT_ANGLE_ROLL = 7,
	FAULT_SWITCH_HALF = 8,
	FAULT_SWITCH_FULL = 9,
	FAULT_DUTY = 10,
	FAULT_STARTUP = 11
} BalanceState;

typedef enum {
	CENTERING = 0,
	TILTBACK_DUTY,
	TILTBACK_HV,
	TILTBACK_LV,
	// Alex Changes. Addition for temp adjust
	TILTBACK_HIGHTEMP,
	TILTBACK_NONE
} SetpointAdjustmentType;

typedef enum {
	OFF = 0,
	HALF,
	ON
} SwitchState;

// Balance thread
static THD_FUNCTION(balance_thread, arg);
static THD_WORKING_AREA(balance_thread_wa, 2048); // 2kb stack for this thread

static thread_t *app_thread;

// Config values
static volatile balance_config balance_conf;
static volatile imu_config imu_conf;
static systime_t loop_time;
static float startup_step_size;
//Alex changes. Added for hightemp tiltback stepsize.
static float tiltback_duty_step_size, tiltback_hv_step_size, tiltback_lv_step_size, tiltback_hightemp_step_size, tiltback_return_step_size;
static float torquetilt_on_step_size, torquetilt_off_step_size, turntilt_step_size;
static float tiltback_variable, tiltback_variable_max_erpm, noseangling_step_size;

//Alex Changes.
//bring in motor configuration settings for maximum current, fet and motor temp limits
static float fet_temp_limit;
static float motor_temp_limit;



// Runtime values read from elsewhere
static float pitch_angle, last_pitch_angle, roll_angle, abs_roll_angle, abs_roll_angle_sin, last_gyro_y;
static float gyro[3];
static float duty_cycle, abs_duty_cycle;
static float erpm, abs_erpm, avg_erpm;
static float motor_current;
static float motor_position;
static float adc1, adc2;
static SwitchState switch_state;

// Rumtime state values
static BalanceState state;
static float proportional, integral, derivative, proportional2, integral2, derivative2;
static float last_proportional, abs_proportional;
static float pid_value;
static float setpoint, setpoint_target, setpoint_target_interpolated;
static float noseangling_interpolated;
static float torquetilt_filtered_current, torquetilt_target, torquetilt_interpolated;
static Biquad torquetilt_current_biquad;
static float turntilt_target, turntilt_interpolated;
static SetpointAdjustmentType setpointAdjustmentType;
static float yaw_proportional, yaw_integral, yaw_derivative, yaw_last_proportional, yaw_pid_value, yaw_setpoint;
static systime_t current_time, last_time, diff_time, loop_overshoot;
static float filtered_loop_overshoot, loop_overshoot_alpha, filtered_diff_time;
static systime_t fault_angle_pitch_timer, fault_angle_roll_timer, fault_switch_timer, fault_switch_half_timer, fault_duty_timer;
static float d_pt1_lowpass_state, d_pt1_lowpass_k, d_pt1_highpass_state, d_pt1_highpass_k;
static float motor_timeout;
static systime_t brake_timeout;


// Alex Changes. Added variables. Some of these are not being used at the moment. 
static float max_motor_current;
static float pitch_filtered;
static float last_pitch_filtered;
static float last_torquetilt_filtered_current;
static float ReadSensorTimeStamp;

static float erpm_filtered; 
static float last_erpm_filtered;
static float last_erpm;
static float last_duty_cycle;

static float rpm_filtered ;
static float last_rpm_filtered ;
static float Kk1;
static float Xk;
static float Kk;
static float last_Xk;
static float last_proportional_filtered;
static float proportional_filtered;
static float adc_lowpass_k;
static float adc1_lowpass_state;
static float adc2_lowpass_state;
static float adc1_filtered ;
static float adc2_filtered ;

static float roll_angle_filtered;
static float roll_angle_lowpass_state;
static float abs_roll_angle_filtered;
static float roll_angle_lowpass_k;

// Alex Changes. Work in progress for non-linear output and variable gain w/speed.
// re-puposing throttle expo settings from VESC Tool
// static float imu_angle_exp;
// static float imu_angle_exp_brake;
// static float imu_angle_exp_mode;
//re-purposing speed PID variables from VESC Tool
//static float psm_x1;
//static float psm_y1;
//static float psm_y2;
//static float psm_y3;
//static float psm_x2;
//bool speed_modifier_enable;
//static float proportional_speed_multiplier;
//static float previous_ERPM;


// Debug values
static int debug_render_1, debug_render_2;
static int debug_sample_field, debug_sample_count, debug_sample_index;
static int debug_experiment_1, debug_experiment_2, debug_experiment_3, debug_experiment_4, debug_experiment_5, debug_experiment_6;

// Function Prototypes
static void set_current(float current, float yaw_current);
static void terminal_render(int argc, const char **argv);
static void terminal_sample(int argc, const char **argv);
static void terminal_experiment(int argc, const char **argv);
static float app_balance_get_debug(int index);
static void app_balance_sample_debug(void);
static void app_balance_experiment(void);

// Exposed Functions
void app_balance_configure(balance_config *conf, imu_config *conf2) {
	balance_conf = *conf;
	imu_conf = *conf2;
	// Set calculated values from config
	loop_time = US2ST((int)((1000.0 / balance_conf.hertz) * 1000.0));

	motor_timeout = ((1000.0 / balance_conf.hertz)/1000.0) * 20; // Times 20 for a nice long grace period

	startup_step_size = balance_conf.startup_speed / balance_conf.hertz;
	tiltback_duty_step_size = balance_conf.tiltback_duty_speed / balance_conf.hertz;
	tiltback_hv_step_size = balance_conf.tiltback_hv_speed / balance_conf.hertz;
	tiltback_lv_step_size = balance_conf.tiltback_lv_speed / balance_conf.hertz;
	tiltback_return_step_size = balance_conf.tiltback_return_speed / balance_conf.hertz;
	torquetilt_on_step_size = balance_conf.torquetilt_on_speed / balance_conf.hertz;
	torquetilt_off_step_size = balance_conf.torquetilt_off_speed / balance_conf.hertz;
	turntilt_step_size = balance_conf.turntilt_speed / balance_conf.hertz;
	noseangling_step_size = balance_conf.noseangling_speed / balance_conf.hertz;

	//Alex changes. Added for higtemp tiltback step size. At the moment it is the same as 
	//duty tiltback until we can update VESC tool.
	tiltback_hightemp_step_size = balance_conf.tiltback_duty_speed / balance_conf.hertz;

	// Init Filters
	if(balance_conf.loop_time_filter > 0){
		loop_overshoot_alpha = 2*M_PI*((float)1/balance_conf.hertz)*balance_conf.loop_time_filter/(2*M_PI*((float)1/balance_conf.hertz)*balance_conf.loop_time_filter+1);
	}
	if(balance_conf.kd_pt1_lowpass_frequency > 0){
		float dT = 1.0 / balance_conf.hertz;
		float RC = 1.0 / ( 2.0 * M_PI * balance_conf.kd_pt1_lowpass_frequency);
		d_pt1_lowpass_k =  dT / (RC + dT);
	}
	if(balance_conf.kd_pt1_highpass_frequency > 0){
		float dT = 1.0 / balance_conf.hertz;
		float RC = 1.0 / ( 2.0 * M_PI * balance_conf.kd_pt1_highpass_frequency);
		d_pt1_highpass_k =  dT / (RC + dT);
	}
	if(balance_conf.torquetilt_filter > 0){ // Torquetilt Current Biquad
		float Fc = balance_conf.torquetilt_filter / balance_conf.hertz;
		biquad_config(&torquetilt_current_biquad, BQ_LOWPASS, Fc);
	}

	// Alex changes. Added filter init for ADC switches. Still need to test.
	if(balance_conf.yaw_current_clamp > 0){
		float dT = 1.0 / balance_conf.hertz;
		float RC = 1.0 / ( 2.0 * M_PI * balance_conf.yaw_current_clamp);
		adc_lowpass_k =  dT / (RC + dT);
	}


	if(balance_conf.roll_steer_erpm_kp > 0){
		float dT = 1.0 / balance_conf.hertz;
		float RC = 1.0 / ( 2.0 * M_PI * balance_conf.roll_steer_erpm_kp);
		roll_angle_lowpass_k =  dT / (RC + dT);
	}

	// Variable nose angle adjustment / tiltback (setting is per 1000erpm, convert to per erpm)
	tiltback_variable = balance_conf.tiltback_variable / 1000;
	if (tiltback_variable > 0) {
		tiltback_variable_max_erpm = fabsf(balance_conf.tiltback_variable_max / tiltback_variable);
	} else {
		tiltback_variable_max_erpm = 100000;
	}

	// Reset loop time variables
	last_time = 0;
	filtered_loop_overshoot = 0;
}

void app_balance_start(void) {
	// First start only, override state to startup
	state = STARTUP;
	// Register terminal commands
	terminal_register_command_callback(
		"app_balance_render",
		"Render debug values on the balance real time data graph",
		"[Field Number] [Plot (Optional 1 or 2)]",
		terminal_render);
	terminal_register_command_callback(
		"app_balance_sample",
		"Output real time values to the terminal",
		"[Field Number] [Sample Count]",
		terminal_sample);
	terminal_register_command_callback(
		"app_balance_experiment",
		"Output real time values to the experiments graph",
		"[Field Number] [Plot 1-6]",
		terminal_experiment);
	// Start the balance thread
	app_thread = chThdCreateStatic(balance_thread_wa, sizeof(balance_thread_wa), NORMALPRIO, balance_thread, NULL);
}

void app_balance_stop(void) {
	if(app_thread != NULL){
		chThdTerminate(app_thread);
		chThdWait(app_thread);
	}
	set_current(0, 0);
	terminal_unregister_callback(terminal_render);
	terminal_unregister_callback(terminal_sample);
}

float app_balance_get_pid_output(void) {
	return pid_value;
}
float app_balance_get_pitch_angle(void) {
	return pitch_angle;
}
float app_balance_get_roll_angle(void) {
	return roll_angle;
}
uint32_t app_balance_get_diff_time(void) {
	return ST2US(diff_time);
}
float app_balance_get_motor_current(void) {
	return motor_current;
}
uint16_t app_balance_get_state(void) {
	return state;
}
uint16_t app_balance_get_switch_state(void) {
	return switch_state;
}
float app_balance_get_adc1(void) {
	return adc1;
}
float app_balance_get_adc2(void) {
	return adc2;
}
float app_balance_get_debug1(void) {
	return app_balance_get_debug(debug_render_1);
}
float app_balance_get_debug2(void) {
	return app_balance_get_debug(debug_render_2);
}

// Internal Functions
static void reset_vars(void){
	// Clear accumulated values.
	integral = 0;
	last_proportional = 0;
	integral2 = 0;
	yaw_integral = 0;
	yaw_last_proportional = 0;
	d_pt1_lowpass_state = 0;
	d_pt1_highpass_state = 0;
	// Set values for startup
	rpm_filtered = 0.0;
    last_rpm_filtered = 0.0;
	setpoint = pitch_angle;
	setpoint_target_interpolated = pitch_angle;
	setpoint_target = 0;
	noseangling_interpolated = 0;
	torquetilt_target = 0;
	torquetilt_interpolated = 0;
	torquetilt_filtered_current = 0;
	biquad_reset(&torquetilt_current_biquad);
	turntilt_target = 0;
	turntilt_interpolated = 0;
	setpointAdjustmentType = CENTERING;
	yaw_setpoint = 0;
	state = RUNNING;
	current_time = 0;
	last_time = 0;
	diff_time = 0;
	brake_timeout = 0;
	// Alex Changes. Added reset of variables.
	last_proportional_filtered = 0;
	proportional_filtered = 0;
	adc1_lowpass_state =  0 ;
	adc2_lowpass_state = 0 ;
	adc1_filtered = 0 ;
	adc2_filtered = 0;
	fet_temp_limit=mc_interface_get_configuration()->l_temp_fet_start;
	motor_temp_limit=mc_interface_get_configuration()->l_temp_motor_start;
	roll_angle_filtered = 0;
	roll_angle_lowpass_state = 0;
	//app_nunchuk_get_decoded_y()=0;

}

static float get_setpoint_adjustment_step_size(void){
	switch(setpointAdjustmentType){
		case (CENTERING):
			return startup_step_size;
		case (TILTBACK_DUTY):
			return tiltback_duty_step_size;
		case (TILTBACK_HV):
			return tiltback_hv_step_size;
		case (TILTBACK_LV):
			return tiltback_lv_step_size;
		// Alex changes. Mitch left room for adding something, so I am using it here. 
		//add hightemp tiltback case. Note this will not show up in the version of 
		//VESC Tool as the correctly named state, but as an unknown state
		// until VESC tool can be updated.
		case (TILTBACK_HIGHTEMP):
			return tiltback_duty_step_size;

		case (TILTBACK_NONE):
			return tiltback_return_step_size;
		default:
			;
	}
	return 0;
}

// Fault checking order does not really matter. From a UX perspective, switch should be before angle.
static bool check_faults(bool ignoreTimers){
	// Check switch
	// Switch fully open
	if(switch_state == OFF){
		if(ST2MS(current_time - fault_switch_timer) > balance_conf.fault_delay_switch_full || ignoreTimers){
			state = FAULT_SWITCH_FULL;
			return true;
		}
	} else {
		fault_switch_timer = current_time;
	}

	// Switch partially open and stopped
	if(!balance_conf.fault_is_dual_switch) {
		if((switch_state == HALF || switch_state == OFF) && abs_erpm < balance_conf.fault_adc_half_erpm){
			if(ST2MS(current_time - fault_switch_half_timer) > balance_conf.fault_delay_switch_half || ignoreTimers){
				state = FAULT_SWITCH_HALF;
				return true;
			}
		} else {
			fault_switch_half_timer = current_time;
		}
	}

	// Check pitch angle
	if(fabsf(pitch_angle) > balance_conf.fault_pitch){
		if(ST2MS(current_time - fault_angle_pitch_timer) > balance_conf.fault_delay_pitch || ignoreTimers){
			state = FAULT_ANGLE_PITCH;
			return true;
		}
	}else{
		fault_angle_pitch_timer = current_time;
	}

	// Check roll angle
	if(fabsf(roll_angle) > balance_conf.fault_roll){
		if(ST2MS(current_time - fault_angle_roll_timer) > balance_conf.fault_delay_roll || ignoreTimers){
			state = FAULT_ANGLE_ROLL;
			return true;
		}
	}else{
		fault_angle_roll_timer = current_time;
	}

	// Check for duty
	if(abs_duty_cycle > balance_conf.fault_duty){
		if(ST2MS(current_time - fault_duty_timer) > balance_conf.fault_delay_duty || ignoreTimers){
			state = FAULT_DUTY;
			return true;
		}
	} else {
		fault_duty_timer = current_time;
	}

	return false;
}

static void calculate_setpoint_target(void){

	//Alex changes. Added limits from motor configuration and assiged to new variables.
	// fet_temp_limit=mc_interface_get_configuration()->l_temp_fet_start;
	// motor_temp_limit=mc_interface_get_configuration()->l_temp_motor_start;

	if(setpointAdjustmentType == CENTERING && setpoint_target_interpolated != setpoint_target){
		// Ignore tiltback during centering sequence
		state = RUNNING;
	}else if(abs_duty_cycle > balance_conf.tiltback_duty){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_duty_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_duty_angle;
		}
		setpointAdjustmentType = TILTBACK_DUTY;
		state = RUNNING_TILTBACK_DUTY;
	}else if(abs_duty_cycle > 0.05 && GET_INPUT_VOLTAGE() > balance_conf.tiltback_hv){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_hv_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_hv_angle;
		}
		setpointAdjustmentType = TILTBACK_HV;
		state = RUNNING_TILTBACK_HIGH_VOLTAGE;
	}else if(abs_duty_cycle > 0.05 && GET_INPUT_VOLTAGE() < balance_conf.tiltback_lv){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_lv_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_lv_angle;
		}
		setpointAdjustmentType = TILTBACK_LV;
		state = RUNNING_TILTBACK_LOW_VOLTAGE;


	//Alex Changes. Added tiltback for high mosfet or motor temp. 
	//I am combining the two into one state for now, as I don't think it is so necessary to have two states.
	// Hardcoding the limits to be 5C below the motor config limits in VESC tool for now.
	// Would like to change this to user configurable in the future within balance app.

	}else if(mc_interface_temp_fet_filtered() > (fet_temp_limit-5) || mc_interface_temp_motor_filtered() > (motor_temp_limit-5)){  
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_duty_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_duty_angle;
		}
		setpointAdjustmentType = TILTBACK_HIGHTEMP;
		state = RUNNING_TILTBACK_HIGHTEMP;

	}else{
		setpointAdjustmentType = TILTBACK_NONE;
		setpoint_target = 0;
		//setpoint_target= (app_nunchuk_get_decoded_y()*balance_conf.ki2);

		
		state = RUNNING;
	}
}

static void calculate_setpoint_interpolated(void){
	if(setpoint_target_interpolated != setpoint_target){
		// If we are less than one step size away, go all the way
		if(fabsf(setpoint_target - setpoint_target_interpolated) < get_setpoint_adjustment_step_size()){
			setpoint_target_interpolated = setpoint_target;
		}else if (setpoint_target - setpoint_target_interpolated > 0){
			setpoint_target_interpolated += get_setpoint_adjustment_step_size();
		}else{
			setpoint_target_interpolated -= get_setpoint_adjustment_step_size();
		}
	}
}

static void apply_noseangling(void){
	// Nose angle adjustment, add variable then constant tiltback
	float noseangling_target = 0;
	if (fabsf(erpm) > tiltback_variable_max_erpm) {
		noseangling_target = fabsf(balance_conf.tiltback_variable_max) * SIGN(erpm);
	} else {
		noseangling_target = tiltback_variable * erpm;
	}

	if(erpm > balance_conf.tiltback_constant_erpm){
		noseangling_target += balance_conf.tiltback_constant;
	} else if(erpm < -balance_conf.tiltback_constant_erpm){
		noseangling_target += -balance_conf.tiltback_constant;
	}

	if(fabsf(noseangling_target - noseangling_interpolated) < noseangling_step_size){
		noseangling_interpolated = noseangling_target;
	}else if (noseangling_target - noseangling_interpolated > 0){
		noseangling_interpolated += noseangling_step_size;
	}else{
		noseangling_interpolated -= noseangling_step_size;
	}
	setpoint += noseangling_interpolated;
}

static void apply_torquetilt(void){
	// Filter current (Biquad)
	// if(balance_conf.torquetilt_filter > 0){
	// 	torquetilt_filtered_current = biquad_process(&torquetilt_current_biquad, motor_current);
	// }else{
	// 	torquetilt_filtered_current  = motor_current;
	// }


	// Wat is this line O_o
	// Take abs motor current, subtract start offset, and take the max of that with 0 to get the current above our start threshold (absolute).
	// Then multiply it by "power" to get our desired angle, and min with the limit to respect boundaries.
	// Finally multiply it by sign motor current to get directionality back
	//torquetilt_target = fminf(fmaxf((fabsf(torquetilt_filtered_current) - balance_conf.torquetilt_start_current), 0) * balance_conf.torquetilt_strength, balance_conf.torquetilt_angle_limit) * SIGN(torquetilt_filtered_current);

	//torquetilt_target = fminf(fmaxf((app_nunchuk_get_decoded_y()),0) * balance_conf.torquetilt_strength, balance_conf.torquetilt_angle_limit);
	torquetilt_target = fminf((app_nunchuk_get_decoded_y()) * balance_conf.torquetilt_strength*100, balance_conf.torquetilt_angle_limit);


	float step_size;
	if((torquetilt_interpolated - torquetilt_target > 0 && torquetilt_target > 0) || (torquetilt_interpolated - torquetilt_target < 0 && torquetilt_target < 0)){
		step_size = torquetilt_off_step_size;
	}else{
		step_size = torquetilt_on_step_size;
	}

	if(fabsf(torquetilt_target - torquetilt_interpolated) < step_size){
		torquetilt_interpolated = torquetilt_target;
	}else if (torquetilt_target - torquetilt_interpolated > 0){
		torquetilt_interpolated += step_size;
	}else{
		torquetilt_interpolated -= step_size;
	}
	setpoint += torquetilt_interpolated;
}

static void apply_turntilt(void){
	// Calculate desired angle
	//turntilt_target = abs_roll_angle_sin * -1*balance_conf.turntilt_strength;
	//abs_roll_angle

	//Apply roll angle filter 
				if(balance_conf.roll_steer_erpm_kp > 0){
					roll_angle_lowpass_state = roll_angle_lowpass_state + roll_angle_lowpass_k * (roll_angle - roll_angle_lowpass_state);
					roll_angle_filtered = roll_angle_lowpass_state;
				} else {
					roll_angle_filtered = roll_angle;
				}	
	abs_roll_angle_filtered = fabsf(roll_angle_filtered);

	turntilt_target = abs_roll_angle_filtered * -0.1*balance_conf.turntilt_strength;

	// Apply cutzone
	if(abs_roll_angle < balance_conf.turntilt_start_angle){
		turntilt_target = 0;
	}

	// Disable below erpm threshold otherwise add directionality
	if(abs_erpm < balance_conf.turntilt_start_erpm){
		turntilt_target = 0;
	}else {
		turntilt_target *= SIGN(erpm);
	}

	// Apply speed scaling
	if(abs_erpm < balance_conf.turntilt_erpm_boost_end){
		turntilt_target *= 1 + ((balance_conf.turntilt_erpm_boost/100.0f) * (abs_erpm / balance_conf.turntilt_erpm_boost_end));
	}else{
		turntilt_target *= 1 + (balance_conf.turntilt_erpm_boost/100.0f);
	}

	// Limit angle to max angle
	if(turntilt_target > 0){
		turntilt_target = fminf(turntilt_target, balance_conf.turntilt_angle_limit);
	}else{
		turntilt_target = fmaxf(turntilt_target, -balance_conf.turntilt_angle_limit);
	}

	// Move towards target limited by max speed
	if(fabsf(turntilt_target - turntilt_interpolated) < turntilt_step_size){
		turntilt_interpolated = turntilt_target;
	}else if (turntilt_target - turntilt_interpolated > 0){
		turntilt_interpolated += turntilt_step_size;
	}else{
		turntilt_interpolated -= turntilt_step_size;
	}
	setpoint += turntilt_interpolated;

}

static float apply_deadzone(float error){
	if(balance_conf.deadzone == 0){
		return error;
	}

	if(error < balance_conf.deadzone && error > -balance_conf.deadzone){
		return 0;
	} else if(error > balance_conf.deadzone){
		return error - balance_conf.deadzone;
	} else {
		return error + balance_conf.deadzone;
	}
}

static void brake(void){
	// Brake timeout logic
	if(balance_conf.brake_timeout > 0 && (abs_erpm > 1 || brake_timeout == 0)){
		brake_timeout = current_time + S2ST(balance_conf.brake_timeout);
	}
	if(brake_timeout != 0 && current_time > brake_timeout){
		return;
	}

	// Reset the timeout
	timeout_reset();
	// Set current
	mc_interface_set_brake_current(balance_conf.brake_current);
	if(balance_conf.multi_esc){
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg *msg = comm_can_get_status_msg_index(i);
			if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
				comm_can_set_current_brake(msg->id, balance_conf.brake_current);
			}
		}
	}
}

static void set_current(float current, float yaw_current){
	// Limit current output to configured max output (does not account for yaw_current)
	if(current > 0 && current > mc_interface_get_configuration()->l_current_max){
		current = mc_interface_get_configuration()->l_current_max;
	}else if(current < 0 && current < mc_interface_get_configuration()->l_current_min){
		current = mc_interface_get_configuration()->l_current_min;
	}
	// Alex Changes. Had to add this when using the non-linear output to re-scale the current command.
	max_motor_current=mc_interface_get_configuration()->l_current_max;
	// Reset the timeout
	timeout_reset();
	// Set current
	if(balance_conf.multi_esc){
		// Set the current delay
		mc_interface_set_current_off_delay(motor_timeout);
		// Set Current
		mc_interface_set_current(current + yaw_current);
		// Can bus
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg *msg = comm_can_get_status_msg_index(i);

			if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
				comm_can_set_current_off_delay(msg->id, current - yaw_current, motor_timeout);// Assume 2 motors, i don't know how to steer 3 anyways
			}
		}
	} else {
		// Set the current delay
		mc_interface_set_current_off_delay(motor_timeout);
		// Set Current
		mc_interface_set_current(current);
	}
}

static THD_FUNCTION(balance_thread, arg) {
	(void)arg;
	chRegSetThreadName("APP_BALANCE");

	while (!chThdShouldTerminateX()) {
		// Update times
		current_time = chVTGetSystemTimeX();  //timer count (10,000 = 1sec).
		if(last_time == 0){
		  last_time = current_time;
		}
		diff_time = current_time - last_time;
		filtered_diff_time = 0.03 * diff_time + 0.97 * filtered_diff_time; // Purely a metric
		last_time = current_time;
		if(balance_conf.loop_time_filter > 0){
			loop_overshoot = diff_time - (loop_time - roundf(filtered_loop_overshoot));
			filtered_loop_overshoot = loop_overshoot_alpha * loop_overshoot + (1-loop_overshoot_alpha)*filtered_loop_overshoot;
		}

		// Read values for GUI
		motor_current = mc_interface_get_tot_current_directional_filtered();
		motor_position = mc_interface_get_pid_pos_now();

		// Set "last" values to previous loops values
		last_pitch_angle = pitch_angle;
		last_gyro_y = gyro[1];
		// Alex Changes. Added last_pitch_filtered, last_torquetilt_filtered_current, last_erpm, last_erpm_filtered, last_rpm_filtered, last_duty_cycle, last_Xk, last_proportional_filtered
		last_pitch_filtered=pitch_filtered;
		last_torquetilt_filtered_current=torquetilt_filtered_current;
		last_erpm = erpm;
		last_erpm_filtered = erpm_filtered;
		last_rpm_filtered=rpm_filtered;
		last_duty_cycle=duty_cycle;
		last_Xk=Xk;
		last_proportional_filtered=proportional_filtered;
		
		// Get the values we want
		float dt = UTILS_AGE_S(ReadSensorTimeStamp); //time elapsed since last sensor read, in seconds.
		ReadSensorTimeStamp = chVTGetSystemTimeX();
		pitch_angle = RAD2DEG_f(imu_get_pitch());	//read pitch from sensor cache in AHRS lib
		roll_angle = RAD2DEG_f(imu_get_roll());
		abs_roll_angle = fabsf(roll_angle);
		abs_roll_angle_sin = sinf(DEG2RAD_f(abs_roll_angle));
		imu_get_gyro(gyro);
		duty_cycle = mc_interface_get_duty_cycle_now();
		abs_duty_cycle = fabsf(duty_cycle);
		erpm = mc_interface_get_rpm();
		abs_erpm = fabsf(erpm);
		if(balance_conf.multi_esc){
			avg_erpm = erpm;
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);
				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					avg_erpm += msg->rpm;
				}
			}
			avg_erpm = avg_erpm/2;// Assume 2 motors, i don't know how to steer 3 anyways
		}
		adc1 = (((float)ADC_Value[ADC_IND_EXT])/4095) * V_REG;
#ifdef ADC_IND_EXT2
		adc2 = (((float)ADC_Value[ADC_IND_EXT2])/4095) * V_REG;
#else
		adc2 = 0.0;
#endif
		
		// Calculate switch state from ADC values
		
		//Alex Changes. I wanted to add some filtering to the ADC values. Still need to test it.

		if(balance_conf.yaw_current_clamp> 0){
					adc1_lowpass_state = adc1_lowpass_state + adc_lowpass_k * (adc1 - adc1_lowpass_state);
					adc1_filtered = adc1_lowpass_state;

				} 
				
				else {
					adc1_filtered = adc1;
		}	


		if(balance_conf.yaw_current_clamp > 0){
					adc2_lowpass_state = adc2_lowpass_state + adc_lowpass_k  * (adc2 - adc2_lowpass_state);
					adc2_filtered = adc2_lowpass_state;

				} else {
					adc2_filtered = adc2;
		}	

		if(balance_conf.fault_adc1 == 0 && balance_conf.fault_adc2 == 0){ // No Switch
			switch_state = ON;
		}else if(balance_conf.fault_adc2 == 0){ // Single switch on ADC1
			if(adc1_filtered > balance_conf.fault_adc1){
				switch_state = ON;
			} else {
				switch_state = OFF;
				
			}
		}else if(balance_conf.fault_adc1 == 0){ // Single switch on ADC2
			if(adc2_filtered > balance_conf.fault_adc2){
				switch_state = ON;
			} else {
				switch_state = OFF;
			}
		}else{ // Double switch
			if(adc1_filtered  > balance_conf.fault_adc1 && adc2_filtered > balance_conf.fault_adc2){
				switch_state = ON;
			}else if(adc1_filtered  > balance_conf.fault_adc1 || adc2_filtered > balance_conf.fault_adc2){
				if (balance_conf.fault_is_dual_switch)
					switch_state = ON;
				else
					switch_state = HALF;
			}else{
				switch_state = OFF;
			}
		}


		// Control Loop State Logic
		switch(state){
			case (STARTUP):
				// Disable output
				brake();
				if(imu_startup_done()){
					reset_vars();
					state = FAULT_STARTUP; // Trigger a fault so we need to meet start conditions to start
				}
				break;
			case (RUNNING):
			case (RUNNING_TILTBACK_DUTY):
			case (RUNNING_TILTBACK_HIGH_VOLTAGE):
			case (RUNNING_TILTBACK_LOW_VOLTAGE):

			//Alex changes. Adding case for high temp.
			case (RUNNING_TILTBACK_HIGHTEMP):

				// Check for faults
				if(check_faults(false)){
					break;
				}

				// Calculate setpoint and interpolation
				calculate_setpoint_target();
				calculate_setpoint_interpolated();
				setpoint = setpoint_target_interpolated;
				apply_noseangling();
				apply_torquetilt();
				apply_turntilt();


				//Apply Proportional filters for derivative term 
				if(balance_conf.kd_pt1_lowpass_frequency > 0){
					d_pt1_lowpass_state = d_pt1_lowpass_state + d_pt1_lowpass_k * (pitch_angle - d_pt1_lowpass_state);
					pitch_filtered = d_pt1_lowpass_state;
				} else {
					pitch_filtered = pitch_angle;
				}	
				
				// See
				// http://math.stackexchange.com/questions/297768/how-would-i-create-a-exponential-ramp-function-from-0-0-to-1-1-with-a-single-val

				// Alex Changes. Added non linear term to output. Note for test setup, ki and kp are reversed in the throttle curve function to get the desired output. 
				// Otherwise, in the app, kp would be for braking and ki is for throttle. Just semantics but easier to think about on the fly for now.
				
				if(balance_conf.yaw_kd > -1){
					pitch_filtered = utils_throttle_curve(pitch_filtered, balance_conf.yaw_ki, balance_conf.yaw_kp, balance_conf.yaw_kd);
					//pitch_filtered = utils_throttle_curve(pitch_filtered, balance_conf.yaw_kp, balance_conf.yaw_ki, balance_conf.yaw_kd)*balance_conf.roll_steer_kp;
				}	

				// // Do PID maths
				proportional = setpoint - pitch_filtered;
				// Apply deadzone
				proportional = apply_deadzone(proportional);
				// if(balance_conf.kd_pt1_highpass_frequency > 0){
				// 	d_pt1_highpass_state = d_pt1_highpass_state + d_pt1_highpass_k * (proportional - d_pt1_highpass_state);
				// 	proportional =  d_pt1_highpass_state;
				// }

				// Resume real PID maths
				integral = integral + proportional;
				
					// Apply I term limit
				if(balance_conf.ki_limit > 0 && fabsf(integral * balance_conf.ki) > balance_conf.ki_limit){
					integral = balance_conf.ki_limit / balance_conf.ki * SIGN(integral);
				}


				// An implementation of the recursive estimator (Kalman filter) described here:
				// Xk = Kk * Zk + (1 - Kk) * (Xk-1)
				// Xk = Current estimation
				// Kk = Kalman gain
				// Zk = Measured value
				// Xk–1 = Previous estimation

				//Alex changes: Repurposing the balance_conf.kd_pt1_highpass_frequency variable to be the Kk1 variable
				// note that it is multiplied by 0.001 to get the resolution we need from VESC Tool.
				//First type of derivative here, based on the setpoint error, in our case the variable "proportional"
				// 
				Kk1= (balance_conf.kd_pt1_highpass_frequency);
				if(Kk1> 0){
				proportional_filtered=Kk1*proportional*0.001 +(1-Kk1*0.001)*(last_proportional_filtered);
				}

				if(fabsf(proportional_filtered)== last_proportional_filtered) {
					derivative = 0.0;
				} else {
					derivative = (proportional_filtered-last_proportional_filtered) / ((dt*2));
				}


				//Alex changes: Repurposing balance_conf.kp2 for Kk here. This has the resolution we need from VESC Tool straight up, don't need to multiply by 0.001.
				//Second type of derivative here, based on the setpoint error, in our case the variable "proportional"
				// *after test notes 5-6-23* this derivative seems to be better than the first version in practice.

				Kk= (balance_conf.kp2);
				if(Kk > 0){
				Xk= Kk*pitch_angle +(1-Kk)*(last_Xk);
				}

				if (fabsf(Xk) == last_Xk) {
				derivative2 = 0.0;
				} else {
				derivative2 = (Xk-last_Xk) / ((dt*2));// calculate derivative using pitch angle difference and time difference.
				}

				// Alex changes. Not using the lowpass filters for D term.

				//Apply D term filters
				// if(balance_conf.kd_pt1_lowpass_frequency > 0){
				// 	d_pt1_lowpass_state = d_pt1_lowpass_state + d_pt1_lowpass_k * (derivative - d_pt1_lowpass_state);
				// 	derivative = d_pt1_lowpass_state;
				// }
				// if(balance_conf.kd_pt1_highpass_frequency > 0){
				// 	d_pt1_highpass_state = d_pt1_highpass_state + d_pt1_highpass_k * (derivative - d_pt1_highpass_state);
				// 	derivative = derivative - d_pt1_highpass_state;
				// }

				// Alex changes. There are two types of derivative terms. One is the derivative of the pitch angle, and the other is the derivative of error (the variable "proportional").
				// I am implementing both here to test which is cleaner. Vedder uses two different methods in the PID speed/position controllers. The reasoning is that the derivative of the error 
				//may be noisier than the derivative of the motor position/motor speed in his implementation. I am not sure if this is true for our application, since our "error" is relative to
				// the variable "setpoint", which is not a signal, but a calculated target, typically zero, or whatever the tiltback angle settings are. 
				// By taking the derivative of the pitch angle directly, this might end up being noisier in our case, since the pitch angle is a signal, and the derivative of a signal is typically noisier.
				// This implmentation below allows testing both methods for now. Also, I have applied a -1 to the second derivative term, and this is because of the way my IMU is mounted. Be aware that depending	
				// on the convention of your IMU, this may need to be removed. I need to come up with a foolproof way to fix this in the future, assuming that the second method for the derrivative term is better.
				// If not, then I will just remove the second derivative term altogether.

				pid_value= (balance_conf.kp * proportional) + (balance_conf.ki * integral) + (balance_conf.kd * derivative) + ((-1)*balance_conf.kd2 * derivative2); 
																															//WARNING: be aware that this -1 may need to be removed depending on the convention of your IMU.
																															// If you get this wrong, the controller WILL be unstable.
				
				// if(balance_conf.pid_mode == BALANCE_PID_MODE_ANGLE_RATE_CASCADE){
				// 	proportional2 = pid_value - gyro[1];
				// 	integral2 = integral2 + proportional2;
				// 	// Apply I term Filter
				// 	if(balance_conf.ki_limit > 0 && fabsf(integral2 * balance_conf.ki2) > balance_conf.ki_limit){
				// 		integral2 = balance_conf.ki_limit / balance_conf.ki2 * SIGN(integral2);
				// 	}

				// 	pid_value = (balance_conf.kp2 * proportional2) + (balance_conf.ki2 * integral2) + (balance_conf.kd2 * derivative2);
				// }

				// last_proportional = proportional;

				// // Apply Booster
				// abs_proportional = fabsf(proportional);
				// if(abs_proportional > balance_conf.booster_angle){
				// 	if(abs_proportional - balance_conf.booster_angle < balance_conf.booster_ramp){
				// 		pid_value += (balance_conf.booster_current * SIGN(proportional)) * ((abs_proportional - balance_conf.booster_angle) / balance_conf.booster_ramp);
				// 	}else{
				// 		pid_value += balance_conf.booster_current * SIGN(proportional);
				// 	}
				// }

				// if(balance_conf.multi_esc){
				// 	// Calculate setpoint
				// 	if(abs_duty_cycle < .02){
				// 		yaw_setpoint = 0;
				// 	} else if(avg_erpm < 0){
				// 		yaw_setpoint = (-balance_conf.roll_steer_kp * roll_angle) + (balance_conf.roll_steer_erpm_kp * roll_angle * avg_erpm);
				// 	} else{
				// 		yaw_setpoint = (balance_conf.roll_steer_kp * roll_angle) + (balance_conf.roll_steer_erpm_kp * roll_angle * avg_erpm);
				// 	}
				// 	// Do PID maths
				// 	yaw_proportional = yaw_setpoint - gyro[2];
				// 	yaw_integral = yaw_integral + yaw_proportional;
				// 	yaw_derivative = yaw_proportional - yaw_last_proportional;

				// 	yaw_pid_value = (balance_conf.yaw_kp * yaw_proportional) + (balance_conf.yaw_ki * yaw_integral) + (balance_conf.yaw_kd * yaw_derivative);

				// 	if(yaw_pid_value > balance_conf.yaw_current_clamp){
				// 		yaw_pid_value = balance_conf.yaw_current_clamp;
				// 	}else if(yaw_pid_value < -balance_conf.yaw_current_clamp){
				// 		yaw_pid_value = -balance_conf.yaw_current_clamp;
				// 	}

				// 	yaw_last_proportional = yaw_proportional;
				// }

				// Alex Changes: non linear output addition. WARNING: Note that to use as implemented, the gains have to be reduced by 100x. 
				// I am planning to change this to a more intuitive implementation in the future.

				// imu_angle_exp=balance_conf.yaw_kp;  
				// imu_angle_exp_brake=balance_conf.yaw_ki ; 
				// imu_angle_exp_mode=balance_conf.yaw_kd;   
				
				

				// if(balance_conf.yaw_kd > -1){
				// 	pid_value = utils_throttle_curve(pid_value, balance_conf.yaw_kp, balance_conf.yaw_ki, balance_conf.yaw_kd);
				// 	pid_value = pid_value*max_motor_current;
				// }	

				// Output to motor
				set_current(pid_value, yaw_pid_value);
				break;
			case (FAULT_ANGLE_PITCH):
			case (FAULT_ANGLE_ROLL):
			case (FAULT_SWITCH_HALF):
			case (FAULT_SWITCH_FULL):
			case (FAULT_STARTUP):
				// Check for valid startup position and switch state
				if(fabsf(pitch_angle) < balance_conf.startup_pitch_tolerance && fabsf(roll_angle) < balance_conf.startup_roll_tolerance && switch_state == ON){
					reset_vars();
					break;
				}
				// Disable output
				brake();
				break;
			case (FAULT_DUTY):
				// We need another fault to clear duty fault.
				// Otherwise duty fault will clear itself as soon as motor pauses, then motor will spool up again.
				// Rendering this fault useless.
				check_faults(true);
				// Disable output
				brake();
				break;
		}

		// Debug outputs
		app_balance_sample_debug();
		app_balance_experiment();

		// Delay between loops
		
		chThdSleep(loop_time - roundf(filtered_loop_overshoot));
	}

	// Disable output
	brake();
}

// Terminal commands
static void terminal_render(int argc, const char **argv) {
	if (argc == 2 || argc == 3) {
		int field = 0;
		int graph = 1;
		sscanf(argv[1], "%d", &field);
		if(argc == 3){
			sscanf(argv[2], "%d", &graph);
			if(graph < 1 || graph > 2){
				graph = 1;
			}
		}
		if(graph == 1){
			debug_render_1 = field;
		}else{
			debug_render_2 = field;
		}
	} else {
		commands_printf("This command requires one or two argument(s).\n");
	}
}

static void terminal_sample(int argc, const char **argv) {
	if (argc == 3) {
		debug_sample_field = 0;
		debug_sample_count = 0;
		sscanf(argv[1], "%d", &debug_sample_field);
		sscanf(argv[2], "%d", &debug_sample_count);
		debug_sample_index = 0;
	} else {
		commands_printf("This command requires two arguments.\n");
	}
}

static void terminal_experiment(int argc, const char **argv) {
	if (argc == 3) {
		int field = 0;
		int graph = 1;
		sscanf(argv[1], "%d", &field);
		sscanf(argv[2], "%d", &graph);
		switch(graph){
			case (1):
				debug_experiment_1 = field;
				break;
			case (2):
				debug_experiment_2 = field;
				break;
			case (3):
				debug_experiment_3 = field;
				break;
			case (4):
				debug_experiment_4 = field;
				break;
			case (5):
				debug_experiment_5 = field;
				break;
			case (6):
				debug_experiment_6 = field;
				break;
		}
		commands_init_plot("Microseconds", "Balance App Debug Data");
		commands_plot_add_graph("1");
		commands_plot_add_graph("2");
		commands_plot_add_graph("3");
		commands_plot_add_graph("4");
		commands_plot_add_graph("5");
		commands_plot_add_graph("6");
	} else {
		commands_printf("This command requires two arguments.\n");
	}
}

// Debug functions
static float app_balance_get_debug(int index){
	switch(index){
		case(1):
			return motor_position;
		case(2):
			return setpoint;
		case(3):
			return torquetilt_filtered_current;
		case(4):
			return derivative;
		case(5):
			return last_pitch_angle - pitch_angle;
		case(6):
			return motor_current;
		case(7):
			return erpm;
		case(8):
			return abs_erpm;
		case(9):
			return loop_time;
		case(10):
			return diff_time;
		case(11):
			return loop_overshoot;
		case(12):
			return filtered_loop_overshoot;
		case(13):
			return filtered_diff_time;
		case(14):
			return integral;
		case(15):
			return integral * balance_conf.ki;
		case(16):
			return integral2;
		case(17):
			return integral2 * balance_conf.ki2;
		default:
			return 0;
	}
}
static void app_balance_sample_debug(){
	if(debug_sample_index < debug_sample_count){
		commands_printf("%f", (double)app_balance_get_debug(debug_sample_field));
		debug_sample_index += 1;
	}
}
static void app_balance_experiment(){
	if(debug_experiment_1 != 0){
		commands_plot_set_graph(0);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_1));
	}
	if(debug_experiment_2 != 0){
		commands_plot_set_graph(1);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_2));
	}
	if(debug_experiment_3 != 0){
		commands_plot_set_graph(2);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_3));
	}
	if(debug_experiment_4 != 0){
		commands_plot_set_graph(3);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_4));
	}
	if(debug_experiment_5 != 0){
		commands_plot_set_graph(4);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_5));
	}
	if(debug_experiment_6 != 0){
		commands_plot_set_graph(5);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_6));
	}
}
