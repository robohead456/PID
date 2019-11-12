/**
 * @file PID.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "PID.h"
#include <CppUtil.h>
using CppUtil::clamp;

/**
 * @brief Constructor for PID controller
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param u_min Minimum response
 * @param u_max Maximum response
 * @param f_ctrl Control frequency
 */
PID::PID(float kp, float ki, float kd, float u_min, float u_max, float f_ctrl)
{
	this->f_ctrl = f_ctrl;
	this->t_ctrl = 1.0f / f_ctrl;
	this->kp = kp;
	this->ki = ki * t_ctrl;
	this->kd = kd * f_ctrl;
	this->u_min = u_min;
	this->u_max = u_max;
	this->u = 0.0f;
	this->up = 0.0f;
	this->ui = 0.0f;
	this->ud = 0.0f;
	this->error_prev = 0.0f;
	this->first_frame = true;
}

/**
 * @brief Default constructor for PID controller
 * 
 * Note: For array instantiation only
 */
PID::PID() : PID(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f)
{
	return;
}

/**
 * @brief Updates response limits
 * @param u_min Minimum response
 * @param u_max Maximum response
 */
void PID::set_limits(float u_min, float u_max)
{
	this->u_min = u_min;
	this->u_max = u_max;
}

/**
 * @brief Calculates response to given error
 * @param error Setpoint error
 * @param ff Feed-forward term (optional)
 */
float PID::update(float error, float ff)
{
	up = clamp(kp * error, u_min, u_max);
	if(u_min < u && u < u_max)
	{	
		ui = clamp(ui + ki * error, u_min, u_max);
	}
	if(first_frame)
	{
		ud = 0.0f;
		first_frame = false;
	}
	else
	{
		ud = clamp(kd * (error - error_prev), u_min, u_max);
	}
	error_prev = error;
	ff = clamp(ff, u_min, u_max);
	u = clamp(up + ui + ud + ff, u_min, u_max);
	return u;
}

/**
 * @brief Resets PID controller (zeros derivative and integral terms)
 */
void PID::reset()
{
	u = 0.0f;
	up = 0.0f;
	ui = 0.0f;
	ud = 0.0f;
	first_frame = true;
}
