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
	set_gains(kp, ki, kd);
	set_limits(u_min, u_max);
	reset();
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
 * @brief Updates PID gains
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void PID::set_gains(float kp, float ki, float kd)
{
	set_kp(kp);
	set_ki(ki);
	set_kd(kd);
}

/**
 * @brief Updates P-gain
 * @param kp Proportional gain
 */
void PID::set_kp(float kp)
{
	this->kp = kp;
}

/**
 * @brief Updates I-gain
 * @param kp Integral gain
 */
void PID::set_ki(float ki)
{
	this->ki = ki * t_ctrl;
}

/**
 * @brief Updates D-gain
 * @param kp Derivative gain
 */
void PID::set_kd(float kd)
{
	this->kd = kd * f_ctrl;
}

/**
 * @brief Updates response limits
 * @param u_min Minimum response
 * @param u_max Maximum response
 */
void PID::set_limits(float u_min, float u_max)
{
	set_u_max(u_max);
	set_u_min(u_min);
}

/**
 * @brief Updates minimum response
 * @param u_min Minimum response
 */
void PID::set_u_min(float u_min)
{
	this->u_min = u_min;
}

/**
 * @brief Updates maximum response
 * @param u_max Maximum response
 */
void PID::set_u_max(float u_max)
{
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
	if ((error > 0.0f && u < u_max) || (error < 0.0f && u > u_min))
	{
		ui = clamp(ui + ki * error, u_min, u_max);
	}
	if (first_frame)
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
	error_prev = 0.0f;
	first_frame = true;
}
