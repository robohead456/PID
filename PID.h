/**
 * @file PID.h
 * @brief Class for implementing discrete-time PID controllers
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once

class PID
{
public:
	PID(float kp, float ki, float kd, float u_min, float u_max, float f_ctrl);
	PID();
	void set_limits(float u_min, float u_max);
	float update(float error, float ff = 0.0f);
	void reset();
private:
	float kp, ki, kd, u_min, u_max, f_ctrl, t_ctrl;
	float u, up, ui, ud;
	float error_prev;
	bool first_frame;
};
