/**
 * @file Pid.h
 * @brief Class for implementing discrete-time PID controllers
 * @author Dan Oates (WPI Class of 2020)
 * 
 * This class implements PID control with the following features:
 * 
 * - Output-constrained: Output u limited to range [u_min, u_max]
 * - Anti-windup: PID terms each constrained separately to prevent windup
 * - Fixed-frequency: Control frequency f_ctrl supplied at construction
 * - Transient-removal: Derivative term is ignored on first frame after reset
 * - Reset method: Resets differentiator and zeros integrator
 * 
 * All calculations are done in IEEE 32-bit floating-point.
 * 
 * Dependencies:
 * - CppUtil: https://github.com/doates625/CppUtil.git
 * 
 * References:
 * - PID Control: https://en.wikipedia.org/wiki/PID_controller
 */
#pragma once

class Pid
{
public:
	Pid(float kp, float ki, float kd, float u_min, float u_max, float f_ctrl);
	Pid();
	float update(float error, float ff = 0.0f);
	void reset();
private:
	float kp, ki, kd, u_min, u_max, f_ctrl, t_ctrl;
	float u, up, ui, ud;
	float error_prev;
	bool first_frame;
};