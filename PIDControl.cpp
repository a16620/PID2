#include "PIDControl.h"
#include "PlaneControl.h"

PIDControl::PIDControl() noexcept {
	kP = kD = kI = 0;
	integ_acc = last_error = last_deriv = 0;
	bIntegrating = true;

	filter = 1 / (2 * MATH_PI * 60); //(2 * MATH_PI * frequency)

	output_limit = 200;
}

void PIDControl::SetOutputLimit(const float& l) noexcept {
	output_limit = l;
}

void PIDControl::Reset()
{
	last_deriv = last_error = integ_acc = 0;
	bIntegrating = true;
}

void PIDControl::SetP(const float& p) noexcept {
	kP = p;
}

void PIDControl::SetI(const float& i) noexcept {
	kI = i;
}

void PIDControl::SetD(const float& d) noexcept {
	kD = d;
}

inline float PIDControl::P_Control(const float& error) const noexcept {
	return kP * error;
}

inline float PIDControl::I_Control(const float& error, const float& dt) noexcept {
	if (bIntegrating) //Ŭ���� ���
		integ_acc += kI * error * dt; //kI�� �̸� ���� 0�� ��쿡 ���� ������ ����
	
	return integ_acc;
}

inline float PIDControl::D_Control(const float& error, const float& dt) noexcept {
	if (kD != 0 && dt != 0) {
		const float d_value = (error - last_error) / dt;

		auto deriv = last_deriv + (dt / (filter + dt)) * (d_value - last_deriv);

		last_error = error;
		last_deriv = deriv;

		return deriv * kD;
	}

	return 0;
}

inline float PIDControl::D_Control_Direct(const float& d_error, const float& dt) noexcept
{
	if (kD != 0 && dt != 0) {
		auto deriv = last_deriv + (dt / (filter + dt)) * (d_error - last_deriv);

		last_deriv = deriv;

		return deriv * kD;
	}

	return 0;
}

float PIDControl::GetControlValue(const float& target, const float& current) noexcept {
	auto time_delta = TimeChecker::getInstance().deltaTime();

	const auto error = target - current;
	auto c_val = P_Control(error) + I_Control(error, time_delta) + D_Control(-current, time_delta);

	bool overflow = abs(c_val) > output_limit;

	bIntegrating = !overflow || comp_nsign(c_val, error);
	if (overflow)
		return copysign(output_limit, c_val);

	return c_val;
}

float PIDControl::GetControlValue(const float& target, const float& current, const float& delta) noexcept
{
	auto time_delta = TimeChecker::getInstance().deltaTime();

	const auto error = target - current;
	auto c_val = P_Control(error) + I_Control(error, time_delta) + D_Control_Direct(-delta, time_delta);

	bool overflow = abs(c_val) > output_limit;

	bIntegrating = !overflow || comp_nsign(c_val, error);
	if (overflow)
		return copysign(output_limit, c_val);

	return c_val;
}
