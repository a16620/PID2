#include "PIDControl.h"
#include "PlaneControl.h"

PIDControl::PIDControl() noexcept {
	kP = kD = kI = 0;
	rD = 0.9;
	integ_acc = last_error = last_deriv = 0;
	bIntegrating = true;

	filter = 1 / (2 * MATH_PI * 60); //(2 * MATH_PI * frequency)

	output_limit = 200;
}

void PIDControl::SetOutputLimit(const double& l) noexcept {
	output_limit = l;
}

void PIDControl::SetP(const double& p) noexcept {
	kP = p;
}

void PIDControl::SetI(const double& i) noexcept {
	kI = i;
}

void PIDControl::SetD(const double& d) noexcept {
	kD = d;
}

double PIDControl::P_Control(const double& error) const noexcept {
	return kP * error;
}

double PIDControl::I_Control(const double& error, const double& dt) noexcept {
	if (bIntegrating) //클리핑이 아닌 클램핑 방식
		integ_acc += kI * error * dt; //kI를 미리 곱해 0인 경우에 값을 더하지 않음
	
	return integ_acc;
}

double PIDControl::D_Control(const double& error, const double& dt) noexcept {
	if (kD != 0 && dt != 0) {
		const double d_value = min((error - last_error) / dt, output_limit); //클리핑

		auto deriv = last_deriv + (dt / (filter + dt)) * (d_value - last_deriv);

		last_error = error;
		last_deriv = deriv;

		return deriv * kD;
	}

	return 0;
}

double PIDControl::GetControlValue(const double& target, const double& current) noexcept {
	static auto& time = TimeChecker::getInstance();

	auto time_delta = time.deltaTime();

	const auto error = target - current;
	auto c_val = P_Control(error) + I_Control(error, time_delta) + D_Control(-current, time_delta);

	if (abs(c_val) > output_limit) {
		if (comp_sign(error, c_val))
			bIntegrating = false;
		c_val = output_limit * math_sign(c_val);
	}
	else {
		bIntegrating = true;
	}

	return c_val;
}
