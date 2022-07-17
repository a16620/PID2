#pragma once
#include "ardu_utility.h"

class PIDControl
{
private:
	double kP, kI, kD, rD;
	
	bool bIntegrating;
	double integ_acc;
	
	Time last_check;
	double last_error, last_deriv, filter;

	double output_limit;

public:
	PIDControl() noexcept {
		kP = kD = kI = 0;
		rD = 0.9;
		integ_acc = last_error = last_deriv = 0;
		bIntegrating = true;

		filter = 1 / (2 * MATH_PI * 60); //(2 * MATH_PI * frequency)

		output_limit = 200;
	}

	void Begin() noexcept {
		last_check = now();
	}

	void SetOutputLimit(const double& l) noexcept {
		output_limit = l;
	}

	void SetP(const double& p) noexcept {
		kP = p;
	}

	void SetI(const double& i) noexcept {
		kI = i;
	}

	void SetD(const double& d) noexcept {
		kD = d;
	}

private:
	double P_Control(const double& error) const noexcept {
		return kP * error;
	}

	double I_Control(const double& error, const double& dt) noexcept {
		if (bIntegrating)
			integ_acc += kI * error * dt;
		//limit?
		return integ_acc;
	}

	double D_Control(const double& error, const double& dt) noexcept {	
		if (kD != 0 && dt != 0) {
			const double d_value = min((error - last_error) / dt, output_limit); //Å¬¸®ÇÎ

			auto deriv = last_deriv + (dt / (filter + dt)) * (d_value - last_deriv);

			last_error = error;
			last_deriv = deriv;

			return deriv * kD;
		}

		return 0;
	}

public:
	double GetControlValue(const double& target, const double& current) noexcept {
		const Time current_time = now();
		auto time_delta = GetTimeDelta(current_time, last_check);
		
		const auto error = target - current;
		auto c_val =  P_Control(error) + I_Control(error, time_delta) + D_Control(-current, time_delta);
		
		last_check = current_time;


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
};

