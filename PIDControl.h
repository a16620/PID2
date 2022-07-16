#pragma once
#include "ardu_time.h"
#include "ardu_math.h"

class PIDControl
{
private:
	double kP, kI, kD, rD;
	
	bool bIntegrating;
	double integ_acc;
	
	Time last_check;
	double last_error, fder;

	double output_limit;

public:
	PIDControl() noexcept {
		kP = kD = kI = 0;
		rD = 0.9;
		integ_acc = last_error = fder = 0;
		bIntegrating = true;

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

	double I_Control(const double& error) noexcept {
		if (bIntegrating)
			integ_acc += error;
		//limit?
		return integ_acc * kI;
	}

	double D_Control(const double& error) noexcept {
		const Time current_time = now();
		
		const double sec_delta = max(0.05, GetTimeDelta(current_time, last_check));
		const double d_value = (error - last_error) / sec_delta;

		last_check = current_time;
		last_error = error;

		fder = rD * fder + (1 - rD) * d_value;

		return fder * kD;
	}

public:
	double GetControlValue(const double& error) noexcept {
		auto c_val =  P_Control(error) + I_Control(error) + D_Control(error);

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

