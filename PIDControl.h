#pragma once
#include "ardu_utility.h"

class PIDControl
{
private:
	double kP, kI, kD;
	
	bool bIntegrating;
	double integ_acc;
	
	double last_error, last_deriv, filter;

	double output_limit;

public:
	PIDControl() noexcept;

	void SetOutputLimit(const double& l) noexcept;

	void SetP(const double& p) noexcept;
	void SetI(const double& i) noexcept;
	void SetD(const double& d) noexcept;

private:
	double P_Control(const double& error) const noexcept;
	double I_Control(const double& error, const double& dt) noexcept;
	double D_Control(const double& error, const double& dt) noexcept;

public:
	double GetControlValue(const double& target, const double& current) noexcept;
};

