#pragma once
#include "ardu_utility.h"

class PIDControl
{
private:
	float kP, kI, kD;
	
	bool bIntegrating;
	float integ_acc;
	
	float last_error, last_deriv, filter;

	float output_limit;

public:
	PIDControl() noexcept;

	void SetOutputLimit(const float& l) noexcept;

	void Reset();

	void SetP(const float& p) noexcept;
	void SetI(const float& i) noexcept;
	void SetD(const float& d) noexcept;

private:
	inline float P_Control(const float& error) const noexcept;
	inline float I_Control(const float& error, const float& dt) noexcept;
	inline float D_Control(const float& error, const float& dt) noexcept;
	
	inline float D_Control_Direct(const float& d_error, const float& dt) noexcept;


public:
	float GetControlValue(const float& target, const float& current) noexcept;
	float GetControlValue(const float& target, const float& current, const float& delta) noexcept;
};

