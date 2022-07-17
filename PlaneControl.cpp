#include "PlaneControl.h"

YawController::YawController()
{
	SetOutputLimit(100);
	Begin();
}

void YawController::Set(const double& angle)
{
	double error = angle - PlaneGyro::getInstance().rot_vec.y;
	double v = GetControlValue(error);
	short cv = map(v, -100, 100, -MATH_PI / 2, MATH_PI / 2);
	PlaneController::SetRudder(cv);
}

PitchController::PitchController()
{
	SetOutputLimit(100);
	Begin();
}

void PitchController::Set(const double& angle)
{
	double error = angle - PlaneGyro::getInstance().rot_vec.x;
	double v = GetControlValue(error);
	short cv = map(v, -100, 100, -MATH_PI / 2, MATH_PI / 2);
	PlaneController::SetElev(cv);
}

RollController::RollController()
{
	SetOutputLimit(100);
	Begin();
}

void RollController::Set(const double& angle)
{
	double error = angle - PlaneGyro::getInstance().rot_vec.z;
	double v = GetControlValue(error);
	short cv = map(v, -100, 100, -MATH_PI / 2, MATH_PI / 2);
	PlaneController::SetAiler1(cv);
	PlaneController::SetAiler2(-cv);
}

void PlaneGyro::update()
{
	//읽기

	//쿼터니언 변환
	rotation = Quat::Euler(rot_vec.x, rot_vec.y, rot_vec.z);
}
