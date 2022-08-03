#include "PlaneControl.h"

constexpr float OutputLimit = 10; //PID제어의 제대 출력
constexpr float MaxAngle = MATH_PI / 3; //서보의 최대 각

PitchController::PitchController()
{
	SetOutputLimit(OutputLimit);
}

void PitchController::Set(const float& angle)
{
	float v = GetControlValue(angle, PlaneGyro::getInstance().rotation.PITCH);
	float rad_v = math_map2(v, OutputLimit, MaxAngle);
	PlaneController::SetElev(rad_v);
}

RollController::RollController()
{
	SetOutputLimit(OutputLimit);
}

void RollController::Set(const float& angle)
{
	float v = GetControlValue(angle, PlaneGyro::getInstance().rotation.ROLL);
	float rad_v = math_map2(v, OutputLimit, MaxAngle);
	PlaneController::SetAiler1(rad_v);
	PlaneController::SetAiler2(-rad_v);
}

YawController::YawController()
{
	SetOutputLimit(OutputLimit);
}

void YawController::Set(const float& angle)
{
	float v = GetControlValue(angle, PlaneGyro::getInstance().rotation.YAW);
	float rad_v = math_map2(v, OutputLimit, MaxAngle);
	PlaneController::SetRudder(rad_v);
}

void PlaneGyro::update()
{
	//읽기
	vec3 sensor_input;

	rotation = sensor_input - calibration_rotation;
}

void PlaneGyro::calibrate()
{
	vec3 sensor_input;

	calibration_rotation = sensor_input;
	update();
}

TimeChecker::TimeChecker()
{
	last_checked = now();
	delta = 0;
}

void TimeChecker::update()
{
	auto _now = now();
	delta = GetTimeDelta(last_checked, _now);
	last_checked = _now;
}

float TimeChecker::deltaTime()
{
	return delta;
}

void MasterControl::ResetController()
{
	pitch_cont.Reset();
	roll_cont.Reset();
	yaw_cont.Reset();
}

MasterControl::MasterControl(Navigator* nav)
{
	mode = MODE::MODE_BEGIN_FLIGHT;
	this->nav = nav;
}

void MasterControl::process()
{
	switch (mode)
	{
	case MODE::MODE_BEGIN_FLIGHT:
	{

		break;
	}
	case MODE::MODE_STEADY:
	{
		pitch_cont.Set(steady_rotation.PITCH);
		roll_cont.Set(steady_rotation.ROLL);
		yaw_cont.Set(steady_rotation.YAW);

		break;
	}
	case MODE::MODE_FORWARD:
	{
		pitch_cont.Set(steady_rotation.PITCH);
		yaw_cont.Set(steady_rotation.YAW);


		break;
	}
	case MODE::MODE_LAND:
		break;
	case MODE::MODE_NAV:
	{
		auto ang_adj = nav->adj_angle();

		if (abs(ang_adj.z) >= MATH_PI / 18) { //10도 이상
			float ang = copysign(math_map(abs(ang_adj.z), MATH_PI / 18, MATH_PI, 0, MaxAngle), ang_adj.z);
			roll_cont.Set(ang);
		}
		else {
			roll_cont.Set(0);
		}

		float rd_ang = math_constrain(ang_adj.z * 4, -MaxAngle, MaxAngle);

		PlaneController::SetRudder(rd_ang);
		break;
	}
	}
}

void MasterControl::setMode(MODE m)
{
	if (m == mode)
		return;

	mode = m;
	ResetController();
}

void MasterControl::setSteady()
{
	steady_rotation = PlaneGyro::getInstance().rotation;
}

void MasterControl::setSteady(vec3 std)
{
	steady_rotation = std;
}

void PlaneController::SetupPin()
{
	SetAiler1(0);
	SetAiler2(0);

	SetElev(0);

	SetRudder(0);
}

void PlaneController::SetAiler1(const float& angle)
{
	static float last = 8; //초기에 0도 세팅시 무시 방지
	if (abs(angle - last) < MATH_PI / 180) { //1도 미만 차이는 무시
		return;
	}

	last = angle;
	//실제 조종
}

void PlaneController::SetAiler2(const float& angle)
{
}

void PlaneController::SetRudder(const float& angle)
{
}

void PlaneController::SetElev(const float& angle)
{
}