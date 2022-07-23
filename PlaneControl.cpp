#include "PlaneControl.h"

constexpr double OutputLimit = 10; //PID������ ���� ���
constexpr double MaxAngle = MATH_PI / 3; //������ �ִ� ��

PitchController::PitchController()
{
	SetOutputLimit(OutputLimit);
}

void PitchController::Set(const double& angle)
{
	double v = GetControlValue(angle, PlaneGyro::getInstance().rotation.y);
	double cv = math_map2(v, OutputLimit, MaxAngle);
	PlaneController::SetElev(cv);
}

RollController::RollController()
{
	SetOutputLimit(OutputLimit);
}

void RollController::Set(const double& angle)
{
	double v = GetControlValue(angle, PlaneGyro::getInstance().rotation.x);
	double cv = math_map2(v, OutputLimit, MaxAngle);
	PlaneController::SetAiler1(cv);
	PlaneController::SetAiler2(-cv);
}

void PlaneGyro::update()
{
	//�б�
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

double TimeChecker::deltaTime()
{
	return delta;
}

TimeChecker& TimeChecker::getInstance()
{
	static TimeChecker inst;

	return inst;
}

MasterControl::MasterControl(Navigator* nav)
{
	this->nav = nav;
}

void MasterControl::process()
{
	nav->update();

	auto ang_adj = nav->adj_angle();

	if (abs(ang_adj.z) >= MATH_PI / 18) { //10�� �̻�
		double ang = math_map(abs(ang_adj.z), MATH_PI / 18, MATH_PI, 0, MaxAngle)*math_sign(ang_adj.z);
		roll_cont.Set(ang);
	}
	else {
		roll_cont.Set(0);
	}

	double rd_ang = math_constrain(ang_adj.z*4, -MaxAngle, MaxAngle);

	PlaneController::SetRudder(rd_ang);
}

void PlaneController::SetupPin()
{
	SetAiler1(0);
	SetAiler2(0);

	SetElev(0);

	SetRudder(0);
}

void PlaneController::SetAiler1(const double& angle)
{
	static double last = 8; //�ʱ⿡ 0�� ���ý� ���� ����
	if (abs(angle - last) < MATH_PI / 180) { //1�� �̸� ���̴� ����
		return;
	}

	last = angle;
	//���� ����
}

void PlaneController::SetAiler2(const double& angle)
{
}

void PlaneController::SetRudder(const double& angle)
{
}

void PlaneController::SetElev(const double& angle)
{
}
