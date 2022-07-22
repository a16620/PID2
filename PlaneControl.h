#pragma once
#include "PIDControl.h"
#include "Navigator.h"
#include "vec_math.h"

class PlaneController {
public:
	static void SetupPin();

	static void SetAiler1(const double& angle);
	static void SetAiler2(const double& angle);

	static void SetRudder(const double& angle);
	
	static void SetElev(const double& angle);
};

//���̷� ���� 3���� �ѹ��� ����. ���� �б�� ����Ʈ���� �б⸦ �и�
class PlaneGyro {
private:
	PlaneGyro() {}

	vec3 calibration_rotation;

public:
	PlaneGyro(const PlaneGyro&) = delete;
	PlaneGyro& operator=(const PlaneGyro&) = delete;

	static PlaneGyro& getInstance() {
		static PlaneGyro inst;

		return inst;
	}
public:
	vec3 rotation; //���Ȱ�

	void update();
	void calibrate();
};

class TimeChecker {
	Time last_checked;
	double delta;

	TimeChecker();
public:
	TimeChecker(const TimeChecker&) = delete;
	TimeChecker& operator=(const TimeChecker&) = delete;
	void update();
	double deltaTime();

	static TimeChecker& getInstance();
};

class PitchController : protected PIDControl {
public:
	PitchController();

	void Set(const double& angle);
};

class RollController : protected PIDControl {
public:
	RollController();

	void Set(const double& angle);
};

class MasterControl {
public:
	Navigator* nav;

private:
	PitchController pitch_cont;
	RollController roll_cont;

public:
	MasterControl() = delete;
	MasterControl(Navigator* nav);

	void process();
};