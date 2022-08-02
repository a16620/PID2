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

	inline static PlaneGyro& getInstance();

public:
	vec3 rotation; //���Ȱ�

	void update();
	void calibrate();
};

//������ �ϰ� ó��->�ð� ���� ����
class TimeChecker {
	Time last_checked;
	double delta;

	TimeChecker();
public:
	TimeChecker(const TimeChecker&) = delete;
	TimeChecker& operator=(const TimeChecker&) = delete;
	void update();
	double deltaTime();

	inline static TimeChecker& getInstance();
};

//����������
class PitchController : protected PIDControl {
public:
	PitchController();

	void Set(const double& angle);
};

//���Ϸ���
class RollController : protected PIDControl {
public:
	RollController();

	void Set(const double& angle);
};

class MasterControl {
public:
	enum class MODE {
		MODE_BEGIN_FLIGHT,
		MODE_STEADY,
		MODE_LAND,
		MODE_NAV
	};
private:
	Navigator* nav;

	PitchController pitch_cont;
	RollController roll_cont;
	PIDControl yaw_cont;

	MODE mode;

	vec3 steady_rotation;

public:
	MasterControl() = delete;
	MasterControl(Navigator* nav);

	void process();

	void setSteady();
};