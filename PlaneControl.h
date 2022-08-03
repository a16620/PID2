#pragma once
#include "PIDControl.h"
#include "Navigator.h"
#include "vec_math.h"

#define YAW z
#define PITCH y
#define ROLL x

class PlaneController {
public:
	static void SetupPin();

	static void SetAiler1(const float& angle);
	static void SetAiler2(const float& angle);

	static void SetRudder(const float& angle);
	
	static void SetElev(const float& angle);
};

//���̷� ���� 3���� �ѹ��� ����. ���� �б�� ����Ʈ���� �б⸦ �и�
class PlaneGyro {
private:
	PlaneGyro() {}

	vec3 calibration_rotation;

public:
	PlaneGyro(const PlaneGyro&) = delete;
	PlaneGyro& operator=(const PlaneGyro&) = delete;

	inline static PlaneGyro& getInstance()
	{
		static PlaneGyro inst;
		return inst;
	}

public:
	vec3 rotation; //���Ȱ�

	void update();
	void calibrate();
};

//������ �ϰ� ó��->�ð� ���� ����
class TimeChecker {
	Time last_checked;
	float delta;

	TimeChecker();
public:
	TimeChecker(const TimeChecker&) = delete;
	TimeChecker& operator=(const TimeChecker&) = delete;
	void update();
	float deltaTime();

	inline static TimeChecker& getInstance()
	{
		static TimeChecker inst;
		return inst;
	}
};

//����������
class PitchController : public PIDControl {
public:
	PitchController();

	void Set(const float& angle);
};

//���Ϸ���
class RollController : public PIDControl {
public:
	RollController();

	void Set(const float& angle);
};

class YawController : public PIDControl {
public:
	YawController();

	void Set(const float& angle);
};

class MasterControl {
public:
	enum class MODE {
		MODE_BEGIN_FLIGHT,
		MODE_STEADY,
		MODE_FORWARD,
		MODE_LAND,
		MODE_NAV
	};
private:
	Navigator* nav;

	PitchController pitch_cont;
	RollController roll_cont;
	YawController yaw_cont;

	MODE mode;

	vec3 steady_rotation;

	void ResetController();

public:
	MasterControl() = delete;
	MasterControl(Navigator* nav);

	void process();

	void setMode(MODE m);

	void setSteady();
	void setSteady(vec3 std);
};