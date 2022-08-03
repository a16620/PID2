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

//자이로 값은 3축을 한번에 읽음. 물리 읽기와 소프트웨어 읽기를 분리
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
	vec3 rotation; //라디안각

	void update();
	void calibrate();
};

//센서값 일괄 처리->시간 또한 통일
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

//엘레베이터
class PitchController : public PIDControl {
public:
	PitchController();

	void Set(const float& angle);
};

//에일러론
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