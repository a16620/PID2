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

//자이로 값은 3축을 한번에 읽음. 물리 읽기와 소프트웨어 읽기를 분리
class PlaneGyro {
private:
	PlaneGyro() {}

	vec3 calibration_rotation;

public:
	PlaneGyro(const PlaneGyro&) = delete;
	PlaneGyro& operator=(const PlaneGyro&) = delete;

	inline static PlaneGyro& getInstance();

public:
	vec3 rotation; //라디안각

	void update();
	void calibrate();
};

//센서값 일괄 처리->시간 또한 통일
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

//엘레베이터
class PitchController : protected PIDControl {
public:
	PitchController();

	void Set(const double& angle);
};

//에일러론
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