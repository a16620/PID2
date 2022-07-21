#pragma once
#include "PIDControl.h"
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
	~PlaneGyro() {}
	
	PlaneGyro(const PlaneGyro&) = delete;
	PlaneGyro& operator=(const PlaneGyro&) = delete;

	vec3 calibration_rotation;

	static PlaneGyro inst;
public:
	static PlaneGyro& getInstance() {
		return inst;
	}
public:
	vec3 rotation; //라디안각

	void update();
	void calibrate();
};

class TimeChecker {
	static TimeChecker inst;

	Time last_checked;
	double delta;

	TimeChecker();

	TimeChecker(const TimeChecker&) = delete;
	TimeChecker& operator=(const TimeChecker&) = delete;
public:
	void update();
	double deltaTime();

	static TimeChecker& getInstance();
};

class YawController : public PIDControl {
public:
	YawController();

	void Set(const double& angle);
};

class PitchController : public PIDControl {
	PitchController();

	void Set(const double& angle);
};

class RollController : public PIDControl {
public:
	RollController();

	void Set(const double& angle);
};