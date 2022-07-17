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

//���̷� ���� 3���� �ѹ��� ����. ���� �б�� ����Ʈ���� �б⸦ �и�
class PlaneGyro {
private:
	PlaneGyro() {}
	~PlaneGyro() {}
	
	PlaneGyro(const PlaneGyro&) = delete;
	PlaneGyro& operator=(const PlaneGyro&) = delete;

	static PlaneGyro inst;
public:
	static PlaneGyro& getInstance() {
		return inst;
	}
public:
	Quat rotation;//�����
	vec3 rot_vec; //���Ȱ�

	void update();
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