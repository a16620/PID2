#pragma once
#include "vec_math.h"

class PlaneGyro;

class Navigator
{
public:
	Navigator();
private:
	PlaneGyro& gyro;

	Quat rotation;
	vec3 position;
	
	bool go_forward;
	vec3 forward_angle_target, target_pos;

	vec3 plane_speed; //아직 사용 안함

public:
	void update();
	
	void goForward();
	void followTarget();

	void setTarget(vec3 target);
	void setForward();

	vec3 projection_angle(vec3 target) const;
	vec3 adj_angle();
};