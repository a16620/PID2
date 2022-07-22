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

	vec3 plane_speed;

public:
	void update();
	
	void goForward();
	void followTarget();

	vec3 projection_angle(vec3 target) const;
	vec3 adj_angle();
};