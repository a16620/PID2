#pragma once
#include "vec_math.h"

class PlaneGyro;

class Navigator
{
public:
	Navigator();
private:
	Quat rotation;
	vec3 position;
	
	vec3 target_pos;

	vec3 plane_speed; //아직 사용 안함

public:
	void update();

	void setTarget(vec3 target);

	vec3 projection_angle(vec3 target) const;
	vec3 adj_angle();
};