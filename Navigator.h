#pragma once
#include "PlaneControl.h"
#include "vec_math.h"

class Navigator
{
public:
	Navigator();
private:
	Quat* rotation;
	vec3 position;
public:
	
	vec3 projection_angle(vec3 target) const;
};

