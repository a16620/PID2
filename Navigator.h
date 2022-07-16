#pragma once
#include "PlaneControl.h"
#include "vec_math.h"

class Navigator
{
	Quat rotation;
	vec3 position;

	vec3 projection_angle(vec3 target) const;
};

