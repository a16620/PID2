#include "Navigator.h"

vec3 Navigator::projection_angle(vec3 target) const
{
    vec3 angle;

    auto rel_pos = target - position;

    const auto up = Quat::rotate(vec3::up, rotation);
    const auto right = Quat::rotate(vec3::right, rotation);
    const auto forward = Quat::rotate(vec3::forward, rotation);

    auto prj_x = vec3::dot(right, rel_pos);
    auto prj_y = vec3::dot(up, rel_pos);
    auto prj_z = vec3::dot(forward, rel_pos);

    const auto size = rel_pos.size();

    angle.x = asin(prj_y / size); //pitch
    angle.y = asin(prj_x / size); //yaw
    //angle.z = asin(prj_z / size); //사용안함


    return vec3();
}
