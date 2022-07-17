#include "Navigator.h"

Navigator::Navigator()
{
    rotation = &PlaneGyro::getInstance().rotation;
    go_forward = true;
}

void Navigator::update()
{
    position += Quat::rotate(plane_speed, *rotation);
}

void Navigator::goForward()
{
    go_forward = true;
    forward_angle_target = PlaneGyro::getInstance().rot_vec;
}

void Navigator::followTarget()
{
    go_forward = false;
}

vec3 Navigator::projection_angle(vec3 target) const
{
    vec3 angle;

    auto rel_pos = target - position;

    const auto up = Quat::rotate(vec3::up, *rotation);
    const auto right = Quat::rotate(vec3::right, *rotation);
    const auto forward = Quat::rotate(vec3::forward, *rotation);

    auto prj_x = vec3::dot(right, rel_pos);
    auto prj_y = vec3::dot(up, rel_pos);
    auto prj_z = vec3::dot(forward, rel_pos);

    const auto size = rel_pos.size();

    angle.x = asin(prj_y / size); //pitch 차
    angle.y = asin(prj_x / size); //yaw 차
    angle.z = prj_z; //거리
    //angle.z = asin(prj_z / size); //사용안함


    return angle;
}

vec3 Navigator::adj_angle()
{
    if (go_forward) {
        return forward_angle_target-PlaneGyro::getInstance().rot_vec;
    }
    else {
        return projection_angle(target_pos);
    }
}
