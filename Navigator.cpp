#include "Navigator.h"
#include "PlaneControl.h"

Navigator::Navigator()
{
}

void Navigator::update()
{
    rotation = Quat::Euler(PlaneGyro::getInstance().rotation);
    position += Quat::rotate(plane_speed, rotation)*TimeChecker::getInstance().deltaTime();
}

void Navigator::setTarget(vec3 target)
{
    target_pos = target;
}

vec3 Navigator::projection_angle(vec3 target) const
{
    vec3 angle;

    auto rel_pos = target - position;

    //회전된 비행기의 방향벡터
    const auto up = Quat::rotate(vec3::up, rotation);
    const auto right = Quat::rotate(vec3::right, rotation);
    const auto forward = Quat::rotate(vec3::forward, rotation);

    //평면과의 거리 계산
    //3d를 2d로 투시
    auto prj_y = vec3::dot(right, rel_pos);
    auto prj_z = vec3::dot(up, rel_pos);
    auto prj_x = vec3::dot(forward, rel_pos);

    const auto size = rel_pos.size();

    angle.PITCH = asinf(prj_z / size); //pitch 차
    angle.YAW = asinf(prj_y / size); //yaw 차
    angle.ROLL = prj_x; //거리


    return angle;
}

vec3 Navigator::adj_angle()
{
    //나중에 수정
    return projection_angle(target_pos);
}
