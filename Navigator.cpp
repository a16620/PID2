#include "Navigator.h"
#include "PlaneControl.h"

Navigator::Navigator() : gyro(PlaneGyro::getInstance())
{
    go_forward = true;
}

void Navigator::update()
{
    rotation = Quat::Euler(gyro.rotation);
    position += Quat::rotate(plane_speed, rotation)*TimeChecker::getInstance().deltaTime();
}

void Navigator::goForward()
{
    go_forward = true;
}

void Navigator::followTarget()
{
    go_forward = false;
}

void Navigator::setTarget(vec3 target)
{
    target_pos = target;
}

void Navigator::setForward()
{
    forward_angle_target = gyro.rotation; //���� �ڼ� ���
}

vec3 Navigator::projection_angle(vec3 target) const
{
    vec3 angle;

    auto rel_pos = target - position;

    //ȸ���� ������� ���⺤��
    const auto up = Quat::rotate(vec3::up, rotation);
    const auto right = Quat::rotate(vec3::right, rotation);
    const auto forward = Quat::rotate(vec3::forward, rotation);

    //������ �Ÿ� ���
    //3d�� 2d�� ����
    auto prj_y = vec3::dot(right, rel_pos);
    auto prj_z = vec3::dot(up, rel_pos);
    auto prj_x = vec3::dot(forward, rel_pos);

    const auto size = rel_pos.size();

    angle.y = asin(prj_z / size); //pitch ��
    angle.z = asin(prj_y / size); //yaw ��
    angle.x = prj_x; //�Ÿ�


    return angle;
}

vec3 Navigator::adj_angle()
{
    if (go_forward) {
        return forward_angle_target-gyro.rotation;
    }
    else {
        return projection_angle(target_pos);
    }
}
