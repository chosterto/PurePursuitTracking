#include "PathFollower.h"


void PathFollower::GeneratePath(double spacing) {
    Interpolate(spacing);
    SmoothPath(0.25, 0.75, 0.001);
}

double PathFollower::GetLeftWheelSpeed() {
    return leftSpeed;
}

double PathFollower::GetRightWheelSpeed() {
    return rightSpeed;
}

void PathFollower::Update(Vec2D robotPos, double robotAngle, double trackWidth) {
    Vec2D closest = ClosestPoint(robotPos);
    double targetVelocity = closest.speed;
    double COA = CurvatureOfArc(robotPos, robotAngle);

    leftSpeed = targetVelocity * (2 + COA*trackWidth) / 2;
    rightSpeed = targetVelocity * (2 - COA*trackWidth) / 2;
}
