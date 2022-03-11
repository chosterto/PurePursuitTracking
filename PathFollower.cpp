#include "PathFollower.h"

#define kP 0.01
#define kV 0.1

void PathFollower::GeneratePath(double spacing) {
    Interpolate(spacing);
    SmoothPath(0.25, 0.75, 0.001);
}

double PathFollower::GetLeftPower() {
    return leftPower;
}

double PathFollower::GetRightPower() {
    return rightPower;
}

void PathFollower::Update(Vec2D robotPos, double robotAngle, double leftVelocity, double rightVelocity, double trackWidth) {
    Vec2D closest = ClosestPoint(robotPos);
    double targetVelocity = closest.speed;
    double COA = CurvatureOfArc(robotPos, robotAngle);

    double leftTargetSpeed = targetVelocity * (2 + COA*trackWidth) / 2;
    double rightTargetSpeed = targetVelocity * (2 - COA*trackWidth) / 2;
    double FB_left = kP * (leftTargetSpeed - leftVelocity);
    double FB_right = kP * (rightTargetSpeed - rightVelocity);

    leftPower = kV * leftTargetSpeed + FB_left;
    rightPower = kV * rightTargetSpeed + FB_right;
}

bool PathFollower::IsDone() {
    return lastClosestPointIdx == path.size() - 1;
}