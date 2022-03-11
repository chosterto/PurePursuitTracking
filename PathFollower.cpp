#include "PathFollower.h"


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

double PathFollower::GetLeftVelocity() {
    return leftTargetVelocity;
}

double PathFollower::GetRightVelocity() {
    return rightTargetVelocity;
}

void PathFollower::SetKP(double value) {
    kP = value;
}

void PathFollower::SetKV(double value) {
    kV = value;
}


void PathFollower::Update(Vec2D robotPos, double robotAngle, double leftVelocity, double rightVelocity, double trackWidth) {
    Vec2D closest = ClosestPoint(robotPos);
    double targetVelocity = closest.speed;
    double COA = CurvatureOfArc(robotPos, robotAngle);

    leftTargetVelocity = targetVelocity * (2 + COA*trackWidth) / 2;
    rightTargetVelocity = targetVelocity * (2 - COA*trackWidth) / 2;
    double FB_left = kP * (leftTargetVelocity - leftVelocity);
    double FB_right = kP * (rightTargetVelocity - rightVelocity);

    // round up to 2 decimal points
    leftPower = floor((kV * leftTargetVelocity + FB_left) * 100 + 0.5) / 100;
    rightPower = floor((kV * rightTargetVelocity + FB_right) * 100 + 0.5) / 100;
}

bool PathFollower::IsDone() {
    return lastClosestPointIdx == path.size() - 1;
}