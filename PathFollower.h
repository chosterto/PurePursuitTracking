#ifndef PATHFOLLOWER_H_INCLUDED
#define PATHFOLLOWER_H_INCLUDED

#include "pursuit.h"

class PathFollower : public PurePursuitController {
public:
    using PurePursuitController::PurePursuitController;
    
    void GeneratePath(double spacing);
    void Update(Vec2D robotPos, double robotAngle, double leftVelocity, double rightVelocity, double trackWidth);
    bool IsDone();
    double GetLeftPower();
    double GetRightPower();
    double GetLeftVelocity();
    double GetRightVelocity();
    void SetKP(double value);
    void SetKV(double value);

private:
    double leftPower;
    double rightPower;
    double leftTargetVelocity;
    double rightTargetVelocity;
    double kP = 0;
    double kV = 0;
};

#endif