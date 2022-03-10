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

private:
    double leftPower;
    double rightPower;
};

#endif