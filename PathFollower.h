#ifndef PATHFOLLOWER_H_INCLUDED
#define PATHFOLLOWER_H_INCLUDED

#include "pursuit.h"

class PathFollower : public PurePursuitController {
public:
    using PurePursuitController::PurePursuitController;
    
    void GeneratePath(double spacing);
    double GetLeftWheelSpeed();
    double GetRightWheelSpeed();
    void Update(Vec2D robotPos, double robotAngle, double trackWidth);

private:
    double leftSpeed;
    double rightSpeed;

};

#endif