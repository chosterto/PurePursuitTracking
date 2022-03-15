#ifndef PATHFOLLOWER_H_INCLUDED
#define PATHFOLLOWER_H_INCLUDED

#include "pursuit.h"

class PathFollower : public PurePursuitController {
public:
    using PurePursuitController::PurePursuitController;
    
    /**
     * @brief Generates robot path
     * 
     * @param spacing distance between interpolated points
     */
    void GeneratePath(double spacing);
    /**
     * @brief updates current state of robot
     * 
     * @param robotPos x and y position of robot
     * @param robotAngle angle of robot
     * @param leftVelocity velocity of left motors
     * @param rightVelocity velocity of right motors
     * @param trackWidth track width of robot
     */
    void Update(Vec2D robotPos, double robotAngle, double leftVelocity, double rightVelocity, double trackWidth);
    /**
     * @brief Checks if the robot is at the end of the path
     * 
     * @return true if the robot is finished
     */
    bool IsDone();
    /**
     * @brief Gets the left power
     * 
     * @return left power of robot between [-1, 1]
     */
    double GetLeftPower();
    /**
     * @brief Gets the right power
     * 
     * @return right power of robot between [-1, 1]
     */
    double GetRightPower();
    /**
     * @brief Gets the target left velocity of robot
     * 
     * @return target left velocity measured in [units]/seconds
     */
    double GetLeftVelocity();
    /**
     * @brief Gets the target right velocity of robot
     * 
     * @return target right velocity measured in [units]/seconds
     */
    double GetRightVelocity();
    /**
     * @brief Sets the kP value
     * 
     * @param value kP value
     */
    void SetKP(double value);
    /**
     * @brief Sets the kV value
     * 
     * @param value kV value
     */
    void SetKV(double value);

private:
    double leftPower;
    double rightPower;
    double leftTargetVelocity;
    double rightTargetVelocity;
    double kP = 1.0;
    double kV = 1.0;
};

#endif