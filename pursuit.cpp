#include "pursuit.h"
#include <iostream>

#define DISTANCE(x, y) (sqrt((x)*(x) + (y)*(y)))

PurePursuitController::PurePursuitController(double maxSpeed, double maxAcceleration): 
l_maxSpeed(maxSpeed), l_maxAcceleration(maxAcceleration) {}


double PurePursuitController::Curvature(Vec2D P, Vec2D Q, Vec2D R) {
    double x1 = P.x,
           y1 = P.y,
           x2 = Q.x,
           y2 = Q.y,
           x3 = R.x,
           y3 = R.y;

    if (x1 == x2) x1 += 0.001;
    double k1 = 0.5 * (x1*x1 + y1*y1 - x2*x2 - y2*y2) / (x1 - x2);
    double k2 = (y1 - y2) / (x1 - x2);
    double b = 0.5 * (x2*x2 - 2*x2*k1 + y2*y2 - x3*x3 + 2*x3*k1 - y3*y3) / (x3*k2 - y3 + y2 - x2*k2);
    double a = k1 - k2 * b;
    double r = DISTANCE(x1 - a, y1 - b);
    
    double curvature = 1 / r;
    return std::isnan(curvature) ? 0 : curvature;
}


void PurePursuitController::CalculateDistances() {
    double distance = 0;
    Vec2D previous;
    for (unsigned int i = 0; i < path.size() - 1; i++) {
        Vec2D neighbour = path[i + 1];
        distance += DISTANCE(neighbour.x - previous.x, neighbour.y - previous.y);
        path[i + 1].distance = distance;
        previous.x = neighbour.x;
        previous.y = neighbour.y;
    }
    path[0].distance = 0;
}

void PurePursuitController::CalculateTargetVelocities() {
    
}

double PurePursuitController::FindMaxVelocity(int index, double maxVelo, double k) {
    Vec2D point = path[index];
    Vec2D backpoint = path[index - 1];
    Vec2D frontpoint = path[index + 1];
    if (index > 0) {
        double curvature = Curvature(backpoint, point, frontpoint);
        return std::min(maxVelo, k / curvature);
    }
    return maxVelo;
}

std::vector<Vec2D> PurePursuitController::GetPath() {
    return path;
}

void PurePursuitController::InjectWaypoint(double x, double y) {
    Vec2D p;
    p.x = x;
    p.y = y;
    p.speed = l_maxSpeed;
    path.push_back(p);
    CalculateDistances();
}

void PurePursuitController::SetOrigin(double x, double y) {
    Vec2D origin;
    origin.x = x;
    origin.y = y;
    origin.speed = l_maxSpeed;
    origin.distance = 0;
    if (path.size() == 0) {
        path.push_back(origin);
    } else {
        path[0] = origin;
    }
    CalculateDistances();
}


void PurePursuitController::Interpolate(double spacing) {
    std::vector<Vec2D> more_points;

    for (unsigned int i = 0; i < path.size() - 1; i++) {
        Vec2D start = path[i];
        Vec2D end = path[i + 1];
        Vec2D v{end.x - start.x, end.y - start.y, l_maxSpeed};

        double magnitude = DISTANCE(v.x, v.y);

        double num_of_points = ceil(magnitude / spacing);

        v.x = v.x / magnitude * spacing;
        v.y = v.y / magnitude * spacing;

        Vec2D new_point;
        for (int j = 0; j < num_of_points; j++) {
            new_point.x = start.x + v.x * j;
            new_point.y = start.y + v.y * j;
            more_points.push_back(new_point);
        }
    }
    more_points.push_back(path.back());
    path = more_points;
    // Update distances
    CalculateDistances();
}


void PurePursuitController::SmoothPath(double a, double b, double tolerance) {
    std::vector<Vec2D> new_path(path);
    
    double change = tolerance;
    while (change >= tolerance) {
        change = 0.0;
        for (unsigned int i = 1; i < path.size() - 1; i++) {
            Vec2D aux = new_path[i];
            Vec2D prev = new_path[i - 1];
            Vec2D next = new_path[i + 1];

            new_path[i].x += a * (path[i].x - aux.x) + b * (prev.x + next.x - 2.0 * aux.x);
            new_path[i].y += a * (path[i].y - aux.y) + b * (prev.y + next.y - 2.0 * aux.y);
            change += std::abs(aux.x - new_path[i].x) + std::abs(aux.y - new_path[i].y);
        }
    }
    path = new_path;
    // Update distances
    CalculateDistances();
}


