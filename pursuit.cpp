#include "pursuit.h"
#include <iostream>

#define DEG_TO_RAD (M_PI / 180)
#define RAD_TO_DEG (180 / M_PI)
#define DISTANCE(x, y) (sqrt((x)*(x) + (y)*(y)))


template <typename T> int signum(T val) {
    return (T(0) < val) - (val < T(0));
}


PurePursuitController::PurePursuitController(double maxSpeed, double maxAcceleration, double lookAheadDistance): 
l_maxSpeed(maxSpeed), l_maxAcceleration(maxAcceleration), l_lookAheadDistance(lookAheadDistance) {}


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
    Vec2D previous = path[0];
    for (unsigned int i = 0; i < path.size() - 1; i++) {
        Vec2D neighbour = path[i + 1];
        distance += DISTANCE(neighbour.x - previous.x, neighbour.y - previous.y);
        path[i + 1].distance = distance;
        previous.x = neighbour.x;
        previous.y = neighbour.y;
    }
    path[0].distance = 0;
}

double PurePursuitController::FindMaxVelocity(int index, double k) {
    Vec2D point = path[index];
    Vec2D backpoint = path[index - 1];
    Vec2D frontpoint = path[index + 1];
    if (index > 0) {
        double curvature = Curvature(backpoint, point, frontpoint);
        return std::min(l_maxSpeed, k / curvature);
    }
    return l_maxSpeed;
}

void PurePursuitController::CalculateTargetVelocities() {
    // First calculate the max target velocities for each point
    for (int i = 1;  i < path.size() - 1; i++)
        path[i].speed = FindMaxVelocity(i, 1);

    path[path.size() - 1].speed = 0;
    path[0].speed = l_maxSpeed;
    for (int i = path.size() - 2; i >= 0; i--) {
        double dist = path[i + 1].distance - path[i].distance;
        double v_i = path[i + 1].speed;
        double v_f = sqrt(v_i * v_i + 2 * l_maxAcceleration * dist);
        path[i].speed = std::min(path[i].speed, v_f);
    }
}


Vec2D PurePursuitController::ClosestPoint(Vec2D pos) {
    int last = lastClosestPointIdx;
    double minDistance = INFINITY;
    Vec2D closestPoint;
    for (int i = last; i < path.size(); i++) {
        Vec2D point = path[i];
        double dist = DISTANCE(pos.x - point.x, pos.y - point.y);
        if (dist < minDistance) {
            lastClosestPointIdx = i;
            minDistance = dist;
            closestPoint = point;
        }
    }
    return closestPoint;
}


Vec2D PurePursuitController::LookAheadPoint(Vec2D start, Vec2D end, Vec2D pos) {
    Vec2D d{end.x - start.x, end.y - start.y};
    Vec2D f{start.x - pos.x, start.y - pos.y};

    double a = d.x*d.x + d.y*d.y;
    double b = 2 * (f.x*d.x + f.y*d.y);
    double c = (f.x*f.x + f.y*f.y) - l_lookAheadDistance*l_lookAheadDistance;
    double discriminant = b*b - 4*a*c;

    if (discriminant < 0) {
        // no intersection
        return Vec2D{NAN, 0};
    } else {
        discriminant = sqrt(discriminant);
        double t1 = (-b - discriminant) / (2*a);
        double t2 = (-b + discriminant) / (2*a);
        double t_f = NAN;

        if (t2 >= 0 && t2 <= 1) {
            t_f = t2;
        }
        if (t1 >= 0 && t1 <= 1) {
            t_f = t1;
        }
        if (!std::isnan(t_f)) {
            Vec2D point{start.x + t_f * d.x, start.y + t_f * d.y};
            return point;
        }
        return Vec2D{NAN, 0};
    }
}


double PurePursuitController::CurvatureOfArc(Vec2D pos, double heading) {
    Vec2D lookAheadPoint = lastLookAheadPoint;
    for (int i = lastClosestPointIdx; i < path.size() - 1; i++) {
        Vec2D point = LookAheadPoint(path[i], path[i + 1], pos);
        if (std::isnan(point.x)) {
            continue;
        }
        lookAheadPoint = point;
        lastLookAheadPoint = point;
    }

    double a = -tan(heading * DEG_TO_RAD);
    double b = 1;
    double c = tan(heading * DEG_TO_RAD) * pos.x - pos.y;

    double x = std::abs(a * lookAheadPoint.x + b * lookAheadPoint.y + c) / DISTANCE(a, b);
    double L = DISTANCE(lookAheadPoint.x - pos.x, lookAheadPoint.y - pos.y);
    double curvature = (2 * x) / (L*L);

    int side = signum(
        sin(heading * DEG_TO_RAD) * (lookAheadPoint.x - pos.x) - cos(heading * DEG_TO_RAD) * (lookAheadPoint.y - pos.y)
        );

    curvature = side * curvature;

    return curvature;
}


std::vector<Vec2D> PurePursuitController::GetPath() {
    return path;
}

void PurePursuitController::ClearPath() {
    path.clear();
    lastClosestPointIdx = 0;
}

void PurePursuitController::Translate(double x, double y) {
    offset.x = x;
    offset.y = y;
}

void PurePursuitController::InjectWaypoint(double x, double y) {
    Vec2D p;
    p.x = x + offset.x;
    p.y = y + offset.y;
    path.push_back(p);
    CalculateDistances();
}

void PurePursuitController::SetOrigin(double x, double y) {
    Vec2D origin;
    origin.x = x + offset.x;
    origin.y = y + offset.y;
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
    // Calculate speeds
    CalculateTargetVelocities();
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
    // Calculate speeds
    CalculateTargetVelocities();
}


