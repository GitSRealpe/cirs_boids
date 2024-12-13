#include <cirs_boids/RRT.hpp>

#include <Eigen/Dense>
#include <vector>
#include <random>
#include <iostream>
#include <cmath>

using Eigen::Vector3f;

RRT::RRT(const Eigen::Vector2d &start, const Eigen::Vector2d &goal, double bias, double maxSegmentLength)
    : start_(start), goal_(goal), bias_(bias), maxSegmentLength_(maxSegmentLength)
{
    nodes_.emplace_back(start, -1); // Root node has no parent
}

// Random point sampling with bias towards the goal
Eigen::Vector2d RRT::samplePoint()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0.0, 1.0);
    static std::uniform_real_distribution<> disX(0.0, 10.0);
    static std::uniform_real_distribution<> disY(0.0, 10.0);

    if (dis(gen) < bias_)
    {
        return goal_;
    }
    else
    {
        return Eigen::Vector2d(disX(gen), disY(gen));
    }
}

// Find the index of the nearest node in the tree
int RRT::findNearestNode(const Eigen::Vector2d &point)
{
    int nearestIndex = -1;
    double minDistance = __DBL_MAX__;

    for (int i = 0; i < nodes_.size(); ++i)
    {
        double distance = (nodes_[i].position - point).norm();
        if (distance < minDistance)
        {
            minDistance = distance;
            nearestIndex = i;
        }
    }
    return nearestIndex;
}

// Steer from nearest point towards random point with a maximum segment length
Eigen::Vector2d RRT::steer(const Eigen::Vector2d &from, const Eigen::Vector2d &to)
{
    Eigen::Vector2d direction = to - from;
    double length = direction.norm();
    if (length > maxSegmentLength_)
    {
        direction = direction.normalized() * maxSegmentLength_;
    }
    return from + direction;
}

// Check if a point is valid (e.g., no collisions)
bool RRT::isValid(const Eigen::Vector2d &point)
{
    // Placeholder for collision detection logic
    // For now, assume all points are valid
    return true;
}

// Run the RRT algorithm
bool RRT::run(int maxIterations)
{
    for (int i = 0; i < maxIterations; ++i)
    {
        Eigen::Vector2d randomPoint = samplePoint();
        int nearestNodeIndex = findNearestNode(randomPoint);
        Eigen::Vector2d newPoint = steer(nodes_[nearestNodeIndex].position, randomPoint);

        if (isValid(newPoint))
        {
            nodes_.emplace_back(newPoint, nearestNodeIndex);

            if ((newPoint - goal_).norm() <= maxSegmentLength_)
            {
                // Connect to the goal
                nodes_.emplace_back(goal_, nodes_.size() - 1);
                return true; // Goal reached
            }
        }
    }
    return false; // Goal not reached within maxIterations
}

// Retrieve the path from start to goal
std::vector<Eigen::Vector2d> RRT::getPath() const
{
    std::vector<Eigen::Vector2d> path;
    int currentIndex = nodes_.size() - 1;
    while (currentIndex != -1)
    {
        path.push_back(nodes_[currentIndex].position);
        currentIndex = nodes_[currentIndex].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}
