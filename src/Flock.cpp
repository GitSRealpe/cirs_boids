#include <cirs_boids/Flock.hpp>

#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>

using Eigen::Vector3f;

// Alignment: Match the velocity of nearby boids
Vector3f Flock::computeAlignment(const Boid &boid)
{
    Vector3f avgVelocity = Vector3f::Zero();
    int count = 0;

    for (const auto &other : boids)
    {
        if ((boid.position - other.position).norm() < perceptionRadius && &boid != &other)
        {
            avgVelocity += other.velocity;
            count++;
        }
    }

    return (count > 0) ? (avgVelocity / count).normalized() : Vector3f::Zero();
}

// Cohesion: Steer towards the average position of nearby boids
Vector3f Flock::computeCohesion(const Boid &boid)
{
    Vector3f avgPosition = Vector3f::Zero();
    int count = 0;

    for (const auto &other : boids)
    {
        if ((boid.position - other.position).norm() < perceptionRadius && &boid != &other)
        {
            avgPosition += other.position;
            count++;
        }
    }

    return (count > 0) ? ((avgPosition / count) - boid.position).normalized() : Vector3f::Zero();
}

// Separation: Avoid crowding nearby boids
Vector3f Flock::computeSeparation(const Boid &boid)
{
    Vector3f repulsion = Vector3f::Zero();

    for (const auto &other : boids)
    {
        double distance = (boid.position - other.position).norm();
        if (distance < separationRadius && &boid != &other)
        {
            repulsion -= (other.position - boid.position) / distance;
        }
    }

    return repulsion.normalized();
}

Vector3f Flock::computeLeaderAttraction(const Boid &boid)
{
    Vector3f toLeader = leaderPosition - boid.position;
    return toLeader.normalized();
}

// Limit the speed of a vector to the maximum speed
Vector3f Flock::limitSpeed(const Vector3f &velocity)
{
    if (velocity.norm() > maxSpeed)
    {
        return velocity.normalized() * maxSpeed;
    }
    return velocity;
}

Flock::Flock(int numBoids, double worldSize, const Vector3f &initialLeaderPosition) : leaderPosition(initialLeaderPosition)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-worldSize, worldSize);

    for (int i = 0; i < numBoids; i++)
    {
        boids.push_back({Vector3f(dis(gen), dis(gen), dis(gen)),
                         Vector3f(dis(gen) * 0.1f, dis(gen) * 0.1f, dis(gen) * 0.1f)});
    }
}

void Flock::updateBoids(double dt)
{
    std::vector<Vector3f> newVelocities;

    for (const auto &boid : boids)
    {
        Vector3f alignment = computeAlignment(boid) * alignmentWeight;
        Vector3f cohesion = computeCohesion(boid) * cohesionWeight;
        Vector3f separation = computeSeparation(boid) * separationWeight;
        Vector3f leaderAttraction = computeLeaderAttraction(boid) * leaderAttractionWeight;

        Vector3f newVelocity = boid.velocity + alignment + cohesion + separation + leaderAttraction;
        newVelocity = limitSpeed(newVelocity);
        newVelocities.push_back(newVelocity);
    }

    for (size_t i = 0; i < boids.size(); i++)
    {
        boids[i].velocity = newVelocities[i];
        boids[i].position += boids[i].velocity * dt;
    }
}

const std::vector<Boid> &Flock::getBoids() const
{
    return boids;
}

void Flock::printSettings()
{
    std::cout << "alignmentWeight: " << alignmentWeight << "\n"
              << "cohesionWeight: " << cohesionWeight << "\n"
              << "separationWeight: " << separationWeight << "\n"
              << "leaderAttractionWeight: " << leaderAttractionWeight << "\n"
              << "maxSpeed: " << maxSpeed << "\n"
              << "perceptionRadius: " << perceptionRadius << "\n"
              << "separationRadius: " << separationRadius << "\n";
}

void Flock::setLeaderPosition(const Vector3f &position)
{
    leaderPosition = position;
}

const Vector3f &Flock::getLeaderPosition() const
{
    return leaderPosition;
}
void Flock::setAlignmentWeight(const double weight)
{
    alignmentWeight = weight;
}

void Flock::setCohesionWeight(const double weight)
{
    cohesionWeight = weight;
}

void Flock::setSeparationWeight(const double weight)
{
    separationWeight = weight;
}

void Flock::setLeaderAttractionWeight(const double weight)
{
    leaderAttractionWeight = weight;
}

void Flock::setMaxSpeed(const double speed)
{
    maxSpeed = speed;
}

void Flock::setPerceptionRadius(const double radius)
{
    perceptionRadius = radius;
}

void Flock::setSeparationRadius(const double radius)
{
    separationRadius = radius;
}
