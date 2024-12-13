#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>

using Eigen::Vector3f;

struct Boid
{
    Vector3f position;
    Vector3f velocity;
};

class Flock
{
private:
    std::vector<Boid> boids;
    Vector3f leaderPosition;
    double alignmentWeight = 0.0;
    double cohesionWeight = 0.0;
    double separationWeight = 0.0;
    double leaderAttractionWeight = 0.0;
    double maxSpeed = 0.0;
    double perceptionRadius = 0.0;
    double separationRadius = 0.0;

    // Alignment: Match the velocity of nearby boids
    Vector3f computeAlignment(const Boid &boid);

    // Cohesion: Steer towards the average position of nearby boids
    Vector3f computeCohesion(const Boid &boid);

    // Separation: Avoid crowding nearby boids
    Vector3f computeSeparation(const Boid &boid);

    Vector3f computeLeaderAttraction(const Boid &boid);

    // Limit the speed of a vector to the maximum speed
    Vector3f limitSpeed(const Vector3f &velocity);

public:
    // Flock(int numBoids, double worldSize);
    Flock(int numBoids, double worldSize, const Vector3f &initialLeaderPosition);
    void updateBoids(double dt);

    const std::vector<Boid> &getBoids() const;

    void setLeaderPosition(const Vector3f &position);
    const Vector3f &getLeaderPosition() const;

    void printSettings();

    void setAlignmentWeight(const double value);
    void setCohesionWeight(const double value);
    void setSeparationWeight(const double value);
    void setLeaderAttractionWeight(const double value);
    void setMaxSpeed(const double value);
    void setPerceptionRadius(const double value);
    void setSeparationRadius(const double value);
};
