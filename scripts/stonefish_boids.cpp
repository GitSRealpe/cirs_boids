#include <ros/ros.h>
#include <ros/file_log.h>
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/utils/SystemUtil.hpp>
#include <stonefish_ros/ROSSimulationManager.h>
#include <stonefish_ros/ROSScenarioParser.h>

#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/entities/solids/Sphere.h>
#include <Stonefish/entities/solids/Box.h>
#include <Stonefish/entities/AnimatedEntity.h>
#include <Stonefish/entities/animation/ManualTrajectory.h>

#include <cirs_boids/Flock.hpp>

class MyManager : public sf::ROSSimulationManager
{
private:
    ros::ServiceClient client;
    ros::ServiceServer service;

    const int numBoids = 20;
    const double worldSize = 2.0;
    const double dt = 0.1;
    std::unique_ptr<Flock> flock;

    sf::AnimatedEntity *anim;
    sf::ManualTrajectory *leader_traj;
    std::vector<sf::ManualTrajectory *> boid_paths;

    ros::Timer timer;
    void timerCb(const ros::TimerEvent &evt);
    float inc = 0;

public:
    MyManager(sf::Scalar rate, std::string scenarioPath);
    ~MyManager();

    void MakeFishes();
    void BuildScenario();
};

MyManager::MyManager(sf::Scalar rate, std::string scenarioPath) : ROSSimulationManager(rate, scenarioPath)
{
    std::cout << "inherited rosmanager" << "\n";
    std::cout << rate << "\n";
    std::cout << scenarioPath << "\n";

    leader_traj = new sf::ManualTrajectory();
    timer = nh.createTimer(ros::Duration(1.0 / 30.0), &MyManager::timerCb, this);
}

MyManager::~MyManager()
{
}

void MyManager::timerCb(const ros::TimerEvent &evt)
{

    // flock->setLeaderPosition(Vector3f(5 * sin(inc), 5 * cos(inc), sin(inc) + 2));
    flock->setLeaderPosition(Vector3f(0, 0, 2));
    flock->updateBoids(1.0 / 30.0);
    auto leader_pos = flock->getLeaderPosition();
    sf::Vector3 point(leader_pos.x(), leader_pos.y(), leader_pos.z());
    sf::Quaternion ori(0, 0, 0, 1);
    leader_traj->setTransform(sf::Transform(ori, point));

    for (int i = 0; i < flock->getBoids().size(); i++)
    {
        auto boid = flock->getBoids().at(i);
        sf::Vector3 point(boid.position.x(), boid.position.y(), boid.position.z());

        // Orientation as quaternion with no roll
        Vector3f forward = boid.velocity.normalized();   // Direction of movement
        Vector3f up = Vector3f::UnitZ();                 // Fixed "up" direction
        Vector3f right = up.cross(forward).normalized(); // Perpendicular right vector
        up = forward.cross(right).normalized();          // Recompute orthogonal up vector
        Eigen::Matrix3f rotationMatrix;
        rotationMatrix.col(0) = forward; // Forward direction
        rotationMatrix.col(1) = right;   // Right direction
        rotationMatrix.col(2) = up;      // Up direction
        Eigen::Quaternionf orientation(rotationMatrix);
        sf::Quaternion ori(orientation.x(), orientation.y(),
                           orientation.z(), orientation.w());

        boid_paths.at(i)->setTransform(sf::Transform(ori, point));
    }

    inc++;
}

void MyManager::MakeFishes()
{

    flock = std::make_unique<Flock>(numBoids, worldSize, Vector3f(0, 0, 0));

    XmlRpc::XmlRpcValue boids_cfg;
    bool gotten = nh.getParam("/boids_settings", boids_cfg);
    std::cout << gotten << "\n";
    int b = 0;
    for (; b < boids_cfg.size(); b++)
    {
        std::cout << "found: " << boids_cfg[b]["name"] << "\n";
        std::cout << boids_cfg[b]["name"] << "\n";
        std::cout << static_cast<double>(boids_cfg[b]["alignmentWeight"]) << "\n";
        flock->setAlignmentWeight(static_cast<double>(boids_cfg[b]["alignmentWeight"]));
        flock->setCohesionWeight(static_cast<double>(boids_cfg[b]["cohesionWeight"]));
        flock->setSeparationWeight(static_cast<double>(boids_cfg[b]["separationWeight"]));
        flock->setLeaderAttractionWeight(static_cast<double>(boids_cfg[b]["leaderAttractionWeight"]));
        flock->setMaxSpeed(static_cast<double>(boids_cfg[b]["maxSpeed"]));
        flock->setPerceptionRadius(static_cast<double>(boids_cfg[b]["perceptionRadius"]));
        flock->setSeparationRadius(static_cast<double>(boids_cfg[b]["separationRadius"]));

        flock->printSettings();
        break;
    }

    CreateLook("fishtex", sf::Color::Gray(1.f), 0.1f, 0.f, 0.f,
               sf::GetDataPath() + "creatures/yellowtang/yellowtang.png");
    // create control ptr for every boid
    for (int i = 0; i < numBoids; i++)
    {
        sf::ManualTrajectory *traj = new sf::ManualTrajectory();
        boid_paths.push_back(traj);

        anim = new sf::AnimatedEntity("AnimMesh" + std::to_string(i), boid_paths.at(i),
                                      sf::GetDataPath() + "creatures/yellowtang/yellowtang.obj",
                                      1.0, sf::I4(),
                                      sf::GetDataPath() + "creatures/yellowtang/yellowtang.obj",
                                      1.0, sf::I4(), "Neutral", "fishtex");
        AddAnimatedEntity(anim);
    }

    std::cout << boid_paths.size() << "\n";
    anim = new sf::AnimatedEntity("LeaderFrame", leader_traj);
    AddAnimatedEntity(anim);
}

void MyManager::BuildScenario()
{
    // Run parser
    sf::ROSScenarioParser parser(this);
    bool success = parser.Parse(scnFilePath);

    // Save log
    std::string logFilePath = ros::file_log::getLogDirectory() + "/stonefish_ros_parser.log";
    bool success2 = parser.SaveLog(logFilePath);

    if (!success)
    {
        ROS_ERROR("Parsing of scenario file '%s' failed!", scnFilePath.c_str());
        if (success2)
            ROS_ERROR("For more information check the parser log file '%s'.", logFilePath.c_str());
    }

    if (!success2)
        ROS_ERROR("Parser log file '%s' could not be saved!", logFilePath.c_str());

    // Create object
    CreateLook("red", sf::Color::RGB(1.f, 0.f, 0.f), 0.1f, 0.f);
    sf::Scalar angle = M_PI / 180.0 * 14.1;
    sf::BodyPhysicsSettings phy;
    phy.mode = sf::BodyPhysicsMode::DISABLED;
    phy.collisions = true;
    sf::Box *box = new sf::Box("Box", phy, sf::Vector3(0.1, 0.1, 0.1), sf::I4(), "Steel", "red");
    AddSolidEntity(box, sf::Transform(sf::Quaternion(0, angle, 0), sf::Vector3(2.5, 0, -1.72)));

    MakeFishes();

    spinner.start();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parsed_simulator", ros::init_options::NoSigintHandler);

    // Check number of command line arguments
    if (argc < 7)
    {
        ROS_FATAL("Not enough command line arguments provided!");
        return 1;
    }

    // Parse arguments
    std::string dataDirPath = std::string(argv[1]) + "/";
    std::string scenarioPath(argv[2]);

    std::cout << "resource path: " << dataDirPath << "\n";
    std::cout << "scenario path: " << scenarioPath << "\n";

    sf::Scalar rate = atof(argv[3]);

    sf::RenderSettings s;
    s.windowW = atoi(argv[4]);
    s.windowH = atoi(argv[5]);

    std::string quality(argv[6]);
    if (quality == "low")
    {
        s.shadows = sf::RenderQuality::LOW;
        s.ao = sf::RenderQuality::DISABLED;
        s.atmosphere = sf::RenderQuality::LOW;
        s.ocean = sf::RenderQuality::LOW;
        s.aa = sf::RenderQuality::LOW;
        s.ssr = sf::RenderQuality::DISABLED;
    }
    else if (quality == "high")
    {
        s.shadows = sf::RenderQuality::HIGH;
        s.ao = sf::RenderQuality::HIGH;
        s.atmosphere = sf::RenderQuality::HIGH;
        s.ocean = sf::RenderQuality::HIGH;
        s.aa = sf::RenderQuality::HIGH;
        s.ssr = sf::RenderQuality::HIGH;
    }
    else // "medium"
    {
        s.shadows = sf::RenderQuality::MEDIUM;
        s.ao = sf::RenderQuality::MEDIUM;
        s.atmosphere = sf::RenderQuality::MEDIUM;
        s.ocean = sf::RenderQuality::MEDIUM;
        s.aa = sf::RenderQuality::MEDIUM;
        s.ssr = sf::RenderQuality::MEDIUM;
    }

    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    h.showSensors = false;
    h.showActuators = false;
    h.showForces = false;

    // sf::ROSSimulationManager manager(rate, scenarioPath);

    MyManager *manager = new MyManager(rate, scenarioPath);
    sf::GraphicalSimulationApp app("Stonefish Simulator", dataDirPath, s, h, manager);

    app.Run();

    return 0;
}
