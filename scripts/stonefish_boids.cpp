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
#include <Stonefish/entities/animation/BSTrajectory.h>

#include <cirs_boids/Flock.hpp>

class MyManager : public sf::ROSSimulationManager
{
private:
    ros::ServiceClient client;
    ros::ServiceServer service;

    sf::AnimatedEntity *anim;

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
}

MyManager::~MyManager()
{
}

void MyManager::MakeFishes()
{

    const int numBoids = 20;
    const double worldSize = 2.0;
    const double dt = 0.1;
    Flock flock(numBoids, worldSize, Vector3f(0, 0, 0));

    XmlRpc::XmlRpcValue boids_cfg;
    bool gotten = nh.getParam("/boids_settings", boids_cfg);
    std::cout << gotten << "\n";
    int b = 0;
    for (; b < boids_cfg.size(); b++)
    {
        std::cout << "found: " << boids_cfg[b]["name"] << "\n";
        std::cout << boids_cfg[b]["name"] << "\n";
        std::cout << static_cast<double>(boids_cfg[b]["alignmentWeight"]) << "\n";
        flock.setAlignmentWeight(static_cast<double>(boids_cfg[b]["alignmentWeight"]));
        flock.setCohesionWeight(static_cast<double>(boids_cfg[b]["cohesionWeight"]));
        flock.setSeparationWeight(static_cast<double>(boids_cfg[b]["separationWeight"]));
        flock.setLeaderAttractionWeight(static_cast<double>(boids_cfg[b]["leaderAttractionWeight"]));
        flock.setMaxSpeed(static_cast<double>(boids_cfg[b]["maxSpeed"]));
        flock.setPerceptionRadius(static_cast<double>(boids_cfg[b]["perceptionRadius"]));
        flock.setSeparationRadius(static_cast<double>(boids_cfg[b]["separationRadius"]));

        flock.printSettings();
        break;
    }

    std::vector<sf::BSTrajectory *> boid_paths;
    for (int i = 0; i < numBoids; i++)
    {
        sf::BSTrajectory *traj = new sf::BSTrajectory(sf::PlaybackMode::REPEAT);
        boid_paths.push_back(traj);
    }

    sf::BSTrajectory *leader_traj = new sf::BSTrajectory(sf::PlaybackMode::REPEAT);

    double t = 0;
    while (t < 100)
    {
        flock.setLeaderPosition(Vector3f(5 * sin(t), 5 * cos(t), 2));
        auto leader_pos = flock.getLeaderPosition();
        leader_traj->AddKeyPoint(t / 10, sf::Transform(sf::Quaternion(0, 0, 0, 1),
                                                       sf::Vector3(leader_pos.x(),
                                                                   leader_pos.y(),
                                                                   leader_pos.z())));
        flock.updateBoids(dt);

        // auto boid = flock.getBoids().at(0);
        // for (const auto &boid : flock.getBoids())
        for (int i = 0; i < flock.getBoids().size(); i++)
        {
            auto boid = flock.getBoids().at(i);
            sf::Vector3 point(boid.position.x(), boid.position.y(), boid.position.z());

            // // Orientation as quaternion
            // Vector3f direction = boid.velocity.normalized();
            // Eigen::Quaternionf orientation;
            // orientation.setFromTwoVectors(Vector3f::UnitX(), direction);

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

            // boid_paths.at(i).push_back(sf::Transform(ori, point));
            boid_paths.at(i)->AddKeyPoint(t / 10, sf::Transform(ori, point));
        }

        t += 1;
    }

    std::cout << boid_paths.size() << "\n";

    CreateLook("fishtex", sf::Color::Gray(1.f), 0.1f, 0.f, 0.f,
               sf::GetDataPath() + "creatures/yellowtang/yellowtang.png");

    for (int i = 0; i < numBoids; i++)
    {
        anim = new sf::AnimatedEntity("AnimMesh" + std::to_string(i), boid_paths.at(i),
                                      sf::GetDataPath() + "creatures/yellowtang/yellowtang.obj",
                                      1.0, sf::I4(),
                                      sf::GetDataPath() + "creatures/yellowtang/yellowtang.obj",
                                      1.0, sf::I4(), "Neutral", "fishtex");
        AddAnimatedEntity(anim);
    }

    anim = new sf::AnimatedEntity("LeaderFrame", leader_traj);
    AddAnimatedEntity(anim);

    // sf::AnimatedEntity *anim = new sf::AnimatedEntity("Frame", traj);

    // std::cout << getEntity("AnimMesh")->getName() << "\n";
    //  dynamic_cast<sf::AnimatedEntity *>(getEntity("AnimMesh"));
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
