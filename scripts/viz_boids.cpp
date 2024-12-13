#include <ros/ros.h>

#include <cirs_boids/Flock.hpp>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "boids_viz");
    ros::NodeHandle nh;

    ros::Publisher markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>("boids_markers", 10);
    ros::Publisher leaderMarkerPub = nh.advertise<visualization_msgs::Marker>("leader_marker", 10);
    ros::Rate rate(30);

    const int numBoids = 20;
    const double worldSize = 20.0;
    const double dt = 0.1f;

    Flock flock(numBoids, worldSize, Vector3f(0, 0, 0));

    XmlRpc::XmlRpcValue boids_cfg;
    nh.getParam("boids_settings/", boids_cfg);
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

    double t = 0.015;
    while (ros::ok())
    {

        flock.setLeaderPosition(Vector3f(5 * sin(t), 5 * cos(t), 2));
        flock.updateBoids(dt);

        t += 0.015;

        visualization_msgs::MarkerArray markerArray;
        int id = 0;

        for (const auto &boid : flock.getBoids())
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world_ned";
            marker.header.stamp = ros::Time::now();
            marker.ns = "boids";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            // Arrow position and orientation
            marker.pose.position.x = boid.position.x();
            marker.pose.position.y = boid.position.y();
            marker.pose.position.z = boid.position.z();

            // Orientation as quaternion
            Vector3f direction = boid.velocity.normalized();
            Eigen::Quaternionf orientation;
            orientation.setFromTwoVectors(Vector3f::UnitX(), direction);
            marker.pose.orientation.x = orientation.x();
            marker.pose.orientation.y = orientation.y();
            marker.pose.orientation.z = orientation.z();
            marker.pose.orientation.w = orientation.w();

            // Arrow size
            marker.scale.x = 0.5; // Length of the arrow shaft
            marker.scale.y = 0.1; // Width of the arrow shaft
            marker.scale.z = 0.1; // Width of the arrow head

            // Color
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.2;
            marker.color.a = 1.0;

            markerArray.markers.push_back(marker);
        }

        markerArrayPub.publish(markerArray);

        // Publish leader marker
        const auto &leaderPosition = flock.getLeaderPosition();
        visualization_msgs::Marker leaderMarker;
        leaderMarker.header.frame_id = "world_ned";
        leaderMarker.header.stamp = ros::Time::now();
        leaderMarker.ns = "leader";
        leaderMarker.id = 0;
        leaderMarker.type = visualization_msgs::Marker::ARROW;
        leaderMarker.action = visualization_msgs::Marker::ADD;

        // Arrow position and orientation for the leader
        leaderMarker.pose.position.x = leaderPosition.x();
        leaderMarker.pose.position.y = leaderPosition.y();
        leaderMarker.pose.position.z = leaderPosition.z();

        leaderMarker.pose.orientation.w = 1.0; // No specific orientation needed for leader marker

        // Arrow size
        leaderMarker.scale.x = 1.0; // Length of the arrow shaft
        leaderMarker.scale.y = 0.2; // Width of the arrow shaft
        leaderMarker.scale.z = 0.2; // Width of the arrow head

        // Color
        leaderMarker.color.r = 0.0;
        leaderMarker.color.g = 1.0;
        leaderMarker.color.b = 0.0;
        leaderMarker.color.a = 1.0;

        leaderMarkerPub.publish(leaderMarker);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}