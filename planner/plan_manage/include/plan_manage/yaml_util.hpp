#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>


namespace YAML {
    struct Waypoint 
    {
        double x;
        double y;
        double z;
        double yaw;
        double stay;
    };

    template<>
    struct convert<Waypoint> {
        static Node encode(const Waypoint& rhs) {
            Node node;
            node["x"] = rhs.x;
            node["y"] = rhs.y;
            node["z"] = rhs.z;
            node["yaw"] = rhs.yaw;
            node["stay"] = rhs.stay;
            return node;
            }

        static bool decode(const Node& node, Waypoint& rhs) {
            if(!node.IsSequence() || node.size() != 4) {
                return false;
            }

            rhs.x = node[0].as<double>();
            rhs.y = node[1].as<double>();
            rhs.z = node[2].as<double>();
            rhs.yaw = node[3].as<double>();
            rhs.stay = node[4].as<double>();
            return true;
        }
    };
}

class WaypointRecorder
{
public:
    // void readWaypointsFromFile()
    // {
    //     YAML::Node node = YAML::LoadFile("/home/henry/waypoints.yaml");
    //     int i = 0;
    //     std::cout << node[0]["x"] << std::endl;
    //     std::cout << node[0]["y"] << std::endl;
    // }

    void recordWaypoint(const Eigen::Vector4d & wp, bool if_stay)
    {
        node.push_back(YAML::Waypoint{wp(0), wp(1), wp(2), wp(3), if_stay});
    }

    void writeWaypointsToFile(const std::string file_path)
    {
        std::ofstream fout(file_path);
        fout << node;
    }

private:
    YAML::Node node;    
};
