//
// Created by gerard on 11/11/2019.
//

#ifndef ROSPLAN_WAYPOINT_SAMPLING_RPFIXEDWAYPOINTGENERATOR_H
#define ROSPLAN_WAYPOINT_SAMPLING_RPFIXEDWAYPOINTGENERATOR_H

#include <ros/ros.h>
#include <queue>
#include <rosplan_interface_mapping/AddWaypoint.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <rosplan_knowledge_msgs/SetInt.h>

namespace KCL_rosplan {
    struct Object {
        double x, y, radius, std_dev;
    };

    class RPFixedWaypointGenerator {
    private:
        ros::NodeHandle nh_;
        std::vector<Object> _objects;
        std::vector<std::pair<int, geometry_msgs::PoseStamped>> _generated_wps;
        ros::Publisher waypoints_pub_; // for visualisation purposes
        rviz_visual_tools::RvizVisualToolsPtr visual_tools_;


        ros::ServiceClient _add_wp_prm;
        int _MAX_ATT;
        std::string wp_reference_frame_, wp_namespace_input_,wp_namespace_output_;
        std::map<int,int> wp_id_to_index_map_;
        std::vector<geometry_msgs::PoseStamped> waypoints_;
        ros::ServiceServer filter_waypoint_service_server_;
        bool generateWPSCb(rosplan_knowledge_msgs::SetInt::Request &req, rosplan_knowledge_msgs::SetInt::Response &res);

        void generate_wps(int starting_id);
        bool loadParams();

        ros::ServiceClient update_kb_client_, update_kb_client_array_;

        nav_msgs::OccupancyGrid _static_map;

        std::map<int,std::map<int, float>> adj_matrix;
        struct dijkstra_comp { /// used by dijkstra method
        public:
            static std::vector<float> distance;
            bool operator() (const int& a, const int& b) {
                // in a priority queue the highest element is removed first
                return distance[a] > distance[b];
            }
        };

        double dijkstra(int a, int b); // Distance between waypoint a and waypoint b
        static int extract_id(const std::string& id);
        void initializeDistancesKB();
        void pubWPGraph();

        bool isVisible(const std::pair<double, double>& a, const std::pair<double, double>& b);
        int sign(double n);
    public:
        RPFixedWaypointGenerator();
        ~RPFixedWaypointGenerator() = default;

    };
}

#endif //ROSPLAN_WAYPOINT_SAMPLING_RPFIXEDWAYPOINTGENERATOR_H
