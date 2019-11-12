//
// Created by gerard on 11/11/2019.
//

#include "rosplan_interface_mapping/RPFixedWaypointGenerator.h"
namespace KCL_rosplan {

    RPFixedWaypointGenerator::RPFixedWaypointGenerator() : nh_("~") {
        nh_.param<std::string>("wp_reference_frame", wp_reference_frame_, "map");
        nh_.param<int>("max_connect_attempts", _MAX_ATT, 20);
        nh_.param<std::string>("wp_namespace_input", wp_namespace_input_, "/rosplan_demo_waypoints");
        nh_.param<std::string>("wp_namespace_output", wp_namespace_output_, "/task_planning_waypoints");
        std::string get_map_srv_name;
        nh_.param<std::string>("get_map_srv_name", get_map_srv_name, "/static_map");


        std::string rosplan_kb_name;
        nh_.param<std::string>("rosplan_kb_name", rosplan_kb_name, "rosplan_knowledge_base");

        std::stringstream ss;
        ss << "/" << rosplan_kb_name << "/update";
        update_kb_client_ = nh_.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>(ss.str());
        ss.str("");
        ss << "/" << rosplan_kb_name << "/update_array";
        update_kb_client_array_ = nh_.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(ss.str());

        waypoints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_sampler/viz/waypoints", 10, true);
        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map","/waypoint_sampler/viz/labels"));

        // Load objects
        XmlRpc::XmlRpcValue objects;

        if (nh_.getParam("bananas", objects)) {
            ROS_ASSERT(objects.getType() == XmlRpc::XmlRpcValue::TypeArray);
            for (auto i = 0; i != objects.size(); ++i) {
                if (objects[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                    Object coord;
                    for (auto it = objects[i].begin(); it != objects[i].end(); it++) {
                        if (it->first == "x") coord.x = it->second;
                        else if (it->first == "y") coord.y = it->second;
                        else if (it->first == "std_dev") coord.std_dev = it->second;
                        else if (it->first == "radius") coord.radius = it->second;
                    }
                    _objects.push_back(coord);
                }
            }
        }
        if (nh_.getParam("doughnuts", objects)) {
            ROS_ASSERT(objects.getType() == XmlRpc::XmlRpcValue::TypeArray);
            for (auto i = 0; i != objects.size(); ++i) {
                if (objects[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                    Object coord;
                    for (auto it = objects[i].begin(); it != objects[i].end(); it++) {
                        if (it->first == "x") coord.x = it->second;
                        else if (it->first == "y") coord.y = it->second;
                        else if (it->first == "std_dev") coord.std_dev = it->second;
                        else if (it->first == "radius") coord.radius = it->second;
                    }
                    _objects.push_back(coord);
                }
            }
        }
        srand (time(NULL));

        _add_wp_prm = nh_.serviceClient<rosplan_interface_mapping::AddWaypoint>("/rosplan_roadmap_server/add_waypoint");

        filter_waypoint_service_server_ = nh_.advertiseService("sample_waypoints",&RPFixedWaypointGenerator::generateWPSCb, this);

        //Get Map
        nav_msgs::GetMap mapSrv;

        // check for service existence
        ros::ServiceClient get_map_client = nh_.serviceClient<nav_msgs::GetMap>(get_map_srv_name);
        if(!get_map_client.waitForExistence(ros::Duration(100))) {
            ROS_ERROR("KCL: (%s) Costmap service not found (%s)", ros::this_node::getName().c_str(), get_map_client.getService().c_str());
            ros::shutdown();
        }

        get_map_client.call(mapSrv);
        _static_map = mapSrv.response.map;

        ROS_INFO("(KCL) fixed waypoint generator: Ready to receive");
    }

    bool RPFixedWaypointGenerator::loadParams() {

        if(!nh_.hasParam(wp_namespace_input_)) {
            ROS_INFO("KCL: (%s) No waypoints found in param server under namespace: %s", ros::this_node::getName().c_str(), wp_namespace_input_.c_str());
            return false;
        }

        // get all waypoints under a namespace
        XmlRpc::XmlRpcValue waypoints;
        std::string wp_reference_frame;
        wp_id_to_index_map_.clear();
        if(nh_.getParam(wp_namespace_input_+"/wp", waypoints)){
            if(waypoints.getType() == XmlRpc::XmlRpcValue::TypeStruct){
                for(auto wit=waypoints.begin(); wit!=waypoints.end(); wit++) {
                    if(wit->second.getType() == XmlRpc::XmlRpcValue::TypeString) {
                        ROS_INFO("KCL: (%s) parsing string parameter (wp reference frame)", ros::this_node::getName().c_str());
                        if(wit->first.c_str() == std::string("waypoint_frameid")) {
                            std::string wp_reference_frame = static_cast<std::string>(wit->second);
                            ROS_INFO("KCL: (%s) Setting new waypoint reference frame : %s", ros::this_node::getName().c_str(), wp_reference_frame.c_str());
                        }
                        else {
                            ROS_ERROR("KCL: (%s) Error: Expected waypoint_frameid, received instead: %s", ros::this_node::getName().c_str(), wit->first.c_str());
                            return false;
                        }
                    }
                    else if(wit->second.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                        // iterate over waypoint coordinates (x, y , theta)
                        std::vector<double> waypoint;
                        std::string wp_id = "";
                        for (size_t i = 0; i < wit->second.size(); i++) {
                            double coordinate = -1.0;
                            if(wit->second[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                                ROS_DEBUG("Warning: coordinate stored in param server as integer, consider adding .0 to your values");
                                coordinate = static_cast<double>(static_cast<int>(wit->second[i]));
                            }
                            else if(wit->second[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                                ROS_DEBUG("coordinates properly stored as double, all ok");
                                coordinate = static_cast<double>(wit->second[i]);
                            }
                            if(i == 0) {
                                ROS_DEBUG("%s x coordinate : %lf", wit->first.c_str(), coordinate);
                                waypoint.push_back(coordinate);
                                wp_id = wit->first.c_str();
                            }
                            else if(i == 1) {
                                ROS_DEBUG("%s y coordinate : %lf", wit->first.c_str(), coordinate);
                                waypoint.push_back(coordinate);
                            }
                            else if(i == 2) {
                                ROS_DEBUG("%s theta angle : %lf", wit->first.c_str(), coordinate);
                                waypoint.push_back(coordinate);
                            }
                            else {
                                ROS_ERROR("KCL: (%s) Error: waypoint must only consist of 3 coordinates: x, y and theta", ros::this_node::getName().c_str());
                                return false;
                            }
                        }

                        // remember index of this ID
                        int paramid = extract_id(wp_id);
                        wp_id_to_index_map_[paramid] = waypoints_.size();

                        // add symbolic wp to KB and store in memory
                        // convert wp to PoseStamped
                        ROS_ASSERT(waypoint.size() == 3);
                        geometry_msgs::PoseStamped wp_as_pose;
                        wp_as_pose.header.frame_id = wp_reference_frame;
                        wp_as_pose.pose.position.x = waypoint.at(0);
                        wp_as_pose.pose.position.y = waypoint.at(1);
                        // convert yaw to quaternion
                        tf2::Quaternion q;
                        q.setRPY(0.0, 0.0, waypoint.at(2));
                        wp_as_pose.pose.orientation.x = q[0];
                        wp_as_pose.pose.orientation.y = q[1];
                        wp_as_pose.pose.orientation.z = q[2];
                        wp_as_pose.pose.orientation.w = q[3];
                        waypoints_.push_back(wp_as_pose);
                    }
                    else {
                        ROS_WARN("KCL (%s) Unrecognized parameter format, allowed formats are: string, list", ros::this_node::getName().c_str());
                    }
                }
            }
            else {
                ROS_ERROR("KCL: (%s) Waypoints are not in list format, failed to load", ros::this_node::getName().c_str());
                return false;
            }
        }
        else{
            ROS_ERROR("KCL: (%s) Parameters exist but still failed to load them", ros::this_node::getName().c_str());
            return false;
        }

        // Get edges
        int N = waypoints_.size();
        //dist_matrix = std::vector<std::vector<float>>(N, std::vector<float>(N, FLT_MAX));
        adj_matrix.clear();
        XmlRpc::XmlRpcValue edges;
        if(nh_.getParam(wp_namespace_input_+"/edge", edges)){
            for (auto it = edges.begin(); it != edges.end(); ++it) {

                int src = wp_id_to_index_map_[extract_id(it->first)];
                std::map<int,float> adjMap;
                adj_matrix[src];

                for (auto wpit=it->second.begin(); wpit != it->second.end(); ++wpit) {
                    int i = wp_id_to_index_map_[extract_id(wpit->first)];
                    adjMap[i] = static_cast<double>(wpit->second);
                    adj_matrix[src] = adjMap;
                }
            }
        }

        return true;
    }

    void RPFixedWaypointGenerator::generate_wps(int starting_id) {
        for (auto it = _objects.begin(); it < _objects.end(); ++it) {
            bool connected = false;
            int i = 0;
            int wp_id = -1;
            std::pair<double, double> coord;
            geometry_msgs::PoseStamped pose;
            while (not connected and i < _MAX_ATT) {

                double rangle = ((double) rand() / (RAND_MAX)) * 2 * M_PI;
                coord.first = it->x + (it->radius * cos(rangle));
                coord.second = it->y + (it->radius * sin(rangle));

                if (not isVisible(coord, std::make_pair(it->x, it->y))) {
                    ++i;
                    ROS_WARN("KCL (%s) Generated a non visible waypoint (wp%d) !", ros::this_node::getName().c_str(), starting_id);
                    continue;
                }

                rosplan_interface_mapping::AddWaypoint awp;
                wp_id = starting_id;
                awp.request.id = "wp" + std::to_string( wp_id);
                pose.header.frame_id = wp_reference_frame_;
                pose.pose.position.x = coord.first;
                pose.pose.position.y = coord.second;
                awp.request.waypoint = pose;
                awp.request.connecting_distance = 2.0;

                connected = _add_wp_prm.call(awp);
                ++i;
            }
            if (connected) {
                ++starting_id;
                _generated_wps.push_back(std::make_pair(wp_id, pose));
                waypoints_.push_back(pose);
            }
        }
    }

    std::vector<float> RPFixedWaypointGenerator::dijkstra_comp::distance = std::vector<float>();
    double RPFixedWaypointGenerator::dijkstra(int a, int b) { //a source, b target

        int N = waypoints_.size();
        RPFixedWaypointGenerator::dijkstra_comp::distance = std::vector<float>(N, FLT_MAX);
        RPFixedWaypointGenerator::dijkstra_comp::distance[a] = 0;

        std::priority_queue<int, std::vector<int>, RPFixedWaypointGenerator::dijkstra_comp> Q;
        Q.push(a);

        while (not Q.empty()) {
            int u = Q.top();
            if (u == b) return RPFixedWaypointGenerator::dijkstra_comp::distance[u];
            Q.pop();
            for (auto it = adj_matrix[u].begin() ; it != adj_matrix[u].end() ; ++it) { //forall neighbours
                double nd = RPFixedWaypointGenerator::dijkstra_comp::distance[u] + it->second;
                if (nd < RPFixedWaypointGenerator::dijkstra_comp::distance[it->first]) {
                    RPFixedWaypointGenerator::dijkstra_comp::distance[it->first] = nd;
                    Q.push(it->first);
                }
            }
        }
        return -1;
    }

    int RPFixedWaypointGenerator::extract_id(const std::string &id) { // assumed id of the form wpXXX
        return std::stoi(id.substr(2));
    }

    void RPFixedWaypointGenerator::initializeDistancesKB() {

        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updatePredSrv;
        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updateFuncSrv;
        bool set_connected = false;

        //for (auto ait = sampled_waypoint_ids_.begin(); ait != sampled_waypoint_ids_.end(); ait++) {
        //for (auto bit = sampled_waypoint_ids_.begin(); bit != sampled_waypoint_ids_.end(); bit++) {
        for (int a=0; a < _generated_wps.size(); a++) {
            for (int b=a+1; b < _generated_wps.size(); b++) {

                double dist = 0;
                if (_generated_wps[a].first != _generated_wps[b].first)
                    dist = dijkstra(_generated_wps[a].first, _generated_wps[b].first);

                // Upload distance
                rosplan_knowledge_msgs::KnowledgeItem item;

                item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
                item.attribute_name = "distance";
                diagnostic_msgs::KeyValue pairFrom;
                pairFrom.key = "a";
                pairFrom.value = "wp"+std::to_string(_generated_wps[a].first);
                item.values.push_back(pairFrom);
                diagnostic_msgs::KeyValue pairTo;
                pairTo.key = "b";
                pairTo.value = "wp"+std::to_string(_generated_wps[b].first);
                item.values.push_back(pairTo);
                item.function_value = dist;

                updateFuncSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
                updateFuncSrv.request.knowledge.push_back(item);

                // upload distance in other direction
                item.values.clear();
                pairFrom.value = "wp"+std::to_string(_generated_wps[b].first);
                item.values.push_back(pairFrom);
                pairTo.value = "wp"+std::to_string(_generated_wps[a].first);
                item.values.push_back(pairTo);

                updateFuncSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
                updateFuncSrv.request.knowledge.push_back(item);

                // Update connected
                if (set_connected) {
                    rosplan_knowledge_msgs::KnowledgeItem item;

                    item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
                    item.attribute_name = "connected";
                    diagnostic_msgs::KeyValue pairFrom;
                    pairFrom.key = "from";
                    pairFrom.value = "wp"+std::to_string(a);
                    item.values.push_back(pairFrom);
                    diagnostic_msgs::KeyValue pairTo;
                    pairTo.key = "to";
                    pairTo.value = "wp"+std::to_string(b);
                    item.values.push_back(pairTo);

                    updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
                    updatePredSrv.request.knowledge.push_back(item);
                }
            }
        }

        // service calls
        if(!update_kb_client_array_.call(updatePredSrv))
            ROS_INFO("KCL: (%s) Failed to call update service.", ros::this_node::getName().c_str());
        if(!update_kb_client_array_.call(updateFuncSrv))
            ROS_INFO("KCL: (%s) Failed to call update service.", ros::this_node::getName().c_str());
    }


    void RPFixedWaypointGenerator::pubWPGraph() {

        visualization_msgs::MarkerArray marker_array;

        // publish nodes as marker array
        std::vector<std::pair<int, geometry_msgs::PoseStamped>>::iterator sit = _generated_wps.begin();
        for (; sit!=_generated_wps.end(); sit++) {

            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation().x() += sit->second.pose.position.x;
            pose.translation().y() += sit->second.pose.position.y;
            pose.translation().y() += 0.5f;
            geometry_msgs::Vector3 text_scale;
            text_scale.x = 0.5;
            text_scale.y = 0.5;
            text_scale.z = 0.5;
            std::stringstream ss;
            ss << "wp" << sit->first;
            visual_tools_->publishText(pose, ss.str(), rviz_visual_tools::WHITE, text_scale, false);//rviz_visual_tools::XXXLARGE, false);

            visualization_msgs::Marker node_marker;
            node_marker.header.stamp = ros::Time();
            node_marker.header.frame_id = wp_reference_frame_;
            node_marker.ns = wp_namespace_output_;
            node_marker.type = visualization_msgs::Marker::CUBE;
            node_marker.action = visualization_msgs::Marker::MODIFY;
            node_marker.pose = sit->second.pose;
            node_marker.scale.x = 0.35f;
            node_marker.scale.y = 0.35f;
            node_marker.scale.z = 0.35f;
            node_marker.color.a = 1.0f;
            node_marker.color.r = 1.0f;
            node_marker.color.g = 0.3f;
            node_marker.color.b = 0.3f;
            node_marker.id = sit->first;

            marker_array.markers.push_back(node_marker);
        }
        waypoints_pub_.publish(marker_array);
        visual_tools_->trigger();
    }

    bool RPFixedWaypointGenerator::generateWPSCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        ROS_INFO("KCL: (%s) Starting waypoint generation.", ros::this_node::getName().c_str());
        XmlRpc::XmlRpcValue waypoints;
        nh_.getParam(wp_namespace_input_+"/wp", waypoints);
        generate_wps(waypoints.size());
        loadParams();
        initializeDistancesKB();
        pubWPGraph();
        waypoints_.clear();
        wp_id_to_index_map_.clear();
        adj_matrix.clear();
        _generated_wps.clear();
        ROS_INFO("KCL: (%s) Completed waypoint generation.", ros::this_node::getName().c_str());
        return true;
    }

    bool RPFixedWaypointGenerator::isVisible(const std::pair<double, double> &a, const std::pair<double, double> &b) {
        double xA = a.first;
        double yA = a.second;
        double dx = b.first - xA;
        double dy = b.second - yA;
        double sx = _static_map.info.resolution * dx/(std::abs(dx)+std::abs(dy));
        double sy = _static_map.info.resolution * dy/(abs(dx)+std::abs(dy));

        while (((sx == 0) or xA*sign(sx) < b.first*sign(sx)) and (sy == 0 or yA*sign(sy) < b.second*sign(sy))) {
            int gx = int((xA - _static_map.info.origin.position.x) / _static_map.info.resolution);
            int gy = int((yA - _static_map.info.origin.position.y) / _static_map.info.resolution);
            if (gx > _static_map.info.width or gx < 0 or gy < 0 or gy >= _static_map.info.height or
                _static_map.data[gx + gy * _static_map.info.width] > 0.12) return false;
            xA += sx;
            yA += sy;
        }
        return true;
    }

    int RPFixedWaypointGenerator::sign(double n) {
        return (n > 0) - (n < 0);
    }

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "fixed_waypoint_gen");
    KCL_rosplan::RPFixedWaypointGenerator fwg;
    ros::spin();
    return 0;
}
