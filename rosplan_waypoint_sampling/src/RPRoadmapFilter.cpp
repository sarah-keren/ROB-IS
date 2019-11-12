/**
 * 
 * Copyright [2019] <KCL King's College London>
 * 
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk)
 * Maintainer: Michael Cashmore (michael.cashmore@kcl.ac.uk)
 */

#include <queue>
#include "rosplan_interface_mapping/RPRoadmapFilter.h"

namespace KCL_rosplan {

    /*-----------------*/
    /* RPRoadmapFilter */
    /*-----------------*/

    /**
     * Constructor
     */
    RPRoadmapFilter::RPRoadmapFilter() : nh_("~"), costmap_received_(false), srv_timeout_(3.0) {

        std::string rosplan_kb_name;

        // get required parameters from param server
        nh_.param<int>("srv_timeout", srv_timeout_, 3.0);
        nh_.param<int>("waypoint_count", waypoint_count_, 10);
        nh_.param<std::string>("wp_namespace_input", wp_namespace_input_, "/rosplan_demo_waypoints");
        nh_.param<std::string>("wp_namespace_output", wp_namespace_output_, "/task_planning_waypoints");
        nh_.param<std::string>("wp_reference_frame", wp_reference_frame_, "map");
        nh_.param<std::string>("rosplan_kb_name", rosplan_kb_name, "rosplan_knowledge_base");
        nh_.param<std::string>("costmap_topic", costmap_topic_, "costmap_topic");
        nh_.param<double>("minimum_sample_separation", minimum_radius_, 1.0);
        nh_.param<bool>("animate_sampling", animate_sampling_, false);


        // subscriptions of this node, robot odometry and costmap
        map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(costmap_topic_, 1, &RPRoadmapFilter::costMapCallback, this);

        // publications of this node (for visualisation purposes), waypoints and connectivity information (edges)
        waypoints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viz/waypoints", 10, true);
        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map","viz/labels"));

        // publications of the modified probability map
        // probability_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("viz/probability", 10, true);
        probability_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

        // services offered by this node, create random wp (create prm), add single wp, remove a waypoint
        filter_waypoint_service_server_ = nh_.advertiseService("sample_waypoints",&RPRoadmapFilter::filterRoadmap, this);

        // services required by this node, to update rosplan KB and to query map from server
        std::stringstream ss;
        ss << "/" << rosplan_kb_name << "/update";
        update_kb_client_ = nh_.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>(ss.str());
        ss.str("");
        ss << "/" << rosplan_kb_name << "/update_array";
        update_kb_client_array_ = nh_.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(ss.str());

        // Get waypoints


        ROS_INFO("KCL: (%s) Ready to receive.", ros::this_node::getName().c_str());
    }

    // update the costmap with received information coming from callback (topic subscription)
    void RPRoadmapFilter::costMapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
        cost_map_ = *msg;
        costmap_received_ = true;
    }

    /*------------------------------*/
    /* Loading WPs from param server */
    /*------------------------------*/

    bool RPRoadmapFilter::loadParams() {

        if(!nh_.hasParam(wp_namespace_input_)) {
            ROS_INFO("KCL: (%s) No waypoints found in param server under namespace: %s", ros::this_node::getName().c_str(), wp_namespace_input_.c_str());
            return false;
        }

        // ensure there is costmap data before proceeding (every 2 seconds)
        ros::Rate loop_rate(0.5);
        while(!costmap_received_ && ros::ok()) {
            ROS_WARN("KCL: (%s) Costmap not received, ensure that costmap topic has data (%s)", ros::this_node::getName().c_str(), costmap_topic_.c_str());
            ROS_INFO("KCL: (%s) Checking for costmap data at 0.5 hz...", ros::this_node::getName().c_str());
            loop_rate.sleep();
            // listen to callbacks
            ros::spinOnce();
        }

        // reset roadmap before loading
        wp_id_to_index_map_.clear();
        waypoints_.clear();
        sampled_waypoint_ids_.clear();
        unsampled_waypoint_ids_.clear();

        if(nh_.hasParam(wp_namespace_output_)) {
            ROS_INFO("KCL: (%s) Old waypoints found in param server under namespace: %s (deleting now)", ros::this_node::getName().c_str(), wp_namespace_output_.c_str());
            nh_.deleteParam(wp_namespace_output_);
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

    /*---------------*/
    /* Filtering PRM */
    /*---------------*/

    bool RPRoadmapFilter::filterRoadmap(rosplan_knowledge_msgs::SetInt::Request &req, rosplan_knowledge_msgs::SetInt::Response &res) {

        // clear previous roadmap from knowledge base
        ROS_INFO("KCL: (%s) Cleaning old roadmap", ros::this_node::getName().c_str());

        // clear waypoint instances from KB
        rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
        updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
        updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
        updateSrv.request.knowledge.instance_type = "waypoint";

        // wait for service existence
        if(!update_kb_client_.waitForExistence(ros::Duration(srv_timeout_))) {
            ROS_ERROR("KCL: (%s) Update KB service not found (%s)", ros::this_node::getName().c_str(), update_kb_client_.getService().c_str());
            return false;
        }

        if(!update_kb_client_.call(updateSrv)) {
            ROS_ERROR("KCL: (%s) Failed to call update service.", ros::this_node::getName().c_str());
            return false;
        }

        // check if wps are available in param server, if so, load them in symbolic KB and visualise them
        loadParams();
        ROS_INFO("KCL: (%s) Waypoints loaded", ros::this_node::getName().c_str());
        nav_msgs::OccupancyGrid map = cost_map_;

        // prepare ID lists
        waypoint_count_ = req.value;
        sampled_waypoint_ids_.clear();
        unsampled_waypoint_ids_.clear();
        for(int i=0;i<waypoints_.size(); i++) unsampled_waypoint_ids_.push_back(i);

        /* initialize random seed: */
        srand (time(NULL));

        // begin sampling
        std::vector<int> wp_weight;
        while(sampled_waypoint_ids_.size()<waypoint_count_ && unsampled_waypoint_ids_.size()>0) {

            // calculate total current unsampled weight
            int w_sum = 0;
            wp_weight.clear();
            std::vector<int>::iterator uwit = unsampled_waypoint_ids_.begin();
            for(; uwit!=unsampled_waypoint_ids_.end(); uwit++) {
                // get cell
                int cell_x = (int) (waypoints_[*uwit].pose.position.x/map.info.resolution); 
                int cell_y = (int) (waypoints_[*uwit].pose.position.y/map.info.resolution);
                int index =  cell_x + cell_y*map.info.width;
                w_sum += map.data[index];
                wp_weight.push_back(map.data[index]);
            }

            // if all remaining waypoints have 0 probability, sample them randomly
            if(w_sum==0) w_sum = 1;

            // sample one waypoint
            double sample =  rand() % w_sum;
            int counter = 0;
            while(sample > 0 && counter < unsampled_waypoint_ids_.size()) {
                sample = sample - wp_weight[counter];
                if(sample > 0) counter++;
            }
            int sampled_wp_id = unsampled_waypoint_ids_[counter];

            unsampled_waypoint_ids_.erase(unsampled_waypoint_ids_.begin() + counter);
            sampled_waypoint_ids_.push_back(sampled_wp_id);
            std::stringstream ss;
            ss << "wp" << (sampled_waypoint_ids_.size()-1);
            uploadWPToParamServer(ss.str(), waypoints_[sampled_wp_id]);

            // reduce probability around waypoint
            int cell_x = (int) (waypoints_[sampled_wp_id].pose.position.x/map.info.resolution); 
            int cell_y = (int) (waypoints_[sampled_wp_id].pose.position.y/map.info.resolution);
            for(int x = cell_x - minimum_radius_/map.info.resolution; x < cell_x + minimum_radius_/map.info.resolution; x++) {
            for(int y = cell_y - minimum_radius_/map.info.resolution; y < cell_y + minimum_radius_/map.info.resolution; y++) {
                if(x<0 || x>=map.info.width) continue;
                if(y<0 || y>=map.info.height) continue;
                double d = (x-cell_x)*map.info.resolution * (x-cell_x)*map.info.resolution;
                d = d + (y-cell_y)*map.info.resolution * (y-cell_y)*map.info.resolution;
                if(d < minimum_radius_ * minimum_radius_) { 
                    int index =  x + y*map.info.width;
                    map.data[index] = 0;
                }
            }}

            // publish probability map
            if(animate_sampling_) {

                ros::Rate loopRate(0.5);
                // visualise
                clearWPGraph();
                pubWPGraph();
                
                grid_map::GridMap gm;
                grid_map::GridMapRosConverter::fromOccupancyGrid(map, "probability", gm);
                grid_map_msgs::GridMap gridMapMessage;
                grid_map::GridMapRosConverter::toMessage(gm, gridMapMessage);
                
                // scale grid map to 1m height
                for(int i=0; i<gridMapMessage.data[0].data.size(); i++)
                    gridMapMessage.data[0].data[i] = gridMapMessage.data[0].data[i]/25.0;

                probability_pub_.publish(gridMapMessage);

                ros::spinOnce();
                loopRate.sleep();
            }
            
        }

        // visualise
        clearWPGraph();
        pubWPGraph();

        // upload to KB
        updateKB();
        initializeDistancesKB();

        res.success = true;
        return true;
    }

    void RPRoadmapFilter::uploadWPToParamServer(const std::string &wp_id, const geometry_msgs::PoseStamped &waypoint) {

        // get theta from quaternion
        double roll, pitch, yaw;
        tf::Quaternion q(waypoint.pose.orientation.x, waypoint.pose.orientation.y, waypoint.pose.orientation.z, waypoint.pose.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::vector<double> pose_as_array;
        pose_as_array.push_back(waypoint.pose.position.x);
        pose_as_array.push_back(waypoint.pose.position.y);
        pose_as_array.push_back(yaw);
        std::stringstream ss;
        ss << wp_namespace_output_ << "/" << wp_id;
        nh_.setParam(ss.str(), pose_as_array);
    }

    void RPRoadmapFilter::pubWPGraph() {

        visualization_msgs::MarkerArray marker_array;
        
        // publish nodes as marker array
        int count = 0;
        std::vector<int>::iterator sit = sampled_waypoint_ids_.begin();
        for (; sit!=sampled_waypoint_ids_.end(); sit++) {

            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation().x() += waypoints_[*sit].pose.position.x;
            pose.translation().y() += waypoints_[*sit].pose.position.y;
            pose.translation().y() += 0.5f;
            geometry_msgs::Vector3 text_scale;
            text_scale.x = 0.5;
            text_scale.y = 0.5;
            text_scale.z = 0.5;
            std::stringstream ss;
            ss << "wp" << count;
            visual_tools_->publishText(pose, ss.str(), rviz_visual_tools::WHITE, text_scale, false);//rviz_visual_tools::XXXLARGE, false);

            visualization_msgs::Marker node_marker;
            node_marker.header.stamp = ros::Time();
            node_marker.header.frame_id = wp_reference_frame_;
            node_marker.ns = wp_namespace_output_;
            node_marker.type = visualization_msgs::Marker::CUBE;
            node_marker.action = visualization_msgs::Marker::MODIFY;
            node_marker.pose = waypoints_[*sit].pose;
            node_marker.scale.x = 0.35f;
            node_marker.scale.y = 0.35f;
            node_marker.scale.z = 0.35f;
            node_marker.color.a = 1.0f;
            node_marker.color.r = 1.0f;
            node_marker.color.g = 0.3f;
            node_marker.color.b = 0.3f;
            node_marker.id = count;
            
            marker_array.markers.push_back(node_marker);
            count++;
        }
        waypoints_pub_.publish(marker_array);
        visual_tools_->trigger();            
    }

    // clears all waypoints and edges
    void RPRoadmapFilter::clearWPGraph() {

        visualization_msgs::MarkerArray marker_array;
        int count = 0;
        std::vector<int>::iterator sit = sampled_waypoint_ids_.begin();
        for (; sit!=sampled_waypoint_ids_.end(); sit++) {
            visualization_msgs::Marker node_marker;
            node_marker.header.stamp = ros::Time();
            node_marker.header.frame_id = wp_reference_frame_;
            node_marker.ns = wp_namespace_output_;
            node_marker.action = visualization_msgs::Marker::DELETE;
            node_marker.id = count;
            marker_array.markers.push_back(node_marker);
            count++;
        }

        waypoints_pub_.publish( marker_array );

        visual_tools_->deleteAllMarkers();
    }

    void RPRoadmapFilter::updateKB() {

        // generate waypoints
        ROS_INFO("KCL: (%s) Updating KB", ros::this_node::getName().c_str());

        // add roadmap to knowledge base
        ROS_INFO("KCL: (%s) Adding knowledge", ros::this_node::getName().c_str());
        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updateInstSrv;
        int count = 0;
        std::stringstream ss;
        std::vector<int>::iterator sit = sampled_waypoint_ids_.begin();
        for (; sit!=sampled_waypoint_ids_.end(); sit++) {

            ss.str("");
            ss << "wp" << count;
            count ++;

            // instance
            rosplan_knowledge_msgs::KnowledgeItem item;
            updateInstSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
            item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
            item.instance_type = "waypoint";
            item.instance_name = ss.str();
            updateInstSrv.request.knowledge.push_back(item);
        }

        // wait for service existence
        if(!update_kb_client_.waitForExistence(ros::Duration(srv_timeout_))) {
            ROS_ERROR("KCL: (%s) Update KB service not found (%s)", ros::this_node::getName().c_str(), update_kb_client_.getService().c_str());
            return;
        }

        if(!update_kb_client_array_.call(updateInstSrv)) {
            ROS_INFO("KCL: (%s) Failed to call update service.", ros::this_node::getName().c_str());
        }
    }

    void RPRoadmapFilter::initializeDistancesKB() {

        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updatePredSrv;
        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updateFuncSrv;
        bool set_connected = false;

        //for (auto ait = sampled_waypoint_ids_.begin(); ait != sampled_waypoint_ids_.end(); ait++) {
            //for (auto bit = sampled_waypoint_ids_.begin(); bit != sampled_waypoint_ids_.end(); bit++) {
        for (int a=0; a<sampled_waypoint_ids_.size(); a++) {
            for (int b=a+1; b<sampled_waypoint_ids_.size(); b++) {

                double dist = 0;
                if (sampled_waypoint_ids_[a] != sampled_waypoint_ids_[b])
                    dist = dijkstra(sampled_waypoint_ids_[a], sampled_waypoint_ids_[b]);

                // Upload distance
                rosplan_knowledge_msgs::KnowledgeItem item;

                item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
                item.attribute_name = "distance";
                diagnostic_msgs::KeyValue pairFrom;
                pairFrom.key = "a";
                pairFrom.value = "wp"+std::to_string(a);
                item.values.push_back(pairFrom);
                diagnostic_msgs::KeyValue pairTo;
                pairTo.key = "b";
                pairTo.value = "wp"+std::to_string(b);
                item.values.push_back(pairTo);
                item.function_value = dist;

                updateFuncSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
                updateFuncSrv.request.knowledge.push_back(item);

                // upload distance in other direction 
                item.values.clear();
                pairFrom.value = "wp"+std::to_string(b);
                item.values.push_back(pairFrom);
                pairTo.value = "wp"+std::to_string(a);
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


    std::vector<float> RPRoadmapFilter::dijkstra_comp::distance = std::vector<float>();
    double RPRoadmapFilter::dijkstra(int a, int b) { //a source, b target

        int N = waypoints_.size();
        RPRoadmapFilter::dijkstra_comp::distance = std::vector<float>(N, FLT_MAX);        
        RPRoadmapFilter::dijkstra_comp::distance[a] = 0;

        std::priority_queue<int, std::vector<int>, RPRoadmapFilter::dijkstra_comp> Q;
        Q.push(a);

        while (not Q.empty()) {
            int u = Q.top();
            if (u == b) return RPRoadmapFilter::dijkstra_comp::distance[u];
            Q.pop();
            for (auto it = adj_matrix[u].begin() ; it != adj_matrix[u].end() ; ++it) { //forall neighbours
                double nd = RPRoadmapFilter::dijkstra_comp::distance[u] + it->second;
                if (nd < RPRoadmapFilter::dijkstra_comp::distance[it->first]) {
                    RPRoadmapFilter::dijkstra_comp::distance[it->first] = nd;
                    Q.push(it->first);
                }
            }
        }
        return -1;
    }

    int RPRoadmapFilter::extract_id(const std::string &id) { // assumed id of the form wpXXX
        return std::stoi(id.substr(2));
    }

} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "waypoint_sampler");
    KCL_rosplan::RPRoadmapFilter rms;
    ros::spin();
    return 0;
}
