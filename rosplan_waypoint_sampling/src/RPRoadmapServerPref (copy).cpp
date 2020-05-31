/**
 * 
 * Copyright [2019] <KCL King's College London>
 * 
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk)
 * Maintainer: Michael Cashmore (michael.cashmore@kcl.ac.uk)
 * Maintainer: Oscar Lima (oscar.lima@dfki.de)
 * 
 * RPRoadmapServerPref class is used to generate N free collision waypoints from a costmap subscription
 * 
 * - Currently this works throught the nav_msgs/GetMap service.
 * - Waypoints are stored symbolically in the Knowledge Base.
 * - Waypoint coordinates are stored in the parameter server.
 * - Connectivity between waypoints is also computed.
 * - Loading existing waypoints from a file is supported.
 * 
 */

#include "robis_interface_mapping/RPRoadmapServerPref.h"
#include <random>
namespace KCL_rosplan {

    /*-----------------*/
    /* RPRoadmapServerPref */
    /*-----------------*/

    /**
     * Constructor
     */
    RPRoadmapServerPref::RPRoadmapServerPref(){        
        
        // SARAH:: initialize preference info
	nh_.param<std::string>("hppits_topic", hppits_topic_, "hppits_map");
	std::cout<<"hppits_topic_ "<< hppits_topic_<<"\n\n\n\n"<<std::endl;
        hppits_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(hppits_topic_, 1, &RPRoadmapServerPref::hppitsMapCallback, this);
        nh_.param<bool>("generate_best_waypoints", _use_preference, true);


   }

   // SARAH:: get preference info
   void RPRoadmapServerPref::hppitsMapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
        hppits_map_ = *msg;
        hppitsmap_received_ = true;
    }


    void RPRoadmapServerPref::createPRMFull(nav_msgs::OccupancyGrid &map,
                unsigned int nr_waypoints,
                double min_distance,
                double casting_distance,
                double connecting_distance,
                double occupancy_threshold,
                int total_attempts) {


        // map info
        int width = map.info.width;
        int height = map.info.height;
        double resolution = map.info.resolution; // m per cell

        if(width==0 || height==0) {
            ROS_INFO("KCL: (%s) Empty map", ros::this_node::getName().c_str());
            return;
        }

	// SARAH:: hppitsmap: get preference info
	if (_use_preference) {	
		ros::Rate loop_rate(10);
		while (not hppitsmap_received_ and ros::ok()) {
		    loop_rate.sleep();
		    ROS_INFO("KCL: (%s) Waiting for hppits map...", ros::this_node::getName().c_str());
		    ros::spinOnce();
		}
	}
        

        // V <-- empty set; E <-- empty set.
        for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci) {
            delete (*ci).second;
        }
        waypoints_.clear();
        edges_.clear();

        ros::Rate r(10);
        while (not odom_received_) {
            ros::spinOnce();
            r.sleep();
            std::cout << "GERERAD" << std::endl;
        }

        // create robot start point
        geometry_msgs::PoseStamped start_pose;
        geometry_msgs::PoseStamped start_pose_transformed;
        start_pose.header = base_odom_.header;
        start_pose.pose.position = base_odom_.pose.position;
        start_pose.pose.orientation = base_odom_.pose.orientation;
        try {
            tf_.waitForTransform( base_odom_.header.frame_id, "map", ros::Time::now(), ros::Duration(100) );
            tf_.transformPose("map", start_pose, start_pose_transformed);
        } catch(tf::LookupException& ex) {
            ROS_ERROR("KCL: (%s) Lookup Error: %s", ros::this_node::getName().c_str(), ex.what());
            return;
        } catch(tf::ConnectivityException& ex) {
            ROS_ERROR("KCL: (%s) Connectivity Error: %s", ros::this_node::getName().c_str(), ex.what());
            return;
        } catch(tf::ExtrapolationException& ex) {
            ROS_ERROR("KCL: (%s) Extrapolation Error: %s", ros::this_node::getName().c_str(), ex.what());
            return;
        }

        occupancy_grid_utils::Cell start_cell = occupancy_grid_utils::pointCell(map.info, start_pose_transformed.pose.position);
        Waypoint* start_wp = new Waypoint("wp0", start_cell.x, start_cell.y, map.info);
        waypoints_[start_wp->wpID] = start_wp;




        int loop_counter = 0;
        while(waypoints_.size() < nr_waypoints && ++loop_counter < total_attempts) {
            std::cout << "DELETE ME: generating WP:" << loop_counter<< "and: "<<_use_preference<<"\n\n\n\n\n\n"<< std::endl;	
            // SARAH:: hppitsmap: get preference info
	    int wp_index = -1 ;
	    // sample a waypoint according to preferences	
	    if (_use_preference) {	
               wp_index = this->chooseWPtoExpand();	
	    }	
            // Sample a random waypoint, if possible with fewer than 6 neighbours.
	    else 
            {
		wp_index = rand() % waypoints_.size();
            }	

            std::map<std::string, Waypoint*>::iterator item = waypoints_.begin();
            std::advance(item, wp_index);
            Waypoint* casting_wp = (*item).second;

            // sample collision-free configuration at random
            int x = rand() % width;
            int y = rand() % height;

            std::stringstream ss;
            ss << "wp" << waypoints_.size();
            Waypoint* wp = new Waypoint(ss.str(), x, y, map.info);

            // Move the waypoint closer so it's no further than @ref{casting_distance} away from the casting_wp.
            wp->update(*casting_wp, casting_distance, map.info);

            if(map.data[y*width+x] > occupancy_threshold) {
                delete wp;
                continue;
            }

            // Check whether this waypoint is connected to any of the existing waypoints.
            geometry_msgs::Point p1, p2;
            p1.x = wp->real_x;
            p1.y = wp->real_y;

            // Ignore waypoint that are too close to existing waypoints.
            float min_distance_to_other_wp = std::numeric_limits<float>::max();
            for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci) {
                float distance = wp->getDistance(*(*ci).second);
                if (distance < min_distance_to_other_wp)
                    min_distance_to_other_wp = distance;
            }

            if (min_distance_to_other_wp < min_distance) {
                delete wp;
                continue;
            }

	    	
            for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci) {
                Waypoint* other_wp = (*ci).second;
                p2.x = other_wp->real_x;
                p2.y = other_wp->real_y;
               
                if (wp->getDistance(*other_wp) < connecting_distance && canConnect(p1, p2, map, occupancy_threshold)) {
                    wp->neighbours.push_back(other_wp->wpID);
                    other_wp->neighbours.push_back(wp->wpID);
                    Edge e(wp->wpID, other_wp->wpID);
                    edges_.push_back(e);
                }
            }
	    

            if (wp->neighbours.size() > 0) {
                waypoints_[wp->wpID] = wp;
            }
        }
    }

    // code taken from http://www.cplusplus.com/reference/random/discrete_distribution/	
    // and https://stackoverflow.com/questions/31153610/setting-up-a-discrete-distribution-in-c
    int RPRoadmapServerPref::chooseWPtoExpand() {
	  //SARAH:: move this to the generation process 
	  // iterate through all waypoints to set their weights          
          std::vector<double> weights;
          double cur_weight = 0;
	  for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci) {
                Waypoint* cur_wp = (*ci).second;
		int cell_x = (int) (cur_wp->real_x/hppits_map_.info.resolution);
                int cell_y = (int) (cur_wp->real_y/hppits_map_.info.resolution);
                cur_weight = hppits_map_.data[cell_x + cell_y*hppits_map_.info.width];	
                std::cout << "for point: ("<<cell_x<<","<<cell_y<<") curweight is:" << cur_weight<< "\n\n\n\n\n\n"<< std::endl;  
                weights.push_back(cur_weight);     
          }
                       

       
          // generate wp according to weights 
          std::discrete_distribution<> distribution(weights.begin(), weights.end());
          std::default_random_engine generator;
          int index = distribution(generator);

	  return index;



	}//chooseWPtoExpand
    
} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_roadmap_server_pref");
    KCL_rosplan::RPRoadmapServerPref rms;
    ros::spin();
    return 0;
}
