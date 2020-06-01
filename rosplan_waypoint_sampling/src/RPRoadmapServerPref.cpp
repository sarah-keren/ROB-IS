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
        hppitsmap_received_ = false;
        hppits_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(hppits_topic_, 1, &RPRoadmapServerPref::hppitsMapCallback, this);
        nh_.param<bool>("generate_best_waypoints", _use_preference, true);



   }

   // SARAH:: get preference info
   void RPRoadmapServerPref::hppitsMapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {

        hppits_map_ = *msg;
        hppitsmap_received_ = true;
   }

    void RPRoadmapServerPref::createPRM(nav_msgs::OccupancyGrid &map,
                unsigned int nr_waypoints,
                double min_distance,
                double casting_distance,
                double connecting_distance,
                double occupancy_threshold,
                int total_attempts) {

	// get the preference map
	if (_use_preference) {	
		ros::Rate loop_rate(10);
		while (not hppitsmap_received_ and ros::ok()) {
		    loop_rate.sleep();
		    ROS_INFO("KCL: (%s) Waiting for hppits map...", ros::this_node::getName().c_str());
		    ros::spinOnce();
		}
	}


        // call super class implementation
        RPRoadmapServer::createPRM(map, nr_waypoints, min_distance, casting_distance, connecting_distance, occupancy_threshold, total_attempts);


    }
    
    void RPRoadmapServerPref::processWP(Waypoint* wp){
	/*
	int cell_x = (int) (wp->real_x/hppits_map_.info.resolution);
        int cell_y = (int) (wp->real_y/hppits_map_.info.resolution);
        double weight = hppits_map_.data[cell_x + cell_y*hppits_map_.info.width];	
        WPweights_.push_back(weight);     
	*/
	return;
	
    }		
    
    // code taken from http://www.cplusplus.com/reference/random/discrete_distribution/	
    // and https://stackoverflow.com/questions/31153610/setting-up-a-discrete-distribution-in-c
    int RPRoadmapServerPref::chooseWPtoExpand() {
	
          	        
	  // iterate through all waypoints to set their weights          
          double cur_weight = 0;  
        
	 
  	std::vector<double> WPweights;
	for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci) {
                Waypoint* cur_wp = (*ci).second;
		int cell_x = (int) (cur_wp->real_x/hppits_map_.info.resolution);
                int cell_y = (int) (cur_wp->real_y/hppits_map_.info.resolution);
                cur_weight = hppits_map_.data[cell_x + cell_y*hppits_map_.info.width];	
		                
                //std::cout << "for point: ("<<cell_x<<","<<cell_y<<") curweight is:" << cur_weight<< "\n\n\n\n\n\n"<< std::endl;  
                WPweights.push_back(cur_weight);     
          }         
 

          // generate wp according to weights 
          std::discrete_distribution<> distribution(WPweights.begin(), WPweights.end());
          //std::default_random_engine generator;
          std::random_device rd;
          std::mt19937 generator(rd());
          int index = distribution(generator);
	  /*while (index >= WPweights.size())
	  {
		std::cout << "index is:" << index<< "WPweights.size()"<< WPweights.size()<< std::endl;                  
		index = distribution(generator);
          }//while
	  */

	 

	  // print waypoint 	
	  std::map<std::string, Waypoint*>::iterator item = waypoints_.begin();
          std::advance(item, index);
          Waypoint* wp = (*item).second;
	  int cell_x = (int) (wp->real_x/hppits_map_.info.resolution);
          int cell_y = (int) (wp->real_y/hppits_map_.info.resolution);
          //std::cout << " index is: " << index<<" choosing WP: ("<< cell_x<< ","<<cell_y<<")"<<std::endl;  
          


	  return index;



	}//chooseWPtoExpand
    
} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_roadmap_server_pref");
    KCL_rosplan::RPRoadmapServerPref rms;
    ros::spin();
    return 0;
}
