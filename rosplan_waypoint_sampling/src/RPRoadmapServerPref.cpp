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
	nh_.param<std::string>("prefs_topic", prefs_topic_, "prefs_map");
	std::cout<<"prefs_topic_ "<< prefs_topic_<<"\n\n\n\n"<<std::endl;
        prefsmap_received_ = false;
        prefs_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(prefs_topic_, 1, &RPRoadmapServerPref::prefsMapCallback, this);
        nh_.param<bool>("generate_best_waypoints", _use_preference, true);

	/*
 	if(!nh_.hasParam("pref_approach")) {	
            prefApproach_ = rospy.get_param('pref_approach');
        
        }
        else
	{
   	    prefApproach_ = prefApproachEnum::a;
	}
	*/	

        prefApproach_ = prefApproachEnum::a;	



   }

   // SARAH:: get preference info
   void RPRoadmapServerPref::prefsMapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {

        prefs_map_ = *msg;
        prefsmap_received_ = true;
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
		while (not prefsmap_received_ and ros::ok()) {
		    loop_rate.sleep();
		    ROS_INFO("KCL: (%s) Waiting for prefs map...", ros::this_node::getName().c_str());
		    ros::spinOnce();
		}
	}


        // call super class implementation
        RPRoadmapServer::createPRM(map, nr_waypoints, min_distance, casting_distance, connecting_distance, occupancy_threshold, total_attempts);


    }

    Waypoint* RPRoadmapServerPref::castNewWP(Waypoint* casting_wp, double casting_distance, double occupancy_threshold, const nav_msgs::OccupancyGrid &map)
    {

       	 if ((_use_preference)&&(prefApproach_ == prefApproachEnum::a)) { 
	 
		 //get map information
		 int width = map.info.width;
		 int height = map.info.height;
		 double resolution = map.info.resolution; // m per cell

	    	 //sample a set of waypoints	 
		 int x = 0;
		 int y = 0;
		 Waypoint* cur_wp = NULL;
		 
	 	 // wps and their preference weight
		 std::vector<double> WPweights;
		 std::vector<Waypoint*> WPs;
		 //TODO SARAH - how to make sure this is not an infinite loop?
		 while (WPs.size() < NUM_CASTING_WPS)
		 {

		   x = rand() % width;
		   y = rand() % height;
		   //std::cout << "for point: ("<<x<<","<<y<<")" << "\n\n\n\n\n\n"<< std::endl;  
		   std::stringstream ss;
		   ss << "wp" << WPs.size();
		   cur_wp = new Waypoint(ss.str(), x, y, map.info);
		   // Move the waypoint closer so it's no further than @ref{casting_distance} away from the casting_wp.
		   //std::cout << "cast wapoint before: ("<<cur_wp->real_x<<","<<cur_wp->real_x<<")" << "\n\n\n\n\n\n"<< std::endl;  
		   cur_wp->update(*casting_wp, casting_distance, map.info);
		   //std::cout << "cast wapoint after: ("<<cur_wp->real_x<<","<<cur_wp->real_x<<")" << "\n\n\n\n\n\n"<< std::endl;  

		   if(map.data[y*width+x] > occupancy_threshold) {
		        delete cur_wp;
		    }//if
		    else
		    {	
			//insert the wp and its weights into the lists   
		        double cur_weight = getPref(cur_wp); 
			WPweights.push_back(cur_weight);  
		        WPs.push_back(cur_wp);  	
		    }//else
	   	 
		 }//while  	
		 
		  // generate wp according to weights 
		  std::discrete_distribution<> distribution(WPweights.begin(), WPweights.end());
		  //std::default_random_engine generator;
		  std::random_device rd;
		  std::mt19937 generator(rd());
		  int index = distribution(generator);	 

		  //get the wp
		  std::vector<Waypoint*>::iterator item = WPs.begin();
		  std::advance(item, index);
		  Waypoint* wp = (*item);
		  //std::cout << "cast wapoint: ("<<wp->real_x<<","<<wp->real_x<<")" << "\n\n\n\n\n\n"<< std::endl;  

		return wp;
	}//if 
	else
	{
	 return RPRoadmapServer::castNewWP(casting_wp, casting_distance, occupancy_threshold, map);
	}//else
    }	

    
    void RPRoadmapServerPref::processWP(Waypoint* wp){
	/*
	int cell_x = (int) (wp->real_x/prefs_map_.info.resolution);
        int cell_y = (int) (wp->real_y/prefs_map_.info.resolution);
        double weight = prefs_map_.data[cell_x + cell_y*prefs_map_.info.width];	
        WPweights_.push_back(weight);     
	*/
	return;
	
    }		

    double RPRoadmapServerPref::getPref(Waypoint* wp){

	int cell_x = (int) (wp->real_x/prefs_map_.info.resolution);
        int cell_y = (int) (wp->real_y/prefs_map_.info.resolution);
        double pref = prefs_map_.data[cell_x + cell_y*prefs_map_.info.width];			                
	return pref;
    }

    
    // code taken from http://www.cplusplus.com/reference/random/discrete_distribution/	
    // and https://stackoverflow.com/questions/31153610/setting-up-a-discrete-distribution-in-c
    int RPRoadmapServerPref::chooseWPtoExpand() {
	  
          if ((_use_preference)&&(prefApproach_ == prefApproachEnum::b)) {   	        
	  // iterate through all waypoints to set their weights          
          double cur_weight = 0;  
        
	 
  	std::vector<double> WPweights;
	for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci) {
                Waypoint* cur_wp = (*ci).second;	
         	cur_weight = getPref(cur_wp);
                //std::cout << "for point: ("<<cell_x<<","<<cell_y<<") curweight is:" << cur_weight<< "\n\n\n\n\n\n"<< std::endl;
                WPweights.push_back(cur_weight);     
          }         
 

          // generate wp according to weights 
          std::discrete_distribution<> distribution(WPweights.begin(), WPweights.end());
          //std::default_random_engine generator;
          std::random_device rd;
          std::mt19937 generator(rd());
          int index = distribution(generator);	 

	  // print waypoint 
	  /*		
	  std::map<std::string, Waypoint*>::iterator item = waypoints_.begin();
          std::advance(item, index);
          Waypoint* wp = (*item).second;
	  int cell_x = (int) (wp->real_x/prefs_map_.info.resolution);
          int cell_y = (int) (wp->real_y/prefs_map_.info.resolution);
          std::cout << " index is: " << index<<" choosing WP: ("<< cell_x<< ","<<cell_y<<")"<<std::endl;  
          */
	  return index;
	 }
	else
	{
  	  return RPRoadmapServer::chooseWPtoExpand();
	} 	



	}//chooseWPtoExpand
    
} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_roadmap_server_pref");
    KCL_rosplan::RPRoadmapServerPref rms;
    ros::spin();
    return 0;
}
