/**
 * 
 * Copyright [2019] <KCL Kings College London>  
 * 
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk)
 * Maintainer: Michael Cashmore (michael.cashmore@kcl.ac.uk)
 * Maintainer: Oscar Lima (oscar.lima@dfki.de)
 * 
 * RPRoadmapServer class is used to generate N free collision waypoints from a costmap subscription
 * 
 * - Currently this works throught the nav_msgs/GetMap service.
 * - Waypoints are stored symbolically in the Knowledge Base.
 * - Waypoint coordinates are stored in the parameter server.
 * - Connectivity between waypoints is also computed.
 * - Loading existing waypoints from a file is supported.
 */

#include <sstream>
#include <string>
#include <rosplan_interface_mapping/RPRoadmapServer.h>

#ifndef KCL_rp_roadmap_server_pref
#define KCL_rp_roadmap_server_pref

namespace KCL_rosplan {

    class RPRoadmapServerPref:public RPRoadmapServer
    {

      public:

      RPRoadmapServerPref();

      protected: 	
        /**
         * @brief Waypoint generation function, deletes all previous data and generates a new waypoint set
         * @param map the costmap expressing the occupied and free cells of the map such that we do not
         * generate waypoints on top of occupied cells
         * @param nr_waypoints the desired number of waypoint to generate
         * @param min_distance the minimum distance allowed between any pair of waypoints
         * @param casting_distance the maximum distance a waypoint can be cast
         * @param connecting_distance the maximum distance that can exists between waypoints for them to be considered connected
         * @param total_attempts maximum amount of attempts to generate random valid waypoints
         */
        virtual void createPRM(nav_msgs::OccupancyGrid &map, unsigned int nr_waypoints, double min_distance, double casting_distance, double connecting_distance, double occupancy_threshold, int total_attempts);


        virtual int chooseWPtoExpand();
	
	/** 
        * @analyze the chosen WP by computing its preference
        * @return void
        */
        virtual void processWP(Waypoint* wp);
 

        // SARAH::
        std::string costmap_topic_,hppits_topic_;
        bool costmap_received_, hppitsmap_received_;
        nav_msgs::OccupancyGrid cost_map_, hppits_map_;
        ros::Subscriber hppits_sub_;
        void hppitsMapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
        bool _use_preference = true;



  
      private:

    };
}
#endif
