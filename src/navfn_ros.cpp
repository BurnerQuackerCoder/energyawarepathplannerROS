/*********************************************************************
* Author: Jay Puppala
*********************************************************************/
#include <enawpl/navfn_ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <sensor_msgs/point_cloud2_iterator.h>
//Added
#include <enawpl/navfn.h> 

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(enawpl::NavfnROS, nav_core::BaseGlobalPlanner)

namespace enawpl {

  NavfnROS::NavfnROS() 
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true),
      // ADDED: Initialize new members
      current_battery_percent_(100.0), // Assume full battery initially
      battery_data_received_(false),
      critical_battery_threshold_(20.0), // Default, will be overridden by param
      base_energy_penalty_per_unit_(10.0),   // Default
      critical_battery_penalty_multiplier_(2.0) {} // Default
      // END ADDED 

  NavfnROS::NavfnROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {
      //initialize the planner
      initialize(name, costmap_ros);
  }

  NavfnROS::NavfnROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {
      //initialize the planner
      initialize(name, costmap, global_frame);
  }

  void NavfnROS::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame){
    if(!initialized_){
      costmap_ = costmap;
      global_frame_ = global_frame;
      planner_ = boost::shared_ptr<NavFn>(new NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));

      ros::NodeHandle private_nh("~/" + name);
      ros::NodeHandle nh; // For global topics like /battery

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      private_nh.param("visualize_potential", visualize_potential_, false);

      //if we're going to visualize the potential array we need to advertise
      if(visualize_potential_)
        potarr_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("potential", 1);

      private_nh.param("allow_unknown", allow_unknown_, true);
      private_nh.param("planner_window_x", planner_window_x_, 0.0);
      private_nh.param("planner_window_y", planner_window_y_, 0.0);
      private_nh.param("default_tolerance", default_tolerance_, 0.0);

      make_plan_srv_ =  private_nh.advertiseService("make_plan", &NavfnROS::makePlanService, this);

      // ADDED: Load energy parameters and set up battery subscriber
      loadEnergyParams(private_nh);
      battery_sub_ = nh.subscribe<std_msgs::Float32>("/battery", 1, &NavfnROS::batteryCallback, this);
      // END ADDED

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }

  void NavfnROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
    
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point){
    return validPointPotential(world_point, default_tolerance_);
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point, double tolerance){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    double resolution = costmap_->getResolution();
    geometry_msgs::Point p;
    p = world_point;

    p.y = world_point.y - tolerance;

    while(p.y <= world_point.y + tolerance){
      p.x = world_point.x - tolerance;
      while(p.x <= world_point.x + tolerance){
        double potential = getPointPotential(p);
        if(potential < POT_HIGH){
          return true;
        }
        p.x += resolution;
      }
      p.y += resolution;
    }

    return false;
  }

  double NavfnROS::getPointPotential(const geometry_msgs::Point& world_point){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return -1.0;
    }

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
      return DBL_MAX;

    unsigned int index = my * planner_->nx + mx;
    return planner_->potarr[index];
  }

  bool NavfnROS::computePotential(const geometry_msgs::Point& world_point){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //make sure to resize the underlying array that Navfn uses
    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
      return false;

    int map_start[2];
    map_start[0] = 0;
    map_start[1] = 0;

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_start);
    planner_->setGoal(map_goal);

    return planner_->calcNavFnDijkstra();
  }

  void NavfnROS::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
  }

  bool NavfnROS::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp){
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = global_frame_;

    return true;
  } 

  void NavfnROS::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
  }

  bool NavfnROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    return makePlan(start, goal, default_tolerance_, plan);
  }

  bool NavfnROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::mutex::scoped_lock lock(mutex_);
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(goal.header.frame_id != global_frame_){
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    if(start.header.frame_id != global_frame_){
      ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_frame_.c_str(), start.header.frame_id.c_str());
      return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)){
      ROS_WARN_THROTTLE(1.0, "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
      return false;
    }

    //ROS_INFO("ROBOT_START_CELL_DEBUG: Current plan request starts at (mx: %u, my: %u)", mx, my);

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, mx, my);

    // JP_MODIFIED: Prepare energy-aware costmap
    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();
    unsigned int map_size = size_x * size_y;
    ROS_INFO_THROTTLE(1.0, "NavfnROS: size_x (%u) size_y (%u)",
                      size_x, size_y);
    unsigned char* original_ros_costmap_chars = costmap_->getCharMap();
    unsigned char* energy_aware_cost_array = new unsigned char[map_size];



    // For simplicity in this PoC, if battery data hasn't been received yet, plan with default high battery assumption
    float actual_battery_for_planning = battery_data_received_ ? current_battery_percent_ : 100.0;
    if (!battery_data_received_) {
        ROS_WARN_ONCE("NavfnROS: Battery data not yet received, planning with assumed full battery.");
    }


    double current_penalty_multiplier = 1.0;
    if (actual_battery_for_planning < critical_battery_threshold_) {
        current_penalty_multiplier = critical_battery_penalty_multiplier_;
        ROS_INFO_THROTTLE(1.0, "NavfnROS: Battery (%.1f%%) is below critical (%.1f%%). Applying penalty multiplier: %.2f",
                      actual_battery_for_planning, critical_battery_threshold_, current_penalty_multiplier);
    } else {
        ROS_INFO_THROTTLE(1.0, "NavfnROS: Battery (%.1f%%) is above critical (%.1f%%). Normal penalties apply.",
                      actual_battery_for_planning, critical_battery_threshold_);
    }

    ROS_INFO_ONCE("NavfnROS: Debug - Checking for cells with specific cost ranges. This might be verbose.");


    for (unsigned int i = 0; i < map_size; ++i) {
        unsigned char ros_cell_cost = original_ros_costmap_chars[i];
        unsigned int current_mx = i % size_x;
        unsigned int current_my = i / size_x;
        //unsigned char base_navfn_cell_cost = calculateBaseNavfnCost(ros_cell_cost, allow_unknown_);
        unsigned char final_ros_cost_for_navfn = ros_cell_cost; // Start with the original

        // MODIFIED: Determine terrain_energy_factor_for_cell based on zones
        double terrain_energy_factor_for_cell = 1.0; // Default factor for cells not in any defined zone
        std::string current_zone_name = "default"; // For logging
        for (const auto& zone_def : zone_terrain_definitions_) {
          //ROS_INFO_THROTTLE(1.0,"Inside For, Cell (%u,%u) is in zone (%u,%u), %s with factor %.2f", current_mx, current_my, zone_def.min_mx, zone_def.max_mx ,zone_def.name.c_str(), terrain_energy_factor_for_cell);
            if (current_mx >= zone_def.min_mx && current_mx <= zone_def.max_mx &&
                current_my >= zone_def.min_my && current_my <= zone_def.max_my) {
                terrain_energy_factor_for_cell = zone_def.energy_usage_factor;
                //ROS_INFO_THROTTLE(5.0,"Cell (%u,%u) is in zone %s with factor %.2f", current_mx, current_my, zone_def.name.c_str(), terrain_energy_factor_for_cell); // Optional debug
                current_zone_name = zone_def.name;
                break; // Use the first matching zone. Define more specific zones first in YAML if they overlap.
            }
        }
        // END MODIFIED

        double applied_additional_ros_penalty = 0.0; // To log the actual penalty

        if (ros_cell_cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) { // e.g. < 253
             applied_additional_ros_penalty = (terrain_energy_factor_for_cell - 1.0) * base_energy_penalty_per_unit_ * current_penalty_multiplier;

            if (applied_additional_ros_penalty < 0.0) applied_additional_ros_penalty = 0.0;

            int potentially_modified_ros_cost = static_cast<int>(ros_cell_cost) + static_cast<int>(applied_additional_ros_penalty);

            // Cap the modified cost to ensure it doesn't become an obstacle unintentionally
            // It should not exceed the value just below inscribed obstacles.
            if (potentially_modified_ros_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                final_ros_cost_for_navfn = costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1; 
            } else {
                final_ros_cost_for_navfn = static_cast<unsigned char>(potentially_modified_ros_cost);
            }

          }

    energy_aware_cost_array[i] = final_ros_cost_for_navfn;
    }

    planner_->setNavArr(size_x, size_y);
    // Pass our dynamically generated energy-aware cost array
    planner_->setCostmap(energy_aware_cost_array, true, allow_unknown_); 
    // END MODIFIED
    // ADDED: Clean up the dynamically allocated cost array
    delete[] energy_aware_cost_array;
    energy_aware_cost_array = nullptr;
    // END ADDED


    //make sure to resize the underlying array that Navfn uses
    // JP Below commented
    //planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    //planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if(!costmap_->worldToMap(wx, wy, mx, my)){
      if(tolerance <= 0.0){
        ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
        return false;
      }
      mx = 0;
      my = 0;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);
    planner_->setGoal(map_start);

    //bool success = planner_->calcNavFnAstar();
    planner_->calcNavFnDijkstra(true);

    double resolution = costmap_->getResolution();
    geometry_msgs::PoseStamped p, best_pose;
    p = goal;

    bool found_legal = false;
    double best_sdist = DBL_MAX;

    p.pose.position.y = goal.pose.position.y - tolerance;

    while(p.pose.position.y <= goal.pose.position.y + tolerance){
      p.pose.position.x = goal.pose.position.x - tolerance;
      while(p.pose.position.x <= goal.pose.position.x + tolerance){
        double potential = getPointPotential(p.pose.position);
        double sdist = sq_distance(p, goal);
        if(potential < POT_HIGH && sdist < best_sdist){
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.pose.position.x += resolution;
      }
      p.pose.position.y += resolution;
    }

    if(found_legal){
      //extract the plan
      if(getPlanFromPotential(best_pose, plan)){
        //make sure the goal we push on has the same timestamp as the rest of the plan
        geometry_msgs::PoseStamped goal_copy = best_pose;
        goal_copy.header.stamp = ros::Time::now();
        plan.push_back(goal_copy);
      }
      else{
        ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
      }
    }

    if (visualize_potential_)
    {
      // Publish the potentials as a PointCloud2
      sensor_msgs::PointCloud2 cloud;
      cloud.width = 0;
      cloud.height = 0;
      cloud.header.stamp = ros::Time::now();
      cloud.header.frame_id = global_frame_;
      sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
      cloud_mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                        "y", 1, sensor_msgs::PointField::FLOAT32,
                                        "z", 1, sensor_msgs::PointField::FLOAT32,
                                        "pot", 1, sensor_msgs::PointField::FLOAT32);
      cloud_mod.resize(planner_->ny * planner_->nx);
      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");

      PotarrPoint pt;
      float *pp = planner_->potarr;
      double pot_x, pot_y;
      for (unsigned int i = 0; i < (unsigned int)planner_->ny*planner_->nx ; i++)
      {
        if (pp[i] < 10e7)
        {
          mapToWorld(i%planner_->nx, i/planner_->nx, pot_x, pot_y);
          iter_x[0] = pot_x;
          iter_x[1] = pot_y;
          iter_x[2] = pp[i]/pp[planner_->start[1]*planner_->nx + planner_->start[0]]*20;
          iter_x[3] = pp[i];
          ++iter_x;
        }
      }
      potarr_pub_.publish(cloud);
    }

    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);

    return !plan.empty();
  }

  void NavfnROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //create a message for the plan 
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    
    if(path.empty()) {
      //still set a valid frame so visualization won't hit transform issues
    	gui_path.header.frame_id = global_frame_;
      gui_path.header.stamp = ros::Time::now();
    } else { 
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }

  bool NavfnROS::getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //clear the plan, just in case
    plan.clear();

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(goal.header.frame_id != global_frame_){
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    double wx = goal.pose.position.x;
    double wy = goal.pose.position.y;

    //the potential has already been computed, so we won't update our copy of the costmap
    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)){
      ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
      return false;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);

    planner_->calcPath(costmap_->getSizeInCellsX() * 4);

    //extract the plan
    float *x = planner_->getPathX();
    float *y = planner_->getPathY();
    int len = planner_->getPathLen();
    ros::Time plan_time = ros::Time::now();

    for(int i = len - 1; i >= 0; --i){
      //convert the plan to world coordinates
      double world_x, world_y;
      mapToWorld(x[i], y[i], world_x, world_y);

      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }

    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    return !plan.empty();
  }
  // ADDED: Battery callback implementation
  void NavfnROS::batteryCallback(const std_msgs::Float32::ConstPtr& msg) {
      current_battery_percent_ = msg->data;
      if (!battery_data_received_){
          ROS_INFO("NavfnROS: Received first battery update: %.1f%%", current_battery_percent_);
          battery_data_received_ = true;
      }
  }
  // END ADDED
  // ADDED: Parameter loading implementation
  /*void NavfnROS::loadEnergyParams(ros::NodeHandle& private_nh) {
      private_nh.param("critical_battery_threshold", critical_battery_threshold_, 20.0);
      private_nh.param("base_energy_penalty_per_cell", base_energy_penalty_per_unit_, 10.0);
      private_nh.param("critical_battery_penalty_multiplier", critical_battery_penalty_multiplier_, 2.0);

      ROS_INFO("NavfnROS Energy Params: CriticalBattery=%.1f%%, BasePenalty=%.1f, BoostMultiplier=%.1f",
              critical_battery_threshold_, base_energy_penalty_per_unit_, critical_battery_penalty_multiplier_);

      XmlRpc::XmlRpcValue terrain_list;
      if (private_nh.getParam("terrain_definitions", terrain_list)) {
          ROS_ASSERT(terrain_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
          if (terrain_list.size() == 0) {
              ROS_WARN("NavfnROS: terrain_definitions list is empty.");
          }

          for (int32_t i = 0; i < terrain_list.size(); ++i) {
              ROS_ASSERT(terrain_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
              TerrainInfo current_terrain;
              bool success = true;

              if (terrain_list[i].hasMember("raw_map_value") && terrain_list[i]["raw_map_value"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                  current_terrain.raw_map_value = static_cast<unsigned char>(static_cast<int>(terrain_list[i]["raw_map_value"]));
              } else {
                  ROS_ERROR("NavfnROS: Terrain definition %d missing 'raw_map_value' or it's not an Int.", i);
                  success = false;
              }

              if (terrain_list[i].hasMember("name") && terrain_list[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString) {
                  current_terrain.name = static_cast<std::string>(terrain_list[i]["name"]);
              } else {
                  ROS_ERROR("NavfnROS: Terrain definition %d missing 'name' or it's not a String.", i);
                  success = false;
              }

              if (terrain_list[i].hasMember("energy_usage_factor") && terrain_list[i]["energy_usage_factor"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                  current_terrain.energy_usage_factor = static_cast<double>(terrain_list[i]["energy_usage_factor"]);
              } else {
                  ROS_ERROR("NavfnROS: Terrain definition %d missing 'energy_usage_factor' or it's not a Double.", i);
                  success = false;
              }

              if(success){
                  terrain_definitions_.push_back(current_terrain);
                  ROS_INFO("NavfnROS: Loaded terrain: %s, raw_value: %d, energy_factor: %.2f",
                          current_terrain.name.c_str(), current_terrain.raw_map_value, current_terrain.energy_usage_factor);
              }
          }
      } else {
          ROS_WARN("NavfnROS: Param 'terrain_definitions' not found. No terrain-specific costs will be applied.");
      }
  }
  // END ADDED */

  // MODIFIED

  void NavfnROS::loadEnergyParams(ros::NodeHandle& private_nh) {
    private_nh.param("critical_battery_threshold", critical_battery_threshold_, 20.0);
    // Ensure you are using the renamed parameter for ROS cost penalty
    private_nh.param("base_ros_cost_penalty_unit", base_energy_penalty_per_unit_, 10.0); 
    private_nh.param("critical_battery_penalty_multiplier", critical_battery_penalty_multiplier_, 2.0);

    ROS_INFO("NavfnROS Energy Params: CriticalBattery=%.1f%%, BaseROSCostPenaltyUnit=%.1f, BoostMultiplier=%.1f",
             critical_battery_threshold_, base_energy_penalty_per_unit_, critical_battery_penalty_multiplier_);

    // Clear previous definitions if any (e.g., on re-initialize, though not typical for planners)
    zone_terrain_definitions_.clear(); 

    XmlRpc::XmlRpcValue zone_list_param; // Changed variable name for clarity
    if (private_nh.getParam("zone_terrain_definitions", zone_list_param)) { // Changed param name
        ROS_ASSERT(zone_list_param.getType() == XmlRpc::XmlRpcValue::TypeArray);
        if (zone_list_param.size() == 0) {
             ROS_WARN("NavfnROS: zone_terrain_definitions list is empty.");
        }

        for (int32_t i = 0; i < zone_list_param.size(); ++i) {
            ROS_ASSERT(zone_list_param[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
            ZoneTerrainInfo current_zone; // Use new struct
            bool success = true;

            // Name (optional but good for debugging)
            if (zone_list_param[i].hasMember("name") && zone_list_param[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString) {
                current_zone.name = static_cast<std::string>(zone_list_param[i]["name"]);
            } else {
                current_zone.name = "zone_" + std::to_string(i); // Default name
            }

            // min_mx
            if (zone_list_param[i].hasMember("min_mx") && zone_list_param[i]["min_mx"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                current_zone.min_mx = static_cast<unsigned int>(static_cast<int>(zone_list_param[i]["min_mx"]));
            } else {
                ROS_ERROR("NavfnROS: Zone definition %d for '%s' missing 'min_mx' or it's not an Int.", i, current_zone.name.c_str());
                success = false;
            }
            // max_mx
            if (zone_list_param[i].hasMember("max_mx") && zone_list_param[i]["max_mx"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                current_zone.max_mx = static_cast<unsigned int>(static_cast<int>(zone_list_param[i]["max_mx"]));
            } else {
                ROS_ERROR("NavfnROS: Zone definition %d for '%s' missing 'max_mx' or it's not an Int.", i, current_zone.name.c_str());
                success = false;
            }
            // min_my
            if (zone_list_param[i].hasMember("min_my") && zone_list_param[i]["min_my"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                current_zone.min_my = static_cast<unsigned int>(static_cast<int>(zone_list_param[i]["min_my"]));
            } else {
                ROS_ERROR("NavfnROS: Zone definition %d for '%s' missing 'min_my' or it's not an Int.", i, current_zone.name.c_str());
                success = false;
            }
            // max_my
            if (zone_list_param[i].hasMember("max_my") && zone_list_param[i]["max_my"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                current_zone.max_my = static_cast<unsigned int>(static_cast<int>(zone_list_param[i]["max_my"]));
            } else {
                ROS_ERROR("NavfnROS: Zone definition %d for '%s' missing 'max_my' or it's not an Int.", i, current_zone.name.c_str());
                success = false;
            }

            // energy_usage_factor
            if (zone_list_param[i].hasMember("energy_usage_factor") && zone_list_param[i]["energy_usage_factor"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                current_zone.energy_usage_factor = static_cast<double>(zone_list_param[i]["energy_usage_factor"]);
            } else {
                ROS_ERROR("NavfnROS: Zone definition %d for '%s' missing 'energy_usage_factor' or it's not a Double.", i, current_zone.name.c_str());
                success = false;
            }

            if(success){
                zone_terrain_definitions_.push_back(current_zone);
                ROS_INFO("NavfnROS: Loaded terrain zone: %s, min_mx: %u, max_mx: %u, min_my: %u, max_my: %u, energy_factor: %.2f",
                         current_zone.name.c_str(), current_zone.min_mx, current_zone.max_mx, current_zone.min_my, current_zone.max_my, current_zone.energy_usage_factor);
            }
        }
    } else {
        ROS_WARN("NavfnROS: Param 'zone_terrain_definitions' not found. No zone-specific energy costs will be applied.");
    }
  }

  // END MODIFIED

  // ADDED: Helper to calculate base navfn cost for a cell
  unsigned char NavfnROS::calculateBaseNavfnCost(unsigned char ros_cost, bool allow_unknown_local) {
      // This logic mirrors parts of NavFn::setCostmap for a single cell
      if (ros_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) { // 253 and 254 (LETHAL)
          return COST_OBS; // NavFn's internal obstacle cost (254)
      } else if (ros_cost == costmap_2d::NO_INFORMATION && allow_unknown_local) { // 255
          // In NavFn::setCostmap, this translates to:
          // v = COST_UNKNOWN_ROS (255) -> if allow_unknown, v = COST_OBS-1
          // Since ros_cost is already COST_UNKNOWN_ROS (255 from costmap_2d::NO_INFORMATION)
          return COST_OBS - 1;
      } else if (ros_cost == costmap_2d::NO_INFORMATION && !allow_unknown_local) {
          return COST_OBS; // Treat unknown as obstacle if not allowed
      }

      // Scale other costs (0 to 252)
      // NavFn uses: v = COST_NEUTRAL + COST_FACTOR * v;
      // where v is the incoming ros_cost. COST_NEUTRAL=50, COST_FACTOR=0.8
      unsigned char cost = COST_NEUTRAL + static_cast<unsigned char>(COST_FACTOR * ros_cost);
      if (cost >= COST_OBS) {
          cost = COST_OBS - 1;
      }
      return cost;
  }
  // END ADDED

};
