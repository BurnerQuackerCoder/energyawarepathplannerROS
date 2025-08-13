#ifndef ENERGY_ZONE_LAYER_H_
#define ENERGY_ZONE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <sensor_msgs/BatteryState.h>
#include <vector>
#include <string>

namespace enawpl
{

// Struct to hold the definition of a single zone
struct ZoneDefinition
{
    std::string name;
    unsigned int min_mx;
    unsigned int max_mx;
    unsigned int min_my;
    unsigned int max_my;
    double energy_usage_factor;
};

class EnergyZoneLayer : public costmap_2d::Layer
{
public:
    EnergyZoneLayer();
    virtual ~EnergyZoneLayer() {}

    // Methods called by the costmap_2d framework
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
    // Callback for the battery subscriber
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

    // Helper method to load zone definitions from the parameter server
    void loadZoneDefinitions();

    ros::NodeHandle nh_;
    ros::Subscriber battery_sub_;

    std::vector<ZoneDefinition> zone_definitions_;
    bool is_battery_low_;
    double critical_battery_threshold_;
    unsigned char zone_cost_value_; // The cost to "paint" the zone with

    bool initialized_; // To prevent updates before initialization is complete
};

} // end namespace

#endif