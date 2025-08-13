#include <enawpl/energy_zone_layer.h>
#include <pluginlib/class_list_macros.h>

// This exports this class as a costmap_2d::Layer plugin
PLUGINLIB_EXPORT_CLASS(enawpl::EnergyZoneLayer, costmap_2d::Layer)

namespace enawpl
{

EnergyZoneLayer::EnergyZoneLayer() : initialized_(false), is_battery_low_(false) {}

void EnergyZoneLayer::onInitialize()
{
    ros::NodeHandle private_nh("~/" + name_); // Use the layer's name as a namespace for parameters
    nh_ = ros::NodeHandle();

    // Get parameters
    private_nh.param("critical_battery_threshold", critical_battery_threshold_, 30.0);
    int zone_cost_int;
    private_nh.param("zone_cost_value", zone_cost_int, 200); // Cost to paint (0-252)
    zone_cost_value_ = static_cast<unsigned char>(zone_cost_int);

    // Load the zone definitions from the parameter server
    loadZoneDefinitions();

    // Subscribe to the battery topic
    battery_sub_ = nh_.subscribe("/battery", 1, &EnergyZoneLayer::batteryCallback, this);

    // We are now initialized
    initialized_ = true;
}

void EnergyZoneLayer::loadZoneDefinitions()
{
    ros::NodeHandle private_nh("~/" + name_);
    XmlRpc::XmlRpcValue zone_list;

    if (private_nh.getParam("zones", zone_list) && zone_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        for (int i = 0; i < zone_list.size(); ++i)
        {
            if (zone_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                ZoneDefinition zone;
                zone.name = static_cast<std::string>(zone_list[i]["name"]);
                zone.min_mx = static_cast<int>(zone_list[i]["min_mx"]);
                zone.max_mx = static_cast<int>(zone_list[i]["max_mx"]);
                zone.min_my = static_cast<int>(zone_list[i]["min_my"]);
                zone.max_my = static_cast<int>(zone_list[i]["max_my"]);
                zone_definitions_.push_back(zone);
            }
        }
    }
}

void EnergyZoneLayer::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    if (!initialized_) return;

    bool new_is_low = (msg->percentage * 100.0) < critical_battery_threshold_;

    // Only update if the state changes to avoid unnecessary work
    if (new_is_low != is_battery_low_)
    {
        is_battery_low_ = new_is_low;
        // The costmap will be updated on the next cycle, no need to force it here
    }
}

void EnergyZoneLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!initialized_ || !is_battery_low_)
    {
        return; // Don't update bounds if not active
    }

    // When the battery is low, we want to update the full area of all zones
    // to ensure their costs are applied.
    for (const auto& zone : zone_definitions_)
    {
        // Convert cell coordinates of zone corners to world coordinates
        double zone_min_wx, zone_min_wy, zone_max_wx, zone_max_wy;
        mapToWorld(zone.min_mx, zone.min_my, zone_min_wx, zone_min_wy);
        mapToWorld(zone.max_mx, zone.max_my, zone_max_wx, zone_max_wy);

        *min_x = std::min(*min_x, zone_min_wx);
        *min_y = std.min(*min_y, zone_min_wy);
        *max_x = std::max(*max_x, zone_max_wx);
        *max_y = std::max(*max_y, zone_max_wy);
    }
}

void EnergyZoneLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!initialized_ || !is_battery_low_)
    {
        return; // Don't apply costs if battery is normal or not initialized
    }

    // "Paint" the cost for each defined zone onto the master grid
    for (const auto& zone : zone_definitions_)
    {
        for (unsigned int mx = zone.min_mx; mx <= zone.max_mx; ++mx)
        {
            for (unsigned int my = zone.min_my; my <= zone.max_my; ++my)
            {
                // Set the cost of the cell, ensuring not to overwrite lethal obstacles
                if (master_grid.getCost(mx, my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                {
                    master_grid.setCost(mx, my, zone_cost_value_);
                }
            }
        }
    }
}

} // end namespace