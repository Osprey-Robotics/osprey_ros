/** Copyright 2024 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>

using namespace gz;
using namespace sim;
using namespace systems;

// Inherit from System and 1 extra interface: ISystemConfigure
class GzRandomRocks
    : public System,
      public ISystemConfigure
{
    // Implement Configure callback, provided by ISystemConfigure
    // and called once at startup.
    virtual void Configure(const Entity &/*_entity*/,
                           const std::shared_ptr<const sdf::Element> &/*_sdf*/,
                           EntityComponentManager &_ecm,
                           EventManager &_eventManager) override
    {
        const int ROCKS = 7;
        int x_min = -750;
        int x_max = 750 - x_min;
        int x_range = x_max - x_min;
        int y_min = -2250;
        int y_max = 2250;
        int y_range = y_max - y_min;

        SdfEntityCreator sec = SdfEntityCreator(_ecm, _eventManager);
        srand(time(0));

        for( int i = 1 ; i <= ROCKS; i++ )
        {
            Entity rockEntity;
            sdf::Model rockModel;
            sdf::SDF rockSDF;
            std::stringstream ss;

            float x = ((rand() % x_range) + x_min) / 1000.0;
            float y = ((rand() % y_range) + y_min) / 1000.0;

            ss << "<?xml version='1.0'?>\
                <sdf version='1.7'>\
                <model name='rock" << i << "'>\
                    <pose>" << std::setw(5) << x << " " << std::setw(5) << y << " 0 0 0 0</pose>\
                    <link name='rock" << i << "'>\
                    <collision name='rock" << i << "_collision'>\
                        <geometry>\
                        <mesh>\
                            <uri>file:///opt/ros_ws/install/osprey_ros/share/osprey_ros/meshes/rocks/rock" << i << ".dae</uri>\
                            <scale>0.5 0.5 0.5</scale>\
                        </mesh>\
                        </geometry>\
                    </collision>\
                    <visual name='rock" << i << "_visual'>\
                        <pose>0 0 0 0 0 0</pose>\
                        <geometry>\
                        <mesh>\
                            <uri>file:///opt/ros_ws/install/osprey_ros/share/osprey_ros/meshes/rocks/rock" << i << ".dae</uri>\
                            <scale>0.5 0.5 0.5</scale>\
                        </mesh>\
                        </geometry>\
                        <meta>\
                        <layer>0</layer>\
                        </meta>\
                    </visual>\
                    </link>\
                    <static>1</static>\
                </model>\
                </sdf>";

            rockSDF.SetFromString(ss.str());
            rockModel.Load(rockSDF.Root()->GetElement("model"));
            rockEntity = sec.CreateEntities(&rockModel);
            sec.SetParent(rockEntity, 4);
        }
    }
};

// Register plugin
IGNITION_ADD_PLUGIN(GzRandomRocks,
                    gz::sim::System,
                    GzRandomRocks::ISystemConfigure)

// Add plugin alias so that we can refer to the plugin without the version namespace
IGNITION_ADD_PLUGIN_ALIAS(GzRandomRocks, "gz::sim::systems::GzRandomRocks")
