#include <gtest/gtest.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include <sdf/sdf.hh>

#include "mid360_simulation/mid360_ode_multiray_shape.h"

TEST(Mid360OdeMultiRayShape, KeepsRobotCollisionSpacesUnchanged)
{
    ASSERT_TRUE(gazebo::setupServer());
    sdf::SDFPtr description(new sdf::SDF);
    sdf::init(description);
    ASSERT_TRUE(sdf::readString(R"(
        <sdf version="1.6">
          <world name="mid360_test">
            <physics name="physics" type="ode"/>
            <model name="robot">
              <static>true</static>
              <link name="base">
                <collision name="body">
                  <geometry><box><size>1 1 1</size></box></geometry>
                </collision>
              </link>
            </model>
          </world>
        </sdf>)", description));
    auto world = gazebo::physics::create_world("mid360_test");
    world->Load(description->Root()->GetElement("world"));
    world->Init();
    auto link = boost::dynamic_pointer_cast<gazebo::physics::ODELink>(
        world->ModelByName("robot")->GetLink("base"));
    ASSERT_TRUE(link);
    auto collision = world->Physics()->CreateCollision("multiray", link);
    auto odeCollision = boost::dynamic_pointer_cast<gazebo::physics::ODECollision>(collision);
    const auto linkSpace = link->GetSpaceId();
    const auto collisionSpace = odeCollision->GetSpaceId();

    {
        gazebo::physics::Mid360OdeMultiRayShape shape(collision);
        EXPECT_EQ(linkSpace, link->GetSpaceId());
        EXPECT_EQ(collisionSpace, odeCollision->GetSpaceId());
        // Keep teardown safe even when the space-isolation assertions fail.
        link->SetSpaceId(linkSpace);
        odeCollision->SetSpaceId(collisionSpace);
    }

    collision.reset();
    odeCollision.reset();
    link.reset();
    world->Fini();
    world.reset();
    gazebo::shutdown();
}
