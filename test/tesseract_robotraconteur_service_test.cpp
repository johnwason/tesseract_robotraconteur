/**
 * @file tesseract_robotraconteur_service_test.cpp
 *
 * @brief Tests that start the real tesseract_robotraconteur_service executable and use it as a client
 *
 * @author John Wason, PhD
 *
 * @copyright Copyright 2026 Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * @par
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <set>
#include <thread>

#include <boost/uuid/uuid_generators.hpp>
#include <RobotRaconteurCompanion/Converters/EigenConverters.h>

#include "robotraconteur_generated.h"
#include "service_process.h"

namespace RR = RobotRaconteur;
namespace RRC_Eigen = RobotRaconteur::Companion::Converters::Eigen;
namespace rr_tesseract = experimental::tesseract_robotics;
namespace rr_env = experimental::tesseract_robotics::environment;
namespace rr_env_cmd = experimental::tesseract_robotics::environment::commands;
namespace rr_sg = experimental::tesseract_robotics::scene_graph;
namespace rr_tasks = experimental::tesseract_robotics::tasks;
namespace rr_common = experimental::tesseract_robotics::common;
namespace rr_command = experimental::tesseract_robotics::command_language;
namespace rr_shapes = com::robotraconteur::geometry::shapes;
namespace rr_action = com::robotraconteur::action;

using tesseract_robotraconteur::test::ServiceProcess;

namespace
{
const std::vector<std::string> ABB_JOINT_NAMES = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

// Starts the service once for all tests and connects a client to it over the local transport
class ServiceEnvironment : public ::testing::Environment
{
  public:
    static rr_tesseract::TesseractRoboticsPtr client;

    void SetUp() override
    {
        node_setup = std::make_unique<RR::ClientNodeSetup>(ROBOTRACONTEUR_SERVICE_TYPES);

        const char* resource_path = std::getenv("TESSERACT_RESOURCE_PATH");
        if (!resource_path || std::strlen(resource_path) == 0)
        {
            ServiceProcess::SetEnv("TESSERACT_RESOURCE_PATH", TESSERACT_RR_TEST_RESOURCE_PATH);
        }

        std::string nodename = "tesseract_rr_test_" + std::to_string(ServiceProcess::CurrentProcessId());
        process.Start(TESSERACT_RR_TEST_SERVICE_EXECUTABLE,
                      {std::string("--urdf-file=") + TESSERACT_RR_TEST_URDF_FILE,
                       std::string("--srdf-file=") + TESSERACT_RR_TEST_SRDF_FILE,
                       std::string("--task-plugin-config-file=") + TESSERACT_RR_TEST_TASK_PLUGIN_CONFIG_FILE,
                       "--robotraconteur-nodename=" + nodename, "--robotraconteur-tcp-enable=false"});

        std::string url = "rr+local:///?nodename=" + nodename + "&service=tesseract";
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
        while (!client)
        {
            ASSERT_TRUE(process.IsRunning()) << "Service exited before a client could connect";
            try
            {
                client = RR::rr_cast<rr_tesseract::TesseractRobotics>(RR::RobotRaconteurNode::s()->ConnectService(url));
            }
            catch (std::exception&)
            {
                ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "Timed out connecting to " << url;
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }
        }
    }

    void TearDown() override
    {
        if (client)
        {
            try
            {
                RR::RobotRaconteurNode::s()->DisconnectService(client);
            }
            catch (std::exception&)
            {}
            client.reset();
        }
        EXPECT_EQ(process.Stop(30000), 0) << "Service did not shut down cleanly";
        node_setup.reset();
    }

  private:
    ServiceProcess process;
    std::unique_ptr<RR::ClientNodeSetup> node_setup;
};

rr_tesseract::TesseractRoboticsPtr ServiceEnvironment::client;

// Each test gets its own copy of the default environment
rr_env::EnvironmentPtr LoadEnvironment(const std::string& name)
{
    auto c = ServiceEnvironment::client;
    EXPECT_EQ(c->load_environment("default", name), name);
    return c->get_environments(name);
}

com::robotraconteur::uuid::UUID NewRandomUuid()
{
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    com::robotraconteur::uuid::UUID ret;
    memcpy(ret.s.uuid_bytes.data(), &uuid, sizeof(uuid));
    return ret;
}

rr_sg::JointPtr MakeFixedJoint(const std::string& name, const std::string& parent, const std::string& child,
                               const Eigen::Isometry3d& origin)
{
    rr_sg::JointPtr joint(new rr_sg::Joint());
    joint->name = name;
    joint->joint_type = rr_sg::JointType::fixed;
    joint->axis = RRC_Eigen::ToVector3(Eigen::Vector3d::UnitZ());
    joint->parent_link_name = parent;
    joint->child_link_name = child;
    joint->parent_to_joint_origin_transform = RRC_Eigen::ToPose(origin);
    return joint;
}

rr_shapes::MaterialPtr MakeMaterial(double r, double g, double b, double a)
{
    rr_shapes::MaterialPtr material(new rr_shapes::Material());
    material->base_color_factor.s.r = static_cast<float>(r);
    material->base_color_factor.s.g = static_cast<float>(g);
    material->base_color_factor.s.b = static_cast<float>(b);
    material->base_color_factor.s.a = static_cast<float>(a);
    return material;
}

rr_sg::LinkPtr MakeLink(const std::string& name, const std::vector<RR::RRValuePtr>& geometries)
{
    rr_sg::LinkPtr link(new rr_sg::Link());
    link->name = name;
    link->visual = RR::AllocateEmptyRRList<rr_sg::Visual>();
    link->collision = RR::AllocateEmptyRRList<rr_sg::Collision>();
    for (size_t i = 0; i < geometries.size(); ++i)
    {
        rr_sg::VisualPtr visual(new rr_sg::Visual());
        visual->name = name + "_visual_" + std::to_string(i);
        visual->origin = RRC_Eigen::ToPose(Eigen::Isometry3d::Identity());
        visual->geometry = geometries[i];
        visual->material = MakeMaterial(0.1, 0.2, 0.3, 1.0);
        link->visual->push_back(visual);

        rr_sg::CollisionPtr collision(new rr_sg::Collision());
        collision->name = name + "_collision_" + std::to_string(i);
        collision->origin = RRC_Eigen::ToPose(Eigen::Isometry3d::Identity());
        collision->geometry = geometries[i];
        link->collision->push_back(collision);
    }
    return link;
}

rr_env_cmd::AddLinkCommandPtr MakeAddLinkCommand(const rr_sg::LinkPtr& link, const std::string& parent)
{
    rr_env_cmd::AddLinkCommandPtr cmd(new rr_env_cmd::AddLinkCommand());
    cmd->link = link;
    cmd->joint = MakeFixedJoint(link->name + "_joint", parent, link->name,
                                Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 2, 3));
    cmd->replace_allowed = false;
    return cmd;
}

std::vector<std::string> ToStringVector(const RR::RRListPtr<RR::RRArray<char>>& list)
{
    return RR::RRListToStringVector(list);
}

// RRList is a linked list, so it has no at()
template <typename T>
boost::intrusive_ptr<T> ListAt(const RR::RRListPtr<T>& list, size_t i)
{
    EXPECT_LT(i, list->size());
    return *std::next(list->begin(), static_cast<std::ptrdiff_t>(i));
}

bool Contains(const std::vector<std::string>& v, const std::string& s)
{
    return std::find(v.begin(), v.end(), s) != v.end();
}

std::string GetKey(const RR::RRMapPtr<std::string, rr_tasks::TaskComposerKey>& keys, const std::string& port)
{
    return RR::RRArrayToString(RR::rr_cast<RR::RRArray<char>>(keys->at(port)->keys));
}

rr_command::MoveInstructionPtr MakeFreespaceMove(const Eigen::Isometry3d& pose)
{
    rr_command::CartesianWaypointPtr wp(new rr_command::CartesianWaypoint());
    wp->transform = RRC_Eigen::ToTransform(pose);
    wp->lower_tolerance = RR::AllocateRRArray<double>(0);
    wp->upper_tolerance = RR::AllocateRRArray<double>(0);

    rr_command::MoveInstructionPtr move(new rr_command::MoveInstruction());
    move->waypoint = wp;
    move->profile = "DEFAULT";
    move->move_type = rr_command::MoveInstructionType::freespace;
    move->uuid = NewRandomUuid();
    return move;
}

rr_tasks::TaskExecutorInputPtr MakeFreespaceInput(const std::string& env_name, std::string& output_key)
{
    auto c = ServiceEnvironment::client;
    auto task_info = c->get_tasks_info()->at("FreespacePipeline");
    output_key = GetKey(task_info->output_keys, "program");
    std::string input_key = GetKey(task_info->input_keys, "planning_input");

    rr_env::EnvironmentHandlePtr env_handle(new rr_env::EnvironmentHandle());
    env_handle->name = env_name;

    rr_common::ManipulatorInfoPtr manip_info(new rr_common::ManipulatorInfo());
    manip_info->manipulator = "manipulator";
    manip_info->tcp_frame = "tool0";
    manip_info->working_frame = "base_link";

    Eigen::Quaterniond q(0.70710678, 0, 0.70710678, 0);
    rr_command::CompositeInstructionPtr program(new rr_command::CompositeInstruction());
    program->order = rr_command::CompositeInstructionOrder::ordered;
    program->instructions = RR::AllocateEmptyRRList<RR::RRValue>();
    program->instructions->push_back(
        MakeFreespaceMove(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.9, -0.3, 1.455) * q));
    program->instructions->push_back(
        MakeFreespaceMove(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.9, 0.3, 1.455) * q));
    program->profile = "DEFAULT";
    program->manipulator_info = manip_info;
    program->uuid = NewRandomUuid();

    rr_tasks::TaskExecutorInputPtr input(new rr_tasks::TaskExecutorInput());
    input->task_name = "FreespacePipeline";
    input->data_storage = RR::AllocateEmptyRRMap<std::string, RR::RRValue>();
    input->data_storage->insert(std::make_pair("environment", env_handle));
    input->data_storage->insert(std::make_pair(input_key, program));
    return input;
}

rr_tasks::TaskExecutorPtr GetDefaultExecutor()
{
    auto c = ServiceEnvironment::client;
    auto executors = c->get_task_executors_info();
    return c->get_task_executors(executors->begin()->first);
}

} // namespace

TEST(TesseractRoboticsService, Info)
{
    auto c = ServiceEnvironment::client;

    auto tasks = c->get_tasks_info();
    ASSERT_TRUE(tasks);
    ASSERT_EQ(tasks->count("FreespacePipeline"), 1U);
    auto freespace = tasks->at("FreespacePipeline");
    EXPECT_EQ(freespace->name, "FreespacePipeline");
    EXPECT_FALSE(freespace->uuid.empty());
    EXPECT_EQ(freespace->input_keys->count("planning_input"), 1U);
    EXPECT_EQ(freespace->output_keys->count("program"), 1U);

    auto executors = c->get_task_executors_info();
    ASSERT_TRUE(executors);
    ASSERT_FALSE(executors->empty());
    for (const auto& e : *executors)
    {
        EXPECT_EQ(e.first, e.second->name);
    }
}

TEST(TesseractRoboticsService, LoadAndCloneEnvironment)
{
    auto c = ServiceEnvironment::client;
    LoadEnvironment("load_env");
    EXPECT_EQ(c->clone_environment("load_env", "load_env_clone"), "load_env_clone");

    auto info = c->get_environments_info();
    ASSERT_EQ(info->count("load_env"), 1U);
    ASSERT_EQ(info->count("load_env_clone"), 1U);
    EXPECT_EQ(info->at("load_env")->name, "load_env");

    EXPECT_EQ(c->get_environments("load_env")->get_name(), "load_env");
    EXPECT_EQ(c->get_environments("load_env_clone")->get_name(), "load_env_clone");
}

TEST(TesseractRoboticsService, InvalidRequests)
{
    auto c = ServiceEnvironment::client;
    EXPECT_THROW(c->load_environment("not_a_resource", "invalid_env"), RR::InvalidArgumentException);
    EXPECT_THROW(c->clone_environment("not_an_environment", "invalid_env"), RR::InvalidArgumentException);
    EXPECT_THROW(c->get_environments("not_an_environment"), RR::InvalidArgumentException);
    EXPECT_THROW(c->get_task_executors("not_an_executor"), RR::InvalidArgumentException);
}

TEST(TesseractRoboticsService, EnvironmentQueries)
{
    auto env = LoadEnvironment("query_env");

    auto joint_names = ToStringVector(env->get_joint_names());
    for (const auto& j : ABB_JOINT_NAMES)
    {
        EXPECT_TRUE(Contains(joint_names, j)) << j;
    }
    auto link_names = ToStringVector(env->get_link_names());
    for (const auto& l : {"base_link", "link_1", "link_6", "tool0"})
    {
        EXPECT_TRUE(Contains(link_names, l)) << l;
    }

    auto joint = env->getf_joint("joint_1");
    ASSERT_TRUE(joint);
    EXPECT_EQ(joint->name, "joint_1");
    EXPECT_EQ(joint->joint_type, rr_sg::JointType::revolute);
    EXPECT_EQ(joint->parent_link_name, "base_link");
    EXPECT_EQ(joint->child_link_name, "link_1");
    EXPECT_TRUE(RRC_Eigen::ToVector(joint->axis).isApprox(Eigen::Vector3d::UnitZ()));
    ASSERT_TRUE(joint->limits);
    EXPECT_NEAR(joint->limits->lower, -3.1416, 1e-6);
    EXPECT_NEAR(joint->limits->upper, 3.1416, 1e-6);
    EXPECT_NEAR(joint->limits->velocity, 2.618, 1e-6);

    // Visual meshes are loaded from package:// resources and converted to Robot Raconteur meshes
    auto link = env->getf_link("link_1");
    ASSERT_TRUE(link);
    EXPECT_EQ(link->name, "link_1");
    ASSERT_TRUE(link->visual);
    ASSERT_FALSE(link->visual->empty());
    std::set<std::string> visual_names;
    for (const auto& visual : *link->visual)
    {
        EXPECT_TRUE(visual_names.insert(visual->name).second) << "Duplicate visual name " << visual->name;
        auto mesh = RR::rr_cast<rr_shapes::Mesh>(visual->geometry);
        ASSERT_TRUE(mesh);
        EXPECT_GT(mesh->triangles->size(), 0U);
        EXPECT_GT(mesh->vertices->size(), 0U);
        for (size_t i = 0; i < mesh->triangles->size(); ++i)
        {
            const auto& t = mesh->triangles->at(i);
            ASSERT_LT(t.s.v1, mesh->vertices->size());
            ASSERT_LT(t.s.v2, mesh->vertices->size());
            ASSERT_LT(t.s.v3, mesh->vertices->size());
        }
    }
    ASSERT_TRUE(link->collision);
    EXPECT_FALSE(link->collision->empty());

    EXPECT_THROW(env->getf_joint("not_a_joint"), RR::InvalidArgumentException);
    EXPECT_THROW(env->getf_link("not_a_link"), RR::InvalidArgumentException);
    // The service is still running after the invalid requests
    EXPECT_EQ(env->get_name(), "query_env");
}

TEST(TesseractRoboticsService, EnvironmentState)
{
    auto env = LoadEnvironment("state_env");
    auto names = RR::stringVectorToRRList(ABB_JOINT_NAMES);

    auto zero = RRC_Eigen::RRArrayToEigen<Eigen::VectorXd>(env->getf_current_joint_values(names));
    EXPECT_TRUE(zero.isZero());

    std::vector<double> target = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
    auto joints = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double>>();
    for (size_t i = 0; i < ABB_JOINT_NAMES.size(); ++i)
    {
        joints->insert(std::make_pair(ABB_JOINT_NAMES[i], RR::ScalarToRRArray(target[i])));
    }
    env->setf_state(joints, nullptr);

    auto values = env->getf_current_joint_values(names);
    ASSERT_EQ(values->size(), target.size());
    for (size_t i = 0; i < target.size(); ++i)
    {
        EXPECT_NEAR((*values)[i], target[i], 1e-9);
    }

    auto state = env->getf_state_all();
    ASSERT_TRUE(state);
    EXPECT_NEAR(RR::RRArrayToScalar(state->joints->at("joint_3")), 0.3, 1e-9);
    EXPECT_EQ(state->link_transforms->count("tool0"), 1U);
    EXPECT_EQ(state->joint_transforms->count("joint_6"), 1U);

    // Setting a state in one environment does not change another
    auto other = LoadEnvironment("state_env_other");
    auto other_values = RRC_Eigen::RRArrayToEigen<Eigen::VectorXd>(other->getf_current_joint_values(names));
    EXPECT_TRUE(other_values.isZero());
}

TEST(TesseractRoboticsService, AddLinkPrimitiveGeometry)
{
    auto env = LoadEnvironment("primitive_env");

    rr_shapes::BoxPtr box(new rr_shapes::Box());
    box->x = 0.1;
    box->y = 0.2;
    box->z = 0.3;
    rr_shapes::SpherePtr sphere(new rr_shapes::Sphere());
    sphere->radius = 0.4;
    rr_shapes::CylinderPtr cylinder(new rr_shapes::Cylinder());
    cylinder->radius = 0.5;
    cylinder->height = 0.6;
    rr_shapes::ConePtr cone(new rr_shapes::Cone());
    cone->radius = 0.7;
    cone->height = 0.8;
    rr_shapes::CapsulePtr capsule(new rr_shapes::Capsule());
    capsule->radius = 0.9;
    capsule->height = 1.0;

    auto link = MakeLink("primitive_link", {box, sphere, cylinder, cone, capsule});
    EXPECT_EQ(env->apply_command(MakeAddLinkCommand(link, "base_link")), 1);

    EXPECT_TRUE(Contains(ToStringVector(env->get_link_names()), "primitive_link"));
    auto joint = env->getf_joint("primitive_link_joint");
    EXPECT_EQ(joint->joint_type, rr_sg::JointType::fixed);
    EXPECT_TRUE(RRC_Eigen::ToIsometry(joint->parent_to_joint_origin_transform)
                    .isApprox(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 2, 3)));

    auto result = env->getf_link("primitive_link");
    ASSERT_EQ(result->visual->size(), 5U);
    ASSERT_EQ(result->collision->size(), 5U);

    auto r_box = RR::rr_cast<rr_shapes::Box>(ListAt(result->visual, 0)->geometry);
    EXPECT_DOUBLE_EQ(r_box->x, 0.1);
    EXPECT_DOUBLE_EQ(r_box->y, 0.2);
    EXPECT_DOUBLE_EQ(r_box->z, 0.3);
    EXPECT_DOUBLE_EQ(RR::rr_cast<rr_shapes::Sphere>(ListAt(result->visual, 1)->geometry)->radius, 0.4);
    auto r_cylinder = RR::rr_cast<rr_shapes::Cylinder>(ListAt(result->visual, 2)->geometry);
    EXPECT_DOUBLE_EQ(r_cylinder->radius, 0.5);
    EXPECT_DOUBLE_EQ(r_cylinder->height, 0.6);
    auto r_cone = RR::rr_cast<rr_shapes::Cone>(ListAt(result->visual, 3)->geometry);
    EXPECT_DOUBLE_EQ(r_cone->radius, 0.7);
    EXPECT_DOUBLE_EQ(r_cone->height, 0.8);
    auto r_capsule = RR::rr_cast<rr_shapes::Capsule>(ListAt(result->visual, 4)->geometry);
    EXPECT_DOUBLE_EQ(r_capsule->radius, 0.9);
    EXPECT_DOUBLE_EQ(r_capsule->height, 1.0);

    auto material = ListAt(result->visual, 0)->material;
    ASSERT_TRUE(material);
    EXPECT_FLOAT_EQ(material->base_color_factor.s.r, 0.1F);
    EXPECT_FLOAT_EQ(material->base_color_factor.s.g, 0.2F);
    EXPECT_FLOAT_EQ(material->base_color_factor.s.b, 0.3F);
    EXPECT_FLOAT_EQ(material->base_color_factor.s.a, 1.0F);

    // Adding the same link again is rejected
    EXPECT_FALSE(env->apply_command(MakeAddLinkCommand(link, "base_link")));
}

TEST(TesseractRoboticsService, AddLinkMesh)
{
    auto env = LoadEnvironment("mesh_env");

    // Unit square made of two triangles
    rr_shapes::MeshPtr mesh(new rr_shapes::Mesh());
    mesh->vertices = RR::AllocateEmptyRRNamedArray<com::robotraconteur::geometry::Point>(4);
    const double xy[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    for (size_t i = 0; i < 4; ++i)
    {
        mesh->vertices->at(i).s.x = xy[i][0];
        mesh->vertices->at(i).s.y = xy[i][1];
        mesh->vertices->at(i).s.z = 0.5;
    }
    mesh->triangles = RR::AllocateEmptyRRNamedArray<rr_shapes::MeshTriangle>(2);
    const uint32_t tris[2][3] = {{0, 1, 2}, {0, 2, 3}};
    for (size_t i = 0; i < 2; ++i)
    {
        mesh->triangles->at(i).s.v1 = tris[i][0];
        mesh->triangles->at(i).s.v2 = tris[i][1];
        mesh->triangles->at(i).s.v3 = tris[i][2];
    }
    // Robot Raconteur arrays must not be null
    mesh->normals = RR::AllocateEmptyRRNamedArray<com::robotraconteur::geometry::Vector3>(0);
    mesh->colors = RR::AllocateEmptyRRNamedArray<com::robotraconteur::color::ColorRGB>(0);
    mesh->mesh_type = rr_shapes::MeshType::mesh;

    auto link = MakeLink("mesh_link", {mesh});
    EXPECT_EQ(env->apply_command(MakeAddLinkCommand(link, "tool0")), 1);

    auto result = env->getf_link("mesh_link");
    ASSERT_EQ(result->visual->size(), 1U);
    auto r_mesh = RR::rr_cast<rr_shapes::Mesh>(ListAt(result->visual, 0)->geometry);
    ASSERT_EQ(r_mesh->vertices->size(), 4U);
    ASSERT_EQ(r_mesh->triangles->size(), 2U);
    EXPECT_EQ(r_mesh->mesh_type, rr_shapes::MeshType::mesh);
    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_DOUBLE_EQ(r_mesh->vertices->at(i).s.x, xy[i][0]);
        EXPECT_DOUBLE_EQ(r_mesh->vertices->at(i).s.y, xy[i][1]);
        EXPECT_DOUBLE_EQ(r_mesh->vertices->at(i).s.z, 0.5);
    }
    for (size_t i = 0; i < 2; ++i)
    {
        EXPECT_EQ(r_mesh->triangles->at(i).s.v1, tris[i][0]);
        EXPECT_EQ(r_mesh->triangles->at(i).s.v2, tris[i][1]);
        EXPECT_EQ(r_mesh->triangles->at(i).s.v3, tris[i][2]);
    }
}

TEST(TesseractRoboticsService, ApplyCommandsAndHistory)
{
    auto env = LoadEnvironment("commands_env");
    auto initial_history = env->getf_command_history();
    ASSERT_TRUE(initial_history);
    ASSERT_TRUE(initial_history->commands);
    size_t initial_count = initial_history->commands->size();
    EXPECT_GT(initial_count, 0U);

    rr_env_cmd::ChangeJointOriginCommandPtr change_origin(new rr_env_cmd::ChangeJointOriginCommand());
    change_origin->joint_name = "joint_1";
    Eigen::Isometry3d new_origin = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.25);
    change_origin->origin = RRC_Eigen::ToPose(new_origin);

    rr_env_cmd::RemoveLinkCommandPtr remove_link(new rr_env_cmd::RemoveLinkCommand());
    remove_link->link_name = "added_link";

    rr_env_cmd::CommandsPtr commands(new rr_env_cmd::Commands());
    commands->commands = RR::AllocateEmptyRRList<RR::RRValue>();
    commands->commands->push_back(MakeAddLinkCommand(MakeLink("added_link", {}), "base_link"));
    commands->commands->push_back(change_origin);
    commands->commands->push_back(remove_link);
    EXPECT_EQ(env->apply_commands(commands), 1);

    EXPECT_FALSE(Contains(ToStringVector(env->get_link_names()), "added_link"));
    EXPECT_TRUE(
        RRC_Eigen::ToIsometry(env->getf_joint("joint_1")->parent_to_joint_origin_transform).isApprox(new_origin));

    auto history = env->getf_command_history();
    ASSERT_EQ(history->commands->size(), initial_count + 3);
    EXPECT_EQ(ListAt(history->commands, initial_count)->RRType(),
              "experimental.tesseract_robotics.environment.commands.AddLinkCommand");
    EXPECT_EQ(ListAt(history->commands, initial_count + 1)->RRType(),
              "experimental.tesseract_robotics.environment.commands.ChangeJointOriginCommand");
    EXPECT_EQ(ListAt(history->commands, initial_count + 2)->RRType(),
              "experimental.tesseract_robotics.environment.commands.RemoveLinkCommand");

    // The command history can be replayed into another environment
    auto replay = LoadEnvironment("commands_env_replay");
    rr_env_cmd::CommandsPtr replay_commands(new rr_env_cmd::Commands());
    replay_commands->commands = RR::AllocateEmptyRRList<RR::RRValue>();
    for (size_t i = initial_count; i < history->commands->size(); ++i)
    {
        replay_commands->commands->push_back(ListAt(history->commands, i));
    }
    EXPECT_EQ(replay->apply_commands(replay_commands), 1);
    EXPECT_TRUE(
        RRC_Eigen::ToIsometry(replay->getf_joint("joint_1")->parent_to_joint_origin_transform).isApprox(new_origin));
}

TEST(TesseractRoboticsService, AppliedCommandsPipe)
{
    auto env = LoadEnvironment("pipe_env");
    auto endpoint = env->get_applied_commands()->Connect(-1);

    EXPECT_EQ(env->apply_command(MakeAddLinkCommand(MakeLink("pipe_link", {}), "base_link")), 1);

    rr_env_cmd::CommandsPtr received;
    ASSERT_TRUE(endpoint->TryReceivePacketWait(received, 10000)) << "No applied commands packet received";
    ASSERT_TRUE(received);
    // Tesseract reports the full command history, ending with the command that was just applied
    auto history = env->getf_command_history();
    ASSERT_EQ(received->commands->size(), history->commands->size());
    auto add_link = RR::rr_cast<rr_env_cmd::AddLinkCommand>(ListAt(received->commands, received->commands->size() - 1));
    EXPECT_EQ(add_link->link->name, "pipe_link");

    endpoint->Close();
}

TEST(TesseractRoboticsService, FreespacePlan)
{
    LoadEnvironment("plan_env");
    std::string output_key;
    auto input = MakeFreespaceInput("plan_env", output_key);

    auto gen = GetDefaultExecutor()->run(input);
    rr_tasks::TaskExecutorStatusPtr status;
    while (gen->TryNext(status))
    {}

    ASSERT_TRUE(status);
    ASSERT_EQ(status->action_status, rr_action::ActionStatusCode::complete);
    ASSERT_EQ(status->data_storage->count(output_key), 1U);
    auto program = RR::rr_cast<rr_command::CompositeInstruction>(status->data_storage->at(output_key));
    ASSERT_TRUE(program);
    ASSERT_GT(program->instructions->size(), 2U);

    for (const auto& instr : *program->instructions)
    {
        auto move = RR::rr_cast<rr_command::MoveInstruction>(instr);
        ASSERT_TRUE(move);
        auto wp = RR::rr_cast<rr_command::StateWaypoint>(move->waypoint);
        ASSERT_TRUE(wp);
        EXPECT_EQ(ToStringVector(wp->joint_names), ABB_JOINT_NAMES);
        EXPECT_EQ(wp->position->size(), 6U);
    }
}

TEST(TesseractRoboticsService, AbortPlan)
{
    LoadEnvironment("abort_env");
    std::string output_key;
    auto input = MakeFreespaceInput("abort_env", output_key);

    auto gen = GetDefaultExecutor()->run(input);
    rr_tasks::TaskExecutorStatusPtr status;
    gen->TryNext(status);
    try
    {
        gen->Abort();
    }
    catch (RR::StopIterationException&)
    {
        // The plan finished before the abort arrived
    }

    // The service stays usable after an abort
    auto c = ServiceEnvironment::client;
    EXPECT_FALSE(c->get_tasks_info()->empty());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new ServiceEnvironment());
    return RUN_ALL_TESTS();
}
