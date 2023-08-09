/*********************************************************************
 * Copyright (c) 2019 Bielefeld University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke
   Desc:   Planning a simple sequence of Cartesian motions
*/

/*
	modified for UR5e by RoboCop 09.08.2023
*/

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace moveit::task_constructor;

Task createTask() {
	Task t;
	t.stages()->setName("Cartesian Path");

	const std::string group = "manipulator";
	const std::string eef = "endeffector";

	// create Cartesian interpolation "planner" to be used in various stages
	auto cartesian_interpolation = std::make_shared<solvers::CartesianPath>();
	// cartesian_interpolation->setMaxAccelerationScaling(0.6);
	// create a joint-space interpolation "planner" to be used in various stages
	auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

	// start from a fixed robot state
	t.loadRobotModel();
	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());

	{
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues(state.getJointModelGroup(group), "home");
		
		auto fixed = std::make_unique<stages::FixedState>("initial state");
		fixed->setState(scene);
		t.add(std::move(fixed));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "base_link";
		direction.vector.x = 0.2;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("y +0.1", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "base_link";
		direction.vector.y = +0.1;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}

	{  // rotate about TCP
		auto stage = std::make_unique<stages::MoveRelative>("rz +45°", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = "base_link";
		twist.twist.angular.z = M_PI / 4.;
		stage->setDirection(twist);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("x -0.1, y +0.1, z -0.1", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "base_link";
		direction.vector.x = -0.1;
		direction.vector.y = +0.1;
		direction.vector.y = -0.1;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}

	{  // perform a Joint motion, defined as a relative offset in joint space
		auto stage = std::make_unique<stages::MoveRelative>("joint move ", joint_interpolation);
		stage->setGroup(group);
		std::map<std::string, double> offsets = { { "elbow_joint", M_PI / 12 },
			                                       { "shoulder_pan_joint", -M_PI / 12 },
			                                       { "wrist_1_joint", 0.1 } };
		stage->setDirection(offsets);
		t.add(std::move(stage));
	}

	{  // perform a Cartesian motion, defined as a relative offset in joint space
		auto stage = std::make_unique<stages::MoveRelative>("joint offset", cartesian_interpolation);
		stage->setGroup(group);
		std::map<std::string, double> offsets = { { "wrist_1_joint", 0.01 }, { "elbow_joint", -0.01 } };
		stage->setDirection(offsets);
		t.add(std::move(stage));
	}

	{  // move from reached state back to the original state, using joint interpolation
		// specifying two groups (arm and hand) will try to merge both trajectories
		stages::Connect::GroupPlannerVector planners = { { group, joint_interpolation }, { eef, joint_interpolation } };
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		t.add(std::move(connect));
	}

	{  // final state is original state again
		auto fixed = std::make_unique<stages::FixedState>("final state");
		fixed->setState(scene);
		t.add(std::move(fixed));
	}

	return t;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");
	// run an asynchronous spinner to communicate with the move_group node and rviz
	ros::AsyncSpinner spinner(1);
	spinner.start();

	auto task = createTask();
	try {
		if (task.plan()) {
			task.introspection().publishSolution(*task.solutions().front());
			ROS_INFO("Planen erfolgreich");
			task.execute(*task.solutions().front());
			ROS_INFO("verfahren");
		}

	} catch (const InitStageException& ex) {
		std::cerr << "planning failed with exception" << std::endl << ex << task;
	}

	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}
