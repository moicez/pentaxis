/**
 * @file PentaxisArmIKSolver.h
 * @author Rasmus Hasle Andersen, Moises Estrada Casta/neda <moises@gmail.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @brief ROS/KDL based interface for the inverse kinematics of the Pentaxis arm
 *
 * @section DESCRIPTION
 *
 * This class provides a KDL based interface to the inverse kinematics of the Pentaxis arm. It inherits from the KDL::ChainIkSolverPos class
 * but also exposes additional functionality to return multiple solutions from an inverse kinematics computation.
 * 
 */

#ifndef PentaxisArmIKSolverH_
#define PentaxisArmIKSolverH_

#include <urdf/model.h>
#include <Eigen/Core>
#include <kdl/chainiksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/PositionIKRequest.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_kdl.h>

#include "PentaxisArmIK.h"

namespace pentaxis_kinematics
{

static const int NO_IK_SOLUTION = -1;
static const int TIMED_OUT = -2;

class PentaxisArmIKSolver : public KDL::ChainIkSolverPos
{

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PentaxisArmIKSolver(PentaxisArmIK &ik,
                    const urdf::ModelInterface &robot_model,
                    const std::string &root_frame_name,
                    const std::string &tip_frame_name,
                    const double &search_discretization_angle,
                    const int &free_angle);

  ~PentaxisArmIKSolver(){};

  /**
   * @brief The Pentaxis inverse kinematics solver
   */
  PentaxisArmIK &pentaxis_arm_ik_;

  /**
   * @brief Indicates whether the solver has been successfully initialized
   */
  bool active_;

  int CartToJnt(const KDL::JntArray& q_init,
                const KDL::Frame& p_in,
                KDL::JntArray& q_out);

  int CartToJntSearch(const KDL::JntArray& q_in,
                      const KDL::Frame& p_in,
                      KDL::JntArray &q_out,
                      const double &timeout);

  void getSolverInfo(moveit_msgs::KinematicSolverInfo &response)
  {
    pentaxis_arm_ik_.getSolverInfo(response);
  }

private:

  bool getCount(int &count, const int &max_count, const int &min_count);

  double search_discretization_angle_;

  int free_angle_;

  std::string root_frame_name_;
  moveit_msgs::KinematicSolverInfo _solver_info;
};

} /* namespace pentaxis_kinematics */

#endif /* PentaxisArmIKSolverH_ */
