/**
 * @file PentaxisArmAnalyticalIK.h
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
 * @section DESCRIPTION
 *
 * This is the base class for the Inverse Analytical calculation.
 * An analytical IK solver for the Pentaxis arm.
 */

#ifndef PENTAXISARMANALYTICALIK_H_
#define PENTAXISARMANALYTICALIK_H_

#include "PentaxisArmIK.h"

namespace pentaxis_kinematics
{

class PentaxisArmAnalyticalIK: public PentaxisArmIK
{
public:
  /**
   * Ctor.
   *
   * @brief Initialize the solver by providing a urdf::Model and a root and tip name.
   * @param robot_model A urdf::Model representation of the Pentaxis robot model
   * @param robot_description The XML string that of a urdf::Model which represents the Pentaxis robot model
   * @param root_name The root joint name of the arm
   * @param joint_name The tip joint name of the arm
   */
  PentaxisArmAnalyticalIK(const urdf::ModelInterface &robot_model,
    const std::string &robot_description,
    const std::string &root_name,
    const std::string &tip_name);

  /**
   * Dtor.
   */
  virtual ~PentaxisArmAnalyticalIK();

	
  /**
   * @see InverseKinematics::CartToJnt
   */
  int CartToJnt(const KDL::JntArray& q_init,
    const KDL::Frame& p_in,
    std::vector<KDL::JntArray>& q_out);

  /**
   * @see InverseKinematics::getSolverInfo
   */
  void getSolverInfo(moveit_msgs::KinematicSolverInfo &response);


private:
	
	/**
	*	@brief Sets the values of nx,ny,...,py,pz. Also calls solveJoint1 then solveJoint5
	* @param frame The Forward Kinematics
	*/
	void ik(const KDL::Frame& frame);
	
	/**
	* @brief Calculates the solutions for Joint 1
	* @param joint5 The value of Joint 5 (may be set to zero if unknown)
	* @param j5_set If true it will assume that Joint 5 has already been calculated.
	*/
	void solveJoint1(double joint5, bool j5_set);
	
	/**
	* @brief Calculates the solutions for Joint 5
	* @param joint1 The value of Joint 1 (may be set to zero if unknown)
	* @param j1_set If true it will assume that Joint 1 has already been calculated.
	*/
	void solveJoint5(double joint1, bool j1_set);
	
	/**
	* @brief Calculates the solutions for Joint 3
	* @param joint1 The value of Joint 1
	* @param joint5 The value of Joint 5
	*/
	void solveJoint3(double joint1, double joint5);
	
	/**
	* @brief Calculates the solutions for Joint 2
	* @param joint1 The value of Joint 1
	* @param joint5 The value of Joint 5
	* @param joint5 The value of Joint 3
	* @param _Gamma The values of @f$ \Gamma @f$, calculated in solveJoint3
	* @param _Omega The values of @f$ \Omega @f$, calculated in solveJoint2
	*/
	void solveJoint2(double joint1, double joint5, double joint3, std::vector<double> &_Gamma, std::vector<double> &_Omega);
	
	/**
	* @brief Calculates the solutions for Joint 4
	* @param joint1 The value of Joint 1
	* @param joint5 The value of Joint 5
	* @param joint3 The value of Joint 3
	* @param joint3 The value of Joint 2
	*/
	void solveJoint4(double joint1, double joint5, double joint3, double joint2);
	
	/**
	* @brief Adds an Inverse Kinematic Solution to the solutions vector
	* @param j1 The value of Joint 1
	* @param j2 The value of Joint 2
	* @param j3 The value of Joint 3
	* @param j4 The value of Joint 4
	* @param j5 The value of Joint 5
	*/
	void addNewSolution(double j1, double j2, double j3, double j4, double j5);

	/**
	* @brief This method encompass both push_back_ifUnique and isJointValid into one method
	* @param j_index The Joint Number (1-5)
	* @param j_element The numeric value of the Joint
	* @param j_vector The vector that contains other Joint values previouly obtained
	* @return True if both called methods returned true.
	*/
	bool validateJoint(int j_index, double j_element, std::vector<double> &j_vector);
	
	/**
	* @brief Tell if the IK corresponse to the given FK
	*
	*	@param joint1 the value of Joint 1 to be tested.
	*	@param joint5 the value of Joint 5 to be tested.
	*	@param joint3 the value of Joint 3 to be tested.
	*	@param joint2 the value of Joint 2 to be tested.
	*	@param joint4 the value of Joint 4 to be tested.
	* @return True if it's a valid solution value else false.
	*/
	bool validateSolutionWithFK(double joint1, double joint5, double joint3, double joint2, double joint4);

	/**
	* @brief Adds a new element to a vector if its unique.
	*	@param vect. The set of elements.
	*	@param newElement. The element to be tested.
	* @return True if a new element has been added in the vector.
	*/
	bool push_back_ifUnique(std::vector<double> &vect, double newElement);

	/**
	* @brief Tell if a single Joint value is inside of the joint limits.
	*	@param jointValue The joint value to be tested.
	* @return True if it's a valid joint value else False.
	*/
	bool isJointValid(double jointValue, int jointNum) const;



private:
  /**
   * Minimum joint limits.
   */
  std::vector<double> _min_angles;

  /**
   * Maximum joint limits.
   */
  std::vector<double> _max_angles;

  /**
   * Information about the IK solver.
   */
  moveit_msgs::KinematicSolverInfo _solver_info;
  
  /**
   * The FK given to solve it's IK.
   */
  double nx, ox, ax, px;
	double ny, oy, ay, py;
	double nz, oz, az, pz;
	
	/*
	* The Solutions found compiled in an Joint Array
	*/	
	std::vector<KDL::JntArray> solutions;
	
	/**
	* Denavit-Hartenberg Parameters for the Pentaxis robot.
	*/
	double d0;
	double d1;
	double d5;
	double d6; // 12.1
	double a2;
	double a3;
	
  
};

} /* namespace pentaxis_kinematics */

#endif /* PENTAXISARMANALYTICALIK_H_ */
