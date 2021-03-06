/**
 * @file PentaxisArmAnalyticalIK.cpp
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
 * This is the class where the actual inverse kinematics calculation take place
 */


#include "PentaxisArmAnalyticalIK.h"

// ROS includes
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <ros/console.h>
// Package includes
#include "../SolverInfoProcessor.h"


namespace pentaxis_kinematics
{



PentaxisArmAnalyticalIK::PentaxisArmAnalyticalIK(
    const urdf::ModelInterface &robot_model,
    const std::string &robot_description,
    const std::string &root_name,
    const std::string &tip_name)
{
	ROS_INFO("IK SOLVER STARTED");
	/*
	* Denavit-Hartenberg Parameters for the Pentaxis robot.
	*/
	d0 = 0.152;
	d1 = 0.259;
	d5 = 0.035;
	d6 = 0.0; // 12.1
	a2 = 0.252; // OG: 0.252
	a3 = 0.250; // TODO this should be somewhere else
  SolverInfoProcessor solver_info_processor(robot_model, tip_name, root_name);
  _solver_info = solver_info_processor.getSolverInfo();
  
  for (unsigned int i = 0; i < _solver_info.joint_names.size(); i++)
  {
    _min_angles.push_back(_solver_info.limits[i].min_position);
    _max_angles.push_back(_solver_info.limits[i].max_position);
  }
}

PentaxisArmAnalyticalIK::~PentaxisArmAnalyticalIK()
{
  // TODO Auto-generated destructor stub
}

int PentaxisArmAnalyticalIK::CartToJnt(const KDL::JntArray& q_init,
    const KDL::Frame& p_in,
    std::vector<KDL::JntArray>& q_out)
{

  // there are no solutions available yet
  q_out.clear();
	solutions.clear();
	ik(p_in);
  if(solutions.size() > 0) {
  	for(unsigned int i = 0; i < solutions.size(); i++) {
  		if(validateSolutionWithFK(solutions[i](0), solutions[i](1), solutions[i](2), solutions[i](3), solutions[i](4)))
  			q_out.push_back(solutions[i]);
  	}
  	if(q_out.size() > 0) {
    	ROS_INFO_STREAM("Inverse Kinematic found " <<  q_out.size() << " solutions.");
    	return 1;
    } else {
    	ROS_INFO_STREAM("Inverse Kinematics found a solution for:\n" <<nx<<"\t"<<ox<<"\t"<<ax<<"\t"<<px<<"\n"<<
	ny<<"\t"<< oy<<"\t"<< ay<<"\t"<< py<<"\n"<<
	nz<<"\t"<< oz<<"\t"<< az<<"\t"<< pz<< "\n But does not correspond to the forward kinematics.");
    }
  } else {
    ROS_WARN("Inverse Kinematic found no solution.");
    return -1;
  }
}


void PentaxisArmAnalyticalIK::getSolverInfo(
    moveit_msgs::KinematicSolverInfo &info) {
	info = _solver_info;
}


void PentaxisArmAnalyticalIK::ik(const KDL::Frame& frame) {
	nx = frame(0,0); ox = frame(0,1); ax = frame(0,2); px = frame(0,3);
	ny = frame(1,0); oy = frame(1,1); ay = frame(1,2); py = frame(1,3);
	nz = frame(2,0); oz = frame(2,1); az = frame(2,2); pz = frame(2,3);

	solveJoint1(0.0, false);
	solveJoint5(0.0, false);
}

/*
* JOINT 1
*/
void PentaxisArmAnalyticalIK::solveJoint1(double joint5, bool j5_set) {
	std::vector<double> j1;

	/// @f$ \theta_{1} = \atan2\left(\frac{p_{y}}{p_{x}}\right) @f$
	double tempSolution = atan2(py,px); // 1st solution
	if(validateJoint(0, tempSolution, j1)) {
		if(j5_set) solveJoint3(j1.back(), joint5);
		else solveJoint5(j1.back(), true);
	}
  
  /// @f$ \theta_{1} = \atan2\left(\frac{a_{y}}{a_{x}}\right) @f$
	tempSolution = atan2(ay,ax); // 2nd solution
	if(validateJoint(0, tempSolution, j1)) {
		if(j5_set) solveJoint3(j1.back(), joint5);
		else solveJoint5(j1.back(), true);
	}
	
	/// @f$ \theta_{1} = \atan2\left(\frac{-o_{x}-a_{z}n_{y}}{o_{y}-a_{z}n_{x}}\right) @f$	
	tempSolution = atan2(-(ox+az*ny),oy-az*nx); // 3rd solution
	if(validateJoint(0, tempSolution, j1)) {
		if(j5_set) solveJoint3(j1.back(), joint5);
		else solveJoint5(j1.back(), true);
	}
	tempSolution = atan2((ox+az*ny),-(oy-az*nx)); // 3rd solution
	if(validateJoint(0, tempSolution, j1)) {
		if(j5_set) solveJoint3(j1.back(), joint5);
		else solveJoint5(j1.back(), true);
	}
	
	/// @f$ \theta_{1} = \atan2\bigg(\frac{n_{x}-a_{z}o_{y}}{-n_{y}-a_{z}o_{x}}\bigg) @f$	
	tempSolution = atan2(nx-az*oy, -ny-az*ox); // 4th solution
	if(validateJoint(0, tempSolution, j1)) {
		if(j5_set) solveJoint3(j1.back(), joint5);
		else solveJoint5(j1.back(), true);
	}
	tempSolution = atan2(-(nx-az*oy), ny+az*ox); // 4th solution
	if(validateJoint(0, tempSolution, j1)) {
		if(j5_set) solveJoint3(j1.back(), joint5);
		else solveJoint5(j1.back(), true);
	}
	
	if(j5_set) {
		double s5 = sin(joint5);
		double c5 = cos(joint5);
		double j1_cos = (s5*ny+c5*oy)/2.0;
		double j1_sin = (-s5*nx-c5*ox)/2.0;
		
		/// @f$ \theta_1 = \atan2 \bigg(\frac{-s_5 n_x - c_5 o_x}{s_5 n_y + c_5 o_y}\bigg) @f$
		tempSolution = atan2(j1_sin, j1_cos); // 5th solution
		if(validateJoint(0, tempSolution, j1))
			solveJoint3(j1.back(), joint5);
	}

	if(j1.size() < 1 && !j5_set) { // when the arm is in the upright position
		ROS_DEBUG("Assuming joint5 = 0");
		
		/// @f$ \theta_1 = \atan2 \bigg(\frac{-o_x}{o_y} \bigg) @f$
		tempSolution = atan2(ox,oy);  // 6th solution
		if(validateJoint(0, tempSolution, j1))
			solveJoint3(j1.back(), 0.0);
	}
	
	if(j1.size() == 0)
		ROS_DEBUG("Unable to find solution for joint 1");
}


/*
* JOINT 5
*/
void PentaxisArmAnalyticalIK::solveJoint5(double joint1, bool j1_set) {
	std::vector<double> j5;
	double tempSolution;
	
	if(j1_set) {
		double c1 = cos(joint1);
		double s1 = sin(joint1);
		double j5_sin = (c1*ny-s1*nx)/2.0;
		double j5_cos = (c1*oy-s1*ox)/2.0;
		/// @f$ \theta_5 = \bigg( \frac{c_1 n_y - s_1 n_x}{c_1 o_y - s_1 o_x} \bigg) @f$
		tempSolution = atan2(j5_sin, j5_cos); // 4th solution
		if(validateJoint(4, tempSolution, j5))
			solveJoint3(joint1, j5.back());
	}
	
	/// @f$ \theta_{5} = \atan2 \left(-\frac{o_z}{n_z}\right) @f$	
	tempSolution = atan2(oz,-nz); // 1st solution
	if(validateJoint(4, tempSolution, j5)) {
		if(j1_set) solveJoint3(joint1, j5.back());
		else solveJoint1(j5.back(), true);
	}
	tempSolution = atan2(-oz,nz); // 1st solution
	if(validateJoint(4, tempSolution, j5)) {
		if(j1_set) solveJoint3(joint1, j5.back());
		else solveJoint1(j5.back(), true);
	}
	
	/// @f$ \theta_{5} = \atan2\left(\frac{o_{y}a_{z}-n_{x}}{-o_{x}-a_{z}n_{y}}\right) @f$	
	tempSolution = atan2(oy*az-nx,-ox-az*ny); // 2nd solution
	if(validateJoint(4, tempSolution, j5)) {
		if(j1_set) solveJoint3(joint1, j5.back());
		else solveJoint1(j5.back(), true);
	}
	tempSolution = atan2(-(oy*az-nx), ox+az*ny); // 2nd solution
	if(validateJoint(4, tempSolution, j5)) {
		if(j1_set) solveJoint3(joint1, j5.back());
		else solveJoint1(j5.back(), true);
	}
	
	/// @f$ \theta_5 = \atan2 \left(\frac{-n_{y}-a_{z}o_{x}}{o_{y}+a_{z}n_{x}} \right) @f$	
	tempSolution = atan2(-ny-az*ox, oy+az*nx); // 3th solution
	if(validateJoint(4, tempSolution, j5)) {
		if(j1_set) solveJoint3(joint1, j5.back());
		else solveJoint1(j5.back(), true);
	}
	tempSolution = atan2(ny+az*ox, -(oy+az*nx)); // 3th solution
	if(validateJoint(4, tempSolution, j5)) {
		if(j1_set) solveJoint3(joint1, j5.back());
		else solveJoint1(j5.back(), true);
	}

	if(j5.size() == 0)
		ROS_DEBUG("Unable to find solution for joint 5");
}

/*
* JOINT 3
*/
void PentaxisArmAnalyticalIK::solveJoint3(double joint1, double joint5) {
	double c1 = cos(joint1);
	double s1 = sin(joint1);
	double c5 = cos(joint5);
	double s5 = sin(joint5);
	std::vector<double> _Gamma;
	std::vector<double> _Omega;
	std::vector<double> j3;
	
	if(c1 != 0.0) {
		push_back_ifUnique(_Gamma, px/c1 - (d5+d6)*(ax/c1)); /// @f$ \Gamma = \frac{p_{x}}{c_{1}}-(d_{5}+d_{6})\frac{a_{x}}{c_{1}} @f$
		if(c5 != 0.0) 
			push_back_ifUnique(_Gamma, px/c1 + (d5+d6)*(nz/c5)); /// @f$ \Gamma = \frac{p_{x}}{c_{1}}+(d_{5}+d_{6})\frac{n_{z}}{c_{5}} @f$
		if(s1 != 0.0) 
			push_back_ifUnique(_Gamma, px/c1 - (d5+d6)*(ay/s1)); /// @f$ \Gamma = \frac{p_{x}}{c_{1}}-(d_{5}+d_{6})\frac{a_{y}}{s_{1}} @f$
		if(s5 != 0.0)
			push_back_ifUnique(_Gamma, px/c1 - (d5+d6)*(oz/s5)); /// @f$ \Gamma = \frac{p_{x}}{c_{1}}-(d_{5}+d_{6})\frac{o_{z}}{s_{5}} @f$
	}
	if(s1 != 0.0) {
		push_back_ifUnique(_Gamma, py/s1 - (d5+d6)*(ay/s1)); /// @f$ \Gamma = \frac{p_{y}}{s_{1}}-(d_{5}+d_{6})\frac{a_{y}}{s_{1}} @f$
		if(c5 != 0.0) 
			push_back_ifUnique(_Gamma, py/s1 + (d5+d6)*(nz/c5)); /// @f$ \Gamma = \frac{p_{y}}{s_{1}}+(d_{5}+d_{6})\frac{n_{z}}{c_{5}} @f$
		if(c1 != 0.0)
			push_back_ifUnique(_Gamma, py/s1 - (d5+d6)*(ax/c1)); /// @f$ \Gamma = \frac{p_{y}}{s_{1}}-(d_{5}+d_{6})\frac{a_{x}}{c_{1}} @f$
		if(s5 != 0.0)
			push_back_ifUnique(_Gamma, py/s1 - (d5+d6)*(oz/s5)); /// @f$ \Gamma = \frac{p_{y}}{s_{1}}-(d_{5}+d_{6})\frac{o_{z}}{s_{5}} @f$
	}

	double term = d0 + d1 - pz;
	_Omega.push_back(term + (d5+d6)*az); /// @f$ \Omega = d_{0}+d_{1}-p_{z} +(d_{5}+d_{6})a_{z} @f$
	if(c1 != 0.0) {
		if(c5 != 0.0)
			push_back_ifUnique(_Omega, term+(d5+d6)*( (nx+s1*s5) / (c1*c5) ) ); /// @f$ \Omega = d_{0}+d_{1}-p_{z} +(d_{5}+d_{6})\frac{n_{x}+s_{1}s_{5}}{c_{1}c_{5}} @f$
		if(s5 != 0.0)
			push_back_ifUnique(_Omega, term+(d5+d6)*( (-ox-s1*c5) / (c1*s5) ) ); /// @f$ \Omega = = d_{0}+d_{1}-p_{z} +(d_{5}+d_{6})\frac{-o_{x}-s_{1}c_{5}}{c_{1}s_{5}} @f$
	}
	if(s1 != 0.0) {
		if(c5 != 0.0)
			push_back_ifUnique(_Omega, term+(d5+d6)*( (ny-c1*s5) / (s1*c5) ) ); /// @f$ \Omega = d_{0}+d_{1}-p_{z} + (d_{5}+d_{6}) \frac{n_{y} - c_{1}s_{5}}{s_{1}c_{5}} @f$
		if(s5 != 0.0)
			push_back_ifUnique(_Omega, term+(d5+d6)*( (-oy+c1*c5) / (s1*s5) ) ); /// @f$ \Omega = d_{0}+d_{1}-p_{z} +(d_{5}+d_{6})\frac{-o_{y}+c_{1}c_{5}}{s_{1}s_{5}} @f$
	}
	
	for(unsigned int r = 0; r < _Gamma.size(); r++) {
		for(unsigned int s = 0; s < _Omega.size(); s++) {
			double j3_cos = (_Gamma[r]*_Gamma[r]+_Omega[s]*_Omega[s]-(a2*a2)-(a3*a3)) / (2.0*a2*a3);
			double j3_sin = sqrt(1 - j3_cos*j3_cos);
			double tempSolution;
			/// @f$ \theta_{3} = atan2\left( \frac{\pm \sqrt{1-{c_3}^2}}{\left(\frac{\Gamma^2+\Omega^2-a_{2}^2-a_{3}^2}{2a_{2}a_{3}} \right)} \right) @f$
			tempSolution = atan2(j3_sin, j3_cos);
			if(validateJoint(2, tempSolution, j3))
				solveJoint2(joint1, joint5, j3.back(), _Gamma, _Omega);
			/// @f$  @f$
			tempSolution = atan2(-j3_sin, j3_cos);
			if(validateJoint(2, tempSolution, j3))
				solveJoint2(joint1, joint5, j3.back(), _Gamma, _Omega);
				
			tempSolution = atan2(-j3_sin, -j3_cos);
			if(validateJoint(2, tempSolution, j3))
				solveJoint2(joint1, joint5, j3.back(), _Gamma, _Omega);
				
			tempSolution = atan2(j3_sin, -j3_cos);
			if(validateJoint(2, tempSolution, j3))
				solveJoint2(joint1, joint5, j3.back(), _Gamma, _Omega);
		}
	}
	
	if(j3.size() == 0)
		ROS_DEBUG("Unable to find solution for joint 3");
}

/*
* JOINT 2
*/
void PentaxisArmAnalyticalIK::solveJoint2(double joint1, double joint5, double joint3, std::vector<double> &Gamma, std::vector<double> &Omega) {
	double c1 = cos(joint1);
	double s1 = sin(joint1);
	double c5 = cos(joint5);
	double s5 = sin(joint5);
	double c3 = cos(joint3);
	double s3 = sin(joint3);
	std::vector<double> j2;
	double tempSolution;
	
	ROS_DEBUG_STREAM("Gamma.size = " << Gamma.size() << " || Omega.size = " << Omega.size());
	
	for(unsigned int s=0; s < Omega.size(); s++) {
		for(unsigned int r=0; r < Gamma.size(); r++) {
			/// @f$ \theta_{2} = \atan2\bigg(\frac{(a_{2}+a_{3}c_{3})\Omega - (a_{3}s_{3})\Gamma}{(a_{2}+a_{3}c_{3}) \Gamma+(a_{3}s_{3})\Omega} \bigg) @f$
			tempSolution = atan2( ((a2+a3*c3)*Omega[s] - (a3*s3)*Gamma[r]) , ((a2+a3*c3)*Gamma[r] + (a3*s3)*Omega[s]));
			if(validateJoint(1, tempSolution, j2))
				solveJoint4(joint1, joint5, joint3, j2.back());

			tempSolution = acos( ( (a2+a3*c3)*Gamma[r]+a3*s3*Omega[s]) / (Gamma[r]*Gamma[r] + Omega[s]*Omega[s]));
			if(validateJoint(1, tempSolution, j2))
				solveJoint4(joint1, joint5, joint3, j2.back());
				
			tempSolution = asin( ( (a2+a3*c3)*Omega[s]-a3*s3*Gamma[r]) / (Gamma[r]*Gamma[r] + Omega[s]*Omega[s]));
			if(validateJoint(1, tempSolution, j2))
				solveJoint4(joint1, joint5, joint3, j2.back());
		}
	}
	if(j2.size() == 0)
		ROS_DEBUG("Unable to find solution for joint 2");
}



/*
* JOINT 4
*/
void PentaxisArmAnalyticalIK::solveJoint4(double joint1, double joint5, double joint3, double joint2) {
	double s1 = sin(joint1);
	double c1 = cos(joint1);
	double s2 = sin(joint2);
	double c2 = cos(joint2);
	double c23 = cos(joint2+joint3);
	double s23 = sin(joint2+joint3);
	double s5 = sin(joint5);
	double c5 = cos(joint5);
	std::vector<double> _c234;
	std::vector<double> _s234;
	std::vector<double> j4;
	
	if(c1 != 0.0) {
		if(c5 != 0.0) 
			push_back_ifUnique( _c234,(nx+s1*s5)/(c1*c5)); /// @f$ = c_{234} = \frac{n_{x} + s_{1}s_{5}}{c_{1}c_{5}} @f$
		if(s5 != 0.0)
			push_back_ifUnique( _c234,(-ox-s1*c5)/(c1*s5)); /// @f$ c_{234} = \frac{-o_{x}-s_{1}c_{5}}{c_{1}s_{5}} @f$
	}
	if(s1 != 0.0) {
		if(c5 != 0.0)
			push_back_ifUnique( _c234,(ny-c1*s5)/(s1*c5)); /// @f$ c_{234} = \frac{n_{y}-c_{1}s_{5}}{s_{1}c_{5}} @f$
		if(s5 != 0.0)
			push_back_ifUnique( _c234,(-oy+c1*c5)/(s1*s5)); /// @f$ c_{234} = \frac{-o_{y}+c_{1}c_{5}}{s_{1}s_{5}} @f$
	}
	push_back_ifUnique( _c234,(1.0/(d5+d6))*(pz-(d0+d1)+a2*s2+a3*s23)); /// @f$ c_{234} = \frac{1}{d_{5}+d_{6}} [p_z - (d_0+d_1)+a_2s_2+a_3s_{23}] @f$
	push_back_ifUnique( _c234,az); /// c_{234} = @f$ a_{z} @f$

	if(c1 != 0.0) {
		push_back_ifUnique( _s234, ax/c1); /// @f$ s_{234} = \frac{a_x}{c_1} @f$
		push_back_ifUnique( _s234, (1.0/(d5+d6))*((px/c1) - a2*c2-a3*c23)); /// @f$ s_{234} = \frac{1}{d_{5}+d_{6}}\bigg( \frac{p_x}{c_1} - a_{2}c_{2}-a_{3}c_{23}  \bigg) @f$
	}
	if(s1 != 0.0) {
		push_back_ifUnique( _s234, ay/s1); /// @f$ s_{234} = \frac{a_y}{s_1} @f$
		push_back_ifUnique( _s234, (1.0/(d5+d6))*((px/s1) - a2*c2-a3*c23) ); /// @f$ s_{234} = \frac{1}{d_{5}+d_{6}}\bigg( \frac{p_x}{s_1} - a_{2}c_{2}-a_{3}c_{23}  \bigg) @f$
	}
	if(c5 != 0.0)
		push_back_ifUnique( _s234, -nz/c5); /// @f$ s_{234} = -\frac{n_{z}}{c_{5}} @f$
	if(s5 != 0.0)
		push_back_ifUnique( _s234, oz/s5); /// @f$ s_{234} = \frac{o_z}{s_5} @f$

	for(int r = 0; r < _c234.size(); r++) {
		for(int s = 0; s < _s234.size(); s++) {
			/// @f$ \theta_{4} = \atan2 \left( \frac{c_{23}s_{234} - s_{23}c_{234}}{s_{23}s_{234} + c_{23}c_{234}} \right) @f$
			double tempSolution = atan2( (c23*_s234[s]-s23*_c234[r]),(s23*_s234[s]+c23*_c234[r]) );
			if(validateJoint(3, tempSolution, j4))
				addNewSolution(joint1, joint2, joint3, j4.back(), joint5);	
			tempSolution = acos(s23*_s234[s]+c23*_c234[r]);
			if(validateJoint(3, tempSolution, j4))
				addNewSolution(joint1, joint2, joint3, j4.back(), joint5);
			tempSolution = asin(c23*_s234[s]-s23*_c234[r]);
			if(validateJoint(3, tempSolution, j4))
				addNewSolution(joint1, joint2, joint3, j4.back(), joint5);
		}
	}

	if(j4.size() == 0) 
		ROS_DEBUG("Unable to find solution for joint 4");
}

void PentaxisArmAnalyticalIK::addNewSolution(double j1, double j2, double j3, double j4, double j5) {
	KDL::JntArray solution(5);
	solution(0) = j1;
	solution(1) = j2;
	solution(2) = j3;
	solution(3) = j4;
	solution(4) = j5;
	solutions.push_back(solution);
}

bool PentaxisArmAnalyticalIK::validateJoint(int j_index, double j_element, std::vector<double> &j_vector) {
	if( isJointValid(j_element, j_index) )
		if( push_back_ifUnique(j_vector, j_element) )
			return true;
	return false;
}

bool PentaxisArmAnalyticalIK::push_back_ifUnique(
		std::vector<double> &vect, double newElement) {
	unsigned int size = vect.size();
	double error = 1e-3;
	if(size < 1) {
		vect.push_back(newElement);
		return true;
	}
	for(unsigned int i = 0; i < size; i++) {
		if(std::abs(vect[i] - newElement) < error) {
			return false;
		}
	}
	vect.push_back(newElement);
	return true;
}


/*
* Checks whether a Joint has a valid value.
*/
bool PentaxisArmAnalyticalIK::isJointValid(double jointValue, int jointNum) const {
	bool valid = true;
	
	if((jointValue <= _min_angles[jointNum]) || (jointValue >= _max_angles[jointNum])) {
      valid = false;
	}
	if(jointValue != jointValue) {
		//ROS_INFO_ONCE("We have a NaN!!!");
		valid = false;
	}
	return valid;
}

bool PentaxisArmAnalyticalIK::validateSolutionWithFK(double joint1, double joint2, double joint3, double joint4, double joint5) {
	double error = 0.001; // Presicion
	double nx_fk, ox_fk, ax_fk, px_fk;
	double ny_fk, oy_fk, ay_fk, py_fk;
	double nz_fk, oz_fk, az_fk, pz_fk;
	double c1 = cos(joint1), s1 = sin(joint1);
	double c2 = cos(joint2), s2 = sin(joint2);
	double c23 = cos(joint2+joint3), s23 = sin(joint2+joint3);
	double c234 = cos(joint2+joint3+joint4);
	double s234 = sin(joint2+joint3+joint4);
	double c5 = cos(joint5), s5 = sin(joint5);

	ROS_DEBUG_STREAM("CHECKING WITH FK");
	// Forward kinematics:
	
	/// @f$ n_{x} = c_{234} c_{1}c_{5} - s_{1}s_{5} @f$
	nx_fk = c234*c1*c5 - s1*s5;
	
	/// @f$ n_{y} = c_{234} s_{1}c_{5} + c_{1}s_{5} @f$
	ny_fk = c234*s1*c5 + c1*s5;
	
	/// @f$ n_{z} = -s_{234}c_{5} @f$
	nz_fk = -s234*c5;
	
	/// @f$ o_{x} = -c_{234}c_{1}s_{5}-s_{1}c_{5} @f$
	ox_fk = -c234*c1*s5 - s1*c5;
	
	/// @f$ o_{y} = -c_{234}s_{1}s_{5}+c_{1}c_{5} @f$
	oy_fk = -c234*s1*s5 + c1*c5;
	
	/// @f$ o_{z} = s_{234}s_{5} @f$
	oz_fk = s234*s5;
	
	/// @f$ a_{x} = s_{234}c_{1} @f$
	ax_fk = s234*c1;
	
	/// @f$ a_{y} = s_{234}s_{1} @f$
	ay_fk = s234*s1;
	
	/// @f$ a_{z} = c_{234} @f$
	az_fk = c234;
	
	/// @f$ p_{x} = c_{1}[a_{2}c_{2}+a_{3}c_{23}+(d_{5}+d_{6})s_{234}] @f$
	px_fk = c1*(a2*c2 + a3*c23 + (d5+d6)*s234);
	
	/// @f$ p_{y} = s_{1}[a_{2}c_{2}+a_{3}c_{23}+(d_{5}+d_{6})s_{234}] @f$
	py_fk = s1*(a2*c2 + a3*c23 + (d5+d6)*s234);
	
	/// @f$ p_{z} = d_{0}+d_{1}-a_{2}s_{2}-a_{3}s_{23}+(d_{5}+d_{6})c_{234} @f$
	pz_fk = d0 + d1 - a2*s2 - a3*s23 + (d5+d6)*c234;

	
	// Comparison
	if(std::abs(nx - nx_fk) > error)
		return false;
	if(std::abs(ny - ny_fk) > error)
		return false;
	if(std::abs(nz - nz_fk) > error)
		return false;
	if(std::abs(ox - ox_fk) > error)
		return false;
	if(std::abs(oy - oy_fk) > error)
		return false;
	if(std::abs(oz - oz_fk) > error)
		return false;
	if(std::abs(ax - ax_fk) > error)
		return false;
	if(std::abs(ay - ay_fk) > error)
		return false;
	if(std::abs(az - az_fk) > error)
		return false; 
	if(std::abs(px - px_fk) > error) {
		ROS_DEBUG_STREAM("PX is faulty, error = " << std::abs(px - px_fk));
		return false;
	}
	if(std::abs(py - py_fk) > error) {
		ROS_DEBUG_STREAM("PY is faulty, error = " << std::abs(py - py_fk));
		return false;
	}
	if(std::abs(pz - pz_fk) > error) {
		ROS_DEBUG_STREAM("PZ is faulty, error = " << std::abs(pz - pz_fk));
		return false;
	}
	
	return true;
}

} /* namespace pentaxis_kinematics */
