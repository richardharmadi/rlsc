/*
 * BaxterClient.cpp
 *
 *  Created on: 20 Feb 2017
 *      Author: Richard Dharmadi
 */

#include "BaxterTools.h"

int main(int argc,char* argv[])
{
  // Create the robot interface object
  BaxterTools bax;
  // Connect to the simulator
  if(argc==2)
  {
    bax.Connect(argv[1]);
  }
  else
  {
    bax.Connect("localhost");
  }
  // Start the simulation
	bax.StartSimulation();

	Eigen::VectorXd q=Eigen::VectorXd::Zero(18); // Joint angles
	Eigen::VectorXd y; // End-effector position
	Eigen::VectorXd target; // Target positions
	Eigen::MatrixXd J; // Jacobian matrix

	//////////////////////////////////////////////////////////////////////
	// Constants for homework
	Eigen::MatrixXd W = Eigen::MatrixXd::Zero(7,7); // Weighting matrix
	for(int i=0;i<7;i++) W(i,i) = ((double)i)/6.0+0.1;
	
	Eigen::MatrixXd Winv = W.inverse();

	Eigen::MatrixXd C = Eigen::MatrixXd::Identity(3,3)*1e3; // Regularisation
        Eigen::MatrixXd Cinv = Eigen::MatrixXd::Identity(3,3)*1e-3;
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7,7);
	Eigen::VectorXd qstart1(18); // Starting pose 1
	Eigen::VectorXd qstart2(18); // Starting pose 2
	Eigen::VectorXd qstart3(18); // Starting pose 3
	qstart1 << M_PI/4.0,0,0,M_PI/2.0,0,0,0,0,0,      -M_PI/4.0,0,0,M_PI/2.0,0,0,0,0,0;
  	qstart2 << -M_PI/4.0,0,0,M_PI/2.0,M_PI/2.0,M_PI/2.0,0,0,0,   M_PI/4.0,0,0,M_PI/2.0,-M_PI/2.0,M_PI/2.0,0,0,0;
  	qstart3 << M_PI/4.0,-M_PI/2.0,0,M_PI/2.0,-M_PI/4.0,-M_PI/4.0,0,0,0,      -M_PI/4.0,-M_PI/2.0,0,M_PI/2.0,M_PI/4.0,-M_PI/4.0,0,0,0;

	Eigen::VectorXd q_comf1(18); // Comfortable pose 1
	Eigen::VectorXd q_comf2(18); // Comfortable pose 2
	q_comf1 << 0,-0.5,0,0.5,0,1.5,0,0,0,  0,-0.5,0,0.5,0,1.5,0,0,0 ;
	q_comf2 << -20.0/180.0*M_PI, 40.0/180.0*M_PI, 70.0/180.0*M_PI, 90.0/180.0*M_PI, 0,0.5,0,0,0, 20.0/180.0*M_PI, 40.0/180.0*M_PI, -70.0/180.0*M_PI, 90.0/180.0*M_PI, 0,0.5,0,0,0;
	
	Eigen::VectorXd qcurrent(18); // Current joint angle
	Eigen::VectorXd qprev(18); // Previous joint angle
	Eigen::VectorXd ystar(3); // Current target
	Eigen::VectorXd eps = Eigen::VectorXd::Ones(18)*0.5;
	//////////////////////////////////////////////////////////////////////


	// Loop until 'q' gets pressed
	char key=0;
	float e=0.01;
	while(key!='q')
  {	
	// Get pressed key
	key=bax.GetKey();
	
	// Get target positions
        bax.GetTargets(target);
	// ================== PART A ==================//
	for(int i=0;i<8;i++) // Iterate for all 8 target positions,
    	{
	 ystar = target.segment(i*3,3);

	 // Iterating inverse kinematic algorithm
	 qcurrent = qstart1; // starting position 1
	 qprev = qcurrent + eps;

	 while ((qcurrent-qprev).norm() > e)
	 {
	  qprev = qcurrent;
	  y=bax.GetIK(qcurrent); // Get end-effector position
	  J=bax.GetJ(qcurrent);  // Get Jacobian of the end effector
	  Eigen::MatrixXd J_pos_right = J.block(0,0,3,7); // Get position Jacobian of the right arm (a 3x7 block at row 0 and column 0)
	  // Compute Inverse Jacobian
	  Eigen::MatrixXd Jinv = Winv*J_pos_right.transpose()*(J_pos_right*Winv*J_pos_right.transpose()+Cinv).inverse(); 
	  // Compute joint angles
	  qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_right)*(q_comf1.segment(0,7)-qcurrent.segment(0,7)); // for reaching with right arm, and use qcomf_1
	  std::cout << "Current q:" << qcurrent << "\n";
	  bax.SetJointAngles(qcurrent);
	  // Update simulation
    	  bax.AdvanceSimulation(); 
	 }
	 // To simulate 
	 //for(int i=0;i<10;i++)
	 //{
	  //bax.SetJointAngles(qcurrent*); matrix yg isi nya 0.1 semua, terus kali element wise
	 //}
	}	
    ////////////////////////////////////////////////


  }
  // Stop simulation and close connection
  bax.StopSimulation();
	return(0);
}

