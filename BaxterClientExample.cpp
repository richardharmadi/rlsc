/*
 * BaxterClient.cpp
 *
 *  Created on: 24 Jan 2014
 *      Author: Vladimir Ivan
 */

// Make sure to have the server side running in V-REP!

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

	double t=0;
	Eigen::VectorXd q=Eigen::VectorXd::Zero(18); // Joint angles
	Eigen::VectorXd x; // End-effector position
	Eigen::VectorXd target; // Target positions
	Eigen::MatrixXd J; // Jacobian matrix


	//////////////////////////////////////////////////////////////////////
	// Constants for homework
	Eigen::MatrixXd W = Eigen::MatrixXd::Zero(7,7); // Weighting matrix
	for(int i=0;i<7;i++) W(i,i) = ((double)i)/6.0+0.1;

	Eigen::MatrixXd C = Eigen::MatrixXd::Identity(3,3)*1e3; // Regularisation
    Eigen::MatrixXd Cinv = Eigen::MatrixXd::Identity(3,3)*1e-3;

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

	//////////////////////////////////////////////////////////////////////


	// Loop until 'q' gets pressed
	char key=0;
	while(key!='q')
  {
	  // Get pressed key
	  key=bax.GetKey();

    // Set joint positions (this is just an example sinusoidal motion)
        q(1)=sin(t*M_PI*0.3)*0.4; // Example motion
	  bax.SetJointAngles(q);

	  // Get end-effector position
	  x=bax.GetIK(q);
	  // Get Jacobian of the end effector
	  J=bax.GetJ(q);

    // Example vector and matrix operations
    // (see http://eigen.tuxfamily.org/dox/group__DenseMatrixManipulation__chapter.html for documentation)
	  if(t==0) // Only run this once to avoid dumping too much text into the terminal
	  {
      std::cout << "x:\n" << x << "\n"; // Print the end-effector position
      // The x vector contains: [x,y,z,roll,pitch,yaw,x,y,z,roll,pitch,yaw] for right and left arm respectively
      std::cout << "J:\n" << J << "\n"; // Print the Jacobian matrix
      // The J marix has rows corresponding to rows of vector x
      // The columns corresond to 7 joints of the right arm and then the 7 joints of the right arm (from torso outwards)

      // Extracting segments of longer vectors
      Eigen::VectorXd left_arm_position = x.segment(6,3); // Getx x,y,z of the left arm end-effector
      std::cout << "Left arm position:\n" << left_arm_position << "\n";
      Eigen::VectorXd right_arm_orientation = x.segment(3,3); // Getx roll,pitch,yaw of the right arm end-effector
      std::cout << "Right arm orientation:\n" << right_arm_orientation << "\n";

      // Extracting blocks of matrices
      Eigen::MatrixXd J_pos_left = J.block(6,7,3,7); // Get position Jacobian of the left arm (a 3x7 block at 6th row and 7th column)
      std::cout << "Left arm Jacobian:\n" << J_pos_left << "\n";
  
      // Transposing a matrix
      Eigen::MatrixXd J_pos_left_transposed = J_pos_left.transpose();
      std::cout << "Size of Jt:" << J_pos_left_transposed.rows() <<"x"<< J_pos_left_transposed.cols() << "\n";
      // Matrix multiplication
      Eigen::MatrixXd JJt=J_pos_left*J_pos_left_transposed;
      std::cout << "Size of J*Jt:" << JJt.rows() <<"x"<< JJt.cols() << "\n";
      // Inverting a matrix
      Eigen::MatrixXd invJJt=JJt.inverse();
      // Matrix determinant
      double det=JJt.determinant();
      std::cout << "Determinant of J*Jt:" << det << "\n";

      // Computing eigen values
      Eigen::EigenSolver<Eigen::MatrixXd> eig(JJt);
      std::cout << "Eigen values of J*Jt:\n" << eig.eigenvalues() << "\n";

      // Get target positions
      bax.GetTargets(target);
      std::cout << "Target 1 (red) position:\n" << target.segment(0,3) << "\n";
      std::cout << "Target 8 (green) position:\n" << target.segment(7*3,3) << "\n";
	  }

    // Update simulation
    bax.AdvanceSimulation();
    t+=0.05;
  }
  // Stop simulation and close connection
  bax.StopSimulation();
	return(0);
}

