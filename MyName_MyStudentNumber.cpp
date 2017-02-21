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
	Eigen::VectorXd costright(32);
	Eigen::VectorXd costleft(32);
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
    // ================== right arm =================//
	for(int i=0;i<16;i++) // Iterate for all 8 target positions, twice for both q_comf1 and q_comf2
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
	  Eigen::MatrixXd Jinv = Winv*J_pos_right.transpose()*(J_pos_right*Winv*J_pos_right.transpose()+Cinv).inverse(); // Compute Inverse Jacobian
	  if (i<8)
	  {
	   qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_right)*(q_comf1.segment(0,7)-qcurrent.segment(0,7)); //use qcomf_1
	  }else{
	   qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_right)*(q_comf2.segment(0,7)-qcurrent.segment(0,7)); //use qcomf_2
	  }	  
	  bax.SetJointAngles(qcurrent);
	  // Update simulation
      bax.AdvanceSimulation(); 
	 }
	 costright(i) = (qstart1-qcurrent).squared_norm()
	 std::cout << "Weighted cost:" << costright(i) << "\n";
	 // To simulate 
	 //for(int i=0;i<10;i++)
	 //{
	  //bax.SetJointAngles(qcurrent*); matrix yg isi nya 0.1 semua, terus kali element wise
	 //}
	}
	// ================== left arm =================//
	for(int i=0;i<16;i++) // Iterate for all 8 target positions, twice for both q_comf1 and q_comf2
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
	  Eigen::MatrixXd J_pos_left = J.block(6,7,3,7); // Get position Jacobian of the right arm (a 3x7 block at row 0 and column 0)
	  Eigen::MatrixXd Jinv = Winv*J_pos_left.transpose()*(J_pos_left*Winv*J_pos_left.transpose()+Cinv).inverse(); // Compute Inverse Jacobian
	  if (i<8)
	  {
	   qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_left)*(q_comf1.segment(0,7)-qcurrent.segment(0,7)); //use qcomf_1
	  }else{
	   qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_left)*(q_comf2.segment(0,7)-qcurrent.segment(0,7)); //use qcomf_2
	  }	  
	  bax.SetJointAngles(qcurrent);
	  // Update simulation
      bax.AdvanceSimulation(); 
	 }
	 costleft(i) = (qstart1-qcurrent).squared_norm()
	 std::cout << "Weighted cost:" << costleft(i) << "\n";
	}
    ////////////////////////////////////////////////
	// ================== PART B ==================//
	 ystar = target.segment(0,3);

	 for (int j=0;j<2;j++)
	 {
	 	for(int i=0;i<3;i++)
	 	{
		 	switch(i){
		 		case 0:
		 			qcurrent=qstart1;
		 			break;
		 		case 1:
		 			qcurrent=qstart2;
		 			break;
		 		case 2:
		 			qcurrent=qstart3;
	 		}
			// Iterating inverse kinematic algorithm
			qprev = qcurrent + eps;

			while ((qcurrent-qprev).norm() > e)
			{
				qprev = qcurrent;
				y=bax.GetIK(qcurrent); // Get end-effector position
				J=bax.GetJ(qcurrent);  // Get Jacobian of the end effector
				Eigen::MatrixXd J_pos_right = J.block(0,0,3,7); // Get position Jacobian of the right arm (a 3x7 block at row 0 and column 0)
				Eigen::MatrixXd Jinv = Winv*J_pos_right.transpose()*(J_pos_right*Winv*J_pos_right.transpose()+Cinv).inverse(); // Compute Inverse Jacobian
				if (j<1)
				{
					qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_right)*(q_comf1.segment(0,7)-qcurrent.segment(0,7)); //use qcomf_1
				}else{
				    qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3)) // use minimum norm for redundancy resolution
				}	  
				bax.SetJointAngles(qcurrent);
				// Update simulation
			    bax.AdvanceSimulation(); 
			}
			// ==== Metric other than cost =====///
			//costright(i) = (qstart1-qcurrent).squared_norm()
	 		//std::cout << "Experiment %d Weighted cost %d :" << costright(i) << "\n";
	 		//runtime
	 		//plot qcurrent di tiap waktu buat 3 starting position aja, dan untuk 2 test experiment, cari append array eigen
	 	}
	 }

	// ================== PART C ==================//
	for(int j=0;j<3;j++)
	{
		for(int i=0;i<8;i++) // Iterate for all 8 target positions, once for q_comf1
	    {
		 ystar = target.segment(i*3,3);

		 // Iterating inverse kinematic algorithm
		 switch(j)
		 {
	 		case 0:
	 			qcurrent=qstart1; //starting position 1
	 			break;
	 		case 1:
	 			qcurrent=qstart2; //starting position 2
	 			break;
	 		case 2:
	 			qcurrent=qstart3; //starting position 3
	 	}
		 qprev = qcurrent + eps;

		 while ((qcurrent-qprev).norm() > e)
		 {
		  qprev = qcurrent;
		  y=bax.GetIK(qcurrent); // Get end-effector position
		  J=bax.GetJ(qcurrent);  // Get Jacobian of the end effector
		  Eigen::MatrixXd J_pos_right = J.block(0,0,3,7); // Get position Jacobian of the right arm (a 3x7 block at row 0 and column 0)
		  Eigen::MatrixXd Jinv = Winv*J_pos_right.transpose()*(J_pos_right*Winv*J_pos_right.transpose()+Cinv).inverse(); // Compute Inverse Jacobian
		  qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_right)*(q_comf1.segment(0,7)-qcurrent.segment(0,7)); //use qcomf_1
		  bax.SetJointAngles(qcurrent);
		  // Update simulation
	      bax.AdvanceSimulation(); 
		 }
		 //costright(i) = (qstart1-qcurrent).squared_norm()
		 //std::cout << "Weighted cost:" << costright(i) << "\n";
		}
	}
	//=========== PCA ===============//
  }
  // Stop simulation and close connection
  bax.StopSimulation();
  return(0);
}

