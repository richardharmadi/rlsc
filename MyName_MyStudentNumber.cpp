/*
 * BaxterClient.cpp
 *
 *  Created on: 20 Feb 2017
 *      Author: Richard Dharmadi
 */

#include "BaxterTools.h"
#include <sys/time.h>

int main(int argc,char* argv[]){ 
	BaxterTools bax; // Create the robot interface object
  	
  	timeval time;
  	// Connect to the simulator
  	if(argc==2){
    	bax.Connect(argv[1]);
  	}else{
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
	Eigen::VectorXd ystar(3); // Current target
	Eigen::VectorXd eps = Eigen::VectorXd::Ones(18)*0.5;
	Eigen::VectorXd costright_a(32);
	Eigen::VectorXd costleft_a(32);
	Eigen::VectorXd yprev(3);
	Eigen::VectorXd qprev(18);

	//std::chrono::time_point<std::chrono::system_clock> start, end;
	
	Eigen::VectorXd cost_b(6);
	Eigen::VectorXd startingq(18); // starting joint angles
	Eigen::VectorXd qdof1,qdof2,qdof3,qdof4,qdof5,qdof6,qdof7;

	// Eigen::VectorXd qfinal(18);
	// Eigen::VectorXd ystar_final(3);
	// Eigen::VectorXd add(3);
	Eigen::VectorXd yinit(3);
	Eigen::VectorXd yinter(3);
	Eigen::MatrixXd pca = Eigen::MatrixXd::Zero(24,7); 
	//////////////////////////////////////////////////////////////////////

	// Loop until 'q' gets pressed
	char key=0;
	float e=1e-3;
	long start_time,end_time;
	double runtime;
	 
	while(key!='q'){		
		// Get pressed key
		key=bax.GetKey();
	
		// Get target positions
    	bax.GetTargets(target);
    	/*
		// ================== PART A ====================//
    	// ================== right arm =================//
		std::cout << "PART A Right Arm \n" ;
		for(int i=0;i<16;i++){ // Iterate for all 8 target positions, twice for q_comf1 and q_comf2
    		if (i<8){
    			ystar = target.segment(i*3,3);
    			e=1e-3;
    		}else {
    			ystar = target.segment((i-8)*3,3);
				//ystar = target.segment(6*3,3);
				if(i==15){
					e=2e-1;
				}
    		}	 		
	 		// Iterating inverse kinematic algorithm
	 		qcurrent = qstart1; // starting position 1
	 		qprev = qcurrent + eps;
	 		y = bax.GetIK(qcurrent); // get end-effector starting position
			gettimeofday(&time, NULL);
			start_time = (time.tv_sec *1000) +(time.tv_usec/1000);
			while ((qcurrent-qprev).squaredNorm()>e){
				y=bax.GetIK(qcurrent); // Get end-effector position 
				J=bax.GetJ(qcurrent);  // Get Jacobian of the end effector
				Eigen::MatrixXd J_pos_right = J.block(0,0,3,7); // Get position Jacobian of the right arm (a 3x7 block at row 0 and column 0)
				Eigen::MatrixXd Jinv = Winv*J_pos_right.transpose()*(J_pos_right*Winv*J_pos_right.transpose()+Cinv).inverse(); // Compute Inverse Jacobian
	  		  	qprev = qcurrent;
	  		  	yprev = bax.GetIK(qprev).segment(0,3);
	  		  	if (i<8){
	   				qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_right)*(q_comf1.segment(0,7)-qcurrent.segment(0,7)); //use qcomf_1
	  			}else{
	   				qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_right)*(q_comf2.segment(0,7)-qcurrent.segment(0,7)); //use qcomf_2
	  			}	
	  			bax.SetJointAngles(qcurrent);
				// Update simulation
			    bax.AdvanceSimulation();			
	 		}
	 		gettimeofday(&time, NULL);
	 		end_time = (time.tv_sec *1000) +(time.tv_usec/1000);
	 		runtime = end_time-start_time;
	 		costright_a(i) = (qstart1-qcurrent).squaredNorm();
	 		std::cout << "Weighted cost" << i << ": " << costright_a(i) << "\n";
	 		std::cout << "Run time: " << runtime << "\n";
		}
		
		// ================== left arm ===================//
		std::cout << "PART A Left Arm \n" ;
		for(int i=0;i<16;i++){ // Iterate for all 8 target positions, twice for both q_comf1 and q_comf2{
	 		e=1e-1;
	 		if (i<8){
    			ystar = target.segment(i*3,3);
    		}else {
    			ystar = target.segment((i-8)*3,3);
    		}
	 		// Iterating inverse kinematic algorithm
	 		qcurrent = qstart1; // starting position 1
	 		bax.SetJointAngles(qcurrent);
	 		qprev = qcurrent + eps;
			y = bax.GetIK(qcurrent); // get end-effector starting position
			// yprev = y.segment(0,3) + eps;
			gettimeofday(&time, NULL);
			start_time = (time.tv_sec *1000) +(time.tv_usec/1000);
	 		while ((qcurrent-qprev).squaredNorm() > e){
	 			y=bax.GetIK(qcurrent); // Get end-effector position	
	  			J=bax.GetJ(qcurrent);  // Get Jacobian of the end effector
	  			Eigen::MatrixXd J_pos_left = J.block(6,7,3,7); // Get position Jacobian of the left arm (a 3x7 block at row 6th and column 7th)
	  			Eigen::MatrixXd Jinv = Winv*J_pos_left.transpose()*(J_pos_left*Winv*J_pos_left.transpose()+Cinv).inverse(); // Compute Inverse Jacobian
	  			qprev = qcurrent;
	  			yprev = bax.GetIK(qprev).segment(0,3);
	  			if (i<8){
	   				qcurrent.segment(9,7) = qcurrent.segment(9,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_left)*(q_comf1.segment(9,7)-qcurrent.segment(9,7)); //use qcomf_1
	 			}else{
	   				qcurrent.segment(9,7) = qcurrent.segment(9,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_left)*(q_comf2.segment(9,7)-qcurrent.segment(9,7)); //use qcomf_2
	  			}	  			
	  			bax.SetJointAngles(qcurrent);
	  			// Update simulation
      			bax.AdvanceSimulation(); 
	 		}
	 		gettimeofday(&time, NULL);
	 		end_time = (time.tv_sec *1000) +(time.tv_usec/1000);
			//runtime = difftime(end_time,start_time);
	 		runtime = end_time-start_time;
	 		costleft_a(i) = (qstart1-qcurrent).squaredNorm();
	 		std::cout << "Weighted cost" << i << ": " << costleft_a(i) << "\n";
	 		std::cout << "Run time: " << runtime << "\n";
		}
		
    	// ================== PART B ==================//
	 	std::cout << "PART B \n" ;
	 	ystar = target.segment(0,3);

	 	for (int j=0;j<2;j++){ // for experiment 1 and 2 (no minimum norm for redundancy resolution)
	 		for(int i=0;i<3;i++){
		 		if(i==1){
		 			qcurrent=qstart1;
		 			startingq=qstart1;
		 			bax.SetJointAngles(qcurrent);
		 		}else if (i==2){
		 			qcurrent=qstart2;
		 			startingq=qstart2;
		 			bax.SetJointAngles(qcurrent);
		 		}else if (i==3){
		 			qcurrent=qstart3;
		 			startingq=qstart3;
		 			bax.SetJointAngles(qcurrent);
		 		}
		 			
				// Iterating inverse kinematic algorithm
	 			qprev = qcurrent + eps;
				gettimeofday(&time, NULL);
				start_time = (time.tv_sec *1000) +(time.tv_usec/1000);
				while ((qcurrent-qprev).squaredNorm()>e){
					y=bax.GetIK(qcurrent); // Get end-effector position	 
					J=bax.GetJ(qcurrent);  // Get Jacobian of the end effector
					Eigen::MatrixXd J_pos_right = J.block(0,0,3,7); // Get position Jacobian of the right arm (a 3x7 block at row 0 and column 0)
					Eigen::MatrixXd Jinv = Winv*J_pos_right.transpose()*(J_pos_right*Winv*J_pos_right.transpose()+Cinv).inverse(); // Compute Inverse Jacobian
					qprev = qcurrent;
					yprev = bax.GetIK(qprev).segment(0,3);
					if (j<1){
						qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_right)*(q_comf1.segment(0,7)-qcurrent.segment(0,7)); //use qcomf_1
					}else{
				    	qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3)); // use minimum norm for redundancy resolution
					} 
					bax.SetJointAngles(qcurrent);
					// Update simulation
			    	bax.AdvanceSimulation(); 
				}
				gettimeofday(&time, NULL);
	 			end_time = (time.tv_sec *1000) +(time.tv_usec/1000);
	 			runtime = end_time-start_time;
				std::cout << "Run time: " << runtime << "\n";
				if (j<1){
					cost_b(i) = (startingq-qcurrent).squaredNorm();
					std::cout << "Experiment " << 1 << "Starting pos " << i+1 << " Weighted cost:" << cost_b(i) << "\n";
				}else{
					cost_b(i+3) = (startingq-qcurrent).squaredNorm();
					std::cout << "Experiment " << j+1 << "Starting pos " << i+1 << " Weighted cost:" << cost_b(i+3) << "\n";
				}
	 		}
	 	}
	 	
		*/
		// ================== PART C ==================//
		for(int j=0;j<3;j++){
			for(int i=0;i<8;i++){ // Iterate for all 8 target positions, once for q_comf1
		 		ystar = target.segment(i*3,3);

				// Iterating inverse kinematic algorithm
		 		switch(j){
	 				case 0:
	 					qcurrent=qstart1; //starting position 1
	 					bax.SetJointAngles(qstart1);
	 					e=1e-3;
	 					break;
	 				case 1:
	 					qcurrent=qstart2; //starting position 2
	 					bax.SetJointAngles(qstart2);
	 					break;
	 				case 2:
	 					qcurrent=qstart3; //starting position 3
	 					bax.SetJointAngles(qstart3);
	 					break;
	 			}
	 			y=bax.GetIK(qcurrent);
				// yprev = y.segment(0,3) + eps;
				gettimeofday(&time, NULL);
				start_time = (time.tv_sec *1000) +(time.tv_usec/1000);
				while ((y.segment(0,3)-ystar).norm()>e){	 
		  			J=bax.GetJ(qcurrent);  // Get Jacobian of the end effector
		  			Eigen::MatrixXd J_pos_right = J.block(0,0,3,7); // Get position Jacobian of the right arm (a 3x7 block at row 0 and column 0)
		  			Eigen::MatrixXd Jinv = Winv*J_pos_right.transpose()*(J_pos_right*Winv*J_pos_right.transpose()+Cinv).inverse(); // Compute Inverse Jacobian
		  			qprev = qcurrent;
		  			yprev = bax.GetIK(qprev).segment(0,3);
		  			qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_right)*(q_comf1.segment(0,7)-qcurrent.segment(0,7)); //use qcomf_1
		  			y=bax.GetIK(qcurrent); // Get end-effector position
		  			bax.SetJointAngles(qcurrent);
		  			// Update simulation
	      			bax.AdvanceSimulation();
		 		}
		 		std::cout << "Start pose "<< j+1 << "Target" << i+1 << "\n";
		 		gettimeofday(&time, NULL);
	 			end_time = (time.tv_sec *1000) +(time.tv_usec/1000);
	 			runtime = end_time-start_time;
				std::cout << "Run time: " << runtime << "\n";
		 	pca.row(j+i) = qcurrent.segment(0,7); // put every result pose into 1 matrix
			}
		}
		//=========== PCA ===============//
		//Eigen::EigenSolver<Eigen::MatrixXd> eig(pca.transpose());
		//std::cout << "Eigen values:\n" << eig.eigenvalues() << "\n"; // 7 eigenvalues
		//std::cout << "Matrix of Eigen vectors:\n" << eig.eigenvectors() << "\n";
		//std::cout << "Eigen value 1: " << eig.eigenvalues()[0] << "\n";
		//std::cout << "Eigen value 2: " << eig.eigenvalues()[1] << "\n";
		//std::cout << "Eigen value 3: " << eig.eigenvalues()[2] << "\n";
		//std::cout << "Eigen value 4: " << eig.eigenvalues()[3] << "\n";
		//std::cout << "Eigen value 5: " << eig.eigenvalues()[4] << "\n";
		//std::cout << "Eigen value 6: " << eig.eigenvalues()[5] << "\n";
		//std::cout << "Eigen value 7: " << eig.eigenvalues()[6] << "\n";
		//Eigen::VectorXd total_eigen(*) *besar nya dimensi eigen value diatas
		//total_eigen = eig.eigenvalues()[0] + eig.eigenvalues()[1] + eig.eigenvalues()[2] + eig.eigenvalues()[3] + eig.eigenvalues()[4] + eig.eigenvalues()[5] + eig.eigenvalues()[6]
  		//std::cout << "Explained variance:" << eig.eigenvalues()[0].norm()/total_eigen.norm() 

  		
  		/*
  		//================ Video Simulation ================//
		for(int i=0;i<8;i++){ // Iterate for all 8 target positions 
	 		ystar = target.segment(i*3,3);
	 		if (i<1){
				qcurrent = qstart1; // set initial pose
	 		}
			yinit = bax.GetIK(qcurrent); // set starting position
	 		for(int t=1;t<20;t++){ // divide the target to several steps
				y=bax.GetIK(qcurrent);
				J=bax.GetJ(qcurrent);
				yinter = yinit.segment(0,3) + ((t/20)*(ystar-yinit.segment(0,3)))
				Eigen::MatrixXd J_pos_right = J.block(0,0,3,7); // Get position Jacobian of the right arm (a 3x7 block at row 0 and column 0)
	  			Eigen::MatrixXd Jinv = Winv*J_pos_right.transpose()*(J_pos_right*Winv*J_pos_right.transpose()+Cinv).inverse(); // Compute Inverse Jacobian
	  			qcurrent.segment(0,7) = qcurrent.segment(0,7) + Jinv*(yinter-y.segment(0,3))+(I-Jinv*J_pos_right)*(q_comf1.segment(0,7)-qcurrent.segment(0,7)); //use qcomf_1
	  			bax.SetJointAngles(qcurrent);
	  			// Update simulation
      			bax.AdvanceSimulation(); 
	 		}
		}*/
	}
  	// Stop simulation and close connection
  	bax.StopSimulation();
  	return(0);
}
