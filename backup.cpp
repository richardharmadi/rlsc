
/*
	 		// Iterating inverse kinematic algorithm
	 		qcurrent = qstart1; // starting position 1
	 		qprev = qcurrent + eps;
		
	 		while ((qcurrent-qprev).norm() > e){
	  			qprev = qcurrent;
	  			y=bax.GetIK(qcurrent); // Get end-effector position
	  			J=bax.GetJ(qcurrent);  // Get Jacobian of the end effector
	  			Eigen::MatrixXd J_pos_left = J.block(6,7,3,7); // Get position Jacobian of the left arm (a 3x7 block at row 6th and column 7th)
	  			Eigen::MatrixXd Jinv = Winv*J_pos_left.transpose()*(J_pos_left*Winv*J_pos_left.transpose()+Cinv).inverse(); // Compute Inverse Jacobian
	  			if (i<8){
	   				qcurrent.segment(9,7) = qcurrent.segment(9,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_left)*(q_comf1.segment(9,7)-qcurrent.segment(9,7)); //use qcomf_1
	 			}else{
	   				qcurrent.segment(9,7) = qcurrent.segment(9,7) + Jinv*(ystar-y.segment(0,3))+(I-Jinv*J_pos_left)*(q_comf2.segment(9,7)-qcurrent.segment(9,7)); //use qcomf_2
	  			}	  
	  			bax.SetJointAngles(qcurrent);
	  			// Update simulation
      			bax.AdvanceSimulation(); 
	 		}



//================ Video Simulation ================//
		qfinal = qstart1; // first starting position
	
		for(int i=0;i<8;i++){ // Iterate for all 8 target positions 
	 		ystar_final = target.segment(i*3,3);
	 		add = ystar_final*0.1;
			std::cout << "add" << add << "\n";
	 		ystar = add;
	 		bool r = ystar.isApprox(ystar_final); // while we haven't reach the original target
	 		while(!r){
	 	 		qcurrent = qfinal; // set the final position (last target) as the starting position
		 		qprev = qcurrent + eps;

		 		while ((qcurrent-qprev).norm() > e){
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
		 		qfinal = qcurrent; // set the final current position (which assumed to be our target), as the final position
		 		ystar+=add; // keep add ystar with small value
		 		r = ystar.isApprox(ystar_final)
	 		}
		}*/