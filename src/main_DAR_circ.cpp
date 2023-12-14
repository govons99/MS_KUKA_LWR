#include<controller/controller_kuka.hpp>
#include<random>


int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;
	int							ResultValue		=	0;
	double 						Time = 0.0;
	double						frequency = 1.0;
	double 						tf = 10.0;

	std::chrono::time_point<std::chrono::system_clock> Tic, Toc;

	double elapsed_time;

	unsigned int TimeoutValueInMicroSeconds = 0;

	Kuka_Vec Q0;

	Kuka_Vec dQ0 = Kuka_Vec::Constant(NUMBER_OF_JOINTS,0.0);

	Kuka_Vec Q_ref;

	Kuka_Vec dQ_ref;

	Kuka_Vec d2Q_ref;

	Kuka_Vec Q_meas;

	Kuka_Vec Q_meas_old;

	Kuka_Vec dQ_meas; //obtained from the measured positions

	Kuka_Vec dQ_meas_old;

	Kuka_Vec d2Q_meas; //obtained from the measured positions

	Kuka_Vec G;

	Kuka_Vec Torques_ref;

	Kuka_Vec Torques_nom;

	Kuka_Vec Torques_measured;

	Kuka_Vec Torques_filtered;

	Kuka_Vec torques_temp;

	Kuka_Vec temp_Vec;

	Kuka_Vec Q_filtered;

	Kuka_Vec dQ_filtered;

	Kuka_Vec d2Q_filtered;

	std::vector<Kuka_Vec> Q_ref_vec;

	std::vector<Kuka_Vec> dQ_ref_vec;

	std::vector<Kuka_Vec> d2Q_ref_vec;

	std::vector<Kuka_Vec> Temp_array;
	
	Kuka_Vec Kuka_temp;

	Kuka_Vec Kuka_temp2;

	std::vector<Eigen::VectorXd> temp;

	Kuka_Mat Mass;

	//std::string Mode("impedence");
	
	std::string Mode("position");
	
	std::string qsave = "Q.txt";
  	std::string dqsave = "dQ.txt";
	std::string d2qsave = "d2Q.txt";
	std::string qsave_filtered = "Q_filtered.txt";
  	std::string dqsave_filtered = "dQ_filtered.txt";
	std::string d2qsave_filtered = "d2Q_filtered.txt";
	std::string dqsave_meas = "dQ_meas.txt";
	std::string torque_meas = "tor_meas.txt";
	std::string torque_th = "tor_th.txt";
	std::string friction = "friction.txt";
	std::string Q_ref_file = "Qref.txt";
	std::string dQ_ref_file = "dQref.txt";
	std::string d2Q_ref_file = "d2Qref.txt";
	std::string qhatsave = "Q_hat.txt";
	std::string dqhatsave = "dQ_hat.txt";
	std::string dqnumsave = "dQ_num.txt";
	std::string torque_save = "torque_ref.txt";

	Kuka_State state;

	bool FLAG = ROBOT_CONTROL;

	controller_kuka Controller(Mode,FLAG);

	if (Controller.FRI->IsMachineOK())
	{
		Controller.MeasureJointPositions();

		for(int i=0;i<NUMBER_OF_JOINTS;i++)
		{
			std::cout<<Controller.JointValuesInRad[i] <<"\n";
			Controller.Q(i) = Controller.JointValuesInRad[i];
			Controller.Qold(i) = Controller.Q(i);
			Q0(i) = Controller.Q(i);
		}
	}

	std::cout << "RUN TIME" << RUN_TIME_IN_SECONDS << "\n";	

	Controller.Qold = Controller.Q;

	Q_meas = Q0;

	dQ_meas = Kuka_Vec::Constant(0.0);

	d2Q_meas = Kuka_Vec::Constant(0.0);

	Q_filtered = Kuka_Vec::Constant(0.0);

	dQ_filtered = Kuka_Vec::Constant(0.0);

	d2Q_filtered = Kuka_Vec::Constant(0.0);

	//Variables for the full state observer

	Kuka_Vec y_tilda = Kuka_Vec::Constant(0.0);
	Kuka_Vec dx1_hat = Kuka_Vec::Constant(0.0);
	Kuka_Vec d2Q_hat = Kuka_Vec::Constant(0.0);

	//Initialization estimated varibales
            
	//Observer
	Controller.Q_hat = Controller.Q;
	Controller.dQ_hat = Kuka_Vec::Constant(0.0);

	Controller.Q_hat_save.push_back(Controller.Q_hat);
    Controller.dQ_hat_save.push_back(Controller.dQ_hat);

	//Initial generalized momentum

	Mass = Controller.GetMass(Controller.Q);
	Controller.p0 = Mass*Controller.dQ;
	Controller.p0_hat = Mass*Controller.dQ_hat;

	//Variables for the 2R planar robot

	double m1 = 2.6;
	double m2 = 2.6;
	double l1 = 0.5;
	double l2 = 0.5;

	Eigen::Matrix<double, 2, 2> Mass_2R;
	Eigen::Matrix<double, 2, 1> Coriolis_2R;
	Eigen::Matrix<double, 2, 1> Gravity_2R;

	// horizontal plane
	/*
	Eigen::Matrix<double, 4, 1> State_2R = {Controller.Q(0), Controller.Q(3), 0, 0};
	Eigen::Matrix<double, 2, 1> acc_2R {0, 0};
	Eigen::Matrix<double, 2, 1> vel_2R {0, 0};
	Eigen::Matrix<double, 2, 1> pos_2R {Controller.Q(0), Controller.Q(3)};
	*/
	// vertical plane
	Eigen::Matrix<double, 4, 1> State_2R = {Controller.Q(1), Controller.Q(3), 0, 0};
	Eigen::Matrix<double, 2, 1> acc_2R {0, 0};
	Eigen::Matrix<double, 2, 1> vel_2R {0, 0};
	Eigen::Matrix<double, 2, 1> pos_2R {Controller.Q(1), Controller.Q(3)};

	// old values
	/*
	Eigen::Matrix<double, 2, 4> PD_Gain {
		{-21.4267, -0.8640, -17.2654, -1.0133},
		{-4.8761, -17.7497, -3.9620, -12.6837},
	};
	*/

	// 25/10
	/*
	Eigen::Matrix<double, 2, 4> PD_Gain {
		{-93.1726, -8.4440, -21.5335, -4.7909},
		{-17.7032, -34.4521, -6.6227, -20.5367},
	};
	*/

	// Gain without uncertainties and no gravity
	/*
	Eigen::Matrix<double, 2, 4> PD_Gain;

	PD_Gain(0,0) = -93.1726;
	PD_Gain(0,1) = -8.4440;
	PD_Gain(0,2) = -21.5335;
	PD_Gain(0,3) = -4.7909;
	PD_Gain(1,0) = -17.7032;
	PD_Gain(1,1) = -34.4521;
	PD_Gain(1,2) = -6.6227;
	PD_Gain(1,3) = -20.5367;
	*/ 

	// Gain with uncertain mass and no gravity
	/*
	Eigen::Matrix<double, 2, 4> PD_Gain;

	PD_Gain(0,0) = -206.7802;
	PD_Gain(0,1) = 25.1567;
	PD_Gain(0,2) = -162.2236;
	PD_Gain(0,3) = 2.4361;
	PD_Gain(1,0) = -20.1485;
	PD_Gain(1,1) = -220.7650;
	PD_Gain(1,2) = -5.0760;
	PD_Gain(1,3) = -131.8218;
	*/

	// Gain with gravity
	/*
	Eigen::Matrix<double, 2, 4> PD_Gain;

	PD_Gain(0,0) = 4976;
	PD_Gain(0,1) = 6267;
	PD_Gain(0,2) = 113;
	PD_Gain(0,3) = 769;
	PD_Gain(1,0) = -20189;
	PD_Gain(1,1) = -19397;
	PD_Gain(1,2) = -1424;
	PD_Gain(1,3) = -2541;
	*/

	// OUTPUT REGULATION

	// REFERENCE TRAJECTORY: ref = [a0 + a1*cos(t*f) + b1*sin(t*f);a0 + a1*cos(t*f) + b1*sin(t*f)]
	// circular trajectory in the task space

	// Variables trajectory

	double a01 = 0.1444;
	double a11 = -0.4403;
	double b11 = -0.2636;
	//double a21 = 0.0815;
	//double b21 = -0.0143;
	double f1 = 5;

	double a02 = 0.9537;
	double a12 = 0.3598;
	double b12 = -0.2501;
	//double a22 = -0.0000;
	//double b22 = 0.0291;
	double f2 = 5;

	double w1 = 0.0;
	double w2 = 0.0;

	// Variables for internal model

	Eigen::MatrixXd eta = Eigen::MatrixXd::Zero(6,1);
	Eigen::MatrixXd eta_dot = Eigen::MatrixXd::Zero(6,1);

	Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(6,6);

	Phi.topRightCorner(4,4).setIdentity();
	Phi(4,2) = -f1*f1;
	Phi(5,3) = -f1*f1;

	Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(6,2);
	Gamma(4,0) = 1;
	Gamma(5,1) = 1;

	// Gain for output regulation

	Eigen::Matrix<double, 2, 10> Theta;	         

	Theta(0,0) = -51479.1506564270;             
	Theta(0,1) = 3568.82244932025;
	Theta(0,2) = -11854.7020446130;
	Theta(0,3) = 1079.87266377154;
	Theta(0,4) = -633338.094409722;
	Theta(0,5) = 4333.26737636959;
	Theta(0,6) = 508505.390501465;
	Theta(0,7) = -47862.5287158016;
	Theta(0,8) = -98358.8518500649;
	Theta(0,9) = 6135.56008409700;	         

	Theta(1,0) = 1263.68859842101;            
	Theta(1,1) = -44137.3005591795;
	Theta(1,2) = 480.698288000380;
	Theta(1,3) = -10896.5985110252;
	Theta(1,4) = -9654.94397920896;
	Theta(1,5) = -440771.759801160;
	Theta(1,6) = -20520.6564484375;
	Theta(1,7) = 470082.073219219;
	Theta(1,8) = 1813.31207683121;
	Theta(1,9) = -82035.0166304382;

	Eigen::Matrix<double, 10, 1> state_output;

	Eigen::Matrix<double, 2, 1> error;

	Eigen::Matrix<double, 2, 1> control {0, 0};

	// virtual saturation
	double th = 15000;

	double DELTAT_2R = 0.0001;

	double Time_2R = 0.0;

	// file for the evolution of the internal model

	std::string eta_save = "eta.txt";

	std::ofstream file(eta_save);

	std::string error_save = "error.txt";

	std::ofstream file_error(error_save);

	// PD Gains

	double P_Gain = 50.0;
	double D_Gain = 10.0;
	
	//SIMULATION LOOP
	while ((float)CycleCounter * Controller.FRI->GetFRICycleTime() < RUN_TIME_IN_SECONDS)
	{		

		Time = Controller.FRI->GetFRICycleTime() * (float)CycleCounter;

		//w1 = a01 + a11*cos(Time_2R*f1) + b11*sin(Time_2R*f1) + a21*cos(2*Time_2R*f1) + b21*sin(2*Time_2R*f1);
		//w2 = a02 + a12*cos(Time_2R*f2) + b12*sin(Time_2R*f2) + a22*cos(2*Time_2R*f2) + b22*sin(2*Time_2R*f2);
		
		w1 = a01 + a11*cos(Time_2R*f1) + b11*sin(Time_2R*f1);
		w2 = a02 + a12*cos(Time_2R*f2) + b12*sin(Time_2R*f2);

		error(0) = Controller.Q(1)-w1;
		error(1) = Controller.Q(3)-w2;

		std::cout << "error \n" << error << std::endl;

		file_error << error.transpose();

		file_error << "\n";

		Time_2R += DELTAT_2R;

		// INTERNAL MODEL

		eta_dot = Phi * eta + Gamma * error;

		eta = eta + DELTAT_2R * eta_dot;
		
		file << eta.transpose();

		file << "\n";

		// 2R SIMULATION

		Mass_2R(0,0) = std::pow(2,l2)*m2+2*l1*l2*m2*cos(State_2R(1))+std::pow(2,l1)*(m1+m2);
		Mass_2R(0,1) = std::pow(2,l2)*m2+l1*l2*m2*cos(State_2R(1));
		Mass_2R(1,0) = std::pow(2,l2)*m2+l1*l2*m2*cos(State_2R(1));
		Mass_2R(1,1) = std::pow(2,l2)*m2;

		Coriolis_2R(0) = -m2*l1*l2*sin(State_2R(1))*std::pow(2,State_2R(3))-2*m2*l1*l2*sin(State_2R(1))*State_2R(2)*State_2R(3);
		Coriolis_2R(1) = m2*l1*l2*sin(State_2R(1))*std::pow(2,State_2R(2));

		Gravity_2R(0) = cos(State_2R(0)+State_2R(1))*m2*9.81*l2 + cos(State_2R(0))*(m1+m2)*l1*9.81;
		Gravity_2R(1) = cos(State_2R(0)+State_2R(1))*m2*9.81*l2;

		state_output(0) = State_2R(0);
		state_output(1) = State_2R(1);
		state_output(2) = State_2R(2);
		state_output(3) = State_2R(3);
		state_output(4) = eta(0);
		state_output(5) = eta(1);
		state_output(6) = eta(2);
		state_output(7) = eta(3);
		state_output(8) = eta(4);
		state_output(9) = eta(5);

		//control = PD_Gain * State_2R;
		control = Theta * state_output;

		if ( control(0) >= th )
		{
			control(0) = th;
			std::cout << "-------------------------------" << std::endl;
		}
		if ( control(1) >= th )
		{
			control(1) = th;
			std::cout << "-------------------------------" << std::endl;
		}
		if ( control(0) <= -th )
		{
			control(0) = -th;
			std::cout << "-------------------------------" << std::endl;
		}
		if ( control(1) <= -th )
		{
			control(1) = -th;
			std::cout << "-------------------------------" << std::endl;
		}

		//acc_2R = Mass_2R.inverse()*(control - Coriolis_2R);
		acc_2R = Mass_2R.inverse()*(control - Coriolis_2R - Gravity_2R);

		vel_2R = vel_2R + DELTAT_2R * acc_2R;

		pos_2R = pos_2R + DELTAT_2R * vel_2R;

		State_2R(0) = pos_2R(0);
		State_2R(1) = pos_2R(1);
		State_2R(2) = vel_2R(0);
		State_2R(3) = vel_2R(1);

		// Assigning the values of the 2R to the respective joints of the KUKA

		Q_ref = Q0;

		// Planar vertical 2R
		Q_ref(1) = pos_2R(0);
		Q_ref(3) = pos_2R(1);

		/*
		Q_ref = Q0 + Kuka_Vec::Constant(0.2*(1.0 - std::cos(Time)));
		
		dQ_ref = Kuka_Vec::Constant(0.2*std::sin(Time));
		
		d2Q_ref = Kuka_Vec::Constant(0.2*std::cos(Time));
		
		Q_ref(0) = Q0(0);
		Q_ref(2) = Q0(2);
		Q_ref(4) = Q0(4);
		Q_ref(5) = Q0(5);
		Q_ref(6) = Q0(6);
		
		dQ_ref(4) = 0.0;
		dQ_ref(5) = 0.0;
		dQ_ref(6) = 0.0;

		d2Q_ref(4) = 0.0;
		d2Q_ref(5) = 0.0;
		d2Q_ref(6) = 0.0;
		*/		

		Tic = std::chrono::system_clock::now();

		Controller.FRI->WaitForKRCTick(TimeoutValueInMicroSeconds);

		Toc = std::chrono::system_clock::now();

		if (!Controller.FRI->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
			break;
		}

		state = Controller.GetState(FLAG);

		// FULL-STATE PBSERVER: Nicosia-Tomei

		/*
		y_tilda = Controller.Q - Controller.Q_hat;

		dx1_hat = Controller.dQ_hat + Controller.kd*y_tilda;

		d2Q_hat = Controller.SimObserver(Controller.Q, y_tilda, dx1_hat, Torques_nom);

		Controller.Q_hat = Controller.EulerIntegration(dx1_hat, Controller.Q_hat);

		Controller.dQ_hat = Controller.EulerIntegration(d2Q_hat, Controller.dQ_hat);
		*/	

		Controller.Qsave.push_back(Controller.Q);

		Controller.dQsave.push_back(Controller.dQ);	

		// UPDATING COORDINATES

		Controller.Qold = Controller.Q;

		Controller.dQold = Controller.dQ;

		Controller.dQ = Controller.EulerIntegration(Controller.d2Q,Controller.dQ);

		Controller.Q = Controller.EulerIntegration(Controller.dQ,Controller.Q);

		Controller.dQ_num = Controller.EulerDifferentiation(Controller.Q,Controller.Qold);
		
		Controller.GetState(FLAG); 

		//ARRAY SAVING

		Q_filtered = Controller.Filter(Controller.Qsave,FILTER_LENGTH);

		dQ_filtered = Controller.Filter(Controller.dQsave,FILTER_LENGTH);

		Controller.d2Q = Controller.EulerDifferentiation(dQ_filtered,Controller.dQsave_filtered.back());

		Controller.d2Qsave.push_back(Controller.d2Q);

		d2Q_filtered = Controller.Filter(Controller.d2Qsave,FILTER_LENGTH);

		Controller.Qsave_filtered.push_back(Q_filtered);
		
		Controller.dQsave_filtered.push_back(dQ_filtered);
		
		Controller.d2Qsave_filtered.push_back(d2Q_filtered);

		// Saving state reference values

		Q_ref_vec.push_back(Q_ref);

		dQ_ref_vec.push_back(dQ_ref);

		d2Q_ref_vec.push_back(d2Q_ref);

		std::cout << "Time = " << Time << std::endl;

		//SAFETY CHECK FOR POSITIONS AND VELOCITIES
		
		if((!Controller.VelocitySafety(Controller.dQ)) || (!Controller.JointSafety(Controller.Q)))
		{	
			//EXITING THE CONTROL LOOP
			std::cout << "Breaking safety controllers for either Velocities and Joints position \n";
			break;
		}

		// POSITION CONTROL

		for(int y=0;y<NUMBER_OF_JOINTS;y++)
		{
			Controller.JointValuesInRad[y] = Q_ref(y);
		}
		
		Controller.FRI->SetCommandedJointPositions(Controller.JointValuesInRad);

		//noisy_torque_vec.push_back(Torques_measured);

		Controller.Tor_th.push_back(Torques_ref);

		// REDUCED OBSERVER

		Controller.dz = Controller.SimReducedObserver(Controller.Q, Controller.dQ_hat, Torques_nom);

		Controller.z = Controller.EulerIntegration(Controller.dz, Controller.z);

		Controller.dQ_hat = Controller.z + Controller.k0*Controller.Q;

		// Saving estimated quantities

		Controller.dQ_hat_save.push_back(Controller.dQ_hat);

		Controller.Q_hat_save.push_back(Controller.Q_hat);

		CycleCounter++;
	}

	fprintf(stdout, "Stopping the robot...\n");

	ResultValue	= Controller.FRI->StopRobot();

	if (ResultValue != EOK)
	{
		fprintf(stderr, "An error occurred during stopping the robot...\n");
	}
	else
	{
		fprintf(stdout, "Robot successfully stopped.\n");
	}

	//Writing Experiment Variables

	file.close();

	file_error.close();

	Controller.FromKukaToDyn(temp,Controller.Qsave);
	Controller.writer.write_data(qsave,temp);
	
	Controller.FromKukaToDyn(temp,Controller.dQsave);
	Controller.writer.write_data(dqsave,temp);

	Controller.FromKukaToDyn(temp,Controller.d2Qsave);
	Controller.writer.write_data(d2qsave,temp);

	Controller.FromKukaToDyn(temp,Controller.dQ_hat_save);
	Controller.writer.write_data(dqhatsave,temp);

	Controller.FromKukaToDyn(temp,Controller.Q_hat_save);
	Controller.writer.write_data(qhatsave,temp);

	Controller.FromKukaToDyn(temp,Controller.dQ_num_save);
	Controller.writer.write_data(dqnumsave,temp);	

	Controller.FromKukaToDyn(temp,Controller.Tor_th);
	Controller.writer.write_data(torque_save,temp);	

	Controller.FromKukaToDyn(temp,Q_ref_vec);
	Controller.writer.write_data(Q_ref_file,temp);

	Controller.FromKukaToDyn(temp,dQ_ref_vec);
	Controller.writer.write_data(dQ_ref_file,temp);

	Controller.FromKukaToDyn(temp,d2Q_ref_vec);
	Controller.writer.write_data(d2Q_ref_file,temp);

	//"Measured" velocities

	Controller.FromKukaToDyn(temp,Controller.dQsave_meas);
	Controller.writer.write_data(dqsave_meas,temp);

	//Filtered variables

	Controller.FromKukaToDyn(temp,Controller.Qsave_filtered);
	Controller.writer.write_data(qsave_filtered,temp);

	Controller.FromKukaToDyn(temp,Controller.dQsave_filtered);
	Controller.writer.write_data(dqsave_filtered,temp);

	Controller.FromKukaToDyn(temp,Controller.d2Qsave_filtered);
	Controller.writer.write_data(d2qsave_filtered,temp);
		

return 0;
}
