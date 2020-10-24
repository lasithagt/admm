#include "kuka_arm_track.h"


KukaArm_TRK::KukaArm_TRK(){}

//const char* const kIiwaUrdf =
//    "drake/manipulation/models/iiwa_description/urdf/"
//    "iiwa14_no_collision.urdf";

const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf";
// using manipulation::planner::ConstraintRelaxingIk;
// using manipulation::kuka_iiwa::kIiwaArmNumJoints;
// using manipulation::util::WorldSimTreeBuilder;
// using manipulation::util::ModelInstanceInfo;
// using systems::RigidBodyPlant;

// const char* const kIiwaUrdf =
//     "drake/manipulation/models/iiwa_description/urdf/"
//     "iiwa7.urdf";

// Add Schunk and Kuka_connector
// const char* kIiwaUrdf = "drake/manipulation/models/iiwa_description/urdf/iiwa7.urdf";
// const char* schunkPath = "drake/manipulation/models/wsg_50_description/urdf/wsg_50_mesh.urdf";
// const char* connectorPath = "drake/manipulation/models/kuka_connector_description/urdf/KukaConnector.urdf";

// iiwa_dt = time step
// iiwa_N = number of knots
// iiwa_xgoal = final goal in state space (7pos, 7vel)
KukaArm_TRK::KukaArm_TRK(double& iiwa_dt, unsigned int& iiwa_N, stateVec_t& iiwa_xgoal)
{
    //#####
    globalcnt = 0;
    //#####
    stateNb = 14;
    commandNb = 7;
    dt = iiwa_dt;
    N = iiwa_N;
    xgoal = iiwa_xgoal;
    fxList.resize(N);
    fuList.resize(N);

    fxxList.resize(stateSize);
    for(unsigned int i=0;i<stateSize;i++)
        fxxList[i].resize(N);
    fxuList.resize(commandSize);
    fuuList.resize(commandSize);
    for(unsigned int i=0;i<commandSize;i++){
        fxuList[i].resize(N);
        fuuList[i].resize(N);
    }

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    lowerCommandBounds << -50.0;
    upperCommandBounds << 50.0;

    H.setZero();
    C.setZero();
    G.setZero();
    Bu.setZero();
    velocity.setZero();
    accel.setZero();
    Xdot_new.setZero();
    // Xdot_new_thread.resize(NUMBER_OF_THREAD);
    // vd_thread.resize(NUMBER_OF_THREAD);

    A1.setZero();
    A2.setZero();
    A3.setZero();
    A4.setZero();
    B1.setZero();
    B2.setZero();
    B3.setZero();
    B4.setZero();
    IdentityMat.setIdentity();

    Xp1.setZero();
    Xp2.setZero();
    Xp3.setZero();
    Xp4.setZero();

    Xm1.setZero();
    Xm2.setZero();
    Xm3.setZero();
    Xm4.setZero();

    AA.setZero();
    BB.setZero();
    A_temp.resize(N);
    B_temp.resize(N);
    
    debugging_print = 0;
    finalTimeProfile.counter0_ = 0;
    finalTimeProfile.counter1_ = 0;
    finalTimeProfile.counter2_ = 0;

    initial_phase_flag_ = 1;
    q.resize(stateSize/2);
    qd.resize(stateSize/2);
    // q_thread.resize(NUMBER_OF_THREAD);
    // qd_thread.resize(NUMBER_OF_THREAD);
    // for(unsigned int i=0;i<NUMBER_OF_THREAD;i++){
    //     q_thread[i].resize(stateSize/2);
    //     qd_thread[i].resize(stateSize/2);
    // }

    finalTimeProfile.time_period1 = 0;
    finalTimeProfile.time_period2 = 0;
    finalTimeProfile.time_period3 = 0;
    finalTimeProfile.time_period4 = 0;

    // if(initial_phase_flag_ == 1){
    //     // Ye's original method
    //     robot_thread_ = std::make_unique<RigidBodyTree<double>>();

    //     parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    //         FindResourceOrThrow(kIiwaUrdf),
    //     multibody::joints::kFixed, robot_thread_.get());
        
    //     initial_phase_flag_ = 0;
    // }
}

KukaArm_TRK::KukaArm_TRK(double& iiwa_dt, unsigned int& iiwa_N, stateVec_t& iiwa_xgoal, ContactModel::SoftContactModel& contact_model,std::vector<Eigen::Matrix<double,6,1> >& iiwa_fk_ref)
{
    //#####
    globalcnt = 0;
    //#####
    stateNb = 14;
    commandNb = 7;
    dt = iiwa_dt;
    N = iiwa_N;
    xgoal = iiwa_xgoal;
    fxList.resize(N);
    fuList.resize(N);

    fxxList.resize(stateSize);
    for(unsigned int i=0;i<stateSize;i++)
        fxxList[i].resize(N);
    fxuList.resize(commandSize);
    fuuList.resize(commandSize);
    for(unsigned int i=0;i<commandSize;i++){
        fxuList[i].resize(N);
        fuuList[i].resize(N);
    }

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    lowerCommandBounds << -50.0, -50, -50, -50, -50, -50, -50;
    upperCommandBounds << 50.0, 50, 50, 50, 50, 50, 50;

    H.setZero();
    C.setZero();
    G.setZero();
    Bu.setZero();
    velocity.setZero();
    accel.setZero();
    Xdot_new.setZero();
    // Xdot_new_thread.resize(NUMBER_OF_THREAD);
    // vd_thread.resize(NUMBER_OF_THREAD);

    A1.setZero();
    A2.setZero();
    A3.setZero();
    A4.setZero();
    B1.setZero();
    B2.setZero();
    B3.setZero();
    B4.setZero();
    IdentityMat.setIdentity();

    Xp1.setZero();
    Xp2.setZero();
    Xp3.setZero();
    Xp4.setZero();

    Xm1.setZero();
    Xm2.setZero();
    Xm3.setZero();
    Xm4.setZero();

    AA.setZero();
    BB.setZero();
    A_temp.resize(N);
    B_temp.resize(N);
    
    debugging_print = 0;
    finalTimeProfile.counter0_ = 0;
    finalTimeProfile.counter1_ = 0;
    finalTimeProfile.counter2_ = 0;

    initial_phase_flag_ = 1;
    q.resize(stateSize/2);
    qd.resize(stateSize/2);
    // q_thread.resize(NUMBER_OF_THREAD);
    // qd_thread.resize(NUMBER_OF_THREAD);
    // for(unsigned int i=0;i<NUMBER_OF_THREAD;i++){
    //     q_thread[i].resize(stateSize/2);
    //     qd_thread[i].resize(stateSize/2);
    // }

    finalTimeProfile.time_period1 = 0;
    finalTimeProfile.time_period2 = 0;
    finalTimeProfile.time_period3 = 0;
    finalTimeProfile.time_period4 = 0;

    // if(initial_phase_flag_ == 1){
    //     robot_thread_ = std::move(totalTree_);        

    //     initial_phase_flag_ = 0;
    // }
}

KukaArm_TRK::KukaArm_TRK(double& iiwa_dt, unsigned int& iiwa_N, stateVec_t& iiwa_xgoal, stateVecTab_t iiwa_xtrack, std::unique_ptr<KUKAModelKDL>& kukaRobot, ContactModel::SoftContactModel& contact_model, std::vector<Eigen::Matrix<double,6,1> >& iiwa_fk_ref)
{
    //#####
    globalcnt = 0;
    //#####
    if(SOFT_CONTACT){
        stateNb = 20;
        q.resize((stateSize-3)/2);
        qd.resize((stateSize-3)/2);
    }
    else
    {
        stateNb = 14;
        q.resize(stateSize/2);
        qd.resize(stateSize/2);
    }
    commandNb = 7;
    dt = iiwa_dt;
    N = iiwa_N;
    xgoal = iiwa_xgoal;
    fk_ref = iiwa_fk_ref;
    xtrack = iiwa_xtrack;

    contact_model0 = &contact_model;
    fxList.resize(N);
    fuList.resize(N);

    fxxList.resize(stateSize);
    for(unsigned int i=0;i<stateSize;i++)
        fxxList[i].resize(N);
    fxuList.resize(commandSize);
    fuuList.resize(commandSize);
    for(unsigned int i=0;i<commandSize;i++){
        fxuList[i].resize(N);
        fuuList[i].resize(N);
    }

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    lowerCommandBounds << -50.0, -50, -50, -50, -50, -50, -50;
    upperCommandBounds << 50.0, 50, 50, 50, 50, 50, 50;


    H.setZero();
    C.setZero();
    G.setZero();
    Bu.setZero();
    velocity.setZero();
    accel.setZero();
    Xdot_new.setZero();
    // Xdot_new_thread.resize(NUMBER_OF_THREAD);
    // vd_thread.resize(NUMBER_OF_THREAD);

    A1.setZero();
    A2.setZero();
    A3.setZero();
    A4.setZero();
    B1.setZero();
    B2.setZero();
    B3.setZero();
    B4.setZero();
    IdentityMat.setIdentity();

    Xp1.setZero();
    Xp2.setZero();
    Xp3.setZero();
    Xp4.setZero();

    Xm1.setZero();
    Xm2.setZero();
    Xm3.setZero();
    Xm4.setZero();

    AA.setZero();
    BB.setZero();
    A_temp.resize(N);
    B_temp.resize(N);
    
    debugging_print = 0;
    finalTimeProfile.counter0_ = 0;
    finalTimeProfile.counter1_ = 0;
    finalTimeProfile.counter2_ = 0;

    initial_phase_flag_ = 1;
    // q_thread.resize(NUMBER_OF_THREAD);
    // qd_thread.resize(NUMBER_OF_THREAD);
    // for(unsigned int i=0;i<NUMBER_OF_THREAD;i++){
    //     q_thread[i].resize(stateSize/2);
    //     qd_thread[i].resize(stateSize/2);
    // }

    finalTimeProfile.time_period1 = 0;
    finalTimeProfile.time_period2 = 0;
    finalTimeProfile.time_period3 = 0;
    finalTimeProfile.time_period4 = 0;

    if(initial_phase_flag_ == 1){
        // Ye's original method
        // robot_thread_ = std::make_unique<RigidBodyTree<double>>();

        kukaRobot_    = std::move(kukaRobot);      
        initial_phase_flag_ = 0;
    }
}

stateVec_t KukaArm_TRK::kuka_arm_dynamics(const stateVec_t& X, const commandVec_t& tau)
{
    // struct timeval tbegin_dynamics, tend_dynamics;
    // gettimeofday(&tbegin_dynamics,NULL);
    finalTimeProfile.counter0_ += 1;

    if(finalTimeProfile.counter0_ == 10)
        gettimeofday(&tbegin_period,NULL);

    
    
    if (WHOLE_BODY)
    {
        if (SOFT_CONTACT)
        {
            //LW
            // q << X.head((stateSize-3)/2);
            // qd << X.segment((stateSize-3)/2, (stateSize-3)/2);
            // force_current << X.tail(3);

            q = X.head((stateSize-3)/2);
            qd = X.segment((stateSize-3)/2, (stateSize-3)/2);
            force_current = X.tail(3);

            //LW-Test

            Eigen::Matrix<double,(stateSize-3)/2+2,1> q_full;
            Eigen::Matrix<double,(stateSize-3)/2+2,1> qd_full;
            Eigen::Matrix<double,(stateSize-3)/2+2,1> vd_full;
            q_full.setZero();
            qd_full.setZero();
            vd_full.setZero();
            q_full.topRows((stateSize-3)/2)=q;
            qd_full.topRows((stateSize-3)/2)=qd;
            // VectorX<double> q_0 = VectorX<double>::Zero(9);
            // VectorX<double> qd_0 = VectorX<double>::Zero(9);
            // VectorX<double> qd_0 = VectorX<double>::Zero(9);

            //    KinematicsCache<double> cache_ = robot_thread_->doKinematics(q, qd);

            /* ----------------------------- get fk kinematic ---------------------------*/ 
            // KinematicsCache<double> cache_ = robot_thread_->CreateKinematicsCache();
            // cache_.initialize(q_full,qd_full);
            // robot_thread_->doKinematics(cache_, true);
            // ---------------------------------------------------------------------------

            //const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
            //gettimeofday(&tbegin_period,NULL);

            /* ----------------------------Mass -----------------------------------------*/
            // MatrixX<double> M_ = robot_thread_->massMatrix(cache_); // Inertial matrix
            // --------------------------------------------------------------------------

            // cout << M_ << endl;
            //gettimeofday(&tend_period,NULL);
            //finalTimeProfile.time_period2 += ((double)(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;

            //gettimeofday(&tbegin_period,NULL);

            /* ------------------------------- fxt -------------------------------------------*/
            // drake::WrenchVector<double> tw;
            // // tw  <<  0,0,0,0,0,100;
            // tw  <<  force_current;         
            // RigidBody<double> const & rb = (*robot_thread_).get_body(10); 
            // drake::eigen_aligned_std_unordered_map< RigidBody<double> const*, drake::TwistVector<double> > f_ext;
            // f_ext[&rb] = tw;
            //----------------------------------------------------------------------------     

            // drake::eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::TwistVector<double>> f_ext;
            //gettimeofday(&tend_period,NULL);
            //finalTimeProfile.time_period3 += ((double)(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
            
            //gettimeofday(&tbegin_period,NULL);
            
            // VectorX<double> bias_term_ = robot_thread_->dynamicsBiasTerm(cache_, f_ext);  // Bias term: M * vd + h = tau + J^T * lambda
            // -------------------------------------------------------------------------------------------------------------------
            // VectorX<double> bias_term_ = robot_thread_->dynamicsBiasTerm(cache_, f_ext);  // Bias term: M * vd + h = tau + J^T * lambda
            //-----------------------------------------------------------------------------------------------------------------------------
            //    VectorX<double> bias_term_2 = robot_thread_->dynamicsBiasTerm(cache_, f_ext, false);
            //gettimeofday(&tend_period,NULL);
            //finalTimeProfile.time_period4 += ((double)(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;

            //=============================================
            //Set false for doing only gravity comp
            //     VectorX<double> gtau = robot_thread_->inverseDynamics(cache_, f_ext, qd_0, false);
            //=============================================
            /*-------------------------------------------------------------------------*/
            // vd_full = M_.inverse()*(tau - bias_term_);
            /*-------------------------------------------------------------------------*/
            //    vd = M_.inverse()*(tau + bias_term_2 - bias_term_ );
            //    vd = M_.inverse()*(tau - bias_term_ + gtau);
            //    vd = M_.inverse()*(tau + gtau);

            // Get derivative of contact force based on soft contact model
                // FK to get the position and orientation of end-effector

            // ----------------------------------------------------------------------------------------------------
            // const auto &body_pose_fk = robot_thread_->CalcFramePoseInWorldFrame(
            //                     cache_, robot_thread_->get_body(CARTESIAN_FRAME), Isometry3d::Identity());
            // math::RotationMatrix<double> R_init(body_pose_fk.linear());
            // math::RollPitchYaw<double> rpy_init(R_init);
            // Eigen::Matrix<double,6,1> fk_cur;
            // fk_cur << body_pose_fk.translation().x(),
            //     body_pose_fk.translation().y(),
            //     body_pose_fk.translation().z(),
            //     rpy_init.roll_angle(),
            //     rpy_init.pitch_angle(),
            //     rpy_init.yaw_angle();
            
            // Eigen::Vector3d position_cart; 
            // Eigen::Vector3d orientation_cart; 
            // Eigen::Vector3d velocity_cart; 
            // Eigen::Vector3d acceleration_cart;  
            Eigen::Vector3d force_dot;
            // Eigen::Matrix3d mass_cart;

            // position_cart = fk_cur.topRows(3);
            // //orientation = fk_cur.bottomRows(3);
            // orientation_cart << 1, 1, 0;
            // std::vector<int> v_indices;
            // Matrix<double, 6, (stateSize-3)/2+2>  J_geometric;
            // J_geometric = robot_thread_->geometricJacobian(cache_, 0, CARTESIAN_FRAME, CARTESIAN_FRAME, false, &v_indices);
            // mass_cart = (J_geometric * M_ * J_geometric.transpose()).block(0,0,3,3);
            // velocity_cart = (J_geometric * qd_full).head(3); // 6-dim: position and orientation
            // acceleration_cart = (robot_thread_->CalcBodySpatialVelocityJacobianDotTimesVInWorldFrame(cache_, rb) + J_geometric * vd_full).head(3); // 6-dim: position and orientation
            // // TODO: velocity = J(q)*qd
            // //       acceleration = Jd*qd + J*vd
            // //       mass_cart = J*M*J^T
            // contact_model0->df(mass_cart, position_cart, orientation_cart, velocity_cart, acceleration_cart, force_current, force_dot);

            // vd = vd_full.head((stateSize-3)/2);

            // ---------------------------------------------------------

            // dynamics vector
            vd.setZero();
            force_dot.setZero();

            // Xdot_new << qd, vd, force_dot;
            //LW---------------

            force_current.setZero();

            kukaRobot_->getForwardDynamics(q.data(), qd.data(), tau, qdd);
            // std::cout << qdd << std::endl;

            Xdot_new << qd, qdd, force_dot;
            // std::cout << Xdot_new << std::endl;
            // Xdot_new.setZero();

            
            if (finalTimeProfile.counter0_ == 10)
            {
                gettimeofday(&tend_period,NULL);
                finalTimeProfile.time_period1 += (static_cast<double>(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
            }
            
            if ((globalcnt%4 == 0) && (globalcnt<  40)) 
            {
                // cout << "=== q ===" << endl << q << endl;
                // cout << "=== qd ===" << endl << qd << endl;
                // cout << "=== kinematic cache Q ===" << endl<< cache_.getQ() <<endl;
                // cout << "===kinematic cache V ===" << endl<< cache_.getV() <<endl;
                // cout << "=== M ===: " << endl << M_ << endl;
                // cout << "===Bias ==:" << endl << bias_term_ << endl;
                // cout << "=== gtau=== : " << endl << gtau << endl;
                // cout << "=== tau=== : " << endl << tau << endl;
                // cout << "size::" <<  f_ext.size() << endl;
                // for (auto& x: f_ext) {
                //     cout << x.first << ": " << x.second << endl;
                // }
            }

            if (globalcnt < 40) {
                globalcnt += 1;
            }

            // vdot is newly calculated using q, qdot, u
            // (qdot, vdot) = f((q, qdot), u) 
        }
       
    }

    // gettimeofday(&tend_dynamics,NULL);
    // cout << "time for computing qdd " << finalTimeProfile.time_period1 << endl;
    // cout << finalTimeProfile.counter0_ << endl;
    return Xdot_new;
}


KukaArm_TRK::timeprofile KukaArm_TRK::getFinalTimeProfile()
{    
    return finalTimeProfile;
}

scalar_t KukaArm_TRK::forwardkin_cost(stateVec_t x, commandVec_t u, Eigen::Matrix<double,6,1> fkgoal, 
                                        CostFunctionKukaArm_TRK*& costFunction, unsigned int last)
{
    // forward dynamics here xList_curr is a vector of states
    if (SOFT_CONTACT)
    {
        // ---------------------------------------------------------------------------
        Eigen::VectorXd qq = Eigen::VectorXd((stateSize-3)/2+2);
        Eigen::VectorXd qqd = Eigen::VectorXd((stateSize-3)/2+2);
        qq.setZero();
        qqd.setZero();
        qq.topRows((stateSize-3)/2) = x.head((stateSize-3)/2);
        qqd.topRows((stateSize-3)/2) = x.segment((stateSize-3)/2, (stateSize-3)/2);    

        /*---------------------------------------*/
        Eigen::Matrix<double,3,3> poseM;
        Eigen::Vector3d poseP;
        Eigen::Vector3d vel;
        Eigen::Vector3d accel;

        kukaRobot_->getForwardKinematics(qq.data(), qqd.data(), qdd.data(), poseM, poseP, vel, accel, false);

        // ----------------------------------------------------

        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(poseM);
        Eigen::Vector3d ea = m.eulerAngles(2, 1, 0); 

        Eigen::Matrix<double, 6, 1> fk_current;
        fk_current << poseP(0), poseP(1), poseP(2), ea(0), ea(1), ea(2);

        // cout << "fk_cur " << fk_cur.transpose() << " fkgoal " << fkgoal.transpose() << endl;
        // if last element, only add state cost

        scalar_t c_mat_to_scalar;

        if (last == 1)
        {
            c_mat_to_scalar = 0.5 * (fk_current.transpose() - fkgoal.transpose()) * costFunction->getT() * (fk_current - fkgoal);
        }
        else 
        {
            c_mat_to_scalar = 0.5 * u.transpose() * costFunction->getR() * u;        
        }

        std::cout << "in this function" << std::endl;

        // c_mat_to_scalar.setZero();

        return c_mat_to_scalar;
    }

    
}

scalar_t KukaArm_TRK::cost_func_expre(const unsigned int& index_k, const stateVec_t& xList_k, const commandVec_t& uList_k, const stateVec_t& xList_bar_k, const commandVec_t& uList_bar_k, CostFunctionKukaArm_TRK*& costFunction){
    scalar_t c_mat_to_scalar;
    unsigned int Nl = NumberofKnotPt;

    if (SOFT_CONTACT)
    {
        if(index_k == Nl){
            c_mat_to_scalar = 0.5*(xList_k.transpose() - xtrack[index_k].transpose()) * costFunction->getQf() * (xList_k - xtrack[index_k]) + 
                    0.5*(xList_k.transpose() - xList_bar_k.transpose()) * costFunction->getRho_state() * (xList_k - xList_bar_k);
            // c_mat_to_scalar = 0.5*(xList_k.transpose() - xgoal.transpose()) * costFunction->getQf() * (xList_k - xgoal);
            // c_mat_to_scalar = forwardkin_cost(xList_k, uList_k, fk_ref[index_k], costFunction, 1);
        }
        else
        {
            c_mat_to_scalar = 0.5*(xList_k.transpose() - xtrack[index_k].transpose()) * costFunction->getQ() * (xList_k - xtrack[index_k]) + 
            0.5*(xList_k.transpose() - xList_bar_k.transpose()) * costFunction->getRho_state() * (xList_k - xList_bar_k);
            // c_mat_to_scalar = 0.5*(xList_k.transpose() - xgoal.transpose())*costFunction->getQ()*(xList_k - xgoal);
            c_mat_to_scalar += 0.5*uList_k.transpose()*costFunction->getR()*uList_k + 
            0.5*(uList_k.transpose() - uList_bar_k.transpose()) * costFunction->getRho_torque() * (uList_k - uList_bar_k);
            // c_mat_to_scalar = forwardkin_cost(xList_k, uList_k, fk_ref[index_k], costFunction, 0);
        }
    }
    else
    {
        if (index_k == Nl)
        {
            c_mat_to_scalar = 0.5*(xList_k.transpose() - xgoal.transpose())*costFunction->getQf()*(xList_k - xgoal) + 
                0.5*(xList_k.transpose() - xList_bar_k.transpose()) * costFunction->getRho_state() * (xList_k - xList_bar_k);
        }
        else
        {
            c_mat_to_scalar = 0.5*(xList_k.transpose() - xgoal.transpose())*costFunction->getQ()*(xList_k - xgoal) + 
                0.5*(xList_k.transpose() - xList_bar_k.transpose()) * costFunction->getRho_state() * (xList_k - xList_bar_k);
            c_mat_to_scalar += 0.5*uList_k.transpose()*costFunction->getR()*uList_k + 
                0.5*(uList_k.transpose() - uList_bar_k.transpose()) * costFunction->getRho_torque()* (uList_k - uList_bar_k);
        }
    } 
    // c_mat_to_scalar.setZero();
    return c_mat_to_scalar;
}

stateVec_t KukaArm_TRK::finite_diff_cx(const unsigned int& index_k, const stateVec_t& xList_k, const commandVec_t& uList_k, const stateVec_t& xList_bar_k, const commandVec_t& uList_bar_k, CostFunctionKukaArm_TRK*& costFunction){
    stateVec_t cx_fd_k;
    unsigned int n = xList_k.size();
    // unsigned int m = uList_k.size();
    scalar_t cp1;
    scalar_t cm1;
    double delta = 1e-3;
    stateMat_t Dx;
    Dx.setIdentity();
    Dx = delta*Dx;
    // State perturbation for cost
    for(unsigned int i=0;i<n;i++){
        cp1 = cost_func_expre(index_k, xList_k+Dx.col(i), uList_k, xList_bar_k, uList_bar_k, costFunction);
        cm1 = cost_func_expre(index_k, xList_k-Dx.col(i), uList_k, xList_bar_k, uList_bar_k, costFunction);
        cx_fd_k.row(i) = (cp1 - cm1)/(2*delta);
    }
    return cx_fd_k;
}

commandVec_t KukaArm_TRK::finite_diff_cu(const unsigned int& index_k, const stateVec_t& xList_k, const commandVec_t& uList_k, const stateVec_t& xList_bar_k, const commandVec_t& uList_bar_k, CostFunctionKukaArm_TRK*& costFunction){
    commandVec_t cu_fd_k;
    // unsigned int n = xList_k.size();
    unsigned int m = uList_k.size();
    scalar_t cp1;
    scalar_t cm1;
    double delta = 1e-3;
    commandMat_t Du;
    Du.setIdentity();
    Du = delta*Du;
    // State perturbation for cost
    for(unsigned int i=0;i<m;i++){
        cp1 = cost_func_expre(index_k, xList_k, uList_k+Du.col(i), xList_bar_k, uList_bar_k, costFunction);
        cm1 = cost_func_expre(index_k, xList_k, uList_k-Du.col(i), xList_bar_k, uList_bar_k, costFunction);
        cu_fd_k.row(i) = (cp1 - cm1)/(2*delta);
    }
    return cu_fd_k;
}

void KukaArm_TRK::kuka_arm_dyn_cst_ilqr(const int& nargout, const stateVecTab_t& xList, const commandVecTab_t& uList, stateVecTab_t& FList, 
                                const stateVecTab_t& xList_bar, const commandVecTab_t& uList_bar, CostFunctionKukaArm_TRK*& costFunction)
{
    // // for a positive-definite quadratic, no control cost (indicated by the iLQG function using nans), is equivalent to u=0
    // for ADMM, add new penalties for augmented Lagrangian terms
    if(debugging_print) TRACE_KUKA_ARM("initialize dimensions\n");
    unsigned int Nl = xList.size();
    
    costFunction->getc() = 0;
    AA.setZero();
    BB.setZero();

    if(debugging_print) TRACE_KUKA_ARM("compute cost function\n");

    scalar_t c_mat_to_scalar;

    if(nargout == 2){
        // Just for debug, never go to here; Originally, this was to imitate the MATLAB function?
        const int nargout_update1 = 3;        
        for (unsigned int k = 0;k < Nl; k++)
        {
            if (k == Nl-1)
            {
                if(debugging_print) TRACE_KUKA_ARM("before the update1\n");
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose()) * costFunction->getQf() * (xList[k] - xgoal) + 
                0.5*(xList[k].transpose() - xList_bar[k].transpose()) * costFunction->getRho_state() * (xList[k] - xList_bar[k]);
                // cout << "CMAT SCALAR 1: " << c_mat_to_scalar << endl;
                costFunction->getc() += c_mat_to_scalar(0,0);
                if(debugging_print) TRACE_KUKA_ARM("after the update1\n");
            } else
            {
                // if u is not NaN (not final), add state and control cost
                if(debugging_print) TRACE_KUKA_ARM("before the update2\n");
                FList[k] = update(nargout_update1, xList[k], uList[k], AA, BB);
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQ()*(xList[k] - xgoal) + 
                0.5*(xList[k].transpose() - xList_bar[k].transpose()) * costFunction->getRho_state() * (xList[k] - xList_bar[k]);
                
                // cout << "CMAT SCALAR 2: " << c_mat_to_scalar << endl;
                
                if(debugging_print) TRACE_KUKA_ARM("after the update2\n");
                c_mat_to_scalar += 0.5*uList[k].transpose()*costFunction->getR()*uList[k] + 
                0.5*(uList[k].transpose() - uList_bar[k].transpose()) * costFunction->getRho_torque()* (uList[k] - uList_bar[k]);
                costFunction->getc() += c_mat_to_scalar(0,0);
            }
        }

    }
    else 
    {
        const int nargout_update2 = 3;
        for (unsigned int k = 0;k < Nl; k++) 
        {
            if (k == Nl-1) 
            {//isNanVec(uList[k])
                // if(debugging_print) TRACE_KUKA_ARM("before the update3\n");
                c_mat_to_scalar = cost_func_expre(k, xList[k], uList[k], xList_bar[k], uList_bar[k], costFunction);
                // c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQf()*(xList[k] - xgoal) + 
                // 0.5*(xList[k].transpose() - xList_bar[k].transpose()) * costFunction->getRho_state() * (xList[k] - xList_bar[k]);
                costFunction->getc() += c_mat_to_scalar(0,0);
                // cout << "CMAT SCALAR 1: " << c_mat_to_scalar << endl;
                // if(debugging_print) TRACE_KUKA_ARM("after the update3\n");
            }
            else {
                // if(debugging_print) TRACE_KUKA_ARM("before the update4\n");
                FList[k] = update(nargout_update2, xList[k], uList[k], AA, BB);//assume three outputs, code needs to be optimized
                // if(debugging_print) TRACE_KUKA_ARM("before the update4-1\n");
                c_mat_to_scalar = cost_func_expre(k, xList[k], uList[k], xList_bar[k], uList_bar[k], costFunction);
                // c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQ()*(xList[k] - xgoal) + 
                // 0.5*(xList[k].transpose() - xList_bar[k].transpose()) * costFunction->getRho_state() * (xList[k] - xList_bar[k]);;
                // if(debugging_print) TRACE_KUKA_ARM("after the update4\n");

                // c_mat_to_scalar += 0.5*uList[k].transpose()*costFunction->getR()*uList[k] + 
                // 0.5*(uList[k].transpose() - uList_bar[k].transpose()) * costFunction->getRho_torque()* (uList[k] - uList_bar[k]);
                costFunction->getc() += c_mat_to_scalar(0,0); // TODO: to be checked
                // if(debugging_print) TRACE_KUKA_ARM("after the update5\n"); 
                // cout << "CMAT SCALAR 2: " << c_mat_to_scalar << endl;
                A_temp[k] = AA;
                B_temp[k] = BB;
            }
        }

        // std::cout << c_mat_to_scalar << std::endl;
        stateVec_t cx_temp;
        // cout << cx_temp << endl;

        if(debugging_print) TRACE_KUKA_ARM("compute dynamics and cost derivative\n");

         //Manually coded finite diff
        unsigned int n = xList[0].size();
        unsigned int m = uList[0].size();

        double delta = 1e-3;
        stateMat_t Dx;
        commandMat_t Du;
        Dx.setIdentity();
        Dx = delta*Dx;
        Du.setIdentity();
        Du = delta*Du;

        stateVec_t cx1;
        stateVec_t cxm1;
        commandVec_t cu1;
        commandVec_t cum1;
        stateVecTab_t cx_fd;
	    commandVecTab_t cu_fd; 
	    stateMatTab_t cxx_fd; 
	    commandR_stateC_tab_t cux_fd; 
	    commandMatTab_t cuu_fd;
        cx_fd.resize(Nl+1);
        cu_fd.resize(Nl+1);
        cxx_fd.resize(Nl+1);
        cux_fd.resize(Nl+1);
        cuu_fd.resize(Nl+1);

        for (unsigned int k = 0;k < Nl; k++)
        {
            if (k<Nl-1)
            {
                fxList[k] = A_temp[k];
                fuList[k] = B_temp[k];
            }
            cx_temp << xList[k] - xgoal;

            cx_fd[k] = finite_diff_cx(k, xList[k], uList[k], xList_bar[k], uList_bar[k], costFunction);
            cu_fd[k] = finite_diff_cu(k, xList[k], uList[k], xList_bar[k], uList_bar[k], costFunction);

            // State perturbation for cxx
            for(unsigned int i=0;i<n;i++){
                cx1 = finite_diff_cx(k, xList[k]+Dx.col(i), uList[k], xList_bar[k], uList_bar[k], costFunction);
                cxm1 = finite_diff_cx(k, xList[k]-Dx.col(i), uList[k], xList_bar[k], uList_bar[k], costFunction);
                cxx_fd[k].col(i) = (cx1 - cxm1)/(2*delta);
            }

            // Control perturbation for cuu
            for(unsigned int i=0;i<m;i++){
                cu1 = finite_diff_cu(k, xList[k], uList[k]+Du.col(i), xList_bar[k], uList_bar[k], costFunction);
                cum1 = finite_diff_cu(k, xList[k], uList[k]-Du.col(i), xList_bar[k], uList_bar[k], costFunction);
                cuu_fd[k].col(i) = (cu1 - cum1)/(2*delta);
            }

            // Analytical derivatives given quadratic cost
            // costFunction->getcx()[k] = costFunction->getQ()*cx_temp;
            // costFunction->getcu()[k] = costFunction->getR()*uList[k];
            // costFunction->getcxx()[k] = costFunction->getQ();
            // costFunction->getcux()[k].setZero();
            // costFunction->getcuu()[k] = costFunction->getR(); 

            //Note that cu , cux and cuu at the final time step will never be used (see ilqrsolver::doBackwardPass)
            costFunction->getcx()[k] = cx_fd[k];
            costFunction->getcu()[k] = cu_fd[k];
            costFunction->getcxx()[k] = cxx_fd[k];
            costFunction->getcux()[k].setZero();
            costFunction->getcuu()[k] = cuu_fd[k];      

            // cout << "debug for derivatives cx" << costFunction->getcx()[k] << endl;
            // cout << "debug for derivatives cu" << costFunction->getcu()[k] << endl;
            // cout << "debug for derivatives cxx" << costFunction->getcxx()[k] << endl;
            // cout << "debug for derivatives cuu" << costFunction->getcuu()[k] << endl;
            // cout << "debug for derivatives cxx" << costFunction->getcxx()[k] << endl;  
            // cout << "Q = " << costFunction->getQ() << endl;     
        }
        if(debugging_print) TRACE_KUKA_ARM("update the final value of cost derivative \n");

        // Analytical derivatives given quadratic cost
        // costFunction->getcx()[Nl-1] = costFunction->getQf()*(xList[Nl-1]-xgoal);
        // costFunction->getcu()[Nl-1] = costFunction->getR()*uList[Nl-1];
        // costFunction->getcxx()[Nl-1] = costFunction->getQf();
        // costFunction->getcux()[Nl-1].setZero();
        // costFunction->getcuu()[Nl-1] = costFunction->getR();

        // cout << "debug for derivatives cx" << costFunction->getcx()[Nl-1] - costFunction->getQf()*(xList[Nl-1]-xgoal) << endl;
        // cout << "debug for derivatives cu" << costFunction->getcu()[Nl-2] << endl;//- costFunction->getR()*uList[Nl-2] << endl;
        // cout << "debug for derivatives cxx" << costFunction->getcxx()[Nl-1] - costFunction->getQf() << endl;
        // cout << "debug for derivatives cuu" << costFunction->getcuu()[Nl-1] - costFunction->getR() << endl;
        // cout << "debug for derivatives cxx" << costFunction->getcxx()[Nl-1] << endl;  
        // cout << "Q = " << costFunction->getQ() << endl;
        // for (unsigned int a=0; a<Nl; a++) {
        //     cout << "cost derivative: " << a << endl;
        //     cout << costFunction->getcx()[a] << endl;
        //     cout << "xList: " << a << endl;
        //     cout << xList[a] << endl;
        // }

        if(debugging_print) TRACE_KUKA_ARM("set unused matrices to zero \n");

        // the following useless matrices are set to Zero.
        //fxx, fxu, fuu are not defined since never used
        for(unsigned int k=0;k<Nl;k++){
            FList[k].setZero();
        }
        costFunction->getc() = 0;
    }
    if(debugging_print) TRACE_KUKA_ARM("finish kuka_arm_dyn_cst\n");

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    //     for(unsigned int k=0;k<Nl-1;k++){
    //         fxList[k] = A_temp[k];
    //         fuList[k] = B_temp[k];
            
    //         // cx_temp << xList[k](0,0)-xgoal(0), xList[k](1,0)-xgoal(1), xList[k](2,0)-xgoal(2), xList[k](3,0)-xgoal(3);
            
    //         cx_temp << xList[k] - xgoal;
            
    //         // if (k==0)
    //         //     cout << cx_temp << endl;

    //         costFunction->getcx()[k] = costFunction->getQ()*cx_temp + costFunction->getRho_state()*(xList[k]-xList_bar[k]);
            
    //         costFunction->getcu()[k] = costFunction->getR()*uList[k] + costFunction->getRho_torque()*(uList[k]-uList_bar[k]);
    //         costFunction->getcxx()[k] = costFunction->getQ() + costFunction->getRho_state();
    //         costFunction->getcux()[k].setZero();
    //         costFunction->getcuu()[k] = costFunction->getR() + costFunction->getRho_torque();            
    //     }
    //     if(debugging_print) TRACE_KUKA_ARM("update the final value of cost derivative \n");

    //     costFunction->getcx()[Nl-1] = costFunction->getQf()*(xList[Nl-1]-xgoal) + costFunction->getRho_state()*(xList[Nl-1]-xList_bar[Nl-1]);
    //     costFunction->getcu()[Nl-1] = costFunction->getR()*uList[Nl-1] + costFunction->getRho_torque()*(uList[Nl-1]-uList_bar[Nl-1]);
    //     costFunction->getcxx()[Nl-1] = costFunction->getQf() + costFunction->getRho_state();
    //     costFunction->getcux()[Nl-1].setZero();
    //     costFunction->getcuu()[Nl-1] = costFunction->getR() + costFunction->getRho_torque();

    //     // for (unsigned int a=0; a<Nl; a++) {
    //     //     cout << "cost derivative: " << a << endl;
    //     //     cout << costFunction->getcx()[a] << endl;
    //     //     cout << "xList: " << a << endl;
    //     //     cout << xList[a] << endl;
    //     // }

    //     if(debugging_print) TRACE_KUKA_ARM("set unused matrices to zero \n");

    //     // the following useless matrices are set to Zero.
    //     //fxx, fxu, fuu are not defined since never used
    //     for(unsigned int k=0;k<Nl;k++){
    //         FList[k].setZero();
    //     }
    //     costFunction->getc() = 0;
    // }
    // if(debugging_print) TRACE_KUKA_ARM("finish kuka_arm_dyn_cst\n");
}

void KukaArm_TRK::kuka_arm_dyn_cst_min_output(const unsigned int& index_k, const double& dt_p, const stateVec_t& xList_curr, const commandVec_t& uList_curr, 
        const stateVec_t& xList_cur_bar, const commandVec_t& uList_cur_bar, const bool& isUNan, stateVec_t& xList_next, CostFunctionKukaArm_TRK*& costFunction){
    if(debugging_print) TRACE_KUKA_ARM("initialize dimensions\n");
    // unsigned int Nc = xList_curr.cols(); //xList_curr is 14x1 vector -> col=1

    costFunction->getc() = 0; // temporary cost container? initializes every timestep
    AA.setZero(); 
    BB.setZero();

    if(debugging_print) TRACE_KUKA_ARM("compute cost function\n");

    scalar_t c_mat_to_scalar;
    xList_next.setZero(); // zeroing previous trajectory timestep by timestep

    const int nargout_update1 = 1;
    if (isUNan) { 
        // cout << "R: " <<  costFunction->getR() << endl;
        // cout << "Q: " <<  costFunction->getQ() << endl;
        // cout << "QF: " <<  costFunction->getQf() << endl;
        // if(debugging_print) TRACE_KUKA_ARM("before the update1\n");
        c_mat_to_scalar = cost_func_expre(index_k, xList_curr, uList_curr, xList_cur_bar, uList_cur_bar, costFunction);
        // c_mat_to_scalar = 0.5*(xList_curr.transpose() - xgoal.transpose()) * costFunction->getQf() * (xList_curr - xgoal) + 
        // 0.5*(xList_curr.transpose() - xList_cur_bar.transpose())*costFunction->getRho_state()*(xList_curr - xList_cur_bar);
        costFunction->getc() += c_mat_to_scalar(0,0);
        // if(debugging_print) TRACE_KUKA_ARM("after the update1\n");
    }
    else {
        // if(debugging_print) TRACE_KUKA_ARM("before the update2\n");
        xList_next = update(nargout_update1, xList_curr, uList_curr, AA, BB);
        c_mat_to_scalar = cost_func_expre(index_k, xList_curr, uList_curr, xList_cur_bar, uList_cur_bar, costFunction);
        // c_mat_to_scalar = 0.5*(xList_curr.transpose() - xgoal.transpose())*costFunction->getQ()*(xList_curr - xgoal) + 
        // 0.5*(xList_curr.transpose() - xList_cur_bar.transpose())*costFunction->getRho_state()*(xList_curr - xList_cur_bar);
        // if(debugging_print) TRACE_KUKA_ARM("after the update2\n");
        // c_mat_to_scalar += 0.5*uList_curr.transpose()*costFunction->getR()*uList_curr + 
        // 0.5*(uList_curr.transpose() - uList_cur_bar.transpose()) * costFunction->getRho_torque()* (uList_curr - uList_cur_bar);
        costFunction->getc() += c_mat_to_scalar(0,0);
    }
    if(debugging_print) TRACE_KUKA_ARM("finish kuka_arm_dyn_cst\n");
}

stateVec_t KukaArm_TRK::update(const int& nargout, const stateVec_t& X, const commandVec_t& U, stateMat_t& A, stateR_commandC_t& B){
    // 4th-order Runge-Kutta step
    if(debugging_print) TRACE_KUKA_ARM("update: 4th-order Runge-Kutta step\n");

    gettimeofday(&tbegin_period4,NULL);

    // output of kuka arm dynamics is xdot = f(x,u)
    Xdot1 = kuka_arm_dynamics(X, U);
    Xdot2 = kuka_arm_dynamics(X + 0.5*dt*Xdot1, U);
    Xdot3 = kuka_arm_dynamics(X + 0.5*dt*Xdot2, U);
    Xdot4 = kuka_arm_dynamics(X + dt*Xdot3, U);
    stateVec_t X_new;
    X_new = X + (dt/6)*(Xdot1 + 2*Xdot2 + 2*Xdot3 + Xdot4);
    // Simple Euler Integration (for debug)
//    X_new = X + (dt)*Xdot1;
    
    if ((globalcnt%4 == 0) && (globalcnt<40)) {
        // cout << "X " << endl << X << endl;
        // cout << "Xdot1 " << endl << Xdot1 << endl;
        // cout << "Xdot2 " << endl << Xdot2 << endl;
        // cout << "Xdot3 " << endl << Xdot3 << endl;
        // cout << "Xdot4 " << endl << Xdot4 << endl;
        // cout << "X_NEW: " << endl << X_new << endl;
    }

    if(debugging_print) TRACE_KUKA_ARM("update: X_new\n");

    //######3
    // int as = 0;
    //#########3

    if(nargout > 1){// && (as!=0)){
        //cout << "NEVER HERE" << endl;
        unsigned int n = X.size();
        unsigned int m = U.size();

        double delta = 1e-7;
        stateMat_t Dx;
        commandMat_t Du;
        Dx.setIdentity();
        Dx = delta*Dx;
        Du.setIdentity();
        Du = delta*Du;

        // State perturbation?
        for(unsigned int i=0;i<n;i++){
            Xp1 = kuka_arm_dynamics(X+Dx.col(i),U);
            Xm1 = kuka_arm_dynamics(X-Dx.col(i),U);
            A1.col(i) = (Xp1 - Xm1)/(2*delta);

            Xp2 = kuka_arm_dynamics(X+0.5*dt*Xdot1+Dx.col(i),U);
            Xm2 = kuka_arm_dynamics(X+0.5*dt*Xdot1-Dx.col(i),U);
            A2.col(i) = (Xp2 - Xm2)/(2*delta);

            Xp3 = kuka_arm_dynamics(X+0.5*dt*Xdot2+Dx.col(i),U);
            Xm3 = kuka_arm_dynamics(X+0.5*dt*Xdot2-Dx.col(i),U);
            A3.col(i) = (Xp3 - Xm3)/(2*delta);

            Xp4 = kuka_arm_dynamics(X+dt*Xdot3+Dx.col(i),U);
            Xm4 = kuka_arm_dynamics(X+dt*Xdot3-Dx.col(i),U);
            A4.col(i) = (Xp4 - Xm4)/(2*delta);
        }

        // Control perturbation?
        for(unsigned int i=0;i<m;i++){
            Xp1 = kuka_arm_dynamics(X,U+Du.col(i));
            Xm1 = kuka_arm_dynamics(X,U-Du.col(i));
            B1.col(i) = (Xp1 - Xm1)/(2*delta);

            Xp2 = kuka_arm_dynamics(X+0.5*dt*Xdot1,U+Du.col(i));
            Xm2 = kuka_arm_dynamics(X+0.5*dt*Xdot1,U-Du.col(i));
            B2.col(i) = (Xp2 - Xm2)/(2*delta);

            Xp3 = kuka_arm_dynamics(X+0.5*dt*Xdot2,U+Du.col(i));
            Xm3 = kuka_arm_dynamics(X+0.5*dt*Xdot2,U-Du.col(i));
            B3.col(i) = (Xp3 - Xm3)/(2*delta);

            Xp4 = kuka_arm_dynamics(X+dt*Xdot3,U+Du.col(i));
            Xm4 = kuka_arm_dynamics(X+dt*Xdot3,U-Du.col(i));
            B4.col(i) = (Xp4 - Xm4)/(2*delta);
        }

        A = (IdentityMat + A4 * dt/6)*(IdentityMat + A3 * dt/3)*(IdentityMat + A2 * dt/3)*(IdentityMat + A1 * dt/6);
        B = B4 * dt/6 + (IdentityMat + A4 * dt/6) * B3 * dt/3 + (IdentityMat + A4 * dt/6)*(IdentityMat + A3 * dt/3)* B2 * dt/3 + (IdentityMat + (dt/6)*A4)*(IdentityMat + (dt/3)*A3)*(IdentityMat + (dt/3)*A2)*(dt/6)*B1;
    }
    if(debugging_print) TRACE_KUKA_ARM("update: X_new\n");

    gettimeofday(&tend_period4,NULL);
    finalTimeProfile.time_period4 += (static_cast<double>(1000.0*(tend_period4.tv_sec-tbegin_period4.tv_sec)+((tend_period4.tv_usec-tbegin_period4.tv_usec)/1000.0)))/1000.0;

    return X_new;
}

void KukaArm_TRK::grad(const stateVec_t& X, const commandVec_t& U, stateMat_t& A, stateR_commandC_t& B){
    unsigned int n = X.size();
    unsigned int m = U.size();

    double delta = 1e-7;
    stateMat_t Dx;
    commandMat_t Du;
    Dx.setIdentity();
    Dx = delta*Dx;
    Du.setIdentity();
    Du = delta*Du;

    AA.setZero();
    BB.setZero();

    int nargout = 1;
    for(unsigned int i=0;i<n;i++){
        Xp = update(nargout, X+Dx.col(i), U, AA, BB);
        Xm = update(nargout, X-Dx.col(i), U, AA, BB);
        A.col(i) = (Xp - Xm)/(2*delta);
    }

    for(unsigned int i=0;i<m;i++){
        Xp = update(nargout, X, U+Du.col(i), AA, BB);
        Xm = update(nargout, X, U-Du.col(i), AA, BB);
        B.col(i) = (Xp - Xm)/(2*delta);
    }
}

// parameters are called by reference. Name doesn't matter
void KukaArm_TRK::hessian(const stateVec_t& X, const commandVec_t& U, stateTens_t& fxx_p, stateR_stateC_commandD_t& fxu_p, stateR_commandC_commandD_t& fuu_p){
    unsigned int n = X.size();
    unsigned int m = U.size();

    double delta = 1e-5;
    stateMat_t Dx;
    commandMat_t Du;
    Dx.setIdentity();
    Dx = delta*Dx;
    Du.setIdentity();
    Du = delta*Du;

    stateMat_t Ap;
    Ap.setZero();
    stateMat_t Am;
    Am.setZero();
    stateR_commandC_t B;
    B.setZero();

    for(unsigned int i=0;i<n;i++){
        fxx_p[i].setZero();
        fxu_p[i].setZero();
        fuu_p[i].setZero();
    }

    for(unsigned int i=0;i<n;i++){
        grad(X+Dx.col(i), U, Ap, B);
        grad(X-Dx.col(i), U, Am, B);
        fxx_p[i] = (Ap - Am)/(2*delta);
    }

    stateR_commandC_t Bp;
    Bp.setZero();
    stateR_commandC_t Bm;
    Bm.setZero();

    for(unsigned int j=0;j<m;j++){
        grad(X, U+Du.col(j), Ap, Bp);
        grad(X, U-Du.col(j), Am, Bm);
        fxu_p[j] = (Ap - Am)/(2*delta);
        fuu_p[j] = (Bp - Bm)/(2*delta);
    }
}

unsigned int KukaArm_TRK::getStateNb()
{
    return stateNb;
}

unsigned int KukaArm_TRK::getCommandNb()
{
    return commandNb;
}

commandVec_t& KukaArm_TRK::getLowerCommandBounds()
{
    return lowerCommandBounds;
}

commandVec_t& KukaArm_TRK::getUpperCommandBounds()
{
    return upperCommandBounds;
}

stateMatTab_t& KukaArm_TRK::getfxList()
{
    return fxList;
}

stateR_commandC_tab_t& KukaArm_TRK::getfuList()
{
    return fuList;
}


void KukaArm_TRK::kuka_arm_dyn_cst_udp(const int& nargout, const stateVecTab_t& xList, const commandVecTab_t& uList, stateVecTab_t& FList,
                                CostFunctionKukaArm_TRK*& costFunction){
    if(debugging_print) TRACE_KUKA_ARM("initialize dimensions\n");
    unsigned int Nl = xList.size();
    
    costFunction->getc() = 0;
    AA.setZero();
    BB.setZero();
    if(debugging_print) TRACE_KUKA_ARM("compute cost function\n");

    scalar_t c_mat_to_scalar;
    c_mat_to_scalar.setZero();

    if(nargout == 2){
        const int nargout_update1 = 3;        
        for(unsigned int k=0;k<Nl;k++){
            if (k == Nl-1){
                if(debugging_print) TRACE_KUKA_ARM("before the update1\n");
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose()) * costFunction->getQf() * (xList[k] - xgoal);
                costFunction->getc() += c_mat_to_scalar(0,0);
                if(debugging_print) TRACE_KUKA_ARM("after the update1\n");
            }else{
                if(debugging_print) TRACE_KUKA_ARM("before the update2\n");
                FList[k] = update(nargout_update1, xList[k], uList[k], AA, BB);
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQ()*(xList[k] - xgoal);
                if(debugging_print) TRACE_KUKA_ARM("after the update2\n");
                c_mat_to_scalar += 0.5*uList[k].transpose()*costFunction->getR()*uList[k];
                costFunction->getc() += c_mat_to_scalar(0,0);
            }
        }
    }else{
        stateVec_t cx_temp;
        if(debugging_print) TRACE_KUKA_ARM("compute cost derivative\n");
        for(unsigned int k=0;k<Nl-1;k++){
            cx_temp << xList[k](0,0)-xgoal(0), xList[k](1,0)-xgoal(1), xList[k](2,0)-xgoal(2), xList[k](3,0)-xgoal(3);
            costFunction->getcx()[k] = costFunction->getQ()*cx_temp;
            costFunction->getcu()[k] = costFunction->getR()*uList[k];
            costFunction->getcxx()[k] = costFunction->getQ();
            costFunction->getcux()[k].setZero();
            costFunction->getcuu()[k] = costFunction->getR();            
        }
        if(debugging_print) TRACE_KUKA_ARM("update the final value of cost derivative \n");
        costFunction->getcx()[Nl-1] = costFunction->getQf()*(xList[Nl-1]-xgoal);
        costFunction->getcu()[Nl-1] = costFunction->getR()*uList[Nl-1];
        costFunction->getcxx()[Nl-1] = costFunction->getQf();
        costFunction->getcux()[Nl-1].setZero();
        costFunction->getcuu()[Nl-1] = costFunction->getR();
        if(debugging_print) TRACE_KUKA_ARM("set unused matrices to zero \n");

        // the following useless matrices and scalars are set to Zero.
        for(unsigned int k=0;k<Nl;k++){
            FList[k].setZero();
        }
        costFunction->getc() = 0;
    }
    if(debugging_print) TRACE_KUKA_ARM("finish kuka_arm_dyn_cst\n");
}
