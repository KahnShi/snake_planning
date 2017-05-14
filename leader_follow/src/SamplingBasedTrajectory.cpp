#include <leader_follow/SamplingBasedTrajectory.h>
USING_NAMESPACE_QPOASES

namespace sampling_based_trajectory
{
  SamplingBasedTrajectory::SamplingBasedTrajectory(ros::NodeHandle nh, ros::NodeHandle nhp, int dimensions, std::string traj_path_pub_topic_name, double debug, double visualize_unit_time):
    m_nh(nh),
    m_nhp(nhp),
    m_dim(dimensions),
    m_debug(debug),
    m_visualize_unit_time(visualize_unit_time),
    m_traj_path_pub_topic_name(traj_path_pub_topic_name)
  {
    m_pub_traj_path = m_nh.advertise<nav_msgs::Path>(m_traj_path_pub_topic_name, 1);
  }

  bool SamplingBasedTrajectory::generateTrajectory()
  {
    // debug
    if (m_debug){
      std::cout << m_dim << ", order: " << m_traj_order << ", dev: " << m_traj_dev_order << ", samples num: " << m_n_samples << "\n";
      for (int i = 0; i < m_sample_x_pos_ptr->rows(); ++i){
        std::cout << "[" << (*m_sample_x_pos_ptr)[i] << ", "  << (*m_sample_y_pos_ptr)[i] << ", " << (*m_sample_z_pos_ptr)[i] << "]\n";
        std::cout << "[" << (*m_sample_x_vel_ptr)[i] << ", "  << (*m_sample_y_vel_ptr)[i] << ", " << (*m_sample_z_vel_ptr)[i] << "]\n";
      }
      std::cout << "\ntime:\n";
      for (int i = 0; i < m_sample_x_vel_ptr->rows(); ++i)
        std::cout << (*m_sample_time_ptr)[i] << ", ";
      std::cout << "\n\n";
    }

    bool do_param_x = generateTrajectory(m_sample_x_pos_ptr, m_sample_x_vel_ptr, m_sample_time_ptr, m_traj_param_x_ptr);
    bool do_param_y = generateTrajectory(m_sample_y_pos_ptr, m_sample_y_vel_ptr, m_sample_time_ptr, m_traj_param_y_ptr);
    bool do_param_z = true;
    if (m_dim == 2){
      for (int i = 0; i < m_traj_order * (m_n_samples - 1); ++i)
        (*m_traj_param_z_ptr)[i] = 0.0;
    }
    else
      do_param_z = generateTrajectory(m_sample_z_pos_ptr, m_sample_z_vel_ptr, m_sample_time_ptr, m_traj_param_z_ptr);

    if (do_param_x && do_param_y && do_param_z)
      return true;
    else{
      ROS_ERROR("Could not calculate the paramaters of trajectory.");
      return false;
    }
  }

  bool SamplingBasedTrajectory::generateTrajectory(VectorXd *sample_pos_ptr, VectorXd *sample_vel_ptr, VectorXd *sample_time_ptr, VectorXd *traj_param_ptr)
  {
    int n_samples = m_n_samples;
    int n_polynomial = n_samples - 1;
    int n_constraints_vel = 0, n_constraints;
    for (int i = 0; i < sample_vel_ptr->rows(); ++i){
      /* We make the set that less than -100.0 means no constraint */
      if ((*sample_vel_ptr)[i] > -100.0 + 1.0){
        n_constraints_vel += 1;
      }
    }
    /* Constraints components:
       1. some sample points with velocity constraints
       2. start and end points' position constrints
       3. middle points have equal constraints in position, velocity, ...., (m_traj_dev_order-1) order
    */
    n_constraints = n_constraints_vel +
      (n_samples - 2) + n_samples
      + (n_samples - 2) * (m_traj_dev_order - 1);
    MatrixXd H = MatrixXd::Zero(m_traj_order * n_polynomial, m_traj_order * n_polynomial);
    MatrixXd A = MatrixXd::Zero(n_constraints, m_traj_order * n_polynomial);
    VectorXd lb_A = VectorXd::Zero(n_constraints);
    VectorXd ub_A = VectorXd::Zero(n_constraints);
    MatrixXd T = MatrixXd::Zero(n_samples, m_traj_order);
    for (int i = 1; i < n_samples; ++i){
      double cal = 1.0, factor = (*sample_time_ptr)[i] - (*sample_time_ptr)[i-1];
      for (int j = 0; j < m_traj_order; ++j){
        T(i, j) = cal;
        cal *= factor;
      }
    }
    /* Except last sample point, every point's position constraint is achieved by checking the related polynomial whose head point is that point.
     For last sample point, we use last polynomial whose end point is that point.*/

    int row_id = 0;
    /* Velocity constriants */
    for (int i = 0; i < n_samples; ++i){
      if ((*sample_vel_ptr)[i] > -100.0 + 1.0){
        int col_id = i * m_traj_order;
        if (i == n_samples - 1){
          for (int j = 1; j < m_traj_order; ++j)
            A(row_id, col_id-m_traj_order+j) = T(n_samples - 1, j - 1) * permutation(j, 1);
        }
        else{
          A(row_id, col_id+1) = 1.0;
        }
        lb_A(row_id) = (*sample_vel_ptr)[i];
        ub_A(row_id) = (*sample_vel_ptr)[i];
        row_id += 1;
      }
    }

    for (int i = 0; i < n_samples; ++i){
      int col_id = i * m_traj_order;
      /* Position constriants */
      if (i == n_samples - 1){
        /* Check the end point of polynomial */
        for (int j = 0; j < m_traj_order; ++j)
          A(row_id, col_id-m_traj_order+j) = T(n_samples - 1, j);
      }
      else{
        A(row_id, col_id) = 1.0;
      }
      lb_A(row_id) = (*sample_pos_ptr)[i];
      ub_A(row_id) = (*sample_pos_ptr)[i];
      row_id += 1;

      /* Position constrints (end point equal to sample point) [merged below] */
      // if (i >= 1 && i <= n_samples - 2){
      //   for (int j = 0; j < m_traj_order; ++j)
      //     A(row_id, col_id-m_traj_order +j) = T(i, j);
      //   lb_A(row_id) = (*sample_pos_ptr)[i];
      //   ub_A(row_id) = (*sample_pos_ptr)[i];
      //   row_id += 1;
      // }

      /* Equal constraints in position, velocity (1 order), ..., (m_traj_dev_order-1) order */
      if (i >= 1 && i <= n_samples - 2){
        for (int order = 0; order < m_traj_dev_order; ++order){
          A(row_id, col_id+order) = double(permutation(order, order));
          for (int j = order; j < m_traj_order; ++j){
            A(row_id, col_id-m_traj_order+j) = -T(i, j-order) * permutation(j, order);
          }
          lb_A(row_id) = 0.0;
          ub_A(row_id) = 0.0;
          row_id += 1;
        }
      }
    }

    /* Get H matrix, minimum objective function */
    for (int i = 0; i < n_polynomial; ++i){
      int id = i * m_traj_order;
      for (int j = m_traj_dev_order; j < m_traj_order; ++j){
        for (int k = m_traj_dev_order; k < m_traj_order; ++k){
          H(id+j, id+k) = pow((*sample_time_ptr)[i+1] - (*sample_time_ptr)[i],
                              j+k-2*m_traj_dev_order+1)
            / (j+k-2*m_traj_dev_order+1) * permutation(j, m_traj_dev_order)
            * permutation(k, m_traj_dev_order);
        }
      }
    }

    if (m_debug){
      std::cout << "Constrints num: " << n_constraints << "\n";
      std::cout << "Constrints vector size: " << row_id << "\n";
    }

    /* Setting up QProblemB object. */
    real_t *H_r = new real_t[m_traj_order * n_polynomial * m_traj_order * n_polynomial];
    real_t *A_r = new real_t[n_constraints * m_traj_order * n_polynomial];
    real_t *lb_A_r = new real_t[n_constraints];
    real_t *ub_A_r = new real_t[n_constraints];

    for (int i = 0; i < m_traj_order * n_polynomial; ++i){
      int id = i * m_traj_order * n_polynomial;
      for (int j = 0; j < m_traj_order * n_polynomial; ++j){
        H_r[id + j] = H(i, j);
      }
    }
    for (int i = 0; i < n_constraints; ++i){
      int id = i * m_traj_order * n_polynomial;
      for (int j = 0; j < m_traj_order * n_polynomial; ++j){
        A_r[id + j] = A(i, j);
      }
    }
    for (int i = 0; i < n_constraints; ++i){
      lb_A_r[i] = lb_A[i];
      ub_A_r[i] = ub_A[i];
    }

    // debug
    if (m_debug){
      // ROS_INFO("Waiting for qp solver.");
      // std::cout << "constrints num: " << n_constraints << "\n";
      // std::cout << "polynomial num: " << n_polynomial << "\n";
      // std::cout << "\n\nH matrix:\n";
      // for (int i = 0; i < m_traj_order * n_polynomial; ++i){
      //   std::cout << "[" << i << "]: ";
      //   int id = i * m_traj_order * n_polynomial;
      //   for (int j = 0; j < m_traj_order * n_polynomial; ++j){
      //     std::cout << H_r[id+j] << ",";
      //   }
      //   std::cout << "\n";
      // }

      // std::cout << "\n\nA matrix:\n";
      // for (int i = 0; i < n_constraints; ++i){
      //   std::cout << "[" << i << "]: ";
      //   int id = i * m_traj_order * n_polynomial;
      //   for (int j = 0; j < m_traj_order * n_polynomial; ++j){
      //     std::cout << A_r[id+j] << ",";
      //   }
      //   std::cout << "\n";
      // }

      // std::cout << "\n\n lb_A_r vector:\n";
      // for (int i = 0; i < n_constraints; ++i){
      //   std::cout << lb_A_r[i] << ",";
      // }
      // std::cout << "\n";

      // std::cout << "\n\n ub_A_r vector:\n";
      // for (int i = 0; i < n_constraints; ++i){
      //   std::cout << ub_A_r[i] << ",";
      // }
      // std::cout << "\n";
    }

    QProblem exampleQ(m_traj_order * n_polynomial, n_constraints);

    Options options;
    //options.enableFlippingBounds = BT_FALSE;
    // options.initialStatusBounds = ST_INACTIVE;
    // options.numRefinementSteps = 1;
    // options.enableCholeskyRefactorisation = 1;
    // options.enableEqualities = BT_TRUE;
    // options.enableFarBounds = BT_TRUE;
    // options.enableFlippingBounds = BT_TRUE;
    options.printLevel = PL_LOW;
    exampleQ.setOptions( options );
    real_t *G_r = new real_t[m_traj_order * n_polynomial];
    for (int i = 0; i < m_traj_order * n_polynomial; ++i)
      G_r[i] = 0.0;
    int_t nWSR = 300;
    exampleQ.init(H_r, G_r, A_r, NULL, NULL, lb_A_r, ub_A_r, nWSR, 0);
    real_t param_r[m_traj_order * n_polynomial];
    exampleQ.getPrimalSolution(param_r);

    for (int i = 0; i <m_traj_order * n_polynomial; ++i)
      (*traj_param_ptr)[i] = param_r[i];

    std::cout << "\n\nParams:\n";
    for (int i = 0; i <m_traj_order * n_polynomial; ++i){
      if (i % m_traj_order == 0)
        std::cout << "\n";
      std::cout << param_r[i] << ", ";
    }
    // todo: judge whether qp is solved
    return true;
  }

  inline int SamplingBasedTrajectory::permutation(int n, int order)
  {
    int result = 1;
    for (int i = 0; i < order; ++i)
      result *= (n - i);
    return result;
  }

  Vector3d SamplingBasedTrajectory::getnOrderPointFromTrajectory(int order, double t)
  {
    if (t < (*m_sample_time_ptr)[0]){
      ROS_WARN("Try to get points less than start time");
      t = (*m_sample_time_ptr)[0];
    }
    else if (t > (*m_sample_time_ptr)[m_n_samples-1]){
      ROS_WARN("Try to get points larger than end time");
      t = (*m_sample_time_ptr)[m_n_samples-1];
    }
    for (int i = 1; i < m_n_samples; ++i){
      if (t <= (*m_sample_time_ptr)[i]){
        Vector3d pt_value;
        pt_value[0] = getnOrderPointFromTrajectory(order, t, i-1, m_traj_param_x_ptr);
        pt_value[1] = getnOrderPointFromTrajectory(order, t, i-1, m_traj_param_y_ptr);
        pt_value[2] = getnOrderPointFromTrajectory(order, t, i-1, m_traj_param_z_ptr);
        return pt_value;
      }
    }
  }

  double SamplingBasedTrajectory::getnOrderPointFromTrajectory(int order, double t, int sample_id, VectorXd *traj_param_ptr)
  {
    int id = sample_id * m_traj_order;
    double result = 0.0, cal = 1.0;
    for (int i = order; i < m_traj_order; ++i){
      result += (*traj_param_ptr)[id + i] * cal * permutation(i, order);
      cal *= (t - (*m_sample_time_ptr)[sample_id]);
    }
    return result;
  }

  void SamplingBasedTrajectory::trajectoryVisualization()
  {
    nav_msgs::Path traj_path;
    traj_path.header.frame_id = std::string("world");
    traj_path.header.stamp = ros::Time::now();
    int n_visualize_pts = int(((*m_sample_time_ptr)[m_n_samples-1] - (*m_sample_time_ptr)[0])
                              / m_visualize_unit_time);
    for (int i = 0; i <= n_visualize_pts; ++i){
      double t = (*m_sample_time_ptr)[0] + double(i) * m_visualize_unit_time;
      if (i == n_visualize_pts)
        t = (*m_sample_time_ptr)[m_n_samples-1];
      geometry_msgs::PoseStamped cur_pose;
      cur_pose.header = traj_path.header;
      Vector3d cur_vec = getnOrderPointFromTrajectory(0, t);
      cur_pose.pose.position.x = cur_vec[0];
      cur_pose.pose.position.y = cur_vec[1];
      cur_pose.pose.position.z = cur_vec[2];
      traj_path.poses.push_back(cur_pose);
    }
    m_pub_traj_path.publish(traj_path);
  }
}
