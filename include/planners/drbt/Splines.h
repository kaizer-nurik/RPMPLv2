//
// Created by nermin on 20.07.24.
//

#ifndef RPMPL_DRGBT_SPLINES_H
#define RPMPL_DRGBT_SPLINES_H

#include "StateSpace.h"
#include "Spline4.h"
#include "Spline5.h"
#include "CompositeSpline.h"
#include "DRGBTConfig.h"
#include "RRTConnectConfig.h"

namespace planning::drbt
{
    class Splines
    {
    public:
        Splines(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_current_, 
                const std::shared_ptr<base::State> q_target_, bool all_robot_vel_same_ = false);

        bool computeSplineNext(Eigen::VectorXf &current_pos, Eigen::VectorXf &current_vel, Eigen::VectorXf &current_acc, 
                               float t_iter_remain, bool non_zero_final_vel = false);
        bool computeSplineSafe(Eigen::VectorXf &current_pos, Eigen::VectorXf &current_vel, Eigen::VectorXf &current_acc, 
                               float t_iter_remain);
        
        inline void setCurrentState(const std::shared_ptr<base::State> q_current_) { q_current = q_current_; }
        inline void setTargetState(const std::shared_ptr<base::State> q_target_) { q_target = q_target_; }
        
        std::shared_ptr<planning::trajectory::Spline> spline_current;           // Current spline that 'q_current' is following in the current iteration
        std::shared_ptr<planning::trajectory::Spline> spline_next;              // Next spline generated from 'q_current' to 'q_target'

    private:
        bool checkSplineCollision(std::shared_ptr<base::State> q_init, float t_iter);
        float computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
                                             const std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points, float delta_t);
        void recordTrajectory(bool spline_computed);

        std::shared_ptr<base::StateSpace> ss;
        std::shared_ptr<base::State> q_current;                                 // Current robot configuration
        std::shared_ptr<base::State> q_target;                                  // Target robot configuration to which the robot is currently heading to
        bool all_robot_vel_same;                                                // Whether all joint velocities are the same
        float max_obs_vel;                                                      // Maximal velocity considering all obstacles
        size_t max_num_iter_spline_next;                                        // Maximal number of iterations when computing spline_next
        
    };
}

#endif //RPMPL_DRGBT_SPLINES_H