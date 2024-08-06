
#include "RealVectorSpaceHPPFCL.h"
#include "RealVectorSpaceConfig.h"
#include "xArm6.h"
// #include <glog/log_severity.h>
// #include <glog/logging.h>

base::RealVectorSpaceHPPFCL::~RealVectorSpaceHPPFCL() {}

base::RealVectorSpaceHPPFCL::RealVectorSpaceHPPFCL(size_t num_dimensions_,
                                                   const std::shared_ptr<robots::AbstractRobot> robot_,
                                                   const std::shared_ptr<env::Environment> env_,
                                                   const MDP::ConfigReader::SceneTask scene_task)
    : RealVectorSpace(num_dimensions_, robot_, env_), collision_manager(std::make_shared<MDP::CollisionManager>(scene_task)) {
    this->setStateSpaceType(base::StateSpaceType::RealVectorSpaceHPPFCL);
}

bool base::RealVectorSpaceHPPFCL::isValid(const std::shared_ptr<base::State> q) {
    fcl::DefaultCollisionData<float> collision_data{};
    std::vector<float> pos(q->getCoord().data(), q->getCoord().data() + q->getCoord().rows() * q->getCoord().cols());

    return !this->collision_manager->check_collision(pos, this->env->getTime());
}

// Return minimal distance from robot in configuration 'q' to obstacles
// Compute minimal distance from each robot's link in configuration 'q' to obstacles, i.e., compute distance profile
// function Moreover, set 'd_c', 'd_c_profile', and corresponding 'nearest_points' for the configuation 'q' If
// 'compute_again' is true, the new distance profile will be computed again!
float base::RealVectorSpaceHPPFCL::computeDistance(const std::shared_ptr<base::State> q, bool compute_again) {
    if (!compute_again && q->getDistance() > 0 && q->getIsRealDistance())
        return q->getDistance();

    float d_c{INFINITY};
    std::vector<float> d_c_profile(this->robot->getNumLinks(), 0);

    bool is_collided = false;
	std::vector<float> pos(q->getCoord().data(), q->getCoord().data() + q->getCoord().rows() * q->getCoord().cols());
    std::vector<MDP::CollisionManager::robot_link_distance_to_obj> distances =
        this->collision_manager->get_distances(pos, this->env->getTime(), is_collided);

    if (is_collided) // The collision occurs
    {
        q->setDistance(0);
        q->setDistanceProfile(d_c_profile);
        q->setIsRealDistance(true);
        q->setNearestPoints(nullptr);
        return 0;
    }

    std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points{
        std::make_shared<std::vector<Eigen::MatrixXf>>(std::vector<Eigen::MatrixXf>(
            distances[0].neares_point_robot.size(), Eigen::MatrixXf(6, this->robot->getNumLinks())))};

    for (size_t i = 0; i < this->robot->getNumLinks(); i++) {
        d_c_profile[i] = distances[i].min_distance;
        for (size_t j = 0; j < distances[i].neares_point_robot.size(); j++) {
            nearest_points->at(j).col(i) << distances[i].neares_point_robot[j].cast<float>(), distances[i].neares_point_obj[j].cast<float>();
        }
        d_c = std::min(d_c, d_c_profile[i]);
    }

    q->setDistance(d_c);
    q->setDistanceProfile(d_c_profile);
    q->setIsRealDistance(true);
    q->setNearestPoints(nearest_points);

    return d_c;
}
