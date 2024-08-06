// ./src/main ../../STRRT/scene_task.json ./ ../../STRRT/strrt_config.json
#include "Scenario.h"

#include "ConfigurationReader.h"
#include "DRGBT.h"
#include "RealVectorSpaceHPPFCL.h"
#include "RealVectorSpaceState.h"
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>
#include <cmath>
#include <chrono>
#include <filesystem>
#include "config_read_writer/config_read.hpp"
#include <iostream>
#include "xArm6.h"

auto read_file(std::string path) -> std::string {
    // reads file from path and stores it in string
    constexpr auto read_size = std::size_t(4096);
    auto stream = std::ifstream(path);
    stream.exceptions(std::ios_base::badbit);

    if (not stream) {
        throw std::ios_base::failure("file does not exist");
    }

    auto out = std::string();
    auto buf = std::string(read_size, '\0');
    while (stream.read(&buf[0], read_size)) {
        out.append(buf, 0, stream.gcount());
    }
    out.append(buf, 0, stream.gcount());
    return out;
}

Eigen::VectorXf vectorDoubleToEigenVector(std::vector<double> vec) {
    Eigen::VectorXf eigenVec(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        eigenVec[i] = vec[i];
    }
    return eigenVec;
}

void parse_my_args(int argc, char** argv, std::string& path_to_scene_json, std::string& path_to_result_folder, std::string& path_to_strrt_config_json) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <path_to_scene_json> <path_to_result_folder> <path_to_strrt_config_json>" << std::endl;
        exit(EXIT_FAILURE);
    }

    path_to_scene_json = argv[1];
    path_to_result_folder = argv[2];
    path_to_strrt_config_json = argv[3];
}

void check_args(const std::string& path_to_scene_json, const std::string& path_to_result_folder, const std::string& path_to_strrt_config_json) {
    if (!std::ifstream(path_to_scene_json) || !std::ifstream(path_to_strrt_config_json) || !std::filesystem::is_directory(path_to_result_folder)) {
        throw std::runtime_error("Invalid file or directory path");
    }
}

int main(int argc, char** argv) {
    const std::string project_path {"./RPMPLv2" };
	ConfigurationReader::initConfiguration(project_path);

    std::string path_to_scene_json;
    std::string path_to_result_folder;
    std::string path_to_strrt_config_json; 
    parse_my_args(argc, argv, path_to_scene_json, path_to_result_folder, path_to_strrt_config_json);
    check_args(path_to_scene_json, path_to_result_folder, path_to_strrt_config_json);

    MDP::ConfigReader SceneTask(path_to_scene_json);

    std::ifstream config_file(path_to_strrt_config_json);
    rapidjson::IStreamWrapper config_wrapper(config_file);
    rapidjson::Document planner_parsed_config;
    planner_parsed_config.ParseStream(config_wrapper);

    
    int max_planning_time_pts = planner_parsed_config["max_planning_time_pts"].GetInt();
    float max_allowed_time_to_move = planner_parsed_config["max_allowed_time_to_move"].GetFloat();
    bool show_GUI = planner_parsed_config["show_GUI"].GetBool();
    float collision_check_interpolation_angle = planner_parsed_config["collision_check_interpolation_angle"].GetFloat();
    float iteration_time_step_sec = planner_parsed_config["iteration_time_step_sec"].GetFloat();
    bool  stop_if_path_found = planner_parsed_config["stop_if_path_found"].GetBool();

    std::vector<std::string> routines // Routines of which the time executions are stored
        {
            "replan [ms]",          // 0
            "computeDistance [us]", // 1
            "generateGBur [ms]",    // 2
            "generateHorizon [us]", // 3
            "updateHorizon [us]"    // 4
        };

    std::vector<float> alg_times{};
    std::vector<float> iter_times{};
    std::vector<float> path_lengths{};

    DRGBTConfig::MAX_PLANNING_TIME = 1000000000000000;

    if (DRGBTConfig::STATIC_PLANNER_TYPE == planning::PlannerType::RGBMTStar) {
        RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = true;
    }

    bool nado=true;
    while(nado){
        std::shared_ptr<env::Environment> env = std::make_shared<env::Environment>();
        std::string json = read_file(path_to_scene_json);
        rapidjson::Document document;
        document.Parse(json.c_str());
        env->parse_json_document(document);
        std::shared_ptr<robots::xArm6> robot = std::make_shared<robots::xArm6>(SceneTask.get_scene_task().robot_urdf_path ,0,false);
        
        std::vector<double> temp_vel(SceneTask.get_scene_task().robot_joint_max_velocity);
        std::vector<float> max_speed_float(temp_vel.begin(), temp_vel.end());
        robot->setMaxVel(max_speed_float );

        std::vector<double> temp_caps(SceneTask.get_scene_task().robot_capsules_radius);
        std::vector<float> robot_capsules_radius_float(temp_caps.begin(), temp_caps.end());
        robot->setCapsulesRadius(robot_capsules_radius_float);

        std::shared_ptr<base::RealVectorSpaceHPPFCL> ss = std::make_shared<base::RealVectorSpaceHPPFCL>(SceneTask.get_scene_task().robot_joint_count, robot, env,SceneTask.get_scene_task());
        std::shared_ptr<base::State> q_start =std::make_shared<base::RealVectorSpaceState>(vectorDoubleToEigenVector(SceneTask.get_scene_task().start_configuration)) ;
        std::shared_ptr<base::State> q_goal  = std::make_shared<base::RealVectorSpaceState>(vectorDoubleToEigenVector(SceneTask.get_scene_task().end_configuration ));
        std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::drbt::DRGBT>(ss, q_start, q_goal);;

        //env->setBaseRadius(std::max(ss->robot->getCapsuleRadius(0), ss->robot->getCapsuleRadius(1)));
        //env->setRobotMaxVel(ss->robot->getMaxVel(0)); // Only velocity of the first joint matters
        // initRandomObstacles(init_num_obs, obs_dim, scenario, max_vel_obs, max_acc_obs);
        // env->parse_json_document(document);
        LOG(INFO) << "Using scenario: " << "./scene.json";
        LOG(INFO) << "Environment parts: " << env->getNumObjects();
        LOG(INFO) << "Number of DOFs: " << ss->num_dimensions;
        LOG(INFO) << "State space type: " << ss->getStateSpaceType();
        LOG(INFO) << "Start: " << q_start;
        LOG(INFO) << "Goal: " << q_goal;

        bool result{planner->solve()};

        LOG(INFO) << planner->getPlannerType() << " planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
        LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
        LOG(INFO) << "Algorithm time: " << planner->getPlannerInfo()->getPlanningTime() << " [s]";
        LOG(INFO) << "Task 1 interrupted: " << (planner->getPlannerInfo()->getTask1Interrupted() ? "true" : "false");
        LOG(INFO) << "Planner data is saved at: "
                  << "./drgbt_test.log";
        planner->outputPlannerData("./drgbt_test.log");

        float path_length{0};
      
    nado = !result;
    }
    return 0;
    }

 