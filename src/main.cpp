//
// Created by dinko on 7.3.21..
//
#include "Scenario.h"

#include "ConfigurationReader.h"
#include "DRGBT.h"
#include "RealVectorSpaceFCL.h"
#include "RealVectorSpaceState.h"
#include "rapidjson/document.h"
#include "rapidjson/pointer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <iostream>

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

std::vector<float> parse_float_vector_json(const rapidjson::Document &json_doc,const  std::string key) {
    // read json array by key

    std::vector<float> result;
    if (!json_doc.HasMember(key.c_str())) {
        throw std::runtime_error("Error: no value for key " + key + " found in json");
    }
    const rapidjson::Value &value = json_doc[key.c_str()]; 
    for (rapidjson::SizeType i = 0; i < value.Size(); i++) { // rapidjson uses SizeType instead of size_t.
        result.push_back(value[i].GetFloat());
    }

    return result;
}

std::vector<std::string> parse_str_vector_json(const rapidjson::Document &json_doc,const  std::string key) {
    // read json array by key

    std::vector<std::string> result;
    if (!json_doc.HasMember(key.c_str())) {
        throw std::runtime_error("Error: no value for key " + key + " found in json");
    }
    const rapidjson::Value &value = json_doc[key.c_str()]; 
    for (rapidjson::SizeType i = 0; i < value.Size(); i++) { // rapidjson uses SizeType instead of size_t.
        result.push_back(value[i].GetString());
    }

    return result;
}

Eigen::VectorXf vectorFloatToEigenVector(std::vector<float> vec) {
    Eigen::VectorXf eigenVec(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        eigenVec[i] = vec[i];
    }
    return eigenVec;
}


int main(int argc, char **argv) {
    int q = argc;
    char** w = argv;
    if (q == **w){
        return 0;
    }
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

    // 1. Parse a JSON string into DOM.
    std::string json = read_file("./scene.json");
    rapidjson::Document document;
    document.Parse(json.c_str());

    std::vector<std::string> joint_order = parse_str_vector_json(document, "robot_joints_order");



    // Eigen::VectorXf start_conf(joint_order.size());
    Eigen::VectorXf start_conf = vectorFloatToEigenVector(parse_float_vector_json(document, "start_configuration"));
    

    // Eigen::VectorXf end_conf(joint_order.size());
    Eigen::VectorXf end_conf = vectorFloatToEigenVector(parse_float_vector_json(document, "end_configuration"));


    DRGBTConfig::MAX_PLANNING_TIME = 1000000000000000;

    if (DRGBTConfig::STATIC_PLANNER_TYPE == planning::PlannerType::RGBMTStar) {
        RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = true;
    }

    bool nado=true;
    while(nado){
        scenario::Scenario scenario("./scene.yaml", "./");
        std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
        // std::shared_ptr<base::State q_start { scenario.getStart() };
        std::shared_ptr<base::State> q_start {  std::make_shared<base::RealVectorSpaceState>(start_conf) };
        // std::shared_ptr<base::State> q_goal { scenario.getGoal() };
        std::shared_ptr<base::State> q_goal { std::make_shared<base::RealVectorSpaceState>(end_conf )};
        std::shared_ptr<env::Environment> env {  scenario.getEnvironment() };
        std::unique_ptr<planning::AbstractPlanner> planner { nullptr };

        //env->setBaseRadius(std::max(ss->robot->getCapsuleRadius(0), ss->robot->getCapsuleRadius(1)));
        //env->setRobotMaxVel(ss->robot->getMaxVel(0)); // Only velocity of the first joint matters
        // initRandomObstacles(init_num_obs, obs_dim, scenario, max_vel_obs, max_acc_obs);
        env->parse_json_document(document);
        LOG(INFO) << "Using scenario: " << "./scene.json";
        LOG(INFO) << "Environment parts: " << env->getNumObjects();
        LOG(INFO) << "Number of DOFs: " << ss->num_dimensions;
        LOG(INFO) << "State space type: " << ss->getStateSpaceType();
        LOG(INFO) << "Start: " << q_start;
        LOG(INFO) << "Goal: " << q_goal;

        planner = std::make_unique<planning::drbt::DRGBT>(ss, q_start, q_goal);
        bool result{planner->solve()};

        LOG(INFO) << planner->getPlannerType() << " planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
        LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
        LOG(INFO) << "Algorithm time: " << planner->getPlannerInfo()->getPlanningTime() << " [s]";
        LOG(INFO) << "Task 1 interrupted: " << (planner->getPlannerInfo()->getTask1Interrupted() ? "true" : "false");
        LOG(INFO) << "Planner data is saved at: "
                  << "./drgbt_test.log";
        planner->outputPlannerData("./drgbt_test.log");

        float path_length{0};
        if (result) {
            // LOG(INFO) << "Found path: ";
            std::vector<std::shared_ptr<base::State>> path{planner->getPath()};
            for (size_t i = 0; i < path.size(); i++) {
                // std::cout << i << ": " << path.at(i)->getCoord().transpose() << std::endl;
                if (i > 0)
                    path_length += ss->getNorm(path.at(i - 1), path.at(i));
            }
            path_lengths.emplace_back(path_length);
            alg_times.emplace_back(planner->getPlannerInfo()->getPlanningTime());
            iter_times.emplace_back(planner->getPlannerInfo()->getPlanningTime() /
                                    planner->getPlannerInfo()->getNumIterations());
            
        }
    nado = !result;
    }
    return 0;
    }

 