//
// Created by lxj on 2026/1/19.
//

#include "Config_Reader.h"
#include <stdexcept>

namespace APS_Planning{
    namespace common {
        Config_Reader::Config_Reader() {
            // 尝试多个可能的配置文件路径
            std::string default_paths[] = {
                "config/aps_planning_config.yaml",
                "../config/aps_planning_config.yaml",
                "../../config/aps_planning_config.yaml",
                "../../../config/aps_planning_config.yaml"
            };
            
            bool loaded = false;
            for (const auto& path : default_paths) {
                try {
                    planning_config = YAML::LoadFile(path);
                    loaded = true;
                    break;
                } catch (...) {
                    continue;
                }
            }
            
            if (!loaded) {
                throw std::runtime_error("Failed to load default config file. Please specify config file path explicitly.");
            }
        }
        Config_Reader::Config_Reader(std::string config_file) {
            planning_config = YAML::LoadFile(config_file);
        }
        Config_Reader::~Config_Reader() {
        }

        void Config_Reader::read_control_trajectory_output_config() {
            control_trajectory_output_config_.lookahead_distance = planning_config["control_trajectory_output"]["lookahead_distance"].as<double>();
            control_trajectory_output_config_.min_points = planning_config["control_trajectory_output"]["min_points"].as<int>();
            control_trajectory_output_config_.sample_interval = planning_config["control_trajectory_output"]["sample_interval"].as<double>();
        }
        void Config_Reader::read_grid_map_config() {
            grid_map_config_.resolution = planning_config["grid_map"]["resolution"].as<double>();
            grid_map_config_.map_width = planning_config["grid_map"]["map_width"].as<double>();
            grid_map_config_.map_height = planning_config["grid_map"]["map_height"].as<double>();
            grid_map_config_.inflation_radius = planning_config["grid_map"]["inflation_radius"].as<double>();
        }
        void Config_Reader::read_parking_space_evaluation_config() {
            parking_space_evaluation_config_.min_width = planning_config["parking_space_evaluation"]["min_width"].as<double>();
            parking_space_evaluation_config_.max_width = planning_config["parking_space_evaluation"]["max_width"].as<double>();
            parking_space_evaluation_config_.min_length = planning_config["parking_space_evaluation"]["min_length"].as<double>();
            parking_space_evaluation_config_.max_length = planning_config["parking_space_evaluation"]["max_length"].as<double>();
            parking_space_evaluation_config_.max_angle_diff = planning_config["parking_space_evaluation"]["max_angle_diff"].as<double>();
        }
        void Config_Reader::read_parking_space_recommendation_config() {
            parking_space_recommendation_config_.max_spaces = planning_config["parking_space_recommendation"]["max_spaces"].as<int>();
            parking_space_recommendation_config_.default_top_k = planning_config["parking_space_recommendation"]["default_top_k"].as<int>();
            parking_space_recommendation_config_.distance_weight = planning_config["parking_space_recommendation"]["distance_weight"].as<double>();
            parking_space_recommendation_config_.angle_weight = planning_config["parking_space_recommendation"]["angle_weight"].as<double>();
            parking_space_recommendation_config_.size_weight = planning_config["parking_space_recommendation"]["size_weight"].as<double>();
        }
        void Config_Reader::read_trajectory_optimization_config() {
            trajectory_optimization_config_.max_curvature = planning_config["trajectory_optimization"]["max_curvature"].as<double>();       
            trajectory_optimization_config_.collision_margin = planning_config["trajectory_optimization"]["collision_margin"].as<double>();
            trajectory_optimization_config_.max_iterations = planning_config["trajectory_optimization"]["max_iterations"].as<int>();
        }
        void Config_Reader::read_trajectory_smoothing_config() {
            trajectory_smoothing_config_.smooth_weight = planning_config["trajectory_smoothing"]["smooth_weight"].as<double>();
            trajectory_smoothing_config_.uniform_weight = planning_config["trajectory_smoothing"]["uniform_weight"].as<double>();
            trajectory_smoothing_config_.geometry_weight = planning_config["trajectory_smoothing"]["geometry_weight"].as<double>();
            trajectory_smoothing_config_.max_iterations = planning_config["trajectory_smoothing"]["max_iterations"].as<int>();
        }
        void Config_Reader::read_velocity_planning_config() {
            velocity_planning_config_.max_velocity = planning_config["velocity_planning"]["max_velocity"].as<double>();
            velocity_planning_config_.max_acceleration = planning_config["velocity_planning"]["max_acceleration"].as<double>();
            velocity_planning_config_.max_deceleration = planning_config["velocity_planning"]["max_deceleration"].as<double>();
            velocity_planning_config_.default_profile = planning_config["velocity_planning"]["default_profile"].as<int>();
        }

    }
}