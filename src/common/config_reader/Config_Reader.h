//
// Created by lxj on 2026/1/19.
//

#ifndef J6B_PARKING_PNC_CONFIG_READER_H
#define J6B_PARKING_PNC_CONFIG_READER_H
#include <yaml-cpp/yaml.h>
#include <string>

namespace APS_Planning{
    namespace common {
        struct control_trajectory_output_config {
            double lookahead_distance;
            int min_points;
            double sample_interval;
        };
        struct grid_map_config {
            double resolution;
            double map_width;
            double map_height;
            double inflation_radius;
        };
    
        struct parking_space_evaluation_config {
            double min_width;
            double max_width;
            double min_length;
            double max_length;
            double max_angle_diff;
        };
    
        struct parking_space_recommendation_config {
            int max_spaces;
            int default_top_k;
            double distance_weight;
            double angle_weight;
            double size_weight;
        };
    
    
        struct trajectory_optimization_config {
            double max_curvature;
            double collision_margin;
            int max_iterations;
        };
        struct trajectory_smoothing_config {
            double smooth_weight;
            double uniform_weight;
            double geometry_weight;
            int max_iterations;
        };
        struct velocity_planning_config {
            double max_velocity;
            double max_acceleration;
            double max_deceleration;
            int default_profile;
        };
    
    class Config_Reader {
        public:
        Config_Reader();
        Config_Reader(std::string config_file);
        ~Config_Reader();
        //read and get config
        void read_control_trajectory_output_config();
        inline control_trajectory_output_config get_control_trajectory_output_config() { return this->control_trajectory_output_config_; }
        void read_grid_map_config();
        inline grid_map_config get_grid_map_config() { return this->grid_map_config_; }
        void read_parking_space_recommendation_config();
        inline parking_space_recommendation_config get_parking_space_recommendation_config() { return this->parking_space_recommendation_config_; }
        void read_parking_space_evaluation_config();
        inline parking_space_evaluation_config get_parking_space_evaluation_config() { return this->parking_space_evaluation_config_; }
        void read_trajectory_optimization_config();
        inline trajectory_optimization_config get_trajectory_optimization_config() { return this->trajectory_optimization_config_; }
        void read_trajectory_smoothing_config();
        inline trajectory_smoothing_config get_trajectory_smoothing_config() { return this->trajectory_smoothing_config_; }
        void read_velocity_planning_config();
        inline velocity_planning_config get_velocity_planning_config() { return this->velocity_planning_config_; }
        
        private:
        YAML::Node planning_config;
        control_trajectory_output_config control_trajectory_output_config_;
        grid_map_config grid_map_config_;
        parking_space_recommendation_config parking_space_recommendation_config_;
        parking_space_evaluation_config parking_space_evaluation_config_;
        trajectory_optimization_config trajectory_optimization_config_;
        trajectory_smoothing_config trajectory_smoothing_config_;
        velocity_planning_config velocity_planning_config_;
    };

    }// common
    }// APS_Planning

#endif //J6B_PARKING_PNC_CONFIG_READER_H