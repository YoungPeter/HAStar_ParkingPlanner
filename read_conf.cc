#include "read_conf.h"

#include <fstream>
#include <iostream>

bool ReadHybridAStarParam(WarmStartConfig &warm_start_config){
    std::ifstream file;
    file.open("config\\hya_param.conf", std::ios_base::in);
    if (!file.is_open()) {
        return false;
    }

    std::string s;
    int n_row = 0;
    while (getline(file, s)){
        size_t pos = s.find(':');
        std::string value_str = s.substr(pos + 2);
        double value = std::stod(value_str);
        switch (n_row){
        	case 0: warm_start_config.xy_grid_resolution = value;
		        break;
	        case 1: warm_start_config.phi_grid_resolution = value;
		        break;
            case 2: warm_start_config.next_node_num = value;
		        break;
            case 3: warm_start_config.step_size = value;
		        break;
            case 4: warm_start_config.traj_forward_penalty = value;
		        break;
            case 5: warm_start_config.traj_back_penalty = value;
		        break;
            case 6: warm_start_config.traj_gear_switch_penalty = value;
		        break;
            case 7: warm_start_config.traj_steer_penalty = value;
		        break;
            case 8: warm_start_config.traj_steer_change_penalty = value;
		        break;
            case 9: warm_start_config.grid_a_star_xy_resolution = value;
		        break;
            case 10: warm_start_config.node_radius = value;
		        break;
            case 11: warm_start_config.delta_t = value;
		        break;
            default: std::cout << "The number of rows is out of range." <<std::endl;
		        break;
	    }
        n_row++;
    }
    file.close();
    return true;
}

bool ReadVehicleParam(VehicleParam &vehicle_param){
    std::ifstream file;
    file.open("config\\vehicle_param.conf", std::ios_base::in);
    if (!file.is_open()) {
        return false;
    }

    std::string s;
    int n_row = 0;
    while (getline(file, s)){
        size_t pos = s.find(':');
        std::string value_str = s.substr(pos + 2);
        double value = std::stod(value_str);
        switch (n_row){
        	case 0: vehicle_param.front_edge_to_center = value;
		        break;
	        case 1: vehicle_param.back_edge_to_center = value;
		        break;
            case 2: vehicle_param.left_edge_to_center = value;
		        break;
            case 3: vehicle_param.right_edge_to_center = value;
		        break;
            case 4: vehicle_param.length = value;
		        break;
            case 5: vehicle_param.width = value;
		        break;
            case 6: vehicle_param.height = value;
		        break;
            case 7: vehicle_param.min_turn_radius = value;
		        break;
            case 8: vehicle_param.max_acceleration = value;
		        break;
            case 9: vehicle_param.max_deceleration = value;
		        break;
            case 10: vehicle_param.max_steer_angle = value;
		        break;
            case 11: vehicle_param.max_steer_angle_rate = value;
		        break;
            case 12: vehicle_param.steer_ratio = value;
		        break;
            case 13: vehicle_param.wheel_base = value;
		        break;
            case 14: vehicle_param.wheel_rolling_radius = value;
		        break;
            case 15: vehicle_param.max_abs_speed_when_stopped = value;
		        break;
            case 16: vehicle_param.brake_deadzone = value;
		        break;
            case 17: vehicle_param.throttle_deadzone = value;
		        break;
            default: std::cout << "The number of rows is out of range." <<std::endl;
		        break;
	    }
        n_row++;
    }
    file.close();
    return true;
}