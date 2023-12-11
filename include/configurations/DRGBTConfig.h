//
// Created by nermin on 14.04.22.
//

#include <string>

class DRGBTConfig
{
public:
    static unsigned long MAX_NUM_ITER;          // Maximal number of algorithm iterations
    static float MAX_ITER_TIME;                 // Maximal runtime of a single iteration in [ms] (also 1/f_s, where f_s is the number of frames per second)
    static float MAX_PLANNING_TIME;             // Maximal algorithm runtime in [ms]
    static int INIT_HORIZON_SIZE;               // Initial horizon size
    static float STEP;                          // Advancing step in C-space in [rad] when moving from current towards next state in a single iteration
    static float TRESHOLD_WEIGHT;               // Treshold for the replanning assessment. Range: between 0 and 1
    static float D_CRIT;                        // Critical distance in W-space to compute critical nodes
    static int MAX_NUM_VALIDITY_CHECKS;         // Maximal number of validity checks when robot moves from previous to current configuration, while the obstacles are moving simultaneously
    static std::string STATIC_PLANNER_NAME;     // Name of a static planner (for obtaining the predefined path). Default: "RGBTConnect" or "RGBMT*"
    static std::string REAL_TIME_SCHEDULING;    // "DPS" - Dynamic Priority Scheduling; "FPS" - Fixed Priority Scheduling; "" - Without real-time scheduling    
    static float TASK1_UTILITY;                 // Maximal utility which Task 1 can take from the processor
};