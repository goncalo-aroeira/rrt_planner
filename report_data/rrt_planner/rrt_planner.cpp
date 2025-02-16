
#include <rrt_planner/rrt_planner.h>

namespace rrt_planner {

    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params) : params_(params), collision_dect_(costmap) {

        costmap_ = costmap->getCostmap();
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);

        nodes_.reserve(params_.max_num_nodes);
        
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        nodes_.clear();

        // Start Node
        createNewNode(start_, -1);

        double *p_rand, *p_new;
        Node nearest_node;

        

        for (unsigned int k = 1; k <= params_.max_num_nodes; k++) {

            p_rand = sampleRandomPoint();
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate

            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                createNewNode(p_new, nearest_node.node_id);
                
            } else {
                continue;
            }

            if(k > params_.min_num_nodes) {
                
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance){
                    //mesure path length
                    
                    double path_length = computePathLength();
                    /*if (path_length < prev_path_length || prev_path_length == 0) {
                        prev_path_length = path_length;
                        return true;
                    }*/
                    return true;
                
                }
            
            }
        }

        return false;
    }

    double RRTPlanner::computePathLength() {

        double path_length = 0;
        int node_id = nodes_.size() - 1;

        while (node_id != 0) {
            path_length += computeDistance(nodes_[node_id].pos, nodes_[nodes_[node_id].parent_id].pos);
            node_id = nodes_[node_id].parent_id;
        }
        
        return path_length;
    }

    int RRTPlanner::getNearestNodeId(const double *point) {
        /**************************
         * Implement your code here
         **************************/

        int nearest_node_id = 0;
        double min_dist = std::numeric_limits<double>::max(); // initialize to max value of double type 
        double dist;

        for (unsigned int i = 0; i < nodes_.size(); i++) {

            dist = computeDistance(point, nodes_[i].pos);

            if (dist < min_dist) {
                min_dist = dist;
                nearest_node_id = i;
            }
        }

        return nearest_node_id;
        

    }

    void RRTPlanner::createNewNode(const double* pos, int parent_node_id) {
        /**************************
         * Implement your code here
         **************************/

        Node new_node;

        new_node.pos[0] = pos[0];
        new_node.pos[1] = pos[1];
        new_node.node_id = nodes_.size();
        new_node.parent_id = parent_node_id;

        nodes_.emplace_back(new_node);
        
    }

    double* RRTPlanner::sampleRandomPoint() {
        /**************************
         * Implement your code here
         **************************/

        // Generate a random probability value in the range [0, 1]
        double random_prob = random_double_x.generate();

        // Probability of sampling a point near the goal (50%)
        double near_goal_probability = 0.5;

        if (random_prob < near_goal_probability) {
            // Sample a point near the goal
            // Generate a random angle in the range [0, 2 * PI]
            double random_angle = 2 * M_PI * random_double_x.generate();
            // Generate a random distance in the range [0, max_distance] or as needed
            double max_distance = 0.1; // Adjust as needed
            // Calculate the random point's coordinates based on the angle and distance
            rand_point_[0] = goal_[0] + max_distance * cos(random_angle);
            rand_point_[1] = goal_[1] + max_distance * sin(random_angle);
        } else {
            // Sample a completely random point
            rand_point_[0] = random_double_x.generate();
            rand_point_[1] = random_double_y.generate();
        }

    return rand_point_;
}


    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand) {

        /**************************
         * Implement your code here
         **************************/

        double dist = computeDistance(point_nearest, point_rand);

        double step = params_.step;

        double dist2 = computeDistance(goal_, point_nearest);

        
        if (dist2 + 0.3 > step){
            
            step = dist2 * 0.40 ;
        }
       
        
        
       
        // if dist is less than step, then candidate point is the random point itself 
        if (dist <= step) {
            candidate_point_[0] = point_rand[0];
            candidate_point_[1] = point_rand[1];
        }
        // else candidate point is the point on the line joining nearest point and random point at a distance of step 
        else {
            candidate_point_[0] = point_nearest[0] + step * (point_rand[0] - point_nearest[0]) / dist;
            candidate_point_[1] = point_nearest[1] + step * (point_rand[1] - point_nearest[1]) / dist;
        }

        return candidate_point_;
    }

    const std::vector<Node>& RRTPlanner::getTree() {

        return nodes_;
    }

    void RRTPlanner::setStart(double *start) {

        start_[0] = start[0];
        start_[1] = start[1];
    }

    void RRTPlanner::setGoal(double *goal) {

        goal_[0] = goal[0];
        goal_[1] = goal[1];
    }

};