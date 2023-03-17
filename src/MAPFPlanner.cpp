#include <MAPFPlanner.h>


struct AstarNode {
    int location;
    int direction;
    int f,g,h;
    AstarNode* parent;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
};


struct cmp {
    bool operator()(AstarNode* a, AstarNode* b) {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};



void MAPFPlanner::initialize(int preprocess_time_limit) {
    cout << "planner initialize done" << endl;
}


// return next states for all agents
vector<Action> MAPFPlanner::plan(int time_limit) {
  std::vector<Action> actions(env->curr_states.size(), Action::W);
  for (int i = 0; i < env->num_of_agents; i++) {
    cout << "start plan for agent " << i;
    list<pair<int,int>> path;
    if (env->goal_locations[i].empty()) {
      cout << ", which does not have any goal left." << endl;
      path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
    } else {
      cout << " with start and goal: ";
      path = single_agent_plan(env->curr_states[i].location,
                               env->curr_states[i].orientation,
                               env->goal_locations[i].front().first);
    }
    cout<< "current location: " << path.front().first << " current direction: " << 
      path.front().second << endl;
    if (path.front().first != env->curr_states[i].location){
      actions[i] = Action::FW;
    } else if (path.front().second!= env->curr_states[i].orientation){
      int incr = path.front().second - env->curr_states[i].orientation;
      if (incr == 1 || incr == -3){
        actions[i] = Action::CR;
      } else if (incr == -1 || incr == 3){
        actions[i] = Action::CCR;
      } 
    }

  }


  return actions;
  // env->curr_states;
}


list<pair<int,int>> MAPFPlanner::single_agent_plan(int start,int start_direct,int end) {
    cout << start<<" "<<start_direct << " " << end << endl;
    list<pair<int,int>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;

    while (!open_list.empty()) {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        if (curr->location == end) {
            //std::cout<<"test"<<std::endl;
            while(curr->parent!=NULL) {
                //std::cout<<curr->location<<", "<<curr->direction<<" "; 
                path.emplace_front(make_pair(curr->location, curr->direction));
                curr = curr->parent;
            }
            //std::cout<<endl;
            break;
        }
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction);
        for (const pair<int,int>& neighbor: neighbors) {
            //std::cout<<"neighbor: "<<neighbor.first<<" "<<neighbor.second<<std::endl;
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            //std::cout<<"1"<<std::endl;
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end()) {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g) {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            } else {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    
    return path;
}


int MAPFPlanner::getManhattanDistance(int loc1, int loc2) {
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

bool MAPFPlanner::validateMove(int loc, int loc2)
{
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;

    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;

    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;

}


list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction) {
    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(make_pair(location,new_direction));
    //turn 180
    //seems like we do not allow turn 180 anymore
    // if (direction == 0 || direction == 1)
    //     new_direction = direction + 2;
    // else
    //     new_direction = direction - 2;
    // neighbors.emplace_back(make_pair(location,new_direction));
    return neighbors;
}
