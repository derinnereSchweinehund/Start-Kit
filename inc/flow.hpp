#ifndef flow_hpp
#define flow_hpp

#pragma once
#include "Types.h"
#include "Grid.h"

//directions for setup traffic for picking stations
const int h_channel_direction[] = {3,1};
const int h_turn_direction[] = {0,2};
const int v_channel_direction[] = {2,0};
const int v_turn_direction[] = {1,3};


void top_traffic(Grid& grid, std::vector<int>& traffic, int tl_corner){
    
    for (int i = 0; i<=2;i++){
        if (i == 0){
            traffic[tl_corner] = h_turn_direction[0];
            traffic[tl_corner+1] = h_channel_direction[1];
            traffic[tl_corner+5] = h_channel_direction[1];
            traffic[tl_corner+6] = h_turn_direction[1];
            continue;
        }
        traffic[tl_corner + i*grid.cols] = h_channel_direction[0];
        traffic[tl_corner + i*grid.cols+1] = h_channel_direction[1];
        traffic[tl_corner + i*grid.cols+5] = h_channel_direction[1];
        traffic[tl_corner + i*grid.cols+6] = h_channel_direction[0];
    }

}

void bottom_traffic(Grid& grid, std::vector<int>& traffic, int tl_corner){

    for (int i = 0; i<=2;i++){
        if (i == 2){
            traffic[tl_corner + i*grid.cols] = h_turn_direction[0];
            traffic[tl_corner + i*grid.cols+1] = h_channel_direction[0];
            traffic[tl_corner + i*grid.cols+5] = h_channel_direction[0];
            traffic[tl_corner + i*grid.cols+6] = h_turn_direction[1];
            continue;
        }
        traffic[tl_corner + i*grid.cols] = h_channel_direction[1];
        traffic[tl_corner + i*grid.cols+1] = h_channel_direction[0];
        traffic[tl_corner + i*grid.cols+5] = h_channel_direction[0];
        traffic[tl_corner + i*grid.cols+6] = h_channel_direction[1];
    }

}

void left_traffic(Grid& grid, std::vector<int>& traffic, int tl_corner){

    for (int i = 0; i<=2;i++){
        if (i == 0){
            traffic[tl_corner] = v_turn_direction[0];
            traffic[tl_corner+grid.cols] = v_channel_direction[1];
            traffic[tl_corner+grid.cols*5] = v_channel_direction[1];
            traffic[tl_corner+grid.cols*6] = v_turn_direction[1];
            continue;
        }
        traffic[tl_corner + i] = v_channel_direction[0];
        traffic[tl_corner+grid.cols + i] = v_channel_direction[1];
        traffic[tl_corner+grid.cols*5 + i] = v_channel_direction[1];
        traffic[tl_corner+grid.cols*6 + i] = v_channel_direction[0];
    }

}

void right_traffic(Grid& grid, std::vector<int>& traffic, int tl_corner){

    for (int i = 0; i<=2;i++){
        if (i == 2){
            traffic[tl_corner + i] = v_turn_direction[0];
            traffic[tl_corner+grid.cols + i] = v_channel_direction[0];
            traffic[tl_corner+grid.cols*5 + i] = v_channel_direction[0];
            traffic[tl_corner+grid.cols*6 + i] = v_turn_direction[1];
            continue;
        }
        traffic[tl_corner + i] = v_channel_direction[1];
        traffic[tl_corner+grid.cols + i] = v_channel_direction[0];
        traffic[tl_corner+grid.cols*5 + i] = v_channel_direction[0];
        traffic[tl_corner+grid.cols*6 + i] = v_channel_direction[1];
    }

}

void build_traffic_endpoint_warehouse(Grid& grid, std::vector<int>& traffic){
    assert(grid.map.size() == traffic.size());
    if (grid.map.size() != traffic.size())
        traffic.resize(grid.map.size(),-1);
    


    bool parttern[] = {false,false,true,true,true,false,false};
    bool match;
    for (int i = 0; i < grid.cols-1; i++){
        if (i + 6 >= grid.cols){
            continue;
        }
        
        match = true;
        for (int j = 0; j < 7; j++){
            if (grid.map[i+j] != parttern[j]){
                match = false;
                break;
            }

        }

        if (match)
            top_traffic(grid,traffic,i);

        match = true;
        int loc = (grid.rows-3)*grid.cols + i;
        for (int j = 0; j < 7; j++){
            if (grid.map[loc+j] != parttern[j]){
                match = false;
                break;
            }

        }

        if (match)
            bottom_traffic(grid,traffic,loc);
    }

    for (int i = 0; i < grid.rows-1; i++){
        if (i + 6 >= grid.rows){
            continue;
        }
        
        match = true;
        int loc = i*grid.cols;
        for (int j = 0; j < 7; j++){
            if (grid.map[loc+j*grid.cols] != parttern[j]){
                match = false;
                break;
            }

        }

        if (match)
            left_traffic(grid,traffic,loc);

        match = true;
        loc = i*grid.cols + grid.cols-3;
        for (int j = 0; j < 7; j++){
            if (grid.map[loc+j*grid.cols] != parttern[j]){
                match = false;
                break;
            }

        }

        if (match)
            right_traffic(grid,traffic,loc);
    }
    
    
}



void build_traffic(Grid& grid, std::vector<int>& traffic){
    //traffic assignes valid outgoing/incoming direction(edge) for each location
    traffic.resize(grid.map.size(),-1);
    //scan for every location on each row, if a location have obstacle on top and bottom but free on left and right
    //assign right (0) to each traffic[location] if have traffic set on current row
    //assign left (2) to each traffic[locaiton] if have traffic set on next row
    int d=0;
    bool have_rule;

    for (int i = 1; i < grid.rows-1; i++){
        if (have_rule && d ==0){
            d = 2;
        }
        else if (have_rule && d == 2){
            d = 0;
        }

        have_rule = false;
        for (int j = 1; j < grid.cols-1; j++){
            int location = i*grid.cols + j;
            if (grid.map[location] == 1){
                traffic[location] = -1;
                continue;
            }
            
            //if location have obstacle on top and bottom but obstacle free on left and right
            //assign d to each traffic[location]
            int candidates[4] = { location + 1,location + grid.cols, location - 1, location - grid.cols};
            if (
                (i == 0 || grid.map[candidates[3]] == 1) &&
                (i == grid.rows - 1 || grid.map[candidates[1]] == 1)
            ){
                if (grid.map[candidates[0]] == 0 && grid.map[candidates[2]] == 0){
                    traffic[location] = d;
                    have_rule = true;
                }
            }
            

            
        }
    }

    //simular thing for columns but check if left and right have obstacle and top and bottom are free
    //assign down (1) to each traffic[location] if have traffic set on current column
    //assign up (3) to each traffic[location] if have traffic set on next column
    d=1;
    have_rule = false;
    for (int j = 1; j < grid.cols-1; j++){
        if (have_rule && d ==1){
            d = 3;
        }
        else if (have_rule && d == 3){
            d = 1;
        }

        have_rule = false;
        for (int i = 1; i < grid.rows-1; i++){
            int location = i*grid.cols + j;
            if (grid.map[location] == 1){
                traffic[location] = -1;
                continue;
            }
            
            //if location have obstacle on left and right but obstacle free on top and bottom
            //assign d to each traffic[location]
            int candidates[4] = { location + 1,location + grid.cols, location - 1, location - grid.cols};
            if (
                (j == 0 || grid.map[candidates[2]] == 1) &&
                (j == grid.cols - 1 || grid.map[candidates[0]] == 1)
            ){
                if (grid.map[candidates[1]] == 0 && grid.map[candidates[3]] == 0){
                    traffic[location] = d;
                    have_rule = true;
                }
            }
            

            
        }
    }
}



#endif
