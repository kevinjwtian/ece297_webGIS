/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include "m1.h"
#include "m4.h"
#include "StreetsDatabaseAPI.h"
#include "m3.h"
#include <iterator>

#define BIG_ENOUGH 999999
using namespace std;
struct IS_Status
{
    vector<int> delivery_status;
    vector<bool> inter_visit_status;    //false, unvisited; true, visited
    float load;
};

//delivery status
#define UNPROCESSED 0
#define PICKED      1
#define DROPED      2

struct Solution
{
    double value;
    vector<int> sequence;
};

#define NI_PICKUP  0 //pickup
#define NI_DROPOFF 1 //dropoff

struct Node_Info
{
    int inter_idx;
    int delivery_idx;    // delivery task idx
    int type;           // Pickup/Dropoff
    double weight;      // weight of deliveries; -1 if inter_idx is a depot
    int nearest_from;   // nearest depot from which travel to this node
    double from_time;   //depot to inter_idx
    int nearest_to;     //nearest depot which this node goes to
    double to_time;     //inter_idx to depot
};

struct Inter_Info
{
    double travel_time;
    int inter_idx;
    int segment_idx;
    int street_idx;
    int from_idx;
    int start_idx;
    int status;         //0 unreached, 1 open, 2 closed
};

struct Inter_F
{
    int inter_idx;
    double travel_time;
};

void visit_inter(const int inter, IS_Status & is_status);//update is_status when visiting the inter
bool init_solution(int num_inters, float truck_capacity, Solution& solution);
void init_is_status(int num_delivery_inters, IS_Status & is_status);//initialize status
bool check_finish(const IS_Status& is_status);
void pertubation(Solution& solution, int no_improvement, float truck_capacity);
pair<bool,Solution> neighborhood_search(Solution& base_solution, float truck_capacity);
Solution fix_solution(const Solution& solution, float truck_capacity);
Solution ils(Solution init_solution, float truck_capacity);
bool check_legality(const Solution &solution, float truck_capacity);//check if a solution is legal or not
int my_randint(int a, int b);
bool m4_cmp_Inter_F(const Inter_F& elm_1, const Inter_F& elm_2);
int try_segment(Inter_Info from_inter_info, int segment_idx, double turn_penalty,
    vector<Inter_F>& f_min_heap, vector<Inter_Info>&inter_info);
void get_travel_time(vector<int> from_inters, double turn_penalty,vector<vector<double>>& travel_time_ret);
void my_swap(vector<int> &sequence, int a, int b);//swap the ath and bth element of a vector
void get_obj(Solution& solution);//get value (objective) of solution
bool try_visit_inter(const int inter, const float truck_capacity, const IS_Status & is_status);
vector<CourierSubpath> convert_solution(const Solution& solution, const double turn_penalty, const vector<int>& depots);

bool m4_cmp_Inter_F(const Inter_F& elm_1, const Inter_F& elm_2)
{
    return elm_1.travel_time > elm_2.travel_time;
}

vector<Node_Info>   node_info;  //store Node_Info
vector<int>         all_inters; 
vector<vector<double>> travel_time_matrix; //2D vectorF
vector<vector<int>> task_vector;

int try_segment(Inter_Info from_inter_info, int segment_idx, double turn_penalty,
    vector<Inter_F>& f_min_heap, vector<Inter_Info>&inter_info)
{
    int to_inter; //which intersection is connected to from_inter through the segment
    InfoStreetSegment seg_info = getInfoStreetSegment(segment_idx);//get info of the segment_idx (connects to from_inter)
    if (seg_info.from == from_inter_info.inter_idx)//if segment from is the given node(from_inter), then segment to is to_inter
        to_inter = seg_info.to;
    else if (seg_info.oneWay)//the segment is oneway only so cannot be used
        return 1;   // 1 means segment it unpassable from from_inter_info through this segment
    else
        to_inter = seg_info.from;//segment from is to _inter

    //if to_inter is in close_inter
    if (inter_info[to_inter].status == 2) //if to_inter is closed, cannot be used, return
        return 2;   // 2 means the end of the segment is closed.

    //to_inter is open or never reached 
    double new_travel_time = from_inter_info.travel_time + find_street_segment_travel_time(segment_idx);//calculate new travel_time
    if (from_inter_info.street_idx != seg_info.streetID)  // needs to add turn penalty if are different streets
        new_travel_time += turn_penalty;
    
    if (new_travel_time >= inter_info[to_inter].travel_time)
        return 3;
    
    //new travel time is smaller than original one
    //make a new Inter_Info, update its information
    inter_info[to_inter] = Inter_Info{
        new_travel_time,
        to_inter,
        segment_idx,
        seg_info.streetID,
        from_inter_info.inter_idx,
        from_inter_info.start_idx,
        1
    };
    //push it into f_min_heap
    f_min_heap.push_back(Inter_F{ to_inter, new_travel_time });
    std::push_heap(f_min_heap.begin(), f_min_heap.end(), m4_cmp_Inter_F);
    return 4;       //means the intersection is open or updated
}

//from_inters, set of start. for any intersection except depot, the size should be 1
//for depots, its size should be the number of depots, which means start a multi distance algorithm
//all_inters, all intersections including pickup, dropoff and depots.
//turn_penalty
//the last, its size should equal to from_inters
void get_travel_time(vector<int> from_inters, double turn_penalty,vector<vector<double>>& travel_time_ret)
{
    //initial travel_time with BIG_ENOUGH, whose size is from_inters.size() * all_inters.size()
    vector<double> default_vector;
    default_vector.resize(all_inters.size(),BIG_ENOUGH);//initialize default_vector
    travel_time_ret.resize(from_inters.size(),default_vector);//initialize travel_time_ret
    
    
    int num_inter = getNumIntersections();
    std::vector<Inter_F> f_min_heap;
    vector<Inter_Info> inter_info; 
    
    //use this default_ii to initialize inter_info
    Inter_Info default_ii{
        BIG_ENOUGH,          //travel time
        -1,         //inter_idx
        -1,         //segment_idx
        -1,         //street_idx
        -1,         //from_idx
        -1,         //start_idx
        0,          //status,0 unreached, 1 open, 2 closed
    };
    
    //to store information of intersections, 
    //intersection A in inter_info[A]
    inter_info.resize(num_inter, default_ii);
    
    //initial start_inter(s), 
    for (auto start_inter : from_inters)
    {
        inter_info[start_inter] = Inter_Info{
            -turn_penalty,      //set travel_time -turn_penalty
            start_inter,    //set curr_inter as start_inter
            -1,
            -1,
            -1,
            start_inter,    //set start intersections,(for multi_distance, multi_starts, need to differentiate)
            1       // set status open
        };
        //push into f_min_heap
        f_min_heap.push_back(Inter_F{ start_inter, 0 });
    }
    make_heap(f_min_heap.begin(), f_min_heap.end(), m4_cmp_Inter_F);
    
    //start to loop, break until all_inters are closed or no more open inters
    while(1)
    {
        // find the intersection with smallest f among all open inters
        Inter_F inter_f;
        bool found_open = false;
        while (1)
        {
            if (f_min_heap.size() == 0) //if empty
                break;

            inter_f = f_min_heap.front();
            std::pop_heap(f_min_heap.begin(), f_min_heap.end(), m4_cmp_Inter_F);
            f_min_heap.pop_back();
            
            if (inter_info[inter_f.inter_idx].status == 1) //its status is open, then the open is found
            {
                found_open = true;          //set found flag true
                inter_info[inter_f.inter_idx].status = 2;  //set its status closed
                break;
            }
        }
        if (!found_open)    //if not open inter is found
        {
            std::cout << "no more open inters" << std::endl;
            break; //break the first while(1)
        }

        Inter_Info smallest_inter_info = inter_info[inter_f.inter_idx];//the result is a struct because this set is sorted according to f value, so the begin one has the smallest f value
        int from_inter = smallest_inter_info.inter_idx;            //let from_inter be the smallest inter red it from above struct

        // set all_closed true,
        bool all_closed = true;
        for (auto dest_inter : all_inters)
        {
            if (inter_info[dest_inter].status != 2)
            {
                all_closed = false;
                break;
            }
        }
        if (all_closed)
            break;

        // spread from the smallest_inter
        //put all neighbour segements of from_inter into segment_idx_vector
        std::vector <int> segment_idx_vector = find_street_segments_of_intersection(from_inter);
        //loop all segments from segment_idx_vector
        for (auto segment_idx : segment_idx_vector)
        {
            try_segment(smallest_inter_info, segment_idx, turn_penalty, f_min_heap, inter_info);//try_segment connected to from_inter based on these values
        }
    }
    
    map<int,int> inter_num;//create a map between intersection's absolute index and its relative index in from_inters
    for(int i=0;i<from_inters.size();i++) //insert pairs into map
    {
        inter_num.insert(make_pair(from_inters[i],i));
    }
    //put travel time into travel_time_ret
    //loop all inters involved
    for(int i = 0; i< all_inters.size(); i++)
    {
        int inter = all_inters[i];  //get intersection i
        int from_idx = inter_info[inter].start_idx;     // get start the intersection's travel time correspongding to 
        //verify here
        double travel_time = inter_info[inter].travel_time; //get the travel time
        if(inter == from_idx)          
            travel_time = 0;
            ///get relative index of inter
        int from_num = inter_num.find(from_idx)->second;//supposed to be 0 if from_inter.size()==1
        travel_time_ret[from_num][i] = travel_time;
    } 
}



void init_is_status(int num_delivery_inters, IS_Status & is_status)//initialize status
{
    is_status.delivery_status.resize(num_delivery_inters/2,UNPROCESSED);
    is_status.inter_visit_status.resize(num_delivery_inters,false);
    is_status.load = 0;
}

bool check_finish(const IS_Status& is_status)
{
    for(auto flag_visit: is_status.inter_visit_status)//check visit status
    {
        if(!flag_visit)
            return false;
    }
    for(auto ds : is_status.delivery_status)//check delivery status
    {
        if(ds!=DROPED)
            return false;
    }
    return true;//finish means that all nodes are visited and dropped
}
bool try_visit_inter(const int inter, const float truck_capacity, const IS_Status & is_status)
{
        //check first if has been visited
    if(is_status.inter_visit_status[inter])
        return false;
    
    //if pickup, check load
    if(node_info[inter].type==NI_PICKUP)
    {
        if(node_info[inter].weight + is_status.load > truck_capacity)
            return false;
    }
    //if drop, check if has been loaded
    else
    {
        if(is_status.delivery_status[node_info[inter].delivery_idx]!=PICKED)
            return false;
    }
    return true;
}
void visit_inter(const int inter, IS_Status & is_status)//update is_status when visiting the inter
{

    is_status.inter_visit_status[inter] = true;//set visit_status to be true
    if(node_info[inter].type==NI_PICKUP)//if it is a pickup node, update load and delivery status
    {
        is_status.load += node_info[inter].weight;
        is_status.delivery_status[node_info[inter].delivery_idx]=PICKED;
    }
    else
    {
        is_status.load -= node_info[inter].weight;
        is_status.delivery_status[node_info[inter].delivery_idx]=DROPED;//if it is a dropoff node, update load and delivery status
    }
}

vector<CourierSubpath> convert_solution(const Solution& solution, const double turn_penalty, const vector<int>& depots)
{
    vector<CourierSubpath> ret;//result container
    
    int curr_index = node_info[solution.sequence[0]].nearest_from;//relative index 
    int curr_inter_index = depots[curr_index];//absolute index
    
    vector<unsigned> pickuped;//pickup items at start intersection
    int next_index;
    int next_inter_index;
    
    //start loop,load all solution information
    for(int i =0;i< solution.sequence.size();i++)
    {
        next_index = solution.sequence[i];//relative index of next node
        next_inter_index = node_info[next_index].inter_idx;//absolute index of next node
        
        CourierSubpath temp_cs;//make a subpath, and load its information
        temp_cs.start_intersection = curr_inter_index;
        temp_cs.end_intersection = next_inter_index;
        temp_cs.subpath = find_path_between_intersections(
                temp_cs.start_intersection,
                temp_cs.end_intersection,
                turn_penalty);
        temp_cs.pickUp_indices = pickuped;
        ret.push_back(temp_cs);//put subpath in result container
        
        //for next loop, turn next node into current node
        curr_index = next_index;
        curr_inter_index = next_inter_index;
        
        pickuped.clear();
        if(node_info[curr_index].type == NI_PICKUP)//load pickup information into pickuped
        {
            unsigned curr_item = node_info[curr_index].delivery_idx;
            pickuped.push_back(curr_item);
        }
    }
    
    //find nearest depot to the last node(dropoff)
    next_index = node_info[curr_index].nearest_to;
    next_inter_index = depots[next_index];
    
    CourierSubpath temp_cs;//make a subpath, and load its information
    temp_cs.start_intersection = curr_inter_index;
    temp_cs.end_intersection = next_inter_index;
    temp_cs.subpath = find_path_between_intersections(
            temp_cs.start_intersection,
            temp_cs.end_intersection,
            turn_penalty);
    temp_cs.pickUp_indices = pickuped;
    ret.push_back(temp_cs);
    
    return ret;
}
//num_delivery_inters equals to deliveries.size()*2;
//0 -num_delivery_inters -1, travel_time_matrix, node_info
//fill solution
bool init_solution(int num_inters, float truck_capacity, Solution& solution)
{
    IS_Status is_status; //an object to keep necessary status
    init_is_status(num_inters, is_status);//initialize is_status
    //find a pickup node with smallest from_time (from depot to pickup node)
    int start = -1;
    int smallest_from = BIG_ENOUGH;
    for(int i=0;i<num_inters;i++)
    {
        if(node_info[i].type == NI_PICKUP && node_info[i].from_time<smallest_from)
        {
            smallest_from = node_info[i].from_time;
            start = i;
        }
    }
    if(!try_visit_inter(start,truck_capacity,is_status))
        return false;//false data
    solution.sequence.push_back(start);
    visit_inter(start,is_status);//update visit_inter
    int curr = start;
    //from start find next
    //policy is : choose the nearest allowed intersection
    //for dropoff, allowed means,the item has been loaded
    //for pickup, allowed means, has enough capacity left to pickup
    while(1)
    {
        int next = -1;
        double smallest_time = BIG_ENOUGH;
        for(int i =0;i<num_inters;i++)//loop all nodes
        {
            if(travel_time_matrix[curr][i]<smallest_time)
            {
                if(try_visit_inter(i,truck_capacity,is_status))//check if this node is allowed
                {
                    next = i;
                    smallest_time = travel_time_matrix[curr][i];//update smallest_time (from current to node i)
                }
            }
        }
        if(next>=0)//find nearest allowed intersection
        {
            visit_inter(next,is_status);
            solution.sequence.push_back(next);
            curr = next;
        }
        else
        {
            if(!check_finish(is_status))//if cannot find nearest allowed intersection and is not finished 
                cout<<"ERROR HERE"<<endl;
            break;
        }
    }
    return true;
}

bool check_legality(const Solution &solution, float truck_capacity)//check if a solution is legal or not
{
    IS_Status is_status;
    init_is_status(solution.sequence.size(), is_status);
    for(auto visit: solution.sequence)
    {
        if(try_visit_inter(visit, truck_capacity, is_status))//try one node
            visit_inter(visit, is_status);//go to this legal node
        else
            return false;
    }
    return true;//go through all legal nodes
}

void get_obj(Solution& solution)//get value (objective) of solution
{
    double obj = 0;
    int num = solution.sequence.size();
    for(int i=0;i<num-1;i++)
    {
        obj += travel_time_matrix[solution.sequence[i]][solution.sequence[i+1]];//travel time between two nodes
    }
    obj += node_info[solution.sequence[0]].from_time;//travel time between start depot to the first node
    obj += node_info[solution.sequence[num-1]].to_time;//travel time between the last node to the end depot
    solution.value = obj;
}

void my_swap(vector<int> &sequence, int a, int b)//swap the ath and bth element of a vector 
{
    int buffer =sequence[a];
    sequence[a] =  sequence[b];
    sequence[b] = buffer;
}

Solution fix_solution(const Solution& solution, float truck_capacity)
{
    Solution base_solution = solution;
    IS_Status is_status;
    init_is_status(base_solution.sequence.size(), is_status);
    for(int i=0;i<base_solution.sequence.size();i++)
    {
        int visit = base_solution.sequence[i];
        if(try_visit_inter(visit, truck_capacity, is_status))
            visit_inter(visit, is_status);
        else
        {
            //illegal when visiting pickup, means overload
            //insert a nearest dropoff before it
            if(node_info[visit].type == NI_PICKUP)  
            {
                //get items that have been loaded
                vector<int> loaded_item;
                for(int j=0;j<is_status.delivery_status.size();j++)
                {
                    if(is_status.delivery_status[j]==PICKED)
                        loaded_item.push_back(j);
                }
                //find best insertion
                double travel_time_incre = BIG_ENOUGH;
                int rel_index = -1;
                for(auto item:loaded_item)
                {
                    int item_dropoff = item*2+1;
                    if(item_dropoff < travel_time_incre)
                    {
                        travel_time_incre = item_dropoff;
                        rel_index = item_dropoff;
                    }
                }
                //do the best found insertion

                
                vector<int>::iterator found_ret;
                found_ret = find(base_solution.sequence.begin(),base_solution.sequence.end(),rel_index);
                base_solution.sequence.erase(found_ret);
                found_ret = find(base_solution.sequence.begin(),base_solution.sequence.end(),visit);
                base_solution.sequence.insert(found_ret,rel_index);
            }
            //illegal when visiting dropoff, mean delivery has not been loaded
            //move this to postion after its pickup
            //visit is a dropoff
            else        
            {
                //erase from its current position
                if(*(base_solution.sequence.begin()+i)!= visit)
                    cout<<"ERROR HERE !!!"<<endl;
                base_solution.sequence.erase(base_solution.sequence.begin()+i);
                //since visit is a dropoff, its pickup equals to visit-1
                vector<int>::iterator iter = find(base_solution.sequence.begin(),base_solution.sequence.end(),visit-1);
                base_solution.sequence.insert(iter+1,visit);
            }
            i=i-1;  //redo this position
        }
    }
    return base_solution;   
}

pair<bool,Solution> neighborhood_search(Solution& base_solution, float truck_capacity)
{
    int num = base_solution.sequence.size();
    get_obj(base_solution);
    Solution best_solution;
    best_solution.value = BIG_ENOUGH;
    Solution temp_solution;
    
    
    for(int i =0;i<num-3;i++)   //exclude the last one
    {
        for(int j=i+2;j<num-1;j++)  
        {
            //try make a new solution
            //exchange [i][i+1] and [j][j+1], 
            temp_solution.sequence = base_solution.sequence;
            //both normally
            my_swap(temp_solution.sequence, i, j);
            my_swap(temp_solution.sequence,i+1,j+1);
            if(check_legality(temp_solution,truck_capacity))
            {
                get_obj(temp_solution);
                if(temp_solution.value < best_solution.value)
                    best_solution = temp_solution;
            }
            //reverse the first
            my_swap(temp_solution.sequence,i,i+1);
            if(check_legality(temp_solution,truck_capacity))
            {
                get_obj(temp_solution);
                if(temp_solution.value < best_solution.value)
                    best_solution = temp_solution;
            }
            //reverse the both
            my_swap(temp_solution.sequence,j,j+1);
            if(check_legality(temp_solution,truck_capacity))
            {
                get_obj(temp_solution);
                if(temp_solution.value < best_solution.value)
                    best_solution = temp_solution;
            }
            //reverse the second
            my_swap(temp_solution.sequence,i,i+1);
            if(check_legality(temp_solution,truck_capacity))
            {
                get_obj(temp_solution);
                if(temp_solution.value < best_solution.value)
                    best_solution = temp_solution;
            }        
        }
        //accept first improvement
        if(best_solution.value < base_solution.value)
        {
            break;
        }
    }
    if(best_solution.value < base_solution.value)
        return make_pair(true,best_solution);
    else
        return make_pair(false,best_solution);
}
#include "math.h"
int my_randint(int a, int b)
{
    int diff = b - a;
    double interval = (double)(RAND_MAX) / diff;
    int p =(int)(rand()/interval);
    return p + a;
}
void pertubation(Solution& solution, int no_improvement, float truck_capacity)
{
    int ni_times[4][2] = {
        {1,3},
        {3,5},
        {5,10},
        {100,15}
    };
    int per_times=0;
    for(int i =0;i<4;i++)
    {
        if (no_improvement < ni_times[i][0])
        {
            per_times = ni_times[i][1];
            break;
        }
    }
    int num = solution.sequence.size();
    
    vector<int>::iterator iter;
    for(int i=0;i<per_times;i++)
    {
        int from = my_randint(0, num-1);
        int to = my_randint(0, num-2);
        int node = solution.sequence[from];
        solution.sequence.erase(solution.sequence.begin()+from);
        solution.sequence.insert(solution.sequence.begin()+to,node);
    }
    solution = fix_solution(solution,truck_capacity);
}

#include "time.h"
time_t start_time;
time_t dp_time;
time_t init_time;
time_t total_time;
#define TIME_LIMIT 45

Solution ils(Solution init_solution, float truck_capacity)
{
    get_obj(init_solution);
    Solution history_best_solution = init_solution;
    Solution base_solution = init_solution;
    time_t now_time;
    
    int ns_counter = 0;
    int ls_counter = 0;
    int no_improvement = 0;
    bool time_left = true;
    while(no_improvement< 10 && time_left)//iterative local search
    {
        ls_counter++;
        bool history_improved = false;
        while(1)//loop neighborhood search
        {
            ns_counter ++;
            time(&now_time);
            if(now_time -  start_time > TIME_LIMIT -2)
            {
                time_left = false;
                break;
            }
            pair<bool,Solution> search_ret = neighborhood_search(base_solution,truck_capacity);
            if(search_ret.first)
            {
                base_solution = search_ret.second;
                if(base_solution.value < history_best_solution.value)
                {
                    history_best_solution = base_solution;
                    history_improved = true;
                }
            }
            else
                break;
        }
        if(history_improved)
            no_improvement = 0;
        else
            no_improvement ++;
        pertubation(base_solution, no_improvement, truck_capacity);

    }
    cout<<"ns: "<<ns_counter<<"\tls: "<<ls_counter<<endl;
    return history_best_solution;
}

 std::vector<CourierSubpath> traveling_courier(
    const std::vector<DeliveryInfo> & deliveries,
    const std::vector<int> & depots,
    const float turn_penalty,
    const float truck_capacity)
{
     cout<<"delivery num : "<<deliveries.size()<<endl;
     //record when this funtion is called
    time(&start_time) ;
    
     
    //clear
    node_info.clear();
    all_inters.clear();
    travel_time_matrix.clear();
    task_vector.clear();
    
    //load pickup and dropoff to all_inters
    for(auto delivery_info : deliveries)
    {
        all_inters.push_back(delivery_info.pickUp);
        all_inters.push_back(delivery_info.dropOff);
    }
    
    //load depots to all_inters
    for(auto dpt : depots)
    {
        all_inters.push_back(dpt);
    }
    
    //create task vector contains all pickup, dropoff and depots
    for(int i =0;i<deliveries.size()*2;i++) //N delivery has 2N pickup and dropoff
    {
        vector<int> temp_vector;
        temp_vector.push_back(all_inters[i]);
        task_vector.push_back(temp_vector);
    }
    task_vector.push_back(depots);
    
    //start to assign tasks
    for(int i=0; i<task_vector.size();i++)
    {
        vector<vector<double>> travel_time_ret; //initialized and filled in get_travel_time
        get_travel_time(task_vector[i], turn_penalty, travel_time_ret);//get travel time between inters in task_vector[i] to all involved inters
        for(auto ret : travel_time_ret)
        {
            travel_time_matrix.push_back(ret); //put travel_time_ret into travel_time_matrix
        }
    }
    
   
    // analyze nearest_from and nearest_to depots of each nodes
    for(int i=0;i<deliveries.size()*2;i++)//loop all pickup and dropoff
    {
        int nearest_to  = -1;
        double to_time = BIG_ENOUGH;
        int nearest_from=-1;
        double from_time = BIG_ENOUGH;
        
        //record new smallest travel time and its position
        for(int j = deliveries.size()*2;j<all_inters.size();j++)
        {
            if(travel_time_matrix[i][j] < to_time)
            {
                to_time = travel_time_matrix[i][j];
                nearest_to = j;
            }
            if(travel_time_matrix[j][i]< from_time)
            {
                from_time = travel_time_matrix[j][i];
                nearest_from = j;
            }
        }
        Node_Info ni;
        ni.inter_idx = all_inters[i]; //inter_ind
        ni.delivery_idx = i / 2; //delivery_idx
        if(i%2 == 0 )
            ni.type = NI_PICKUP;//pickup
        else
            ni.type = NI_DROPOFF;//dropoff
        ni.weight = deliveries[ni.delivery_idx].itemWeight;//delivery weight
        ni.nearest_from = nearest_from - deliveries.size()*2;//index in depots
        ni.from_time = from_time;//travel time from nearest_from to this node
        ni.nearest_to = nearest_to - deliveries.size()*2;//index in depot
        ni.to_time = to_time;//travel time from this node to nearest_to
        node_info.push_back(ni);
    }
    time(&dp_time);
    cout<<"total took "<<dp_time - start_time<<"\tdata prepararion takes "<<dp_time - start_time<<endl;
            
    //Finish data preparation and start algorithm
    Solution solution;//define a solution
    init_solution(deliveries.size()*2, truck_capacity, solution);//get an initial solution (how many nodes need to be sorted, truck_capacity, solution to be filled)
    time(&init_time);
    cout<<"total took "<<init_time - start_time<<"\tinit_solution takes "<<init_time-dp_time<<endl;
    
    solution = ils(solution,truck_capacity);
//    output_detail(solution,turn_penalty,truck_capacity,depots,deliveries);
    vector<CourierSubpath> ret = convert_solution(solution, turn_penalty, depots);//covert solution into required format
    time(&total_time);
    
    cout<<"total took "<<total_time - start_time<<"\topt_solution takes "<<total_time-init_time<<endl;
    cout<<"legality : "<<check_legality(solution, truck_capacity)<<endl;
    get_obj(solution);
    cout<<"obj is "<<solution.value<<endl;
    return ret;
}   
