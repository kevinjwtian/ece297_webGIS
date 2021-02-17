/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <iostream>
#include <map>
#include <vector>
#include <set>
#include <algorithm>
#define AVG_SPEED 3000000       //set AVG_SPEED big to ensure optimality
#include "m3.h"
#include "m1.h"
#define unreached 0
#define open 1
#define closed 2
#define updated 3

struct Inter_Info //information of an intersection
{
    double f;       //f value of a start algorithm in path find. walk_time in walk with pick up 
    double g;       //g value of a star algorithm in path find. flag value of if it is a edge intersection, 1 means yes, otherwise no
    double h;
    int inter_idx;  //which current intersection
    int segment_idx; //which segment the path reaches the current intersection through
    int street_idx; //the street that the segment belongs to 
    int from_idx;  //from_idx --> inter_idx through segment_idx
    bool operator < (const Inter_Info& other)&  //compare two inter_info sizes only through f value
    {
        return f < other.f;
    }
};

struct Status_Index //an intersection's status and position where its information is stored
{
    int status; //status of the intersection, 0 unreached, 1 open, 2 closed
    int pos;  //position of the intersection information in information vector
};

struct Inter_F //use to find intersection with the smallest f value, connect f and inter_idx
{
    int inter_idx;
    double f;
};

std::vector<Inter_Info> inter_info_vector;          // container for all open or closed inters, store open or closed intersection information

//pop the element with the smallest f value
// a min heap, arranged by f value. 
// there may be some duplicated elements in it. 
// for example, an intersection is open and pushed into min_heap.
// before it is poped, its f value is updated, so it is pushed into min heap again.
// in such case, smaller f value is pop out before the bigger one, and it is set to be closed.
// so each element popped out from min heap, needs status check, if it has been closed, then continue.
std::vector<Inter_F> f_min_heap;                    

//each intersection has its own status and position 
// a vector of each intersection's status and its storage position in inter_info_vector
// the vector size equals to number of total intersections. 
// status_index_vector[i] is corresponding intersection i. .status is intersection's status, (0 - unreached, 1 - open, 2 - closed)
// .pos is the storage position of the intersection in inter_info_vector
// inter_info_vector [status_index_vector[i].pos] is the information about i
std::vector<Status_Index> status_index_vector;

//function declarations, since m3.h is read only.
int try_segment(Inter_Info &from_inter_info, int segment_idx, double turn_penalty);
int walk_try_segment(Inter_Info &from_inter_info, int segment_idx, double turn_penalty, const double walking_speed, const double walking_time_limit);
double travel_time_between_intersections(int from, int to, double turn_penalty);
double get_manhatton_distance(int inter_1, int inter_2);
bool cmp_Inter_F(const Inter_F& elm_1, const Inter_F& elm_2);
//declare a global variable
int global_dest_inter;

// min heap comparison function, makes the heap be a min heap
bool cmp_Inter_F(const Inter_F& elm_1, const Inter_F& elm_2) 
{
    return elm_1.f > elm_2.f;
}

double get_manhatton_distance(int inter_1, int inter_2) // manhatton distance (H) = |lat1-lat2| + |lon1-lon2|
{
    LatLon ll_inter_1 = getIntersectionPosition(inter_1);//lat and lon of inter_1
    LatLon ll_inter_2 = getIntersectionPosition(inter_2);//lat and lon of inter_2
    double manhatton_distance = 0;
    LatLon ll_mid_point(ll_inter_1.lat(), ll_inter_2.lon());// lat1 lon2
    manhatton_distance += find_distance_between_two_points(std::make_pair(ll_inter_1, ll_mid_point));//lat1 lon1 to lat1 lon2
    manhatton_distance += find_distance_between_two_points(std::make_pair(ll_inter_2, ll_mid_point));//lat1 lon2 to lat2 lon2
    return manhatton_distance;
}

int try_segment(Inter_Info &from_inter_info, int segment_idx, double turn_penalty)
{
    int to_inter; //which intersection is connected to from_inter through the segment
    InfoStreetSegment seg_info = getInfoStreetSegment(segment_idx);//get info of the segment_idx (connects to from_inter)
    if (seg_info.from == from_inter_info.inter_idx)//if segment from is the given node(from_inter), then segment to is to_inter
        to_inter = seg_info.to;
    else if (seg_info.oneWay)//the segment is oneway only so cannot be used
        return open;   // segment it unpassable from from_inter_info through this segment
    else
        to_inter = seg_info.from;//segment from is to _inter

    //if to_inter is in close_inter
    if (status_index_vector[to_inter].status == closed) //if to_inter is closed, cannot be used, return
        return closed;   // the end of the segment is closed.
        
    //to_inter is open or never reached, calculate information of to_inter
    double new_g = from_inter_info.g + find_street_segment_travel_time(segment_idx);//calculate new g
    if (from_inter_info.street_idx != seg_info.streetID)  // needs to add turn penalty if are different streets
        new_g += turn_penalty;
    double new_h = get_manhatton_distance(from_inter_info.inter_idx, to_inter) / AVG_SPEED; //calculate new h 
    Inter_Info to_inter_info;//new struct  to_inter_info 
    to_inter_info.inter_idx = to_inter;
    to_inter_info.segment_idx = segment_idx;
    to_inter_info.street_idx = seg_info.streetID;
    to_inter_info.from_idx = from_inter_info.inter_idx;
    to_inter_info.f = new_g + new_h;
    to_inter_info.g = new_g;
    to_inter_info.h = new_h;
    
    
    //if to_inter is in open_inters, compare to_inter_info.f with the inter_info in map
    if (status_index_vector[to_inter].status == open)//to_inter belongs to open-inters
    {
        // if to_inter exists in open_inters, compare
        int pos = status_index_vector[to_inter].pos;
        
        //compare new f value to previous f value of to_inters
        if (to_inter_info.f < inter_info_vector[pos].f)    // means better (smaller) f of to_inter is found second:pointer (address in open_inter_info_set)
        {
            //update to_inter_info
            inter_info_vector[pos] = to_inter_info;
            
            //push inter_F into f_min_heap
            f_min_heap.push_back(Inter_F{to_inter, to_inter_info.f});
            std::push_heap(f_min_heap.begin(),f_min_heap.end(),cmp_Inter_F);
        }
    }
    else //to_inter is not in open
    {
         //add to_inter_info into inter_info_vector
        inter_info_vector.push_back(to_inter_info);
        
        // add inter_info into status_index_vector, sets it status open and record its position in inter_info_vector
        status_index_vector[to_inter].status = open;
        status_index_vector[to_inter].pos = inter_info_vector.size() - 1;
        
        //push inter_F into f_min_heap
        f_min_heap.push_back(Inter_F{to_inter, to_inter_info.f});
        std::push_heap(f_min_heap.begin(),f_min_heap.end(),cmp_Inter_F);
       
    }
    return updated;       //means the intersection is open or updated
}

std::vector<int> find_path_between_intersections(int start_inter, int dest_inter, const double turn_penalty)
{
    if(start_inter == dest_inter)
    {
        std::vector<int> ret;
        return ret;
    }
    global_dest_inter = dest_inter;//turn destination into global variable

    inter_info_vector.clear();//clear containers at first
    f_min_heap.clear();
    status_index_vector.clear();
    int inter_count = getNumIntersections();//get the number of intersections
    
    //default element in status_index_vector, which means intersection status is unreached and intersection position = -1
    Status_Index default_sd{ 0,-1 };    
    
    status_index_vector.resize(inter_count, default_sd);//initialize status_index_vector with default_sd
    
    //initial start works
    double manhatton_distance_time = get_manhatton_distance(start_inter, dest_inter) / AVG_SPEED; //time = distance/speed
    Inter_Info start_inter_info{ manhatton_distance_time,       // F total distance = real + estimate
        0,                                                                      // G real distance
        manhatton_distance_time,                          // H estimate by manhatton
        start_inter,                                                             // current intersection                                                                      
        -1,                                                                     // by which segment reaching current
        -1,                                                                     // id of street which segment belongs to
        -1};                                                                    // from which inters come to current
                                                                                // -1 means this is the start
    //push inter_info into inter_info_vector
    inter_info_vector.push_back(start_inter_info);
    
    //get start_inter_info position in inter_info_vector;
    int data_position = inter_info_vector.size() - 1;
    
    //set status and index in status_index_vector
    status_index_vector[start_inter].status = open;    // set status open
    status_index_vector[start_inter].pos = data_position;    // set its index 
    
    //push start_inter into f_min_heap (vector), and turns f_min_heap to a heap
    f_min_heap.push_back(Inter_F{ start_inter,start_inter_info.f });
    std::make_heap(f_min_heap.begin(), f_min_heap.end(), cmp_Inter_F); //which vectors want to be turned into heap,, comparison function
    
    //start AStart algorithm, loop until dest is reached or no more intersections in open_inters
    bool path_found = false;//whether find a path or not 
    while (1)//start the loop 
    {
        // find the intersection with smallest f among all open inters
        Inter_F inter_f;
        bool found_open = false;//the flag of whether an open intersection is found
        while (1)//loop until an open is poped from f_min_heap or f_min_heap is empty
        {
            if (f_min_heap.size() == 0) //if empty, then break
                break;
            
            //take out the element with the smallest f value 
            inter_f = f_min_heap.front();
            std::pop_heap(f_min_heap.begin(), f_min_heap.end(),cmp_Inter_F);
            f_min_heap.pop_back();
            
            if (status_index_vector[inter_f.inter_idx].status == open) //the found intersection is open
            {
                found_open = true;          //set found flag true
                status_index_vector[inter_f.inter_idx].status = closed;  //set its status closed
                break;
            }
        }
        if (!found_open)    //if no open inter is found
        {
            path_found = false; //cannot find a path to destination
            break;//break from the first while(1)
        }
        int pos = status_index_vector[inter_f.inter_idx].pos;//find the position of the found open intersection
        Inter_Info smallest_inter_info = inter_info_vector[pos];//retrieve the information of the found open intersection
        int from_inter = smallest_inter_info.inter_idx;            //let from_inter to be the smallest found open intersection

        if (from_inter == dest_inter) //the total shortest path is found
        {
            path_found = true;
            break;
        }
        // spread from the smallest_inter
        //put all neighbour segements of from_inter into segment_idx_vector
        std::vector <int> segment_idx_vector = find_street_segments_of_intersection(from_inter);
        //loop all segments from segment_idx_vector
        for (auto segment_idx : segment_idx_vector)
        {
            try_segment(smallest_inter_info, segment_idx, turn_penalty);//try_segment connected to from_inter based on these values
        }
    }
    std::vector<int> segment_result;
    
    if(!path_found)
    {
        return segment_result;//return empty result
    }    
    
    int pos = -1;
    int to_inter = dest_inter;//start from dest_inter first 
    while (1)
    {
/*        if(status_index_vector[to_inter].status != closed)//if to_inter status is not closed, something is wrong 
        {
            break;
        }
*/        pos = status_index_vector[to_inter].pos;//find to_inter position 
        Inter_Info to_inter_info = inter_info_vector[pos];
        
        segment_result.push_back(to_inter_info.segment_idx);
        to_inter = to_inter_info.from_idx;//trackback again, turn from_idx into to_inter
        if (to_inter == start_inter)//reach the start_inter, stop tracking back 
            break;
    }
    // reverse the result
    std::reverse(segment_result.begin(),segment_result.end());//because is back tracked, so need to reverse the order 
    return segment_result;
}

double compute_path_travel_time(const std::vector<StreetSegmentIndex>& path, const double turn_penalty)
{
    if(path.size()==0)//empty path
        return 0;
    
    int curr_streetID = getInfoStreetSegment(path[0]).streetID;//turn current_streetID to the first street ID
    double travel_time = 0;
    for(auto segment_idx : path)//loop all segment_idx in path
    {
        int next_streetID = getInfoStreetSegment(segment_idx).streetID;//next streetID
        if(curr_streetID != next_streetID)//if current streetID is different from next streetID, add turn penalty
        {
            travel_time += turn_penalty;
            curr_streetID = next_streetID; //update current streetID to next streetID
        }
        travel_time += find_street_segment_travel_time(segment_idx);//add travel time of current segment to the total travel time
    }
    return travel_time;
}

double compute_path_walking_time(const std::vector<StreetSegmentIndex>& path,
        const double walking_speed,
        const double turn_penalty)
{
     if(path.size()==0)
        return 0;
    
    int curr_streetID = getInfoStreetSegment(path[0]).streetID;
    double travel_time = 0;
    for(auto segment_idx : path)
    {
        int next_streetID = getInfoStreetSegment(segment_idx).streetID;
        if(curr_streetID != next_streetID)
        {
            travel_time += turn_penalty;
            curr_streetID = next_streetID;
        }
        travel_time += find_street_segment_length(segment_idx)/walking_speed;//time=distance/speed
    }
    return travel_time;   
}


#define VERY_BIG 10000000000000

// very similar to inter_info_vector, f_min_heap, status_index_vector
std::vector<Inter_Info>     walk_inter_info_vector;
std::vector<Inter_F>        walk_f_min_heap;
std::vector<Status_Index>   walk_status_index_vector;


struct Inter_MD
{
    int inter_idx;
    double manhatton_distance;
    bool operator< (const Inter_MD& other)const
    {
        return manhatton_distance< other.manhatton_distance;
    }
};

double travel_time_between_intersections(int from, int to, double turn_penalty)
{
    std::vector<int>path = find_path_between_intersections(from , to, turn_penalty);
    return compute_path_travel_time(path, turn_penalty);
}
// .first = walking path; .second = driving path
std::pair<std::vector<int>, std::vector<int>> 
find_path_with_walk_to_pick_up(
    const int start_intersection,
    const int end_intersection,
    const double turn_penalty,
    const double walking_speed,
    const double walking_time_limit)
{
    walk_inter_info_vector.clear();
    walk_f_min_heap.clear();
    walk_status_index_vector.clear();
    
    int inter_count = getNumIntersections();
    Status_Index default_sd{ 0,-1 };    //default element in status_index_vecotr, which means status unreached and index = -1
    walk_status_index_vector.resize(inter_count, default_sd);
    
    //initial start works
    Inter_Info start_inter_info{ -turn_penalty,//walk_time took by walking from start inter to current - turn penalty due to -1 ( id of street which segment belongs to) [no matter which direction it goes, start point -1 always has a turn penalty]       
        -1,                                           //flag of if it is a edge inter
        -1,                          // not used in walk pick up
        start_intersection,                                                             // current intersection                                                                      
        -1,                                                                     // by which segment reaching current
        -1,                                                                     // id of street which segment belongs to
        -1};                                                                    // from which inters come to current
                                                                                // -1 means this is the start
    //push inter_info in walk_inter_info_vector
    walk_inter_info_vector.push_back(start_inter_info);
    //get its position in walk_inter_info_vector;
    int data_position = walk_inter_info_vector.size() - 1;
    //set status and index in walk_status_index_vector
    walk_status_index_vector[start_intersection].status = open;    // set status open
    walk_status_index_vector[start_intersection].pos = data_position;    // set its position
    //push it into min heap
    walk_f_min_heap.push_back(Inter_F{ start_intersection,start_inter_info.f });
    std::make_heap(walk_f_min_heap.begin(), walk_f_min_heap.end(), cmp_Inter_F);
    //start AStart algorithm, loop until dest is reached or no more intersections in open_inters

    while (1)//start the loop, stop all intersection within walk_time_limit are reached
    {
        // find the intersection with smallest f among all open inters
        Inter_F inter_f;
        bool found_open = false;
        while (1)
        {
            if (walk_f_min_heap.size() == 0) //if empty
                break;
            //get the top of walk_f_min_heap, AKA the smallest.
            inter_f = walk_f_min_heap.front();
            std::pop_heap(walk_f_min_heap.begin(), walk_f_min_heap.end(),cmp_Inter_F);
            walk_f_min_heap.pop_back();
            
            // if its status is open, then OK and go ahead. else, pop next from walk_f_min_heap
            if (walk_status_index_vector[inter_f.inter_idx].status == open) //its status is open, then the open is found
            {
                found_open = true;          //set found flag true
                walk_status_index_vector[inter_f.inter_idx].status = closed;  //set its status closed
                break;
            }
            else if (walk_status_index_vector[inter_f.inter_idx].status == closed) //its status is close, then try next
            {
                continue;
            }
        }
        if (!found_open)    //if not open inter is found
        {
            break;
        }
        // access the information of the found intersection
        int pos = walk_status_index_vector[inter_f.inter_idx].pos;      // get the storage position
        Inter_Info smallest_inter_info = walk_inter_info_vector[pos];   //the result is a struct because this set is sorted according to f value, so the begin one has the smallest f value
        int from_inter = smallest_inter_info.inter_idx;                 //let from_inter be the smallest inter red it from above struct
        // spread from the smallest_inter
        // put all neighbour segements of from_inter into segment_idx_vector
        std::vector <int> segment_idx_vector = find_street_segments_of_intersection(from_inter);

         //loop all segments from segment_idx_vector
        for (auto segment_idx : segment_idx_vector)
        {
            // try segment_idx.
            int res = walk_try_segment(smallest_inter_info, segment_idx, turn_penalty, walking_speed, walking_time_limit);//try_segment connected to from_inter based on these values
            //res = 1, means from current intersection, going through segment_idx, reached an intersection beyond walk_time_limit
            // so, the current intersection is an edge intersection, set the flag variable 1
            if(res == 1)        
                walk_inter_info_vector[pos].g = 1;      //means it is an edge intersection
        }
    }

    // pick up all edge intersections from walk_inter_info_vector
    std::vector<Inter_MD> inter_md_vec;
    for(auto wii : walk_inter_info_vector)//loop all elements of walk_inter_info_vector
    {
        if(wii.g == 1)//it is a edge inter
        {
            //put the edge inter into inter_md_vec
            int inter_idx = wii.inter_idx;
            double md = get_manhatton_distance(inter_idx, end_intersection);
            inter_md_vec.push_back(Inter_MD{inter_idx,md});
        }
        
    }
    std::sort(inter_md_vec.begin(),inter_md_vec.end());//sort inter_md_vec according to manhattan distance
    double shortest_travel_time = VERY_BIG;
    int mid_inter = -1;//pickup intersection
    for(auto inter_md : inter_md_vec)//loop all intersections
    {
        //record travel time from pickup inter to destination
        double travel_time = travel_time_between_intersections(inter_md.inter_idx, end_intersection, turn_penalty);
        if(travel_time < shortest_travel_time)
        {
            // in two cases, travel_time equals to 0.
            // first one, the midway inter and the end inter are the same.
            // second, there is no any legal path between midway and end intersections, so path finding algorithm return an empty vector and thus travel_time is 0
            // so, we need to tell which case is happening. if the second, then next.
            if(travel_time == 0)
            {
                if(inter_md.inter_idx != end_intersection)//second case
                    continue;
                else
                {
                    //first case
                    shortest_travel_time = travel_time;
                    mid_inter = inter_md.inter_idx;
                    break;
                }
            }
            //travel time doesn't equal to 0
            shortest_travel_time = travel_time;
            mid_inter = inter_md.inter_idx;
        }
    }
    //trackback from midinter
    std::vector<int> walk_path;
    int pos = -1;
    int to_inter = mid_inter;//start from dest_inter first 
    while (1)
    {
        if (to_inter == start_intersection)//reach the start_inter, stop tracking back 
            break;
        pos = walk_status_index_vector[to_inter].pos;//find to_inter from close_inters and put the result into map_iter
        Inter_Info to_inter_info = walk_inter_info_vector[pos];
        //can find from close_inters
        walk_path.push_back(to_inter_info.segment_idx);//put second: inter_info 's segment_idx into result 
        to_inter = to_inter_info.from_idx;//trackback again, turn from_idx into to_inter
    }
    // reverse the result
    std::reverse(walk_path.begin(),walk_path.end());//because is back tracked, so need to reverse the order 
    std::vector<int> drive_path = find_path_between_intersections(mid_inter, end_intersection, turn_penalty);
    return std::make_pair(walk_path,drive_path);
}


int walk_try_segment(Inter_Info &from_inter_info, int segment_idx, double turn_penalty, const double walking_speed, const double walking_time_limit)
{
    int to_inter; //which intersection is connected to from_inter through the segment
    InfoStreetSegment seg_info = getInfoStreetSegment(segment_idx);//get info of the segment_idx (connects to from_inter)
    if (seg_info.from == from_inter_info.inter_idx)//if segment from is the given node(from_inter), then segment to is to_inter
        to_inter = seg_info.to;
    else
        to_inter = seg_info.from;//segment from is to _inter

    //if to_inter is in close_inter
    if (walk_status_index_vector[to_inter].status == closed) //if to_inter is closed, cannot be used, return
        return closed;   // the end of the segment is closed.
        
    //to_inter is open or never reached 
    double new_f = from_inter_info.f + find_street_segment_length(segment_idx) / walking_speed;//calculate new walk time
    if (from_inter_info.street_idx != seg_info.streetID)  // needs to add turn penalty if are different streets
        new_f += turn_penalty;

    // if walking time walking from start to to_inter beyond walk_time_limit, return 1.
    if(new_f > walking_time_limit)
        return 1;                   //means this intersection is beyond walking time limit

    // construct to_inter_info
    Inter_Info to_inter_info;//new struct  to_inter_info 
    to_inter_info.inter_idx = to_inter;
    to_inter_info.segment_idx = segment_idx;
    to_inter_info.street_idx = seg_info.streetID;
    to_inter_info.from_idx = from_inter_info.inter_idx;
    to_inter_info.g = -1;
    to_inter_info.h = -1;
    to_inter_info.f = new_f;
    
    //if to_inter is in open_inters, compare to_inter_info.f with the inter_info in map
    if (walk_status_index_vector[to_inter].status == open)//to_inter belongs to open-inters
    {
        // if to_inter exists in open_inters, compare
        int pos = walk_status_index_vector[to_inter].pos;
        if (to_inter_info.f < walk_inter_info_vector[pos].f)    // means better (smaller) f of to_inter is found second:pointer (address in open_inter_info_set)
        {
            walk_inter_info_vector[pos] = to_inter_info;
            walk_f_min_heap.push_back(Inter_F{to_inter, to_inter_info.f});
            std::push_heap(walk_f_min_heap.begin(),walk_f_min_heap.end(),cmp_Inter_F);
        }
    }
    else //to_inter is not in open
    {
         //add to_inter_info into open_inters
        walk_inter_info_vector.push_back(to_inter_info);
        
        // add inter_info into map record the address of to_inter_info in open_inter_info_set
        walk_status_index_vector[to_inter].status = open;
        walk_status_index_vector[to_inter].pos = walk_inter_info_vector.size() - 1;
        
        walk_f_min_heap.push_back(Inter_F{to_inter, to_inter_info.f});
        std::push_heap(walk_f_min_heap.begin(),walk_f_min_heap.end(),cmp_Inter_F);
    }
    return updated;       //means the intersection is open or updated
}
