/*
 * Copyright 2020 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated
 * documentation files (the "Software") in course work at the University
 * of Toronto, or for personal use. Other uses are prohibited, in
 * particular the distribution of the Software either publicly or to third
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include <cmath>
#include "OSMDatabaseAPI.h"
#include "LatLon.h"
#include "methods.h"
#include <array>
#include <string>
#include <algorithm>
#include <map>
#include <unordered_map>
//to store street_name and its index
struct st_name_idx
{
	std::string street_name;
	int idx;
};
bool compare_st_name_idx(const st_name_idx& elem1, const st_name_idx& elem2);

//turn upper letters into lower
void gstringU2L(std::string& str)
{
	for (int i = 0; i < str.length(); i++)
	{
		if (str[i] >= 'A' && str[i] <= 'Z')
			str[i] += 32;
	}
}

//erase spaces in the given string
void gtrim(std::string& str)
{
	if (str.empty())
		return;
	while (true)
	{
		std::string::size_type iter = str.find(" ");
		if (iter == str.npos)
			break;

		str.erase(iter, 1);
	}
}


double cal_street_segment_length(int street_segment_id);
//to calculate street segment length of the given segment idx
double cal_street_segment_length(int street_segment_id) {
	//to store points of the given segment
	std::vector<LatLon> seg_points;
	//to get points of the segment, including curve point, if there are any
	InfoStreetSegment seg_info = getInfoStreetSegment(street_segment_id);
	LatLon seg_from = getIntersectionPosition(seg_info.from);
	LatLon seg_to = getIntersectionPosition(seg_info.to);
	seg_points.push_back(seg_from);
	for (int i = 0; i < seg_info.curvePointCount; i++)
	{
		LatLon temp = getStreetSegmentCurvePoint(i, street_segment_id);
		seg_points.push_back(temp);
	}
	seg_points.push_back(seg_to);//no curve points
	//calculate the distance
	double dist = 0;
	for (int i = 0; i < seg_points.size() - 1; i++)
	{
		std::pair<LatLon, LatLon> temp_pair(seg_points[i], seg_points[i + 1]);
		dist += find_distance_between_two_points(temp_pair);
	}
	return dist;
}
//the comparion function for the vector sort
bool compare_st_name_idx(const st_name_idx& elem1, const st_name_idx& elem2)
{
	return elem1.street_name < elem2.street_name;
}
//the flag of load_map status
bool bLoadMap = false;
//to store intersections' segments
std::vector<std::vector<int>> inter_seg_vec;
//to store streets' segments
std::vector<std::vector<int>> street_seg_vec;
//to store streets' intersections
std::vector<std::vector<int>> street_inter_vec;
//to store streets' trimmed and lowered names
std::vector<st_name_idx> street_name_vec;
//to store segments' length
std::vector<double> segment_length_vec;
//to store segments' travel time
std::vector<double> segment_travel_time_vec;
//maps between node/way and index
std::unordered_map<OSMID, int> node_id_int_map;
std::unordered_map<OSMID, int> way_id_int_map;


bool load_map(std::string map_path)
{
	if (bLoadMap)
	{
		//if a map has been loaded, close it.
		close_map();
		bLoadMap = false;
	}
	//load streetsdatabaseBIN
	bool loaded = loadStreetsDatabaseBIN(map_path);
	if (!loaded)
		return loaded;
	//load OSMdatabaseBIN
	map_path.replace(map_path.find("streets"), 7, "osm");
	loaded = loadOSMDatabaseBIN(map_path);

	if (loaded) {
		//load segments of intersections
		int inter_count = getNumIntersections();
		inter_seg_vec.resize(inter_count);
		for (int i = 0; i < inter_count; i++)
		{
			int inter_seg_count = getIntersectionStreetSegmentCount(i);
			for (int j = 0; j < inter_seg_count; j++)
			{
				inter_seg_vec[i].push_back(getIntersectionStreetSegment(i, j));
			}
		}
		//load segment length
		for (int i = 0; i < getNumStreetSegments(); i++)
		{
			double seg_length = cal_street_segment_length(i);
			double speedLimit = getInfoStreetSegment(i).speedLimit;  // km/h
			//length is in meters, so turn km/h into m/s
			speedLimit = speedLimit * 1000 / 3600;
			segment_length_vec.push_back(seg_length);
			segment_travel_time_vec.push_back(seg_length / speedLimit);
		}
		//load segments of streets
		int street_count = getNumStreets();
		street_seg_vec.resize(street_count);
		for (int i = 0; i < getNumStreetSegments(); i++)
		{
			int streetID = getInfoStreetSegment(i).streetID;
			street_seg_vec[streetID].push_back(i);
		}
		//load intersections of streets
		street_inter_vec.resize(street_count);
		for (int i = 0; i < street_count; i++)
		{
			street_inter_vec[i] = find_intersections_of_street(i);
		}
		//load and preprocess street name
		street_name_vec.reserve(street_count);//memory
		for (int i = 0; i < street_count; i++)
		{
			std::string street_name = getStreetName(i);
			gtrim(street_name);
			gstringU2L(street_name);
			street_name_vec.push_back(st_name_idx{ street_name , i });
		}
		//sort the street name vector for convenient find
		std::sort(street_name_vec.begin(), street_name_vec.end(), compare_st_name_idx);

		//load node_id_int_map and way_id_int_map
		int num_nodes = getNumberOfNodes();
		for (int i = 0; i < num_nodes; i++)
		{
			OSMID node_id = getNodeByIndex(i)->id();
			node_id_int_map.insert(std::pair<OSMID, int>(node_id, i));
		}
		int num_ways = getNumberOfWays();
		for (int i = 0; i < num_ways; i++)
		{
			OSMID way_id = getWayByIndex(i)->id();
			way_id_int_map.insert(std::pair<OSMID, int>(way_id, i));
		}
               
		bLoadMap = true;
	}
	else {
		return loaded;
	}
	bLoadMap = true;
	return true;
}

void close_map()
{
	//release memory of containers
	std::vector<std::vector<int>>().swap(inter_seg_vec);
	std::vector<std::vector<int>>().swap(street_seg_vec);
	std::vector<std::vector<int>>().swap(street_inter_vec);
	std::vector<st_name_idx>().swap(street_name_vec);
	std::vector<double>().swap(segment_length_vec);
	std::vector<double>().swap(segment_travel_time_vec);
	node_id_int_map.clear();
	way_id_int_map.clear();
	//close database
	closeStreetDatabase();
	closeOSMDatabase();
	//set status flag
	bLoadMap = false;
}

//find distance between two points
double find_distance_between_two_points(std::pair<LatLon, LatLon> points)
{
	double lat_1 = points.first.lat();
	double lon_1 = points.first.lon();
	double lat_2 = points.second.lat();
	double lon_2 = points.second.lon();

	double lat_avg = (lat_1 + lat_2) * 0.5;

	double x1 = lon_1 * cos(DEGREE_TO_RADIAN * lat_avg);
	double y1 = lat_1;
	double x2 = lon_2 * cos(DEGREE_TO_RADIAN * lat_avg);
	double y2 = lat_2;

	double dist = EARTH_RADIUS_METERS * DEGREE_TO_RADIAN * sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
	return dist;
}
// find street segment length
double find_street_segment_length(int street_segment_id) {
	return segment_length_vec[street_segment_id];
}

// find segment travel time
double find_street_segment_travel_time(int street_segment_id) {
	return segment_travel_time_vec[street_segment_id];
}

// find the closest intersection of the given position
int find_closest_intersection(LatLon my_position) {
	// initial finding parameters
	int nearest_intersection = 0;
	int temp = 0;
	LatLon temp_pos = getIntersectionPosition(temp);
	std::pair<LatLon, LatLon> temp_pair(my_position, temp_pos);
	double temp_dist = find_distance_between_two_points(temp_pair);
	double nearest_dist = temp_dist;
	// loop all intersections
	for (int i = 1; i < getNumIntersections(); i++)
	{
		temp = i;
		temp_pos = getIntersectionPosition(temp);
		temp_pair = std::make_pair(my_position, temp_pos);
		temp_dist = find_distance_between_two_points(temp_pair);
		if (temp_dist < nearest_dist)
		{
			// if a closer intersection is found
			nearest_dist = temp_dist;
			nearest_intersection = temp;
		}
	}
	return nearest_intersection;
}

// find street segments of the given intersection
std::vector<int> find_street_segments_of_intersection(int intersection_id) {
	return inter_seg_vec[intersection_id];
}

// find all street names of the given intersection
std::vector<std::string> find_street_names_of_intersection(int intersection_id) {
	//the container of the result
	std::vector<std::string> result_vec;
	//loop all segments of the intersection, and find the corresponding street name
	std::vector<int> seg_vec = find_street_segments_of_intersection(intersection_id);
	for (int i = 0; i < seg_vec.size(); i++)
	{
		StreetSegmentIndex seg_id = seg_vec[i];
		StreetIndex st_id = getInfoStreetSegment(seg_id).streetID;
		std::string st_name = getStreetName(st_id);
		result_vec.push_back(st_name);
	}
	return result_vec;
}

// tell whether the intersections are connected
bool are_directly_connected(std::pair<int, int> intersection_ids)
{
	IntersectionIndex inter_1 = intersection_ids.first;
	IntersectionIndex inter_2 = intersection_ids.second;
	// if they are the same, return true
	if (inter_1 == inter_2)
		return true;
	// loop all adjacent intersection of the first, check if the second in
	std::vector <StreetSegmentIndex> seg_vec = find_adjacent_intersections(inter_1);
	for (int i = 0; i < seg_vec.size(); i++)
	{
		if (inter_2 == seg_vec[i])
			return true;
	}
	return false;
}

// find all adjacent intersection of the given intersection
std::vector<int> find_adjacent_intersections(int intersection_id) {
	//the container of the result
	std::vector<int> result_vec;
	//loop all the segments of the intersection
	std::vector <StreetSegmentIndex> seg_vec = find_street_segments_of_intersection(intersection_id);
	for (int i = 0; i < seg_vec.size(); i++)
	{
		InfoStreetSegment seg_info = getInfoStreetSegment(seg_vec[i]);
		if (intersection_id == seg_info.from)
			result_vec.push_back(seg_info.to);
		if (intersection_id == seg_info.to && !seg_info.oneWay)
			result_vec.push_back(seg_info.from);
	}
	//erase duplicated elements
	gunique(result_vec);
	return result_vec;
}

// find all segments of the given street
std::vector<int> find_street_segments_of_street(int street_id) {
	return street_seg_vec[street_id];
}

// find all intersections of the given street
std::vector<int> find_intersections_of_street(int street_id) {
	// the container of the result
	std::vector<int> result_vec;
	// loop all segments of the given street, push FROM and TO into result container
	std::vector<int> seg_of_st_vec = find_street_segments_of_street(street_id);
	for (int i = 0; i < seg_of_st_vec.size(); i++)
	{
		StreetSegmentIndex seg_id = seg_of_st_vec[i];
		IntersectionIndex inter_from = getInfoStreetSegment(seg_id).from;
		IntersectionIndex inter_to = getInfoStreetSegment(seg_id).to;
		result_vec.push_back(inter_from);
		result_vec.push_back(inter_to);
	}
	// make result unique
	gunique(result_vec);
	return result_vec;
}

// find the common intersections of two streets
std::vector<int> find_intersections_of_two_streets(std::pair<int, int> street_ids) {
	// the container of the result
	std::vector<int> result_vec;
	// get two vectors of intersections of the streets
	std::vector<int> inter_of_st1 = street_inter_vec[street_ids.first];
	std::vector<int> inter_of_st2 = street_inter_vec[street_ids.second];
	// find common ones
	for (int inter_1 : inter_of_st1)
	{
		for (int inter_2 : inter_of_st2)
		{
			if (inter_1 == inter_2)
				result_vec.push_back(inter_1);
		}
	}
	return result_vec;
}

// find street ids from partial street name
std::vector<int> find_street_ids_from_partial_street_name(std::string street_prefix) {
	//the result container
	std::vector<int> result_vec;
	// trim and upper to lower the given prefix
	gtrim(street_prefix);
	gstringU2L(street_prefix);

	// return an empty vector when the prefix length is 0
	if (street_prefix.length() == 0)
		return result_vec;

	// loop all street names.
	// names are stored in the sequence from small to big
	for (int i = 0; i < street_name_vec.size(); i++) {
		// if the street name's initial letter is smaller than the one of the prefix, continue
		if (street_name_vec[i].street_name[0] < street_prefix[0])
			continue;
		// if the street name's initial letter is larger than the one of the prefix,
		// means no more names to find. so break
		if (street_name_vec[i].street_name[0] > street_prefix[0])
		{
			break;
		}
		// if street name is shorter than the prefix, continue
		if (street_name_vec[i].street_name.length() < street_prefix.length())
			continue;
		// compare street names and the prefix, if match, then push back
		if (street_name_vec[i].street_name.compare(0, street_prefix.length(), street_prefix) == 0)
			result_vec.push_back(street_name_vec[i].idx);
	}
	return result_vec;
}
// find feature area
double find_feature_area(int feature_id) {
	const int numFeaturePoints = getFeaturePointCount(feature_id);
	int firstPoint = 0;
	int lastPoint = numFeaturePoints - 1;
	double area = 0.0; // in case of open area

	//Lats and Lons need to be in radian
	std::vector<double> lats;
	std::vector<double> lons;
	double sumLat = 0;
	double avgLat = 0;

	//Reserve spaces
	lats.reserve(numFeaturePoints);
	lons.reserve(numFeaturePoints);

	//Load 
	for (int i = 0; i < numFeaturePoints; i++) {
		lats.push_back(getFeaturePoint(i, feature_id).lat()); //in Deg
		lons.push_back(getFeaturePoint(i, feature_id).lon());
		sumLat += getFeaturePoint(i, feature_id).lat(); //in Deg
	}
	if ((lats[firstPoint] == lats[lastPoint]) && (lons[firstPoint] == lons[lastPoint])) {//is Closed 
		//X and Ys
		double Xi, Yi, Xj, Yj;
		avgLat = sumLat / numFeaturePoints; //in Deg  
		// Calculate value of shoelace formula;  Y is lat

		//int j = numFeaturePoints-2;
		int j;
		for (int i = 0; i < numFeaturePoints - 1; i++) {
			//area += (X[j] + X[i]) * (Y[j] - Y[i]);
			j = i + 1; //J is next to i
			Xi = DEGREE_TO_RADIAN * lons[i] * std::cos(avgLat * DEGREE_TO_RADIAN);
			Xj = DEGREE_TO_RADIAN * lons[j] * std::cos(avgLat * DEGREE_TO_RADIAN);
			Yi = DEGREE_TO_RADIAN * lats[i];
			Yj = DEGREE_TO_RADIAN * lats[j];

			area += (Xi * Yj - Xj * Yi);
		}
		area = std::abs(EARTH_RADIUS_METERS * EARTH_RADIUS_METERS * area);
		area = area * 0.5;
	}
	return area; //will return 0 if feature is an open area
}

// find the length of the given way, in OSMID
double find_way_length(OSMID way_id) {
	//find the way pointer and get OSMID of points vector
	//get way index from wayID_index map, and get the way pointer by getWayByIndex
	const OSMWay* pWay = getWayByIndex(way_id_int_map.at(way_id));
	//get OSMIDs of nodes forming the way
	std::vector<OSMID> nodes = getWayMembers(pWay);
	
	//ll_vec, is to store LatLon of all the nodes
	std::vector<LatLon> ll_vec;
	// loop all the nodes forming the way
	for (int i = 0; i < nodes.size(); i++)
	{
		// get OSMID of node i
		OSMID temp = nodes[i];
		// get its pointer
		const OSMNode* pNode = getNodeByIndex(node_id_int_map.at(temp));
		// get its LatLon and push into vector
		ll_vec.push_back(pNode->coords());
	}

	double dist = 0;
	// loop all adjacent LatLons in the vec, get the distances of the pairs, accumulate them
	for (int i = 0; i < ll_vec.size() - 1; i++)
	{
		std::pair<LatLon, LatLon> temp_pair(ll_vec[i], ll_vec[i + 1]);
		double increment = find_distance_between_two_points(temp_pair);
		dist += increment;
	}
	return dist;
}
