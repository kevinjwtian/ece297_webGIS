/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "m2.h"
#include <iostream>
#include "LatLon.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include "math.h"
#include "BK-Tree.h"
#include "methods.h"
#include "m3.h"
#include <list>
#include <iterator>

BKTree bk_tree; //declare glboal bk_tree variable

std::pair<std::vector<StreetSegmentIndex>, std::vector<StreetSegmentIndex>> total_path; //walk path, drive path
double turn_penalty = 25;
double walking_speed = 0;
double walking_time_limit = 0;
bool routing = false;
bool walking = false;
std::list<int> from_to;
std::vector<int> drive_path; //contains the driving path
std::vector<int> walk_path; //contains the walking path
//Frame_Info frame_info;
Frame_Info global_frame_info; //declare global variable
using boost::property_tree::ptree;
using boost::property_tree::read_json;

GtkWidget* entry_1 = NULL; // input street 1 here
GtkWidget* entry_2 = NULL; // input street 2 here

GtkWidget* entry_1_1 = NULL;
GtkWidget* entry_1_2 = NULL;
GtkWidget* entry_2_1 = NULL;
GtkWidget* entry_2_2 = NULL;
GtkWidget* walk_drive_button = NULL; //walk or just drive button
GtkWidget* entry_walk_time = NULL;
GtkWidget* entry_walk_speed = NULL;

#define  MOUSE_PICK                     1 //click the mouse send this message
#define  MAX_CORRECTION_LENGTH          2
#define  STREET_SIZE_LIMIT_TO_DRAW      200
#define  FEATURE_SIZE_LIMIT_TO_DRAW     100
#define  MAJOR_STREET_THRESHOLD         3000
#define  HIGH_WAY_THRESHOLD             80
#define  POI_NAME_SIZE_LIMIT_TO_DRAW    0.5
#define  STREET_NAME_OFFSET             0.05
#define  Minor_Street_Width             2
#define  Major_Street_Width             3
#define  Highway_Width                  5

//traffic accident icon
std::string traffic_accident_icon_path = "./crash.png";
//std::string traffic_accident_icon_path = "//nfs//ug//homes-1//t//tianji21//ece297//work//mapper//crash.png";
int traffic_incident_icon_width = 45;
int traffic_incident_icon_height = 45;
std::vector<LatLon> incident_position_to_hl;

//highlight icon
std::string inter_hl_icon_path = "./highlight.png";
//std::string inter_hl_icon_path = "//nfs//ug//homes-1//t//tianji21//ece297//work//mapper//highlight.png";
int inter_hl_icon_width = 45;
int inter_hl_icon_height = 45;
std::vector<int> inter_idx_to_hl;

bool flag_just_drive = TRUE; //true: just drive false: walk&drive
bool flag_pick_intersection = false; //whether use mouse to choose start and end intersections true:mouse false:entry

struct incident_info {
    std::string id;
    double lat;
    double lon;

};

std::vector<incident_info> incident_info_vec;

/*
 library of colors for day time street and text
 */
static Street_Style street_style_lib[] = {
    //TExt day
    {ezgl::color(103, 114, 119, 255)},
    //standard color
    {ezgl::color(255, 255, 255, 250)},
    //color for highway day 
    {ezgl::color(255, 241, 178, 255)},

};
/*
 library of colors for night time street and text
 */
static Street_Style street_style_night_lib[] = {

    //TExt self defined
    {ezgl::color(212, 216, 219, 255)},

    //self-define color
    {ezgl::color(78, 81, 84, 255)},

    //color for highway night
    {ezgl::color(148, 109, 61, 255)}

};

/*
 feature color object
 */
struct Feature_Style {
    ezgl::color c;
};
/*
 library of colors for day time features
 */
static Feature_Style feature_style_lib[] = //standard library
{
    //Unknown
    {ezgl::color(218, 240, 224, 255)},
    //Park
    {ezgl::color(198, 231, 198, 255)},
    //Beach
    {ezgl::color(252, 239, 199, 255)},
    //Lake,
    {ezgl::color(172, 219, 253, 255)},
    //River
    {ezgl::color(172, 219, 253, 255)},
    //Island
    {ezgl::color(248, 249, 250, 255)},
    //Building
    {ezgl::color(225, 225, 225, 255)},
    //Greenspace
    {ezgl::color(198, 231, 198, 255)},
    //Golfcourse
    {ezgl::color(198, 231, 198, 255)},
    //Stream
    {ezgl::color(172, 219, 253, 255)}
};

/*
 library of colors for night time street and text
 */
static Feature_Style feature_style_night_lib[] = //self-defined library
{
    //Unknown
    {ezgl::color(44, 45, 47, 255)},
    //Park
    {ezgl::color(39, 53, 53, 255)},
    //Beach
    {ezgl::color(39, 53, 53, 255)},
    //Lake,
    {ezgl::color(54, 67, 101, 255)},
    //River
    {ezgl::color(54, 67, 101, 255)},
    //Island
    {ezgl::color(44, 45, 47, 255)},
    //Building
    {ezgl::color(50, 53, 60, 255)},
    //Greenspace
    {ezgl::color(39, 53, 53, 255)},
    //Golfcourse
    {ezgl::color(39, 53, 53, 255)},
    //Stream
    {ezgl::color(54, 67, 101, 255)}
};


Feature_Style* feature_style = feature_style_lib;
Street_Style* street_style = street_style_lib;

void get_bbox(double& lat_min, double& lon_min, double& lat_max, double& lon_max) {
    int num_nodes = getNumberOfNodes();
    lat_min = getNodeByIndex(0)->coords().lat();
    lon_min = getNodeByIndex(0)->coords().lon();
    lat_max = lat_min;
    lon_max = lon_min;
    for (int i = 1; i < num_nodes; i++) {
        LatLon ll = getNodeByIndex(i)->coords(); //node i's coordinates
        if (ll.lat() < lat_min)
            lat_min = ll.lat();
        if (ll.lat() > lat_max)
            lat_max = ll.lat();
        if (ll.lon() < lon_min)
            lon_min = ll.lon();
        if (ll.lon() > lon_max)
            lon_max = ll.lon();
    }
}

std::pair<double, double> init_convert_ll_xy(double lat, double lon, Frame_Info& frame_info) {
    double x = lon * cos(DEGREE_TO_RADIAN * frame_info.lat_avg);
    double y = lat;
    return std::pair<double, double>(x, y);
}

std::pair<double, double> convert_ll_relative_xy(double lat, double lon, Frame_Info& frame_info) {
    double x = lon * frame_info.param_cos_avg_lat;
    double y = lat;
    x = (x - frame_info.x_min) / frame_info.x_diff * frame_info.WIDTH;
    y = (y - frame_info.y_min) / frame_info.y_diff * frame_info.HEIGHT;
    return std::pair<double, double>(x, y);
}

LatLon convert_relative_xy_ll(double x, double y, Frame_Info& frame_info) {
    double abs_x = x * frame_info.x_diff / frame_info.WIDTH + frame_info.x_min;
    double abs_y = y * frame_info.y_diff / frame_info.HEIGHT + frame_info.y_min;
    // abs_y is the lat
    LatLon ll(abs_y, abs_x / frame_info.param_cos_avg_lat);
    return ll;

}

size_t process_data(void* buffer, size_t size, size_t nmemb, void* userp)//(memory pointer, how large is this memory)
{
    userp = userp;
    char* p = (char*) buffer; //convert to char* type
    std::cout << p << std::endl;
    //judge whether the information is complete.
    std::string str_buffer(p);
    std::string::size_type iter = str_buffer.find("}]}}");
    if (iter == str_buffer.npos)//cannot find }]}}
    {
        show_message(NULL, "information is incomplete\nplease try again later");
        return nmemb;
    }
    std::cout << size << std::endl;
    ptree ptRoot;
    std::istringstream json_data((char*) buffer); //init json_data with buffer, convert buffer to istringstream type
    read_json(json_data, ptRoot); //library function, push data into ptree (this data must be istringstream type)
    ptree sub_tree, sub_sub_tree;
    sub_tree = ptRoot.get_child("tm").get_child("poi"); //sub_tree is "poi"
    for (ptree::iterator it = sub_tree.begin(); it != sub_tree.end(); it++)//loop all elements in the array of poi
    {
        incident_info info;
        sub_sub_tree = it->second; //it-->second: value
        info.id = sub_sub_tree.get<std::string>("id"); //get the value corresponding the key "id"
        info.lat = sub_sub_tree.get_child("p").get<double>("y"); //get the tree of key "p" and get the value corresponding the key "y"
        info.lon = sub_sub_tree.get_child("p").get<double>("x"); //get the tree of key "p" and get the value corresponding the key "x"
        incident_info_vec.push_back(info); //push all info to incident_info_vec
    }
    return nmemb;
}

bool b_show_traffic = false;
//false means no traffic info is being showed.
//true means traffic info is being showd.

void curl_traffic_live(GtkWidget* widget, ezgl::application* application) {
    widget = widget;
    incident_info_vec.clear();
    if (b_show_traffic) {
        b_show_traffic = false;
        application->refresh_drawing();
        return;
    }

    b_show_traffic = true;

    CURLcode res = curl_global_init(CURL_GLOBAL_ALL);
    if (res != CURLE_OK) {
        std::cout << "ERROR: Unable to initialize libcurl" << std::endl;
        std::cout << curl_easy_strerror(res) << std::endl;
        return;
    }
    CURL* curlHandle = curl_easy_init(); //successfully init
    if (!curlHandle) {
        std::cout << "ERROR: Unable to get easy handle" << std::endl;
        return;
    }
    std::string api_url = "https://api.tomtom.com/traffic/services/4/incidentDetails/s3/LAT_MIN%2CLON_MIN%2CLAT_MAX%2CLON_MAX/10/-1/json?projection=EPSG4326&key=a6cKe72wkE2tiJ7Xm9zMhTkorQLm7NES";
    //create a correct sized square to fit URL to access live traffic info
    api_url.replace(api_url.find("LAT_MIN"), 7, std::to_string(global_frame_info.lat_min)); //convert lat_min to string and replace "LAT_MIN" (of URL) withit 
    api_url.replace(api_url.find("LAT_MAX"), 7, std::to_string(global_frame_info.lat_max));
    api_url.replace(api_url.find("LON_MIN"), 7, std::to_string(global_frame_info.lon_min));
    api_url.replace(api_url.find("LON_MAX"), 7, std::to_string(global_frame_info.lon_max));

    res = curl_easy_setopt(curlHandle, CURLOPT_URL, api_url.c_str());
    res = curl_easy_setopt(curlHandle, CURLOPT_WRITEFUNCTION, process_data); //link process_data function to accessed data from URL
    std::cout << api_url << std::endl;

    //res = curl_easy_setopt(curlHandle, CURLOPT_WRITEFUNCTION, write_data);
    if (res != CURLE_OK) {
        std::cout << "ERROR: Unable to set libcurl option" << std::endl;
        std::cout << curl_easy_strerror(res) << std::endl;
    } else {
        res = curl_easy_perform(curlHandle); //actually do the curl performance, get information from URL
    }
    std::cout << std::endl << std::endl;
    if (res == CURLE_OK) {
        std::cout << "All good! res == CURLE_OK!" << std::endl;
    } else {
        std::cout << "ERROR: res == " << res << std::endl;
        std::cout << curl_easy_strerror(res) << std::endl;
    }
    curl_easy_cleanup(curlHandle);
    curlHandle = nullptr;
    curl_global_cleanup();

    application->refresh_drawing();
}

void get_walk_parameters() {
    const gchar* entry_text_walk_speed;
    entry_text_walk_speed = gtk_entry_get_text(GTK_ENTRY(entry_walk_speed)); //get text from entry control 
    walking_speed = atof(entry_text_walk_speed); //convert text into double

    const gchar* entry_text_walk_time;
    entry_text_walk_time = gtk_entry_get_text(GTK_ENTRY(entry_walk_time));
    walking_time_limit = atof(entry_text_walk_time);
}
std::vector<int> find_path_intersections; //store start and end intersections

void act_on_mouse_press(ezgl::application* application, GdkEventButton *event, double x, double y)//(application, which button,position of button)
{
    //left button
    if (event->button == 1) {

        LatLon coord = convert_relative_xy_ll(x, y, global_frame_info); //convert clicked xy to latlon 
        IntersectionIndex inter_idx = find_closest_intersection(coord); //find closest intersection index of given coordinate (where it is clicked)

        if (flag_pick_intersection)//use mouse to choose start and end intersections
        {
            find_path_intersections.push_back(inter_idx); //put inter_idx into vector
            if (find_path_intersections.size() == 1)//is the start intersection
            {
                inter_idx_to_hl.clear(); //clear previous highlights
            }
            if (find_path_intersections.size() == 2)//is the end intersection
            {
                if (flag_just_drive)//just drive
                {
                    total_path.first.clear(); //clear the walk path
                    total_path.second = find_path_between_intersections(
                            find_path_intersections[0],
                            find_path_intersections[1],
                            turn_penalty); //call algorithm to get drive path
                } else {
                    total_path = find_path_with_walk_to_pick_up(
                            find_path_intersections[0],
                            find_path_intersections[1],
                            turn_penalty,
                            walking_speed,
                            walking_time_limit); //call algorithm to get total path
                }
                output_travel_details();
                flag_pick_intersection = false; //turn off the mouse function  (mouse only works once)
                find_path_intersections.clear(); //clear start and end intersections
            }
        }
        //output intersection info here
        inter_idx_to_hl.push_back(inter_idx); //push index into inter_idx_to_hl
        output_inter_info(inter_idx_to_hl); //output intersections info from inter_idx_to_hl on terminal
        application->refresh_drawing(); //redraw
        //m3 implements
        if (routing) {//last two entries are from and to    
            //load from to vec
            int size = from_to.size();

            if (size == 2) {
                from_to.pop_front(); //remove earlier entry
            }
            from_to.push_back(inter_idx);
            //call find_route();
            size = from_to.size();
            if (size == 2) {
                find_route();
            }
            //end find route
        }
    }
    //2 for mid button, 3 for right button
    if (event->button == 3) {
        inter_idx_to_hl.clear(); //clear all highlighted info
        from_to.clear();
        total_path.first.clear(); //clear walk path
        total_path.second.clear(); //clear drive path
        application->refresh_drawing(); //redraw
    }
}

void act_on_mouse_move(ezgl::application* application, GdkEventButton *event, double x, double y) {
    application = application;
    event = event;
    x = x;
    y = y;
}

void act_on_key_press(ezgl::application* application, GdkEventKey *event, char* key_name) {
    application = application;
    key_name = key_name;
    event = event;

}

void on_entry_changed(GtkEntry* entry, gpointer user_data)//this function is called when the entry is changed
{

    user_data = user_data;
    const gchar* entry_text;
    entry_text = gtk_entry_get_text(GTK_ENTRY(entry)); //get what is on entry
    std::string street_name_prefix((const char*) entry_text); //convert char* to string

    std::vector<int> street_id_vec = find_street_ids_from_partial_street_name(street_name_prefix);
    if (street_id_vec.size() > 10)
        return;

    if (street_id_vec.size() == 0)
        return;

    for (int i = 0; i < street_id_vec.size(); i++) {
        std::string street_name_0 = getStreetName(street_id_vec[0]); //first street name
        std::string street_name_i = getStreetName(street_id_vec[i]); //ith street name
        if (street_name_0.compare(street_name_i) != 0) //if names are not equal
            return;
    }
    gtk_entry_set_text(entry, getStreetName(street_id_vec[0]).c_str()); //1 street name is same as all others, show the first street name on entry
    return;
}

void show_message(GObject* parent, const gchar* message)//parent dialog and show message
{
    GtkWidget* dialog;
    //    gchar title []= "Message";
    dialog = gtk_message_dialog_new((GtkWindow*) parent,
            GTK_DIALOG_DESTROY_WITH_PARENT,
            GTK_MESSAGE_INFO,
            GTK_BUTTONS_OK,
            "%s", message); //,              title);
    gtk_window_set_title(GTK_WINDOW(dialog), "Information");
    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);
}

bool get_path_intersections(std::pair<int, int>& start_end_inters) {
    std::vector<int> inter_1 = find_inters(entry_1_1, entry_1_2);
    std::vector<int> inter_2 = find_inters(entry_2_1, entry_2_2);

    if (inter_1.size() == 0 || inter_2.size() == 0)//cannot find intersections
        return false;
    start_end_inters.first = inter_1[0]; //use the first result to be the start intersection
    start_end_inters.second = inter_2[0]; //use the first result to be the end intersection

    return true;

}

void on_find_path_response(GtkDialog* dialog, gint response_id, gpointer user_data) //to response find path dialog's buttons
{
    // For demonstration purposes, this will show the enum name and int value of the button that was pressed

    switch (response_id) {//which button is pressed
        case MOUSE_PICK:
            flag_pick_intersection = TRUE;
            if (!flag_just_drive)
                get_walk_parameters(); //walk&drive
            show_message(NULL, "please click start and end intersections on the map");
            break;
        case GTK_RESPONSE_ACCEPT: //find button use entries to find start and end intersections
        {
            if (!flag_just_drive)
                get_walk_parameters(); //walk&drive
            std::pair<int, int> start_end_inters;
            //use reference to change start_end_inters  use the return bool to tell if intersection is found
            if (!get_path_intersections(start_end_inters)) {
                show_message((GObject*) dialog, "No intersections found!");
                return;
            }

            if (flag_just_drive)//just drive
            {
                total_path.first.clear();
                total_path.second = find_path_between_intersections(
                        start_end_inters.first,
                        start_end_inters.second,
                        turn_penalty);
            } else {
                total_path = find_path_with_walk_to_pick_up(//walk & drive
                        start_end_inters.first,
                        start_end_inters.second,
                        turn_penalty,
                        walking_speed,
                        walking_time_limit);
            }
            output_travel_details();
            find_path_intersections.clear();
            ((ezgl::application *)user_data)->refresh_drawing(); //redraw canvas
            break;
        }
        case GTK_RESPONSE_REJECT://cancel 
            break;
        default:
            break;
    }

    // This will cause the dialog to be destroyed and close
    // without this line the dialog remains open unless the
    // response_id is GTK_RESPONSE_DELETE_EVENT which
    // automatically closes the dialog without the following line.
    gtk_widget_destroy(GTK_WIDGET(dialog)); //destroy dialog
    entry_1_1 = NULL;
    entry_1_2 = NULL;
    entry_2_1 = NULL;
    entry_2_2 = NULL;
    entry_walk_time = NULL;
    entry_walk_speed = NULL;
}

//handle all response of the dialog, if the response_id is ACCEPT, then output intersection information

void on_find_dialog_response(GtkDialog* dialog, gint response_id, gpointer user_data)//which dialog sent the message, what message, 
{
    // For demonstration purposes, this will show the enum name and int value of the button
    //that was pressed
    std::vector<int> inters; //container contains intersections

    switch (response_id) {

        case GTK_RESPONSE_ACCEPT: //have typed 2 streets

            inters = find_inters(entry_1, entry_2); //all intersections found
            if (inters.size() == 0) //number of intersections found is 0 
            {
                show_message((GObject*) dialog, "No intersections found!");
                return;
            }
            output_inter_info(inters); //output all intersections information on terminal
            inter_idx_to_hl.clear(); //clear previous highlighted intersections
            inter_idx_to_hl.swap(inters); //highlight becomes inters, inters becomes empty
            ((ezgl::application *)user_data)->refresh_drawing(); //redraw
            break;
        case GTK_RESPONSE_REJECT:
            break;
        default:
            break;
    }

    // This will cause the dialog to be destroyed and close
    // without this line the dialog remains open unless the
    // response_id is GTK_RESPONSE_DELETE_EVENT which
    // automatically closes the dialog without the following line.
    gtk_widget_destroy(GTK_WIDGET(dialog)); //destroy dialog
    entry_1 = NULL;
    entry_2 = NULL;
}
// in this function, a dialog is popped up
//the function is supposed to be able to create a dialog where users can input street names

void find_inter_from_streets(GtkWidget* window, ezgl::application * application) {
    window = window;
    GObject* parent_window; // the parent window over which to add the dialog
    GtkWidget* content_area; // the content area of the dialog
    GtkWidget* label_1; // the label we will create to display a message in the content area
    GtkWidget* label_2; // the label we will create to display a message in the content area
    GtkWidget* dialog; // the dialog box we will create
    //create the dialog
    //parent_window = application->get_object(application->get_main_window_id());//parent window
    parent_window = application->get_object("MainWindow");


    dialog = gtk_dialog_new_with_buttons(//create a new dialog
            "Find Intersection", //title
            (GtkWindow*) parent_window, //parent window
            GTK_DIALOG_MODAL, //GtkDialog flags
            ("OK"), //button name
            GTK_RESPONSE_ACCEPT, //button sent message
            ("CANCEL"),
            GTK_RESPONSE_REJECT,
            NULL // a Null value is the end of the pairs
            );
    // get the content_area of the dialog
    // content_area means the client area
    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog)); //get available area

    // Create a label and attach it to the content area of the dialog
    label_1 = gtk_label_new("street 1:"); //create a new label named "street 1:"
    gtk_container_add(GTK_CONTAINER(content_area), label_1); //put this label into content_area
    entry_1 = gtk_entry_new(); //create a new blank entry which can type works
    gtk_container_add(GTK_CONTAINER(content_area), entry_1); //put this entry into content_area
    g_signal_connect(entry_1, "changed", G_CALLBACK(on_entry_changed), NULL); //when entry_1 send "changed" message, call on_entry_changed function


    label_2 = gtk_label_new("street 2:");
    gtk_container_add(GTK_CONTAINER(content_area), label_2);
    entry_2 = gtk_entry_new();
    gtk_container_add(GTK_CONTAINER(content_area), entry_2);
    g_signal_connect(entry_2, "changed", G_CALLBACK(on_entry_changed), NULL);

    // The main purpose of this is to show dialog's child widget, label
    gtk_widget_show_all(dialog); //show this dialog
    g_signal_connect(GTK_DIALOG(dialog), "response", G_CALLBACK(on_find_dialog_response), application); //when dialog send "response" message, call on_find_dialog_response function

    return;
}

std::vector<int> find_inters(GtkWidget* local_entry_1, GtkWidget* local_entry_2) {
    std::vector<int> result_vec;

    if (local_entry_1 == NULL || local_entry_2 == NULL)
        return result_vec;

    std::string response = "";
    const gchar* prefix_1 = gtk_entry_get_text(GTK_ENTRY(local_entry_1)); //get entry's content, put them into prefix_1
    const gchar* prefix_2 = gtk_entry_get_text(GTK_ENTRY(local_entry_2));
    std::vector<int> street_id_vec_1, street_id_vec_2;
    street_id_vec_1 = find_street_ids_from_partial_street_name(std::string(prefix_1)); //find all streets' ids started with prefix_1
    street_id_vec_2 = find_street_ids_from_partial_street_name(std::string(prefix_2));

    if (street_id_vec_1.size() == 0)//cannot find any result so the typed name is wrong
    {
        response.append("no street name can be found to match input of entry 1\n");
        std::vector<std::pair<int, std::string>> auto_correction_result = bk_tree.Find(std::string(prefix_1), MAX_CORRECTION_LENGTH);
        if (auto_correction_result.size() == 0)//within current max_correction_length, no corrected street name is found
            response.append("no corrected street name with given length exists\n");
        else {
            int temp_length = MAX_CORRECTION_LENGTH + 1;
            std::string str_value;
            for (auto pair_data : auto_correction_result) {
                if (pair_data.first < temp_length) {
                    temp_length = pair_data.first; //update temp_length to make it shortest 
                    str_value = pair_data.second; //update corrected street name
                    if (pair_data.first == 1)//if distance is already the shortest one
                        break;
                }
            }
            response.append("input street name has been corrected as : ");
            response.append(str_value);
            response.append("\n");
            street_id_vec_1 = find_street_ids_from_partial_street_name(str_value); //find ids from new corrected street name
        }
    }
    if (street_id_vec_2.size() == 0) {
        response.append("no street name can be found to match input of entry 2\n");
        std::vector<std::pair<int, std::string>> auto_correction_result = bk_tree.Find(std::string(prefix_2), MAX_CORRECTION_LENGTH);
        if (auto_correction_result.size() == 0)
            response.append("no corrected name with given length exists\n");
        else {
            int temp_length = MAX_CORRECTION_LENGTH + 1;
            std::string str_value;
            for (auto pair_data : auto_correction_result) {
                if (pair_data.first < temp_length) {
                    temp_length = pair_data.first;
                    str_value = pair_data.second;
                    if (pair_data.first == 1)
                        break;
                }
            }
            response.append("input has been corrected as : ");
            response.append(str_value);
            response.append("\n");
            street_id_vec_2 = find_street_ids_from_partial_street_name(str_value);
        }
    }
    if (response.length() > 0)//if response is not empty, it means something is wrong and needs correction
        show_message(NULL, response.c_str());
    for (int i = 0; i < street_id_vec_1.size(); i++) {
        for (int j = 0; j < street_id_vec_2.size(); j++) {
            std::vector<int> temp = find_intersections_of_two_streets(std::pair<int, int>(street_id_vec_1[i], street_id_vec_2[j])); //call m1 function to find intersection
            result_vec.insert(result_vec.end(), temp.begin(), temp.end());
        }
    }
    gunique(result_vec); //if have repeated intersections, erase repeated ones
    return result_vec;
}

void output_inter_info(std::vector<int> inters)//output all intersections information
{
    for (int i = 0; i < inters.size(); i++) {
        //std::cout << "intersection index : " << inters[i] << std::endl;
        std::cout << "street names of intersection " << inters[i] << ":" << std::endl;

        std::vector<std::string> street_names = find_street_names_of_intersection(inters[i]); // 1 2 4 3 2 1
        std::sort(street_names.begin(), street_names.end()); //1 1 2 2 3 4 
        std::vector<std::string>::iterator iter = std::unique(street_names.begin(), street_names.end()); //1 2 3 4 1 2 and return position of the second 1
        street_names.erase(iter, street_names.end()); //delete everything after the second 1

        for (int j = 0; j < street_names.size(); j++) {
            std::cout << "\t" << street_names[j] << std::endl;
        }
    }
}
// time costing, try not to call it often

Frame_Info get_frame_info() {
    Frame_Info frame_info;
    get_bbox(frame_info.lat_min, frame_info.lon_min, frame_info.lat_max, frame_info.lon_max);
    frame_info.lat_avg = (frame_info.lat_min + frame_info.lat_max) * 0.5;
    frame_info.param_cos_avg_lat = cos(DEGREE_TO_RADIAN * frame_info.lat_avg);
    std::pair<double, double> init_min_xy = init_convert_ll_xy(frame_info.lat_min, frame_info.lon_min, frame_info); //(lat,lon)-->(x,y)
    std::pair<double, double> init_max_xy = init_convert_ll_xy(frame_info.lat_max, frame_info.lon_max, frame_info);
    frame_info.x_min = init_min_xy.first;
    frame_info.y_min = init_min_xy.second;
    frame_info.x_max = init_max_xy.first;
    frame_info.y_max = init_max_xy.second;

    frame_info.x_diff = frame_info.x_max - frame_info.x_min;
    frame_info.y_diff = frame_info.y_max - frame_info.y_min;
    if (frame_info.y_diff > frame_info.x_diff) {
        frame_info.HEIGHT = BASE_SIZE;
        frame_info.WIDTH = frame_info.HEIGHT / frame_info.y_diff * frame_info.x_diff;
    } else {
        frame_info.WIDTH = BASE_SIZE;
        frame_info.HEIGHT = frame_info.WIDTH / frame_info.x_diff * frame_info.y_diff;
    }
    // calculate the real distance between wWS to NE
    LatLon ll_min(frame_info.lat_min, frame_info.lon_min);
    LatLon ll_max(frame_info.lat_max, frame_info.lon_max);
    double distance_min_max = find_distance_between_two_points(std::pair<LatLon, LatLon>(ll_min, ll_max));
    // caculate the distance between WS to NE in world coordinate system
    double distance_world_coordinate = sqrt(pow(frame_info.HEIGHT, 2) + pow(frame_info.WIDTH, 2));
    frame_info.rate_distance_xy = distance_min_max / distance_world_coordinate;

    return frame_info;
}

/*function called whenever map is redrawn*/
void draw_main_canvas(ezgl::renderer *g) {
    draw_features(g);
    draw_streets(g);
    draw_POI_names(g);
    draw_high_lights(g);
    draw_path(g);
    return;
}

void draw_path(ezgl::renderer *g) {
    Street_Style walk_style{ezgl::RED};
    Street_Style drive_style{ezgl::color(73,163,249,255)};
    for (auto segment_idx : total_path.first)//loop all segment_idx, draw walk path
    {
        g->set_line_dash(ezgl::line_dash::asymmetric_5_3);
        draw_street_segment(g, segment_idx, walk_style, walk_style);
    }
    for (auto segment_idx : total_path.second)//loop all segment_idx,draw drive path
    {
        g->set_line_dash(ezgl::line_dash::none); // Solid line
        draw_street_segment(g, segment_idx, drive_style, drive_style);
    }

}

void draw_high_lights(ezgl::renderer* g) {

    //draw intersections
    ezgl::surface *inter_hl_icon = ezgl::renderer::load_png(inter_hl_icon_path.c_str()); //load png, convert string to char
    for (int i = 0; i < inter_idx_to_hl.size(); i++)//loop all intersections to highlight
    {
        LatLon temp_ll = getIntersectionPosition(inter_idx_to_hl[i]); //latlon of intersection
        std::pair<double, double> temp_xy = convert_ll_relative_xy(temp_ll.lat(), temp_ll.lon(), global_frame_info); //latlon-->xy


        g->draw_surface(inter_hl_icon, //icon png 
        {temp_xy.first - inter_hl_icon_width * 0.5 / get_zoom_times(g),
            temp_xy.second + inter_hl_icon_height / get_zoom_times(g)}); //position of icon
    }
    ezgl::renderer::free_surface(inter_hl_icon);

    //draw traffic incident
    ezgl::surface *incident_icon = ezgl::renderer::load_png(traffic_accident_icon_path.c_str());
    for (int i = 0; i < incident_info_vec.size(); i++) {
        incident_info temp_info = incident_info_vec[i];
        std::pair<double, double> temp_xy = convert_ll_relative_xy(temp_info.lat, temp_info.lon, global_frame_info);


        g->draw_surface(incident_icon,{temp_xy.first - traffic_incident_icon_width * 0.5 / get_zoom_times(g),
            temp_xy.second + traffic_incident_icon_height * 0.5 / get_zoom_times(g)});
    }
    ezgl::renderer::free_surface(incident_icon);
}

void draw_POI_names(ezgl::renderer * g) {
    g->set_text_rotation(0); //no degree
    g->set_font_size(18);
    g->set_color(ezgl::color(103, 114, 119, 255));
    int nPOI_Count = getNumPointsOfInterest();
    for (int i = 0; i < nPOI_Count; i++)//loop all pois
    {
        std::string POI_name = getPointOfInterestName(i);
        LatLon ll = getPointOfInterestPosition(i);
        std::pair<double, double> xy_POI = convert_ll_relative_xy(ll.lat(), ll.lon(), global_frame_info);

        g->draw_text({xy_POI.first, xy_POI.second}, POI_name, POI_NAME_SIZE_LIMIT_TO_DRAW, POI_NAME_SIZE_LIMIT_TO_DRAW);
    }
}

double get_zoom_times(ezgl::renderer* g) {
    const ezgl::rectangle rc = g->get_visible_world(); //visible range can be seen on the map

    // how many times a object is shown in the screen
    return global_frame_info.WIDTH / rc.width(); //real world/world on the map
}
//visualize all street

void draw_streets(ezgl::renderer *g) {
    double zoom_times = get_zoom_times(g); //calculate how many times a object is shown in the screen
    g->set_line_cap(ezgl::line_cap::butt); // Butt ends
    g->set_line_dash(ezgl::line_dash::none); // Solid line

    //vector for points
    std::vector<LatLon> seg_points;
    //ezgl::rectangle visuable_size = 
    //loop all streets
    int nStreet_Count = getNumStreets();
    for (int i = 0; i < nStreet_Count; i++)//loop each street
    {
        //find street segments
        std::vector<int> street_seg_vec = find_street_segments_of_street(i);
        Street_Style style;
        Street_Style style2; //name
        double street_length = 0;
        double speedLimit = 0;
        for (auto inter_idx : street_seg_vec)//loop each segment
        {
            street_length += find_street_segment_length(inter_idx); //calculate each street's length
            if (speedLimit <= getInfoStreetSegment(inter_idx).speedLimit) speedLimit = getInfoStreetSegment(inter_idx).speedLimit;
        }
        // too small to show
        if (street_length / global_frame_info.rate_distance_xy * zoom_times < STREET_SIZE_LIMIT_TO_DRAW)//length seen on the map smaller than
            continue; //do not show, end the loop
        // tell major or minor
        if (speedLimit >= HIGH_WAY_THRESHOLD && street_length > 10000)//decide by speed
        {
            g->set_line_width(Highway_Width);
            style = street_style[1];
            style2 = street_style[0];
            //  g->set_color(style.c);

        } else if ((street_length >= MAJOR_STREET_THRESHOLD) && (street_length <= HIGH_WAY_THRESHOLD)) { //major
            g->set_line_width(Major_Street_Width);
            style = street_style[1];
            style2 = street_style[0];
            // g->set_color(style.c);
        } else {

            g->set_line_width(Minor_Street_Width); //
            style = street_style[1];
            style2 = street_style[0];
            // g->set_color(style.c);
        }

        for (auto inter_idx : street_seg_vec)//loop each street segment
            draw_street_segment(g, inter_idx, style, style2);

    }
}

void draw_street_segment(ezgl::renderer* g, IntersectionIndex inter_idx, Street_Style style, Street_Style style2) {
    g->set_font_size(20); //font size of the word
    g->set_color(style.c);
    std::vector<std::pair<double, double>> seg_points;

    IntersectionIndex from = getInfoStreetSegment(inter_idx).from; //index of start point of seg
    IntersectionIndex to = getInfoStreetSegment(inter_idx).to; //index of end point of seg
    LatLon seg_from = getIntersectionPosition(from); //latlon position of seg start point
    LatLon seg_to = getIntersectionPosition(to); //latlon position of seg end point
    //start point
    std::pair<double, double> xy_from = convert_ll_relative_xy(seg_from.lat(), seg_from.lon(), global_frame_info); //latlon-->relative xy
    seg_points.push_back(xy_from); //push relative xy to seg points

    //curve points
    for (int j = 0; j < getInfoStreetSegment(inter_idx).curvePointCount; j++)//loop curve points
    {
        LatLon temp = getStreetSegmentCurvePoint(j, inter_idx);
        std::pair<double, double> xy_curve = convert_ll_relative_xy(temp.lat(), temp.lon(), global_frame_info); //curve point relative xy
        seg_points.push_back(xy_curve);
    }
    //end point
    std::pair<double, double> xy_to = convert_ll_relative_xy(seg_to.lat(), seg_to.lon(), global_frame_info); //latlon-->relative xy
    seg_points.push_back(xy_to); //push relative xy to seg points

    for (int j = 0; j < seg_points.size() - 1; j++) {
        g->draw_line({seg_points[j].first, seg_points[j].second},
        {
            seg_points[j + 1].first, seg_points[j + 1].second
        }); //draw line
    }

    if (seg_points.size() == 2)//write name only for straight line (only start and end points) 
    {
        StreetIndex street_idx = getInfoStreetSegment(inter_idx).streetID; //get id of this street
        std::string street_name = getStreetName(street_idx); //get name from id 

        //get new name if it is one-way
        //using <<< or >>> to show the direction
        if (getInfoStreetSegment(inter_idx).oneWay) //if the segment is one-way
        {
            if (seg_points[0].first < seg_points[1].first) // if start point x is smaller than end point x, from is left , end is right
            {
                street_name.append(">>>>>");
            } else if (seg_points[0].first > seg_points[1].first) {// if start point x is bigger than end point x, from is right , end is left
                street_name.insert(0, std::string("<<<<<"));

            } else {//x is equal, so street segement is vertical
                if (seg_points[0].second < seg_points[1].second)
                    street_name.append(">>>>>");
                else
                    street_name.insert(0, std::string("<<<<<"));
            }
        }
        double degree = 0;
        double x_delta = seg_points[0].first - seg_points[1].first;
        double y_delta = seg_points[0].second - seg_points[1].second;
        double rate = 0;
        double rad = 0;
        if (x_delta == 0)
            degree = 90; //street is vertical
        else {
            rate = y_delta / x_delta;
            rad = atan(rate);
            degree = rad / 2 / 3.1415926 * 360;
            if (degree < 0)
                degree += 360; //270-360  0-90
        }
        g->set_text_rotation(degree);

        double x_offset, y_offset;
        y_offset = cos(rad) * STREET_NAME_OFFSET;
        x_offset = sin(rad) * STREET_NAME_OFFSET;

        g->set_color(style2.c);
        g->draw_text({(seg_points[0].first + seg_points[1].first)*0.5 + x_offset,
            (seg_points[0].second + seg_points[1].second)*0.5 + y_offset}, //middle of the seg

        street_name,
                1 * sqrt(pow(x_delta, 2) + pow(y_delta, 2)),
                1 * sqrt(pow(x_delta, 2) + pow(y_delta, 2))
                );

    }
    seg_points.clear();

}

void choose_map(GtkWidget* widget, ezgl::application* application) {
    widget = widget;

    std::cout << "in choose map fun" << std::endl;
    GtkWidget *dialog;
    GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_OPEN;
    gint res;
    //create a new file dialog
    dialog = gtk_file_chooser_dialog_new("Open File", //title
            (GtkWindow*) application->get_object("MainWindow"), //parent window
            action, //what does this dialog do? open the file
            ("_Cancel"), //button name
            GTK_RESPONSE_CANCEL, //button sent message when it is clicked
            ("_Open"),
            GTK_RESPONSE_ACCEPT,
            NULL);
    res = gtk_dialog_run(GTK_DIALOG(dialog)); //run the dialog, res is the response when the button is clicked
    if (res == GTK_RESPONSE_ACCEPT) {
        char* filename;
        GtkFileChooser * chooser = GTK_FILE_CHOOSER(dialog);
        filename = gtk_file_chooser_get_filename(chooser);
        std::cout << filename << std::endl; //output the file
        std::string str_filename(filename);
        if (str_filename.find(".streets.bin") == str_filename.npos)//if cannot find ".streets.bin"
        {
            show_message((GObject*) dialog, "please choose files ending with .street.bin");
        } else //if can find ".streets.bin"
        {
            bool loaded = load_map(std::string(str_filename)); //load map
            if (loaded) {
                std::cout << "map loaded successfully" << std::endl;
                init_BKtree(); //load bk-tree with trimmed and lowered street names
                Frame_Info frame_info = get_frame_info(); //get frame info of the map
                global_frame_info = frame_info; //local variable to global variable
                application->refresh_drawing();
            } else {
                show_message((GObject*) dialog, "previous loaded map is closed.\nbut no new map is loaded.\nplease choose files ending with .street.bin");
            }
        }
    }
    gtk_widget_destroy(GTK_WIDGET(dialog)); //destroy dialog

}

void init_BKtree() {
    bk_tree.Clear();

    int street_count = getNumStreets();
    for (int i = 0; i < street_count; i++) {
        std::string street_name = getStreetName(i); //get all street names
        gtrim(street_name); //delete space
        gstringU2L(street_name); //Upper case to lower case
        bk_tree.Add(street_name); //add all these street names to tree

    }
}

void initial_setup(ezgl::application* application, bool new_window) {
    new_window = new_window;
    application->update_message("CD015 Mapper");
    application->create_button("Find Intersection", 6, find_inter_from_streets);
    application->create_button("Select Map", 7, choose_map);
    application->create_button("Live Traffic Congestion", 8, curl_traffic_live);
    application->create_button("Night Mode", 9, night_mode_button);
    application->create_button("Find Path", 10, find_path);
    //application->create_button("Routing", 11, pool);
    application->create_button("Help", 12, show_help);
}

void path_mode_change(GtkWidget* window, ezgl::application * application) {
    window = window; //unused parameter
    application = application; //unused parameter
    if (flag_just_drive) {
        flag_just_drive = FALSE;
        gtk_button_set_label(GTK_BUTTON(window), "walk & drive");
    } else {
        flag_just_drive = TRUE;
        gtk_button_set_label(GTK_BUTTON(window), "just drive");
    }

}

void output_travel_details() {
    double walking_time = compute_path_walking_time(total_path.first,walking_speed,turn_penalty);;
    double drving_time = compute_path_travel_time(total_path.second,turn_penalty);
    int total = (int) (walking_time + drving_time);
    total = total / 60; //in min
    
    std::cout << "travel directions are:" << std::endl;
    std::cout << "\twalking routines:" << std::endl;
    int curr_streetID = -1;
    for (auto segment_idx : total_path.first)//loop all segment in walk path
    {
        int streetID = getInfoStreetSegment(segment_idx).streetID; //get streetID of segment
        if (curr_streetID != streetID)//if streetID change
        {
            std::cout << "\t\tgo to " << getStreetName(streetID) << std::endl; //output new street name
            curr_streetID = streetID;
        }
    }
    std::cout << "\tdriving routines:" << std::endl;
    curr_streetID = -1;
    for (auto segment_idx : total_path.second) {
        int streetID = getInfoStreetSegment(segment_idx).streetID;
        if (curr_streetID != streetID) {
            std::cout << "\t\tgo to " << getStreetName(streetID) << std::endl;
            curr_streetID = streetID;
        }
    }
    if(total<1){
        std::cout << "You will arrive at the destination in less than 1 minute."<< std::endl;
    }
    else{
        
        std::cout << "You arrive at the destination in " << total<<" minutes."<< std::endl;
    }
}

void show_help(GtkWidget* window, ezgl::application * application) {
    application = application;
    window = window;
    const char* to_show =
            "Quick Start\n"
            "Move map with Buttons on the right "
            "Zoom map with either mouse wheel or buttons on the right\n"
            "Click on Choose Map and switch to a different map\n"
            "Click on Find Intersection and input street names to locate an intersection\n"
            "Click on Night Mode for darker colors\n"
            "Click the 'Find Path' button for navigation\n"
            "   Type street names of start and end intersections\n"
            "   Choose routing mode\n"
            "   If walk & drive mode chosen, please enter walk time and walk speed\n"
            "   Click the 'FIND' button\n"
            "*  Or click the 'MOUSE' button then click start and end intersections on the map\n"
            //"*  OR you can click on Routing and click on any two spots\n"
            "*  Walk path is colored in red and dotted and drive path is colored in blue\n"
            "*  Driving Instructions are shown on the terminal\n";
    show_message(NULL, to_show);

}

void find_path(GtkWidget* window, ezgl::application * application) {
    window = window;
    GObject* parent_window; // the parent window over which to add the dialog
    GtkWidget* content_area; // the content area of the dialog
    GtkWidget* street_1_1; // the label we will create to display a message in the content area
    GtkWidget* street_1_2; // the label we will create to display a message in the content area
    GtkWidget* street_2_1; // the label we will create to display a message in the content area
    GtkWidget* street_2_2; // the label we will create to display a message in the content area
    GtkWidget* dialog; // the dialog box we will create

    parent_window = application->get_object("MainWindow");


    dialog = gtk_dialog_new_with_buttons(//create a new dialog
            "Find Path", //title
            (GtkWindow*) parent_window, //parent window
            GTK_DIALOG_MODAL, //GtkDialog flags
            ("MOUSE"), //button name
            MOUSE_PICK, //button sent message
            ("FIND"), //button name
            GTK_RESPONSE_ACCEPT, //button sent message
            ("CANCEL"),
            GTK_RESPONSE_REJECT,
            NULL // a Null value is the end of the pairs
            );
    // get the content_area of the dialog
    // content_area means the client area
    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog)); //get available area

    // Create a label and attach it to the content area of the dialog
    street_1_1 = gtk_label_new("street 1 of start intersection:"); //create a new label named "street_1_1"
    gtk_container_add(GTK_CONTAINER(content_area), street_1_1); //put this label into content_area
    entry_1_1 = gtk_entry_new(); //create a new blank entry which can type works
    gtk_container_add(GTK_CONTAINER(content_area), entry_1_1); //put this entry into content_area
    g_signal_connect(entry_1_1, "changed", G_CALLBACK(on_entry_changed), NULL); //when entry_1_1 send "changed" message, call on_entry_changed function

    street_1_2 = gtk_label_new("street 2 of start intersection:"); //create a new label named "street 1_2:"
    gtk_container_add(GTK_CONTAINER(content_area), street_1_2); //put this label into content_area
    entry_1_2 = gtk_entry_new(); //create a new blank entry which can type works
    gtk_container_add(GTK_CONTAINER(content_area), entry_1_2); //put this entry into content_area
    g_signal_connect(entry_1_2, "changed", G_CALLBACK(on_entry_changed), NULL); //when entry_1_2 send "changed" message, call on_entry_changed function

    street_2_1 = gtk_label_new("street 1 of end intersection:"); //create a new label named "street 2_1:"
    gtk_container_add(GTK_CONTAINER(content_area), street_2_1); //put this label into content_area
    entry_2_1 = gtk_entry_new(); //create a new blank entry which can type works
    gtk_container_add(GTK_CONTAINER(content_area), entry_2_1); //put this entry into content_area
    g_signal_connect(entry_2_1, "changed", G_CALLBACK(on_entry_changed), NULL); //when entry_2_1 send "changed" message, call on_entry_changed function


    street_2_2 = gtk_label_new("street 2 of end intersection:"); //create a new label named "street 2_2:"
    gtk_container_add(GTK_CONTAINER(content_area), street_2_2); //put this label into content_area
    entry_2_2 = gtk_entry_new(); //create a new blank entry which can type works
    gtk_container_add(GTK_CONTAINER(content_area), entry_2_2); //put this entry into content_area
    g_signal_connect(entry_2_2, "changed", G_CALLBACK(on_entry_changed), NULL); //when entry_2_2 send "changed" message, call on_entry_changed function


    GtkWidget* temp_label_1 = gtk_label_new("switch path mode:"); //create new label
    gtk_container_add(GTK_CONTAINER(content_area), temp_label_1);

    walk_drive_button = gtk_button_new_with_label("just drive"); //create new button
    gtk_container_add(GTK_CONTAINER(content_area), walk_drive_button); //put this entry into content_area
    g_signal_connect(walk_drive_button, "clicked", G_CALLBACK(path_mode_change), NULL); //connect button and its response function

    GtkWidget* temp_label_2 = gtk_label_new("\n\n"); //create space
    gtk_container_add(GTK_CONTAINER(content_area), temp_label_2);

    GtkWidget* label_walk_time = gtk_label_new("walk time (second):"); //create a new label named "walked time (second):"
    gtk_container_add(GTK_CONTAINER(content_area), label_walk_time); //put this label into content_area
    entry_walk_time = gtk_entry_new(); //create a new blank entry which can type works
    gtk_container_add(GTK_CONTAINER(content_area), entry_walk_time); //put this entry into content_area

    GtkWidget* label_walk_speed = gtk_label_new("walk speed (m/s):"); //create a new label named "walk speed (m/s):"
    gtk_container_add(GTK_CONTAINER(content_area), label_walk_speed); //put this entry into content_area
    entry_walk_speed = gtk_entry_new(); //create a new blank entry which can type works
    gtk_container_add(GTK_CONTAINER(content_area), entry_walk_speed); //put this entry into content_area


    // The main purpose of this is to show dialog's child widget, label
    gtk_widget_show_all(dialog); //show this dialog

    //when dialog send "response" message, call on_find_dialog_response function
    g_signal_connect(GTK_DIALOG(dialog), "response", G_CALLBACK(on_find_path_response), application);

    return;
}

void pool(GtkWidget* window, ezgl::application * application) {//function for finding route by typing
    window = window;
    // make a boolean value true
    routing = !routing;
    if (routing)
        application->update_message("Routing");
    else
        application->update_message("Team 015 Map");

}

void find_route() {//only called in at least 2 intersections are displayed && routing is true
    //find a path between last two entries(size - 1, size -2);
    std::cout << "routing" << endl;
    std::cout << "from " << from_to.front() << " to " << from_to.back() << endl;
    if (from_to.front() == from_to.back()) {
        // start = end
        std::cout << "start = end, quitting" << endl;
        return;
    }

    walk_path.clear();
    drive_path.clear();

    if (walking) {//express pool
        /*std::pair<std::vector<int>, std::vector<int>> 
    find_path_with_walk_to_pick_up(
    const int start_intersection,
    const int end_intersection,
    const double turn_penalty,
    const double walking_speed,
    const double walking_time_limit)*/
        //make a pair and call m3 function
        std::pair<std::vector<int>, std::vector<int>> walk_n_drive = find_path_with_walk_to_pick_up(
                from_to.front(), from_to.back(), turn_penalty, walking_speed, walking_time_limit
                );
        walk_path = walk_n_drive.first;
        drive_path = walk_n_drive.second;
        walk_n_drive.first.clear();
        walk_n_drive.second.clear();
    } else {//normal pool
        drive_path = find_path_between_intersections(
                from_to.front(), from_to.back(), turn_penalty
                );
    }
    if (drive_path.size() >= 2) {
        std::cout << "from " << drive_path.at(0) << " to " << drive_path.at(drive_path.size() - 1) << endl;
    }

}

void night_mode_button(GtkWidget* window, ezgl::application * application) {
    window = window;
    if (feature_style == feature_style_lib)//exchange between standard library and self-defined library
    {
        application->update_message("Night Mode");
        feature_style = feature_style_night_lib;
        //day time base color:  ezgl::color(248,249,250,255)
        //night time base color ezgl::color(44,45,47,255)
        application->get_canvas(application->get_main_canvas_id())->bgc(ezgl::color(44, 45, 47, 255));
    } else {
        application->update_message("Day Mode");
        feature_style = feature_style_lib;
        application->get_canvas(application->get_main_canvas_id())->bgc(ezgl::color(248, 249, 250, 255));
    }
    if (street_style == street_style_lib)//exchange between standard library and self-defined library
        street_style = street_style_night_lib;
    else
        street_style = street_style_lib;

    application->refresh_drawing(); //tell application to redraw

}

//loop through all features and draw with prepicked colors

void draw_features(ezgl::renderer* g) {//draw all features

    double scale = get_zoom_times(g);
    g->set_line_cap(ezgl::line_cap::butt); // Butt ends
    g->set_line_dash(ezgl::line_dash::none); // Solid line
    int numFeature = getNumFeatures();
    for (int i = 0; i < numFeature; i++)//loop all features
    {
        std::vector<ezgl::point2d> points;
        // to store feature size
        double lat_min = global_frame_info.lat_max;
        double lat_max = global_frame_info.lat_min;
        double lon_min = global_frame_info.lon_max;
        double lon_max = global_frame_info.lon_min;
        int numPoints = getFeaturePointCount(i);
        points.reserve(numPoints);
        for (int j = 0; j < numPoints; j++)//loop all points 
        {
            LatLon ll = getFeaturePoint(j, i);
            if (ll.lat() > lat_max)
                lat_max = ll.lat();
            if (ll.lat() < lat_min)
                lat_min = ll.lat();
            if (ll.lon() > lon_max)
                lon_max = ll.lon();
            if (ll.lon() < lon_min)
                lon_min = ll.lon();

            std::pair<double, double> dot = convert_ll_relative_xy(ll.lat(), ll.lon(), global_frame_info); //laton-->xy
            points.push_back(ezgl::point2d(dot.first, dot.second));
        }
        //
        std::pair<LatLon, LatLon> diag(LatLon(lat_min, lon_min), LatLon(lat_max, lon_max));
        double feature_size = find_distance_between_two_points(diag); //calculate diagonal of the feature 
        if (feature_size / global_frame_info.rate_distance_xy * scale < FEATURE_SIZE_LIMIT_TO_DRAW)//only draw if feature is smaller than
        {
            points.clear(); //don't draw the feature
            continue;
        }

        const std::string name = getFeatureName(i);
        FeatureType ftype = getFeatureType(i);
        //        TypedOSMID ID = getFeatureOSMID(i);
        bool closed = (points[0] == points[points.size() - 1]);

        //choose normal colors

        Feature_Style style = feature_style[ftype];
        g->set_color(style.c);

        if (points.size() > 1) {// avoid fill-poly fail
            if (closed) {
                g->fill_poly(points);
            } else {
                for (int j = 0; j < points.size() - 1; j++)
                    g->draw_line(points[j], points[j + 1]); //circle the feature if open
            }
        }
    }
}

void draw_map() {

    Frame_Info frame_info = get_frame_info(); //get frame info of the map
    global_frame_info = frame_info; //local variable to global variable
    ezgl::rectangle initial_world{
        {0, 0}, frame_info.WIDTH, frame_info.HEIGHT
    };

    init_BKtree(); //put all street names into BKtree
    ezgl::application::settings settings;
    settings.main_ui_resource = "./test.ui";
    //settings.main_ui_resource = "//nfs//ug/homes-1//t//tianji21//ece297//work//mapper//main//src//main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";
    ezgl::application application(settings);
    application.add_canvas("MainCanvas", draw_main_canvas, initial_world, ezgl::color(240, 240, 240, 255)); //call draw_main_canvas function when redraw the map each time 
    int res = application.run(initial_setup,
            act_on_mouse_press,
            act_on_mouse_move,
            act_on_key_press);
    res = res;
    return;
}

