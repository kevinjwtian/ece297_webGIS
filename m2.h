/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   m2.h
 * Author: Zixin Shen
 *
 * Created on February 22, 2020, 5:26 PM
 */
#include <iostream>
#include <string>
#include "OSMDatabaseAPI.h"
#include "m1.h"
#include "gtk/gtk.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "ezgl/camera.hpp"
#include "StreetsDatabaseAPI.h"
#include <curl/curl.h>

#ifndef M2_H
#define M2_H

#define BASE_SIZE 1000



struct Street_Style {
    ezgl::color c;
};
struct Frame_Info
{
    double lat_min;
    double lat_max;
    double lon_min;
    double lon_max;
    double lat_avg;
    double x_min;	//also base x
    double x_max;
    double y_min;	//also base y
    double y_max;
    double x_diff;
    double y_diff;
    double HEIGHT;
    double WIDTH;
    double param_cos_avg_lat;
    double rate_distance_xy;    // conversion rate between real distance and world coordinate   
};

size_t process_data(void* buffer, size_t size, size_t nmemb, void* userp);
void curl_traffic_live(GtkWidget* widget, ezgl::application* application);
void act_on_key_press(ezgl::application* application, GdkEventKey *event, char* key_name);
void act_on_mouse_move(ezgl::application* application, GdkEventButton *event, double x, double y);
void act_on_mouse_press(ezgl::application* application, GdkEventButton *event, double x, double y);
void output_inter_info(std::vector<int> inters);//output all intersections information
void get_bbox(double& lat_min, double& lon_min, double& lat_max, double& lon_max);
std::pair<double, double> init_convert_ll_xy(double lat, double lon, Frame_Info& frame_info);
std::pair<double, double> convert_ll_relative_xy(double lat, double lon, Frame_Info& frame_info);
LatLon convert_relative_xy_ll(double x, double y, Frame_Info& frame_info);
Frame_Info get_frame_info();
void draw_main_canvas(ezgl::renderer *g);
void draw_streets(ezgl::renderer *g);
void draw_POI_names(ezgl::renderer * g);
void draw_high_lights(ezgl::renderer* g);
void draw_street_segment(ezgl::renderer* g, IntersectionIndex inter_idx,Street_Style style, Street_Style style2);

double get_zoom_times(ezgl::renderer* g);

void night_mode_button(GtkWidget* window, ezgl::application * application);
void init_BKtree();
std::vector<int> find_inters(GtkWidget* local_entry_1, GtkWidget* local_entry_2);
void find_inter_from_streets(GtkWidget* window, ezgl::application * application);
void on_find_dialog_response(GtkDialog* dialog, gint response_id, gpointer user_data);//which dialog sent the message, what message, 
void show_message(GObject* parent, const gchar* message);
void on_entry_changed(GtkEntry* entry,gpointer  user_data);
void choose_map(GtkWidget* widget, ezgl::application* application);
void initial_setup(ezgl::application* application, bool new_window);
void draw_features(ezgl::renderer* g) ;
void draw_map();
//m3 functions
void pool(GtkWidget* window, ezgl::application * application);
void express(GtkWidget* window, ezgl::application * application);
void find_route();
void output_travel_details();
void draw_path(ezgl::renderer *g);
void find_path(GtkWidget* window, ezgl::application * application);
void show_help(GtkWidget* window, ezgl::application * application);
void path_mode_change(GtkWidget* window, ezgl::application * application);
void on_find_path_response(GtkDialog* dialog, gint response_id, gpointer user_data);
bool get_path_intersections(std::pair<int,int>& start_end_inters);
void get_walk_parameters();
#endif /* M2_H */

