/*
 * CostManager.h
 *
 *  Created on: Mar 8, 2013
 *      Author: garzonov
 */

#ifndef COSTMANAGER_H_
#define COSTMANAGER_H_

//#include <algorithm>
#include <cstring>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

#define INF 1E20
#define UNKNOWN_CELL -1
#define FREE_CELL 0
#define OCCUPIED_CELL 100

  /**
   * @class CostManager
   * @brief Creates a single Cost Map from occupancy and cost maps and
   * a set of parameters to handle their weights or the interactions.
   */
class CostManager
{
	public:
	/**
	 * @brief  Constructor for the Class
	 * @param tf A reference to a TransformListener
	 */
	CostManager(tf::TransformListener& tf);
	/**
	 * @brief Destructor for the Class
	 */
	virtual ~CostManager();
	/**
	 * @brief Callback to handle the map messages received.
	 * @param mapMsg Pointer to the OccupancyGrid message
	 */
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
	/**
	 * @brief Main loop, computes and publishes the global cost map.
	 * @param tEvent Pointer to the timer event that controls execution.
	 */
	void spin(const ros::TimerEvent& tEvent);

	/**
	 * @brief Init Map from base, creates a new map based on the input and with the new resolution.
	 * @param input_map the map taken as base for the creation of the new one.
	 * @param output_map the resulting map with a new resolution.
	 * @param new_resolution the desired resolution for the new map.
	 */
	void initFromBaseMap(const nav_msgs::OccupancyGrid& map);
	void initFromBaseMap(const nav_msgs::OccupancyGrid& input_map, nav_msgs::OccupancyGrid& output_map, const double new_resolution);

	/**
	 * @brief Linear interpolation, for a given x between two points (x1,y1) and (x2,y1)
	 * @param y1 y = f(x) value in the point x1
	 * @param y2 y = f(x) value in the point x2
	 * @param x1 point x2
	 * @param x2 point x2
	 * @param x x position for the desired new value.
	 * @return y interpolated value for the desired x position.
	 */
	int linearInt(int y1, int y2, int x1, int x2, int x);


	/**
	 * @brief gets the value of the occupancy grid in the given position.
	 * @param height the height or y position of the map
	 * @param width the width or x position of the map
	 * @map if a map is given, it returns the value of that map, otherwise returns the value of the global map.
	 * @return i Map value in the given position.
	 */
	int getMapValue(int height, int width);
	int getMapValue(int height, int width, const nav_msgs::OccupancyGrid& map);
	float getMapValue(int height, int width, const float *temp_map, int map_width);
	/**
	 * @brief sets the value of the occupancy grid in the given position.
	 * @param height the height or y position of the map
	 * @param width the width or x position of the map
	 * @map if a map is given, it sets the value of that map, otherwise sets the value of the global map.
	 */
	void setMapValue(int height, int width, int Val);
	void setMapValue(int height, int width, int Val, nav_msgs::OccupancyGrid& map);
	void setMapValue(int height, int width, float Val, float *temp_map, int map_width);

	/**
	 * @brief applies the distance transform to a map.
	 * @param map the map to be processed.
	 */
	void distanceTransform(nav_msgs::OccupancyGrid& map);
	/**
	 * @brief Computes the distance transform algorithm for the squared distance in one-dimension
	 * @param f The vector of points in the one dimensional grid.
	 * @param grid_size The size of the grid to be processed
	 */

	float *distanceTransform1D(float *f, int grid_size);
	/**
	 * @brief inverts the values of the map, frees occupied or unknown values.
	 * @param map the map to be processed.
	 */
	void  invertMapValues(nav_msgs::OccupancyGrid& map);
	/**
	 * @brieg Converts the OccupancyGrip to a vector of floats.
	 * @param map the map that is being converted
	 * @return pointer to the vector of floats containing the map data.
	 */
	float *preProcessMap(const nav_msgs::OccupancyGrid& map);

	/**
	 * @brieg Converts a vector of floats to an OccupancyGrip.
	 * @param temp_map the vector of floats containing the map data
	 * @param map_width the width of the map
	 * @param map_height the height of the map
	 * @param map the map in which the data is stored.
	 */
	void postProcessMap(const float * temp_map, unsigned int map_width,
			unsigned int map_height, nav_msgs::OccupancyGrid& map);

	/**
	 * @brieg Prints the map info to the screen.
	 * @param map the map to be printed
	 */
	void printMap(const nav_msgs::OccupancyGrid& map);
	void printMap(const float * map, unsigned int map_width, unsigned int map_height);
	/**
	 * ROS Publishers and subscribers.
	 */

	ros::Subscriber mapSubscriber; // Subscriber for the map topic (OccupancyGrid);
	ros::Publisher GlobalMapPub;

	nav_msgs::OccupancyGrid GlobalMap_;

	//we need this to be able to initialize the map using a latched topic approach
	//strictly speaking, we don't need the lock, but since this all happens on startup
	//and there is little overhead... we'll be careful and use it anyways just in case
	//the compiler or scheduler does something weird with the code
	boost::recursive_mutex map_data_lock_;
	nav_msgs::MapMetaData map_meta_data_;

	bool map_initialized_; 		// Controls the initialization of the map.
	bool use_max_value_; // Uses the maximum value instead of performing linear interpolation.

	unsigned int global_map_width;
	unsigned int global_map_height;

	ros::Timer loopTimer;


	/**
	 * Variables used to read and store the ROS parameters
	 */
	std::string map_type;
	std::string global_frame_;
	std::string robot_base_frame_;
	std::string tf_prefix_;
	double global_map_resolution_;
	double max_allowed_distance_;


};

#endif /* COSTMANAGER_H_ */
