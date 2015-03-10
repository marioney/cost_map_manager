/*
 * CostManager.cpp
 *
 *  Created on: Mar 8, 2013
 *      Author: garzonov
 */

#include <cost_map_manager/cost_manager.h>

CostManager::CostManager(tf::TransformListener& tf) :
	map_initialized_(false)
{
	// TODO Auto-generated constructor stub

	ros::NodeHandle private_nh("~");

	//get our tf prefix
	ros::NodeHandle prefix_nh;
	tf_prefix_ = tf::getPrefixParam(prefix_nh);




    // variables for the global map parameters.
    double map_origin_x;
    double map_origin_y;
    double publishRate;

    std::string map_topic;
    std::string global_map_topic;

    private_nh.param("map_topic", map_topic, std::string("/map"));
    private_nh.param("global_map_topic", global_map_topic, std::string("/Global_Cost"));
    private_nh.param("publish_rate", publishRate, 0.1);
    private_nh.param("global_map_resolution", global_map_resolution_, 0.5);
    private_nh.param("origin_x", map_origin_x, 0.0);
    private_nh.param("origin_y", map_origin_y, 0.0);
    private_nh.param("use_max_occupancy", use_max_value_, true);
    private_nh.param("max_allowed_distance", max_allowed_distance_, 2.0);
	private_nh.param("global_frame", global_frame_, std::string("/map"));



    //we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Cost Manager: Requesting the map");
    mapSubscriber = private_nh.subscribe(map_topic, 1, &CostManager::mapCallback, this);

    ros::Rate r(1.0);

    //Since a Base Map is needed, This cycle locks the program until the map has been correctly initialized
    while(!map_initialized_ && ros::ok())
    {
    	ros::spinOnce();
    	ROS_INFO("Still waiting on map...\n");
    	r.sleep();
    }
    ROS_INFO("Received a %d X %d map at %f m/pix",
    		map_meta_data_.width, map_meta_data_.height, map_meta_data_.resolution);

    r.sleep();
    ROS_INFO("Start computing distance transform");

    distanceTransform(GlobalMap_);


    // The Global map, is published.
    GlobalMapPub = private_nh.advertise<nav_msgs::OccupancyGrid>(global_map_topic, 1, true);

    //Start loop
    loopTimer = private_nh.createTimer(ros::Duration(1.0/publishRate), &CostManager::spin, this);

}

CostManager::~CostManager() {
	// TODO Auto-generated destructor stub
}

void CostManager::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
	 if(!map_initialized_)
	 {
	     initFromBaseMap(*map_msg);
	     map_initialized_ = true;
	     ROS_INFO("Cost Manager: Map Initialized OK.");
	 }/*
	    else if(costmap_initialized_)
	      updateStaticMap(*new_map);*/
}

void CostManager::spin(const ros::TimerEvent& tEvent)
{
	 //ROS_INFO("Cost Manager: In Spin");

	 GlobalMapPub.publish(GlobalMap_);

}

void CostManager::initFromBaseMap(const nav_msgs::OccupancyGrid& map)
{
	initFromBaseMap(map, GlobalMap_, global_map_resolution_);
	global_map_height = GlobalMap_.info.height;
	global_map_width = GlobalMap_.info.width;
}

void CostManager::initFromBaseMap(const nav_msgs::OccupancyGrid& input_map, nav_msgs::OccupancyGrid& output_map, const double new_resolution)
{

    double res_ratio; // Ratio between the Original map resolution and the global one.
    int new_map_height;
    int new_map_width;

	boost::recursive_mutex::scoped_lock lock(map_data_lock_); // Locks the data to avoid any strange behavior.

	// First the resolution ratio is calculated.
	res_ratio = new_resolution / input_map.info.resolution; //first find the ratio, hopefully will be an integer.

	// we calculate the new  width and height using the given resolution ratio.

	new_map_width  = (unsigned int)(input_map.info.width / res_ratio);
	new_map_height = (unsigned int)(input_map.info.height / res_ratio);

	// Create the output map (GlobalMap_) Its empty for now.
	// uses similar info as the original map.
	output_map.header.stamp = ros::Time::now();
	output_map.header.frame_id = global_frame_;
	output_map.info = input_map.info;
	output_map.info.width = new_map_width;
	output_map.info.height = new_map_height;
	output_map.info.resolution = new_resolution;
	output_map.data.clear();
	output_map.data.resize(new_map_width * new_map_height);


    if (res_ratio == 1) //if its the same resolution just copy the fields.
    {
    	output_map.data = input_map.data;
    }
    else // If the resolution is different it uses a bilinear interpolation to fill the new map
    {
    	// For each data in the new map.
    	for(unsigned int h = 0; h <  output_map.info.height; h++)
    	{

    		double scaledH = h*res_ratio; 	// finds the Scaled  height
    		// The two top corners for the linear interpolation are found
			int hl = floor(scaledH + 0.000001);
			int hr = floor(scaledH + 0.000001);
			if(res_ratio > 1)
				hr = hr + floor(res_ratio + 0.000001) - 1;
			else
				hr = hr + 1;

			//ROS_INFO("h: %d, ScaledH: %f, hl: %d, hr: %d", h, scaledH, hl, hr);

    		for(unsigned int w = 0; w <  output_map.info.width; w++)
    		{
    		//	double scaledH = h*res_ratio;
    			double scaledW = w*res_ratio; // finds the Scaled  width

    			// The four corners for the linear interpolation are found

    			int wl = floor(scaledW + 0.000001);
    			int wr = floor(scaledW + 0.000001);
    			if(res_ratio > 1)
    				wr = wr +  floor(res_ratio + 0.000001) - 1;
    			else
    				wr = wr + 1;
    			// The value for each one of the corners is also found
    			int y_hl_wl = getMapValue(hl,wl, input_map);	// Top Left value.
    			int y_hl_wr = getMapValue(hl,wr, input_map);	// Top Right value.
    			int y_hr_wl = getMapValue(hr,wl, input_map);	// Bottom Left value.
    			int y_hr_wr = getMapValue(hr,wr, input_map);	// Bottom Right value.

    			// If is set to use linear int instead of maximum value
    			if (!use_max_value_)
    			{
    				//ROS_INFO("Cost Manager:Using Bilinear Interpolation, Ratio: %f", res_ratio);
    				int val_hl = linearInt(y_hl_wl, y_hl_wr, wl, wr, scaledW);	// Finds the linear interpolation of the top row.
    				int val_hr = linearInt(y_hr_wl, y_hr_wr, wl, wr, scaledW);	// Finds the linear interpolation of the bottom row.
    				int new_val = linearInt(val_hl, val_hr, hl, hr, scaledH);	// Finds the interpolation of the two previously found points, which is the desired value.

    				//ROS_INFO("hl: %d, y: %d, hr: %d, y: %d, new W: %f - Res: %d",hl,y_hl_wl,hr,y_hl_wr,scaledW,val_hl);
    				//ROS_INFO("wl: %d, y: %d, wr: %d, y: %d, new W: %f - Res: %d",wl,y_hr_wl,wr,y_hr_wr,scaledW,val_hr);
    				//ROS_INFO(" New H: %f, New Val %d",scaledH, new_val);

    				setMapValue(h,w,new_val,output_map);	// Sets the new value for the map.
    				//if(new_val != -1)
    				//	getchar();
    			}
    			// Uses the maximum value as the new value for the cell
    			else
    			{
    				//ROS_INFO("Cost Manager:Using Max Value to init");
    				int new_val = UNKNOWN_CELL;
    				for (int top_down = hl; top_down <= hr; top_down ++)
    				{
    					for (int right_left = wl; right_left <= wr; right_left ++)
    					{
    						int temp_value;
    						temp_value = getMapValue(top_down,right_left, input_map);

    						if (temp_value == UNKNOWN_CELL)
    						{
    							temp_value = OCCUPIED_CELL;
    						}

    						//ROS_INFO("Pos H: %d, Pos W: %d, new_val :%d, temp_val: %d",top_down,right_left,new_val,temp_value);
    						if ( temp_value > new_val)
    						{
    							new_val = temp_value;
    						}
    						setMapValue(h,w,new_val,output_map);	// Sets the new value for the map.
    					}

    				}

    			}
    		}
    	}
    }
    map_meta_data_ = input_map.info; // Copies the map info to a global variable (just to print a message)

}


int CostManager::linearInt(int y1, int y2, int x1, int x2, int x)
{
	if(x1 == x2)	// if the x value is the same then there is no need for interpolation.
	{
		return y1;	// Return the same value, since y1 is equal to y2.
	}


	int inc = ((y2-y1)/(x2 - x1)) * (x - x1);	// Defines the increment according to the slope and the distance.

	double val = y1 + inc;		// The value is the addition of the previous value and the increment.
	return val;					// Returns the interpolated value.
}

int CostManager::getMapValue(int height, int width, const nav_msgs::OccupancyGrid& map)
{
	int position = map.info.width * height + width; // Finds the position in the data vector.
	return map.data[position];						// Gets the value from the position
}

int CostManager::getMapValue(int height, int width)
{
	int position = GlobalMap_.info.width * height + width;	// Finds the position in the data vector.
	return GlobalMap_.data[position];						// Gets the value from the position
}
float CostManager::getMapValue(int height, int width, const float *temp_map, int map_width)
{
	int position = map_width * height + width;
	return temp_map[position];
}
void CostManager::setMapValue(int height, int width, int Val, nav_msgs::OccupancyGrid& map)
{
	int position = map.info.width * height + width; // Finds the position in the data vector.
	map.data[position] = Val;						// Sets the value at that position
}
void CostManager::setMapValue(int height, int width, int Val)
{
	int position = GlobalMap_.info.width * height + width;	// Finds the position in the data vector.
	GlobalMap_.data[position] = Val;						// Sets the value at that position
}
void CostManager::setMapValue(int height, int width, float Val, float *temp_map, int map_width)
{
	int position = map_width * height + width; // Finds the position in the data vector.
	temp_map[position] = Val;					// Sets the value at that position
}


float * CostManager::distanceTransform1D(float *f, int grid_size)
{
	float s; 				// Interception point of the parabolas;
	float *distances_vec; 	// Vector of distances to be returned
	int *v; 				// Locations of parabolas in lower envelopes
	float *z;				//Locations of boundaries between parabolas
	int k;					// Index of rightmost parabola lower envelope

	distances_vec = new float[grid_size];
	v = new int[grid_size];
	z = new float[grid_size+1];
	k = 0;

	v[0] = 0;
	z[0] = -INF;
	z[1] = +INF;

	for (int q = 1; q <= grid_size-1; q++) // Compute the lower envelope
	{
		s  = ((f[q]+pow(q,2))-(f[v[k]]+pow(v[k],2)))/(2*q-2*v[k]);  // Find the first interception point
	    while (s <= z[k])
	    {
	    	k--;
	    	s  = ((f[q]+pow(q,2))-(f[v[k]]+pow(v[k],2)))/(2*q-2*v[k]);
	    }
	    k++;
	    v[k] = q;
	    z[k] = s;
	    z[k+1] = +INF;
	}

	k = 0;
	for (int q = 0; q <= grid_size-1; q++)
	{
		while (z[k+1] < q)
		{
			k++;
		}
		distances_vec[q] = pow(q-v[k],2) + f[v[k]];
	}
	delete [] v;
	delete [] z;
	return distances_vec;
}
void  CostManager::distanceTransform(nav_msgs::OccupancyGrid& map)
{
	ROS_INFO("Start Transform");
	int width = map.info.width;
	int height = map.info.height;


	float *f_width = new float[width];
	float *f_height = new float[height];

	float *temp_map;

	temp_map = preProcessMap(map);


	// transform along columns
	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			f_height[y] = getMapValue(y,x,temp_map,map.info.width);//(im, x, y);  //Gets the value in the corresponding position
		}
		float *d = distanceTransform1D(f_height, height);
		for (int y = 0; y < height; y++)
		{
			setMapValue(y,x,d[y],temp_map, width);//imRef(im, x, y) = d[y];
		}
		delete [] d;
	}
	ROS_INFO("Vertical pass Map Values: \n");
	printMap(temp_map,width,height);
	// transform along rows
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			f_width[x] = getMapValue(y,x,temp_map,map.info.width);//imRef(im, x, y);
		}
		float *d = distanceTransform1D(f_width, width);
		for (int x = 0; x < width; x++)
		{
			setMapValue(y,x,d[x],temp_map, width);//imRef(im, x, y) = d[x];
		}
		delete [] d;
	}
	//Set the occupancy grid with the new values.
	postProcessMap(temp_map,width,height,map);


	delete f_width;
	delete f_height;
	delete temp_map;
	ROS_INFO("Map transformed");
}

void  CostManager::invertMapValues(nav_msgs::OccupancyGrid& map)
{
	for (unsigned int y = 0; y < map.info.height; y++)
	{
		for (unsigned int x = 0; x < map.info.width; x++)
		{
			if (getMapValue(y,x,map) == OCCUPIED_CELL || getMapValue(y,x,map) == UNKNOWN_CELL )
				setMapValue(y,x,FREE_CELL,map);
			else
				setMapValue(y,x,OCCUPIED_CELL,map);
		}
	}
}
float * CostManager::preProcessMap(const nav_msgs::OccupancyGrid& map)
{
	float *temp_map;

	temp_map = new float[map.data.size()];
	for (unsigned int y = 0; y < map.info.height; y++)
	{
		for (unsigned int x = 0; x < map.info.width; x++)
		{
			if (getMapValue(y,x,map) == OCCUPIED_CELL || getMapValue(y,x,map) == UNKNOWN_CELL )
			{
				setMapValue(y,x,0,temp_map, map.info.width);
			}
			else
			{
				setMapValue(y,x,pow(max_allowed_distance_/global_map_resolution_,2),temp_map, map.info.width);
			}
		}
	}
	return temp_map;
}
void CostManager::postProcessMap(const float * temp_map, unsigned int map_width,
		unsigned int map_height, nav_msgs::OccupancyGrid& map)
{
	//First clear all the previous data in the map.
	map.data.clear();
	//Then resize the map to the new values
	map.data.resize(map_width * map_height);

	for (unsigned int y = 0; y < map_height; y++)
	{
		for (unsigned int x = 0; x < map_width ; x++)
		{
			float temp_value;
			temp_value = sqrt(getMapValue(y,x,temp_map,map.info.width));
			temp_value *= global_map_resolution_;
			temp_value *= OCCUPIED_CELL/max_allowed_distance_;
			if(temp_value > OCCUPIED_CELL )
			{
				temp_value = OCCUPIED_CELL;
			}
			setMapValue(y,x,(int)(OCCUPIED_CELL - temp_value));
		}
	}

}
void CostManager::printMap(const nav_msgs::OccupancyGrid& map)
{
	for (unsigned int y = 0; y < map.info.height; y++)
	{
		for (unsigned int x = 0; x < map.info.width; x++)
		{
			int temp_value;
			temp_value = getMapValue(y,x,map);
			if(temp_value > 100)
				temp_value = 100;
			printf(" %3d",(int)(temp_value));
		}
		printf(" \n");
	}


}
void CostManager::printMap(const float * map, unsigned int map_width, unsigned int map_height)
{
	for (unsigned int y = 0; y < map_height; y++)
	{
		for (unsigned int x = 0; x < map_width; x++)
		{
			float temp_value;
			temp_value = getMapValue(y,x,map,map_width);
			if(temp_value > 999)
				temp_value = 999;
			printf(" %3.1f",float(temp_value));
		}
		printf(" \n");
	}
}

