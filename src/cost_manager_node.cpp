/*
 * CostManager.cpp
 *
 *  Created on: Mar 8, 2013
 *      Author: garzonov
 */

#include <cost_map_manager/cost_manager.h>

class CostManagerNode
{
    public:
	CostManagerNode(tf::TransformListener& tf) : costManager_(tf){}
    private:
	CostManager costManager_;

  };

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "Cost_Manager");

  tf::TransformListener tf(ros::Duration(10));

  CostManagerNode* myCostManager;
  myCostManager = new CostManagerNode(tf);

  // Start Spinning
  ros::spin();

  delete myCostManager;


  return 0;
}
