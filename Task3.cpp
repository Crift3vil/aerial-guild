#include <gnc_functions.hpp>

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node;

    init_publisher_subscriber(gnc_node);
    
	wait4connect();

	
	wait4start();

	
	initialize_local_frame();

    takeoff(3);

	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = -90;
	waypointList.push_back(nextWayPoint);

	ros::Rate rate(2.0);
	
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		
	}
	return 0;
}
