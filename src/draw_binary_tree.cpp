#include<ros_test/binary_tree_drawer.h>

void setTurtleAtBottomCenter(ros::NodeHandle& nh);

int main(int argc, char** argv)
{
	float base_length = 0.0;
	float branch_angle = 0.0;
	float factor = 0.0;
	int depth = 0;
	int branches = 0;
	
	ros::init(argc, argv, "binary_tree");
	ros::NodeHandle nh;
	nh.param<float>("/turtle_controll/length", base_length, 2);
	nh.param<float>("/turtle_controll/angle", branch_angle, 60);
	nh.param<float>("/turtle_controll/factor", factor, 0.8);
	nh.param<int>("/turtle_controll/depth", depth, 2);
	nh.param<int>("/turtle_controll/branches", branches, 2);
	
	// Set the position of turtle at bottom center
	setTurtleAtBottomCenter(nh);
	
	std::shared_ptr<tutorial::AbstractTurtle> turtle;
	turtle = std::make_shared<tutorial::AbstractTurtle>();
	std::shared_ptr<BinaryTreeDrawer> tree;
	tree = std::make_shared<BinaryTreeDrawer>(base_length, branch_angle, depth, branches, factor, turtle);
	tree->startDrawingTree();
	
	ROS_INFO_STREAM("Finished");
}

void setTurtleAtBottomCenter(ros::NodeHandle& nh)
{
	// Teleport the turtle
	ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>
																("turtle1/teleport_absolute");
	turtlesim::TeleportAbsolute teleSrv;
	teleSrv.request.x = 5.0;
	teleSrv.request.y = 0.75;
	teleSrv.request.theta = 1.56799995899;
	
	ros::service::waitForService("turtle1/teleport_absolute", -1);
	teleportClient.call(teleSrv);
	
	ros::Duration(0.1).sleep();		// turtlesim doesn't clear the sim sometimes. So it's a fix.
	
	// Clear the sim
	ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("clear");
	std_srvs::Empty clearSrv;
	
	ros::service::waitForService("clear", -1);
	clearClient.call(clearSrv);
}