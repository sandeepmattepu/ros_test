#include <std_srvs/Empty.h>
#include <turtlesim/TeleportAbsolute.h>
#include <ros_test/turtle_abstract.h>

/*
*	BinaryTreeDrawer instance uses AbstractTurtle instance to draw a binary tree based on the options
*	entered in the constructor of the BinaryTreeDrawer. 
**/
class BinaryTreeDrawer{
private:
	float base_line_length;
	float branch_angle;
	int tree_depth;
	int branches;
	float branch_reduction_factor;
	std::shared_ptr<tutorial::AbstractTurtle> turtle;
	
	/*
	 *	After drawing a line turtle returns back to the branch starting point to draw a new branch.
	 *  Here it needs to know how much it has to rotate to start making a new sub-branch.
	 *  This function helps in determining this angle.
	 * 
	 *  @param current_branch Current branch number the turtle wants to draw.
	 *  @param total_branches Total number of branches the turtle needs to draw.
	 *  @return sum of `values`, or 0.0 if `values` is empty.
	 **/
	float determineAngleToRotate(float current_branch);
	
	/*
	 *  Draws the branches based on the provided arguments.
	 * 
	 *  @param line_length Base line length to be drawn.
	 *  @param depth_of_tree Number of branches to be drawn.
	 * */
	void drawBranches(float line_length, int depth_of_tree);
	
public:
	BinaryTreeDrawer(float line_length, float angle_of_branch, int depth_of_tree, int num_branches,
					float current_to_previous_branch, 
					std::shared_ptr<tutorial::AbstractTurtle> turtle_to_use);
	
	/*
	 * This makes the turtle to start drawing the binary tree
	 * */
	void startDrawingTree();
};