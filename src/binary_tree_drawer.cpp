#include<ros_test/binary_tree_drawer.h>

BinaryTreeDrawer::BinaryTreeDrawer(float line_length, float angle_of_branch, int depth_of_tree, 
									int num_branches, float current_to_previous_branch,
									std::shared_ptr<tutorial::AbstractTurtle> turtle_to_use)
{
	base_line_length = line_length;
	branch_angle = angle_of_branch;
	tree_depth = depth_of_tree;
	branches = num_branches;
	branch_reduction_factor = current_to_previous_branch;
	turtle = turtle_to_use;
}

float BinaryTreeDrawer::determineAngleToRotate(float current_branch)
{
	float angle_to_turn = 0;
	// Turtle is just about to draw first branch. It's angle to rotate will be different with others
	if(current_branch == 1)
	{
		angle_to_turn = -(branch_angle / 2);
	}
	// Angle needed to rotate after it has returned to branch from previous sub-branch
	else
	{
		// 3 branches divides a region into 2 regions.
		float angle_between_regions = (branch_angle / (branches - 1));
		angle_to_turn = -(180 - angle_between_regions);
	}
}

void BinaryTreeDrawer::startDrawingTree()
{
	// Sanity checks
	if(base_line_length <= 0)
	{
		ROS_ERROR("Line length cannot be less than 0");
		return;
	}
	else if(branch_angle >= 180 || branch_angle <= 0)
	{
		ROS_ERROR("Angle of branch should be in range of 0 to 180 degrees.");
		return;
	}
	else if(tree_depth <= 0)
	{
		ROS_ERROR("Depth of tree should be atleast 1");
		return;
	}
	else if(branch_reduction_factor <= 0 || branch_reduction_factor > 1)
	{
		ROS_ERROR("Current to previous branch ratio should lie between 0(exclusive) and 1(inclusive)");
		return;
	}
	else if(branches < 2)
	{
		ROS_ERROR("Number of branches should be atleast 2");
		return;
	}
	else
	{
		drawBranches(base_line_length, tree_depth);
	}
}

void BinaryTreeDrawer::drawBranches(float line_length, int depth_of_tree)
{
	double actual_distance_travelled = 0.0;
	if(turtle->collision_aware_forward(line_length, actual_distance_travelled))
	{
		// Start drawing branches after drawing base line
		for(int current_branch = 1; current_branch <= branches; current_branch++)
		{
			float angle_to_turn = determineAngleToRotate(current_branch);
			turtle->turn(angle_to_turn);
		
			// If depth exists then draw other sub-branch else draw final line(leaf)
			float new_line_length = line_length * branch_reduction_factor;
			if(depth_of_tree > 1)
			{
				drawBranches(new_line_length, (depth_of_tree - 1));		// Recursive to draw sub-branch
			}
			else
			{
				turtle->forward(new_line_length);
				// Return back to parent branch
				turtle->turn(180);
				turtle->forward(new_line_length);
			}
		}
	
		// After drawing the branches return to tip of base line
		float angle_to_turn = determineAngleToRotate(1);		// produces angle aligned with base line
		turtle->turn(angle_to_turn);
		turtle->forward(actual_distance_travelled);
	}
	else
	{
		ROS_WARN_STREAM("Current branch is skipped due to turtle being very closer to collision");
		turtle->turn(180);
		turtle->forward(actual_distance_travelled);
	}
} 