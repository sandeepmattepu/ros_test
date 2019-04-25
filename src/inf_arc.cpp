#include <ros_test/turtle_abstract.h>

std::shared_ptr<tutorial::AbstractTurtle> turtle;

void draw_u( bool cw ) {
    int sign;
    if ( cw ) {
        sign = -1;
    } else {
        sign = 1;
    }

    turtle->turn( sign * 45 );
    turtle->arc( sign * 1.5, 180);
    turtle->turn( sign * 45 );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inf_arc");

    bool use_arc = true;
    ros::NodeHandle pnh("~");

    turtle = std::make_shared<tutorial::AbstractTurtle>();

    // turtle should draw an infinty sign
    turtle->turn(45);

    turtle->forward( sqrt(4.5) );
    draw_u( true );
    turtle->forward( 2 * sqrt(4.5) );
    draw_u( false );
    turtle->forward( sqrt(4.5) );

    turtle.reset();

    ROS_INFO_STREAM("Finished");

    return 0;
}
