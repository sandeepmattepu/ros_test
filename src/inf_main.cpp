#include <ros_test/turtle_abstract.h>

std::shared_ptr<tutorial::AbstractTurtle> turtle;

void draw_u( bool cw, bool use_arc ) {
    int sign;
    if ( cw ) {
        sign = -1;
    } else {
        sign = 1;
    }

    turtle->turn( sign * 45 );
    if(use_arc){
        turtle->arc( sign * 1.5, 180);
        turtle->turn( sign * 45 );
    }else {
        for (int i = 0; i < 3; i++) {
            turtle->forward(3);
            turtle->turn( sign * 90 );
        }
        turtle->turn( sign * -45 );
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inf");

    bool use_arc = false;
    ros::NodeHandle pnh("~");
    pnh.param<bool>("use_arc", use_arc, false);

    turtle = std::make_shared<tutorial::AbstractTurtle>();

    // turtle should draw an infinty sign
    turtle->turn(45);

    turtle->forward( sqrt(4.5) );
    draw_u( true, use_arc );
    turtle->forward( 2 * sqrt(4.5) );
    draw_u( false, use_arc );
    turtle->forward( sqrt(4.5) );

    turtle.reset();

    ROS_INFO_STREAM("Finished");

    return 0;
}
