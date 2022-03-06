#include "distance.h"

int main(int argc, char const *argv[])
{
    debug("Main Loop Started");

    Point p1(0, 0);
    Point p2(12, 12);

    { // Start of scope tutorials
        // Declare that this scope is using the tutorials namespace
        using namespace tutorials;

        PointManager pmanager;

        pmanager.set_next_point(p1);
        pmanager.set_next_point(p2);

        debug("distance between " << p1.as_tuple() << " and " << p2.as_tuple() << " is " << distance_between_points(p1, p2));

        std::vector<Point> interpolation_data = pmanager.interpolate(12);
        pmanager.display_interpolation(interpolation_data);
    }; // End of scope tutorials

    // Calling this function will create an error, as 
    // pmanager and interpolation_data are out of scope
    // pmanager.display_interpolation(interpolation_data);

    // However, you can call this function as p1 is in the global scope
    debug(p1);

    debug("Main Loop Ended Gracefully");
    return 0;
}
