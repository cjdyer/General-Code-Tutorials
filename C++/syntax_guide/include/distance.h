#ifndef DISTANCE_H
#define DISTANCE_H

#include <iostream>
#include <math.h>
#include <vector>

// Sneaky way of using std::ostream, without as much typing
#define debug(x) std::cout << x << std::endl;

// A simple point class, containing an x and y position
struct Point
{
    double x, y;

    // Use of default values, when constructing
    Point(double x = 0, double y = 0) : x(x), y(y) { }

    // Creates a tuple from a Point 
    std::string as_tuple();

    // Operator overloading to get the difference between two points in x and y respectively
    Point operator- (const Point& a) const
    {
        return Point(a.x - x, a.y - y);
    }

    // The friend operator creates a non-templated version of ostreams << operator
    // In simple terms ostream has a function called '<<'
    // We are then telling the complier that we have our own version of that function
    friend std::ostream& operator<< (std::ostream& os, const Point& p);
}; // struct Point

namespace tutorials
{
    double distance_between_points(const Point& p1, const Point& p2);

    class PointManager
    {
    public:
        PointManager();
        ~PointManager();

        // Sets the next point, and updates the current point
        void set_next_point(const Point p);
        // Interpolates between tthe wo points
        std::vector<Point> interpolate(const uint8_t num_output_points) const;
        // Display given interpolation data
        void display_interpolation(std::vector<Point>& points) const;

    private:
        Point m_next_point;
        Point m_current_point;
    }; // class PointManager
} // namespace tutorials

#endif // DISTANCE_H