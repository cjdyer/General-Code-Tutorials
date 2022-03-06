#include "distance.h"

std::ostream& operator<<(std::ostream& os, const Point& p)
{
    os << "X : " << p.x << " Y : " << p.y;
    return os;
}

std::string Point::as_tuple()
{
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

namespace tutorials
{
    double distance_between_points(const Point& p1, const Point& p2)
    {
        Point dist = p1 - p2;
        return sqrt(pow(dist.x, 2) + pow(dist.y, 2)); // h = sqrt(a^2 + b^2)
    }

    PointManager::PointManager() 
    {
        debug("Point Manager : Created");
    }

    PointManager::~PointManager() 
    {
        debug("Point Manager : Ended Gracefully");
    }
    
    void PointManager::set_next_point(const Point p)
    {
        m_current_point = m_next_point;
        m_next_point = p;
        debug("Next point set to " << m_next_point << " Current point set to " << m_current_point);
    }

    // Interpolate between the two points, using a 2D line formula and sampling at given points
    std::vector<Point> PointManager::interpolate(const uint8_t num_output_points) const
    {
        std::vector<Point> output_points;

        Point difference = (m_current_point - m_next_point);
        double gradient = difference.y / difference.x;
        double distance_between_points = difference.x / num_output_points;

        for (size_t i = 0; i < num_output_points; i++)
        {
            double x = i * distance_between_points + m_current_point.x;
            output_points.push_back({x , gradient * (x - m_next_point.x) + m_next_point.y});
        }
        
        output_points.push_back(m_next_point);

        return output_points;
    }

    void PointManager::display_interpolation(std::vector<Point>& points) const
    {
        // For each Point p in points
        for (Point& p : points)
        {
            debug(p);
        }
    }

} // namespace tutorials
