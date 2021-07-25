#ifndef LIDAR_PERCEPTION_SRC_UTILS_BBOX_H_
#define LIDAR_PERCEPTION_SRC_UTILS_BBOX_H_

#include <array>

struct PointType2f
{
     float x;
     float y;

     PointType2f(){}
     PointType2f(float x_, float y_)
     :x(x_), y(y_){}

     PointType2f operator-(const PointType2f& other) const
     {
          return PointType2f(x - other.x, y - other.y);
     }
};

float DotProduct(const PointType2f&u, const PointType2f&v)
{
     return u.x * v.x + u.y * v.y;
}



struct BBox
{
     float x;
     float y;
     float angle;
     float extend_x;
     float extend_y;

     BBox(){}

     BBox(const float x_, const float y_, const float angle_, const float extend_x_, const float extend_y_)
     :x(x_), y(y_), angle(Modulus2PI(angle_)), extend_x(extend_x_), extend_y(extend_y_){}

     BBox(const PointType2f& pin_point, const float angle_, const PointType2f& diag_point)
     :x(pin_point.x), y(pin_point.y), angle(Modulus2PI(angle_))
     {
          float cos_angle = std::cos(angle_), sin_angle = std::sin(angle_);
          PointType2f unit_x(cos_angle, sin_angle);
          PointType2f unit_y(-sin_angle, cos_angle);
          PointType2f vec = diag_point - pin_point;
          extend_x = DotProduct(vec, unit_x);
          extend_y = DotProduct(vec, unit_y);
     }

     BBox(const PointType2f &center, const float angle_, const float length_, const float width_)
     :x(center.x), y(center.y), angle(Modulus2PI(angle_)), extend_x(length_), extend_y(width_)
     {
          const auto diag_point = getCenter();
          x = 2 * center.x - diag_point.x;
          y = 2 * center.y - diag_point.y;
     }

     inline PointType2f getPinPoint() const
     {
          return PointType2f{x,y};
     }

     inline PointType2f getCenter() const
     {
          return PointType2f{x + 0.5f * (extend_x * std::cos(angle) - extend_y * std::sin(angle)),
                             y + 0.5f * (extend_x * std::sin(angle) + extend_y * std::cos(angle))};
     }

     inline PointType2f getXAxisPoint() const
     {
          return PointType2f {x + extend_x * std::cos(angle), y + extend_x * std::sin(angle)};  
     }

     inline PointType2f getYAxisPoint() const
     {
          return  PointType2f{x - extend_y * std::sin(angle), y + extend_y * std::cos(angle)};
     }

     inline PointType2f getDiagonalPoint() const
     {
          return PointType2f{x + extend_x * std::cos(angle) - extend_y * std::sin(angle),
                             y + extend_x * std::sin(angle) + extend_y * std::cos(angle)};
     }

     inline std::array<PointType2f, 4> getRectPoints() const
     {
          return std::array<PointType2f, 4>{ getPinPoint(),
                                             getXAxisPoint(),
                                             getDiagonalPoint(),
                                             getYAxisPoint()};
     }

     inline std::array<PointType2f, 4> getRectPointsHorizontal() const
     {
          return getRectPoints();
     }

     inline std::array<PointType2f, 4> getRectPointsVertical() const
     {
          return std::array<PointType2f, 4>{ getPinPoint(),
                                             getYAxisPoint(),
                                             getDiagonalPoint(),
                                             getXAxisPoint()};
     }

     inline float getArea() const
     {
          return std::abs(extend_x * extend_y);
     }

     inline float getLength() const
     {
          return std::abs(extend_x);
     }

     inline float getWidth() const
     {
          return std::abs(extend_y);
     }

     static float Modulus2PI(float angle){return angle + 2.0 * M_PI * std::floor((M_PI - angle)/(2.0 * M_PI));}

};


#endif // LIDAR_PERCEPTION_SRC_UTILS_BBOX_H_