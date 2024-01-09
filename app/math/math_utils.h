/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-09 20:10:53
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-09 20:22:54
 */
#include <stdint.h>

#ifndef __MATH_UTILS_H__
#define __MATH_UTILS_H__

#include "vec2f.h"
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265359
#endif

namespace auto_parking_planning
{

    inline float Sqr(const float x) { return x * x; }

    inline float CrossProd(const Vec2f &start_point, const Vec2f &end_point_1, const Vec2f &end_point_2)
    {
        return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
    }

    inline float InnerProd(const Vec2f &start_point, const Vec2f &end_point_1,
                           const Vec2f &end_point_2)
    {
        return (end_point_1 - start_point).InnerProd(end_point_2 - start_point);
    }

    inline float CrossProd(const float x0, const float y0, const float x1,
                           const float y1)
    {
        return x0 * y1 - x1 * y0;
    }

    inline float InnerProd(const float x0, const float y0, const float x1,
                           const float y1)
    {
        return x0 * x1 + y0 * y1;
    }

    inline float WrapAngle(const float angle)
    {

        const float new_angle = std::fmodf(angle, M_PI * 2.0f);
        return new_angle < 0 ? new_angle + M_PI * 2.0f : new_angle;
    }
    inline float NormalizeAngle(const float angle)
    {
        float a = std::fmod(angle + M_PI, 2.0f * M_PI);
        if (a < 0.0f)
        {
            a += (2.0f * M_PI);
        }
        return a - M_PI;
    }

    inline float AngleDiff(const float from, const float to) { return NormalizeAngle(to - from); }

    template <typename T>
    inline T Square(const T value)
    {
        return value * value;
    }
    template <typename T>
    T Clamp(const T value, T bound1, T bound2)
    {
        if (bound1 > bound2)
        {
            std::swap(bound1, bound2);
        }

        if (value < bound1)
        {
            return bound1;
        }
        else if (value > bound2)
        {
            return bound2;
        }
        return value;
    }

    float Gaussian(const float u, const float std, const float x)
    {
        return (1.0f / std::sqrtf(2 * M_PI * std * std)) *
               std::expf(-(x - u) * (x - u) / (2 * std * std));
    }

    inline float Sigmoid(const float x) { return 1.0f / (1.0f + std::expf(-x)); }

    inline std::pair<double, double> RFUToFLU(const double x, const double y)
    {
        return std::make_pair(y, -x);
    }

    inline std::pair<float, float> FLUToRFU(const float x, const float y)
    {
        return std::make_pair(-y, x);
    }

    inline void L2Norm(int feat_dim, float *feat_data)
    {
        if (feat_dim == 0)
        {
            return;
        }
        // feature normalization
        float l2norm = 0.0f;
        for (int i = 0; i < feat_dim; ++i)
        {
            l2norm += feat_data[i] * feat_data[i];
        }
        if (l2norm == 0)
        {
            float val = 1.f / std::sqrt(static_cast<float>(feat_dim));
            for (int i = 0; i < feat_dim; ++i)
            {
                feat_data[i] = val;
            }
        }
        else
        {
            l2norm = std::sqrt(l2norm);
            for (int i = 0; i < feat_dim; ++i)
            {
                feat_data[i] /= l2norm;
            }
        }
    }

    std::pair<float, float> Cartesian2Polar(float x, float y)
    {
        float r = std::sqrt(x * x + y * y);
        float theta = std::atan2(y, x);
        return std::make_pair(r, theta);
    }

    template <class T>
    typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
    almost_equal(T x, T y, int ulp)
    {

        return std::fabs(x - y) <=
                   std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp ||
               std::fabs(x - y) < std::numeric_limits<T>::min();
    }

}

#endif /* __MATH_UTILS_H__ */
