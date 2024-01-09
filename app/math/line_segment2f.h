/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-09 20:07:43
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-09 21:53:33
 */
#include <stdint.h>

#ifndef __LINE_SEGMENT2F_H__
#define __LINE_SEGMENT2F_H__

#include "vec2f.h"
#include "math_utils.h"
namespace auto_parking_planning
{

    bool IsWithin(float val, float bound1, float bound2)
    {
        if (bound1 > bound2)
        {
            std::swap(bound1, bound2);
        }
        return val >= bound1 - kMathEpsilon && val <= bound2 + kMathEpsilon;
    }

    class LineSegment2f
    {

    public:
        LineSegment2f()
        {
            m_unitDirection = Vec2f(1.0f, 0.0f);
        }

        LineSegment2f(const Vec2f &start, const Vec2f &end)
            : m_start(start), m_end(end)
        {
            const float dx = m_end.x() - m_start.x();
            const float dy = m_end.y() - m_start.y();
            m_length = hypotf(dx, dy);
            m_unitDirection =
                (m_length <= kMathEpsilon ? Vec2f(0.0f, 0.0f)
                                          : Vec2f(dx / m_length, dy / m_length));
            m_heading = m_unitDirection.Angle();
        }

        const Vec2f &Start() const { return m_start; }

        const Vec2f &End() const { return m_end; }

        const Vec2f &UnitDirection() const { return m_unitDirection; }

        Vec2f Center() const { return (m_start + m_end) * 0.5f; }

        Vec2f Rotate(const float angle)
        {
            Vec2f diff_vec = m_end - m_start;

            diff_vec.SelfRotate(angle);
            return m_start + diff_vec;
        }

        float Heading() const { return m_heading; }

        float CosHeading() const { return m_unitDirection.x(); }

        float SinHeading() const { return m_unitDirection.y(); }

        float Length() const { return m_length; }

        float LengthSqr() const { return m_length * m_length; }

        float DistanceTo(const Vec2f &point) const
        {
            if (m_length <= kMathEpsilon)
            {
                return point.DistanceTo(m_start);
            }
            const float x0 = point.x() - m_start.x();
            const float y0 = point.y() - m_start.y();
            const float proj = x0 * m_unitDirection.x() + y0 * m_unitDirection.y();
            if (proj <= 0.0)
            {
                return hypotf(x0, y0);
            }
            if (proj >= m_length)
            {
                return point.DistanceTo(m_end);
            }
            return std::fabsf(x0 * m_unitDirection.y() - y0 * m_unitDirection.x());
        }

        float DistanceTo(const Vec2f &point, Vec2f *const nearest_pt) const
        {

            if (m_length <= kMathEpsilon)
            {
                *nearest_pt = m_start;
                return point.DistanceTo(m_start);
            }
            const double x0 = point.x() - m_start.x();
            const double y0 = point.y() - m_start.y();
            const double proj = x0 * m_unitDirection.x() + y0 * m_unitDirection.y();
            if (proj < 0.0)
            {
                *nearest_pt = m_start;
                return hypotf(x0, y0);
            }
            if (proj > m_length)
            {
                *nearest_pt = m_end;
                return point.DistanceTo(m_end);
            }
            *nearest_pt = m_start + m_unitDirection * proj;
            return std::abs(x0 * m_unitDirection.y() - y0 * m_unitDirection.x());
        }

        float DistanceSquareTo(const Vec2f &point) const
        {
            if (m_length <= kMathEpsilon)
            {
                return point.DistanceSquareTo(m_start);
            }
            const double x0 = point.x() - m_start.x();
            const double y0 = point.y() - m_start.y();
            const double proj = x0 * m_unitDirection.x() + y0 * m_unitDirection.y();
            if (proj <= 0.0)
            {
                return Square(x0) + Square(y0);
            }
            if (proj >= m_length)
            {
                return point.DistanceSquareTo(m_end);
            }
            return Square(x0 * m_unitDirection.y() - y0 * m_unitDirection.x());
        }

        float DistanceSquareTo(const Vec2f &point, Vec2f *const nearest_pt) const
        {
            if (m_length <= kMathEpsilon)
            {
                *nearest_pt = m_start;
                return point.DistanceSquareTo(m_start);
            }
            const double x0 = point.x() - m_start.x();
            const double y0 = point.y() - m_start.y();
            const double proj = x0 * m_unitDirection.x() + y0 * m_unitDirection.y();
            if (proj <= 0.0)
            {
                *nearest_pt = m_start;
                return Square(x0) + Square(y0);
            }
            if (proj >= m_length)
            {
                *nearest_pt = m_end;
                return point.DistanceSquareTo(m_end);
            }
            *nearest_pt = m_start + m_unitDirection * proj;
            return Square(x0 * m_unitDirection.y() - y0 * m_unitDirection.x());
        }

        bool IsPointIn(const Vec2f &point) const {}

        bool HasIntersect(const LineSegment2f &other_segment) const {}

        bool GetIntersect(const LineSegment2f &other_segment, Vec2f *const point) const {}

        float ProjectOntoUnit(const Vec2f &point) const {}

        float ProductOntoUnit(const Vec2f &point) const {}

        float GetPerpendicularFoot(const Vec2f &point, Vec2f *const foot_point) const {}

    private:
        Vec2f m_start;
        Vec2f m_end;
        Vec2f m_unitDirection;
        float m_heading;
        float m_length;
    };

}

#endif /* __LINE_SEGMENT2F_H__ */
