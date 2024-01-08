/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-08 21:39:41
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-08 22:12:23
 */
#include <stdint.h>

#ifndef __VEC2F_H__
#define __VEC2F_H__

#include <cmath>
#include <string>

namespace auto_parking_planning
{
    constexpr float kMathEpsilon = 1e-10f;
    class Vec2f
    {
    public:
        constexpr Vec2f(const float x, const float y) noexcept : m_x(x), m_y(y) {}

        constexpr Vec2f() noexcept : Vec2f(0, 0) {}

        static Vec2f CreateUnitVec2f(const float angle)
        {
            return Vec2f(cosf(angle), sinf(angle));
        }

        float x() const { return m_x; }

        float y() const { return m_y; }

        void SetX(const float x) { m_x = x; }

        void SetY(const float y) { m_y = y; }

        float Length() const { return hypotf(m_x, m_y); }

        float LengthSquare() const { return m_x * m_x + m_y * m_y; }

        float Angle() const { return atan2f(m_y, m_x); }

        void Normalize()
        {
            const float l = Length();
            if (l > kMathEpsilon)
            {
                m_x /= l;
                m_y /= l;
            }
        }

        float DistanceTo(const Vec2f &other) const
        {
            return hypotf(m_x - other.x(), m_y - other.y());
        }

        float DistanceSquareTo(const Vec2f &other) const
        {
            const float dx = m_x - other.x();
            const float dy = m_y - other.y();
            return dx * dx + dy * dy;
        }

        float CrossProd(const Vec2f &other) const
        {
            return m_x * other.y() - m_y * other.x();
        }

        float InnerProd(const Vec2f &other) const
        {
            return m_x * other.x() + m_y * other.y();
        }

        Vec2f Rotate(const float angle) const
        {
            return Vec2f(m_x * cosf(angle) - m_y * sinf(angle),
                         m_x * sinf(angle) + m_y * cosf(angle));
        }

        void SelfRotate(const float angle)
        {
            float tmp_x = m_x;
            m_x = m_x * cosf(angle) - m_y * sinf(angle);
            m_y = tmp_x * sinf(angle) + m_y * cosf(angle);
        }

        Vec2f operator+(const Vec2f &other) const
        {
            return Vec2f(m_x + other.x(), m_y + other.y());
        }

        Vec2f operator-(const Vec2f &other) const
        {
            return Vec2f(m_x - other.x(), m_y - other.y());
        }

        Vec2f operator*(const float ratio) const
        {
            return Vec2f(m_x * ratio, m_y * ratio);
        }

        Vec2f operator/(const float ratio) const
        {
            return Vec2f(m_x / ratio, m_y / ratio);
        }

        Vec2f &operator+=(const Vec2f &other)
        {
            m_x += other.x();
            m_y += other.y();
            return *this;
        }

        Vec2f &operator-=(const Vec2f &other)
        {
            m_x -= other.x();
            m_y -= other.y();
            return *this;
        }

        Vec2f &operator*=(const float ratio)
        {
            m_x *= ratio;
            m_y *= ratio;
            return *this;
        }

        Vec2f &operator/=(const float ratio)
        {
            m_x /= ratio;
            m_y /= ratio;
            return *this;
        }

        bool operator==(const Vec2f &other) const
        {
            return (std::abs(m_x - other.x()) < kMathEpsilon &&
                    std::abs(m_y - other.y()) < kMathEpsilon);
        }

    protected:
        float m_x = 0;
        float m_y = 0;
    };

    Vec2f operator*(const float ratio, const Vec2f &vec) { return vec * ratio; }
} // namespace auto_parking_planning

#endif /* __VEC2F_H__ */
