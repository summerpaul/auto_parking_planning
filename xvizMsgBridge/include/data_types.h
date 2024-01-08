/**
 * @Author: Xia Yunkai
 * @Date:   2023-12-31 02:28:45
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-07 09:55:40
 */

#ifndef __DATA_TYPES_H__
#define __DATA_TYPES_H__

#include <vector>
#include <string>
#include <iostream>
#include <cmath>
// 对外开放的基本数据类型
namespace xviz
{

    struct Header
    {
        double timeStamp = 0.0;
        unsigned int seq = 2;
        std::string frameId = "world";
    };

    /// @brief 位姿
    struct Pose
    {

        Pose() : x(0.0f), y(0.0f), yaw(0.0f) {}
        Pose(float x, float y, float yaw) : x(x), y(y), yaw(yaw) {}
        float x, y, yaw;
        Header header;
    };

    struct Vec2f
    {
        Vec2f() : x(0.0f), y(0.0f) {}
        Vec2f(float x, float y) : x(x), y(y) {}
        void SetZero() { x = 0.0f, y = 0.0f; }
        float x, y;
    };

    struct Vec3f
    {
        Vec3f() : x(0.0f), y(0.0f), z(0.0f) {}
        Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
        void SetZero() { x = 0.0f, y = 0.0f, z = 0.0f; }
        float x, y, z;
    };

    struct Vec2i
    {
        Vec2i() : x(0), y(0) {}
        Vec2i(int x, int y) : x(x), y(y) {}
        void SetZero() { x = 0, y = 0; }
        int x, y;
    };

    struct PointXYZ
    {
        PointXYZ() : x(0.0f), y(0.0f), z(0.0f) {}
        PointXYZ(float x, float y, float z) : x(x), y(y), z(z) {}
        float x, y, z;
    };

    struct GridMap
    {
        Header header;
        unsigned char *m_data;
        unsigned int m_dataPtr;
        float m_res;
        Vec2f m_origin;
        Vec2f m_size;
        float m_originYaw;
        bool m_usePtr = false;
    };

    struct Bezier
    {
        Header header;
        Vec2f p0;
        Vec2f p1;
        Vec2f p2;
        Vec2f p3;
    };

    struct Circle
    {
        Header header;
        Vec2f center;
        float radius;
    };

    enum class MarkerType
    {
        PATH = 0,
        POLYGON = 1,
        CIRCLE = 3,
        BEZIER = 4,
    };

    struct PointCloud3f
    {
        Header header;
        std::vector<PointXYZ> points;
    };

    struct PointCloud2f
    {
        Header header;
        std::vector<Vec2f> points;
    };

    struct Path2f
    {
        Header header;
        std::vector<Vec2f> points;
    };

    struct Polygon2f
    {
        Header header;
        std::vector<Vec2f> points;
    };

    struct Polygons2f
    {
        Header header;
        std::vector<Polygon2f> polygons;
    };

    struct Marker
    {
        Header header;
        MarkerType type;
        Path2f path;
        Polygon2f polygon;
        Circle circle;
        Bezier bezier;
        unsigned int color;
    };

    struct MarkerArray
    {
        Header header;
        std::vector<Marker> markers;
    };

    struct Rot
    {
        Rot() = default;

        explicit Rot(float angle)
        {
            m_sin = sinf(angle);
            m_cos = cosf(angle);
            
        }

        void Set(float angle)
        {

            m_sin = sinf(angle);
            m_cos = cosf(angle);
            
        }

        void SetIdentity()
        {
            m_sin = 0.0f;
            m_cos = 1.0f;
        }

        float GetAngle() const
        {
            return atan2f(m_sin, m_cos);
        }

        Vec2f GetXAxis() const
        {
            return Vec2f(m_cos, m_sin);
        }

        Vec2f GetYAxis() const
        {
            return Vec2f(-m_sin, m_cos);
        }

        float m_sin = 0.0f;
        float m_cos = 1.0f;
    };

    struct Transform
    {
        Transform() { SetIdentity(); }
        Transform(float x, float y, float angle) { Set(Vec2f(x, y), angle); }
        Transform(const Vec2f &position, const Rot &rotation) : m_trans(position), m_rot(rotation) {}

        void Print()
        {
            std::cout << " position x is " << m_trans.x << " y is " << m_trans.y << " yaw is " << m_rot.GetAngle() << std::endl;
        }

        void SetIdentity()
        {
            m_trans.SetZero();
            m_rot.SetIdentity();
        }

        void Set(const Vec2f &position, float angle)
        {
            m_trans = position;
            m_rot.Set(angle);
        }

        Transform Inv() const
        {
            float inv_angle = -m_rot.GetAngle();
            float inv_x = -m_trans.x * m_rot.m_cos - m_trans.y * m_rot.m_sin;
            float inv_y = m_trans.x * m_rot.m_sin - m_trans.y * m_rot.m_cos;
            return Transform(inv_x, inv_y, inv_angle);
        }

        Vec2f m_trans;
        Rot m_rot;
    };

    const std::string TF_ROOT_NAME = "none";

    struct TransformNode
    {
        Transform m_transform;
        std::string m_frameId;
        std::string m_parentFrameId = TF_ROOT_NAME;
    };

    const std::string MSG_PATH = "MSG_PATH";
    const std::string MSG_POSE = "MSG_POSE";
    const std::string MSG_POINTCLOUD = "MSG_POINTCLOUD";
    const std::string MSG_POLYGON = "MSG_POLYGON";
    const std::string MSG_POLYGONS = "MSG_POLYGONS";
    const std::string MSG_CIRCLE = "MSG_CIRCLE";
    const std::string MSG_BEZIER = "MSG_BEZIER";
    const std::string MSG_MARKER_ARRAY = "MSG_MARKER_ARRAY";
    const std::string MSG_FLOAT_DATA = "MSG_FLOAT_DATA";
    const std::string MSG_STRING_DATA = "MSG_STRING_DATA";
    const std::string MSG_GRID_MAP = "MSG_GRID_MAP";
    const std::string MSG_TRANSFORM = "MSG_TRANSFORM";
    enum class ColorType
    {
        WHITE = 0,
        BLACK = 1,
        BLUE = 2,
        GREEN = 3,
        RED = 4,
        YELLOW = 5,
        CYAN = 6,
        MAGENTA = 7,
        GRAY = 8,
        PURPLE = 9,
        PINK = 10,
        LIGHT_BLUE = 11,
        LIME_GREEN = 12,
        SLATE_GRAY = 13,
        COLOR_COUNT
    };

} // namespace xviz

#endif /* __DATA_TYPES_H__ */
