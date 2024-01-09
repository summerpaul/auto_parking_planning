/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-09 20:07:43
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-09 20:10:30
 */
#include <stdint.h>

#ifndef __LINE_SEGMENT2F_H__
#define __LINE_SEGMENT2F_H__

#include "vec2f.h"

namespace auto_parking_planning
{

    class LineSegment2f
    {

    public:
        LineSegment2f() {}

        LineSegment2f(const Vec2f &start, const Vec2f &end);

    private:
        Vec2f m_start;
        Vec2f m_end;
        Vec2f m_unitDirection;
        float m_heading;
        float m_length;
    };

}

#endif /* __LINE_SEGMENT2F_H__ */
