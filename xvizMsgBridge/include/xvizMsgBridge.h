/**
 * @Author: Xia Yunkai
 * @Date:   2023-12-29 09:22:25
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-08 19:41:50
 */
#include <stdint.h>

#ifndef __XVIZMSGSENDER_H__
#define __XVIZMSGSENDER_H__

#include "data_types.h"
#include <mutex>
#include <string>
#include <memory>
#include <functional>
#include <thread>
namespace zmq
{
    class context_t;
    class socket_t;

}

namespace xviz
{
    typedef std::function<void(const Pose &)> PoseCallbackFunc;

    class XvizMsgBridge
    {
    public:
        typedef std::unique_ptr<XvizMsgBridge> Ptr;
        XvizMsgBridge();
        ~XvizMsgBridge();
        bool Init(const std::string &pub_connect, const std::string &sub_connect);
        void Run();
        void SetInitPoseFunc(const PoseCallbackFunc &func);
        void SetTargetPoseFunc(const PoseCallbackFunc &func);
        void PathPub(const std::string &topic, const Path2f &path);
        void PosePub(const std::string &topic, const Pose &pose);
        void PointCloudPub(const std::string &topic, const PointCloud3f &pointcloud);
        void PolygonPub(const std::string &topic, const Polygon2f &polygon);
        void PolygonsPub(const std::string &topic, const Polygons2f &polygons);
        void CirclePub(const std::string &topic, const Circle &circle);
        void BezierPub(const std::string &topic, const Bezier &bezier);
        void MarkerArrayPub(const std::string &topic, const MarkerArray &markerArray);
        void FloatDataPub(const std::string &name, const float data);
        void StringDataPub(const std::string &name, const std::string &data);
        void GridMapPub(const std::string &name, const GridMap &map);
        void TransformPub(const std::string &name, const TransformNode &transform);
        void Shutdown();

    private:
        template <typename PROTO_MSG>
        void PubProto(const PROTO_MSG &proto_msg,
                      const std::string &topic, const std::string &msg_type);
        void ReceiveLoop();

    private:
        std::unique_ptr<zmq::context_t> m_ctx;
        std::unique_ptr<zmq::socket_t> m_pub;
        std::unique_ptr<zmq::socket_t> m_sub;
        std::thread m_receiveThread;
        std::string m_pubConnect;
        std::string m_subConnect;
        bool m_running;
        std::mutex m_mtx;
        PoseCallbackFunc m_initPoseCB;
        bool m_bSetInitPoseCB = false;
        PoseCallbackFunc m_tarPoseCB;
        bool m_bSetTarPoseCB = false;
    };
}
#endif /* __XVIZMSGSENDER_H__ */
