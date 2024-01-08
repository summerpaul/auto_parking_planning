cmake_minimum_required(VERSION 3.10.0)
project(auto_parking_planning)



include_directories(xvizMsgBridge/include)

if(WIN32)
    link_directories(xvizMsgBridge/lib/x64-windows/Release)
elseif(APPLE)
    link_directories(xvizMsgBridge/lib/arm64-osx)
elseif(UNIX)
endif()

