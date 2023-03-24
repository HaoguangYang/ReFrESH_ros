#ifndef REFRESH_ROS2__VISIBILITY_CONTROL_HPP_
#define REFRESH_ROS2__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(REFRESH_ROS2_BUILDING_DLL) || defined(REFRESH_ROS2_EXPORTS)
    #define REFRESH_ROS2_PUBLIC __declspec(dllexport)
    #define REFRESH_ROS2_LOCAL
  #else  // defined(REFRESH_ROS2_BUILDING_DLL) || defined(REFRESH_ROS2_EXPORTS)
    #define REFRESH_ROS2_PUBLIC __declspec(dllimport)
    #define REFRESH_ROS2_LOCAL
  #endif  // defined(REFRESH_ROS2_BUILDING_DLL) || defined(REFRESH_ROS2_EXPORTS)
#elif defined(__linux__)
  #define REFRESH_ROS2_PUBLIC __attribute__((visibility("default")))
  #define REFRESH_ROS2_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define REFRESH_ROS2_PUBLIC __attribute__((visibility("default")))
  #define REFRESH_ROS2_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // REFRESH_ROS2__VISIBILITY_CONTROL_HPP_
