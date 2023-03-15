#ifndef REFRESH_TASK_ENGINE__VISIBILITY_CONTROL_HPP_
#define REFRESH_TASK_ENGINE__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(REFRESH_TASK_ENGINE_BUILDING_DLL) || defined(REFRESH_TASK_ENGINE_EXPORTS)
    #define REFRESH_TASK_ENGINE_PUBLIC __declspec(dllexport)
    #define REFRESH_TASK_ENGINE_LOCAL
  #else  // defined(REFRESH_TASK_ENGINE_BUILDING_DLL) || defined(REFRESH_TASK_ENGINE_EXPORTS)
    #define REFRESH_TASK_ENGINE_PUBLIC __declspec(dllimport)
    #define REFRESH_TASK_ENGINE_LOCAL
  #endif  // defined(REFRESH_TASK_ENGINE_BUILDING_DLL) || defined(REFRESH_TASK_ENGINE_EXPORTS)
#elif defined(__linux__)
  #define REFRESH_TASK_ENGINE_PUBLIC __attribute__((visibility("default")))
  #define REFRESH_TASK_ENGINE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define REFRESH_TASK_ENGINE_PUBLIC __attribute__((visibility("default")))
  #define REFRESH_TASK_ENGINE_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // REFRESH_TASK_ENGINE__VISIBILITY_CONTROL_HPP_
