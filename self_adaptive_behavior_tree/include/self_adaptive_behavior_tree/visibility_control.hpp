#ifndef SELF_ADAPTIVE_BEHAVIOR_TREE__VISIBILITY_CONTROL_HPP_
#define SELF_ADAPTIVE_BEHAVIOR_TREE__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
#if defined(SELF_ADAPTIVE_BEHAVIOR_TREE_BUILDING_DLL) || \
    defined(SELF_ADAPTIVE_BEHAVIOR_TREE_EXPORTS)
#define SELF_ADAPTIVE_BEHAVIOR_TREE_PUBLIC __declspec(dllexport)
#define SELF_ADAPTIVE_BEHAVIOR_TREE_LOCAL
#else  // defined(SELF_ADAPTIVE_BEHAVIOR_TREE_BUILDING_DLL) ||
       // defined(SELF_ADAPTIVE_BEHAVIOR_TREE_EXPORTS)
#define SELF_ADAPTIVE_BEHAVIOR_TREE_PUBLIC __declspec(dllimport)
#define SELF_ADAPTIVE_BEHAVIOR_TREE_LOCAL
#endif  // defined(SELF_ADAPTIVE_BEHAVIOR_TREE_BUILDING_DLL) ||
        // defined(SELF_ADAPTIVE_BEHAVIOR_TREE_EXPORTS)
#elif defined(__linux__)
#define SELF_ADAPTIVE_BEHAVIOR_TREE_PUBLIC __attribute__((visibility("default")))
#define SELF_ADAPTIVE_BEHAVIOR_TREE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define SELF_ADAPTIVE_BEHAVIOR_TREE_PUBLIC __attribute__((visibility("default")))
#define SELF_ADAPTIVE_BEHAVIOR_TREE_LOCAL __attribute__((visibility("hidden")))
#else
#error "Unsupported Build Configuration"
#endif

#endif  // SELF_ADAPTIVE_BEHAVIOR_TREE__VISIBILITY_CONTROL_HPP_
