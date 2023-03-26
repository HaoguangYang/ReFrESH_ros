#ifndef SELF_AWARENESS_ATTACHMENT__VISIBILITY_CONTROL_HPP_
#define SELF_AWARENESS_ATTACHMENT__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
#if defined(SELF_AWARENESS_ATTACHMENT_BUILDING_DLL) || defined(SELF_AWARENESS_ATTACHMENT_EXPORTS)
#define SELF_AWARENESS_ATTACHMENT_PUBLIC __declspec(dllexport)
#define SELF_AWARENESS_ATTACHMENT_LOCAL
#else  // defined(SELF_AWARENESS_ATTACHMENT_BUILDING_DLL) ||
       // defined(SELF_AWARENESS_ATTACHMENT_EXPORTS)
#define SELF_AWARENESS_ATTACHMENT_PUBLIC __declspec(dllimport)
#define SELF_AWARENESS_ATTACHMENT_LOCAL
#endif  // defined(SELF_AWARENESS_ATTACHMENT_BUILDING_DLL) ||
        // defined(SELF_AWARENESS_ATTACHMENT_EXPORTS)
#elif defined(__linux__)
#define SELF_AWARENESS_ATTACHMENT_PUBLIC __attribute__((visibility("default")))
#define SELF_AWARENESS_ATTACHMENT_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define SELF_AWARENESS_ATTACHMENT_PUBLIC __attribute__((visibility("default")))
#define SELF_AWARENESS_ATTACHMENT_LOCAL __attribute__((visibility("hidden")))
#else
#error "Unsupported Build Configuration"
#endif

#endif  // SELF_AWARENESS_ATTACHMENT__VISIBILITY_CONTROL_HPP_
