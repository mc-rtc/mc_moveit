#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define MoveItFSM_DLLIMPORT __declspec(dllimport)
#  define MoveItFSM_DLLEXPORT __declspec(dllexport)
#  define MoveItFSM_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define MoveItFSM_DLLIMPORT __attribute__((visibility("default")))
#    define MoveItFSM_DLLEXPORT __attribute__((visibility("default")))
#    define MoveItFSM_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define MoveItFSM_DLLIMPORT
#    define MoveItFSM_DLLEXPORT
#    define MoveItFSM_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef MoveItFSM_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define MoveItFSM_DLLAPI
#  define MoveItFSM_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef MoveItFSM_EXPORTS
#    define MoveItFSM_DLLAPI MoveItFSM_DLLEXPORT
#  else
#    define MoveItFSM_DLLAPI MoveItFSM_DLLIMPORT
#  endif // MoveItFSM_EXPORTS
#  define MoveItFSM_LOCAL MoveItFSM_DLLLOCAL
#endif // MoveItFSM_STATIC
