// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SDTL_ACTION_CLIENT__VISIBILITY_CONTROL_H_
#define SDTL_ACTION_CLIENT__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SDTL_ACTION_CLIENT_EXPORT __attribute__ ((dllexport))
    #define SDTL_ACTION_CLIENT_IMPORT __attribute__ ((dllimport))
  #else
    #define SDTL_ACTION_CLIENT_EXPORT __declspec(dllexport)
    #define SDTL_ACTION_CLIENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef SDTL_ACTION_CLIENT_BUILDING_DLL
    #define SDTL_ACTION_CLIENT_PUBLIC SDTL_ACTION_CLIENT_EXPORT
  #else
    #define SDTL_ACTION_CLIENT_PUBLIC SDTL_ACTION_CLIENT_IMPORT
  #endif
  #define SDTL_ACTION_CLIENT_PUBLIC_TYPE SDTL_ACTION_CLIENT_PUBLIC
  #define SDTL_ACTION_CLIENT_LOCAL
#else
  #define SDTL_ACTION_CLIENT_EXPORT __attribute__ ((visibility("default")))
  #define SDTL_ACTION_CLIENT_IMPORT
  #if __GNUC__ >= 4
    #define SDTL_ACTION_CLIENT_PUBLIC __attribute__ ((visibility("default")))
    #define SDTL_ACTION_CLIENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SDTL_ACTION_CLIENT_PUBLIC
    #define SDTL_ACTION_CLIENT_LOCAL
  #endif
  #define SDTL_ACTION_CLIENT_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // SDTL_ACTION_CLIENT__VISIBILITY_CONTROL_H_