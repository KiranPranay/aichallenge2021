// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VEHICLE_POSE_PUBLISHER__VISIBILITY_CONTROL_HPP_
#define VEHICLE_POSE_PUBLISHER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(VEHICLE_POSE_PUBLISHER_BUILDING_DLL) || defined(VEHICLE_POSE_PUBLISHER_EXPORTS)
    #define VEHICLE_POSE_PUBLISHER_PUBLIC __declspec(dllexport)
    #define VEHICLE_POSE_PUBLISHER_LOCAL
  #else  // defined(VEHICLE_POSE_PUBLISHER_BUILDING_DLL) || defined(VEHICLE_POSE_PUBLISHER_EXPORTS)
    #define VEHICLE_POSE_PUBLISHER_PUBLIC __declspec(dllimport)
    #define VEHICLE_POSE_PUBLISHER_LOCAL
  #endif  // defined(VEHICLE_POSE_PUBLISHER_BUILDING_DLL) || defined(VEHICLE_POSE_PUBLISHER_EXPORTS)
#elif defined(__linux__)
  #define VEHICLE_POSE_PUBLISHER_PUBLIC __attribute__((visibility("default")))
  #define VEHICLE_POSE_PUBLISHER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define VEHICLE_POSE_PUBLISHER_PUBLIC __attribute__((visibility("default")))
  #define VEHICLE_POSE_PUBLISHER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // VEHICLE_POSE_PUBLISHER__VISIBILITY_CONTROL_HPP_
