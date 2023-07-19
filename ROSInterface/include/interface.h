#pragma once

#include "Eigen/Core"
#include <optional>

#if defined(_MSC_VER)
#define MY_LIB_API __declspec(dllexport) // Microsoft
#elif defined(__GNUC__)
#define MY_LIB_API __attribute__((visibility("default"))) // GCC
#else
#define MY_LIB_API // Most compilers export all the symbols by default. We hope for the best here.
#pragma warning Unknown dynamic link import/export semantics.
#endif

struct Quaternion {
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 1;

};

struct NativeVector3 {
    double x = 0;
    double y = 0;
    double z = 0;

};

struct NativeTwist {
    NativeVector3 linear;
    NativeVector3 angular;

};

struct NativeTransform {
    char *frame_id = nullptr;
    char *child_frame_id = nullptr;
    NativeVector3 translation;
    Quaternion rotation;
};

struct NativeOdom {
    NativeTransform pose;
    double pose_covariance[36] = {};
    NativeVector3 linear_velocity;
    NativeVector3 angular_velocity;
    double twist_covariance[36] = {};
};

struct NativeActivity {
    int person_eating;
    int person_taking_medicine;
};

void initROS();

extern "C" {
MY_LIB_API void PublishTF(std::intptr_t handle, NativeTransform *input);
MY_LIB_API void PublishActivity(std::intptr_t handle, NativeActivity *input);
MY_LIB_API void PublishOdom(std::intptr_t handle, NativeOdom *input);
MY_LIB_API void ReceiveCmdVel(std::intptr_t handle, NativeTwist *output);
MY_LIB_API std::intptr_t Init();
MY_LIB_API void Destroy(std::intptr_t handle);
}
