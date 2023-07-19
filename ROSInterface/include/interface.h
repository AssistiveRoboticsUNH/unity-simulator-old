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

struct Vector3 {
    double x = 0;
    double y = 0;
    double z = 0;

};

struct NativeTransform {
    char *frame_id = nullptr;
    Vector3 translation;
    Quaternion rotation;
};

void initROS();

extern "C" {
MY_LIB_API void PublishTF(std::intptr_t handle, NativeTransform *input);
MY_LIB_API std::intptr_t Init();
MY_LIB_API void Destroy(std::intptr_t handle);
}
