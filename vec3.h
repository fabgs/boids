#ifndef VEC3_H
#define VEC3_H

#include <math.h>

// estructura que representa un vector 3D
typedef struct {
    float x;
    float y;
    float z;
} vec3;

// suma de dos vectores 3D
static inline vec3 vec3_add(vec3 a, vec3 b) {
    return (vec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

// resta de dos vectores 3D
static inline vec3 vec3_sub(vec3 a, vec3 b) {
    return (vec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

// escala un vector 3D por un escalar
static inline vec3 vec3_scale(vec3 v, float s) {
    return (vec3){v.x * s, v.y * s, v.z * s};
}

// producto escalar de dos vectores 3D
static inline float vec3_dot(vec3 a, vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// longitud al cuadrado de un vector 3D
static inline float vec3_length2(vec3 v) {
    return vec3_dot(v, v);
}

// normaliza un vector 3D, si el vector es cero, devuelve el vector cero
static inline vec3 vec3_normalize(vec3 v) {
    float len2 = vec3_length2(v);
    if (len2 > 0) {
        float inv_len = 1.0f / sqrtf(len2);
        return vec3_scale(v, inv_len);
    }
    return (vec3){0, 0, 0};
}

#endif // VEC3_H
