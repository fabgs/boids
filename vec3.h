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

// producto vectorial de dos vectores 3D
static inline vec3 vec3_cross(vec3 a, vec3 b) {
    return (vec3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
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

// interpolacion lineal entre dos vectores 3D
static inline vec3 vec3_lerp(vec3 a, vec3 b, float t) {
    return (vec3){
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

static inline vec3 vec3_clamp_length(vec3 v, float min, float max) {
    float len2 = vec3_length2(v);

    if (len2 > max*max){
        return vec3_scale(vec3_normalize(v), max);
    } else if ((len2 < min*min) && (len2 > 0)){
        return vec3_scale(vec3_normalize(v), min);
    }

    return v;
}

#endif // VEC3_H
