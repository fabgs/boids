#include <stdio.h>

#include "vec3.h"

// estructura que representa un boid, posición, velocidad y aceleración
typedef struct {
    vec3 position;
    vec3 velocity;
    vec3 acceleration;
} boid;

// estructura que representa la configuración para el sistema de boids
typedef struct{
    int num_boids;
    float min_speed;
    float max_speed;
    float vision_radius; // radio de visión de los boids
    float blind_angle; // ángulo muerto trasero de visión de los boids
    float cos_blind_angle; // coseno del ángulo muerto trasero de visión de los boids
    //float max_force; // fuerza máxima de aceleración de los boids
    //float world_size; // tamaño del mundo en el que se mueven los boids
    //float separation_weight; // peso de la fuerza de separación
    //float alignment_weight; // peso de la fuerza de alineación
    //float cohesion_weight; // peso de la fuerza de cohesión
} config;

int main() {

    vec3 v1 = {1.0f, 2.0f, 3.0f};
    vec3 v2 = {4.0f, 5.0f, 6.0f};

    vec3 v3 = vec3_add(v1, v2);

    printf("v3 = (%f, %f, %f)\n", v3.x, v3.y, v3.z);

    float v4 = vec3_dot(v1, v2);

    printf("v4 = %f\n", v4);

    vec3 v5 = {3.0f, 4.0f, 0.0f};

    float v6 = vec3_length2(v5);

    printf("v6 = %f\n", v6);

    vec3 v7 = vec3_normalize(v5);
    
    printf("v7 = (%f, %f, %f)\n", v7.x, v7.y, v7.z);

    vec3 v8 = {0.0f, 0.0f, 0.0f};

    vec3 v9 = vec3_normalize(v8);

    printf("v9 = (%f, %f, %f)\n", v9.x, v9.y, v9.z);

    vec3 v10 = {1.0f, 0.0f, 0.0f};
    vec3 v11 = {0.0f, 1.0f, 0.0f};

    float v12 = vec3_dot(v10, v11);

    printf("v12 = %f\n", v12);

    vec3 v13 = {-1.0f, 0.0f, 0.0f};

    float v14 = vec3_dot(v10, v13);

    printf("v14 = %f\n", v14);

    return 0;
}
