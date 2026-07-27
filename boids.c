#include <stdio.h>
#include <stdlib.h>
//#include <raylib.h>
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
    float world_size; // tamaño del mundo en el que se mueven los boids
    //float separation_weight; // peso de la fuerza de separación
    //float alignment_weight; // peso de la fuerza de alineación
    //float cohesion_weight; // peso de la fuerza de cohesión
} config;

// genera un float aleatorio entre dos valores
float rand_range(float a, float b){
    return a + (rand() / (float)RAND_MAX) * (b-a);
}

// inicializa la posición, velocidad y aceleración de los boids
void boids_init(boid *boids, const config *cfg){
    for (int i = 0; i < cfg->num_boids; i++){
        //genero posicion de los boids
        boids[i].position.x = rand_range(-(cfg->world_size), cfg->world_size);
        boids[i].position.y = rand_range(-(cfg->world_size), cfg->world_size);
        boids[i].position.z = rand_range(-(cfg->world_size), cfg->world_size);

        //genero velocidades de boids en dirección aleatoria
        //primero creo las posiciones iniciales a las que apunta el boid
        vec3 aux;
        aux.x = rand_range(-1, 1);
        aux.y = rand_range(-1, 1);
        aux.z = rand_range(-1, 1);

        //luego las normalizo para que este vector tenga longitud 1
        vec3 direccion = vec3_normalize(aux);

        //genero la velocidad
        float velocidad = rand_range(cfg->min_speed, cfg->max_speed);

        //la multiplico a la dirección
        boids[i].velocity = vec3_scale(direccion, velocidad);

        //aceleracion inicial a 0

        boids[i].acceleration = (vec3){0,0,0};
    }
}

int main() {
    config cfg = {
        .num_boids = 5,
        .min_speed = 1.0f,
        .max_speed = 5.0f,
        .vision_radius = 10.0f,
        .blind_angle = 1.0f, //radianes (aprox 57 grados)
        .cos_blind_angle = 0.0f, // pendiente: precalcular con cosf(blind_angle)
        .world_size = 10,
    };

    // semilla aleatoria
    srand(1);

    // array dinamico para los boids
    boid *boids = malloc(sizeof(boid) * cfg.num_boids);

    //check null de la asignacion anterior
    if (boids == NULL) return 1;
    
    //generación inicial de los boids
    boids_init(boids, &cfg);

    for (int i = 0; i < cfg.num_boids; i++){
        printf("Boid %i: \n", i);
        printf("    Posición: (%f,%f,%f) \n", boids[i].position.x, boids[i].position.y, boids[i].position.z);
        printf("    Velocidad:  (%f,%f,%f) \n", boids[i].velocity.x, boids[i].velocity.y, boids[i].velocity.z);
        printf("    Aceleración: (%f,%f,%f) \n", boids[i].acceleration.x, boids[i].acceleration.y, boids[i].acceleration.z);
    }

    //liberar memoria del array de boids
    free(boids);

    return 0;
}
