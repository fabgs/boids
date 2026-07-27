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
    float delta_time; //velocidad de simulación
} config;

// genera un float aleatorio entre dos valores
float rand_range(float a, float b){
    return a + (rand() / (float)RAND_MAX) * (b-a);
}

// inicializa la posición, velocidad y aceleración de los boids
void boids_init(boid *boids, const config *cfg){
    for (int i = 0; i < cfg->num_boids; i++){
        //creo un puntero a la posicion del boid
        boid *b = &boids[i];

        //genero posicion de los boids
        b->position.x = rand_range(-(cfg->world_size), cfg->world_size);
        b->position.y = rand_range(-(cfg->world_size), cfg->world_size);
        b->position.z = rand_range(-(cfg->world_size), cfg->world_size);

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
        b->velocity = vec3_scale(direccion, velocidad);

        //aceleracion inicial a 0

        b->acceleration = (vec3){0,0,0};
    }
}

// actualización de las posiciones de los boids, recibe array de boids y configuración
void boids_update(boid *boids, const config *cfg){
    for (int i = 0; i < cfg->num_boids; i++){
        boid *b = &boids[i];

        //aceleración modifica la velocidad en este frame
        b->velocity = vec3_add(b->velocity, vec3_scale(b->acceleration, cfg->delta_time));

        //forzar rapidez que no se pase de los limites
        b->velocity = vec3_clamp_length(b->velocity, cfg->min_speed, cfg->max_speed);

        //la velocidad hace que cambie la posición
        b->position = vec3_add(b->position, vec3_scale(b->velocity, cfg->delta_time));

        //vaciar aceleracion acumulada para el siguiente frame
        b->acceleration = (vec3){0,0,0};
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
        .delta_time = 0.016f, //1/60 de segundo, para que sea 60 frames por segundo
    };

    // semilla aleatoria
    srand(1);

    // array dinamico para los boids
    boid *boids = malloc(sizeof(boid) * cfg.num_boids);

    //check null de la asignacion anterior
    if (boids == NULL) return 1;
    
    //generación inicial de los boids
    boids_init(boids, &cfg);

    //TODO: quitar, es para comprobar posiciones iniciales de los boids
    for (int i = 0; i < cfg.num_boids; i++){
        printf("Boid %i: \n", i);
        printf("    Posición: (%f,%f,%f) \n", boids[i].position.x, boids[i].position.y, boids[i].position.z);
        printf("    Velocidad:  (%f,%f,%f) \n", boids[i].velocity.x, boids[i].velocity.y, boids[i].velocity.z);
        printf("    Aceleración: (%f,%f,%f) \n", boids[i].acceleration.x, boids[i].acceleration.y, boids[i].acceleration.z);
    }

    //simular 100 ticks
    for (int i = 0; i < 100; i++){
        boids_update(boids, &cfg);
        if (i%10 == 0){
            printf("Boid 0: \n");
            printf("    Posición: (%f,%f,%f) \n", boids[0].position.x, boids[0].position.y, boids[0].position.z);
        }
    }

    //liberar memoria del array de boids
    free(boids);

    return 0;
}
