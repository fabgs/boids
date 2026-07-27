#include <stdio.h>
#include <stdlib.h>
#include "vec3.h"
#include "raylib.h"

typedef enum {
    BOUNDARY_BOUNCE,
    BOUNDARY_WRAP
} boundary_mode;

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
    boundary_mode boundary_mode;
} config;

// genera un float aleatorio entre dos valores
float rand_range(float a, float b){
    return a + (rand() / (float)RAND_MAX) * (b-a);
}

void boid_apply_boundary(boid *b, const config *cfg){
    if (cfg->boundary_mode == BOUNDARY_WRAP){
        // teletransporte en x
        if (b->position.x > cfg->world_size){
            b->position.x -= cfg->world_size * 2.0f;
        } else if (b->position.x < -(cfg->world_size)){
            b->position.x += cfg->world_size * 2.0f;
        }

        // teletransporte en y
        if (b->position.y > cfg->world_size){
            b->position.y -= cfg->world_size * 2.0f;
        } else if (b->position.y < -(cfg->world_size)){
            b->position.y += cfg->world_size * 2.0f;
        }

        // teletransporte en z
        if (b->position.z > cfg->world_size){
            b->position.z -= cfg->world_size * 2.0f;
        } else if (b->position.z < -(cfg->world_size)){
            b->position.z += cfg->world_size * 2.0f;
        }
    } else {
        // rebote en x
        if (b->position.x > cfg->world_size){
            b->position.x = 2.0f * cfg->world_size - b->position.x;
            b->velocity.x = -b->velocity.x;
        } else if (b->position.x < -(cfg->world_size)){
            b->position.x = -2.0f * cfg->world_size - b->position.x;
            b->velocity.x = -b->velocity.x;
        }

        // rebote en y
        if (b->position.y > cfg->world_size){
            b->position.y = 2.0f * cfg->world_size - b->position.y;
            b->velocity.y = -b->velocity.y;
        } else if (b->position.y < -(cfg->world_size)){
            b->position.y = -2.0f * cfg->world_size - b->position.y;
            b->velocity.y = -b->velocity.y;
        }

        // rebote en z
        if (b->position.z > cfg->world_size){
            b->position.z = 2.0f * cfg->world_size - b->position.z;
            b->velocity.z = -b->velocity.z;
        } else if (b->position.z < -(cfg->world_size)){
            b->position.z = -2.0f * cfg->world_size - b->position.z;
            b->velocity.z = -b->velocity.z;
        }
    }
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
void boids_update(boid *boids, const config *cfg, float dt){
    for (int i = 0; i < cfg->num_boids; i++){
        boid *b = &boids[i];

        //aceleración modifica la velocidad en este frame
        b->velocity = vec3_add(b->velocity, vec3_scale(b->acceleration, dt));

        //forzar rapidez que no se pase de los limites
        b->velocity = vec3_clamp_length(b->velocity, cfg->min_speed, cfg->max_speed);

        //la velocidad hace que cambie la posición
        b->position = vec3_add(b->position, vec3_scale(b->velocity, dt));

        boid_apply_boundary(b, cfg);

        //vaciar aceleracion acumulada para el siguiente frame
        b->acceleration = (vec3){0,0,0};
    }
}

int main() {
    config cfg = {
        .num_boids = 50,
        .min_speed = 1.0f,
        .max_speed = 5.0f,
        .vision_radius = 10.0f,
        .blind_angle = 1.0f, //radianes (aprox 57 grados)
        .cos_blind_angle = 0.0f, // pendiente: precalcular con cosf(blind_angle)
        .world_size = 10,
        .boundary_mode = BOUNDARY_BOUNCE,
    };

    // semilla aleatoria
    srand(1);

    // array dinamico para los boids
    boid *boids = malloc(sizeof(boid) * cfg.num_boids);

    //check null de la asignacion anterior
    if (boids == NULL) return 1;
    

    //generación inicial de los boids
    boids_init(boids, &cfg);

    InitWindow(1280, 720, "Boids 3D");
    SetTargetFPS(60);
    DisableCursor();

    Camera3D camera = {0};
    camera.position = (Vector3){cfg.world_size*2.0f, cfg.world_size*2.0f, cfg.world_size*2.0f};
    camera.target = (Vector3){0.0f, 0.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE; 

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        UpdateCamera(&camera, CAMERA_FREE);
        boids_update(boids, &cfg, dt);

        BeginDrawing();
        ClearBackground((Color){20, 22, 28, 255});

        BeginMode3D(camera);

        // dibujar el suelo y los limites del mundo
        //DrawGrid(20, 1.0f); //TODO: se dibujara cuando se implemente el grid parametrizado para comprobaciones locales
        DrawCubeWires(
            (Vector3){0.0f, 0.0f, 0.0f},
            cfg.world_size * 2.0f,
            cfg.world_size * 2.0f,
            cfg.world_size * 2.0f,
            GRAY
        );

        // dibujar cada boid como un cono orientado en su direccion de vuelo
        for (int i = 0; i < cfg.num_boids; i++) {
            boid *b = &boids[i];
            vec3 direction = vec3_normalize(b->velocity);
            const float half_length = 0.3f;

            Vector3 base = {
                b->position.x - direction.x * half_length,
                b->position.y - direction.y * half_length,
                b->position.z - direction.z * half_length
            };
            Vector3 tip = {
                b->position.x + direction.x * half_length,
                b->position.y + direction.y * half_length,
                b->position.z + direction.z * half_length
            };

            // pinto uno de amarillo para poder seguirlo
            if (i == 0){
                DrawCylinderEx(base, tip, 0.18f, 0.0f, 8, YELLOW);
            } else {
                DrawCylinderEx(base, tip, 0.18f, 0.0f, 8, SKYBLUE);
            }
        }

        EndMode3D();

        DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();

    //liberar memoria del array de boids
    free(boids);

    return 0;
}
