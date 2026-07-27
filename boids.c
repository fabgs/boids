#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
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
    float separation_weight; // peso de la fuerza de separación
    float alignment_weight; // peso de la fuerza de alineación
    float cohesion_weight; // peso de la fuerza de cohesión
    boundary_mode boundary_mode;
} config;

// genera un float aleatorio entre dos valores
float rand_range(float a, float b){
    return a + (rand() / (float)RAND_MAX) * (b-a);
}

// para cambiar el comportamiento con respecto a los limites
void simulation_handle_input(config *cfg) {
    if (IsKeyPressed(KEY_B)) {
        if (cfg->boundary_mode == BOUNDARY_BOUNCE) {
            cfg->boundary_mode = BOUNDARY_WRAP;
        } else {
            cfg->boundary_mode = BOUNDARY_BOUNCE;
        }
    }
}

// comprueba que las restas de las distancias no se pasen del tamaño del mapa
float wrapped_delta(float delta, float limit) {
    float world_width = limit * 2.0f;

    if (delta > limit) {
        delta -= world_width;
    } else if (delta < -limit) {
        delta += world_width;
    }

    return delta;
}

// devuelve el vector mas corto entre a y b
vec3 boids_offset(const boid *a, const boid *b, const config *cfg) {
    vec3 offset = vec3_sub(b->position, a->position);

    if (cfg->boundary_mode == BOUNDARY_WRAP) {
        offset.x = wrapped_delta(offset.x, cfg->world_size);
        offset.y = wrapped_delta(offset.y, cfg->world_size);
        offset.z = wrapped_delta(offset.z, cfg->world_size);
    }

    return offset;
}

// devuelve true si la distancia entre boids es igual o menor que el radio de vision limite
bool boids_are_neighbors(const boid *a, const boid *b, const config *cfg) {
    if (a == b) {
        return false;
    }

    vec3 offset = boids_offset(a, b, cfg);
    // distancia al cuadrado
    float distance2 = vec3_length2(offset);
    //radio de vision al cuadrado
    float vision_radius2 = cfg->vision_radius * cfg->vision_radius;

    return distance2 <= vision_radius2;
}

// aplica el teletransporte a la dimension recibida
void boid_apply_dimension_wrap(float *position, float limit){
    if (*position > limit) {
        //se le resta el tamaño del mapa *2 a la posicion para aparecer en el otro lado
        *position -= limit * 2.0f;
    } else if (*position < -limit) {
        *position += limit * 2.0f;
    }
}

// aplica los rebotes de la dimension que reciba
void boid_apply_dimension_bounce(float *position, float *velocity, float limit){
    if (*position > limit) {
        //resta el tamaño del mapa*2 - la posicion para continuar con excesos, ej: 10.2 pasa a ser 9.8 si el mapa es de 10, evitando la perdida de esos 0.2
        *position = 2.0f * limit - *position;
        //se invierte la velocidad
        *velocity = -*velocity;
    } else if (*position < -limit) {
        *position = -2.0f * limit - *position;
        *velocity = -*velocity;
    }
}

// aplica los teletransportes/rebotes en base al tipo elegido
void boid_apply_boundary(boid *b, const config *cfg){
    float limit = cfg->world_size;

    if (cfg->boundary_mode == BOUNDARY_WRAP){
        // teletransporte en x
        boid_apply_dimension_wrap(&b->position.x, limit);
        // teletransporte en y
        boid_apply_dimension_wrap(&b->position.y, limit);
        // teletransporte en z
        boid_apply_dimension_wrap(&b->position.z, limit);
    } else {
        // rebote en x
        boid_apply_dimension_bounce(&b->position.x, &b->velocity.x, limit);
        // rebote en x
        boid_apply_dimension_bounce(&b->position.y, &b->velocity.y, limit);
        // rebote en x
        boid_apply_dimension_bounce(&b->position.z, &b->velocity.z, limit);
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

        // aplica comportamiento con el borde del mapa
        boid_apply_boundary(b, cfg);

        //vaciar aceleracion acumulada para el siguiente frame
        b->acceleration = (vec3){0,0,0};
    }
}

void boids_compute_accelerations(boid *boids, const config *cfg) {
    for (int i = 0; i < cfg->num_boids; i++) {
        //recorro todos los boids, me marco el de referencia
        boid *observer = &boids[i];

        int neighbor_count = 0;

        //vectores para acumular las 3 reglas
        vec3 separation = {0, 0, 0};
        vec3 alignment = {0, 0, 0};
        vec3 cohesion = {0, 0, 0};

        //recorro de nuevo todos los boids, para ver cuales son vecinos del referente
        for (int j = 0; j < cfg->num_boids; j++) {
            if (i == j) {
                continue;
            }

            const boid *other = &boids[j];

            if (!boids_are_neighbors(observer, other, cfg)) {
                continue;
            }

            //separacion
            vec3 vector_huida = boids_offset(other, observer, cfg); 
            float distance = sqrtf(vec3_length2(vector_huida));

            if (distance > 0.001f) { // Evitar divisiones por cero
                // cuanto más cerca, más fuerte es el rechazo
                vec3 push_force = vec3_scale(vector_huida, 1.0f / distance);
                separation = vec3_add(separation, push_force);
            }

            //alineacion
            alignment = vec3_add(alignment, other->velocity);

            //cohesion
            vec3 vector_acercar = boids_offset(observer, other, cfg);
            cohesion = vec3_add(cohesion, vector_acercar);

            neighbor_count++;
        }

        if (neighbor_count > 0){
            //promediar las fuerzas
            alignment = vec3_scale(alignment, 1.0f / neighbor_count);
            cohesion = vec3_scale(cohesion, 1.0f / neighbor_count);

            //aplicar pesos
            vec3 sep_force = vec3_scale(separation, cfg->separation_weight);
            vec3 ali_force = vec3_scale(alignment, cfg->alignment_weight);
            vec3 coh_force = vec3_scale(cohesion, cfg->cohesion_weight);

            //sumo las fuerzas a la aceleración
            observer->acceleration = vec3_add(observer->acceleration, sep_force);
            observer->acceleration = vec3_add(observer->acceleration, ali_force);
            observer->acceleration = vec3_add(observer->acceleration, coh_force);
        }
    }
}

int main() {
    config cfg = {
        .num_boids = 1000,
        .min_speed = 1.0f,
        .max_speed = 5.0f,
        .vision_radius = 10.0f,
        .blind_angle = 1.0f, //radianes (aprox 57 grados)
        .cos_blind_angle = 0.0f, // pendiente: precalcular con cosf(blind_angle)
        .world_size = 100,
        .separation_weight = 0.3f,
        .alignment_weight = 0.3f,
        .cohesion_weight = 0.3f,
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
    camera.position = (Vector3){cfg.world_size+10.0f, cfg.world_size+10.0f, cfg.world_size+10.0f};
    camera.target = (Vector3){0.0f, 0.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE; 

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        simulation_handle_input(&cfg);
        UpdateCamera(&camera, CAMERA_FREE);

        //calculo de reglas (separacion, alineacion, cohesion)
        boids_compute_accelerations(boids, &cfg);
        //actualizacion de boids
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

            Color color;

            if (i == 0) {
                color = YELLOW;
            } else if (boids_are_neighbors(&boids[0], b, &cfg)) {
                color = GREEN;
            } else {
                color = SKYBLUE;
            }

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

            //dibujo el boid y lo pinto en funcion de quien es
            DrawCylinderEx(base, tip, 0.18f, 0.0f, 8, color);
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
