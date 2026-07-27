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

// grid espacial para reducir las comparaciones entre boids
typedef struct {
    int *head; //array del tamaño del número total de celdas
    int *next; //array del tamaño de num_boids
    int cells_x, cells_y, cells_z; //celdas por eje
    int total_cells;
    float cell_size;
    float world_offset; // pasar de -mundo, +mundo a 0, mundo*2
} spatial_grid;

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

// calcula las aceleraciones en base a las reglas de reynolds
void boids_compute_accelerations(boid *boids, const config *cfg, const spatial_grid *g) {
    for (int i = 0; i < cfg->num_boids; i++) {
        boid *observer = &boids[i];
        int neighbor_count = 0;

        vec3 separation = {0, 0, 0};
        vec3 alignment = {0, 0, 0};
        vec3 cohesion = {0, 0, 0};

        //calcular en qué celda está el boid observador
        int cx = (int)((observer->position.x + g->world_offset) / g->cell_size);
        int cy = (int)((observer->position.y + g->world_offset) / g->cell_size);
        int cz = (int)((observer->position.z + g->world_offset) / g->cell_size);
        
        //clampear para no salirnos del grid
        if (cx < 0) cx = 0; else if (cx >= g->cells_x) cx = g->cells_x - 1;
        if (cy < 0) cy = 0; else if (cy >= g->cells_y) cy = g->cells_y - 1;
        if (cz < 0) cz = 0; else if (cz >= g->cells_z) cz = g->cells_z - 1;

        //buscar en la propia celda y en las 26 vecinas
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    int nx = cx + dx;
                    int ny = cy + dy;
                    int nz = cz + dz;

                    //gestionar los límites según el boundary_mode
                    if (cfg->boundary_mode == BOUNDARY_WRAP) {
                        // Si nos salimos por la derecha, miramos por la izquierda
                        if (nx < 0) nx += g->cells_x; else if (nx >= g->cells_x) nx -= g->cells_x;
                        if (ny < 0) ny += g->cells_y; else if (ny >= g->cells_y) ny -= g->cells_y;
                        if (nz < 0) nz += g->cells_z; else if (nz >= g->cells_z) nz -= g->cells_z;
                    } else {
                        //en modo rebote, las celdas fuera del mapa no existen
                        if (nx < 0 || nx >= g->cells_x || ny < 0 || ny >= g->cells_y || nz < 0 || nz >= g->cells_z) {
                            continue;
                        }
                    }

                    //calcular índice 1D de esta celda vecina
                    int cell_idx = nx + ny * g->cells_x + nz * g->cells_x * g->cells_y;
                    
                    //recorrer los boids que están en esta celda vecina
                    int j = g->head[cell_idx];
                    
                    while (j != -1) { //mientras haya boids en la lista
                        if (i != j) {
                            const boid *other = &boids[j];

                            if (boids_are_neighbors(observer, other, cfg)) {
                                // Separación
                                vec3 vector_huida = boids_offset(other, observer, cfg); 
                                float distance = sqrtf(vec3_length2(vector_huida));
                                if (distance > 0.001f) {
                                    vec3 push_force = vec3_scale(vector_huida, 1.0f / distance);
                                    separation = vec3_add(separation, push_force);
                                }

                                // Alineación
                                alignment = vec3_add(alignment, other->velocity);

                                // Cohesión
                                vec3 vector_acercar = boids_offset(observer, other, cfg);
                                cohesion = vec3_add(cohesion, vector_acercar);

                                neighbor_count++;
                            }
                        }
                        
                        //siguiente boid de esta celda
                        j = g->next[j]; 
                    }
                }
            }
        }

        // Aplicar promedios y pesos
        if (neighbor_count > 0){
            alignment = vec3_scale(alignment, 1.0f / neighbor_count);
            cohesion = vec3_scale(cohesion, 1.0f / neighbor_count);

            vec3 sep_force = vec3_scale(separation, cfg->separation_weight);
            vec3 ali_force = vec3_scale(alignment, cfg->alignment_weight);
            vec3 coh_force = vec3_scale(cohesion, cfg->cohesion_weight);

            observer->acceleration = vec3_add(observer->acceleration, sep_force);
            observer->acceleration = vec3_add(observer->acceleration, ali_force);
            observer->acceleration = vec3_add(observer->acceleration, coh_force);
        }
    }
}

spatial_grid spatial_grid_init(const config *cfg){
    spatial_grid g;
    g.cell_size = cfg->vision_radius;

    //numero de celdas por dimension
    g.cells_x = (int)((2.0f * cfg->world_size) / g.cell_size) + 1;
    g.cells_y = g.cells_x;
    g.cells_z = g.cells_x;
    g.total_cells = g.cells_x*g.cells_y*g.cells_z;
    g.world_offset = cfg->world_size;

    g.head = malloc(g.total_cells * sizeof(int));
    g.next = malloc(cfg->num_boids * sizeof(int));
    
    return g;
}

// actualizador de posiciones de boids en el grid
void grid_build(spatial_grid *g, const boid *boids, const config *cfg) {
    //vaciar el grid
    for (int i = 0; i < g->total_cells; i++) {
        g->head[i] = -1;
    }

    //insertar cada boid en su celda correspondiente
    for (int i = 0; i < cfg->num_boids; i++) {
        //desplazar la posición para que siempre sea positiva y dividir por el tamaño de celda
        int cx = (int)((boids[i].position.x + g->world_offset) / g->cell_size);
        int cy = (int)((boids[i].position.y + g->world_offset) / g->cell_size);
        int cz = (int)((boids[i].position.z + g->world_offset) / g->cell_size);

        //limitar por seguridad si un boid se sale un poco del límite antes de rebotar
        if (cx < 0) cx = 0; else if (cx >= g->cells_x) cx = g->cells_x - 1;
        if (cy < 0) cy = 0; else if (cy >= g->cells_y) cy = g->cells_y - 1;
        if (cz < 0) cz = 0; else if (cz >= g->cells_z) cz = g->cells_z - 1;

        //convertir indice 3d a 1d
        int cell_idx = cx + cy * g->cells_x + cz * g->cells_x * g->cells_y;
        
        //el boid apunta al boid que antes estaba de head y este se convierte en el nuevo head
        g->next[i] = g->head[cell_idx];
        g->head[cell_idx] = i;
    }
}

int main() {
    config cfg = {
        .num_boids = 10000,
        .min_speed = 1.0f,
        .max_speed = 5.0f,
        .vision_radius = 10.0f,
        .blind_angle = 1.0f, //radianes (aprox 57 grados)
        .cos_blind_angle = 0.0f, // pendiente: precalcular con cosf(blind_angle)
        .world_size = 200,
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

    //generación del grid
    spatial_grid grid = spatial_grid_init(&cfg);

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

        //rellenar grid
        grid_build(&grid, boids, &cfg);
        //calculo de reglas (separacion, alineacion, cohesion)
        boids_compute_accelerations(boids, &cfg, &grid);
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

            //dibujo el boid
            DrawCylinderEx(base, tip, 0.18f, 0.0f, 8, SKYBLUE);
        }

        EndMode3D();

        DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();

    //librerar arrays del grid
    free(grid.head);
    free(grid.next);
    //liberar memoria del array de boids
    free(boids);

    return 0;
}
