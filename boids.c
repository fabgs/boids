#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "vec3.h"
#include "raylib.h"
#include "raymath.h"

#define RAYGUI_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#include "raygui.h"
#pragma GCC diagnostic pop

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
    int allocated_cells; // para saber cuánta memoria tenemos reservada
    float cell_size;
    float world_offset; // pasar de -mundo, +mundo a 0, mundo*2
} spatial_grid;

// estructura que representa la configuración para el sistema de boids
typedef struct{
    int num_boids;
    int max_boids; // limite de boids totales
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
    float wander_weight; // peso de aleatoriedad de movimiento
    float urgency_multiplier; // para que aceleren por distintos factores
    float agility; // facilidad para girar y frenar
    float noise_table[1024];
    boundary_mode boundary_mode;
} config;

const char *instancingVS =
    "#version 330\n"
    "in vec3 vertexPosition;\n"
    "in vec3 vertexNormal;\n"
    "in mat4 instanceTransform;\n"
    "uniform mat4 mvp;\n"
    "out vec3 fragNormal;\n"
    "void main()\n"
    "{\n"
    "    fragNormal = normalize(mat3(instanceTransform) * vertexNormal);\n"
    "    gl_Position = mvp * instanceTransform * vec4(vertexPosition, 1.0);\n"
    "}\n";

const char *instancingFS =
    "#version 330\n"
    "uniform vec4 colDiffuse;\n"
    "out vec4 finalColor;\n"
    "void main()\n"
    "{\n"
    "    finalColor = colDiffuse;\n"
    "}\n";
// genera un float aleatorio entre dos valores
float rand_range(float a, float b){
    return a + (rand() / (float)RAND_MAX) * (b-a);
}

// para cambiar el comportamiento con respecto a los limites
void simulation_handle_input(config *cfg, bool *is_paused) {
    if (IsKeyPressed(KEY_B)) {
        if (cfg->boundary_mode == BOUNDARY_BOUNCE) {
            cfg->boundary_mode = BOUNDARY_WRAP;
        } else {
            cfg->boundary_mode = BOUNDARY_BOUNCE;
        }
    }

    if (IsKeyPressed(KEY_P)) {
        *is_paused = !*is_paused;
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

// devuelve true si la distancia entre boids es igual o menor que el radio de vision limite y si no esta en punto ciego
bool boids_are_neighbors(const boid *a, const boid *b, const config *cfg) {
    if (a == b) {
        return false;
    }

    vec3 offset = boids_offset(a, b, cfg);
    // distancia al cuadrado
    float distance2 = vec3_length2(offset);
    // devuelve si la distancia es menor o igual que el radio de vision y si la distancia es mayor a 0
    return (distance2 <= cfg->vision_radius * cfg->vision_radius && distance2 > 0.0001f);
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
    for (int i = 0; i < cfg->max_boids; i++){
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
void boids_compute_accelerations(boid *boids, const config *cfg, const spatial_grid *g, float time_sec) {
    #pragma omp parallel for
    for (int i = 0; i < cfg->num_boids; i++) {
        boid *observer = &boids[i];
        int total_neighbors = 0; // para saber si estoy solo y frenar
        int visual_neighbors = 0; // para saber a quien seguir

        //direccion a la que mira el boid
        float speed2 = vec3_length2(observer->velocity);
        float current_speed = sqrtf(speed2);
        vec3 fwd_dir = {0, 0, 0};
        if (current_speed > 0.001f) {
            fwd_dir.x = observer->velocity.x / current_speed;
            fwd_dir.y = observer->velocity.y / current_speed;
            fwd_dir.z = observer->velocity.z / current_speed;
        }

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
                                vec3 offset_to_other = boids_offset(observer, other, cfg); 
                                float distance = sqrtf(vec3_length2(offset_to_other));
                                
                                // separacion tiene en cuenta todo al rededor
                                vec3 push_force = vec3_scale(offset_to_other, -1.0f / distance);
                                separation = vec3_add(separation, push_force);
                                total_neighbors++;

                                // angulo ciego para cohesion y alineación
                                float dir_x = offset_to_other.x / distance;
                                float dir_y = offset_to_other.y / distance;
                                float dir_z = offset_to_other.z / distance;
                                float dot_prod = (fwd_dir.x * dir_x) + (fwd_dir.y * dir_y) + (fwd_dir.z * dir_z);
                                
                                if (dot_prod >= cfg->cos_blind_angle) {
                                    alignment = vec3_add(alignment, other->velocity);
                                    cohesion = vec3_add(cohesion, offset_to_other);
                                    visual_neighbors++;
                                }
                            }
                        }
                        
                        //siguiente boid de esta celda
                        j = g->next[j]; 
                    }
                }
            }
        }

        // genero del ruido para el movimiento aleatorio
        float t = time_sec * 1.5f; // Velocidad del aleteo (mas bajo = mas suave)
        int idx1 = (int)t;
        int idx2 = idx1 + 1;
        float fract = t - (float)idx1; // Porcentaje de transición entre casillas

        // Eje X
        int x1 = (i * 3 + idx1) % 1024;
        int x2 = (i * 3 + idx2) % 1024;
        float noise_x = cfg->noise_table[x1] * (1.0f - fract) + cfg->noise_table[x2] * fract;

        // Eje Y
        int y1 = (i * 7 + idx1 + 333) % 1024;
        int y2 = (i * 7 + idx2 + 333) % 1024;
        float noise_y = cfg->noise_table[y1] * (1.0f - fract) + cfg->noise_table[y2] * fract;

        // Eje Z
        int z1 = (i * 11 + idx1 + 666) % 1024;
        int z2 = (i * 11 + idx2 + 666) % 1024;
        float noise_z = cfg->noise_table[z1] * (1.0f - fract) + cfg->noise_table[z2] * fract;

        //direccion deseada
        vec3 desired_dir = {
            noise_x * cfg->wander_weight,
            noise_y * cfg->wander_weight,
            noise_z * cfg->wander_weight
        };

        // si hay vecinos visible se aplica alineacion y cohesion
        if (visual_neighbors > 0) {
            alignment = vec3_scale(alignment, 1.0f / visual_neighbors);
            cohesion = vec3_scale(cohesion, 1.0f / visual_neighbors);
        }

        if (total_neighbors > 0) desired_dir = vec3_add(desired_dir, vec3_scale(separation, cfg->separation_weight));
        if (visual_neighbors > 0) desired_dir = vec3_add(desired_dir, vec3_scale(alignment, cfg->alignment_weight));
        if (visual_neighbors > 0) desired_dir = vec3_add(desired_dir, vec3_scale(cohesion, cfg->cohesion_weight));

        float dir_len = sqrtf(vec3_length2(desired_dir));
        if (dir_len > 0.001f) {
            // Normalizamos la dirección para saber hacia donde ir
            vec3 dir_norm = vec3_scale(desired_dir, 1.0f / dir_len);

            // Usamos el multiplicador desde la configuración
            float urgency = dir_len * cfg->urgency_multiplier;
            
            // Limitamos la urgencia entre 0.0 y 1.0
            if (urgency > 1.0f) urgency = 1.0f;
            
            // Interpolar entre la velocidad de relax y la de pánico
            float target_speed = cfg->min_speed + (cfg->max_speed - cfg->min_speed) * urgency;

            // Calculamos el vector de velocidad deseado final
            vec3 desired_vel = vec3_scale(dir_norm, target_speed);
            
            // Aplicamos el Steering
            observer->acceleration.x += (desired_vel.x - observer->velocity.x) * cfg->agility;
            observer->acceleration.y += (desired_vel.y - observer->velocity.y) * cfg->agility;
            observer->acceleration.z += (desired_vel.z - observer->velocity.z) * cfg->agility;
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
    g.allocated_cells = g.total_cells;
    g.world_offset = cfg->world_size;

    g.head = malloc(g.allocated_cells * sizeof(int));
    g.next = malloc(cfg->max_boids * sizeof(int));
    
    return g;
}

// actualizador de posiciones de boids en el grid
void grid_build(spatial_grid *g, const boid *boids, const config *cfg) {
    // Recalcular parametros por si el usuario ha movido el slider
    g->cell_size = cfg->vision_radius;
    g->cells_x = (int)((2.0f * cfg->world_size) / g->cell_size) + 1;
    g->cells_y = g->cells_x;
    g->cells_z = g->cells_x;
    int new_total = g->cells_x * g->cells_y * g->cells_z;
    g->world_offset = cfg->world_size;

    // si el nuevo mundo necesita más celdas de las que tenemos se amplia memoria
    if (new_total > g->allocated_cells) {
        g->head = realloc(g->head, new_total * sizeof(int));
        g->allocated_cells = new_total;
    }
    g->total_cells = new_total;

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
        .num_boids = 50000,
        .max_boids = 100000,
        .min_speed = 5.0f,
        .max_speed = 15.0f,
        .vision_radius = 10.0f,
        .blind_angle = 1.0f, //radianes (aprox 57 grados)
        .cos_blind_angle = 0.0f,
        .world_size = 200,
        .separation_weight = 0.5f,
        .alignment_weight = 0.3f,
        .cohesion_weight = 0.5f,
        .wander_weight = 0.1f,
        .urgency_multiplier = 0.5f,
        .agility = 5.0f,
        .boundary_mode = BOUNDARY_WRAP,
    };

    //calculo del coseno del angulo muerto
    cfg.cos_blind_angle = cosf(PI - cfg.blind_angle);

    // calculo de una tabla de aleatoriedad para no hacer calculos con el ruido
    for (int i = 0; i < 1024; i++) {
        cfg.noise_table[i] = rand_range(-1.0f, 1.0f);
    }

    // semilla aleatoria
    srand(1);

    // array dinamico para los boids
    boid *boids = malloc(sizeof(boid) * cfg.max_boids);

    //check null de la asignacion anterior
    if (boids == NULL) return 1;
    

    //generación inicial de los boids
    boids_init(boids, &cfg);

    //generación del grid
    spatial_grid grid = spatial_grid_init(&cfg);

    bool show_ui = true;
    bool is_paused = false;

    InitWindow(1920, 1080, "Boids 3D");
    SetTargetFPS(60);

    //estilos para la ui

    // color del fondo
    GuiSetStyle(DEFAULT, BACKGROUND_COLOR, 0x15171CFF);
    GuiSetStyle(DEFAULT, LINE_COLOR, 0x2A2F3AFF);

    // Colores Base
    GuiSetStyle(DEFAULT, BASE_COLOR_NORMAL, 0x20242BFF); 
    GuiSetStyle(DEFAULT, BASE_COLOR_FOCUSED, 0x2A2F3AFF);
    GuiSetStyle(DEFAULT, BASE_COLOR_PRESSED, 0x5BB2D9FF);

    // Colores de los Bordes
    GuiSetStyle(DEFAULT, BORDER_COLOR_NORMAL, 0x2A2F3AFF); 
    GuiSetStyle(DEFAULT, BORDER_COLOR_FOCUSED, 0x5BB2D9FF);
    GuiSetStyle(DEFAULT, BORDER_COLOR_PRESSED, 0x0492C7FF);

    // Colores del Texto
    GuiSetStyle(DEFAULT, TEXT_COLOR_NORMAL, 0x8FA4B5FF);
    GuiSetStyle(DEFAULT, TEXT_COLOR_FOCUSED, 0xC9EFFEFF);
    GuiSetStyle(DEFAULT, TEXT_COLOR_PRESSED, 0xC9EFFEFF);

    if (show_ui) EnableCursor();
    else DisableCursor();

    //creo un cono para un boid
    Mesh boidMesh = GenMeshCone(0.18f, 0.6f, 8);

    //asigno material y pinto de azul
    Material boidMaterial = LoadMaterialDefault();
    boidMaterial.maps[MATERIAL_MAP_ALBEDO].color = SKYBLUE;

    //shader para visualizacion de los boids
    Shader shader = LoadShaderFromMemory(instancingVS, instancingFS);

    shader.locs[SHADER_LOC_MATRIX_MVP] = GetShaderLocation(shader, "mvp");
    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocationAttrib(shader, "instanceTransform");

    boidMaterial.shader = shader;

    //reserva de memoria
    Matrix *boidTransforms = malloc(sizeof(Matrix) * cfg.max_boids);

    Camera3D camera = {0};
    camera.position = (Vector3){cfg.world_size+10.0f, cfg.world_size+10.0f, cfg.world_size+10.0f};
    camera.target = (Vector3){0.0f, 0.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE; 

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        simulation_handle_input(&cfg, &is_paused);

        //UpdateCamera(&camera, CAMERA_FREE); //camara antes
        //velocidad de la camara
        float base_speed = 150.0f;
        
        // Sprint al mantener Control
        if (IsKeyDown(KEY_LEFT_CONTROL)) base_speed *= 3.0f; 
        
        float cam_speed = base_speed * dt;

        Vector3 movement = {0};
        if (IsKeyDown(KEY_W)) movement.x += cam_speed;
        if (IsKeyDown(KEY_S)) movement.x -= cam_speed;
        if (IsKeyDown(KEY_D)) movement.y += cam_speed;
        if (IsKeyDown(KEY_A)) movement.y -= cam_speed;
        
        // espacio y shift para subir y bajar
        if (IsKeyDown(KEY_SPACE)) movement.z += cam_speed; 
        if (IsKeyDown(KEY_LEFT_SHIFT)) movement.z -= cam_speed; 

        // desactivar ui
        if (IsKeyPressed(KEY_TAB)) {
            show_ui = !show_ui;
            if (show_ui) EnableCursor();
            else DisableCursor();
        }

        // rotación con el ratón (solo si la ui esta oculta)
        Vector3 rotation = {0};
        if (!show_ui) {
            Vector2 mouseDelta = GetMouseDelta();
            rotation.x = mouseDelta.x * 0.1f;
            rotation.y = mouseDelta.y * 0.1f;
        }

        // zoom con la rueda
        float zoom = GetMouseWheelMove() * 2.0f;

        // se pasa el calculo a update camera
        UpdateCameraPro(&camera, movement, rotation, zoom);

        if (!is_paused) {
            //rellenar grid
            grid_build(&grid, boids, &cfg);
            //tiempo actual
            float time_sec = (float)GetTime();
            //calculo de reglas (separacion, alineacion, cohesion)
            boids_compute_accelerations(boids, &cfg, &grid, time_sec);
            //actualizacion de boids
            boids_update(boids, &cfg, dt);
        }

        #pragma omp parallel for
        for (int i = 0; i < cfg.num_boids; i++) {
            boid *b = &boids[i];
            
            //la dirección en la que vuela el boid normalizada
            vec3 dir_v = vec3_normalize(b->velocity);
            Vector3 dir = { dir_v.x, dir_v.y, dir_v.z };
            
            //los conos de Raylib se generan mirando hacia arriba
            Vector3 up_default = { 0.0f, 1.0f, 0.0f };
            
            //se calcula como rotar desde arriba hasta donde mira el boid
            Quaternion q = QuaternionFromVector3ToVector3(up_default, dir);
            Matrix rot = QuaternionToMatrix(q);
            
            //se calcula la matriz de su posicion en el mundo
            Matrix trans = MatrixTranslate(b->position.x, b->position.y, b->position.z);
            
            //multiplicacion de las matrices, primero se rota y luego se translada
            boidTransforms[i] = MatrixMultiply(rot, trans);
        }

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

        //dibujado directo de todos los boids
        DrawMeshInstanced(boidMesh, boidMaterial, boidTransforms, cfg.num_boids);

        EndMode3D();

        if (show_ui) {
            int pW = 340;
            int pH = 450;
            
            // Ancho total de pantalla, menos el ancho del panel, menos 10 píxeles de margen
            int pX = GetScreenWidth() - pW - 10; 
            int pY = 10; // Pegado arriba
            
            GuiPanel((Rectangle){ (float)pX, (float)pY, (float)pW, (float)pH }, "Parameters (TAB to fly)");

            // Coordenadas base para los sliders relativas al panel
            int sX = pX + 110;
            int sY = pY + 40;
            int sW = 160;
            int sH = 15;
            int space = 28;

            // Número de Boids
            float active_boids = (float)cfg.num_boids;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Num Boids", TextFormat("%d", cfg.num_boids), &active_boids, 100.0f, (float)cfg.max_boids);
            cfg.num_boids = (int)active_boids;

            // Tamaño del Mundo
            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "World Size", TextFormat("%.0f", cfg.world_size), &cfg.world_size, 50.0f, 400.0f);

            // Comportamiento de los boids
            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Min Speed", TextFormat("%.1f", cfg.min_speed), &cfg.min_speed, 1.0f, 20.0f);

            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Max Speed", TextFormat("%.1f", cfg.max_speed), &cfg.max_speed, 10.0f, 50.0f);

            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Vision Radius", TextFormat("%.1f", cfg.vision_radius), &cfg.vision_radius, 1.0f, 50.0f);
            
            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Blind Angle", TextFormat("%.2f rad", cfg.blind_angle), &cfg.blind_angle, 0.0f, PI);
            cfg.cos_blind_angle = cosf(PI - cfg.blind_angle); 

            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Separation", TextFormat("%.2f", cfg.separation_weight), &cfg.separation_weight, 0.0f, 2.0f);
            
            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Alignment", TextFormat("%.2f", cfg.alignment_weight), &cfg.alignment_weight, 0.0f, 2.0f);
            
            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Cohesion", TextFormat("%.2f", cfg.cohesion_weight), &cfg.cohesion_weight, 0.0f, 2.0f);
            
            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Wander", TextFormat("%.2f", cfg.wander_weight), &cfg.wander_weight, 0.0f, 1.0f);
            
            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Urgency", TextFormat("%.2f", cfg.urgency_multiplier), &cfg.urgency_multiplier, 0.01f, 1.0f);
            
            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Agility", TextFormat("%.1f", cfg.agility), &cfg.agility, 0.1f, 10.0f);

            // Comportamiento de bordes
            sY += space;
            bool isWrap = (cfg.boundary_mode == BOUNDARY_WRAP);
            GuiCheckBox((Rectangle){ (float)sX, (float)sY, 15, 15 }, "Boundary: WRAP (B to swap)", &isWrap);
            cfg.boundary_mode = isWrap ? BOUNDARY_WRAP : BOUNDARY_BOUNCE;

            // pausar/reanudar la simulacion
            sY += space;
            GuiCheckBox((Rectangle){ (float)sX, (float)sY, 15, 15 }, "Paused (P to toggle)", &is_paused);
        }

        DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();

    //liberar memoria del mesh de boids
    free(boidTransforms);
    UnloadMesh(boidMesh);
    //librerar arrays del grid
    free(grid.head);
    free(grid.next);
    //liberar memoria del array de boids
    free(boids);

    return 0;
}
