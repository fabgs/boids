#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "vec3.h"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#define RAYGUI_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#include "raygui.h"
#pragma GCC diagnostic pop

typedef enum {
    BOUNDARY_BOUNCE,
    BOUNDARY_WRAP,
    BOUNDARY_STEER // las paredes se esquivan con la misma fuerza de evasion que los obstaculos
} boundary_mode;

// modo de la camara: libre o siguiendo a un boid concreto
typedef enum {
    CAM_MODE_FREE,
    CAM_MODE_FIRST_PERSON,
    CAM_MODE_THIRD_PERSON
} camera_mode;

// estructura que representa un boid, posición, velocidad y aceleración
typedef struct {
    vec3 position;
    vec3 velocity;
    vec3 acceleration;
} boid;

// figura del obstaculo
typedef enum {
    OBSTACLE_SPHERE,
    OBSTACLE_BOX,
    OBSTACLE_TORUS,
    OBSTACLE_CYLINDER,
    OBSTACLE_TYPE_COUNT
} obstacle_type;

// obstaculo colocado en tiempo real que los boids deben esquivar
typedef struct {
    obstacle_type type;
    vec3 position;
    float radius; // esfera/cilindro: radio, cubo: semi-arista, donut: radio del anillo
    float yaw;    // rotacion alrededor del eje Y (radianes)
    float pitch;  // rotacion alrededor del eje X (radianes), aplicada antes que el yaw
} obstacle;

#define MAX_OBSTACLES 64
// separacion extra que los boids intentan mantener con la superficie del obstaculo
#define OBSTACLE_CLEARANCE 1.0f
// margen de contacto fisico al resolver colisiones directas
#define OBSTACLE_CONTACT_MARGIN 0.3f
// segundos de trayectoria que un boid anticipa al buscar impactos
#define OBSTACLE_LOOKAHEAD_TIME 1.5f
// grosor del tubo del donut respecto al radio del anillo
#define TORUS_TUBE_RATIO 0.35f

// radio de la esfera que envuelve la figura (para descartes rapidos y picking)
float obstacle_bounding_radius(const obstacle *o) {
    switch (o->type) {
        case OBSTACLE_BOX: return o->radius * 1.7320508f; // diagonal del cubo
        case OBSTACLE_TORUS: return o->radius * (1.0f + TORUS_TUBE_RATIO);
        case OBSTACLE_CYLINDER: return o->radius * 1.4142136f;
        default: return o->radius;
    }
}

// distancia con signo del punto a la superficie de la figura (negativa por dentro)
float obstacle_sdf(const obstacle *o, vec3 p) {
    vec3 q = vec3_sub(p, o->position);
    float r = o->radius;

    // pasar el punto al espacio local de la figura deshaciendo su rotacion (yaw inverso y luego pitch inverso)
    if (o->type != OBSTACLE_SPHERE && (o->yaw != 0.0f || o->pitch != 0.0f)) {
        float cy = cosf(o->yaw), sy = sinf(o->yaw);
        float cp = cosf(o->pitch), sp = sinf(o->pitch);
        vec3 t = { cy * q.x - sy * q.z, q.y, sy * q.x + cy * q.z };
        q = (vec3){ t.x, cp * t.y + sp * t.z, -sp * t.y + cp * t.z };
    }

    switch (o->type) {
        case OBSTACLE_BOX: {
            float dx = fabsf(q.x) - r, dy = fabsf(q.y) - r, dz = fabsf(q.z) - r;
            float ox = fmaxf(dx, 0.0f), oy = fmaxf(dy, 0.0f), oz = fmaxf(dz, 0.0f);
            return sqrtf(ox*ox + oy*oy + oz*oz) + fminf(fmaxf(dx, fmaxf(dy, dz)), 0.0f);
        }
        case OBSTACLE_TORUS: {
            // anillo en el plano xy con el agujero mirando a z, igual que la malla de raylib
            float ring = sqrtf(q.x*q.x + q.y*q.y) - r;
            return sqrtf(ring*ring + q.z*q.z) - r * TORUS_TUBE_RATIO;
        }
        case OBSTACLE_CYLINDER: {
            float dr = sqrtf(q.x*q.x + q.z*q.z) - r;
            float dy = fabsf(q.y) - r;
            float rr = fmaxf(dr, 0.0f), ry = fmaxf(dy, 0.0f);
            return fminf(fmaxf(dr, dy), 0.0f) + sqrtf(rr*rr + ry*ry);
        }
        default: // esfera
            return sqrtf(vec3_length2(q)) - r;
    }
}

// normal aproximada de la superficie por gradiente numerico del sdf
vec3 obstacle_normal(const obstacle *o, vec3 p) {
    const float e = 0.05f;
    return vec3_normalize((vec3){
        obstacle_sdf(o, (vec3){p.x + e, p.y, p.z}) - obstacle_sdf(o, (vec3){p.x - e, p.y, p.z}),
        obstacle_sdf(o, (vec3){p.x, p.y + e, p.z}) - obstacle_sdf(o, (vec3){p.x, p.y - e, p.z}),
        obstacle_sdf(o, (vec3){p.x, p.y, p.z + e}) - obstacle_sdf(o, (vec3){p.x, p.y, p.z - e})
    });
}

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
    float avoid_weight; // peso de la fuerza de evasion de obstaculos y paredes
    float noise_table[1024];
    boundary_mode boundary_mode;
    bool show_grid; // debug: dibujar las celdas ocupadas del grid espacial
    bool show_vision; // debug: dibujar radio de vision y angulo ciego del boid seguido
    bool show_world_bounds; // debug: dibujar el cubo de limites del mundo
    bool show_fps; // debug: mostrar contador de fps
    bool show_crosshair; // debug: mostrar mirilla central al volar con la ui oculta
    bool show_speed_heatmap; // debug: colorear boids segun su velocidad actual
} config;

// el color por instancia viaja de contrabando en la fila inferior de la matriz (m3, m7, m11),
// que en una transformacion afin siempre es (0, 0, 0, 1); hay que limpiarla antes de proyectar
const char *instancingVS =
    "#version 330\n"
    "in vec3 vertexPosition;\n"
    "in vec3 vertexNormal;\n"
    "in mat4 instanceTransform;\n"
    "uniform mat4 mvp;\n"
    "out vec3 fragNormal;\n"
    "out vec3 instanceColor;\n"
    "void main()\n"
    "{\n"
    "    instanceColor = vec3(instanceTransform[0].w, instanceTransform[1].w, instanceTransform[2].w);\n"
    "    mat4 transform = instanceTransform;\n"
    "    transform[0].w = 0.0;\n"
    "    transform[1].w = 0.0;\n"
    "    transform[2].w = 0.0;\n"
    "    fragNormal = normalize(mat3(transform) * vertexNormal);\n"
    "    gl_Position = mvp * transform * vec4(vertexPosition, 1.0);\n"
    "}\n";

const char *instancingFS =
    "#version 330\n"
    "in vec3 instanceColor;\n"
    "out vec4 finalColor;\n"
    "void main()\n"
    "{\n"
    "    finalColor = vec4(instanceColor, 1.0);\n"
    "}\n";
// genera un float aleatorio entre dos valores
float rand_range(float a, float b){
    return a + (rand() / (float)RAND_MAX) * (b-a);
}

// para cambiar el comportamiento con respecto a los limites
void simulation_handle_input(config *cfg, bool *is_paused) {
    if (IsKeyPressed(KEY_B)) {
        // ciclo BOUNCE -> WRAP -> STEER
        cfg->boundary_mode = (cfg->boundary_mode + 1) % 3;
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
        // BOUNCE, y red de seguridad del modo STEER si la evasion no frena a tiempo
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

// reinicia la simulación al estado cero re-sembrando el rng con la semilla dada
void boids_reset(boid *boids, config *cfg, int seed, float *sim_time){
    srand(seed);
    for (int i = 0; i < 1024; i++) {
        cfg->noise_table[i] = rand_range(-1.0f, 1.0f);
    }
    boids_init(boids, cfg);
    *sim_time = 0.0f;
}

// fuerza un float a un rango valido (NaN o menor cae al minimo)
float clamp_float(float v, float min_v, float max_v) {
    if (!(v >= min_v)) return min_v;
    if (v > max_v) return max_v;
    return v;
}

// asegura que una config cargada de disco no tenga valores que rompan la simulacion (division por cero, overflow del grid, NaN)
// los rangos son los mismos que permiten los sliders de la ui
void config_sanitize(config *cfg) {
    if (cfg->num_boids < 0) cfg->num_boids = 0;
    if (cfg->num_boids > cfg->max_boids) cfg->num_boids = cfg->max_boids;
    cfg->world_size = clamp_float(cfg->world_size, 50.0f, 400.0f);
    cfg->min_speed = clamp_float(cfg->min_speed, 1.0f, 20.0f);
    cfg->max_speed = clamp_float(cfg->max_speed, 10.0f, 50.0f);
    cfg->vision_radius = clamp_float(cfg->vision_radius, 1.0f, 50.0f);
    cfg->blind_angle = clamp_float(cfg->blind_angle, 0.0f, PI);
    cfg->separation_weight = clamp_float(cfg->separation_weight, 0.0f, 2.0f);
    cfg->alignment_weight = clamp_float(cfg->alignment_weight, 0.0f, 2.0f);
    cfg->cohesion_weight = clamp_float(cfg->cohesion_weight, 0.0f, 2.0f);
    cfg->wander_weight = clamp_float(cfg->wander_weight, 0.0f, 1.0f);
    cfg->urgency_multiplier = clamp_float(cfg->urgency_multiplier, 0.01f, 1.0f);
    cfg->agility = clamp_float(cfg->agility, 0.1f, 10.0f);
    cfg->avoid_weight = clamp_float(cfg->avoid_weight, 0.0f, 3.0f);
    cfg->cos_blind_angle = cosf(PI - cfg->blind_angle);
}

#define STATE_MAGIC "BSTA"
// v7: los obstaculos incluyen rotacion yaw/pitch (cambia sizeof(obstacle))
#define STATE_VERSION 7

// cabecera del archivo de snapshot guardado
typedef struct {
    char magic[4];
    int version;
    float sim_time;
} state_header;

// vuelca a un archivo binario la config completa (para retomar tambien los parametros), la posicion/velocidad/aceleracion de cada boid activo y los obstaculos colocados
bool boids_save_state(const boid *boids, const config *cfg, const obstacle *obstacles, int num_obstacles, float sim_time, const char *filename) {
    FILE *f = fopen(filename, "wb");
    if (f == NULL) return false;

    state_header header = { .version = STATE_VERSION, .sim_time = sim_time };
    memcpy(header.magic, STATE_MAGIC, 4);

    bool ok = fwrite(&header, sizeof(header), 1, f) == 1
           && fwrite(cfg, sizeof(config), 1, f) == 1
           && fwrite(boids, sizeof(boid), (size_t)cfg->num_boids, f) == (size_t)cfg->num_boids
           && fwrite(&num_obstacles, sizeof(int), 1, f) == 1
           && (num_obstacles == 0 || fwrite(obstacles, sizeof(obstacle), (size_t)num_obstacles, f) == (size_t)num_obstacles);

    fclose(f);
    return ok;
}

// restaura la config, el estado de los boids y los obstaculos desde un archivo generado por boids_save_state
bool boids_load_state(boid *boids, config *cfg, float *sim_time, obstacle *obstacles, int *num_obstacles, const char *filename) {
    FILE *f = fopen(filename, "rb");
    if (f == NULL) return false;

    state_header header;
    config loaded_cfg;
    if (fread(&header, sizeof(header), 1, f) != 1 || memcmp(header.magic, STATE_MAGIC, 4) != 0 || header.version != STATE_VERSION
        || fread(&loaded_cfg, sizeof(config), 1, f) != 1
        || loaded_cfg.num_boids < 0 || loaded_cfg.num_boids > cfg->max_boids) {
        fclose(f);
        return false;
    }

    bool ok = fread(boids, sizeof(boid), (size_t)loaded_cfg.num_boids, f) == (size_t)loaded_cfg.num_boids;

    // obstaculos guardados a continuacion de los boids
    int loaded_obstacles = 0;
    ok = ok && fread(&loaded_obstacles, sizeof(int), 1, f) == 1
            && loaded_obstacles >= 0 && loaded_obstacles <= MAX_OBSTACLES;
    if (ok && loaded_obstacles > 0) {
        ok = fread(obstacles, sizeof(obstacle), (size_t)loaded_obstacles, f) == (size_t)loaded_obstacles;
    }
    fclose(f);

    if (!ok) return false;

    for (int i = 0; i < loaded_obstacles; i++) {
        obstacles[i].radius = clamp_float(obstacles[i].radius, 0.5f, 100.0f);
        obstacles[i].yaw = clamp_float(obstacles[i].yaw, -2.0f * PI, 2.0f * PI);
        obstacles[i].pitch = clamp_float(obstacles[i].pitch, -2.0f * PI, 2.0f * PI);
        if ((int)obstacles[i].type < 0 || obstacles[i].type >= OBSTACLE_TYPE_COUNT) obstacles[i].type = OBSTACLE_SPHERE;
    }
    *num_obstacles = loaded_obstacles;

    // el limite de memoria reservada es del programa actual, no el guardado en el snapshot
    int max_boids = cfg->max_boids;
    *cfg = loaded_cfg;
    cfg->max_boids = max_boids;

    // un snapshot corrupto no debe poder colar valores que rompan la simulacion
    config_sanitize(cfg);

    *sim_time = header.sim_time;
    return true;
}

#define FILE_LIST_MAX_COUNT 64
#define FILE_LIST_MAX_NAME 64

// convierte el modo de borde a texto para guardarlo en el archivo
const char *boundary_mode_to_string(boundary_mode mode) {
    if (mode == BOUNDARY_WRAP) return "WRAP";
    if (mode == BOUNDARY_STEER) return "STEER";
    return "BOUNCE";
}

// convierte el texto leido del archivo al modo de borde correspondiente
boundary_mode boundary_mode_from_string(const char *text) {
    if (strcmp(text, "WRAP") == 0) return BOUNDARY_WRAP;
    if (strcmp(text, "STEER") == 0) return BOUNDARY_STEER;
    return BOUNDARY_BOUNCE;
}

// exporta a un archivo de texto los parametros de la config ajustables desde la ui
bool config_save_preset(const config *cfg, const char *filepath) {
    FILE *f = fopen(filepath, "w");
    if (f == NULL) return false;

    fprintf(f, "num_boids=%d\n", cfg->num_boids);
    fprintf(f, "world_size=%f\n", cfg->world_size);
    fprintf(f, "min_speed=%f\n", cfg->min_speed);
    fprintf(f, "max_speed=%f\n", cfg->max_speed);
    fprintf(f, "vision_radius=%f\n", cfg->vision_radius);
    fprintf(f, "blind_angle=%f\n", cfg->blind_angle);
    fprintf(f, "separation_weight=%f\n", cfg->separation_weight);
    fprintf(f, "alignment_weight=%f\n", cfg->alignment_weight);
    fprintf(f, "cohesion_weight=%f\n", cfg->cohesion_weight);
    fprintf(f, "wander_weight=%f\n", cfg->wander_weight);
    fprintf(f, "urgency_multiplier=%f\n", cfg->urgency_multiplier);
    fprintf(f, "agility=%f\n", cfg->agility);
    fprintf(f, "avoid_weight=%f\n", cfg->avoid_weight);
    fprintf(f, "boundary_mode=%s\n", boundary_mode_to_string(cfg->boundary_mode));
    fprintf(f, "show_grid=%d\n", cfg->show_grid ? 1 : 0);
    fprintf(f, "show_vision=%d\n", cfg->show_vision ? 1 : 0);
    fprintf(f, "show_world_bounds=%d\n", cfg->show_world_bounds ? 1 : 0);
    fprintf(f, "show_fps=%d\n", cfg->show_fps ? 1 : 0);
    fprintf(f, "show_crosshair=%d\n", cfg->show_crosshair ? 1 : 0);
    fprintf(f, "show_speed_heatmap=%d\n", cfg->show_speed_heatmap ? 1 : 0);

    fclose(f);
    return true;
}

// sobreescribe la config actual con los valores leidos de un archivo de preset
bool config_load_preset(config *cfg, const char *filepath) {
    FILE *f = fopen(filepath, "r");
    if (f == NULL) return false;

    char key[64];
    char value[64];
    char line[128];

    while (fgets(line, sizeof(line), f) != NULL) {
        if (sscanf(line, "%63[^=]=%63s", key, value) != 2) continue;

        if (strcmp(key, "num_boids") == 0) cfg->num_boids = atoi(value);
        else if (strcmp(key, "world_size") == 0) cfg->world_size = (float)atof(value);
        else if (strcmp(key, "min_speed") == 0) cfg->min_speed = (float)atof(value);
        else if (strcmp(key, "max_speed") == 0) cfg->max_speed = (float)atof(value);
        else if (strcmp(key, "vision_radius") == 0) cfg->vision_radius = (float)atof(value);
        else if (strcmp(key, "blind_angle") == 0) cfg->blind_angle = (float)atof(value);
        else if (strcmp(key, "separation_weight") == 0) cfg->separation_weight = (float)atof(value);
        else if (strcmp(key, "alignment_weight") == 0) cfg->alignment_weight = (float)atof(value);
        else if (strcmp(key, "cohesion_weight") == 0) cfg->cohesion_weight = (float)atof(value);
        else if (strcmp(key, "wander_weight") == 0) cfg->wander_weight = (float)atof(value);
        else if (strcmp(key, "urgency_multiplier") == 0) cfg->urgency_multiplier = (float)atof(value);
        else if (strcmp(key, "agility") == 0) cfg->agility = (float)atof(value);
        else if (strcmp(key, "avoid_weight") == 0) cfg->avoid_weight = (float)atof(value);
        else if (strcmp(key, "boundary_mode") == 0) cfg->boundary_mode = boundary_mode_from_string(value);
        else if (strcmp(key, "show_grid") == 0) cfg->show_grid = (atoi(value) != 0);
        else if (strcmp(key, "show_vision") == 0) cfg->show_vision = (atoi(value) != 0);
        else if (strcmp(key, "show_world_bounds") == 0) cfg->show_world_bounds = (atoi(value) != 0);
        else if (strcmp(key, "show_fps") == 0) cfg->show_fps = (atoi(value) != 0);
        else if (strcmp(key, "show_crosshair") == 0) cfg->show_crosshair = (atoi(value) != 0);
        else if (strcmp(key, "show_speed_heatmap") == 0) cfg->show_speed_heatmap = (atoi(value) != 0);
    }

    fclose(f);

    // recalcula el coseno derivado y clampea todo a rangos validos
    // (un preset editado a mano podria colar valores que rompan la simulacion)
    config_sanitize(cfg);

    return true;
}

// convierte el tipo de figura a la palabra clave del archivo de mapa
const char *obstacle_type_to_string(obstacle_type type) {
    switch (type) {
        case OBSTACLE_BOX: return "box";
        case OBSTACLE_TORUS: return "torus";
        case OBSTACLE_CYLINDER: return "cylinder";
        default: return "sphere";
    }
}

// convierte la palabra clave leida del archivo al tipo de figura
obstacle_type obstacle_type_from_string(const char *text) {
    if (strcmp(text, "box") == 0) return OBSTACLE_BOX;
    if (strcmp(text, "torus") == 0) return OBSTACLE_TORUS;
    if (strcmp(text, "cylinder") == 0) return OBSTACLE_CYLINDER;
    return OBSTACLE_SPHERE;
}

// exporta los obstaculos actuales a un archivo de texto (una figura por linea)
bool obstacles_save_map(const obstacle *obstacles, int num_obstacles, const char *filepath) {
    FILE *f = fopen(filepath, "w");
    if (f == NULL) return false;

    for (int i = 0; i < num_obstacles; i++) {
        fprintf(f, "%s %f %f %f %f %f %f\n", obstacle_type_to_string(obstacles[i].type), obstacles[i].position.x, obstacles[i].position.y, obstacles[i].position.z, obstacles[i].radius,
                obstacles[i].yaw * RAD2DEG, obstacles[i].pitch * RAD2DEG);
    }

    fclose(f);
    return true;
}

// reemplaza los obstaculos actuales por los leidos de un archivo de mapa
bool obstacles_load_map(obstacle *obstacles, int *num_obstacles, const char *filepath) {
    FILE *f = fopen(filepath, "r");
    if (f == NULL) return false;

    char line[128];
    char kind[16];
    int count = 0;
    obstacle obs;

    while (count < MAX_OBSTACLES && fgets(line, sizeof(line), f) != NULL) {
        // los angulos (en grados) son opcionales para mantener compatibilidad con mapas antiguos
        float yaw_deg = 0.0f, pitch_deg = 0.0f;
        if (sscanf(line, "%15s %f %f %f %f %f %f", kind, &obs.position.x, &obs.position.y, &obs.position.z, &obs.radius, &yaw_deg, &pitch_deg) < 5) continue;
        obs.type = obstacle_type_from_string(kind);
        obs.radius = clamp_float(obs.radius, 0.5f, 100.0f);
        obs.yaw = clamp_float(yaw_deg, -360.0f, 360.0f) * DEG2RAD;
        obs.pitch = clamp_float(pitch_deg, -360.0f, 360.0f) * DEG2RAD;
        obstacles[count++] = obs;
    }

    fclose(f);
    *num_obstacles = count;
    return true;
}

// escanea una carpeta en busca de archivos con la extension dada y construye la lista de nombres y el texto del desplegable
// (usado tanto para los presets de configuracion como para los snapshots de simulacion)
int refresh_file_list(const char *dir, const char *extension, char names[][FILE_LIST_MAX_NAME], char *dropdown_text, int dropdown_text_size, const char *empty_message) {
    dropdown_text[0] = '\0';
    int count = 0;

    if (!DirectoryExists(dir)) return 0;

    FilePathList files = LoadDirectoryFilesEx(dir, extension, false);

    for (unsigned int i = 0; i < files.count && count < FILE_LIST_MAX_COUNT; i++) {
        const char *name = GetFileNameWithoutExt(files.paths[i]);

        if (count > 0) strncat(dropdown_text, ";", dropdown_text_size - strlen(dropdown_text) - 1);
        strncat(dropdown_text, name, dropdown_text_size - strlen(dropdown_text) - 1);

        strncpy(names[count], name, FILE_LIST_MAX_NAME - 1);
        names[count][FILE_LIST_MAX_NAME - 1] = '\0';
        count++;
    }

    UnloadDirectoryFiles(files);

    if (count == 0) {
        strncpy(dropdown_text, empty_message, dropdown_text_size - 1);
        dropdown_text[dropdown_text_size - 1] = '\0';
    }

    return count;
}

// actualización de las posiciones de los boids, recibe array de boids y configuración
void boids_update(boid *boids, const config *cfg, const obstacle *obstacles, int num_obstacles, float dt){
    for (int i = 0; i < cfg->num_boids; i++){
        boid *b = &boids[i];

        //aceleración modifica la velocidad en este frame
        b->velocity = vec3_add(b->velocity, vec3_scale(b->acceleration, dt));

        //forzar rapidez que no se pase de los limites
        b->velocity = vec3_clamp_length(b->velocity, cfg->min_speed, cfg->max_speed);

        //la velocidad hace que cambie la posición
        b->position = vec3_add(b->position, vec3_scale(b->velocity, dt));

        // contacto duro: si pese a la evasion un boid toca un obstaculo, desliza por su superficie
        for (int k = 0; k < num_obstacles; k++) {
            const obstacle *o = &obstacles[k];

            // descarte rapido con la esfera envolvente
            vec3 to_center = vec3_sub(o->position, b->position);
            float reach = obstacle_bounding_radius(o) + OBSTACLE_CONTACT_MARGIN;
            if (vec3_length2(to_center) > reach * reach) continue;

            float d = obstacle_sdf(o, b->position);
            if (d >= OBSTACLE_CONTACT_MARGIN) continue;

            vec3 n = obstacle_normal(o, b->position);
            b->position = vec3_add(b->position, vec3_scale(n, OBSTACLE_CONTACT_MARGIN - d));
            // se elimina la componente de velocidad que entra en el obstaculo
            float vn = vec3_dot(b->velocity, n);
            if (vn < 0.0f) b->velocity = vec3_sub(b->velocity, vec3_scale(n, vn));
        }

        // aplica comportamiento con el borde del mapa
        boid_apply_boundary(b, cfg);

        //vaciar aceleracion acumulada para el siguiente frame
        b->acceleration = (vec3){0,0,0};
    }
}

// evasion de entorno: anticipa impactos contra obstaculos (y paredes en modo STEER) mirando hacia
// delante, y devuelve un empuje lateral cuya magnitud crece con la inminencia del impacto
vec3 boid_compute_avoidance(const boid *b, vec3 fwd_dir, float current_speed, const config *cfg, const obstacle *obstacles, int num_obstacles) {
    vec3 avoidance = {0, 0, 0};

    // a mas velocidad, mas lejos anticipa el boid
    float lookahead = current_speed * OBSTACLE_LOOKAHEAD_TIME;
    if (lookahead < 0.001f) return avoidance;

    for (int k = 0; k < num_obstacles; k++) {
        const obstacle *o = &obstacles[k];

        // descarte rapido con la esfera envolvente
        vec3 to_center = vec3_sub(o->position, b->position);
        float reach = obstacle_bounding_radius(o) + OBSTACLE_CLEARANCE + lookahead;
        if (vec3_length2(to_center) > reach * reach) continue;

        // ya dentro del margen de seguridad: empuje por la normal a maxima urgencia
        float d0 = obstacle_sdf(o, b->position) - OBSTACLE_CLEARANCE;
        if (d0 < 0.0f) {
            avoidance = vec3_add(avoidance, obstacle_normal(o, b->position));
            continue;
        }

        // raymarch por la trayectoria: el propio sdf marca el mayor paso seguro
        float t = d0;
        bool hit = false;
        for (int step = 0; step < 10 && t <= lookahead; step++) {
            vec3 sample = vec3_add(b->position, vec3_scale(fwd_dir, t));
            float d = obstacle_sdf(o, sample) - OBSTACLE_CLEARANCE;
            if (d < 0.1f) { hit = true; break; }
            t += d;
        }
        if (!hit) continue;

        // empuje lateral: normal en el punto de impacto sin su componente sobre el avance
        vec3 n = obstacle_normal(o, vec3_add(b->position, vec3_scale(fwd_dir, t)));
        vec3 lateral = vec3_sub(n, vec3_scale(fwd_dir, vec3_dot(n, fwd_dir)));
        if (vec3_length2(lateral) < 0.0001f) {
            // impacto frontal perfecto: cualquier lateral perpendicular al avance vale
            vec3 ref = (fabsf(fwd_dir.y) < 0.99f) ? (vec3){0.0f, 1.0f, 0.0f} : (vec3){1.0f, 0.0f, 0.0f};
            lateral = vec3_cross(fwd_dir, ref);
        }
        lateral = vec3_normalize(lateral);

        // inminencia 0..1 segun lo cerca que quede el impacto dentro del alcance
        avoidance = vec3_add(avoidance, vec3_scale(lateral, 1.0f - t / lookahead));
    }

    // las paredes entran en el mismo sistema: empuje hacia dentro proporcional a la cercania,
    // sin los rebotes bruscos del modo BOUNCE
    if (cfg->boundary_mode == BOUNDARY_STEER) {
        float margin = (lookahead < cfg->world_size) ? lookahead : cfg->world_size;

        float pos[3] = { b->position.x, b->position.y, b->position.z };
        float push[3] = {0};
        for (int axis = 0; axis < 3; axis++) {
            float d_pos = cfg->world_size - pos[axis];
            float d_neg = pos[axis] + cfg->world_size;
            if (d_pos < margin) push[axis] -= 1.0f - clamp_float(d_pos, 0.0f, margin) / margin;
            if (d_neg < margin) push[axis] += 1.0f - clamp_float(d_neg, 0.0f, margin) / margin;
        }
        avoidance = vec3_add(avoidance, (vec3){ push[0], push[1], push[2] });
    }

    return avoidance;
}

// calcula las aceleraciones en base a las reglas de reynolds
void boids_compute_accelerations(boid *boids, const config *cfg, const spatial_grid *g, const obstacle *obstacles, int num_obstacles, float time_sec) {
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

        // evasion de obstaculos y paredes con su propio peso
        vec3 avoidance = boid_compute_avoidance(observer, fwd_dir, current_speed, cfg, obstacles, num_obstacles);
        desired_dir = vec3_add(desired_dir, vec3_scale(avoidance, cfg->avoid_weight));

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

// limite de celdas por eje para que el grid no pida gigas de memoria con mundos grandes y radios de vision pequeños
// (celdas mas grandes que el radio de vision siguen dando resultados correctos, solo son menos eficientes)
#define GRID_MAX_CELLS_PER_AXIS 128

// calcula tamaño de celda y numero de celdas por eje a partir de la config actual
void grid_compute_dims(spatial_grid *g, const config *cfg) {
    g->cell_size = cfg->vision_radius;

    float min_cell = (2.0f * cfg->world_size) / GRID_MAX_CELLS_PER_AXIS;
    if (g->cell_size < min_cell) g->cell_size = min_cell;

    g->cells_x = (int)((2.0f * cfg->world_size) / g->cell_size) + 1;
    g->cells_y = g->cells_x;
    g->cells_z = g->cells_x;
    g->world_offset = cfg->world_size;
}

spatial_grid spatial_grid_init(const config *cfg){
    spatial_grid g;
    grid_compute_dims(&g, cfg);
    g.total_cells = g.cells_x*g.cells_y*g.cells_z;
    g.allocated_cells = g.total_cells;

    g.head = malloc(g.allocated_cells * sizeof(int));
    g.next = malloc(cfg->max_boids * sizeof(int));
    
    return g;
}

// actualizador de posiciones de boids en el grid
void grid_build(spatial_grid *g, const boid *boids, const config *cfg) {
    // Recalcular parametros por si el usuario ha movido el slider
    grid_compute_dims(g, cfg);
    int new_total = g->cells_x * g->cells_y * g->cells_z;

    // si el nuevo mundo necesita más celdas de las que tenemos se amplia memoria
    if (new_total > g->allocated_cells) {
        int *new_head = realloc(g->head, new_total * sizeof(int));
        if (new_head == NULL) {
            fprintf(stderr, "boids: sin memoria para el grid espacial (%d celdas)\n", new_total);
            exit(1);
        }
        g->head = new_head;
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

// mismos planos de recorte que usa raylib por defecto
#define CAMERA_NEAR_PLANE 0.01f
#define CAMERA_FAR_PLANE 1000.0f
// esfera que envuelve el cono del boid para el culling
#define BOID_BOUNDING_RADIUS 0.7f
// color base de los boids (SKYBLUE) cuando el heatmap esta desactivado
#define BOID_BASE_COLOR (vec3){0.4f, 0.75f, 1.0f}
// color de los obstaculos colocados
#define OBSTACLE_COLOR (Color){ 225, 95, 70, 255 }

// los 6 planos (a,b,c,d) del volumen visible de la camara
typedef struct {
    float planes[6][4];
} frustum;

// extrae los planos del frustum de la matriz vista*proyeccion (metodo gribb-hartmann)
frustum frustum_from_view_projection(Matrix m) {
    float rows[4][4] = {
        {m.m0, m.m4, m.m8,  m.m12},
        {m.m1, m.m5, m.m9,  m.m13},
        {m.m2, m.m6, m.m10, m.m14},
        {m.m3, m.m7, m.m11, m.m15}
    };

    frustum f;
    for (int i = 0; i < 6; i++) {
        int axis = i / 2; // 0 = x (izq/der), 1 = y (abajo/arriba), 2 = z (cerca/lejos)
        float sign = (i % 2 == 0) ? 1.0f : -1.0f;

        for (int c = 0; c < 4; c++) {
            f.planes[i][c] = rows[3][c] + sign * rows[axis][c];
        }

        // normalizar el plano para que las distancias sean reales
        float len = sqrtf(f.planes[i][0] * f.planes[i][0] + f.planes[i][1] * f.planes[i][1] + f.planes[i][2] * f.planes[i][2]);
        if (len > 0.0f) {
            for (int c = 0; c < 4; c++) f.planes[i][c] /= len;
        }
    }

    return f;
}

// true si una esfera toca el volumen visible de la camara
bool frustum_contains_sphere(const frustum *f, vec3 center, float radius) {
    for (int i = 0; i < 6; i++) {
        float dist = f->planes[i][0] * center.x + f->planes[i][1] * center.y + f->planes[i][2] * center.z + f->planes[i][3];
        if (dist < -radius) return false;
    }
    return true;
}

// matriz mundo del boid: orienta el cono (+Y) hacia dir y lo coloca en pos
// construccion directa (formula de rodrigues) para evitar quaternion + multiplicacion de matrices
Matrix boid_transform(vec3 pos, vec3 dir) {
    // caso degenerado: mirar recto hacia abajo (rotacion de 180 grados sobre x)
    if (dir.y < -0.9999f) {
        return (Matrix){
            1.0f,  0.0f,  0.0f, pos.x,
            0.0f, -1.0f,  0.0f, pos.y,
            0.0f,  0.0f, -1.0f, pos.z,
            0.0f,  0.0f,  0.0f, 1.0f
        };
    }

    float k = 1.0f / (1.0f + dir.y);
    float kxz = k * dir.x * dir.z;

    return (Matrix){
        dir.y + k * dir.z * dir.z,  dir.x,  -kxz,                       pos.x,
        -dir.x,                     dir.y,  -dir.z,                     pos.y,
        -kxz,                       dir.z,  dir.y + k * dir.x * dir.x,  pos.z,
        0.0f,                       0.0f,   0.0f,                       1.0f
    };
}

// matriz mundo del obstaculo: escala la malla unitaria por su radio, la rota y la coloca en su posicion
Matrix obstacle_draw_transform(const obstacle *o) {
    Matrix m = MatrixScale(o->radius, o->radius, o->radius);
    // la malla del cilindro de raylib nace en su base: se centra antes de rotar
    if (o->type == OBSTACLE_CYLINDER) m.m13 -= o->radius;
    if (o->pitch != 0.0f) m = MatrixMultiply(m, MatrixRotateX(o->pitch));
    if (o->yaw != 0.0f) m = MatrixMultiply(m, MatrixRotateY(o->yaw));
    m.m12 += o->position.x;
    m.m13 += o->position.y;
    m.m14 += o->position.z;
    return m;
}

// paleta del heatmap: azul oscuro (lento) -> azul (media) -> celeste brillante (rapido)
vec3 speed_heatmap_color(float t) {
    const vec3 slow = {0.05f, 0.08f, 0.35f};
    const vec3 mid  = {0.15f, 0.35f, 0.95f};
    const vec3 fast = {0.85f, 0.97f, 1.00f};
    if (t < 0.5f) return vec3_lerp(slow, mid, t * 2.0f);
    return vec3_lerp(mid, fast, (t - 0.5f) * 2.0f);
}

// dibuja en modo debug solo las celdas del grid que contienen algun boid
void debug_draw_grid(const spatial_grid *g) {
    for (int cz = 0; cz < g->cells_z; cz++) {
        for (int cy = 0; cy < g->cells_y; cy++) {
            for (int cx = 0; cx < g->cells_x; cx++) {
                int cell_idx = cx + cy * g->cells_x + cz * g->cells_x * g->cells_y;
                if (g->head[cell_idx] == -1) continue;

                Vector3 center = {
                    (cx + 0.5f) * g->cell_size - g->world_offset,
                    (cy + 0.5f) * g->cell_size - g->world_offset,
                    (cz + 0.5f) * g->cell_size - g->world_offset
                };
                DrawCubeWires(center, g->cell_size, g->cell_size, g->cell_size, Fade(GREEN, 0.3f));
            }
        }
    }
}

// dibuja en modo debug el radio de vision y el cono del angulo ciego trasero de un boid
void debug_draw_vision(const boid *b, const config *cfg) {
    Vector3 pos = { b->position.x, b->position.y, b->position.z };
    DrawSphereWires(pos, cfg->vision_radius, 12, 12, Fade(YELLOW, 0.5f));

    if (cfg->blind_angle <= 0.001f) return; // sin angulo ciego no hay cono que dibujar

    vec3 fwd = vec3_normalize(b->velocity);
    if (vec3_length2(fwd) < 0.5f) return; // boid parado, direccion indefinida

    // base ortonormal alrededor del eje trasero del boid
    vec3 back = vec3_scale(fwd, -1.0f);
    vec3 ref = (fabsf(back.y) < 0.99f) ? (vec3){0.0f, 1.0f, 0.0f} : (vec3){1.0f, 0.0f, 0.0f};
    vec3 u = vec3_normalize(vec3_cross(back, ref));
    vec3 v = vec3_cross(back, u);

    // anillo donde el cono ciego corta la esfera de vision
    float ring_dist = cfg->vision_radius * cosf(cfg->blind_angle);
    float ring_radius = cfg->vision_radius * sinf(cfg->blind_angle);
    vec3 ring_center = vec3_add(b->position, vec3_scale(back, ring_dist));

    const int segments = 24;
    Vector3 prev = {0};
    for (int s = 0; s <= segments; s++) {
        float a = (2.0f * PI * s) / segments;
        vec3 offset = vec3_add(vec3_scale(u, cosf(a) * ring_radius), vec3_scale(v, sinf(a) * ring_radius));
        vec3 p = vec3_add(ring_center, offset);
        Vector3 point = { p.x, p.y, p.z };

        if (s > 0) DrawLine3D(prev, point, RED);
        // generatrices del cono desde el boid hasta el anillo
        if (s % 4 == 0) DrawLine3D(pos, point, Fade(RED, 0.6f));
        prev = point;
    }
}

// dimensiones del panel de la ui (tambien usadas para ignorar los clics de seleccion sobre el)
#define UI_PANEL_WIDTH 340
#define UI_PANEL_HEIGHT 1061

// tolerancia angular del picking (~1 grado): un boid lejano ocupa un par de pixeles,
// asi que se acepta todo lo que quede dentro de un pequeño cono alrededor del cursor
#define PICK_TOLERANCE_RAD 0.02f

// devuelve el boid mas centrado respecto al rayo del cursor dentro de la tolerancia, o -1 si no hay ninguno
int boid_pick_from_ray(Ray ray, const boid *boids, int num_boids) {
    int picked = -1;
    float best_angular2 = 0.0f;

    for (int i = 0; i < num_boids; i++) {
        vec3 to_boid = vec3_sub(boids[i].position, (vec3){ ray.position.x, ray.position.y, ray.position.z });

        // distancia a lo largo del rayo; negativa = detras de la camara
        float t = to_boid.x * ray.direction.x + to_boid.y * ray.direction.y + to_boid.z * ray.direction.z;
        if (t <= 0.0f) continue;

        // distancia perpendicular al rayo (al cuadrado)
        float perp2 = vec3_length2(to_boid) - t * t;

        // radio aceptado: el cuerpo del boid mas un margen que crece con la distancia
        float tolerance = BOID_BOUNDING_RADIUS + t * PICK_TOLERANCE_RAD;
        if (perp2 > tolerance * tolerance) continue;

        // gana el que este mas centrado en el cursor (menor angulo), no el mas cercano
        float angular2 = perp2 / (t * t);
        if (picked < 0 || angular2 < best_angular2) {
            best_angular2 = angular2;
            picked = i;
        }
    }

    return picked;
}

// primer obstaculo atravesado por el rayo (el mas cercano a la camara), o -1 si ninguno
int obstacle_pick_from_ray(Ray ray, const obstacle *obstacles, int num_obstacles) {
    int picked = -1;
    float best_along = 0.0f;

    for (int i = 0; i < num_obstacles; i++) {
        vec3 to_center = vec3_sub(obstacles[i].position, (vec3){ ray.position.x, ray.position.y, ray.position.z });

        float along = to_center.x * ray.direction.x + to_center.y * ray.direction.y + to_center.z * ray.direction.z;
        if (along <= 0.0f) continue;

        float perp2 = vec3_length2(to_center) - along * along;
        float bounding = obstacle_bounding_radius(&obstacles[i]);
        if (perp2 > bounding * bounding) continue;

        if (picked < 0 || along < best_along) {
            best_along = along;
            picked = i;
        }
    }

    return picked;
}

// pasa al siguiente modo de camara (libre -> 1a persona -> 3a persona)
// al salir del modo libre elige un boid aleatorio si no habia ninguno seleccionado
void camera_mode_cycle(camera_mode *mode, int *followed_boid, int num_boids) {
    *mode = (*mode + 1) % 3;

    if (*mode != CAM_MODE_FREE && *followed_boid < 0) {
        if (num_boids > 0) *followed_boid = rand() % num_boids;
        else *mode = CAM_MODE_FREE;
    }
}

// coloca la camara pegada al boid seguido: en el morro (1a persona) o detras y algo por encima (3a persona)
void camera_follow_boid(Camera3D *camera, const boid *b, camera_mode mode, float distance) {
    vec3 dir = vec3_normalize(b->velocity);
    if (vec3_length2(dir) < 0.5f) dir = (vec3){0.0f, 0.0f, 1.0f}; // boid parado

    vec3 pos, target;
    if (mode == CAM_MODE_FIRST_PERSON) {
        // justo delante de la punta del cono para no ver el propio cuerpo
        pos = vec3_add(b->position, vec3_scale(dir, 0.7f));
        target = vec3_add(b->position, vec3_scale(dir, 10.0f));
    } else {
        pos = vec3_add(b->position, vec3_scale(dir, -distance));
        pos.y += distance * 0.35f;
        target = b->position;
    }

    camera->position = (Vector3){ pos.x, pos.y, pos.z };
    camera->target = (Vector3){ target.x, target.y, target.z };
    // si el boid vuela casi en vertical la vista se alinearia con el up por defecto y la camara se voltearia
    camera->up = (dir.x * dir.x + dir.z * dir.z < 0.001f) ? (Vector3){0.0f, 0.0f, 1.0f} : (Vector3){0.0f, 1.0f, 0.0f};
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
        .avoid_weight = 1.5f,
        .boundary_mode = BOUNDARY_WRAP,
        .show_grid = false,
        .show_vision = false,
        .show_world_bounds = true,
        .show_fps = true,
        .show_crosshair = true,
        .show_speed_heatmap = false,
    };

    //calculo del coseno del angulo muerto
    cfg.cos_blind_angle = cosf(PI - cfg.blind_angle);

    // semilla configurable desde la ui
    int seed = 1;

    // semilla aleatoria
    srand(seed);

    // calculo de una tabla de aleatoriedad para no hacer calculos con el ruido
    for (int i = 0; i < 1024; i++) {
        cfg.noise_table[i] = rand_range(-1.0f, 1.0f);
    }

    // array dinamico para los boids
    boid *boids = malloc(sizeof(boid) * cfg.max_boids);

    //check null de la asignacion anterior
    if (boids == NULL) return 1;
    

    //generación inicial de los boids
    boids_init(boids, &cfg);

    //generación del grid
    spatial_grid grid = spatial_grid_init(&cfg);

    //check null de las reservas del grid
    if (grid.head == NULL || grid.next == NULL) return 1;

    bool show_ui = true;
    bool is_paused = false;
    bool seed_edit_mode = false;

    // camara: modo actual, boid seguido y distancia de la vista en tercera persona
    camera_mode cam_mode = CAM_MODE_FREE;
    int followed_boid = -1;
    float follow_distance = 8.0f;

    // carpeta de presets junto al ejecutable
    char presets_dir[512];
    snprintf(presets_dir, sizeof(presets_dir), "%spresets", GetApplicationDirectory());
    if (!DirectoryExists(presets_dir)) MakeDirectory(presets_dir);

    char preset_names[FILE_LIST_MAX_COUNT][FILE_LIST_MAX_NAME];
    char preset_dropdown_text[1024];
    int preset_count = refresh_file_list(presets_dir, ".cfg", preset_names, preset_dropdown_text, sizeof(preset_dropdown_text), "(sin presets)");
    int preset_selected = 0;
    bool preset_edit_mode = false;
    char preset_name_input[FILE_LIST_MAX_NAME] = "";
    bool preset_name_edit_mode = false;

    // carpeta de snapshots (estado completo de la simulacion) junto al ejecutable
    char snapshots_dir[512];
    snprintf(snapshots_dir, sizeof(snapshots_dir), "%ssnapshots", GetApplicationDirectory());
    if (!DirectoryExists(snapshots_dir)) MakeDirectory(snapshots_dir);

    char snapshot_names[FILE_LIST_MAX_COUNT][FILE_LIST_MAX_NAME];
    char snapshot_dropdown_text[1024];
    int snapshot_count = refresh_file_list(snapshots_dir, ".snap", snapshot_names, snapshot_dropdown_text, sizeof(snapshot_dropdown_text), "(sin snapshots)");
    int snapshot_selected = 0;
    bool snapshot_edit_mode = false;
    char snapshot_name_input[FILE_LIST_MAX_NAME] = "";
    bool snapshot_name_edit_mode = false;

    // obstaculos colocados en tiempo real y estado del modo de colocacion
    obstacle obstacles[MAX_OBSTACLES];
    int num_obstacles = 0;
    bool placing_obstacles = false;
    int place_type = OBSTACLE_SPHERE; // figura seleccionada en el desplegable
    float place_distance = 40.0f; // distancia de la camara a la superficie mas cercana del fantasma
    float place_radius = 10.0f;
    float place_yaw = 0.0f;
    float place_pitch = 0.0f;

    // carpeta de mapas de obstaculos junto al ejecutable
    char obstacles_dir[512];
    snprintf(obstacles_dir, sizeof(obstacles_dir), "%sobstacles", GetApplicationDirectory());
    if (!DirectoryExists(obstacles_dir)) MakeDirectory(obstacles_dir);

    char obstacle_map_names[FILE_LIST_MAX_COUNT][FILE_LIST_MAX_NAME];
    char obstacle_map_dropdown_text[1024];
    int obstacle_map_count = refresh_file_list(obstacles_dir, ".obs", obstacle_map_names, obstacle_map_dropdown_text, sizeof(obstacle_map_dropdown_text), "(sin mapas)");
    int obstacle_map_selected = 0;
    bool obstacle_map_edit_mode = false;
    char obstacle_map_name_input[FILE_LIST_MAX_NAME] = "";
    bool obstacle_map_name_edit_mode = false;

    // paso y reloj de la simulación fijos, para que no dependan del framerate real y sea reproducible
    const float sim_dt = 1.0f / 60.0f;
    float sim_time = 0.0f;

    // cache de render: las matrices solo se recalculan si la simulacion avanza o cambia la vista
    int visible_boids = 0;
    int prev_num_boids = -1;
    bool transforms_dirty = true;

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

    // mallas unitarias de los obstaculos (radio 1, escaladas al dibujar)
    Mesh obstacleMeshes[OBSTACLE_TYPE_COUNT];
    obstacleMeshes[OBSTACLE_SPHERE] = GenMeshSphere(1.0f, 8, 16);
    obstacleMeshes[OBSTACLE_BOX] = GenMeshCube(2.0f, 2.0f, 2.0f);
    obstacleMeshes[OBSTACLE_TORUS] = GenMeshTorus(TORUS_TUBE_RATIO, 2.0f, 16, 24);
    obstacleMeshes[OBSTACLE_CYLINDER] = GenMeshCylinder(1.0f, 2.0f, 16);
    Material obstacleMaterial = LoadMaterialDefault();

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

    //check null de la asignacion anterior
    if (boidTransforms == NULL) {
        CloseWindow();
        return 1;
    }

    Camera3D camera = {0};
    camera.position = (Vector3){cfg.world_size+10.0f, cfg.world_size+10.0f, cfg.world_size+10.0f};
    camera.target = (Vector3){0.0f, 0.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE; 

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // mientras se edita un campo de texto, el teclado no debe disparar atajos ni mover la camara
        bool typing = seed_edit_mode || preset_name_edit_mode || snapshot_name_edit_mode || obstacle_map_name_edit_mode;

        if (!typing) simulation_handle_input(&cfg, &is_paused);

        // reinicio rápido de la simulación (deshabilitado mientras se edita algun campo de texto)
        if (IsKeyPressed(KEY_R) && !typing) {
            boids_reset(boids, &cfg, seed, &sim_time);
            transforms_dirty = true;
        }

        // alternar modo de camara
        if (IsKeyPressed(KEY_C) && !typing) {
            camera_mode_cycle(&cam_mode, &followed_boid, cfg.num_boids);
            transforms_dirty = true;
        }

        // alternar modo de colocacion de obstaculos
        if (IsKeyPressed(KEY_O) && !typing) placing_obstacles = !placing_obstacles;

        // si el boid seguido deja de existir (slider de num boids o carga de snapshot), volver a camara libre
        if (followed_boid >= cfg.num_boids) {
            followed_boid = -1;
            cam_mode = CAM_MODE_FREE;
        }

        bool following = (cam_mode != CAM_MODE_FREE && followed_boid >= 0);

        //UpdateCamera(&camera, CAMERA_FREE); //camara antes
        //velocidad de la camara
        float base_speed = 150.0f;
        
        // Sprint al mantener Control
        if (IsKeyDown(KEY_LEFT_CONTROL)) base_speed *= 3.0f; 
        
        float cam_speed = base_speed * dt;

        Vector3 movement = {0};
        if (!typing) {
            if (IsKeyDown(KEY_W)) movement.x += cam_speed;
            if (IsKeyDown(KEY_S)) movement.x -= cam_speed;
            if (IsKeyDown(KEY_D)) movement.y += cam_speed;
            if (IsKeyDown(KEY_A)) movement.y -= cam_speed;

            // espacio y shift para subir y bajar
            if (IsKeyDown(KEY_SPACE)) movement.z += cam_speed;
            if (IsKeyDown(KEY_LEFT_SHIFT)) movement.z -= cam_speed;
        }

        // desactivar ui
        if (IsKeyPressed(KEY_TAB) && !typing) {
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

        // punto de apuntado compartido por seleccion de boids y colocacion de obstaculos:
        // con la ui visible es el cursor (fuera del panel), con la ui oculta el centro de la pantalla
        Vector2 aim = { GetScreenWidth() * 0.5f, GetScreenHeight() * 0.5f };
        bool aim_valid = true;

        if (show_ui) {
            aim = GetMousePosition();
            Rectangle panel_rect = { (float)(GetScreenWidth() - UI_PANEL_WIDTH - 10), 10.0f, (float)UI_PANEL_WIDTH, (float)UI_PANEL_HEIGHT };
            aim_valid = !CheckCollisionPointRec(aim, panel_rect);
        }

        // en modo colocacion la rueda ajusta la distancia del fantasma en lugar del zoom de camara
        if (placing_obstacles) {
            place_distance = clamp_float(place_distance + zoom * 2.5f, 5.0f, 300.0f);
            zoom = 0.0f;

            // flechas para rotar la figura antes de colocarla
            if (!typing) {
                float rot = 1.5f * dt;
                if (IsKeyDown(KEY_RIGHT)) place_yaw += rot;
                if (IsKeyDown(KEY_LEFT)) place_yaw -= rot;
                if (IsKeyDown(KEY_UP)) place_pitch += rot;
                if (IsKeyDown(KEY_DOWN)) place_pitch -= rot;
                if (place_yaw > PI) place_yaw -= 2.0f * PI; else if (place_yaw < -PI) place_yaw += 2.0f * PI;
                if (place_pitch > PI) place_pitch -= 2.0f * PI; else if (place_pitch < -PI) place_pitch += 2.0f * PI;
            }
        }

        Ray aim_ray = GetScreenToWorldRay(aim, camera);
        // el fantasma se proyecta por delante de la camara: la distancia se mide hasta su superficie, no a su centro
        obstacle ghost = { (obstacle_type)place_type, {0.0f, 0.0f, 0.0f}, place_radius, place_yaw, place_pitch };
        float ghost_dist = place_distance + obstacle_bounding_radius(&ghost);
        ghost.position = (vec3){
            aim_ray.position.x + aim_ray.direction.x * ghost_dist,
            aim_ray.position.y + aim_ray.direction.y * ghost_dist,
            aim_ray.position.z + aim_ray.direction.z * ghost_dist
        };

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && aim_valid) {
            if (placing_obstacles) {
                if (num_obstacles < MAX_OBSTACLES) obstacles[num_obstacles++] = ghost;
            } else {
                int picked = boid_pick_from_ray(aim_ray, boids, cfg.num_boids);
                if (picked >= 0) {
                    followed_boid = picked;
                    if (cam_mode == CAM_MODE_FREE) cam_mode = CAM_MODE_THIRD_PERSON;
                    following = true;
                    transforms_dirty = true;
                }
            }
        }

        // clic derecho en modo colocacion: borrar el obstaculo apuntado
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && placing_obstacles && aim_valid) {
            int picked = obstacle_pick_from_ray(aim_ray, obstacles, num_obstacles);
            if (picked >= 0) obstacles[picked] = obstacles[--num_obstacles];
        }

        if (!following) {
            camera.up = (Vector3){0.0f, 1.0f, 0.0f};
            // se pasa el calculo a update camera
            UpdateCameraPro(&camera, movement, rotation, zoom);
        } else {
            // en modo seguimiento la rueda ajusta la distancia de la tercera persona
            follow_distance = clamp_float(follow_distance - zoom, 2.0f, 60.0f);
        }

        if (!is_paused) {
            //rellenar grid
            grid_build(&grid, boids, &cfg);
            //avanzar el reloj de simulación con un paso fijo (no el tiempo real de reloj)
            sim_time += sim_dt;
            //calculo de reglas (separacion, alineacion, cohesion)
            boids_compute_accelerations(boids, &cfg, &grid, obstacles, num_obstacles, sim_time);
            //actualizacion de boids
            boids_update(boids, &cfg, obstacles, num_obstacles, sim_dt);
        }

        // la camara se pega al boid despues de moverlo para no ir un frame por detras
        if (following) camera_follow_boid(&camera, &boids[followed_boid], cam_mode, follow_distance);

        // solo se recalculan matrices si los boids se han movido o la vista ha cambiado
        bool camera_moved = (movement.x != 0.0f) || (movement.y != 0.0f) || (movement.z != 0.0f)
                         || (rotation.x != 0.0f) || (rotation.y != 0.0f) || (zoom != 0.0f) || IsWindowResized();

        if (!is_paused || camera_moved || cfg.num_boids != prev_num_boids || transforms_dirty) {
            // planos del frustum para descartar los boids que la camara no ve
            Matrix view = GetCameraMatrix(camera);
            float aspect = (float)GetScreenWidth() / (float)GetScreenHeight();
            Matrix proj = MatrixPerspective(camera.fovy * DEG2RAD, aspect, CAMERA_NEAR_PLANE, CAMERA_FAR_PLANE);
            frustum fr = frustum_from_view_projection(MatrixMultiply(view, proj));

            visible_boids = 0;

            #pragma omp parallel for
            for (int i = 0; i < cfg.num_boids; i++) {
                boid *b = &boids[i];

                // en primera persona el boid seguido no se dibuja para no tapar la vista
                if (cam_mode == CAM_MODE_FIRST_PERSON && i == followed_boid) continue;

                if (!frustum_contains_sphere(&fr, b->position, BOID_BOUNDING_RADIUS)) continue;

                //la dirección en la que vuela el boid normalizada (la longitud se reutiliza para el heatmap)
                float speed = sqrtf(vec3_length2(b->velocity));
                vec3 dir = (speed > 0.0f) ? vec3_scale(b->velocity, 1.0f / speed) : (vec3){0.0f, 0.0f, 0.0f};
                Matrix transform = boid_transform(b->position, dir);

                // color por instancia inyectado en la fila libre de la matriz (m3, m7, m11)
                vec3 color = BOID_BASE_COLOR;
                if (cfg.show_speed_heatmap) {
                    float t = (speed - cfg.min_speed) / fmaxf(cfg.max_speed - cfg.min_speed, 0.001f);
                    color = speed_heatmap_color(clamp_float(t, 0.0f, 1.0f));
                }
                transform.m3 = color.x;
                transform.m7 = color.y;
                transform.m11 = color.z;

                //reservar hueco en el array compactado de boids visibles
                int slot;
                #pragma omp atomic capture
                slot = visible_boids++;

                boidTransforms[slot] = transform;
            }

            prev_num_boids = cfg.num_boids;
            transforms_dirty = false;
        }

        BeginDrawing();
        ClearBackground((Color){20, 22, 28, 255});

        BeginMode3D(camera);

        // dibujar el suelo y los limites del mundo
        //DrawGrid(20, 1.0f);
        if (cfg.show_world_bounds) {
            DrawCubeWires(
                (Vector3){0.0f, 0.0f, 0.0f},
                cfg.world_size * 2.0f,
                cfg.world_size * 2.0f,
                cfg.world_size * 2.0f,
                GRAY
            );
        }

        //dibujado directo de los boids visibles
        if (visible_boids > 0) DrawMeshInstanced(boidMesh, boidMaterial, boidTransforms, visible_boids);

        // obstaculos activos: relleno translucido mas pasada en alambre para perfilar la figura
        for (int i = 0; i < num_obstacles; i++) {
            Matrix obs_transform = obstacle_draw_transform(&obstacles[i]);
            obstacleMaterial.maps[MATERIAL_MAP_ALBEDO].color = Fade(OBSTACLE_COLOR, 0.35f);
            DrawMesh(obstacleMeshes[obstacles[i].type], obstacleMaterial, obs_transform);
            rlEnableWireMode();
            obstacleMaterial.maps[MATERIAL_MAP_ALBEDO].color = Fade(OBSTACLE_COLOR, 0.8f);
            DrawMesh(obstacleMeshes[obstacles[i].type], obstacleMaterial, obs_transform);
            rlDisableWireMode();
        }

        // fantasma de previsualizacion del obstaculo a colocar
        if (placing_obstacles && aim_valid && num_obstacles < MAX_OBSTACLES) {
            rlEnableWireMode();
            obstacleMaterial.maps[MATERIAL_MAP_ALBEDO].color = Fade(GREEN, 0.6f);
            DrawMesh(obstacleMeshes[ghost.type], obstacleMaterial, obstacle_draw_transform(&ghost));
            rlDisableWireMode();
        }

        // overlays de debug: celdas ocupadas del grid y vision del boid seguido
        if (cfg.show_grid) debug_draw_grid(&grid);
        if (cfg.show_vision && followed_boid >= 0 && followed_boid < cfg.num_boids) {
            debug_draw_vision(&boids[followed_boid], &cfg);
        }

        EndMode3D();

        // mirilla de referencia para seleccionar boids con la ui oculta
        if (!show_ui && cfg.show_crosshair) {
            int cx = GetScreenWidth() / 2;
            int cy = GetScreenHeight() / 2;
            DrawLine(cx - 8, cy, cx + 8, cy, Fade(RAYWHITE, 0.7f));
            DrawLine(cx, cy - 8, cx, cy + 8, Fade(RAYWHITE, 0.7f));
        }

        // recordatorio de controles del modo colocacion
        if (placing_obstacles) {
            DrawText(TextFormat("Colocar: clic izq  |  Borrar: clic dcho  |  Rueda: distancia %.0f  |  Flechas: rotar (yaw %.0f, pitch %.0f)",
                     place_distance, place_yaw * RAD2DEG, place_pitch * RAD2DEG),
                     10, GetScreenHeight() - 30, 20, RAYWHITE);
        }

        if (show_ui) {
            int pW = UI_PANEL_WIDTH;
            int pH = UI_PANEL_HEIGHT;
            
            // Ancho total de pantalla, menos el ancho del panel, menos 10 píxeles de margen
            int pX = GetScreenWidth() - pW - 10; 
            int pY = 10; // Pegado arriba
            
            GuiPanel((Rectangle){ (float)pX, (float)pY, (float)pW, (float)pH }, "Parameters (TAB to fly)");

            // Coordenadas base para los sliders relativas al panel
            int sX = pX + 110;
            int sY = pY + 40;
            int sW = 160;
            int sH = 15;
            int space = 26;

            // ancho de fila para los controles con boton lateral (nombre/desplegable + Save/Load), alineado con el resto de la ui
            int rowGap = 10;
            int rowBtnW = 55;
            int rowFieldW = sW + 10 - rowGap - rowBtnW;

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

            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Avoidance", TextFormat("%.2f", cfg.avoid_weight), &cfg.avoid_weight, 0.0f, 3.0f);

            // Comportamiento de bordes: BOUNCE -> WRAP -> STEER (evasion suave)
            sY += space;
            if (GuiButton((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, TextFormat("Boundary: %s (B)", boundary_mode_to_string(cfg.boundary_mode)))) {
                cfg.boundary_mode = (cfg.boundary_mode + 1) % 3;
            }

            // pausar/reanudar la simulacion
            sY += space;
            GuiCheckBox((Rectangle){ (float)sX, (float)sY, 15, 15 }, "Paused (P to toggle)", &is_paused);

            // semilla del rng
            sY += space;
            if (GuiValueBox((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Seed", &seed, 0, 999999, seed_edit_mode)) {
                seed_edit_mode = !seed_edit_mode;
            }

            // reset de la simulacion con la semilla actual
            sY += space;
            if (GuiButton((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH + 10 }, "Reset Simulation (R)")) {
                boids_reset(boids, &cfg, seed, &sim_time);
                transforms_dirty = true;
            }

            // alternar modo de camara (libre / 1a persona / 3a persona)
            sY += space;
            const char *cam_label = (cam_mode == CAM_MODE_FREE) ? "Camera: Free (C)"
                                  : (cam_mode == CAM_MODE_FIRST_PERSON) ? TextFormat("Camera: 1st #%d (C)", followed_boid)
                                  : TextFormat("Camera: 3rd #%d (C)", followed_boid);
            if (GuiButton((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH + 10 }, cam_label)) {
                camera_mode_cycle(&cam_mode, &followed_boid, cfg.num_boids);
                transforms_dirty = true;
            }

            // elegir un boid aleatorio a seguir (tambien se puede clicar un boid en el mundo)
            sY += space + 10;
            if (GuiButton((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH + 10 }, "Follow Random Boid") && cfg.num_boids > 0) {
                followed_boid = rand() % cfg.num_boids;
                if (cam_mode == CAM_MODE_FREE) cam_mode = CAM_MODE_THIRD_PERSON;
                transforms_dirty = true;
            }

            // nombre bajo el que se guardara la config actual como preset, con su boton de guardado al lado
            sY += space + 10;
            GuiLine((Rectangle){ (float)(pX + 10), (float)(sY - 8), (float)(pW - 20), 10 }, "Preset");
            sY += space - 10;
            if (GuiTextBox((Rectangle){ (float)sX, (float)sY, (float)rowFieldW, (float)sH }, preset_name_input, FILE_LIST_MAX_NAME, preset_name_edit_mode)) {
                preset_name_edit_mode = !preset_name_edit_mode;
            }
            if (GuiButton((Rectangle){ (float)(sX + rowFieldW + rowGap), (float)sY, (float)rowBtnW, (float)sH }, "Save") && preset_name_input[0] != '\0') {
                char filepath[600];
                snprintf(filepath, sizeof(filepath), "%s/%s.cfg", presets_dir, preset_name_input);
                config_save_preset(&cfg, filepath);
                preset_count = refresh_file_list(presets_dir, ".cfg", preset_names, preset_dropdown_text, sizeof(preset_dropdown_text), "(sin presets)");
                if (preset_selected >= preset_count) preset_selected = (preset_count > 0) ? preset_count - 1 : 0;
            }

            // desplegable de presets guardados, con su boton de carga al lado
            sY += space;
            Rectangle presetDropdownRect = { (float)sX, (float)sY, (float)rowFieldW, (float)sH };
            if (GuiButton((Rectangle){ (float)(sX + rowFieldW + rowGap), (float)sY, (float)rowBtnW, (float)sH }, "Load") && preset_count > 0) {
                char filepath[600];
                snprintf(filepath, sizeof(filepath), "%s/%s.cfg", presets_dir, preset_names[preset_selected]);
                config_load_preset(&cfg, filepath);
            }

            // snapshots: guardan la config y la posicion/velocidad/aceleracion de todos los boids con nombre propio para retomar la simulacion despues
            sY += space + 10;
            GuiLine((Rectangle){ (float)(pX + 10), (float)(sY - 8), (float)(pW - 20), 10 }, "Snapshot");
            sY += space - 10;
            if (GuiTextBox((Rectangle){ (float)sX, (float)sY, (float)rowFieldW, (float)sH }, snapshot_name_input, FILE_LIST_MAX_NAME, snapshot_name_edit_mode)) {
                snapshot_name_edit_mode = !snapshot_name_edit_mode;
            }
            if (GuiButton((Rectangle){ (float)(sX + rowFieldW + rowGap), (float)sY, (float)rowBtnW, (float)sH }, "Save") && snapshot_name_input[0] != '\0') {
                char filepath[600];
                snprintf(filepath, sizeof(filepath), "%s/%s.snap", snapshots_dir, snapshot_name_input);
                boids_save_state(boids, &cfg, obstacles, num_obstacles, sim_time, filepath);
                snapshot_count = refresh_file_list(snapshots_dir, ".snap", snapshot_names, snapshot_dropdown_text, sizeof(snapshot_dropdown_text), "(sin snapshots)");
                if (snapshot_selected >= snapshot_count) snapshot_selected = (snapshot_count > 0) ? snapshot_count - 1 : 0;
            }

            sY += space;
            Rectangle snapshotDropdownRect = { (float)sX, (float)sY, (float)rowFieldW, (float)sH };
            if (GuiButton((Rectangle){ (float)(sX + rowFieldW + rowGap), (float)sY, (float)rowBtnW, (float)sH }, "Load") && snapshot_count > 0) {
                char filepath[600];
                snprintf(filepath, sizeof(filepath), "%s/%s.snap", snapshots_dir, snapshot_names[snapshot_selected]);
                if (boids_load_state(boids, &cfg, &sim_time, obstacles, &num_obstacles, filepath)) transforms_dirty = true;
            }

            // obstaculos: modo de colocacion con el raton, radio y mapas guardables por separado
            sY += space + 10;
            GuiLine((Rectangle){ (float)(pX + 10), (float)(sY - 8), (float)(pW - 20), 10 }, "Obstacles");
            sY += space - 10;
            GuiCheckBox((Rectangle){ (float)sX, (float)sY, 15, 15 }, TextFormat("Place mode (O)  %d/%d", num_obstacles, MAX_OBSTACLES), &placing_obstacles);
            sY += space;
            GuiComboBox((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Sphere;Cube;Donut;Cylinder", &place_type);
            sY += space;
            GuiSliderBar((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Radius", TextFormat("%.1f", place_radius), &place_radius, 1.0f, 50.0f);
            sY += space;
            if (GuiButton((Rectangle){ (float)sX, (float)sY, (float)sW, (float)sH }, "Clear Obstacles")) num_obstacles = 0;
            sY += space;
            if (GuiTextBox((Rectangle){ (float)sX, (float)sY, (float)rowFieldW, (float)sH }, obstacle_map_name_input, FILE_LIST_MAX_NAME, obstacle_map_name_edit_mode)) {
                obstacle_map_name_edit_mode = !obstacle_map_name_edit_mode;
            }
            if (GuiButton((Rectangle){ (float)(sX + rowFieldW + rowGap), (float)sY, (float)rowBtnW, (float)sH }, "Save") && obstacle_map_name_input[0] != '\0') {
                char filepath[600];
                snprintf(filepath, sizeof(filepath), "%s/%s.obs", obstacles_dir, obstacle_map_name_input);
                obstacles_save_map(obstacles, num_obstacles, filepath);
                obstacle_map_count = refresh_file_list(obstacles_dir, ".obs", obstacle_map_names, obstacle_map_dropdown_text, sizeof(obstacle_map_dropdown_text), "(sin mapas)");
                if (obstacle_map_selected >= obstacle_map_count) obstacle_map_selected = (obstacle_map_count > 0) ? obstacle_map_count - 1 : 0;
            }

            sY += space;
            Rectangle obstacleMapDropdownRect = { (float)sX, (float)sY, (float)rowFieldW, (float)sH };
            if (GuiButton((Rectangle){ (float)(sX + rowFieldW + rowGap), (float)sY, (float)rowBtnW, (float)sH }, "Load") && obstacle_map_count > 0) {
                char filepath[600];
                snprintf(filepath, sizeof(filepath), "%s/%s.obs", obstacles_dir, obstacle_map_names[obstacle_map_selected]);
                obstacles_load_map(obstacles, &num_obstacles, filepath);
            }

            // seccion de debug: overlays de visualizacion en tiempo real
            sY += space + 10;
            GuiLine((Rectangle){ (float)(pX + 10), (float)(sY - 8), (float)(pW - 20), 10 }, "Debug");
            sY += space - 10;
            GuiCheckBox((Rectangle){ (float)sX, (float)sY, 15, 15 }, "Show Grid Cells", &cfg.show_grid);
            sY += space;
            GuiCheckBox((Rectangle){ (float)sX, (float)sY, 15, 15 }, "Show Vision (followed boid)", &cfg.show_vision);
            sY += space;
            GuiCheckBox((Rectangle){ (float)sX, (float)sY, 15, 15 }, "Show World Bounds", &cfg.show_world_bounds);
            sY += space;
            GuiCheckBox((Rectangle){ (float)sX, (float)sY, 15, 15 }, "Show FPS", &cfg.show_fps);
            sY += space;
            GuiCheckBox((Rectangle){ (float)sX, (float)sY, 15, 15 }, "Show Crosshair", &cfg.show_crosshair);
            sY += space;
            bool prev_heatmap = cfg.show_speed_heatmap;
            GuiCheckBox((Rectangle){ (float)sX, (float)sY, 15, 15 }, "Speed Heatmap", &cfg.show_speed_heatmap);
            // en pausa las matrices no se recalculan, hay que forzar el refresco del color
            if (cfg.show_speed_heatmap != prev_heatmap) transforms_dirty = true;

            // los desplegables se dibujan al final para que sus listas aparezcan por encima del resto de controles
            if (GuiDropdownBox(obstacleMapDropdownRect, obstacle_map_dropdown_text, &obstacle_map_selected, obstacle_map_edit_mode)) {
                obstacle_map_edit_mode = !obstacle_map_edit_mode;
            }
            if (GuiDropdownBox(presetDropdownRect, preset_dropdown_text, &preset_selected, preset_edit_mode)) {
                preset_edit_mode = !preset_edit_mode;
            }
            if (GuiDropdownBox(snapshotDropdownRect, snapshot_dropdown_text, &snapshot_selected, snapshot_edit_mode)) {
                snapshot_edit_mode = !snapshot_edit_mode;
            }
        }

        if (cfg.show_fps) DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();

    //liberar memoria del mesh de boids
    free(boidTransforms);
    UnloadMesh(boidMesh);
    //liberar mallas de los obstaculos
    for (int i = 0; i < OBSTACLE_TYPE_COUNT; i++) UnloadMesh(obstacleMeshes[i]);
    //librerar arrays del grid
    free(grid.head);
    free(grid.next);
    //liberar memoria del array de boids
    free(boids);

    return 0;
}
