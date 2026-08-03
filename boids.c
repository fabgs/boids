#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
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
    cfg->cos_blind_angle = cosf(PI - cfg->blind_angle);
}

#define STATE_MAGIC "BSTA"
#define STATE_VERSION 2

// cabecera del archivo de snapshot guardado
typedef struct {
    char magic[4];
    int version;
    float sim_time;
} state_header;

// vuelca a un archivo binario la config completa (para retomar tambien los parametros) y la posicion/velocidad/aceleracion de cada boid activo
bool boids_save_state(const boid *boids, const config *cfg, float sim_time, const char *filename) {
    FILE *f = fopen(filename, "wb");
    if (f == NULL) return false;

    state_header header = { .version = STATE_VERSION, .sim_time = sim_time };
    memcpy(header.magic, STATE_MAGIC, 4);

    bool ok = fwrite(&header, sizeof(header), 1, f) == 1
           && fwrite(cfg, sizeof(config), 1, f) == 1
           && fwrite(boids, sizeof(boid), (size_t)cfg->num_boids, f) == (size_t)cfg->num_boids;

    fclose(f);
    return ok;
}

// restaura la config y el estado de los boids desde un archivo generado por boids_save_state
bool boids_load_state(boid *boids, config *cfg, float *sim_time, const char *filename) {
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
    fclose(f);

    if (!ok) return false;

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
    return (mode == BOUNDARY_WRAP) ? "WRAP" : "BOUNCE";
}

// convierte el texto leido del archivo al modo de borde correspondiente
boundary_mode boundary_mode_from_string(const char *text) {
    return (strcmp(text, "WRAP") == 0) ? BOUNDARY_WRAP : BOUNDARY_BOUNCE;
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
    fprintf(f, "boundary_mode=%s\n", boundary_mode_to_string(cfg->boundary_mode));

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
        else if (strcmp(key, "boundary_mode") == 0) cfg->boundary_mode = boundary_mode_from_string(value);
    }

    fclose(f);

    // recalcula el coseno derivado y clampea todo a rangos validos
    // (un preset editado a mano podria colar valores que rompan la simulacion)
    config_sanitize(cfg);

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

// dimensiones del panel de la ui (tambien usadas para ignorar los clics de seleccion sobre el)
#define UI_PANEL_WIDTH 340
#define UI_PANEL_HEIGHT 700

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
        .boundary_mode = BOUNDARY_WRAP,
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
        bool typing = seed_edit_mode || preset_name_edit_mode || snapshot_name_edit_mode;

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

        // seleccion de boid con clic (solo con la ui visible y fuera del panel)
        if (show_ui && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Vector2 mouse = GetMousePosition();
            Rectangle panel_rect = { (float)(GetScreenWidth() - UI_PANEL_WIDTH - 10), 10.0f, (float)UI_PANEL_WIDTH, (float)UI_PANEL_HEIGHT };

            if (!CheckCollisionPointRec(mouse, panel_rect)) {
                int picked = boid_pick_from_ray(GetScreenToWorldRay(mouse, camera), boids, cfg.num_boids);
                if (picked >= 0) {
                    followed_boid = picked;
                    if (cam_mode == CAM_MODE_FREE) cam_mode = CAM_MODE_THIRD_PERSON;
                    following = true;
                    transforms_dirty = true;
                }
            }
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
            boids_compute_accelerations(boids, &cfg, &grid, sim_time);
            //actualizacion de boids
            boids_update(boids, &cfg, sim_dt);
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

                //la dirección en la que vuela el boid normalizada
                vec3 dir = vec3_normalize(b->velocity);
                Matrix transform = boid_transform(b->position, dir);

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
        //DrawGrid(20, 1.0f); //TODO: se dibujara cuando se implemente el grid parametrizado para comprobaciones locales
        DrawCubeWires(
            (Vector3){0.0f, 0.0f, 0.0f},
            cfg.world_size * 2.0f,
            cfg.world_size * 2.0f,
            cfg.world_size * 2.0f,
            GRAY
        );

        //dibujado directo de los boids visibles
        if (visible_boids > 0) DrawMeshInstanced(boidMesh, boidMaterial, boidTransforms, visible_boids);

        EndMode3D();

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
            int space = 28;

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

            // Comportamiento de bordes
            sY += space;
            bool isWrap = (cfg.boundary_mode == BOUNDARY_WRAP);
            GuiCheckBox((Rectangle){ (float)sX, (float)sY, 15, 15 }, "Boundary: WRAP (B to swap)", &isWrap);
            cfg.boundary_mode = isWrap ? BOUNDARY_WRAP : BOUNDARY_BOUNCE;

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
            if (GuiTextBox((Rectangle){ (float)sX, (float)sY, (float)rowFieldW, (float)sH }, snapshot_name_input, FILE_LIST_MAX_NAME, snapshot_name_edit_mode)) {
                snapshot_name_edit_mode = !snapshot_name_edit_mode;
            }
            if (GuiButton((Rectangle){ (float)(sX + rowFieldW + rowGap), (float)sY, (float)rowBtnW, (float)sH }, "Save") && snapshot_name_input[0] != '\0') {
                char filepath[600];
                snprintf(filepath, sizeof(filepath), "%s/%s.snap", snapshots_dir, snapshot_name_input);
                boids_save_state(boids, &cfg, sim_time, filepath);
                snapshot_count = refresh_file_list(snapshots_dir, ".snap", snapshot_names, snapshot_dropdown_text, sizeof(snapshot_dropdown_text), "(sin snapshots)");
                if (snapshot_selected >= snapshot_count) snapshot_selected = (snapshot_count > 0) ? snapshot_count - 1 : 0;
            }

            sY += space;
            Rectangle snapshotDropdownRect = { (float)sX, (float)sY, (float)rowFieldW, (float)sH };
            if (GuiButton((Rectangle){ (float)(sX + rowFieldW + rowGap), (float)sY, (float)rowBtnW, (float)sH }, "Load") && snapshot_count > 0) {
                char filepath[600];
                snprintf(filepath, sizeof(filepath), "%s/%s.snap", snapshots_dir, snapshot_names[snapshot_selected]);
                if (boids_load_state(boids, &cfg, &sim_time, filepath)) transforms_dirty = true;
            }

            // ambos desplegables se dibujan al final para que sus listas aparezcan por encima del resto de controles
            if (GuiDropdownBox(presetDropdownRect, preset_dropdown_text, &preset_selected, preset_edit_mode)) {
                preset_edit_mode = !preset_edit_mode;
            }
            if (GuiDropdownBox(snapshotDropdownRect, snapshot_dropdown_text, &snapshot_selected, snapshot_edit_mode)) {
                snapshot_edit_mode = !snapshot_edit_mode;
            }
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
