#!/usr/bin/env bash
# Compila la version web (WebAssembly) en dist/.
#
# Requiere emcc (Emscripten) en el PATH. lib/libraylib.web.a se compilo con emsdk 6.0.9 y conviene usar la
# misma version, porque la libc de emscripten cambia entre versiones y el enlace puede fallar.
#
# Uso: ./build_web.sh [--no-threads]
#   --no-threads  compila sin pthreads (un solo hilo). No necesita SharedArrayBuffer ni cabeceras COOP/COEP
#                 en el servidor, a cambio de perder el paralelismo de la simulacion.
#
# Como se genero lib/libraylib.web.a (desde el src/ de raylib 6.0):
#   for f in rcore rshapes rtextures rtext rmodels utils raudio; do
#     emcc -c $f.c -o $f.o -Os -std=gnu99 -D_GNU_SOURCE -DPLATFORM_WEB -DGRAPHICS_API_OPENGL_ES3 -pthread -I. -Iexternal/glfw/include
#   done
#   emar rcs libraylib.web.a *.o
# GRAPHICS_API_OPENGL_ES3 (WebGL 2) es imprescindible: el render instanciado no existe en WebGL 1.
#
# Memoria: heap fijo de 256 MB (el maximo teorico de la simulacion, 100k boids + grid de 128^3 celdas, ronda los 25 MB).
# No se usa ALLOW_MEMORY_GROWTH porque combinado con pthreads penaliza los accesos a memoria desde JavaScript.
set -euo pipefail
cd "$(dirname "$0")"

THREADS=1
for arg in "$@"; do
    case "$arg" in
        --no-threads) THREADS=0 ;;
        *) echo "argumento desconocido: $arg" >&2; exit 1 ;;
    esac
done

if ! command -v emcc >/dev/null 2>&1; then
    echo "[ERROR] No se ha encontrado 'emcc'. Instala emsdk y activa el entorno (source emsdk_env.sh)." >&2
    exit 1
fi

mkdir -p dist

# ficheros de ejemplo que viajan con la web (se copian al almacenamiento persistente en el primer arranque)
PRELOAD=(--preload-file presets@/bundled/presets)
if [ -d obstacles ] && [ -n "$(ls -A obstacles 2>/dev/null)" ]; then
    PRELOAD+=(--preload-file obstacles@/bundled/obstacles)
fi

FLAGS=(
    -O3 -msimd128 -Wall
    -DPLATFORM_WEB
    -I./include
    -sUSE_GLFW=3 -sGL_ENABLE_GET_PROC_ADDRESS
    -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2
    -sINITIAL_MEMORY=268435456 -sSTACK_SIZE=1048576
    -sFORCE_FILESYSTEM=1 -lidbfs.js
    -sEXPORTED_RUNTIME_METHODS=FS,IDBFS,addRunDependency,removeRunDependency
    --shell-file web/shell.html
)
if [ "$THREADS" = 1 ]; then
    # el pool de workers se precrea al arrancar con un worker por nucleo (ver parallel_init en boids.c)
    FLAGS+=(-pthread -sPTHREAD_POOL_SIZE=navigator.hardwareConcurrency)
fi

echo "[INFO] Compilando boids.c -> dist/index.html (hilos: $THREADS)"
emcc boids.c lib/libraylib.web.a "${FLAGS[@]}" "${PRELOAD[@]}" -o dist/index.html

cp web/coi-serviceworker.js dist/
echo "[EXITO] Salida en dist/. Prueba en local con: python web/serve.py"
