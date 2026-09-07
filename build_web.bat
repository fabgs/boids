@echo off
setlocal
echo ==========================================
echo  Compilando Boids 3D Simulator (Web/WASM)
echo ==========================================
echo.

:: Uso: build_web.bat [--no-threads]
::   --no-threads  compila sin pthreads (un solo hilo). No necesita SharedArrayBuffer ni cabeceras
::                 COOP/COEP en el servidor, a cambio de perder el paralelismo de la simulacion.
::
:: Requiere emcc (Emscripten). lib\libraylib.web.a se compilo con emsdk 6.0.9; conviene usar la misma version.
:: Si emcc no esta en el PATH se intenta activar el emsdk instalado en %USERPROFILE%\emsdk.
:: Ver build_web.sh para los detalles de cada flag y de como se genero la libreria de raylib para web.

set THREADS=1
if /I "%~1"=="--no-threads" set THREADS=0

where emcc >nul 2>nul
if %ERRORLEVEL% neq 0 (
    if exist "%USERPROFILE%\emsdk\emsdk_env.bat" (
        echo [INFO] Activando emsdk desde %USERPROFILE%\emsdk
        call "%USERPROFILE%\emsdk\emsdk_env.bat" >nul 2>nul
    )
)
where emcc >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo [ERROR] No se ha encontrado 'emcc'. Instala emsdk ^(https://emscripten.org^) y activa el entorno.
    echo.
    pause
    exit /b 1
)

if not exist dist mkdir dist

:: ficheros de ejemplo que viajan con la web (se copian al almacenamiento persistente en el primer arranque)
set PRELOAD=--preload-file presets@/bundled/presets
if exist obstacles\*.obs set PRELOAD=%PRELOAD% --preload-file obstacles@/bundled/obstacles

set FLAGS=-O3 -msimd128 -Wall -DPLATFORM_WEB -I.\include
set FLAGS=%FLAGS% -sUSE_GLFW=3 -sGL_ENABLE_GET_PROC_ADDRESS -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2
set FLAGS=%FLAGS% -sINITIAL_MEMORY=268435456 -sSTACK_SIZE=1048576
set FLAGS=%FLAGS% -sFORCE_FILESYSTEM=1 -lidbfs.js -sEXPORTED_RUNTIME_METHODS=FS,IDBFS,addRunDependency,removeRunDependency
set FLAGS=%FLAGS% --shell-file web\shell.html
if "%THREADS%"=="1" set FLAGS=%FLAGS% -pthread -sPTHREAD_POOL_SIZE=navigator.hardwareConcurrency

echo [INFO] Compilando boids.c -^> dist\index.html (hilos: %THREADS%)
echo.
call emcc boids.c lib\libraylib.web.a %FLAGS% %PRELOAD% -o dist\index.html

if %ERRORLEVEL% equ 0 (
    copy /Y web\coi-serviceworker.js dist\ >nul
    echo.
    echo [EXITO] Salida en dist\. Prueba en local con: python web\serve.py
) else (
    echo.
    echo [ERROR] La compilacion ha fallado.
)

echo.
pause
