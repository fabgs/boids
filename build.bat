@echo off
echo ==========================================
echo  Compilando Boids 3D Simulator (Windows)
echo ==========================================
echo.

:: Comprobacion de que GCC (MinGW) esta instalado y en el PATH
where gcc >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo [ERROR] No se ha encontrado 'gcc'. 
    echo Asegurate de tener MinGW instalado y configurado en las variables de entorno.
    echo.
    pause
    exit /b 1
)

echo [INFO] Iniciando compilacion de boids.c...
echo.

:: FLAGS:
:: -O3           : Optimizacion
:: -Wall         : Muestra todas las advertencias del compilador
:: -fopenmp      : Activa el procesamiento multihilo
:: -I.\include   : Indica donde buscar los encabezados (.h)
:: -L.\lib       : Indica donde buscar las librerias (.a)
:: -lraylibdll   : Vincula la libreria dinamica de Raylib

gcc boids.c -o boids.exe -O3 -Wall -fopenmp -I.\include -L.\lib -lraylibdll

if %ERRORLEVEL% equ 0 (
    echo [EXITO] Compilacion completada correctamente!
    echo         Se ha generado o actualizado 'boids.exe'.
    echo         ^(Recuerda que 'raylib.dll' debe estar junto al .exe para ejecutarse^)
) else (
    echo.
    echo [ERROR] La compilacion ha fallado.
)

echo.
pause
