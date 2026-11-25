@echo off
python "%~dp0build.py"
if %ERRORLEVEL% NEQ 0 (
    echo Derleme basarisiz oldu.
    pause
    exit /b %ERRORLEVEL%
)
echo.
echo Cikti: dist\PigeonRingGenerator.exe
pause
