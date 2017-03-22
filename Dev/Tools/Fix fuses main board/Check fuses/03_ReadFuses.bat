:: 
:: bqZUM_BT328 production script. Spanish
::
:: author - Raul de Pablos Martin
:: version - 20140728
@echo off
cd tools

:: ONLY PROGRAMMING

:programming
 set /p var= Presione ENTER para iniciar%=%
 
 echo.
 echo Puertos COM:
 echo.
  mode | findstr "COM" | cut -d" " -f4 | cut -d":" -f1
 echo.
 
 set /p comPort= Escriba el numero del puerto COM:%=%
 ::@echo %comPort%

 
avrdude -Cavrdude.conf -patmega328p -carduino -PCOM%comPort% -b19200 -D -U lfuse:r:-:i -v

if ERRORLEVEL 1 (echo ERROR & pause & goto :end)

echo EXITO
pause
echo.
:: goto :programming

:end

