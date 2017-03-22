@echo off
cd tools

:: ONLY PROGRAMMING



:programming
 set /p var=Press INTRO to launch%=%
 echo Programming...
 
 :: Uncomment this to force the change
 :: avrdude -Cavrdude.conf -patmega328p -F -cstk500v1 -P\\.\COM25 -b19200 -Uefuse:w:0x05:m  2> nul
 
 avrdude -Cavrdude.conf -patmega328p -cstk500v1 -P\\.\COM25 -b19200 -Uefuse:w:0x05:m
 
 if ERRORLEVEL 1 (
  echo. & echo. & echo /!\/!\/!\/!\ ERROR /!\/!\/!\/!\ & echo. & echo. & echo. & goto :programming
 ) else (
  echo. & echo. & echo - SUCCESS -  & echo. & echo. & echo. & goto :programming
 )

:end

 

