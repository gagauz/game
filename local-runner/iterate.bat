set JAVA_HOME=r:/jdk17
set PAth=%JAVA_HOME%\bin;%Path%

call java -cp ./../target/classes Iter %cd% 0.0 0.01 0.005 0.03 0.07 0.01 1.25 1.35 0.01 0 0 1 0 0 1 0 0 1
pause