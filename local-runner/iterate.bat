set JAVA_HOME=r:/jdk17
set PAth=%JAVA_HOME%\bin;%Path%

call java -cp ./../target/classes Iter %cd% 0 0.1 0.01 0 0.5 0.05 0.5 1.5 0.1 0 0 1 0 0 1 0 0 1
pause