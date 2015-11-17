set JAVA_HOME=r:/jdk17
set PAth=%JAVA_HOME%\bin;%Path%

call java -cp ./../target/classes Iter %cd% 0.0 0.0 0.005 0.06 0.06 0.01 1.29 1.39 0.01 0.1 1.0 0.1 0.1 1.0 0.1 0 0 1
pause