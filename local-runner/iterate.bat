set JAVA_HOME=r:/jdk17
set PAth=%JAVA_HOME%\bin;%Path%

call java -cp ./../target/classes Iter %cd% 0.0 0.5 0.5 1.0 0.1 0.5 1.0 1.3 2.0 4.0 15.0 30.0 1.1 1.6 
pause