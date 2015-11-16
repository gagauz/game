set JAVA_HOME=r:/jdk17
set PAth=%JAVA_HOME%\bin;%Path%

call java -cp ./../target/classes Iter %cd% -2.0 2.0 -5.0 5.0 -1200.0 1200.0 0.7 0.7 0.7 0.7 20.0 20.0 
pause