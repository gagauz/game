set JAVA_HOME=r:/jdk17
set PAth=%JAVA_HOME%\bin;%Path%


start java -jar "local-runner.jar" local-runner-console.properties
call java -cp ./../target/classes Runner %1 %2 %3 %4 %5 %6 >> log.csv

