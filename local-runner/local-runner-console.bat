set JAVA_HOME=r:/jdk17
set Path=%JAVA_HOME%\bin;%Path%


start java -Xmx800M -jar "local-runner.jar" local-runner-console.properties > lc.log
call java -Xmx256M  -cp ./../target/classes Runner %1 %2 %3 %4 %5 %6 >> log.csv

