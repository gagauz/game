set JAVA_HOME=r:/jdk17
set PAth=%JAVA_HOME%\bin;%Path%

call javaw -jar "local-runner.jar" local-runner.properties

call java -cp ./../target/classes Runner > log.log
