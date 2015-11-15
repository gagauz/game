set JAVA_HOME=r:/jdk17
set PAth=%JAVA_HOME%\bin;%Path%

call java -cp ./../target/classes Iter > log.log
