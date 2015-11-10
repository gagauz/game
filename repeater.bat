set JAVA_HOME=r:/jdk17
set PAth=%JAVA_HOME%\bin;%Path%
start java -cp ".;*;%~dp0/*" -jar repeater.jar %1