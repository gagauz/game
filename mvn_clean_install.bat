set file.encoding=UTF-8
set JAVA_HOME=R:/jdk1.7
call mvn -e clean install -Dmaven.test.skip=false -DdownloadSources=true -DdownloadSource=true


pause