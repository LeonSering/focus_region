#!/bin/bash
javac -cp lib/jts-1.8.jar:lib/cplex.jar:lib/jgrapht-jdk1.6.jar:lib/poly2tri-core-0.1.0.jar:lib/jump-core-1.2.jar:lib/slf4j-api-1.6.0.jar src/*.java -d bin
jar cfm FocusMap.jar Manifest.txt -C bin . -C lib .

