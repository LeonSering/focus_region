#!/bin/bash
java -Djava.library.path=$HOME/opt/CPLEX/cplex/bin/x86-64_linux -cp FocusMap.jar:lib/cplex.jar:lib/jgrapht-jdk1.6.jar:lib/jts-1.8.jar:lib/jump-core-1.2.jar:lib/poly2tri-core-0.1.0.jar:lib/slf4j-api-1.6.0.jar focusmap.Main 
