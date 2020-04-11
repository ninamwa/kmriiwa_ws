## 1. Description

This package includes the Sunrise Application which needs to be installed on the KMR iiwa. 
It is described how you can do this in section 3.2 in the specialization project report.

## 2. Requirements
The Sunrise application is implemented using the listed software :
- Sunrise.OS 1.16
- Sunrise.Mobility 1.10 KMP 200
- Sunrise.Mobility 1.10 KMP\_KMR oM
- Sunrise.Workbench 1.16
- KUKA.NavigationSolution 1.13 KMP 200

The following .jar packages are required in addition to the defaults:

- com.kuka.nav.robot.fdi.api
- com.kuka.nav.encryption.provider
- com.kuka.nav.provider
- com.kuka.common
    
- log4j_1.2.17
- slf4j.api_1.7.7
- slf4j.log4j12_1.7.7
- bcprov-jdk15on-154

All the packages can be found in the software supplied by Sunrise in the plugins folder, but are not added to a project by default.
The exception is the last package listed. This package can be found by unpacking the *com.kuka.nav.encryption.provider* .jar file, and navigate to the lib folder.
