## 1. Description

This package contains the Java program that is to be installed and launched on the Sunrise Cabinet. 

In addition to the main application, KMRiiwaSunriseApplication, the package include the following communication nodes:

- kmp_commander
- kmp_sensor_data
- kmp_status_data
- lbr_commander
- lbr_sensor_data
- lbr_status_data
- lbr_commander

The Javadocs of the package can be found at https://ninamwa.github.io/kmriiwa_ws/

The connection type (UDP/TCP) and port can be set for each node in the KMRiiwaSunriseApplication. 
The IP address to the remote computer must be defined in either of the TCPSocket or UDPsocket classes, depending on the choice of protocol. 

The files must be downloaded to a Sunrise project and synchronized to the controller from Sunrise Workbench. 
In addition to the default KUKAJavaLib, the following .jar packages must be added to the library of the project:

- com.kuka.common
- com.kuka.nav.encryption.provider.api
- com.kuka.nav.provider
- com.kuka.nav.provider
- com.kuka.robot.fdi.api
- bjprov-djk15on-154
- log4j
- slf4j.api
- slf4j.log4j12

## 2. Run
The KMRiiwaSunriseApplication can be launched from the smartPAD when the project is installed on the Sunrise Cabinet.
