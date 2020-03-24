// Copyright 2019 Nina Marie Wahl og Charlotte Heggem.
// Copyright 2019 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package API_ROS2_Sunrise;

public abstract class Node extends Thread{
	
	// Runtime Variables
	public volatile static boolean shutdown = false;
	public volatile boolean closed = false;
	public volatile boolean EmergencyStop = false;
	
	public volatile static boolean isPathFinished;
	
	public volatile static boolean isKMPmoving;
	public volatile static boolean isLBRmoving;

	public volatile static boolean isKMPconnected;
	public volatile static boolean isLBRconnected;

	// Socket
	ISocket socket;
	String ConnectionType;
	String CommandStr;
	int port;
	
	// For KMP sensor reader:
	ISocket laser_socket;
	ISocket odometry_socket;
	int KMP_laser_port;
	int KMP_odometry_port;
	String LaserConnectionType;
	String OdometryConnectionType;
	
	
	public Node(int port1, String Conn1, int port2, String Conn2){
		this.KMP_laser_port = port1;
		this.KMP_odometry_port = port2;
		this.LaserConnectionType = Conn1;
		this.OdometryConnectionType = Conn2;
		
		createSocket("Laser");
		createSocket("Odom");
		
	}
	
	public Node(int port, String Conn) {
		this.ConnectionType=Conn;
		this.port = port;
		
		createSocket();
		
	}
	
	public void createSocket(){
		if (this.ConnectionType == "TCP") {
			 socket = new TCPSocket(this.port);
		}
		else {
			socket = new UDPSocket(this.port);
		}
	}
	
	public void createSocket(String Type){
		if (Type=="Laser"){
			if(LaserConnectionType == "TCP") {
				laser_socket = new TCPSocket(KMP_laser_port);
	
			}else {
				laser_socket = new UDPSocket(KMP_laser_port);
			}
		}else if(Type=="Odom") {
			if(OdometryConnectionType == "TCP") {
				odometry_socket = new TCPSocket(KMP_odometry_port);

			}else {
				odometry_socket = new UDPSocket(KMP_odometry_port);
			}
		}
	}
	
	public boolean isSocketConnected() {
		return socket.isConnected();
	}
	
	public boolean isNodeRunning() {
		return isSocketConnected() && (!(closed) && !shutdown);
	}
	
	public void setEmergencyStop(boolean es){
		this.EmergencyStop = es;
	}
	
	public boolean getEmergencyStop(){
		return EmergencyStop;
	}
	
	public void runmainthread(){
		this.run();
	}
	
	public boolean getShutdown() {
		return shutdown;
	}
	
	public void setShutdown(boolean in) {
		shutdown=in;
	}
	
	public abstract void run();
	
	public abstract void close();
	
	public boolean getisPathFinished() {
		return isPathFinished;
	}
	
	public void setisPathFinished(boolean in) {
		isPathFinished = in;
	}
	
	public boolean getisLBRMoving() {
		return isLBRmoving;
	}
	
	public void setisLBRMoving(boolean in) {
		isLBRmoving = in;
	}
	
	public boolean getisKMPMoving() {
		return isKMPmoving;
	}
	
	public void setisKMPMoving(boolean in) {
		isKMPmoving = in;
	}
	
	public boolean getisLBRConnected() {
		return isLBRconnected;
	}
	
	public void setisLBRConnected(boolean in) {
		isLBRconnected = in;
	}
	
	public boolean getisKMPConnected() {
		return isKMPconnected;
	}
	
	public void setisKMPConnected(boolean in) {
		isKMPconnected = in;
	}

}
