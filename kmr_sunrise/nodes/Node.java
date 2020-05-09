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


import API_ROS2_Sunrise.ISocket;
import API_ROS2_Sunrise.TCPSocket;
import API_ROS2_Sunrise.UDPSocket;


public abstract class Node extends Thread{
	
	// Runtime Variables
	private volatile static boolean shutdown;
	public volatile boolean closed = false;
	private static volatile boolean EmergencyStop;
	
	private volatile static boolean PathFinished;
	
	private volatile static boolean isKMPmoving;
	private volatile static boolean isLBRmoving;

	private volatile static boolean isKMPconnected;
	private volatile static boolean isLBRconnected;

	public static int connection_timeout = 5000;

	// Socket
	protected ISocket socket;
	private String ConnectionType;
	private String CommandStr;
	private int port;
	
	// For KMP sensor reader:
	protected ISocket laser_socket;
	protected ISocket odometry_socket;
	private int KMP_laser_port;
	private int KMP_odometry_port;
	private String LaserConnectionType;
	private String OdometryConnectionType;
	
	protected String node_name;
// TODO: LEGGE INN NAVN PÅ NODER SOM PRINTES I TCP SOCKET! orker ikke mer porter	
	
	public Node(int port1, String Conn1, int port2, String Conn2, String nodeName){
		this.KMP_laser_port = port1;
		this.KMP_odometry_port = port2;
		this.LaserConnectionType = Conn1;
		this.OdometryConnectionType = Conn2;
		this.node_name = nodeName;
		setShutdown(false);
		setEmergencyStop(false);
		
		createSocket("Laser");
		createSocket("Odom");
		
	}
	
	public Node(int port, String Conn, String nodeName) {
		this.ConnectionType=Conn;
		this.port = port;
		this.node_name = nodeName;
		setShutdown(false);
		setEmergencyStop(false);
		
		createSocket();
		
	}
	
	public void createSocket(){
		if (this.ConnectionType == "TCP") {
			 this.socket = new TCPSocket(this.port);
		}
		else {
			this.socket = new UDPSocket(this.port);
		}
	}
	
	public void createSocket(String Type){
		if (Type=="Laser"){
			if(LaserConnectionType == "TCP") {
				this.laser_socket = new TCPSocket(KMP_laser_port);
			}else {
				this.laser_socket = new UDPSocket(KMP_laser_port);
			}
		}else if(Type=="Odom") {
			if(OdometryConnectionType == "TCP") {
				this.odometry_socket = new TCPSocket(KMP_odometry_port);

			}else {
				this.odometry_socket = new UDPSocket(KMP_odometry_port);
			}
		}
	}
	
	public boolean isSocketConnected() {
			return this.socket.isConnected();
	}
	
	public boolean isNodeRunning() {
		return this.isSocketConnected() && (!(closed) && !shutdown);
	}
	
	public static void setEmergencyStop(boolean es){
		EmergencyStop = es;
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
		System.out.println("shutdown set by " + this.node_name + " to " + in);
		shutdown=in;
	}
	
	public abstract void run();
	
	public abstract void close();
	
	public boolean getisPathFinished() {
		return PathFinished;
	}
	
	public void setisPathFinished(boolean in) {
		PathFinished = in;
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
