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
import java.net.InetSocketAddress;


// Implemented classes
import API_ROS2_Sunrise.DataController;
import com.kuka.nav.fdi.FDIConnection;

public class KMP_sensor_reader extends Node{
	

    // Data retrieval socket via FDI 
    public FDIConnection fdi;
	public DataController listener;
	String FDI_IP = "172.31.1.102";
	int FDI_port = 34001; 
	
	// Laser ports on controller
	private static int laser_B1 = 1801;
	private static int laser_B4 = 1802;
	

	
	public KMP_sensor_reader(int laserport, int odomport, String LaserConnectionType, String OdometryConnectionType) {
		super(laserport, LaserConnectionType, odomport, OdometryConnectionType, "KMP sensor reader");

		
		if (!(isLaserSocketConnected())) {
			Thread monitorLaserConnections = new MonitorLaserConnectionThread();
			monitorLaserConnections.start();
			}
				
		if (!(isOdometrySocketConnected())) {
			Thread monitorOdometryConnections = new MonitorOdometryConnectionThread();
			monitorOdometryConnections.start();
			}
		
		if (isSocketConnected()){ 
			this.fdiConnection();
		}
	}
	
	public void fdiConnection(){
		InetSocketAddress fdi_adress = new InetSocketAddress(FDI_IP,FDI_port);
		this.fdi = new FDIConnection(fdi_adress);
		this.listener = new DataController(laser_socket, odometry_socket);
		this.fdi.addConnectionListener(this.listener);
		this.fdi.addDataListener(this.listener);
		this.fdi.connect();
	}
	

	
	public class MonitorLaserConnectionThread extends Thread {
		public void run(){
			while(!(isLaserSocketConnected()) && (!(closed))) {

				createSocket("Laser");
				if(isLaserSocketConnected()){
					break;
				}	
				try {
					Thread.sleep(connection_timeout);
				} catch (InterruptedException e) {
					System.out.println("");
				}
			}
			if(!closed){
				if (fdi==null) {
					fdiConnection();
				}
				listener.setLaserSocket(laser_socket);
				subscribe_kmp_laser_data();	
				System.out.println("Connection with KMP Laser Node OK!");

				}	
		}
	}
	
	public class MonitorOdometryConnectionThread extends Thread {
		public void run(){
			while(!(isOdometrySocketConnected()) && (!(closed))) {
				
				createSocket("Odom");
				if (isOdometrySocketConnected()){
					break;
				}
				try {
					Thread.sleep(connection_timeout);
				} catch (InterruptedException e) {
					System.out.println("");
				}
			
		}	
			if(!closed){
				if (fdi==null) {
					fdiConnection();
				}
				listener.setOdometrySocket(odometry_socket);
				subscribe_kmp_odometry_data();
				System.out.println("Connection with KMP Odometry Node OK!");
				}
		}
	}
	
	public void subscribe_kmp_odometry_data() {
		while(!this.fdi.getSubscription().isOdometrySubscribed()){
    		this.fdi.getNewOdometry();

		}

	}
	public void subscribe_kmp_laser_data(){
		while(!this.fdi.getSubscription().isLaserSubscribed(laser_B1)){
			this.fdi.getNewLaserScan(laser_B1);
		}
		while(!this.fdi.getSubscription().isLaserSubscribed(laser_B4)){
			this.fdi.getNewLaserScan(laser_B4);
		}
	}
	
	@Override
	public void run() {
		if(isLaserSocketConnected()) {
			subscribe_kmp_laser_data();
		}
		if(isOdometrySocketConnected()) {
			subscribe_kmp_odometry_data();
		}
		boolean attempt_reconnection=true;
		while(!closed){
			if(!attempt_reconnection){attempt_reconnection = true;}
			
			while(!getListener().fdi_isConnected){
				if(attempt_reconnection){
					this.fdi.disconnect();					
					System.out.println("Initiate new FDI connection");
					this.fdiConnection();
					attempt_reconnection=false;
				}if(!attempt_reconnection && this.fdi.isConnected()){
					System.out.println("Reconnected FDI connection. Subscribing to sensor data:");
					subscribe_kmp_laser_data();
					subscribe_kmp_odometry_data();
				}
			}
		}
	}
	
	private DataController getListener() {
		return this.listener;
	}

	@Override
	public void close() {
		closed = true;
		try{
			this.fdi.disconnect();
			System.out.println("closing fdi");
		}catch(Exception e){
			System.out.println("Can not close FDI connection! : " + e);
		}
		try{
			laser_socket.close();
		}catch(Exception b){
			System.out.println("Can not close laser socket connection! : " + b);
		}
		try{
			odometry_socket.close();
		}catch(Exception c){
			System.out.println("Can not close odometry socket connection! : " + c);
		}
		System.out.println("KMP sensor closed!");
	}
	
	public boolean isLaserSocketConnected() {
		boolean res = false;
		try {
			res = laser_socket.isConnected();
		}catch(Exception e) {}
		return res;
	}
	
	public boolean isOdometrySocketConnected() {		
		boolean res = false;
		try {
			res = odometry_socket.isConnected();
		}catch(Exception e) {}
		return res;
	}
	
	public boolean isFDIConnected() {
		return this.listener.fdi_isConnected;
	}
	
	@Override
	public boolean isSocketConnected()  {
		return (isOdometrySocketConnected() || isLaserSocketConnected()); 
	}
	
}