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


//Implemented classes
import java.util.concurrent.TimeUnit;


//RoboticsAPI
import API_ROS2_Sunrise.ISocket;
import API_ROS2_Sunrise.TCPSocket;
import API_ROS2_Sunrise.UDPSocket;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.generated.ioAccess.ScannerSignalsIOGroup;
import com.kuka.roboticsAPI.deviceModel.OperationMode;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
//TODO: importere alle klasser fra SunriseOmniMoveMobilePlatform, scannerIO
public class KMP_status_reader extends Thread{

	// Runtime variables
	public volatile boolean closed = false;
	private volatile boolean KMP_is_Moving = false;
	private volatile boolean KMPemergencyStop = false;

	// Robot
	KmpOmniMove kmp;
	
	// Status variables
	private OperationMode operation_mode = null;
	private Object isReadyToMove = null; 
	ScannerSignalsIOGroup scannerIOGroup;
	private volatile boolean WarningField = false;
	private volatile boolean ProtectionField = false;

	// Socket
	int port;
	ISocket socket;
	String ConnectionType;
	
	public KMP_status_reader(int UDPport, KmpOmniMove robot,String ConnectionType, Controller controller) {
		this.port = UDPport;
		this.kmp = robot;
		this.ConnectionType = ConnectionType;
		createSocket();
		this.scannerIOGroup = new ScannerSignalsIOGroup(controller);
		
		if (!(isSocketConnected())) {
			System.out.println("Starting thread to connect KMP status node....");
			Thread monitorKMPStatusConnections = new MonitorKMPStatusConnectionsThread();
			monitorKMPStatusConnections.start();
			}
	}
	
	public void createSocket(){
		if (this.ConnectionType == "TCP") {
			 this.socket = new TCPSocket(this.port);
		}
		else {
			this.socket = new UDPSocket(this.port);
		}
	}
	public void run() {
		while(isSocketConnected() && (!(closed)))
		{	
// TODO: FIND OUT HOW MUCH TO SLEEP. SAMME RATE SOM ODOMETRY?
			updateOperationMode();
			updateReadyToMove();
			updateWarningFieldState();
			updateProtectionFieldState();
			sendStatus();
			
			if(!isSocketConnected() || (closed)){
				break;
			}
			try {
				TimeUnit.MILLISECONDS.sleep(30);
			} catch (InterruptedException e) {
			}
		}
 }

	
	private void updateOperationMode() {
		this.operation_mode = kmp.getOperationMode();
	}
	
	private void updateReadyToMove() {
		this.isReadyToMove = kmp.isReadyToMove();
	}
	
	// scannerIO
	private void updateWarningFieldState() {
//		signalnames =  Arrays.asList("WarningField_B1", "WarningField_B4", "WarningFieldComplete");
			try{
				this.WarningField  = kmp.getMobilePlatformSafetyState().isWarningFieldBreached();
			}catch(Exception e){
//					System.out.println("Could not read warning field: " + e);
				}
		}
	
	// scannerIO
	private void updateProtectionFieldState() {
//		signalnames =  Arrays.asList("WarningField_B1", "WarningField_B4", "WarningFieldComplete");
			try{
				// TRUE IF VIOLATED
				this.ProtectionField = kmp.getMobilePlatformSafetyState().isSafetyFieldBreached();
			}catch(Exception e){
//				System.out.println("Could not read protection field: " + e);
				}
		
		}

	
	// TODO: LEGG INN KMP_is_MOVING
	private String generateStatusString() {
		return 	">kmp_statusdata ,"  + System.nanoTime() + 
				",OperationMode:"+ this.operation_mode.toString() + 
				",ReadyToMove:" + this.isReadyToMove + 
				",WarningField:" + !this.WarningField + 
				",ProtectionField:" + !this.ProtectionField + 
				",isKMPmoving:" + KMP_is_Moving +
				",KMPsafetyStop:" + KMPemergencyStop;
	
	}
	
	public void sendStatus() {
		String toSend = this.generateStatusString();
		if(isSocketConnected() && (!(closed))){
			try{
				this.socket.send_message(toSend);
				if(closed){
					System.out.println("KMP status sender selv om han ikke får lov");
				}
			}catch(Exception e){
				System.out.println("Could not send Operation mode to ROS: " + e);
			}
		}
	}
	
	public class MonitorKMPStatusConnectionsThread extends Thread {
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				
				createSocket();
				if (isSocketConnected()){
					break;
				}
				try {
					Thread.sleep(5000);
				} catch (InterruptedException e) {
					System.out.println("");
				}
				
			}
			if(!closed){
				System.out.println("Connection with KMP Status Node OK!");
				runmainthread();					
				}	
		}
	}
	
	public void runmainthread(){
		this.run();
	}
	
	public void setKMPisMoving(boolean moving){
		KMP_is_Moving = moving;
	}
	
	public void setKMPemergencyStop(boolean stop){
		KMPemergencyStop  = stop;
	}

	public void close() {
		closed = true;
		socket.close();
		System.out.println("KMP status closed!");

	}
	
	public boolean isSocketConnected() {
		return socket.isConnected();
	}
	public boolean isSocketCreated() {
		return !(socket==null);
	}

}
