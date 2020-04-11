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
public class KMP_status_reader extends Node{

	// Runtime variables
	// TODO: Gjør om denne til node
	private volatile boolean KMP_is_Moving = false;

	// Robot
	KmpOmniMove kmp;
	
	// Status variables
	private OperationMode operation_mode = null;
	private Object isReadyToMove = null; 
	private volatile boolean WarningField = false;
	private volatile boolean ProtectionField = false;

	public KMP_status_reader(int port, KmpOmniMove robot,String ConnectionType, Controller controller) {
		super(port, ConnectionType);

		this.kmp = robot;
		
		if (!(isSocketConnected())) {
			System.out.println("Starting thread to connect KMP status node....");
			Thread monitorKMPStatusConnections = new MonitorKMPStatusConnectionsThread();
			monitorKMPStatusConnections.start();
			}
	}
	
	@Override
	public void run() {
		while(isNodeRunning())
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
				System.out.println("KMP status thread could not sleep");
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
				",KMPsafetyStop:" + EmergencyStop;
	
	}
	
	public void sendStatus() {
		String toSend = this.generateStatusString();
		if(isNodeRunning()){
			try{
				this.socket.send_message(toSend);
				if(closed){
					System.out.println("KMP status sender selv om han ikke f�r lov");
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

	public void setKMPisMoving(boolean moving){
		KMP_is_Moving = moving;
	}
	
	public void setKMPemergencyStop(boolean stop){
		EmergencyStop  = stop;
	}
	
	@Override
	public void close() {
		closed = true;
		socket.close();
		System.out.println("KMP status closed!");

	}


}