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


import java.util.concurrent.TimeUnit;

import API_ROS2_Sunrise.ISocket;
import API_ROS2_Sunrise.TCPSocket;
import API_ROS2_Sunrise.UDPSocket;

import com.kuka.roboticsAPI.deviceModel.LBR;


public class LBR_status_reader extends Node{
	
	// Robot
	LBR lbr;
	
	// LBR status variables
	private Object isReadyToMove = null;
	private boolean hasActiveCommand;

	
	public LBR_status_reader(int port, LBR robot, String ConnectionType) {
		super(port,ConnectionType, "LBR status reader");
		this.lbr = robot;

		if (!(isSocketConnected())) {
			System.out.println("Starting thread to connect LBR status node....");
			Thread monitorLBRStatusConnections = new MonitorLBRStatusConnectionsThread();
			monitorLBRStatusConnections.start();
			}
		}
	
	@Override
	public void run() {
		while( isNodeRunning())
		{	
			sendStatus();
			try {
				TimeUnit.MILLISECONDS.sleep(10);
			} catch (InterruptedException e) {
				System.out.println(this.node_name + " thread could not sleep");
			}
		}
	}
	private boolean getReadyToMove() {
		return lbr.isReadyToMove();
	}
	
	private void updateActiveCommand(){
		try{
			hasActiveCommand = lbr.hasActiveMotionCommand();
		}catch(Exception e){}
	}
	
	
	private String generateStatusString() {

		String toSend= 	">lbr_statusdata ,"  + System.nanoTime() + 
				",ReadyToMove:" + getReadyToMove() + 
				",isLBRmoving:" + getisLBRMoving() + 
				",LBRsafetyStop:" + getEmergencyStop()+
				",PathFinished:" + getisPathFinished()
				; 
	return toSend;
	}
	
	public void sendStatus() {
		String statusString = generateStatusString();
		if(isNodeRunning()){
			try{
				this.socket.send_message(statusString);
				if(closed){
					System.out.println(this.node_name +" tried to send a message when application was closed");
				}
			}catch(Exception e){
				System.out.println("Could not send "+ this.node_name + " message to ROS: " + e);
			}
		}
	}
	
	public class MonitorLBRStatusConnectionsThread extends Thread {
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				
				createSocket();
				if (isSocketConnected()){
					break;
				}
				try {
					Thread.sleep(connection_timeout);
				} catch (InterruptedException e) {
					System.out.println("");
				}
				
			}
			if(!closed){
				System.out.println("Connection with LBR Status Node OK!");
				runmainthread();					
				}	
		}
	}
	
	
	public void setLBRemergencyStop(boolean stop){
		setEmergencyStop(stop);
	}

	@Override
	public void close() {
		closed = true;
		socket.close();
		System.out.println("LBR status closed!");

	}
	
}
