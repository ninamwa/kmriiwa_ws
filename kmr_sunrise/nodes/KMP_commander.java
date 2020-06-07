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


import API_ROS2_Sunrise.KMPjogger;

import com.kuka.jogging.provider.api.common.ICartesianJoggingSupport;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;


public class KMP_commander extends Node{

	// Robot
	KmpOmniMove kmp;
	
	// Motion variables: KMP
	ICommandContainer KMP_currentMotion;
	double[] velocities = {0.0,0.0,0.0};
	KMPjogger kmp_jogger;
	long jogging_period  = 1L;

	public KMP_commander(int port, KmpOmniMove robot, String ConnectionType) {
		super(port,ConnectionType, "KMP commander");
		this.kmp = robot;
		this.kmp_jogger = new KMPjogger((ICartesianJoggingSupport)kmp, jogging_period);
		
		
		if (!(isSocketConnected())) {
			Thread monitorKMPCommandConnections = new MonitorKMPCommandConnectionsThread();
			monitorKMPCommandConnections.start();
			}else {
				setisKMPConnected(true);
		}
	}

	@Override
	public void run() {
		Thread emergencyStopThread = new MonitorEmergencyStopThread();
		emergencyStopThread.start();
		
		while(isNodeRunning())
		{
			String Commandstr = this.socket.receive_message(); 
	    	String []splt = Commandstr.split(" ");
	    	if( !getShutdown() && !closed){
		    	if ((splt[0]).equals("shutdown")){
		    		System.out.println("KMP received shutdown");
					setShutdown(true);	
					break;
					}
		
		    	if ((splt[0]).equals("setTwist") && !getEmergencyStop()){
						setNewVelocity(Commandstr);
						}
		    	if ((splt[0]).equals("setPose") && !getEmergencyStop()){

					setNewPose(Commandstr);

					}

		
	    	}
		}System.out.println("KMPcommander no longer running");
    }
	
	public class MonitorEmergencyStopThread extends Thread {
		public void run(){
			while(isNodeRunning()) {
				if (getEmergencyStop() && getisKMPMoving()){
					  setisKMPMoving(false);
					  kmp_jogger.killJoggingExecution(getisKMPMoving());
					  System.out.println("successfully killed jogging execution from emergency stop");
					  if(!(KMP_currentMotion==null)){
						  KMP_currentMotion.cancel();
					  }
				}
				}
			}
		}
	
	public void setNewPose(String data){
		String []lineSplt = data.split(" ");
		if (lineSplt.length==4){
			double pose_x = Double.parseDouble(lineSplt[1]);
			double pose_y = Double.parseDouble(lineSplt[2]);
			double pose_theta = Double.parseDouble(lineSplt[3]);

			MobilePlatformRelativeMotion MRM = new MobilePlatformRelativeMotion(pose_x, pose_y, pose_theta);

			MRM.setVelocity(300, 10);
			MRM.setTimeout(100);
			MRM.setAcceleration(10, 10);

			if(kmp.isReadyToMove()) {
				System.out.println("moving");
				this.KMP_currentMotion =  kmp.moveAsync(MRM);
			}
			else {
				System.out.println("Kmp is not ready to move!");
			}

		}else{
			System.out.println("Unacceptable Mobile Platform Relative Velocity command!");

		}
	}

	
	public void setNewVelocity(String vel){
		String []lineSplt = vel.split(" ");

		if(lineSplt.length==4){
				this.velocities[0] =Double.parseDouble(lineSplt[1]); // x
				this.velocities[1] = Double.parseDouble(lineSplt[2]); // y
				this.velocities[2] = Double.parseDouble(lineSplt[3]);  // theta
				
				if(velocities[0] == 0 && velocities[1] == 0 && velocities[2] ==0) {
					  if(getisKMPMoving()) {
						  setisKMPMoving(false);
						  this.kmp_jogger.killJoggingExecution(true);
						  }
				}else{
					  if(getisKMPMoving()&& !getEmergencyStop()) {
						  this.kmp_jogger.updateVelocities(this.velocities);
					  }
					  else if(!getisKMPMoving() && !getEmergencyStop() && kmp.isReadyToMove()){
						  setisKMPMoving(true);
						  this.kmp_jogger.updateVelocities(this.velocities);
						  this.kmp_jogger.startJoggingExecution();
					  }else{System.out.println("Can not jog robot, is ready to move is: " +kmp.isReadyToMove() + " calculated value: " + KmpOmniMove.calculateReadyToMove(kmp.getSafetyState()));

					  }
					  
				  }
				}
		}
	
	public class MonitorKMPCommandConnectionsThread extends Thread {
		int timeout = 3000;
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				if(getisLBRConnected()) {
					timeout = 5000;
				}
				createSocket();
				if (isSocketConnected()){
					setisKMPConnected(true);
					break;
				}
				try {
					Thread.sleep(timeout);
				} catch (InterruptedException e) {
					System.out.println(node_name + " connection thread could not sleep");
				}
			}
			if(!closed){
				System.out.println("Connection with KMP Command Node OK!");
				runmainthread();
				}	
		}
	}


	@Override
	public void close() {
		closed = true;
		try{
			 this.kmp_jogger.killJoggingExecution(getisKMPMoving());
			 System.out.println("KMPJogger ended successfully");
		}catch(Exception e){
			System.out.println("Could not kill jogging execution");
		}
		try{
			this.socket.close();
		}catch(Exception e){
				System.out.println("Could not close KMP commander connection: " +e);
			}
		System.out.println("KMP commander closed!");
 	}
	
}