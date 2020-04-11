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

// Implemented classes

// Robotics API
import API_ROS2_Sunrise.ISocket;
import API_ROS2_Sunrise.TCPSocket;
import API_ROS2_Sunrise.UDPSocket;
import API_ROS2_Sunrise.LBR_commander.PTPpoint;


// RoboticsAPI
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.ArrayList;
import java.util.List;


public class LBR_commander extends Node{
	

	// Robot Specific
	LBR lbr;
	
	// Motion: LBR
	private JointPosition CommandedjointPos;
	IMotionContainer currentmotion;
	private final double defaultVelocity = 0.2;
	AbstractFrame drivePos;
	ICommandContainer LBR_currentMotion;
	
	private List<SplineMotionJP<?>> splineSegments = new ArrayList<SplineMotionJP<?>>();
	PathThread followPathThread;


	public LBR_commander(int port,LBR robot, String ConnectionType, AbstractFrame drivepos) {
		super(port, ConnectionType);
		
		this.lbr = robot;
//		this.drivePos = drivepos;
//		lbr.setHomePosition(drivePos);
		
		if (!(isSocketConnected())) {
			System.out.println("Starting thread to connect LBR command node....");
			Thread monitorLBRCommandConnections = new MonitorLBRCommandConnectionsThread();
			monitorLBRCommandConnections.start();
			}else {
				setisLBRConnected(true);
			}

	}
	
	@Override
	public void run() {
		Thread emergencystopthread = new MonitorEmergencyStopThread();
		emergencystopthread.start();
		
		followPathThread = new PathThread();
		CommandedjointPos = lbr.getCurrentJointPosition();
		
		while(isNodeRunning())
		{   
			String Commandstr = socket.receive_message(); 
	    	String []splt = Commandstr.split(" ");
	    	
	    	if ((splt[0]).equals("shutdown")){
	    		System.out.println("LBR RECEIVED SHUTDOWN");
	    		setShutdown(true);
	    		break;
				}
	    	
			if ((splt[0]).equals("setLBRmotion")&& (!(getEmergencyStop()))){
				JointLBRMotion(Commandstr);
				}
			if ((splt[0]).equals("pathPointLBR")&& (!(getEmergencyStop()))){
				addPointToSegment(Commandstr);
				}
		}
    }
	private void addPointToSegment(String commandstr){
		int ArrayLength = lbr.getJointCount();
		String []lineSplt = commandstr.split(">");
		String pointType = lineSplt[1];
		

		double[] poses = new double[ArrayLength];
		double[] velocities = new double[ArrayLength];;
		double[] accelerations = new double[ArrayLength];;
		
		
		// Read message
		for(int i = 0; i < ArrayLength ; ++i){
			poses[i] = Double.parseDouble(lineSplt[2].split(" ")[i]);
			double vel = Math.abs(Double.parseDouble(lineSplt[3].split(" ")[i]));
			if(vel>1){
				vel = 1;
			}else if(vel<-1){
				vel = -1;
			}
			velocities[i] = vel;
			double accel = Math.abs(Double.parseDouble(lineSplt[4].split(" ")[i]));
			if(accel>1){
				accel = 1.0;
			}else if(vel<-1){
				accel = -1.0;
			}
			accelerations[i] = accel;
		}
		
		// KAN IKKE BRUKE STARTPOINT fordi velocity = 0
		if(pointType.equals("StartPoint")){
			splineSegments.clear();
		}else{
			PTPpoint ptp = new PTPpoint(pointType, new JointPosition(poses), velocities, accelerations);
			splineSegments.add(ptp.getPTP());
			if(pointType.equals("EndPoint")){
				followPathThread.run();
			}
		}
	}
	private void addPointToSegment(String commandstr){
		int ArrayLength = lbr.getJointCount();
		String []lineSplt = commandstr.split(">");
		String pointType = lineSplt[1];
		

		double[] poses = new double[ArrayLength];
		double[] velocities = new double[ArrayLength];;
		double[] accelerations = new double[ArrayLength];;
		
		
		// Read message
		for(int i = 0; i < ArrayLength ; ++i){
			poses[i] = Double.parseDouble(lineSplt[2].split(" ")[i]);
			double vel = Math.abs(Double.parseDouble(lineSplt[3].split(" ")[i]));
			if(vel>1){
				vel = 1;
			}else if(vel<-1){
				vel = -1;
			}
			velocities[i] = vel;
			double accel = Math.abs(Double.parseDouble(lineSplt[4].split(" ")[i]));
			if(accel>1){
				accel = 1.0;
			}else if(vel<-1){
				accel = -1.0;
			}
			accelerations[i] = accel;
		}
		
		// Skip start point as velocity and acceleration is 0
		if(pointType.equals("StartPoint")){
			splineSegments.clear();
		}else{
			PTPpoint ptp = new PTPpoint(pointType, new JointPosition(poses), velocities, accelerations);
			splineSegments.add(ptp.getPTP());
			if(pointType.equals("EndPoint")){
				followPathThread.run();
			}
		}
	}
	
	// TODO: what happens if an emergency stop is triggered?
	private class PathThread extends Thread {
		public void run(){
			if( !(getEmergencyStop()) && !splineSegments.isEmpty() && (isSocketConnected()) && (!(closed)) ){
					SplineJP spline = new SplineJP(splineSegments.toArray(new SplineMotionJP<?>[splineSegments.size()]));
					LBR_is_Moving = true;
					currentmotion = lbr.move(spline);
					if(currentmotion.isFinished()){
						LBR_is_Moving = false;
						
						splineSegments.clear();
						// TODO: DO SOMETHING WITH THE GRIPPER :D
					}
					}else{
						LBR_is_Moving = false;
					}
				}
			
		}

	
	private void JointLBRMotion(String commandstr) {
		if(isNodeRunning() && !(getEmergencyStop())){
			String []lineSplt = commandstr.split(" ");
			int jointIndex = Character.getNumericValue(lineSplt[1].charAt(1)) - 1 ;
			double direction = Double.parseDouble(lineSplt[2]);
			double jointAngle;
				if(jointIndex==-1){ // A10: Stop all
					if(getisLBRMoving()){
						currentmotion.cancel();
					}
					setisLBRMoving(false);
					CommandedjointPos.set(lbr.getCurrentJointPosition());
				}else if(jointIndex == 8){
				 // A9: Move to driveposition
					if(getisLBRMoving()){
						currentmotion.cancel();
					}
				    lbr.move(ptp(drivePos).setJointVelocityRel(defaultVelocity));
					setisLBRMoving(false);
				    CommandedjointPos.set(lbr.getCurrentJointPosition());
				}else{
					jointAngle = setJointAngle(jointIndex, direction);
					if(!(CommandedjointPos.get(jointIndex)==jointAngle)){
						CommandedjointPos.set(jointIndex, jointAngle);
						if(getisLBRMoving()){
							currentmotion.cancel();
						}
						currentmotion = lbr.moveAsync(ptp(CommandedjointPos).setJointVelocityRel(defaultVelocity));
						setisLBRMoving(true);
					}
				}	
		}
	}
	private double setJointAngle(int jointIndex, double direction) {
		double jointAngle = lbr.getCurrentJointPosition().get(jointIndex);
		if(direction==-1){
			jointAngle = Math.ceil(lbr.getJointLimits().getMinJointPosition().get(jointIndex)*100)/100;
		}else if(direction==1){
			jointAngle = Math.floor(lbr.getJointLimits().getMaxJointPosition().get(jointIndex)*100)/100;
		}
		return jointAngle;
	}
	
	public class MonitorEmergencyStopThread extends Thread {
		public void run(){
			while(isNodeRunning()) {
				if (getEmergencyStop()){
						if (getisLBRMoving()){
							currentmotion.cancel();
							CommandedjointPos.set(lbr.getCurrentJointPosition());
						}
						setisLBRMoving(true);
						}
				}
			}
		}
	
	public class MonitorLBRCommandConnectionsThread extends Thread {
		int timeout = 3000;
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				if(getisKMPConnected()) {
					timeout = 5000;
				}
				createSocket();
				if (isSocketConnected()){
					setisLBRConnected(true);
					break;
				}
				try {
					Thread.sleep(timeout);
				} catch (InterruptedException e) {
					System.out.println("Waiting for connection to LBR commander node ..");
				}
			}
			if(!closed){
				System.out.println("Connection with KMP Command Node OK!");
				runmainthread();
				}	
		}
	}
	
	
	@Override
	public void close(){
		setShutdown(true);
		closed = true;
		CommandedjointPos.set(lbr.getCurrentJointPosition());

		if(getisLBRMoving()){
			try{
			currentmotion.cancel();
			}catch(Exception e){
				System.out.println("LBR could not stop motion: " +e);
			}
		}
		socket.close();
		System.out.println("LBR command closed!");

	}

	
}
