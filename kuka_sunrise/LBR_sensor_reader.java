package API_ROS2_Sunrise;


import java.util.concurrent.TimeUnit;

import API_ROS2_Sunrise.ISocket;
import API_ROS2_Sunrise.TCPSocket;
import API_ROS2_Sunrise.UDPSocket;

import com.kuka.roboticsAPI.deviceModel.LBR;


public class LBR_sensor_reader extends Thread{
	
	// Runtime variables
	public volatile boolean closed = false;
	Boolean LBR_sensor_requested;

	// Robot
	LBR lbr;
	
	// LBR sensor
	private double[] JointPosition;
	private double[] MeasuredTorque;
	
	// Socket
	int port;
	ISocket socket;
	String ConnectionType;
	
	
	public LBR_sensor_reader(int UDPport, LBR robot, String ConnectionType) {
		this.port = UDPport;
		this.lbr = robot;
		LBR_sensor_requested = false;
		this.ConnectionType = ConnectionType;
		createSocket();
		
		if(!(isSocketConnected())){
			Thread monitorLBRsensorConnections = new MonitorSensorConnectionThread();
			monitorLBRsensorConnections.start();
		}else {
			LBR_sensor_requested=true;
		}

	}
	
	public void createSocket(){
		if (this.ConnectionType == "TCP") {
			 socket = new TCPSocket(this.port);
		}
		
		else {
			socket = new UDPSocket(this.port);
		}
	}
		
	public class MonitorSensorConnectionThread extends Thread {
		int timeout = 3000;
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {

				createSocket();
				if (isSocketConnected()){
					break;
				}
				try {
					Thread.sleep(timeout);
				} catch (InterruptedException e) {
					System.out.println("Waiting for connection to LBR commander node ..");
				}
			}
			if(!closed){
				LBR_sensor_requested=true;
				System.out.println("Connection with KMP Command Node OK!");
				runmainthread();
				}	
		}
	}
	
	public void runmainthread(){
		this.run();
	}
	
	public void run() {
		while(isSocketConnected() && (!(closed)))
		{	
			//FIND OUT HOW MUCH TO SLEEP. SAMME RATE SOM ODOMETRY?
			updateMeasuredTorque();
			updateJointPosition();
			
			if(!isSocketConnected() || (closed)){
				break;
			}
			sendStatus();
			try {
				TimeUnit.MILLISECONDS.sleep(30);
			} catch (InterruptedException e) {
				System.out.println("LBR sensor thread could not sleep");
			}
		}
	}

	private void updateJointPosition() {
		try{
		JointPosition = lbr.getCurrentJointPosition().getInternalArray();
		}catch(Exception e){}
	}

	private void updateMeasuredTorque() {
		try{
		MeasuredTorque = lbr.getMeasuredTorque().getTorqueValues();
		}catch(Exception e){}
	}

	private String generateSensorString() {
		return 	">lbr_sensordata ,"  + System.nanoTime() +  
				",JointPosition:" + JointPosition + 
				",MeasuredTorque:" + MeasuredTorque ;
	}
	
	public void sendStatus() {
		String sensorString = generateSensorString();
		if(isSocketConnected() && (!(closed))){
			try{
				this.socket.send_message(sensorString);
				if(closed){
					System.out.println("LBR sensor sender selv om han ikke får lov");
				}
			}catch(Exception e){
				System.out.println("Could not send Operation mode to ROS: " + e);
			}
		}
	}
	
	public void close() {
		closed = true;
		socket.close();
		System.out.println("LBR sensor closed!");

	}
	
	public boolean isSocketConnected() {
		return socket.isConnected();
	}
	
	public boolean isRequested() {
		return (LBR_sensor_requested); 
	}

	
}
