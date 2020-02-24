package API_ROS2_Sunrise;


import java.util.concurrent.TimeUnit;

import API_ROS2_Sunrise.ISocket;
import API_ROS2_Sunrise.TCPSocket;
import API_ROS2_Sunrise.UDPSocket;

import com.kuka.roboticsAPI.deviceModel.LBR;


public class LBR_status_reader extends Thread{
	
	int port;
	ISocket socket;
	String ConnectionType;
	public volatile boolean closed = false;


	LBR lbr;
	
	private Object isReadyToMove = null;

	private boolean LBR_is_Moving= false;
	private boolean LBRemergencyStop = false;
	
	//
	public LBR_status_reader(int UDPport, LBR robot, String ConnectionType) {
		this.port = UDPport;
		this.lbr = robot;
		this.ConnectionType = ConnectionType;
		createSocket();
		if (!(isSocketConnected())) {
			System.out.println("Starting thread to connect LBR status node....");
			Thread monitorLBRStatusConnections = new MonitorLBRStatusConnectionsThread();
			monitorLBRStatusConnections.start();
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
	
	public void run() {
		while(isSocketConnected() && (!(closed)))
		{	
			//FIND OUT HOW MUCH TO SLEEP. SAMME RATE SOM ODOMETRY?
			updateReadyToMove();
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
	private void updateReadyToMove() {
		this.isReadyToMove = lbr.isReadyToMove();
	}
	
	
	private String generateStatusString() {
		return 	">lbr_statusdata ,"  + System.nanoTime() + 
				",ReadyToMove:" + this.isReadyToMove + 
				",LBRMoving:" + LBR_is_Moving + 
				",LBRsafetyStop:" + LBRemergencyStop;
	}
	
	public void sendStatus() {
		String statusString = generateStatusString();
		if(isSocketConnected() && (!(closed))){
			try{
				this.socket.send_message(statusString);
				if(closed){
					System.out.println("LBR status sender selv om han ikke får lov");
				}
			}catch(Exception e){
				System.out.println("Could not send Operation mode to ROS: " + e);
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
					Thread.sleep(5000);
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
	
	public void runmainthread(){
		this.run();
	}
	
	public void setLBRisMoving(boolean moving){
		LBR_is_Moving = moving;
	}
	
	public void setLBRemergencyStop(boolean stop){
		LBRemergencyStop  = stop;
	}

	public void close() {
		closed = true;
		socket.close();
	}
	
	public boolean isSocketConnected() {
		return socket.isConnected();
	}
	
}
