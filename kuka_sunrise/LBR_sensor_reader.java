package testwithrobot;


import com.kuka.roboticsAPI.deviceModel.ITorqueSensitiveRobot;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;

import testwithrobot.UDPSocket;
import testwithrobot.TCPSocket;
import testwithrobot.ISocket;



public class LBR_sensor_reader extends Thread{
	
	int port;
	ISocket socket;
	String ConnectionType;
	public volatile boolean close = false;

	Boolean LBR_sensor_requested;


	ITorqueSensitiveRobot LBR_torque_reader;
	TorqueSensorData LBR_sensor_data;
	
	LBR lbr;
	
	
	public LBR_sensor_reader(int UDPport, LBR robot, String ConnectionType) {
		this.port = UDPport;
		this.lbr = robot;
		this.LBR_torque_reader = (ITorqueSensitiveRobot)lbr;
		LBR_sensor_requested = false;
		this.ConnectionType = ConnectionType;
		createSocket();
		
		if(!(isSocketConnected())){
			Thread monitorSensorConnections = new MonitorSensorConnectionThread();
			monitorSensorConnections.start();
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
		public void run(){
			while(!(isSocketConnected()) && (!(close))) {
				
				createSocket();
				if (isSocketConnected()){
					break;
				}
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					System.out.println("");
				}
			}
			if(!close){
				System.out.println("Connection with LBRsensorNode OK!");
				LBR_sensor_requested = true;
				runmainthread();					
				}	
		}
	}
	
	public void runmainthread(){
		this.run();
	}
	
	
	public void run() {
		while(isSocketConnected() && (!(close)))
		{
			sendTorque();
			
			// SLEEP BELOW

		}
    }
	
	
	public void sendTorque() {
		//LBR_sensor_data = LBR_torque_reader.measure();
		String sensorString = LBR_sensor_data.toString();
		socket.send_message(sensorString);
	}
	

	public void close() {
		close = true;
		socket.close();
	}
	
	public boolean isSocketConnected() {
		return socket.isConnected();
	}
	
	public boolean isRequested() {
		return (LBR_sensor_requested); 
	}

	
}
