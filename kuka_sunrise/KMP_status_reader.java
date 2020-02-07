package testwithrobot;

// Implemented classes
import java.util.concurrent.TimeUnit;

import testwithrobot.UDPSocket;
import testwithrobot.TCPSocket;
import testwithrobot.ISocket;

// RoboticsAPI
import com.kuka.roboticsAPI.deviceModel.OperationMode;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;



public class KMP_status_reader extends Thread{
	
	int port;
	ISocket socket;
	String ConnectionType;

	KmpOmniMove kmp;
	private OperationMode operation_mode;
	
	
	public KMP_status_reader(int UDPport, KmpOmniMove robot,String ConnectionType) {
		this.port = UDPport;
		this.kmp = robot;
		this.ConnectionType = ConnectionType;
		createSocket();
		operation_mode = null;
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
		while(isSocketConnected())
		{	
			try {
				TimeUnit.SECONDS.sleep(1);
			} catch (InterruptedException e) {
				System.out.println("statusthread could not sleep");
			}
			getOperationMode();
			//get
			//sørg for at dataen er ny
			//sendStatus();

		}
    }
	
	// TODO: send eller send_message?
	private void getOperationMode() {
		this.operation_mode = kmp.getOperationMode();
	}
	
	
	public void sendStatus() {
		String statusString = ">status " + this.operation_mode.toString() + " ";
		if(isSocketConnected()){
			try{
				this.socket.send_message(statusString);
			}catch(Exception e){
				System.out.println("Could not send Operation mode to ROS: " + e);
			}
		}
	}
	

	public void close() {
		try {
		socket.close();
		}catch(Exception e) {
			System.out.println("Could not close KMP status connection to ROS: " + e);
		}
	}
	
	public boolean isSocketConnected() {
		return socket.isConnected();
	}
	public boolean isSocketCreated() {
		return !(socket==null);
	}
}
