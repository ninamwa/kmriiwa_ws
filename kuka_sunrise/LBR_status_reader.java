package testwithrobot;


import com.kuka.roboticsAPI.deviceModel.LBR;
import testwithrobot.UDPSocket;
import testwithrobot.TCPSocket;
import testwithrobot.ISocket;


public class LBR_status_reader extends Thread{
	
	int port;
	ISocket socket;
	String ConnectionType;


	LBR lbr;
	
	
	public LBR_status_reader(int UDPport, LBR robot, String ConnectionType) {
		this.port = UDPport;
		this.lbr = robot;
		this.ConnectionType = ConnectionType;
		createSocket();
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
			sendStatus();
			sendOperationMode();

		}
    }
	
	public void sendStatus() {
		String statusString = "";
		socket.send_message(statusString);
	}
	
	private void sendOperationMode() {
		String opMode = lbr.getOperationMode().toString();
		if(isSocketConnected()){
		try{
			socket.send_message("OperationMode " + opMode);
		}catch(Exception e){
			System.out.println("Could not send Operation mode to ROS: " + e);
		}
	}
	}
	

	public void close() {
		try {
		socket.close();
		}catch(Exception e) {
			System.out.println("Could not close LBR status connection to ROS: " + e);
		}
	}
	
	public boolean isSocketConnected() {
		return socket.isConnected();
	}
	
}
