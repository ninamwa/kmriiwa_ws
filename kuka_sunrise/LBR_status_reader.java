package testwithrobot;


import com.kuka.roboticsAPI.deviceModel.LBR;
import testwithrobot.UDPSocket;
import testwithrobot.KMP_status_reader.MonitorKMPStatusConnectionsThread;
import testwithrobot.TCPSocket;
import testwithrobot.ISocket;


public class LBR_status_reader extends Thread{
	
	int port;
	ISocket socket;
	String ConnectionType;
	public volatile boolean close = false;


	LBR lbr;
	
	
	public LBR_status_reader(int UDPport, LBR robot, String ConnectionType) {
		this.port = UDPport;
		this.lbr = robot;
		this.ConnectionType = ConnectionType;
		createSocket();
		if (!(isSocketConnected())) {
			System.out.println("Starting thread to connect KMP status node....");
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
	
	public class MonitorLBRStatusConnectionsThread extends Thread {
		public void run(){
			while(!(isSocketConnected()) && (!(close))) {
				
				createSocket();
				if (isSocketConnected()){
					break;
				}
				try {
					this.sleep(5000);
				} catch (InterruptedException e) {
					System.out.println("");
				}
			}
			if(!close){
				System.out.println("Connection with LBR status Node OK!");
				runmainthread();					
				}	
		}
	}
	public void runmainthread(){
		this.run();
	}
	

	public void close() {
		close = true;
		socket.close();
	}
	
	public boolean isSocketConnected() {
		return socket.isConnected();
	}
	
}
