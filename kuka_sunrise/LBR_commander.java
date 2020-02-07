package testwithrobot;

// Implemented classes
import testwithrobot.UDPSocket;
import testwithrobot.TCPSocket;
import testwithrobot.ISocket;

// Robotics API
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;


public class LBR_commander extends Thread{
	
	public volatile boolean shutdown;
	int port;
	ISocket socket;
	String ConnectionType;
	
	LBR lbr;
	String CommandStr;
	boolean LBR_is_Moving;

	
	// Motion variables: LBR
	ICommandContainer LBR_currentMotion;
	
	public LBR_commander(int port, LBR robot, String ConnectionType) {
		this.port = port;
		this.lbr = robot;
		this.shutdown = false;
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
			String Commandstr = socket.receive_message(); 
	    	String []splt = Commandstr.split(" ");
	    	if ((splt[0]).equals("shutdown")){
				this.shutdown = true;
				break;
				}
	    	
			if ((splt[0]).equals("setPose")){
				
				}
		}
    }
	
	public boolean isLBRMoving() {
		return LBR_is_Moving;
	}
	
	public boolean getShutdown() {
		return this.shutdown;
	}
	public void close() {
		socket.close();
	}
	
	public boolean isSocketConnected() {
		return socket.isConnected();
	}
	
}
