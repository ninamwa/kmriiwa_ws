package API_ROS2_Sunrise;

// Implemented classes

// Robotics API
import API_ROS2_Sunrise.ISocket;
import API_ROS2_Sunrise.TCPSocket;
import API_ROS2_Sunrise.UDPSocket;

// RoboticsAPI
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;


public class LBR_commander extends Thread{
	
	// Runtime Variables
	public volatile boolean shutdown = false;
	public volatile boolean closed = false;
	public volatile boolean EmergencyStop = false;
	boolean isKMPConnected;

	// Robot Specific
	LBR lbr;
	
	// Motion: LBR
	private JointPosition CommandedjointPos;
	IMotionContainer currentmotion;
	private final double defaultVelocity = 0.2;
	AbstractFrame drivePos;
	ICommandContainer LBR_currentMotion;
	public volatile boolean LBR_is_Moving = false;

	// Socket
	ISocket socket;
	String ConnectionType;
	String CommandStr;
	int port;


	public LBR_commander(int port, LBR robot, String ConnectionType, AbstractFrame drivepos) {
		this.port = port;
		this.lbr = robot;
		this.ConnectionType = ConnectionType;
		this.drivePos = drivepos;
		lbr.setHomePosition(drivePos);
		
		createSocket();
		
		if (!(isSocketConnected())) {
			System.out.println("Starting thread to connect LBR command node....");
			Thread monitorLBRCommandConnections = new MonitorLBRCommandConnectionsThread();
			monitorLBRCommandConnections.start();
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
	
	public boolean isSocketConnected() {
		return socket.isConnected();
	}
	
	public void run() {
		Thread emergencystopthread = new MonitorEmergencyStopThread();
		emergencystopthread.start();
		
		CommandedjointPos = lbr.getCurrentJointPosition();
		
		while(isSocketConnected() && (!(closed)))
		{   
			String Commandstr = socket.receive_message(); 
	    	String []splt = Commandstr.split(" ");
	    	
	    	if ((splt[0]).equals("shutdown")){
	    		System.out.println("LBR RECEIVED SHUTDOWN");
				this.shutdown = true;
				break;
				}
	    	
			if ((splt[0]).equals("setLBRmotion")&& (!(getEmergencyStop()))){
				JointLBRMotion(Commandstr);
				}
		}
    }
	
	private void JointLBRMotion(String commandstr) {
		if(isSocketConnected() && !(closed) && !(getEmergencyStop())){
			String []lineSplt = commandstr.split(" ");
			int jointIndex = Character.getNumericValue(lineSplt[1].charAt(1)) - 1 ;
			double direction = Double.parseDouble(lineSplt[2]);
			double jointAngle;
				if(jointIndex==-1){ // A10: Stop all
					if(LBR_is_Moving){
						currentmotion.cancel();
					}
					LBR_is_Moving = false;
					CommandedjointPos.set(lbr.getCurrentJointPosition());
				}else if(jointIndex == 8){
				 // A9: Move to driveposition
					if(LBR_is_Moving){
						currentmotion.cancel();
					}
				    lbr.move(ptp(drivePos).setJointVelocityRel(defaultVelocity));
				    LBR_is_Moving = false;
				    CommandedjointPos.set(lbr.getCurrentJointPosition());
				}else{
					jointAngle = setJointAngle(jointIndex, direction);
					if(!(CommandedjointPos.get(jointIndex)==jointAngle)){
						CommandedjointPos.set(jointIndex, jointAngle);
						if(LBR_is_Moving){
							currentmotion.cancel();
						}
						currentmotion = lbr.moveAsync(ptp(CommandedjointPos).setJointVelocityRel(defaultVelocity));
						LBR_is_Moving = true;
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
			while((isSocketConnected()) && (!(closed))) {
				if (getEmergencyStop()){
						if (LBR_is_Moving){
							currentmotion.cancel();
							CommandedjointPos.set(lbr.getCurrentJointPosition());
						}
						LBR_is_Moving = false;
						}
				}
			}
		}
	
	public void setEmergencyStop(boolean es){
		this.EmergencyStop = es;
	}
	
	public boolean getEmergencyStop(){
		return EmergencyStop;
	}
	
	public class MonitorLBRCommandConnectionsThread extends Thread {
		int timeout = 3000;
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				if(isKMPConnected) {
					timeout = 5000;
				}
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
				System.out.println("Connection with KMP Command Node OK!");
				runmainthread();
				}	
		}
	}
	public void runmainthread(){
		this.run();
	}
	
	public boolean isLBRMoving() {
		return LBR_is_Moving;
	}
	
	public boolean getShutdown() {
		return this.shutdown;
	}
	
	public void setKMPConnected(boolean KMPConnected) {
		this.isKMPConnected = KMPConnected;
	}
	
	public void close(){
		shutdown = true;
		closed = true;
		CommandedjointPos.set(lbr.getCurrentJointPosition());

		if(LBR_is_Moving){
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
