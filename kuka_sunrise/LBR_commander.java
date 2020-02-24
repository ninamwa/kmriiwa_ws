package API_ROS2_Sunrise;

// Implemented classes

// Robotics API
import API_ROS2_Sunrise.ISocket;
import API_ROS2_Sunrise.TCPSocket;
import API_ROS2_Sunrise.UDPSocket;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;


public class LBR_commander extends Thread{
	
	// RuntimeVariables
	public volatile boolean shutdown;
	int port;
	public volatile boolean closed = false;
	boolean isKMPConnected;
	public volatile boolean EmergencyStop = false;


	// Robot Specific
	LBR lbr;
	double a1_min;
	double a1_max; 
	double a2_min; 
	double a2_max;
	double a3_min; 
	double a3_max;
	double a4_min;
	double a4_max;		
	double a5_min;
	double a5_max;
	double a6_min;
	double a6_max;
	double a7_min;
	double a7_max;
	
	// Motion
	boolean LBR_is_Moving;
	private JointPosition CommandedjointPos;
	IMotionContainer currentmotion;
	private final double defaultVelocity = 0.3;

	// Socket
	ISocket socket;
	String ConnectionType;
	String CommandStr;

	// Motion variables: LBR
	ICommandContainer LBR_currentMotion;
	
	public LBR_commander(int port, LBR robot, String ConnectionType) {
		this.port = port;
		this.lbr = robot;
		this.shutdown = false;
		this.ConnectionType = ConnectionType;
		LBR_is_Moving = false;
		createSocket();
		setJointMaxAngles();
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
	
	public void run() {
		CommandedjointPos = lbr.getCurrentJointPosition();
		while(isSocketConnected() && (!(closed)))
		{   
			String Commandstr = socket.receive_message(); 
	    	String []splt = Commandstr.split(" ");
	    	if ((splt[0]).equals("shutdown")){
				this.shutdown = true;
				break;
				}
	    	
			if ((splt[0]).equals("setLBRmotion")){
				JointLBRMotion(Commandstr);
				}
		}
    }
	
	private void JointLBRMotion(String commandstr) {
		String []lineSplt = commandstr.split(" ");
		int jointIndex = Character.getNumericValue(lineSplt[1].charAt(1)) - 1 ;
		double direction = Double.parseDouble(lineSplt[2]);
		double jointAngle = 0;
		switch(jointIndex){
			case -1: // stop all
				if(LBR_is_Moving){
					currentmotion.cancel();
				}
				LBR_is_Moving = false;
				CommandedjointPos.set(lbr.getCurrentJointPosition());
				break;
			case 0: //A1
				if(direction==-1){
					jointAngle = a1_min;
				}else if(direction==1){
					jointAngle = a1_max;
				}
				break;
			case 1: //A2
				if(direction==-1){
					jointAngle = a2_min;
				}else if(direction==1){
					jointAngle = a2_max;
				}
				break;
			case 2: //A3
				if(direction==-1){
					jointAngle = a3_min;
				}else if(direction==1){
					jointAngle = a3_max;
				}
				break;
			case 3: //A4
				if(direction==-1){
					jointAngle = a4_min;
				}else if(direction==1){
					jointAngle = a4_max;
				}
				break;
			case 4:	//A5
				if(direction==-1){
					jointAngle = a5_min;
				}else if(direction==1){
					jointAngle = a5_max;
				}
				break;
			case 5: //A6
				if(direction==-1){
					jointAngle = a6_min;
				}else if(direction==1){
					jointAngle = a6_max;
				}
				break;
			case 6: //A7
				if(direction==-1){
					jointAngle = a7_min;
				}else if(direction==1){
					jointAngle = a7_max;
				}
				break;
			case 9: // A10: Move to driveposition
				if(LBR_is_Moving){
					currentmotion.cancel();
				}
	            lbr.move(ptp(lbr.getFrame("/DrivePos")).setJointVelocityRel(defaultVelocity));
	            LBR_is_Moving = false;
	            CommandedjointPos.set(lbr.getCurrentJointPosition());
	            break;
			default:
				System.out.println("Not a valid jointIndex: " +jointIndex);
		}
		
		if(!(jointIndex==-1 || jointIndex == 9)){
			if (direction== 0){
				jointAngle = lbr.getCurrentJointPosition().get(jointIndex);
			}
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
	
	public class MonitorEmergencyStopThread extends Thread {
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				if (CheckEmergencyStop()){
					if(!(EmergencyStop)){
						System.out.println("EMERGENCY STOP");
						if (LBR_is_Moving){
							currentmotion.cancel();
							LBR_is_Moving = false;		
						}
					}
					setEmergencyStop(true);
				}else{
					if(EmergencyStop){
						setEmergencyStop(false);
						System.out.println("Emergency liquidated!");
						}
					}
				}
			}
		}
	
	public boolean CheckEmergencyStop(){
		boolean emergencystop = this.EmergencyStop;
		try{
			emergencystop = (lbr.getSafetyState().getSafetyStopSignal().toInt()==2); 
		}catch(Exception e){}
		return emergencystop;
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
	public void close() {
		closed = true;
		if(LBR_is_Moving){
			currentmotion.cancel();
		}
		socket.close();
	}
	
	public boolean isSocketConnected() {
		return socket.isConnected();
	}
	public void setKMPConnected(boolean KMPConnected) {
		this.isKMPConnected = KMPConnected;
	}
	
	private void setJointMaxAngles() {
		// Get min and max angles in radians for all joints
		a1_min = Math.ceil(lbr.getJointLimits().getMinJointPosition().get(0)*100)/100;
		a1_max = Math.floor(lbr.getJointLimits().getMaxJointPosition().get(0)*100)/100;
		a2_min = Math.ceil(lbr.getJointLimits().getMinJointPosition().get(1)*100)/100;
		a2_max =  Math.floor(lbr.getJointLimits().getMaxJointPosition().get(1)*100)/100;		
		a3_min = Math.ceil(lbr.getJointLimits().getMinJointPosition().get(2)*100)/100;
		a3_max =  Math.floor(lbr.getJointLimits().getMaxJointPosition().get(2)*100)/100;		
		a4_min = Math.ceil(lbr.getJointLimits().getMinJointPosition().get(3)*100)/100;
		a4_max =  Math.floor(lbr.getJointLimits().getMaxJointPosition().get(3)*100)/100;		
		a5_min = Math.ceil(lbr.getJointLimits().getMinJointPosition().get(4)*100)/100;
		a5_max =  Math.floor(lbr.getJointLimits().getMaxJointPosition().get(4)*100)/100;
		a6_min = Math.ceil(lbr.getJointLimits().getMinJointPosition().get(5)*100)/100;
		a6_max =  Math.floor(lbr.getJointLimits().getMaxJointPosition().get(5)*100)/100;
		a7_min = Math.ceil(lbr.getJointLimits().getMinJointPosition().get(6)*100)/100;
		a7_max =  Math.floor(lbr.getJointLimits().getMaxJointPosition().get(6)*100)/100;
		
	}
	
}
