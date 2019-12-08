package API_0612;

import java.net.InetSocketAddress;
import API_0612.KMPjogger;

import javax.inject.Inject;
import javax.inject.Named;

import org.apache.log4j.BasicConfigurator;

import API_0612.DataController;
import API_0612.TCPsocket;

import com.kuka.jogging.provider.api.common.ICartesianJoggingSupport;
import com.kuka.nav.fdi.FDIConnection;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformVelocityMotion;

public class API_ROS2_KMR extends RoboticsAPIApplication{
	
	// Declare KMP and Sunrise Cabinet
	@Inject
	@Named("KMR_200_2")
	public KmpOmniMove kmp;
	public Controller controller;
	
	// Motion variables
	ICartesianJoggingSupport joggabledevice;
	ICommandContainer _currentMotion;
	double[] velocities = new double[3];
	KMPjogger kmpJogger;
	boolean isKMPMoving;

	// Socket connection to ROS via TCP
	TCPsocket socket;
    
    // Data retrieval socket via FDI 
    public FDIConnection fdi;
	public DataController listener;
	
	// Laser ports
	private static int laser_B1 = 1801;
	private static int laser_B4 = 1802;
	
	// Standard velocities for the robot
	private static double jog_max_x = 1; //			 	  m/s
	private static double jog_max_y = 0.56;  // 		  m/s
	private static double jog_max_xy = 0.56; //			  m/s
	private static double jog_max_rot = 0.51; // 		rad/s
	
	// Decleare threads
		//Thread monitor_thread;
	

	public void initialize() {
		BasicConfigurator.configure();
		controller = getController("KUKA_Sunrise_Cabinet_1");
		getLogger().info("Initializing robot");
		kmp = getContext().getDeviceFromType(KmpOmniMove.class);
		joggabledevice  = (ICartesianJoggingSupport)kmp;
		kmpJogger = new KMPjogger(joggabledevice);

		//ROS CONNECTION
		socket = new TCPsocket();
		while(!socket.isConnected){}
		
		//DATA RETRIEVAL CONNECTION
		String serverIP = "172.31.1.102"; 
		int port = 34001; // do not edit
		this.fdiConnection(serverIP,port);
		
	}
	
	public void fdiConnection(String serverIP, int port){
		InetSocketAddress adr = new InetSocketAddress(serverIP,port);
		listener = new DataController(socket);
		fdi= new FDIConnection(adr);
		fdi.addConnectionListener(listener);
		fdi.addDataListener(listener);
		fdi.connect();
	}
	

	private void getOperationMode() {
		String opMode = kmp.getOperationMode().toString();
		if(this.socket.isConnected){
		try{
			socket.send_message("OperationMode " + opMode);
		}catch(Exception e){
			System.out.println("Could not send Operation mode to ROS: " + e);
		}
	}
	}
	
	public void setMobilePlatformVelocity(String data) {
		String []lineSplt = data.split(" ");
		if (lineSplt.length==4){
			double vx = Double.parseDouble(lineSplt[1]);
			double vy = Double.parseDouble(lineSplt[2]);
			double vTheta = Double.parseDouble(lineSplt[3]);
			double override = 1; //
			System.out.println("moveit 	");
			MobilePlatformVelocityMotion MP_vel_motion = new MobilePlatformVelocityMotion(vx,vy,vTheta,override);
		
			if(kmp.isReadyToMove()) {
				this._currentMotion =  kmp.move(MP_vel_motion);
			}
			else {
				getLogger().warn("Kmp is not ready to move!");
			}
		}else{
			getLogger().info("Unacceptable Mobile Platform Velocity Motion command!");
		}
	}
	
	public void setNewPose(String data){
		String []lineSplt = data.split(" ");
		System.out.println(data);
		if (lineSplt.length==4){
			double pose_x = Double.parseDouble(lineSplt[1]);
			double pose_y = Double.parseDouble(lineSplt[2]);
			double pose_theta = Double.parseDouble(lineSplt[3]);
		
			MobilePlatformRelativeMotion MRM = new MobilePlatformRelativeMotion(pose_x, pose_y, pose_theta);
			MRM.setVelocity(300, 10);
			MRM.setTimeout(100);
			MRM.setAcceleration(10, 10);
			
			if(kmp.isReadyToMove()) {
				System.out.println("moving");
				this._currentMotion =  kmp.moveAsync(MRM);
			}
			else {
				getLogger().warn("Kmp is not ready to move!");
			}
		}else{
			getLogger().info("Unacceptable Mobile Platform Relative Velocity command!");
		}
		
	}

	
	public void setNewVelocity(String vel){
		String []lineSplt = vel.split(" ");
		if (lineSplt.length==4){
			this.velocities[0] =Double.parseDouble(lineSplt[1]); // x
			this.velocities[1] = Double.parseDouble(lineSplt[2]); // y
			this.velocities[2] = Double.parseDouble(lineSplt[3]);  // theta
		}
		if(velocities[0] !=0 ||velocities[1] !=0 ||velocities[2] !=0 ) {
			  if(isKMPMoving) {
				  this.kmpJogger.updateVelocities(this.velocities);
			  }
			  else {
				  this.kmpJogger.updateVelocities(this.velocities);
				  this.kmpJogger.startJoggingExecution();
				  isKMPMoving = true;
			  }
		  }else {
			  if(isKMPMoving) {
				  this.kmpJogger.killJoggingExecution();
				  isKMPMoving=false;
			  }
		  }
	}
	
	public void shutdown_application(){
		setNewVelocity("vel 0 0 0");
		try{
			this.kmpJogger.killJoggingExecution();
		}catch(Exception e){
			System.out.println("Could not kill jogging execution");
		}
		System.out.println("----- Shutting down application -----");
		try{
			this._currentMotion.cancel();
		}catch(Exception e){
			System.out.println("No current motion to end: " + e);
		}
		
		try{	
	    	fdi.disconnect();
			}catch(Exception e){
				System.out.println("Could not close FDI connection: " +e);
			}
		try{	
	    	socket.close();
			}catch(Exception e){
				System.out.println("Could not close ROS connection: " +e);
			}
    	System.out.println("Disconnected ROS and FDI");
    	try{
    		dispose();
    	}catch(Exception e){
    		System.out.println("Application could not be terminated cleanly: " + e);
    	}
    	}
	
	public void get_kmp_data(){
    	if (socket.isConnected && listener.fdi_isConnected){
    		fdi.getNewOdometry();
    		fdi.getNewLaserScan(laser_B1);
    		fdi.getNewLaserScan(laser_B4);
    	}
	}

	public Runnable send_status_data = new Runnable(){
	    public void run(){
	    	while (socket.isConnected && listener.fdi_isConnected)
	    	{	if(Thread.interrupted()){
				break;
			}
	    		getOperationMode();
	   	
	    	}
	    }
	};
	
	public Runnable monitor_KMP = new Runnable(){
	    public void run(){
	    	while (socket.isConnected && listener.fdi_isConnected)
	    	{	if(Thread.interrupted()){
				break;
			}
	    		// CHECK LASER STATUS
	    		
	    		// CHECK IF MOTION IS ENABLED
	    		if (!kmp.isMotionEnabled()){
	    			System.out.println("Motion is not enabled for the KMP");
					shutdown_application();
	    		}
	    	}
	    }
	};
	
	public void run() {
		get_kmp_data();
		
		//monitor_thread = new Thread(monitor_KMP);
		//monitor_thread.start();
		
		while(socket.TCPConn.isConnected() && listener.fdi_isConnected)
		{   
	    	String CommandStr = socket.receive_message();
	    	String []lineSplt = CommandStr.split(" ");
	    	if ((lineSplt[0]).equals("shutdown")){
	    		System.out.println("Shutdown message received from ROS");
				shutdown_application();				
				}
			if ((lineSplt[0]).equals("setTwist")){
				setNewVelocity(CommandStr);
				}
			
			if ((lineSplt[0]).equals("setPose")){
				setNewPose(CommandStr);
			}
		}
		System.out.println("- - - APPLICATION TERMINATED - - -");
		shutdown_application();
	}

	public static void main(String[] args){
		API_ROS2_KMR app = new API_ROS2_KMR();

		app.runApplication();
	}
	
	
	

}
