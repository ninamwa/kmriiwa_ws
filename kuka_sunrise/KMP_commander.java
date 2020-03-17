package API_ROS2_Sunrise;

// Implemented classes


// RoboticsAPI
import API_ROS2_Sunrise.ISocket;
import API_ROS2_Sunrise.KMPjogger;
import API_ROS2_Sunrise.TCPSocket;
import API_ROS2_Sunrise.UDPSocket;

import com.kuka.jogging.provider.api.common.ICartesianJoggingSupport;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;


public class KMP_commander extends Node{
	

	// Robot
	KmpOmniMove kmp;
	
	// Motion variables: KMP
	ICommandContainer KMP_currentMotion;
	double[] velocities = {0.0,0.0,0.0};
	KMPjogger kmp_jogger;
	public volatile boolean KMP_is_Moving = false;

	
	public KMP_commander(int port, KmpOmniMove robot, String ConnectionType) {
		super(port,ConnectionType);
		this.kmp = robot;
		this.kmp_jogger = new KMPjogger((ICartesianJoggingSupport)kmp);
		
		if (!(isSocketConnected())) {
			System.out.println("Starting thread to connect KMP command node....");
			Thread monitorKMPCommandConnections = new MonitorKMPCommandConnectionsThread();
			monitorKMPCommandConnections.start();
			}else {
			setisKMPConnected(true);
		}
	}

	@Override
	public void run() {
		Thread emergencyStopThread = new MonitorEmergencyStopThread();
		emergencyStopThread.start();
		
		while(isNodeRunning())
		{
			String Commandstr = socket.receive_message(); 
	    	String []splt = Commandstr.split(" ");
	    
	    	if ((splt[0]).equals("shutdown")){
	    		System.out.println("KMP received shutdown");
				setShutdown(true);	
				break;
				}
	    	
	    	if ((splt[0]).equals("setTwist") && !getEmergencyStop()){
					setNewVelocity(Commandstr);
					}
	    	
		}
    }
	
	public class MonitorEmergencyStopThread extends Thread {
		public void run(){
			while(isNodeRunning()) {
				if (getEmergencyStop()){
					setNewVelocity("vel 0 0 0");
					}
				}
			}
		}
	
	

	public void setNewVelocity(String vel){
		String []lineSplt = vel.split(" ");

		if(lineSplt.length==4){
				this.velocities[0] =Double.parseDouble(lineSplt[1]); // x
				this.velocities[1] = Double.parseDouble(lineSplt[2]); // y
				this.velocities[2] = Double.parseDouble(lineSplt[3]);  // theta
				
				if(velocities[0] !=0 ||velocities[1] !=0 ||velocities[2] !=0 ) {
					  if(KMP_is_Moving) {
						  this.kmp_jogger.updateVelocities(this.velocities);
					  }
					  else {
						  this.kmp_jogger.updateVelocities(this.velocities);
						  this.kmp_jogger.startJoggingExecution();
						  KMP_is_Moving = true;
					  }
				  }else {
					  if(KMP_is_Moving) {
						  this.kmp_jogger.killJoggingExecution();
						  KMP_is_Moving=false;
					  }
				  }
		}
	}
	
	public class MonitorKMPCommandConnectionsThread extends Thread {
		int timeout = 3000;
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				if(getisLBRConnected()) {
					timeout = 5000;
				}
				createSocket();
				if (isSocketConnected()){
					setisKMPConnected(true);
					break;
				}
				try {
					Thread.sleep(timeout);
				} catch (InterruptedException e) {
					System.out.println("Waiting for connection to KMP commander node ..");
				}
			}
			if(!closed){
				System.out.println("Connection with KMP Command Node OK!");
				runmainthread();
				}	
		}
	}
	
	

	public boolean isKMPmoving() {
		return KMP_is_Moving;
	}
	
	@Override
	public void close() {
		closed = true;
		setShutdown(true);
		try{
			setNewVelocity("vel 0 0 0");
		}catch(Exception e){
			System.out.println("Could not kill jogging execution");
		}
		try{
			this.socket.close();
			}catch(Exception e){
				System.out.println("Could not close KMP commander connection: " +e);
			}
		System.out.println("KMP commander closed!");
 	}
	
}
