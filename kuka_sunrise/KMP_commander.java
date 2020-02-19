package testwithrobot;

// Implemented classes
import testwithrobot.KMPjogger;
import testwithrobot.UDPSocket;
import testwithrobot.TCPSocket;
import testwithrobot.ISocket;


// RoboticsAPI
import com.kuka.jogging.provider.api.common.ICartesianJoggingSupport;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;


public class KMP_commander extends Thread{
	
	public volatile boolean shutdown;
	public volatile boolean closed = false;
	public volatile boolean EmergencyStop = false;
	public volatile boolean KMP_is_Moving;


	int port;
	ISocket socket;
	String ConnectionType;
	boolean isLBRConnected;

	KMPjogger kmp_jogger;
	KmpOmniMove kmp;
	
	// Motion variables: KMP
	ICommandContainer KMP_currentMotion;
	double[] velocities = new double[3];
	
	public KMP_commander(int port, KmpOmniMove robot, String ConnectionType) {
		this.port = port;
		this.kmp = robot;
		this.kmp_jogger = new KMPjogger((ICartesianJoggingSupport)kmp);
		this.shutdown = false;
		this.ConnectionType = ConnectionType;
		createSocket();
		
		if (!(isSocketConnected())) {
			System.out.println("Starting thread to connect KMP command node....");
			Thread monitorKMPCommandConnections = new MonitorKMPCommandConnectionsThread();
			monitorKMPCommandConnections.start();
			}
	}

	public void createSocket(){
		if (this.ConnectionType == "TCP") {
			 this.socket = new TCPSocket(this.port);
		}
		else {
			this.socket = new UDPSocket(this.port);
		}
	}
	
	public void run() {
		Thread emergencyStopThread = new MonitorEmergencyStopThread();
		emergencyStopThread.start();
		
		while(isSocketConnected() && (!closed))
		{
			String Commandstr =this.socket.receive_message(); 
	    	String []splt = Commandstr.split(" ");
	    	if ((splt[0]).equals("shutdown")){
				this.closed = true;	
				this.shutdown = true;	
				break;
				}
	    	
			if ((splt[0]).equals("setTwist") && (!(EmergencyStop))){
				setNewVelocity(Commandstr);
				}
			
		}
		System.out.println("KMP_COMMAND DISCONNECTED");
    }
	
	public class MonitorEmergencyStopThread extends Thread {
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				if (CheckEmergencyStop()){
					if(!(EmergencyStop)){
						System.out.println("EMERGENCY STOP");
						if (KMP_is_Moving){
							kmp_jogger.killJoggingExecution();
							KMP_is_Moving = false;		
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
	
	// TODO: HVA SKJER HVIS JOG STOPPER AV PROTECTION ZONE????	- nå stopper den bare å venter. forhåpentligvis får vi kjøre igjen.
	public boolean CheckEmergencyStop(){
		boolean emergencystop = this.EmergencyStop;
		try{
			emergencystop = (kmp.getSafetyState().getSafetyStopSignal().toInt()==2);
		}catch(Exception e){}
		return emergencystop;
	}

	public void setNewVelocity(String vel){
		String []lineSplt = vel.split(" ");
		if (lineSplt.length==4){
			this.velocities[0] =Double.parseDouble(lineSplt[1]); // x
			this.velocities[1] = Double.parseDouble(lineSplt[2]); // y
			this.velocities[2] = Double.parseDouble(lineSplt[3]);  // theta
		}
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
	
	public class MonitorKMPCommandConnectionsThread extends Thread {
		int timeout = 3000;
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				if(isLBRConnected) {
					timeout = 5000;
				}
				createSocket();
				if (isSocketConnected()){
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
	
	
	
	public void setEmergencyStop(boolean es){
		this.EmergencyStop = es;
	}
	public void runmainthread(){
		this.run();
	}
	
	public boolean isKmpMoving() {
		return KMP_is_Moving;
	}
	
	public boolean getShutdown() {
		return this.shutdown;
	}
	
	public void close() {
		closed = true;
		setNewVelocity("vel 0 0 0");
		try {
			kmp_jogger.killJoggingExecution();
		}catch(Exception e){
			System.out.println("Could not kill jogging execution");
		}
		try{
			this.socket.close();
			}catch(Exception e){
				System.out.println("Could not close KMP commander connection: " +e);
			}
	}
	
	public boolean isSocketConnected() {
		return this.socket.isConnected();
	}
	
	public boolean isSocketCreated() {
		return !(this.socket==null);
	}
	
	public void setLBRConnected(boolean LBRConnected) {
		this.isLBRConnected = LBRConnected;
	}
}
