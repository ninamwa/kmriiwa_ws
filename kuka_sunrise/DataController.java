package com.kuka.roboticsAPI;

import com.kuka.nav.fdi.DataConnectionListener;
import com.kuka.nav.fdi.DataListener;
import com.kuka.nav.fdi.data.CommandedVelocity;
import com.kuka.nav.fdi.data.Odometry;
import com.kuka.nav.fdi.data.RobotPose;
import com.kuka.nav.provider.LaserScan;
import com.kuka.task.ITaskLogger;

public class DataController implements DataListener, DataConnectionListener{
	private String odom;
	private String laserscan;
	boolean RUN;
	ITaskLogger logger;
	
	public DataController(ITaskLogger log) {
		this.logger = log;
	}


	@Override
	public void onNewCmdVelocity(CommandedVelocity arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onNewLaserData(LaserScan arg0) {
		this.logger.info("New LaserData in Datacontroller");
		this.logger.info(arg0.toString());


	}

	@Override
	public void onNewOdometryData(Odometry arg0) {
		this.logger.info("New OdometryData in Datacontroller");
		this.logger.info(arg0.toString());

		

	}

	@Override
	public void onNewRobotPoseData(RobotPose arg0) {
		this.logger.info("New RobotPoseData in Datacontroller");
		
	}

	@Override
	public void onConnectionClosed() {
		RUN = false;
		this.logger.info("Connection closed in Datacontroller");

		
	}

	@Override
	public void onConnectionFailed(Exception arg0) {
		RUN = false;
		this.logger.info("Connection failed in Datacontroller");

	}

	@Override
	public void onConnectionSuccessful() {
		RUN = true;
		this.logger.info("Connection successfull in Datacontroller");
		
	}

	@Override
	public void onConnectionTimeout() {
		// TODO Auto-generated method stub
		this.logger.info("Connection timeout in Datacontroller");

		
	}

	@Override
	public void onReceiveError(Exception arg0) {
		this.logger.info("Receive error in Datacontroller");
		
	}
	
}
