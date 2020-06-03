// Copyright 2019 Nina Marie Wahl og Charlotte Heggem.
// Copyright 2019 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package API_ROS2_Sunrise;

// Implemented classes


// RoboticsAPI
import API_ROS2_Sunrise.ISocket;

import com.kuka.nav.fdi.DataConnectionListener;
import com.kuka.nav.fdi.DataListener;
//import com.kuka.nav.fdi.FDIConnection;
import com.kuka.nav.fdi.data.CommandedVelocity;
import com.kuka.nav.fdi.data.Odometry;
import com.kuka.nav.fdi.data.RobotPose;
import com.kuka.nav.provider.LaserScan;

public class DataController implements DataListener, DataConnectionListener{

//	private static int laser_B1 = 1801;
//	private static int laser_B4 = 1802;
	public boolean fdi_isConnected;
	ISocket laser_socket;
	ISocket odometry_socket;

	Odometry odom;
	

	public DataController(ISocket laser_socket, ISocket odometry_socket) {
		this.fdi_isConnected=false;
		this.laser_socket = laser_socket;
		this.odometry_socket = odometry_socket;
	}


	@Override
	public void onNewCmdVelocity(CommandedVelocity arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onNewLaserData(LaserScan scan) {

		// TODO: sending both lasers
		if(fdi_isConnected && this.laser_socket.isConnected()){
			String scan_data = ">laserScan " +  scan.getTimestamp() + " " + scan.getLaserId()  + " " + scan.getRangesAsString();
			try{
				this.laser_socket.send_message(scan_data);
			}catch(Exception e){
				System.out.println("Could not send KMP laserdata to ROS: " + e);
		}
		}
	}

	@Override
	public void onNewOdometryData(Odometry odom) {
		if(fdi_isConnected && this.odometry_socket.isConnected()){
			try{
				String odom_data = ">odometry " + odom.getTimestamp() + " " + odom.getPose().toString() + " " + odom.getVelocity().toString();
				this.odometry_socket.send_message(odom_data);
			}catch(Exception e){
				System.out.println("Could not send KMP odometry data to ROS: " + e);
			}
	}
	}

	@Override
	public void onNewRobotPoseData(RobotPose arg0) {
		// TODO Auto-generated method stub
	}

	@Override
	public void onConnectionClosed() {
		fdi_isConnected = false;
		System.out.println("FDIConnection closed");
		}

	@Override
	public void onConnectionFailed(Exception arg0) {
		fdi_isConnected = false;
		System.out.println("FDIConnection failed");
		System.out.println(arg0);

	}

	@Override
	public void onConnectionSuccessful() {
		fdi_isConnected = true;
		System.out.println("FDIConnection successful");
		
	}

	@Override
	public void onConnectionTimeout() {
		System.out.println("FDIConnection timeout");
	}

	@Override
	public void onReceiveError(Exception arg0) {
		System.out.println("FDIconnection - Received error");
		System.out.println(arg0);
		
	}


	public void setLaserSocket(ISocket laser_socket2) {
			this.laser_socket = laser_socket2;
	}


	public void setOdometrySocket(ISocket odometry_socket2) {
		this.odometry_socket = odometry_socket2;
	}	
}
