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


// RoboticsAPI
import com.kuka.jogging.provider.api.common.ICartesianJoggingSupport;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
//import com.kuka.service.remoteControlSupport.internal.jog.interfaces.IRemoteControlJogger;

// Java Util
import java.util.TimerTask;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;





public class KMPjogger
{

  
//  private static final int JOG_START_DELAY = 0;

  
//  private static final int JOG_UPDATE_PERIOD = 50;

  
//  private static final int EXECUTOR_THREAD_POOL_SIZE = 2;

  
  public ScheduledExecutorService _executor;

  
  private ICartesianJoggingSupport _joggableDevice;
  
  private double[] _velocities;
  

  
  public KMPjogger( ICartesianJoggingSupport joggableDevice) {
	this._velocities = new double[3];
    this._joggableDevice = joggableDevice;
  }


  
  public class JogTimerTask
    extends TimerTask
  {
    public JogTimerTask() {}

    
    public void run() {
        
        if (KMPjogger.this._joggableDevice instanceof SunriseOmniMoveMobilePlatform && 
          !((SunriseOmniMoveMobilePlatform)KMPjogger.this._joggableDevice).isReadyToMove())
        {
        	KMPjogger.this._executor.shutdown();
        }
        KMPjogger.this._joggableDevice.jog(KMPjogger.this._velocities);
      
    
    }
  }


  public void stopDevice() {
    for (int i = 0; i < this._velocities.length; i++)
    {
      this._velocities[i] = 0.0D;
    }
    try{
    	this._joggableDevice.jog(this._velocities);
    }catch(Exception e){
    	System.out.println("Could not stop jogging device in KMPJogger");
    }

  }


  
  public void startJoggingExecution() {
    if (this._executor == null || this._executor.isShutdown())
    {
      this._executor = Executors.newScheduledThreadPool(2);
    }
    this._executor.scheduleAtFixedRate(new JogTimerTask(), 
        0L, 
        50L, 
        TimeUnit.MILLISECONDS);
  }


  public void killJoggingExecution() {
	  System.out.println("kill jogging execution in kmpjogger");
	  if (this._executor != null) {
      try{
    	  this._executor.shutdown();
      }catch(Exception e){
    	  System.out.println("Could not stop executor in KMPjogger");
      }
    } 
    stopDevice();
  }
  
  public void updateVelocities(double[] vel) {
	  this._velocities[0] = vel[0];
	  this._velocities[1] = vel[1];
	  this._velocities[2] = vel[2];

  }
  
  

	  
  
  
}
