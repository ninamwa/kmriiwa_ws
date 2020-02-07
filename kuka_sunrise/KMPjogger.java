package testwithrobot;


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

  
  private static final int JOG_START_DELAY = 0;

  
  private static final int JOG_UPDATE_PERIOD = 50;

  
  private static final int EXECUTOR_THREAD_POOL_SIZE = 2;

  
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
    this._joggableDevice.jog(this._velocities);
//    x = 05
//    y = 0.28 & xy = 0.28
//    theta = 0.25

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
	  System.out.println("stop jogger");
	  if (this._executor != null) {
      
      this._executor.shutdown();
    } 
    stopDevice();
  }
  
  public void updateVelocities(double[] vel) {
	  this._velocities[0] = vel[0];
	  this._velocities[1] = vel[1];
	  this._velocities[2] = vel[2];

  }
  
  

	  
  
  
}


