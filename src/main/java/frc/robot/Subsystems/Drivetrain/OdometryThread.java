package frc.robot.Subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

public class OdometryThread extends Thread {
    private final Lock signalLock = new ReentrantLock();
    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static boolean isCANFD = new CANBus("*").isNetworkFD();
    private static OdometryThread instance = null;


    public static OdometryThread getOdometryThreadInstance(){
        if(instance == null){
            instance = new OdometryThread();
        }
        return instance;
    }

    private OdometryThread(){
        setName("OdometryThread");
        setDaemon(true);
    }

    public void start(){
        if(timestampQueues.size() > 0){
            super.start();
        }
    }

    public Queue<Double> registerSignal(StatusSignal<Angle> signal){
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalLock.lock();
        Drivetrain.odometryLock.lock();

        try{
            BaseStatusSignal[] newSignals = new BaseStatusSignal[(phoenixSignals.length + 1)];
            System.arraycopy(phoenixSignals,0,newSignals,0,phoenixSignals.length);
            newSignals[phoenixSignals.length] = signal;
            phoenixSignals = newSignals;
            phoenixQueues.add(queue);
        }finally{
            signalLock.unlock();
            Drivetrain.odometryLock.unlock();
        }
        return queue;
    }

    public Queue<Double> registerSignal(DoubleSupplier signal){
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalLock.lock();
        Drivetrain.odometryLock.lock();

        try{
            genericSignals.add(signal);
            genericQueues.add(queue);
        }finally{
            signalLock.unlock();
            Drivetrain.odometryLock.unlock();
        }
        return queue;
    }

    public Queue<Double> makeTimeStampQueue(){
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drivetrain.odometryLock.lock();
        try{
            timestampQueues.add(queue);
        }finally{
            Drivetrain.odometryLock.unlock();
        }
        return queue;
    }

    public void run(){
        while(true){
            signalLock.lock();
            try{
                if(isCANFD && phoenixSignals.length > 0){
                    BaseStatusSignal.waitForAll(2.0/DrivetrainConstants.odometryFrequency, phoenixSignals);
                }else{
                    Thread.sleep((long) (1000/DrivetrainConstants.odometryFrequency));
                    if(phoenixSignals.length > 0)BaseStatusSignal.refreshAll(phoenixSignals);
                }

            }catch(InterruptedException e){
                e.printStackTrace();
            }finally{
                signalLock.unlock();
            }


            Drivetrain.odometryLock.lock();
            try{
                double timeStamp = RobotController.getFPGATime() / 1e6;
                double latency = 0.0;
                for(BaseStatusSignal signal: phoenixSignals){
                    latency += signal.getTimestamp().getLatency();
                }

                if(phoenixSignals.length > 0){
                    timeStamp -= latency / phoenixSignals.length; //avg latency
                }

                for(int i = 0; i < phoenixSignals.length; i ++){
                    phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
                }

                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                }

                for(int i = 0; i < timestampQueues.size(); i ++){
                    timestampQueues.get(i).offer(timeStamp);
                }
            }finally{
                Drivetrain.odometryLock.unlock();
            }
        }
    }
}
