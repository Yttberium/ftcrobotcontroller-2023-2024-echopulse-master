package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.helpers.InstantCommand;
import com.smartcluster.oracleftc.commands.helpers.SequentialCommand;
import com.smartcluster.oracleftc.commands.helpers.WaitCommand;
import com.smartcluster.oracleftc.hardware.Subsystem;
import com.smartcluster.oracleftc.hardware.SubsystemFlavor;
import com.smartcluster.oracleftc.math.control.MotionState;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class DepositSubsystem extends Subsystem {
    // Hardware
    private final DigitalChannel rightDepositSensor, leftDepositSensor;
    private final ServoImplEx rightDeposit, leftDeposit, pitchDeposit, plane;

    // Config
    public static TrapezoidalMotionProfile depositMotionProfile = new TrapezoidalMotionProfile(3,4,4);
    public static double[] closedPositions=new double[] {0.02,0.04};
    public static double[] openPositions = new double[] {0.33,0.31};
    public static double[] planePositions = new double[] {0.0, 0.3};
    public DepositSubsystem(OpMode opMode) {
        super(opMode);
        rightDepositSensor=hardwareMap.get(DigitalChannel.class, "rightDepositSensor");
        leftDepositSensor=hardwareMap.get(DigitalChannel.class, "leftDepositSensor");
        rightDeposit=hardwareMap.get(ServoImplEx.class, "rightDeposit");
        leftDeposit=hardwareMap.get(ServoImplEx.class, "leftDeposit");
        pitchDeposit=hardwareMap.get(ServoImplEx.class, "pitchDeposit");
        plane = hardwareMap.get(ServoImplEx.class, "plane");
        closed[0]=false;
        closed[1]=false;
    }

    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.Mixed;
    }
    public Command reset()
    {
        return new SequentialCommand(
                new InstantCommand(()->plane.setPosition(planePositions[0])),
                close(),
                new InstantCommand(()->{
                    pitchDeposit.setPosition(angleToPosition(60));
                    updateTargetAngle(60);
                }),
                new WaitCommand(500)
        );
    }


    public Command launchPlane()
    {
        return new SequentialCommand(
                new InstantCommand(()->plane.setPosition(planePositions[1])),
                new WaitCommand(250),
                new InstantCommand(()->plane.setPosition(planePositions[0]))
        );
    }
    public boolean[] closed = new boolean[2];
    public Command close()
    {
        return new InstantCommand(()->{
            rightDeposit.setPosition(closedPositions[0]);
            leftDeposit.setPosition(closedPositions[1]);
            closed[0]=closed[1]=true;
        });
    }
    public Command openLeft()
    {
        return new InstantCommand(()->{
            leftDeposit.setPosition(openPositions[1]);
            closed[1]=false;
        });
    }
    public Command openRight()
    {
        return new InstantCommand(()->{
            rightDeposit.setPosition(openPositions[0]);
            closed[0]=false;
        });
    }

    public Command open()
    {
        return new InstantCommand(()->{
            rightDeposit.setPosition(openPositions[0]);
            leftDeposit.setPosition(openPositions[1]);
            closed[0]=closed[1]=false;
        });
    }
    public Command pitch(AtomicReference<Double> angle)
    {
        return Command.builder()
                .init(()->{
                    initialPitchPosition=pitchDeposit.getPosition();
                    targetPitchPosition=angleToPosition(angle.get());
                })
                .finished(()->Math.abs(pitchDeposit.getPosition()- targetPitchPosition)<=0.05)
                .requires(this)
                .build();
    }
    public Command update()
    {
        final ElapsedTime time = new ElapsedTime();
        return Command.builder()
                .update(()->{
                    if(Math.abs(pitchDeposit.getPosition()- targetPitchPosition)<=0.05)
                    {
                        time.reset();
                        initialPitchPosition=targetPitchPosition;
                        telemetry.addData("pitchSetPoint", targetPitchPosition);
                        pitchDeposit.setPosition(targetPitchPosition);

                    }else {
                        MotionState motionState = depositMotionProfile.getMotionState(Math.abs(targetPitchPosition-initialPitchPosition), time.seconds());
                        double position = initialPitchPosition+motionState.position*Math.signum(targetPitchPosition-initialPitchPosition);
                        pitchDeposit.setPosition(position);
                        telemetry.addData("pitchSetPoint", position);
                        telemetry.addData("pitchPosition", pitchDeposit.getPosition());

                    }
                })
                .requires(this)
                .build();
    }
    public Command telemetry()
    {
        return Command.builder()
                .update(()->{
                    telemetry.addData("rightDepositSensor", (!rightDepositSensor.getState())?1:0);
                    telemetry.addData("leftDepositSensor", (!leftDepositSensor.getState())?1:0);
                })
                .build();
    }

    // Helpers
    public boolean bothPixelsDropped()
    {
        return (leftDeposit.getPosition()>0.1) && (rightDeposit.getPosition()>0.1);
    }
    public boolean bothPixelsCleared()
    {
        return leftDepositSensor.getState() && rightDepositSensor.getState();
    }
    public boolean pixelsInBucket()
    {
        return !leftDepositSensor.getState() || !rightDepositSensor.getState();
    }
    private double initialPitchPosition=0;
    private double targetPitchPosition=0;
    public void updateTargetAngle(double angle)
    {
        if(!Double.isNaN(angle))
            targetPitchPosition=angleToPosition(angle);
    }
    public static double angleToPosition(double angle)
    {
        // 0.49 -- 90
        //
        return ((178.26+(angle-90))-19.5)/324.0;
    }

    public static class Manual {
        public double rightDepositPosition = closedPositions[0];
        public double leftDepositPosition = closedPositions[1];
        public double pitchDepositPosition=0.49;
        public double planePosition=planePositions[0];
    }

    public static Manual manual = new Manual();

    public Command manual()
    {
        return Command.builder()
                .update(()->{
                    rightDeposit.setPosition(manual.rightDepositPosition);
                    leftDeposit.setPosition(manual.leftDepositPosition);
                    pitchDeposit.setPosition(manual.pitchDepositPosition);
                    plane.setPosition(manual.planePosition);
                })
                .requires(this)
                .build();
    }

}
