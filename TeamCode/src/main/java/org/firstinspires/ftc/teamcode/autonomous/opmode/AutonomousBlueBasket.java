package org.firstinspires.ftc.teamcode.autonomous.opmode;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.helpers.InstantCommand;
import com.smartcluster.oracleftc.commands.helpers.ParallelCommand;
import com.smartcluster.oracleftc.commands.helpers.RaceCommand;
import com.smartcluster.oracleftc.commands.helpers.SequentialCommand;
import com.smartcluster.oracleftc.commands.helpers.WaitCommand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.utils.Kinematics;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class AutonomousBlueBasket extends LinearOpMode {
    private CommandScheduler scheduler;

    private static Action commandToAction(Command c) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    c.init();
                    initialized = true;
                }
                c.update();
                if (c.finished()) {
                    c.end(false);
                    return false;
                } else return true;
            }
        };
    }

    long lastTime = -1;
    private IntakeSubsystem intakeSubsystem;
    private DepositSubsystem depositSubsystem;
    private LiftSubsystem liftSubsystem;
    private SliderSubsystem sliderSubsystem;
    Pose2d startPosition = new Pose2d(-35.0,-63,Math.toRadians(-90));
    public static double speed=50;
    public static double[][] positions = new double[][] {
            new double[] { 335, 200},
            new double[] { 360, 300},
            new double[] { 385, 400},
            new double[] { 385, 560},
    }; //Position[<Which pair of coords>][<0 = x; 1 = y>]
    public static double PITCHDEPOSIT = 70;
    AtomicReference<Double> targetX=new AtomicReference<>(0.0);
    AtomicReference<Double> targetY=new AtomicReference<>(0.0);
    AtomicReference<Double> targetHeight=new AtomicReference<>(0.0);
    AtomicReference<Double> targetDistance=new AtomicReference<>(0.0);
    AtomicReference<Double> targetAngle=new AtomicReference<>(0.0);
    AtomicReference<Double> targetPitch=new AtomicReference<>(0.0);
    ElapsedTime time=new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        scheduler = new CommandScheduler();
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startPosition);
        depositSubsystem = new DepositSubsystem(this);
        intakeSubsystem = new IntakeSubsystem(this);
        liftSubsystem = new LiftSubsystem(this);
        sliderSubsystem = new SliderSubsystem(this);





        Command.run(new SequentialCommand(
                depositSubsystem.reset(),
                liftSubsystem.reset(),
                sliderSubsystem.reset(),
                intakeSubsystem.reset(),
                new InstantCommand(sliderSubsystem::resetPitch),

                new RaceCommand(
                        depositSubsystem.update(),
                        depositSubsystem.pitch(new AtomicReference<>(13.86))
                )
        ));
        scheduler.schedule(new ParallelCommand(
                depositSubsystem.update(),
                liftSubsystem.update(),
                sliderSubsystem.update()
        ));


        waitForStart();

        Action autoAction = new SequentialAction(
                  mecanumDrive.actionBuilder(startPosition)
                          .setTangent(Math.toRadians(90))
                          .splineToLinearHeading(new Pose2d(-47.5,-47.5,Math.toRadians(225)),Math.toRadians(90))
                          .build(),

                commandToAction(new SequentialCommand(
                        new InstantCommand(()->{
                            double[] ik = Kinematics.inverseKinematics(positions[3][0], positions[3][1]);
                            targetX.set(positions[3][0]);
                            targetY.set(positions[3][1]);
                            targetDistance.set(ik[0]);
                            targetHeight.set(ik[1]);
                            targetAngle.set(PITCHDEPOSIT);
                            targetPitch.set(ik[3]);
                        }),
                        new ParallelCommand(
                                depositSubsystem.pitch(targetAngle ),
                                sliderSubsystem.move(targetDistance),
                                liftSubsystem.move(targetHeight)

                        ),
                        new WaitCommand(100),
                        depositSubsystem.open(),
                        new WaitCommand(350),
                        new SequentialCommand(
                                new WaitCommand(50),
                                new SequentialCommand(
                                        new InstantCommand(()->{
                                            double[] ik = Kinematics.inverseKinematics(positions[1][0], positions[1][1]);
                                            targetX.set(positions[1][0]);
                                            targetY.set(positions[1][1]);
                                            targetDistance.set(ik[0]);
                                            targetHeight.set(ik[1]);
                                            targetAngle.set(PITCHDEPOSIT);
                                            targetPitch.set(ik[3]);
                                        }),
                                        new WaitCommand(50),
                                        new InstantCommand(()->{
                                            double newX = targetX.get()+Math.cos(Math.toRadians(60))*70;
                                            double newY = targetY.get()+Math.cos(Math.toRadians(60))*70;
                                            targetX.set(newX);
                                            targetY.set(newY);
                                            double[] ik = Kinematics.inverseKinematics(targetX.get(), targetY.get());
                                            targetDistance.set(ik[0]);
                                            targetHeight.set(ik[1]);
                                            targetAngle.set(PITCHDEPOSIT);
                                            targetPitch.set(ik[3]);
                                        })
                                ),
                                new ParallelCommand(
                                        sliderSubsystem.move(targetDistance),
                                        liftSubsystem.move(targetHeight),
                                        depositSubsystem.pitch(targetAngle)
                                ),
                                new WaitCommand(400),
                                new ParallelCommand(
                                        new SequentialCommand(
                                                new WaitCommand(500),
                                                sliderSubsystem.retract()
                                        ),
                                        new SequentialCommand(
                                                new WaitCommand(250),
                                                depositSubsystem.pitch(new AtomicReference<>(13.86)),
                                                depositSubsystem.close()
                                        ),
                                        new SequentialCommand(
                                                new WaitCommand(750),
                                                liftSubsystem.move(new AtomicReference<>(0.0))
                                        )
                                ),
                                intakeSubsystem.lift()))

                ),

                mecanumDrive.actionBuilder(new Pose2d(-47.5,-47.5,Math.toRadians(225)))
                        .splineToLinearHeading(new Pose2d(-47.6,-42,Math.toRadians(270)),Math.toRadians(90))
                        .build(),

                commandToAction(new SequentialCommand(
                        intakeSubsystem.stack(0),
                        new SequentialCommand(
                                intakeSubsystem.intake(),
                                new WaitCommand(300),
                                intakeSubsystem.stop(),
                                new ParallelCommand(
                                        intakeSubsystem.lift(),
                                        sliderSubsystem.closeContact()
                                ),
                                new WaitCommand(100),
                                intakeSubsystem.outtake(),
                                new WaitCommand(1000),
                                sliderSubsystem.releaseContact(),
                                intakeSubsystem.stop()
                        ))
                ),

                mecanumDrive.actionBuilder(new Pose2d(-47.6,-42,Math.toRadians(270)))
                        .splineToLinearHeading(new Pose2d(-47.5,-47.5,Math.toRadians(225)),Math.toRadians(90))
                        .build(),

                commandToAction(new SequentialCommand(
                        new InstantCommand(()->{
                                    double[] ik = Kinematics.inverseKinematics(positions[3][0], positions[3][1]);
                                    targetX.set(positions[3][0]);
                                    targetY.set(positions[3][1]);
                                    targetDistance.set(ik[0]);
                                    targetHeight.set(ik[1]);
                                    targetAngle.set(PITCHDEPOSIT);
                                    targetPitch.set(ik[3]);
                                }),
                                new ParallelCommand(
                                        depositSubsystem.pitch(targetAngle ),
                                        sliderSubsystem.move(targetDistance),
                                        liftSubsystem.move(targetHeight)

                                ),
                                new WaitCommand(100),
                                depositSubsystem.open(),
                                new WaitCommand(350),
                                new SequentialCommand(
                                        new WaitCommand(50),
                                        new SequentialCommand(
                                                new InstantCommand(()->{
                                                    double[] ik = Kinematics.inverseKinematics(positions[1][0], positions[1][1]);
                                                    targetX.set(positions[1][0]);
                                                    targetY.set(positions[1][1]);
                                                    targetDistance.set(ik[0]);
                                                    targetHeight.set(ik[1]);
                                                    targetAngle.set(PITCHDEPOSIT);
                                                    targetPitch.set(ik[3]);
                                                }),
                                                new WaitCommand(50),
                                                new InstantCommand(()->{
                                                    double newX = targetX.get()+Math.cos(Math.toRadians(60))*70;
                                                    double newY = targetY.get()+Math.cos(Math.toRadians(60))*70;
                                                    targetX.set(newX);
                                                    targetY.set(newY);
                                                    double[] ik = Kinematics.inverseKinematics(targetX.get(), targetY.get());
                                                    targetDistance.set(ik[0]);
                                                    targetHeight.set(ik[1]);
                                                    targetAngle.set(PITCHDEPOSIT);
                                                    targetPitch.set(ik[3]);
                                                })
                                        ),
                                        new ParallelCommand(
                                                sliderSubsystem.move(targetDistance),
                                                liftSubsystem.move(targetHeight),
                                                depositSubsystem.pitch(targetAngle)
                                        ),
                                        new WaitCommand(400),
                                        new ParallelCommand(
                                                new SequentialCommand(
                                                        new WaitCommand(500),
                                                        sliderSubsystem.retract()
                                                ),
                                                new SequentialCommand(
                                                        new WaitCommand(250),
                                                        depositSubsystem.pitch(new AtomicReference<>(13.86)),
                                                        depositSubsystem.close()
                                                ),
                                                new SequentialCommand(
                                                        new WaitCommand(750),
                                                        liftSubsystem.move(new AtomicReference<>(0.0))
                                                )
                                        ),
                                        intakeSubsystem.lift()))

                        ),
                mecanumDrive.actionBuilder(new Pose2d(-47.5,-47.5,Math.toRadians(225)))
                        .splineToLinearHeading(new Pose2d(-34,-13,Math.toRadians(0)),Math.toRadians(60))
                        .build(),
                mecanumDrive.actionBuilder(new Pose2d(-34,-13,Math.toRadians(0)))
                        .splineToLinearHeading(new Pose2d(-31,-13,Math.toRadians(0)),Math.toRadians(60))
                        .build(),
                commandToAction(new SequentialCommand(
                        liftSubsystem.move(new AtomicReference<>(2950.0)),
                        sliderSubsystem.move(new AtomicReference<>(400.0)),
                        depositSubsystem.pitch(new AtomicReference<>(135.0))
                ))


        );
        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for(LynxModule lynxModule: lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        Canvas c = new com.acmerobotics.dashboard.canvas.Canvas();
        autoAction.preview(c);
        boolean b = true;
        while (b && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());

            b = autoAction.run(p);
            scheduler.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            for(LynxModule lynxModule: lynxModules)
                lynxModule.clearBulkCache();
        }

        while (opModeIsActive()){
            Actions.runBlocking(autoAction);

        }


    }
}


