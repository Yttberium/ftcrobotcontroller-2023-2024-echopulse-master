//package org.firstinspires.ftc.teamcode.autonomous.opmode;
//
//import android.util.Size;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.MinMax;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Twist2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.smartcluster.meepmeep.AutonomousBlueBackdropPoses;
//import com.smartcluster.meepmeep.AutonomousUtils;
//import com.smartcluster.oracleftc.commands.Command;
//import com.smartcluster.oracleftc.commands.CommandScheduler;
//import com.smartcluster.oracleftc.commands.helpers.InstantCommand;
//import com.smartcluster.oracleftc.commands.helpers.ParallelCommand;
//import com.smartcluster.oracleftc.commands.helpers.RaceCommand;
//import com.smartcluster.oracleftc.commands.helpers.SequentialCommand;
//import com.smartcluster.oracleftc.commands.helpers.WaitCommand;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.autonomous.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.autonomous.vision.CaseDetectionVisionProcessor;
//import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
//import org.firstinspires.ftc.teamcode.utils.Kinematics;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import java.util.HashMap;
//import java.util.List;
//import java.util.concurrent.atomic.AtomicBoolean;
//import java.util.concurrent.atomic.AtomicReference;
//
//@Autonomous
//public class AutonomousBlueCorner extends LinearOpMode {
//    private CommandScheduler scheduler;
//    private AutonomousUtils.AllianceColor color= AutonomousUtils.AllianceColor.Blue;
//
//    private AutonomousUtils.Park park= AutonomousUtils.Park.Side;
//    private static Action commandToAction(Command c)
//    {
//        return new Action() {
//            private boolean initialized=false;
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if(!initialized) {
//                    c.init();
//                    initialized = true;
//                }
//                c.update();
//                if(c.finished())
//                {
//                    c.end(false);
//                    return false;
//                }else return true;
//            }
//        };
//    }
//    long lastTime=-1;
//    private IntakeSubsystem intakeSubsystem;
//    private DepositSubsystem depositSubsystem;
//    private LiftSubsystem liftSubsystem;
//    private SliderSubsystem sliderSubsystem;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        AutonomousUtils.StartPosition startPosition = AutonomousUtils.StartPosition.Corner;
//        scheduler = new CommandScheduler();
//        CaseDetectionVisionProcessor processor = new CaseDetectionVisionProcessor(color, startPosition);
//        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, AutonomousBlueBackdropPoses.getStartPose(color, startPosition));
//        depositSubsystem = new DepositSubsystem(this);
//        intakeSubsystem = new IntakeSubsystem(this);
//        liftSubsystem = new LiftSubsystem(this);
//        sliderSubsystem = new SliderSubsystem(this);
//
//        WebcamName c270 = hardwareMap.get(WebcamName.class, "C270");
////        WebcamName arducam = hardwareMap.get(WebcamName.class, "Arducam");
////        SwitchableCameraName switchableCamera = ClassFactory.getInstance()
////                .getCameraManager().nameForSwitchableCamera(c270, arducam);
//
//        VisionPortal portal = new VisionPortal.Builder()
//                .setCamera(c270)
//                .setCameraResolution(new Size(1280, 720))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .setAutoStopLiveView(true)
//                .addProcessor(processor)
////                .addProcessor(frontProcessor)
//                .build();
//        AutonomousUtils.AutoCase autoCase = null;
//
////        portal.setProcessorEnabled(frontProcessor, false);
//        Command.run(new SequentialCommand(
//                depositSubsystem.reset(),
//                liftSubsystem.reset(),
//                sliderSubsystem.reset(),
//                new InstantCommand(sliderSubsystem::resetPitch),
//                new RaceCommand(
//                        depositSubsystem.update(),
//                        depositSubsystem.pitch(new AtomicReference<>(13.86))
//                )
//        ));
//        scheduler.schedule(new ParallelCommand(
//                depositSubsystem.update(),
//                liftSubsystem.update(),
//                sliderSubsystem.update()
//        ));
//        AtomicBoolean transferCondition= new AtomicBoolean();
//        HashMap<AutonomousUtils.AutoCase, Action> auto = new HashMap<>();
//        double[] firstRowKinematics = Kinematics.inverseKinematics(320, 120);
//        double[] safeRowKinematics = Kinematics.inverseKinematics(365, 180);
//        double[] secondRowKinematics = Kinematics.inverseKinematics(340, 260);
//        for (AutonomousUtils.AutoCase i : AutonomousUtils.AutoCase.values()) {
//            auto.put(i, new SequentialAction(
//                    new ParallelAction(
//                            caseAndBackdrop(mecanumDrive,color,i),
//                            commandToAction(intakeSubsystem.reset())
//                    ),
//                    commandToAction(
//                            new ParallelCommand(
//                                    sliderSubsystem.move(new AtomicReference<>(firstRowKinematics[0])),
//                                    liftSubsystem.move(new AtomicReference<>(firstRowKinematics[1])),
//                                    depositSubsystem.pitch(new AtomicReference<>(firstRowKinematics[2]))
//                            )
//                    ),
//                    commandToAction(
//                            new SequentialCommand(
//                                    new WaitCommand(300),
//                                    depositSubsystem.open(),
//                                    new ParallelCommand(
//                                            sliderSubsystem.move(new AtomicReference<>(safeRowKinematics[0])),
//                                            liftSubsystem.move(new AtomicReference<>(safeRowKinematics[1])),
//                                            depositSubsystem.pitch(new AtomicReference<>(safeRowKinematics[2]))
//                                    ),
//                                    new WaitCommand(500)
//
//                            )
//                    ),
//                    new ParallelAction(
//                            commandToAction(
//                                    new SequentialCommand(
//                                            depositSubsystem.close(),
//                                            new ParallelCommand(
//                                                    new SequentialCommand(
//                                                            new WaitCommand(250),
//                                                            depositSubsystem.pitch(new AtomicReference<>(13.86))
//                                                    ),
//                                                    new SequentialCommand(
//                                                            new WaitCommand(500),
//                                                            sliderSubsystem.move(new AtomicReference<>(0.0))
//                                                    ),
//                                                    new SequentialCommand(
//                                                            new WaitCommand(750),
//                                                            liftSubsystem.move(new AtomicReference<>(0.0))
//                                                    )
//                                            )
//                                    )
//                            ),
//                            commandToAction(
//                                    new SequentialCommand(
//                                            new WaitCommand(()->mecanumDrive.pose.position.x<0),
//                                            intakeSubsystem.stack(4)
//                                    )
//                            ),
//                            backdropToStack(mecanumDrive, color,i)
//                    ),
//                    oscillateUntilIntake(mecanumDrive,color),
//                    commandToAction(new WaitCommand(5000)),
//                    new ParallelAction(
//                            stackToBackdrop(mecanumDrive,color),
//                            commandToAction(
//                                    new ParallelCommand(
//                                            new SequentialCommand(
//                                                    new WaitCommand(300),
//                                                    intakeSubsystem.outtake(),
//                                                    new WaitCommand(500),
//                                                    intakeSubsystem.stop(),
//                                                    intakeSubsystem.lift(),
//                                                    //sliderSubsystem.closeContact(),
//                                                    intakeSubsystem.unlockGrippers(),
//                                                    new WaitCommand(200),
//                                                    intakeSubsystem.outtake(),
//                                                    new WaitCommand(800),
//                                                    intakeSubsystem.stop()
////                                                    sliderSubsystem.releaseContact()
//                                            ),
//                                            new SequentialCommand(
//                                                    new WaitCommand(()->mecanumDrive.pose.position.x> AutonomousUtils.TILE_WIDTH&&depositSubsystem.pixelsInBucket()&&sliderSubsystem.updateMode!= SliderSubsystem.UpdateMode.ENSURE_CONTACT),
//                                                    new ParallelCommand(
//                                                            sliderSubsystem.move(new AtomicReference<>(secondRowKinematics[0])),
//                                                            liftSubsystem.move(new AtomicReference<>(secondRowKinematics[1])),
//                                                            depositSubsystem.pitch(new AtomicReference<>(secondRowKinematics[2]))
//                                                    )
//                                            )
//                                    )
//
//                            )
//                    ),
//                    commandToAction(
//                            new SequentialCommand(
//                                    new WaitCommand(300),
//                                    depositSubsystem.open(),
//                                    new WaitCommand(500)
//
//                            )
//                    ),
//                    new ParallelAction(
//                            commandToAction(
//                                    new SequentialCommand(
//                                            depositSubsystem.close(),
//                                            new ParallelCommand(
//                                                    new SequentialCommand(
//                                                            new WaitCommand(250),
//                                                            depositSubsystem.pitch(new AtomicReference<>(13.86))
//                                                    ),
//                                                    new SequentialCommand(
//                                                            new WaitCommand(500),
//                                                            sliderSubsystem.move(new AtomicReference<>(0.0))
//                                                    ),
//                                                    new SequentialCommand(
//                                                            new WaitCommand(750),
//                                                            liftSubsystem.move(new AtomicReference<>(0.0))
//                                                    )
//                                            )
//                                    )
//                            )
//
//                    ),
//                    mecanumDrive.actionBuilder(AutonomousBlueBackdropPoses.getBackdropExterior(color, startPosition))
//                            .setTangent(AutonomousBlueBackdropPoses.getParkTangent(color, park))
//                            .splineToLinearHeading(AutonomousBlueBackdropPoses.getParkPose(color,park), AutonomousBlueBackdropPoses.getParkTangent(color, park))
//                            .build()
//            ));
//        }
//        while (opModeInInit()) {
//            autoCase = processor.autoCase;
//            telemetry.addData("autoCase", autoCase);
//            telemetry.update();
//        }
//        portal.setProcessorEnabled(processor, false);
//        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule module :
//                modules) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//        AtomicBoolean finishedAction = new AtomicBoolean(false);
//        Actions.runBlocking(
//                new ParallelAction(
//                        commandToAction(Command.builder()
//                                .update(() -> {
//                                    for (LynxModule module :
//                                            modules) {
//                                        module.clearBulkCache();
//                                    }
//                                    try {
//                                        scheduler.update();
//                                    } catch (InterruptedException e) {
//                                        e.printStackTrace();
//                                    }
//                                    long currentTime = System.nanoTime();
//                                    if (lastTime != -1)
//                                        telemetry.addData("loop(hz)", 1000 / ((currentTime - lastTime) / 1E6));
//                                    lastTime = System.nanoTime();
//                                    telemetry.update();
//                                })
//                                .finished(finishedAction::get)
//                                .build()),
//                        new SequentialAction(
//                                auto.get(autoCase),
//                                new InstantAction(() -> finishedAction.set(true))
//                        )
//                )
//        );
//    }
//    public static Action caseAndBackdrop(MecanumDrive drive, AutonomousUtils.AllianceColor color, AutonomousUtils.AutoCase autoCase)
//    {
//        final Pose2d startPose = AutonomousBlueBackdropPoses.getStartPose(color, AutonomousUtils.StartPosition.Corner);
//        final Pose2d casePose = AutonomousBlueBackdropPoses.getCasePose(color, AutonomousUtils.StartPosition.Corner,autoCase);
//        final Pose2d backdropPose = AutonomousBlueBackdropPoses.getBackdropPose(color, AutonomousUtils.StartPosition.Corner, autoCase);
//        final Twist2d backAway = new Twist2d(new Vector2d(-12,0), AutonomousUtils.mirrorColor(Math.toRadians(50),color));
//        return drive.actionBuilder(startPose)
//                .setTangent(startPose.heading)
//                .splineToLinearHeading(casePose, casePose.heading)
//                .setTangent(casePose.heading.log()-Math.PI)
//                .splineToLinearHeading(casePose.plus(backAway), casePose.plus(backAway).heading.log()-Math.PI)
//                .setTangent(Math.toRadians(0))
//                .splineToSplineHeading(backdropPose, Math.toRadians(0))
//                .build();
//    }
//
//    public static Action backdropToStack(MecanumDrive drive, AutonomousUtils.AllianceColor color, AutonomousUtils.AutoCase autoCase)
//    {
//        final Pose2d backdropPose = AutonomousBlueBackdropPoses.getBackdropPose(color, AutonomousUtils.StartPosition.Corner, autoCase);
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -2* AutonomousUtils.TILE_WIDTH- AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -2* AutonomousUtils.TILE_WIDTH- AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        final Pose2d stackPose = AutonomousBlueBackdropPoses.getStackPose(color, AutonomousUtils.StartPosition.Corner);
//        return drive.actionBuilder(backdropPose)
//                .setTangent(Math.toRadians(180))
//                .setReversed(true)
//                .splineToLinearHeading(cotPose, Math.toRadians(180))
//                .splineToLinearHeading(cot2Pose, Math.toRadians(180))
//                .splineToLinearHeading(stackPose, stackPose.heading.log()-Math.PI)
//                .build();
//    }
//    public static Action backdropExteriorToStack(MecanumDrive drive, AutonomousUtils.AllianceColor color)
//    {
//        final Pose2d backdropPose = AutonomousBlueBackdropPoses.getBackdropExterior(color, AutonomousUtils.StartPosition.Corner);
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -2* AutonomousUtils.TILE_WIDTH- AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -2* AutonomousUtils.TILE_WIDTH- AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        final Pose2d stackPose = AutonomousBlueBackdropPoses.getStack2Pose(color, AutonomousUtils.StartPosition.Corner);
//        return drive.actionBuilder(backdropPose)
//                .setTangent(Math.toRadians(180))
//                .setReversed(true)
//                .splineToSplineHeading(cotPose, Math.toRadians(180))
//                .splineToLinearHeading(cot2Pose, Math.toRadians(180))
//                .splineToLinearHeading(stackPose, stackPose.heading.log()-Math.PI)
//                .build();
//    }
//    public Action oscillateUntilIntake(MecanumDrive drive, AutonomousUtils.AllianceColor color)
//    {
//        final Pose2d stackPose = AutonomousBlueBackdropPoses.getStackPose(color, AutonomousUtils.StartPosition.Corner);
//        final Pose2d nearStackPose = stackPose.plus(new Twist2d(new Vector2d(-5.75,0),0));
//
//        TrajectoryActionBuilder[] builders = new TrajectoryActionBuilder[]
//                {
//                        drive.actionBuilder(stackPose)
//                                .setTangent(Math.toRadians(180))
//                                .setReversed(true)
//                                .splineToLinearHeading(nearStackPose, Math.toRadians(0), ((pose2dDual, posePath, v) -> 30), (pose2dDual, posePath, v) -> new MinMax(-30,30)),
//                        drive.actionBuilder(nearStackPose)
//                                .setTangent(Math.toRadians(0))
//                                .setReversed(false)
//                                .splineToLinearHeading(stackPose, Math.toRadians(180), ((pose2dDual, posePath, v) -> 30), (pose2dDual, posePath, v) -> new MinMax(-30,30))
//                };
//        Action[] oscillations = new Action[] {
//                builders[0].build(),
//                builders[1].build()
//        };
//        return new ParallelAction(
//                commandToAction(intakeSubsystem.intakeUntilPixels(false)),
//                new Action() {
//                    boolean init = false;
//                    int times=0;
//                    int oscillationIndex=0;
//                    ElapsedTime time=new ElapsedTime();
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        if(!init)
//                        {
//                            init=true;
//                            time.reset();
//                        }
//
//                        boolean a = oscillations[oscillationIndex].run(telemetryPacket);
//                        if(!a){
//                            times++;
//                            oscillations[oscillationIndex]=builders[oscillationIndex].build();
//                            oscillationIndex=(1-oscillationIndex);
//                            if(oscillationIndex==0)
//                                scheduler.schedule(intakeSubsystem.stack(0));
//                            if(oscillationIndex==1&&times>3)
//                                scheduler.schedule(
//                                        new SequentialCommand(
//                                                new WaitCommand(300),
//                                                intakeSubsystem.outtake(),
//                                                new WaitCommand(300),
//                                                intakeSubsystem.intake()
//                                        ));
//                        }
//                        if(time.seconds()>7.5)
//                        {
//                            return false;
//                        }
//
//
//
//                        return !intakeSubsystem.bothPixelsDetected();
//                    }
//                }
//        );
//
//
//    }
//    public Action oscillateUntilIntake2(MecanumDrive drive, AutonomousUtils.AllianceColor color)
//    {
//        final Pose2d stackPose = AutonomousBlueBackdropPoses.getStack2Pose(color, AutonomousUtils.StartPosition.Corner);
//        final Pose2d nearStackPose = stackPose.plus(new Twist2d(new Vector2d(-5.75,0),0));
//
//        TrajectoryActionBuilder[] builders = new TrajectoryActionBuilder[]
//                {
//                        drive.actionBuilder(stackPose)
//                                .setTangent(Math.toRadians(180))
//                                .setReversed(true)
//                                .splineToLinearHeading(nearStackPose, Math.toRadians(0), ((pose2dDual, posePath, v) -> 30), (pose2dDual, posePath, v) -> new MinMax(-30,30)),
//                        drive.actionBuilder(nearStackPose)
//                                .setTangent(Math.toRadians(0))
//                                .setReversed(false)
//                                .splineToLinearHeading(stackPose, Math.toRadians(180), ((pose2dDual, posePath, v) -> 30), (pose2dDual, posePath, v) -> new MinMax(-30,30))
//                };
//        Action[] oscillations = new Action[] {
//                builders[0].build(),
//                builders[1].build()
//        };
//        return new ParallelAction(
//                commandToAction(intakeSubsystem.intakeUntilPixels(false)),
//                new Action() {
//                    boolean init = false;
//                    int times=0;
//                    int oscillationIndex=0;
//                    ElapsedTime time=new ElapsedTime();
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        if(!init)
//                        {
//                            init=true;
//                            time.reset();
//                        }
//
//                        boolean a = oscillations[oscillationIndex].run(telemetryPacket);
//                        if(!a){
//                            times++;
//                            oscillations[oscillationIndex]=builders[oscillationIndex].build();
//                            oscillationIndex=(1-oscillationIndex);
//                            if(oscillationIndex==0)
//                                scheduler.schedule(intakeSubsystem.stack(0));
//                            if(oscillationIndex==1&&times>3)
//                                scheduler.schedule(
//                                        new SequentialCommand(
//                                                new WaitCommand(300),
//                                                intakeSubsystem.outtake(),
//                                                new WaitCommand(300),
//                                                intakeSubsystem.intake()
//                                        ));
//                        }
//                        if(time.seconds()>7.5)
//                        {
//                            return false;
//                        }
//
//
//
//                        return !intakeSubsystem.bothPixelsDetected();
//                    }
//                }
//        );
//
//
//    }
//    public static Action stackToBackdrop(MecanumDrive drive, AutonomousUtils.AllianceColor color) {
//        final Pose2d backdropPose = AutonomousBlueBackdropPoses.getBackdropExterior(color, AutonomousUtils.StartPosition.Corner);
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -2* AutonomousUtils.TILE_WIDTH- AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -2* AutonomousUtils.TILE_WIDTH- AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        final Pose2d stackPose = AutonomousBlueBackdropPoses.getStackPose(color, AutonomousUtils.StartPosition.Corner);
//        return drive.actionBuilder(stackPose)
//                .setTangent(Math.toRadians(0))
//                .setReversed(false)
//                .splineToLinearHeading(cot2Pose, Math.toRadians(0))
//
//                .splineToLinearHeading(cotPose, Math.toRadians(0))
//                .splineToLinearHeading(backdropPose, Math.toRadians(0))
//                .build();
//    }
//    public static Action stack2ToBackdropExterior(MecanumDrive drive, AutonomousUtils.AllianceColor color) {
//        final Pose2d backdropPose = AutonomousBlueBackdropPoses.getBackdropExterior(color, AutonomousUtils.StartPosition.Corner);
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -2* AutonomousUtils.TILE_WIDTH- AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -2* AutonomousUtils.TILE_WIDTH- AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        final Pose2d stackPose = AutonomousBlueBackdropPoses.getStack2Pose(color, AutonomousUtils.StartPosition.Corner);
//        return drive.actionBuilder(stackPose)
//                .setTangent(Math.toRadians(0))
//                .setReversed(false)
//                .splineToSplineHeading(cot2Pose, Math.toRadians(0))
//                .splineToLinearHeading(cotPose, Math.toRadians(0))
//                .splineToLinearHeading(backdropPose, Math.toRadians(0))
//                .build();
//    }
//}
