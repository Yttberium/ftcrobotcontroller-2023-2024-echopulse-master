package org.firstinspires.ftc.teamcode.autonomous.opmode;
import org.firstinspires.ftc.teamcode.autonomous.roadrunner.MecanumDrive;
import android.util.Size;

import androidx.annotation.NonNull;

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
import com.smartcluster.meepmeep.AutonomousBlueBackdropPoses;
import com.smartcluster.meepmeep.AutonomousUtils;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.helpers.InstantCommand;
import com.smartcluster.oracleftc.commands.helpers.ParallelCommand;
import com.smartcluster.oracleftc.commands.helpers.RaceCommand;
import com.smartcluster.oracleftc.commands.helpers.SequentialCommand;
import com.smartcluster.oracleftc.commands.helpers.WaitCommand;

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
@Autonomous
public class AutonomousBlueCorner extends LinearOpMode{

    private CommandScheduler scheduler;
    private AutonomousUtils.AllianceColor color= AutonomousUtils.AllianceColor.Blue;

    private AutonomousUtils.Park park= AutonomousUtils.Park.Side;
    private static Action commandToAction(Command c)
    {
        return new Action() {
            private boolean initialized=false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized) {
                    c.init();
                    initialized = true;
                }
                c.update();
                if(c.finished())
                {
                    c.end(false);
                    return false;
                }else return true;
            }
        };
    }
    long lastTime=-1;
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousUtils.StartPosition startPosition = AutonomousUtils.StartPosition.Corner;
        scheduler = new CommandScheduler();
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, AutonomousBlueBackdropPoses.getStartPose(color, startPosition));
        AutonomousUtils.AutoCase autoCase = null;


    }
}
