package com.smartcluster.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class AutonomousTruss {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPosition = new Pose2d(-35.0,-63,Math.toRadians(90));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 75, Math.toRadians(180), Math.toRadians(180), 10.754825427067597363146047734721)
                .setDimensions(352 / 25.4, 382 / 25.4)
                .build();

        Action auto = bot.getDrive().actionBuilder(startPosition)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-46,-46,Math.toRadians(225)),Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-47.4,-42,Math.toRadians(270)),Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-46,-46,Math.toRadians(225)),Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-34,-13,Math.toRadians(0)),Math.toRadians(60))
                .build();

        bot.runAction(auto);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
