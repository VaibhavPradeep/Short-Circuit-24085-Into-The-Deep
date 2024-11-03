package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "ScrimBotAutoHumanPlayerSide")
public class ScrimBotAutoHumanPlayerSide extends OpMode{
    ScrimBotAutoMethods auto = new ScrimBotAutoMethods();

    // 1
    private double forwardToHighRung = 24; // Distance to drive forward to the high rung (inches)
    private int moveArmMotorToScoreSpecimen = 12; //encoder needed to move arm up for position
    private double backwardDistance = 12; // Distance to drive backward slightly (inches)

    // 2
    private double strafeToAlignToBlocks = 12; // strafe it to align to a block

    // 3
    private double driveForwardToBlock1 = 12; // dirve forward to align with block the first time

    //4
    private double driveToPutBlockInHumanPlayerBox1 = 12; // first itme to do this

    // 5 (Loops)
    private double driveForwardToBlock2 = 12; // drive forward to align with block in loop
    private double driveIncrement = 1; // the second block will be a little farther in the loop so it will need more drive inches to get there

    //6 park
    private double driveParallelToBars = 24; // driving off the human player station to go to bars
    private int moveArmMotorToTouchBars = 100; // in encoder values
    private double driveTowardsBars = 12.2; // driving to the bars

    // Universal
    private double drivePower = 0.5; // Power for driving
    private double armPower = 1.0; // Power for the arm
    private double turnPower = 0.5; // Power for turning
    private int moveArmSmallHumanPlayer = 10;

    @Override
    public void init() {
        auto.resetEncoders();
        auto.init(hardwareMap);
        auto.setZeroPowerModeAsBrake();
        auto.updateAutoTelemetry(telemetry);
    }

    @Override
    public void loop() {

        /*methods usable for later
        1.reset encoders
        2.stop motors
        3.set motor powers
        4.drive (forward or reverse)
        5.strafe (left or right)
        6.turn (left or right)
        7.set target position
        8.move arm slide
        9.run intake
        10.move arm motor
         */
        // Objective: Score a specimen on the bars, then go to the right and collect 3 bricks and bring it back to the human player station, then park on bars

        // 1. move forward to place the specimen and score it and move backward a bit
        auto.drive(forwardToHighRung, "FORWARD");
        auto.moveArmMotor(armPower, moveArmMotorToScoreSpecimen, "FORWARD");
        auto.updateAutoTelemetry(telemetry);

        // 2. strafe right to align with a block and put arm down again
        auto.strafe(strafeToAlignToBlocks, "RIGHT");
        auto.moveArmMotor(armPower, moveArmMotorToScoreSpecimen, "REVERSE");
        auto.updateAutoTelemetry(telemetry);

        // 3. move forward a bit to get to the brick and intake it
        auto.drive(driveForwardToBlock1, "FORWARD");
        auto.runIntake(2000, "IN");
        auto.updateAutoTelemetry(telemetry);

        // 4.bring it a little up turn around and put in human player section the turn around and put arm back down
        auto.moveArmMotor(armPower, moveArmSmallHumanPlayer, "FORWARD");
        auto.turn(turnPower,180,"LEFT");
        auto.updateAutoTelemetry(telemetry);

        auto.drive(driveToPutBlockInHumanPlayerBox1, "FORWARD");
        auto.runIntake(2000, "OUT");
        auto.updateAutoTelemetry(telemetry);

        auto.turn(turnPower,180,"CLOSEST");
        auto.moveArmMotor(armPower, moveArmSmallHumanPlayer, "REVERSE");
        auto.updateAutoTelemetry(telemetry);

        // 5. Loop this
        for (int i = 0; i < 2; i++) {
            if (i == 0) {
                auto.drive(driveToPutBlockInHumanPlayerBox1, "FORWARD");
            } else if (i == 1) {
                auto.drive(driveToPutBlockInHumanPlayerBox1 + driveIncrement, "FORWARD");
            }
            auto.updateAutoTelemetry(telemetry);

            auto.runIntake(2000, "IN");
            auto.moveArmMotor(armPower, moveArmSmallHumanPlayer, "FORWARD");
            auto.turn(turnPower,180,"LEFT");
            auto.updateAutoTelemetry(telemetry);

            if (i == 0) {
                auto.drive(driveToPutBlockInHumanPlayerBox1, "FORWARD");
            } else if (i == 1) {
                auto.drive(driveToPutBlockInHumanPlayerBox1 + driveIncrement, "FORWARD");
            }
            auto.updateAutoTelemetry(telemetry);

            auto.runIntake(2000, "OUT");
            auto.turn(turnPower,180,"CLOSEST");
            auto.moveArmMotor(armPower, moveArmSmallHumanPlayer, "REVERSE");
            auto.updateAutoTelemetry(telemetry);
        }

        //6. Go to the bars to touch arms with it and to park
        auto.drive(driveParallelToBars, "FORWARD");
        auto.turn(turnPower, 90, "LEFT");
        auto.updateAutoTelemetry(telemetry);

        auto.moveArmMotor(armPower,moveArmMotorToTouchBars, "FORWARD");
        auto.drive(driveTowardsBars, "FORWARD");
        auto.updateAutoTelemetry(telemetry);

    }
}
