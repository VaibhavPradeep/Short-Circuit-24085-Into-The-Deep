package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "ScrimBotAutoBasketSide")
public class ScrimBotAutoBasketSide extends OpMode {

    ScrimBotAutoMethods auto = new ScrimBotAutoMethods();

    // 1
    private double forwardToHighRung = 24; // Distance to drive forward to the high rung (inches)

    // 2
    private int moveArmMotorToScoreSpecimen = 12; //encoder needed to move arm up for position
    private double backwardDistance = 12; // Distance to drive backward slightly (inches)

    // 3
    private double turnLeft90 = 12; // Adjust as needed for turning

    // 4
    private double driveToYellowBlock = 36; // Distance to drive forward to the yellow blocks (inches)

    // 5
    private double strafeRightToPickup = 12; // Distance to strafe left to position for pickup (inches)

    // 7
    private double pickupTurn180 = 180; // Turn 180 degrees

    // 8
    private int moveArmMotorToScoreBlock = 12; // amount to move arm up to score block (encoder)
    private int armExtendDistance = 12; // Distance to extend the arm to score (inches)

    //9
    private double backwardDistanceFromBasket = 12; // Distance to drive backward slightly from basket

    //10
    private double turnToBeFacingBars = 100; // degrees
    private double strafeLeftAlignToBars = 12; // inches
    private double driveForwardToBars = 24; // inches
    private int moveArmMotorToTouchBars = 100; // in encoder values

    // Universal
    private double drivePower = 0.5; // Power for driving
    private double armPower = 1.0; // Power for the arm
    private double turnPower = 0.5; // Power for turning

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
        // Objective: going to start near the left center near the basket to release specimen

        // 1. Drive forward to the high rung and release the preloaded sample
        auto.drive(forwardToHighRung, "FORWARD");
        auto.moveArmMotor(armPower, moveArmMotorToScoreSpecimen, "FORWARD");
        auto.updateAutoTelemetry(telemetry);

        // Repeat steps 2-8 for the remaining two yellow blocks

        // 2. Drive backwards slightly
        auto.drive(backwardDistance, "REVERSE");
        auto.moveArmMotor(armPower, moveArmMotorToScoreSpecimen, "REVERSE");
        auto.updateAutoTelemetry(telemetry);

        // 3. Turn 90 degrees left
        auto.turn(turnPower, turnLeft90, "LEFT");
        auto.updateAutoTelemetry(telemetry);

        // 4. Drive forward to the yellow blocks
        auto.drive(driveToYellowBlock, "FORWARD");
        auto.updateAutoTelemetry(telemetry);

        // 5. Strafe right to position for pickup
        auto.strafe(strafeRightToPickup, "RIGHT");
        auto.updateAutoTelemetry(telemetry);

        // 6. Pick up the yellow block
        auto.runIntake(2000, "IN");
        auto.updateAutoTelemetry(telemetry);

        // 7. Turn 180 degrees
        auto.turn(turnPower, pickupTurn180, "LEFT");
        auto.updateAutoTelemetry(telemetry);

        // 8. Extend the arm and score the block on the high rung
        auto.moveArmMotor(armPower, moveArmMotorToScoreBlock, "FORWARD");
        auto.extendArmSlide(armPower,armExtendDistance, "FORWARD");
        auto.runIntake(2000,"OUT");
        auto.updateAutoTelemetry(telemetry);

        // 9. Come a bit backwards and de-extend and put arm back down
        auto.drive(backwardDistanceFromBasket, "REVERSE");
        auto.moveArmMotor(armPower, moveArmMotorToScoreBlock, "REVERSE");
        auto.extendArmSlide(armPower,armExtendDistance, "REVERSE");

        //10. turn left to be parallel to bars that we are going to touch with arm and park at bars
        auto.turn(turnPower, turnToBeFacingBars, "CLOSEST");
        auto.strafe(strafeLeftAlignToBars, "LEFT");
        auto.moveArmMotor(armPower,moveArmMotorToTouchBars, "FORWARD");
        auto.drive(driveForwardToBars, "FORWARD");

        auto.stopMotors();
        auto.updateAutoTelemetry(telemetry);
    }
}