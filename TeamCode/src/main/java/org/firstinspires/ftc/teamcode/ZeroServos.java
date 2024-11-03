
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "wristServoZero")
public class ZeroServos extends OpMode {

    //Servo claw;
    Servo Servo1;
    Servo Servo2;
    Servo Servo3;
    Servo Servo4;

    @Override
    public void init() {

        Servo1 = hardwareMap.get(Servo.class, "leftCV4BServo");
        Servo2 = hardwareMap.get(Servo.class, "rightCV4BServo");
        Servo3 = hardwareMap.get(Servo.class, "rotateIntakeServo");
        Servo4 = hardwareMap.get(Servo.class, "specimenServo");

        Servo1.setPosition(0);
        Servo2.setPosition(0);
        Servo3.setPosition(0);
        Servo4.setPosition(0);
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            Servo1.setPosition(0);
            Servo2.setPosition(0);
            Servo3.setPosition(0);
            Servo4.setPosition(0);
        }
    }
}

