package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot.MID_SERVO;

public class BucketBotHardware {
    // Attributes
    public DcMotor[] motors;

    public Servo lidLifter;

    public HardwareMap hwMap;

    public BucketBotHardware() {
        // Constructor remains empty
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motors = new DcMotor[4];
        for(int i = 0; i < 4; i++) {
            motors[i] = hwMap.get(DcMotor.class,"motor"+Integer.toString(i));
            if (i % 2 == 0){// if this == 0 the number is even
                motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if (i % 2 == 1){// otherwise (1) it is odd
                motors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            //set power to all drive motors to 0
            motors[i].setPower(0);

            //then, change their mode to run without encoders
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Define and initialize ALL installed servos.
        lidLifter  = hwMap.get(Servo.class, "lid lifter");
    }

}
