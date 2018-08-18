package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Prestolie on 11/3/2017.
 */

@Autonomous(name="autonomousTopRight", group= "TeamBot")
public class AutonomousTeleOpTR extends LinearOpMode {

    public CompetitionHardware2 robot = new CompetitionHardware2();
    private ElapsedTime runtime = new ElapsedTime();
    VuforiaLocalizer vuforia;

    // Preference Constants
    double STANDARD_SPEED = 0.5;
    double STANDARD_ELEVATOR_SPEED = 1;
    double STANDARD_CLAW_SPEED = 0.25;
    double INC_SERVO = 0.1;
    double X_HOME = 0.5;
    double Y_HOME = 0.7;
    double Y_DOWN = 0.1;
    double X_LEFT = 0.2;
    double X_RIGHT = 0.8;
    String TEAM = "blue";

    // Calculated constants
    int DRIVE_GEAR_REDUCTION = 20*2;
    int PULSES_PER_REV = 24;
    int COUNTS_PER_REV = PULSES_PER_REV*DRIVE_GEAR_REDUCTION;
    double WHEEL_DIAMETER = 5; // inches
    double INCHES_PER_REV = WHEEL_DIAMETER*Math.PI;
    int COUNTS_PER_INCH = (int)(COUNTS_PER_REV/INCHES_PER_REV);
    int distance;//

    // Measured Constants
    int DIST_TO_RIGHT = (int)(30*COUNTS_PER_INCH); // inches to centimeters
    int DIST_TO_MIDDLE = (int)(36*COUNTS_PER_INCH); // inches to centimeters
    int DIST_TO_LEFT = (int)(42*COUNTS_PER_INCH); // inches to centimeters

    DriveThread dt = new DriveThread(robot,0,STANDARD_SPEED,false);
    ElevatorThread et = new ElevatorThread(robot, STANDARD_ELEVATOR_SPEED, true);
    ClawThread ct = new ClawThread(robot,STANDARD_CLAW_SPEED);

    @Override
    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);

        // set up Vuforia
        // TODO: IMPORTANT---The key will never be seen by the time goPark begins. To fix this, adjust logic
        ScanThread st = new ScanThread(robot,vuforia);
        //int camerMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        setupRobot();

        waitForStart();
        // After pressing start, robot needs to start doing stuff. I've outlined where each part of
        // algorithm should go with telemetry statements
        long milliseconds = 3000;
        st.start();

        ct = new ClawThread(robot,-STANDARD_CLAW_SPEED);
        ct.start();
        telemetry.addData("Algorithm Status:", "Closing Claw...");
        telemetry.update();

        et = new ElevatorThread(robot,-STANDARD_ELEVATOR_SPEED,false);
        et.start();
        telemetry.addData("Algorithm Status:", "Raising Elevator...");
        telemetry.update();

        knockGem(1);
        telemetry.addData("Algorithm Status:","Knocking over gem...");
        telemetry.update();
        sleep(milliseconds);

        // Need to bring cryptokey into view, this will affect the goPark function

        //scanKey();
        //telemetry.addData("Algorithm Status:","Reading cryptokey...");
        //telemetry.update();

        // Maybe wait until key is scanned (or timed out)

        goPark(robot.vuMarkID);
        telemetry.addData("Algorithm Status:","Grabbing block and moving to cryptobox...");
        telemetry.update();

        while (dt.isAlive()) {
            telemetry.addData("Algorithm Status:", "Waiting for robot to park...");
        }

        sleep(milliseconds);
       //PlaceCube(); does nothing ATM; just a placeholder.
        telemetry.addData("Algorithm Status:","Placing block..");
        telemetry.update();

        shutdownRobot();
    }

    public void knockGem(double GemState){
        boolean xMovement = false;

        if (GemState == 1){
            //sets down the gem stick
            robot.gemY.setPosition(Y_DOWN);
            GemState = 2;
        }
        if (GemState == 2) {
            runtime.reset();
            while (runtime.seconds() < 2){
                // wait for stick to go down
            }
            runtime.reset();
            //checks for blue ball
            if (TEAM.equals("red")){

            }
            while(opModeIsActive() && runtime.seconds() < 2 && GemState == 2){
                if (TEAM.equals("blue")) {
                    if (robot.Gem.blue() >= 2) {
                        robot.gemX.setPosition(X_LEFT);
                        xMovement = true;
                        GemState = 3;
                    }
                }
                else{
                    if (TEAM.equals("red")){
                        if (robot.Gem.red() >= 2){
                            robot.gemX.setPosition(X_LEFT);
                            xMovement = true;
                            GemState = 3;
                        }
                    }
                }
            }
            //gemX will do nothing
            GemState = 3;
        }
        if (GemState == 3) {
            sleep(750);
            if (xMovement == true) {
                //sets the x-axis back to the home position
                robot.gemX.setPosition(X_HOME);
                xMovement = false;
                //then takes y back to home position
                robot.gemY.setPosition(Y_HOME);
                GemState = 0;
            } else {
                // move the robot for x amount of inches
                robot.gemY.setPosition(Y_HOME);
                GemState = 4;
            }
        }
        //lifts the gem stick back to the home position on y axis
        if (GemState == 4){
            sleep(750);
            robot.gemY.setPosition(Y_HOME);
        }

        while (GemState > 0){
            telemetry.addData("Algorithm Status:","Knocking over gem.");
            telemetry.update();
            idle();
        }
    }

    //the following function is not complete, only gets to the point where it faces cubes
    // TODO: Change this
    public void goPark(int vuMarkID) {
        // first, the robot will drive towards the parking zone before it makes a turn toward the pile of cubes
        int distance;
        boolean adjust = false;
        if (vuMarkID == 1) {
            //drive the robot the furthest distance, reaching the left side of the cryptobox
            telemetry.addData("VuMark:", "Left Detected");
            distance = DIST_TO_LEFT;
        } else if (vuMarkID == 2) {
            //drive the robot towards the middle of the cryptobox
            telemetry.addData("VuMark:", "Middle Detected");
            distance = DIST_TO_MIDDLE;
        } else if (vuMarkID == 3) {
            //drive the robot the shortest distance before turning
            telemetry.addData("VuMark:", "Right Detected");
            distance = DIST_TO_RIGHT;
        } else {
            //this would be the situation where the cryptokey wasn't detected, assume middle
            telemetry.addData("Vumark:", "Not Detected");
            distance = DIST_TO_MIDDLE;
        }
        dt = new DriveThread(robot,distance,-0.5,false);
        dt.start();
        while (dt.isAlive()) {
            // Do nothing, robot is driving
            if (robot.vuMarkID != 0 && robot.vuMarkID != 2) {
                adjust = true;
            }
        }
        if (adjust) {
            // Adjust away from middle (6 inches)
            if(robot.vuMarkID == 1){
                dt = new DriveThread(robot,6*COUNTS_PER_INCH,-0.5,false);
                dt.start();
                while (dt.isAlive()) {
                    // Do nothing
                }
            } else {
                dt = new DriveThread(robot,6*COUNTS_PER_INCH,-0.5,false);
                dt.start();
                while (dt.isAlive()) {
                    // Do nothing
                }
            }
        }
        dt = new DriveThread(robot,0,-0.5,true);
        dt.start(); // Don't have to wait. Main function does that for us.
    }

    // TODO: Actually implement this!!
    public void placeCube(){

    }

    // Below this point are "helper functions" meaning that they don't do anything special for
    // controlling the robot. They just do some functions that we use a lot.

    private void setModeToDrivetrain(DcMotor.RunMode mode) {
        robot.FLmotor.setMode(mode);
        robot.FRmotor.setMode(mode);
        robot.RLmotor.setMode(mode);
        robot.RRmotor.setMode(mode);
    }

    private void setupRobot() {
        ct = new ClawThread(robot,STANDARD_CLAW_SPEED);
        ct.start();

        robot.Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setModeToDrivetrain(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.gemX.setPosition(X_HOME);
        robot.gemY.setPosition(Y_HOME);
    }

    private void shutdownRobot() {
        // Open the claw
        ct = new ClawThread(robot,STANDARD_CLAW_SPEED);
        ct.start();
    }

    private static class ClawThread extends Thread {
        // Always True
        int OPEN_CLEFT = 0;
        int OPEN_CRIGHT = 0;
        int CLOSE_CLEFT = -268;
        int CLOSE_CRIGHT = 268;

        // Created when needed
        CompetitionHardware2 robot;
        ElapsedTime runtime;
        double speed;

        public ClawThread(CompetitionHardware2 robot, double speed) {
            this.robot = robot;
            this.speed = speed;
        }
        public void run() {
            try {
                openClose(this.speed);
            } catch (Exception e) {
                // Something bad happened
                //System.out.println(e.toString());
            }
        }
        private void openClose(double speed) {
            double leftSpeed = speed;
            double rightSpeed = -speed;
            if (speed < 0){
                //does incremental close
                robot.Cleft.setTargetPosition(CLOSE_CLEFT);
                robot.Cright.setTargetPosition(CLOSE_CRIGHT);
            }
            else {
                //does incremental open
                robot.Cleft.setTargetPosition(OPEN_CLEFT);
                robot.Cright.setTargetPosition(OPEN_CRIGHT);
            }

            robot.Cleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Cright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.Cleft.setPower(leftSpeed);
            robot.Cright.setPower(rightSpeed);

            runtime = new ElapsedTime();
            while(!isInterrupted() && robot.Cleft.isBusy() && robot.Cright.isBusy()) {
                // busy waiting
                if (robot.tLeft.isPressed() && speed > 0) {
                    break;
                }
                if (robot.tRight.isPressed() && speed > 0) {
                    break;
                }
                if (runtime.seconds() > 1) {
                    break;
                }
            }

            if(robot.tLeft.isPressed()) {
                robot.Cleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (robot.tRight.isPressed()){
                robot.Cright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            robot.Cleft.setPower(0);
            robot.Cright.setPower(0);

            robot.Cleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Cright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private static class DriveThread extends Thread {
        // Hardware Constants
        double WHEEL_DIAMETER = 5; // inches
        double ROBOT_LENGTH = 18;
        double ROBOT_WIDTH = 13; // Wheels are roughly 2.5 inches in
        int PULSES_PER_REV = 24; // After the internal gearbox
        int ENCODER_ERROR = 0;
        int DRIVE_GEAR_REDUCTION = (20*2+ENCODER_ERROR); // Using 20's with a 2:1 on the outside

        // Calculated Constants
        double ROBOT_DIAMETER = Math.sqrt(Math.pow(ROBOT_LENGTH,2) + Math.pow(ROBOT_WIDTH,2));
        double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double ROBOT_CIRCUMFERENCE = Math.PI * ROBOT_DIAMETER;
        double COUNTS_PER_INCH = PULSES_PER_REV * DRIVE_GEAR_REDUCTION/(Math.PI*WHEEL_DIAMETER);
        int COUNTS_PER_REV = PULSES_PER_REV*DRIVE_GEAR_REDUCTION;
        double NUM_REV_TO_90 = ROBOT_CIRCUMFERENCE/(4*WHEEL_CIRCUMFERENCE);
        int COUNTS_TO_90 = (int)Math.ceil(NUM_REV_TO_90*COUNTS_PER_REV);

        // Created when needed
        CompetitionHardware2 robot;
        ElapsedTime runtime;
        String decision;
        int count;
        double speed;
        boolean turn90;

        public DriveThread(CompetitionHardware2 robot, int count, double speed, boolean turn90) {
            this.robot = robot;
            this.runtime = runtime;
            this.count = count;
            this.speed = speed;
            this.turn90 = turn90;
        }
        public void run() {
            try {
                Drive(this.count,this.speed,this.turn90);
            } catch (Exception e) {
                // Something bad happened
                //System.out.println(e.toString());
            }
        }

        private void Drive(int count, double speed, boolean turn90) {
            // Prepare to use encoders and reset them
            setModeToDrivetrain(DcMotor.RunMode.RUN_USING_ENCODER);
            resetAllEncoders();
            int currentFL;
            int currentFR;
            int currentRR;
            int currentRL;
            int posCount = count;
            int negCount = -count;
            double leftSpeed = speed;
            double rightSpeed = speed;
            if (speed < 0) {
                if (!turn90) {
                    // Reverse: Power stays the same; In this direction, the counts are positive
                    currentFL = robot.FLmotor.getCurrentPosition()+posCount;
                    currentFR = robot.FRmotor.getCurrentPosition()+posCount;
                    currentRR = robot.RRmotor.getCurrentPosition()+posCount;
                    currentRL = robot.RLmotor.getCurrentPosition()+posCount;
                } else {
                    // Clockwise: Right is backward (negative) and left is forward (positive); In
                    // this direction, right should be positive counts and left should be negative.
                    rightSpeed = -rightSpeed;
                    posCount = COUNTS_TO_90;
                    negCount = -COUNTS_TO_90;
                    currentFL = robot.FLmotor.getCurrentPosition()+negCount;
                    currentFR = robot.FRmotor.getCurrentPosition()+posCount;
                    currentRR = robot.RRmotor.getCurrentPosition()+posCount;
                    currentRL = robot.RLmotor.getCurrentPosition()+negCount;
                }
            } else {
                if (!turn90) {
                    // Forward: Power stays the same; In this direction, counts are negative.
                    currentFL = robot.FLmotor.getCurrentPosition()+negCount;
                    currentFR = robot.FRmotor.getCurrentPosition()+negCount;
                    currentRR = robot.RRmotor.getCurrentPosition()+negCount;
                    currentRL = robot.RLmotor.getCurrentPosition()+negCount;
                } else {
                    // Counterclockwise: Left goes reverse (negative), and right goes forward (positive).;
                    // In this directio, left counts are positive and right counts are negative.
                    leftSpeed = -leftSpeed;
                    posCount = COUNTS_TO_90;
                    negCount = -COUNTS_TO_90;
                    currentFL = robot.FLmotor.getCurrentPosition()+posCount;
                    currentFR = robot.FRmotor.getCurrentPosition()+negCount;
                    currentRR = robot.RRmotor.getCurrentPosition()+negCount;
                    currentRL = robot.RLmotor.getCurrentPosition()+posCount;
                }
            }

            setModeToDrivetrain(DcMotor.RunMode.RUN_TO_POSITION);

            setTargetPositionOfDrivetrain(currentFL,currentRL,currentFR,currentRR);

            setPowerToDrivetrain(leftSpeed,rightSpeed);

            while (!isInterrupted() && robot.FLmotor.isBusy() && robot.FRmotor.isBusy() &&
                    robot.RLmotor.isBusy() && robot.RRmotor.isBusy()){
                //busy waiting
            }

            setPowerToDrivetrain(0,0);

            resetAllEncoders();

            // Assume you're going back to running manual
            setModeToDrivetrain(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        private void setModeToDrivetrain(DcMotor.RunMode mode) {
            robot.FLmotor.setMode(mode);
            robot.FRmotor.setMode(mode);
            robot.RLmotor.setMode(mode);
            robot.RRmotor.setMode(mode);
        }

        private void setPowerToDrivetrain(double leftPower, double rightPower) {
            robot.FLmotor.setPower(leftPower);
            robot.RLmotor.setPower(leftPower);
            robot.FRmotor.setPower(rightPower);
            robot.RRmotor.setPower(rightPower);
        }

        private void setTargetPositionOfDrivetrain(int FL, int RL, int FR, int RR) {
            robot.FLmotor.setTargetPosition(FL);
            robot.FRmotor.setTargetPosition(FR);
            robot.RLmotor.setTargetPosition(RL);
            robot.RRmotor.setTargetPosition(RR);
        }

        private void resetAllEncoders() {
            robot.FLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.RLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    private static class ElevatorThread extends Thread {

        int ELEVATOR_MAX = 5300;
        int COUNTS_TO_NEXT_MODE = (int) Math.ceil(ELEVATOR_MAX / 2);

        //created when needed
        CompetitionHardware2 robot;
        ElapsedTime runtime;
        double speed;
        boolean home;

        public ElevatorThread(CompetitionHardware2 robot, double speed, boolean home) {
            this.robot = robot;
            this.runtime = runtime;
            this.speed = speed;
            this.home = home;
        }

        public void run() {
            try {
                raiseLower(speed, home);
            } catch (Exception e) {
                //baddie baddie oh no.
            }
        }

        private void raiseLower(double speed, boolean home) {
            int NewPos;
            // I've taken telemetries out due to them not being static? if you ask me ¯\_ツ_/¯
            // (probably makes sense tho, Brandon is sleepy Zzz...)
            if (robot.elevatorMode < 0) {
                // Moving from manual to mode select (simple---Go Home first)
                robot.elevatorMode = 0;
                NewPos = 0;
            } else if (speed < 0 && robot.elevatorMode < 2 && !home) {
                // set the elevator to move upward
                int cur = robot.Elevator.getCurrentPosition();
                NewPos = cur - COUNTS_TO_NEXT_MODE;
                robot.elevatorMode++;
            } else if (speed > 0 && robot.elevatorMode > 0 && !home) {
                //sets the elevator to move downward
                int cur = robot.Elevator.getCurrentPosition();
                NewPos = cur + COUNTS_TO_NEXT_MODE;
                robot.elevatorMode--;
            } else if (home) {
                NewPos = 0;
                robot.elevatorMode = 0;
            } else {
                int cur = robot.Elevator.getCurrentPosition();
                NewPos = cur;
            }

            robot.Elevator.setTargetPosition(NewPos);

            robot.Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.Elevator.setPower(speed);

            while(!isInterrupted() && robot.Elevator.isBusy()){
                if (robot.tBot.isPressed() && speed > 0) {
                    break;
                }
                if (robot.Elevator.getCurrentPosition() < -ELEVATOR_MAX && speed < 0) {
                    break;
                }
            }

            if (robot.tBot.isPressed()) {
                robot.Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            robot.Elevator.setPower(0);
            robot.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private class ScanThread extends Thread{
        CompetitionHardware2 robot;
        VuforiaLocalizer vuforia;

        public ScanThread(CompetitionHardware2 robot, VuforiaLocalizer vuforia){
            this.robot = robot;
            this.vuforia = vuforia;
        }
        public void run(){
            try{
                while (!isInterrupted() && robot.vuMarkID == 0 ) {
                    scanKey();
                }
            }
            catch (Exception e ){
                // Something bad happened, oh well
            }
        }
        public void scanKey(){
            int camerMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(camerMonitorViewId);
            parameters.vuforiaLicenseKey = "AcV+Jsr/////AAAAGXHHw0GRIUwCpyZYG3N75VorKFs2D5Mw0kS8wVa0vpU47CzS6YPGhc3P1C4CUr4N4Qhio5ug/8GrmhBgAxK6AtikaT32qg3RXmC8D5jA7qn3gdQblkCxnLhH85YGcOA81URmFaK9ubqN8noR/08eR43sik1p+wdsyJgqOUHgEEs9CsYxJd3cm7IPj/wlUlEpa21R+KG/KFtnz9nNV6u5Ckgl5EztpBxur1dOabC1yxK5A7tlxQfbQJZSO0uz+hEOmvK5h3dUZwb7qPHb07MStcmTW+TkOkZSTJBN4VYEIFgY8yxqLnpEwz6uEZqoJrf1WmD49RR33UoTq+Pl58IJxJ1RvwVSz+9dvt1rXt3MHnM2";//380 character license key, how about not having it go across the screen?
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            runtime.reset();
            relicTrackables.activate();
            while (opModeIsActive() && (dt.isAlive() && runtime.seconds() < 1.5 && robot.vuMarkID == 0)) {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    robot.vuMarkID = 1;
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    robot.vuMarkID = 2;
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    robot.vuMarkID = 3;
                } else {
                    robot.vuMarkID = 0;
                }
                if (robot.vuMarkID != 0){
                    return;
                }
            }
            return;
        }

    }

}

