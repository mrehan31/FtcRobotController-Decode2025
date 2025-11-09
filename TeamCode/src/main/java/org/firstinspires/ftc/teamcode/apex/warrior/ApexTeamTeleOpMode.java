package org.firstinspires.ftc.teamcode.apex.warrior;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// Apex Team main TeleOp Mode
@TeleOp(name = "Apex Warrior Robot Manual Drive - 2025", group = "Linear Opmode")
public class ApexTeamTeleOpMode extends LinearOpMode {

    /* Declare OpMode members */
    RobotHardwareConfigurator myRobotHW = new RobotHardwareConfigurator();
    private static final double shooterDCMotorPowerScaleFactor = 0.38;
    private static final double transferDCMotorPowerScale = 0.9;
    private static final double intakeDCMotorPowerScale = 0.9;

    // Servo positions (0.0 to 1.0) Rotator Assembly
    private static final double SHOOTER_ROTATOR_SERVO_START_POS = 0.36;  // starting position
    private static final double SHOOTER_ROTATOR_SERVO_END_POS = 0.43;  // 150° away from start
    private static final double STEP = 0.01;  // // Step rate controls how fast the servo moves (smaller = slower)
    private final double FEEDER_SERVO_POS_0_DEG = 0.0;   // represents 0 degrees
    private final double FEEDER_SERVO_POS_120_DEG = 0.07; // approx 120 degrees on a 5-turn servo
    private final double STOPPER_SERVO_POS_INITIAL_GATE = 0.60;
    private final double STOPPER_SERVO_POS_OPEN_GATE = 0.70;
    private boolean stopperButtonStateForFirstTime = false;

    private DcMotor frontLeftChassisDC;
    private DcMotor frontRightChassisDC;
    private DcMotor backLeftChassisDC;
    private DcMotor backRightChassisDC;
    private DcMotor leftShooterDC;
    private DcMotor rightShooterDC;
    private DcMotor transferMechDC;
    private DcMotor intakeMechDC;
    private Servo shooterMechRotatorServo;
    private Servo feederEnablerServo;
    private Servo specStopperServo;

    private boolean lastLeftTriggerState = false;
    private boolean lastRightTriggerState = false;
    private double rotorServoCurrentPosition;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "HW Configuration Mapping Initialized");
        myRobotHW.configureRobotHardware(hardwareMap);
        telemetry.update();

        telemetry.addData("Status", "DC Motors & Servo Motors - Configuration Initialized");
        //Initialize DC Motors & Servo Motors
        initMotorAndServo();
        telemetry.update();

        telemetry.addData(">>>", "Robot Ready.  Press Play.");
        telemetry.addData("specStopperServo Position", specStopperServo.getPosition());
        telemetry.addData("specStopperServo direction", specStopperServo.getDirection());
        telemetry.update();

        waitForStart();
        telemetry.log().clear();

        // Variable to track button press state for feeder stopper Servo
        boolean lastButtonState = false;

        while (opModeIsActive()) {

            specStopperServo.setPosition(STOPPER_SERVO_POS_INITIAL_GATE);
            // Current trigger states (threshold of 0.5)
            boolean leftTriggerPressed  = gamepad2.left_trigger  > 0.5;
            boolean rightTriggerPressed = gamepad2.right_trigger > 0.5;

            // ---------- Left Trigger Edge Detect ----------
            if (leftTriggerPressed && !lastLeftTriggerState) {
                onLeftTriggerPressed();
            }

            // ---------- Right Trigger Edge Detect ----------
            if (rightTriggerPressed && !lastRightTriggerState) {
                onRightTriggerPressed();
            }

            // Update previous states
            lastLeftTriggerState = leftTriggerPressed;
            lastRightTriggerState = rightTriggerPressed;

            mecanumDriveGamepadOneJoyStickControlled();

            if (gamepad1.dpad_left) {
                //Todo();
            }
            if (gamepad1.dpad_right) {
                //Todo();
            }
            // D-Pad Up → Move to end (150°)
            if (gamepad1.dpad_up) {
                rotorServoCurrentPosition = SHOOTER_ROTATOR_SERVO_END_POS;
                telemetry.addData("D-Pad Up clicked!", rotorServoCurrentPosition);
                telemetry.update();
                shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
            }
            // D-Pad Down → Move back to start
            if (gamepad1.dpad_down) {
                rotorServoCurrentPosition = SHOOTER_ROTATOR_SERVO_START_POS;
                telemetry.addData("D-Pad Down clicked!", rotorServoCurrentPosition);
                telemetry.update();
                shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
            }
            if (gamepad1.left_bumper) {
                //Todo();
            }
            if (gamepad1.right_bumper) {
                //Todo();
            }


            /* GamePade 2 -> Intake DC Motor movement - START */
            if (gamepad2.x) {
                runIntakeMechDCMotor();
            }
            /* GamePade 2 -> Intake DC Motor movement - STOP */
            if (gamepad2.b) {
                stopIntakeMechDCMotor();
            }
            /* GamePade 2 -> Transfer DC Motor movement - START */
            if (gamepad2.y) {
                runTransferMechDCMotor();
            }
            /* GamePade 2 -> Transfer DC Motor movement - STOP */
            if (gamepad2.a) {
                stopTransferMechDCMotor();
            }
            /* GamePade 2 -> Shooter DC Motor movement - START */
            if (gamepad2.dpad_up) {
                runShooterDCMotors();
            }
            /* GamePade 2 -> Shooter DC Motor movement - STOP */
            if (gamepad2.dpad_down) {
                stopShooterDCMotors();
                //transferMechDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (gamepad2.left_bumper) {
                //TODO
            }
            if (gamepad2.right_bumper) {
                operateFeederServo();
            }
            /* GamePade 2 -> operator Rotator Servo movement based on Gamepad2 both triggers LT and RT */
            //operateRotatorServo();
            telemetry.update();
            idle();
        }
    }

    // 🔵 Called once when LEFT TRIGGER is pressed
    private void onLeftTriggerPressed() {
        telemetry.addLine("LEFT trigger clicked!");
        telemetry.update();
        rotorServoCurrentPosition = rotorServoCurrentPosition - STEP;
        // Limit within defined range
        rotorServoCurrentPosition = Math.max(SHOOTER_ROTATOR_SERVO_START_POS, rotorServoCurrentPosition);// Set new position
        // Set new position
        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
        telemetry.addData("LEFT trigger clicked: rotorServoCurrentPosition", rotorServoCurrentPosition);
        telemetry.update();
    }

    // 🔴 Called once when RIGHT TRIGGER is pressed
    private void onRightTriggerPressed() {
        telemetry.addLine("RIGHT trigger clicked!");
        telemetry.update();
        rotorServoCurrentPosition = rotorServoCurrentPosition + STEP;
        // Limit within defined range
        rotorServoCurrentPosition = Math.min(SHOOTER_ROTATOR_SERVO_END_POS, rotorServoCurrentPosition);// Set new position
        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
        telemetry.addData("RIGHT trigger clicked: rotorServoCurrentPosition", rotorServoCurrentPosition);
        telemetry.update();
    }

    private void operateFeederServo() {
        // Move up to 120°
        specStopperServo.setDirection(Servo.Direction.FORWARD);
        if(!stopperButtonStateForFirstTime) {
            stopperButtonStateForFirstTime = true;
        } else{
            specStopperServo.setPosition(STOPPER_SERVO_POS_OPEN_GATE);
        }
        feederEnablerServo.setPosition(FEEDER_SERVO_POS_120_DEG);
        telemetry.addData("stopperButtonStateForFirstTime flag:", stopperButtonStateForFirstTime);
        telemetry.addData("feederEnablerServo Position", feederEnablerServo.getPosition());
        telemetry.addData("specStopperServo Position", specStopperServo.getPosition());
        telemetry.addData("specStopperServo direction", specStopperServo.getDirection());
        telemetry.update();
        sleep(400); // adjust as needed for speed
        specStopperServo.setDirection(Servo.Direction.REVERSE);
        specStopperServo.setPosition(STOPPER_SERVO_POS_OPEN_GATE);
        feederEnablerServo.setPosition(FEEDER_SERVO_POS_0_DEG);
        telemetry.addData("feederEnablerServo Position", feederEnablerServo.getPosition());
        telemetry.addData("specStopperServo Position", specStopperServo.getPosition());
        telemetry.addData("specStopperServo direction", specStopperServo.getDirection());
        telemetry.update();
        sleep(400); // adjust as needed for speed
    }

    private void initMotorAndServo() {
        initiateChassisDCMotors();
        initiateOtherDCMotors();
        initiateServoMotors();
    }

    private void initiateChassisDCMotors() {
        telemetry.addData("DcMotor initiateChassisDCMotors:", "Configuration Started");

        backLeftChassisDC = myRobotHW.getBackLeftChassisDC();
        backRightChassisDC = myRobotHW.getBackRightChassisDC();
        frontLeftChassisDC = myRobotHW.getFrontLeftChassisDC();
        frontRightChassisDC = myRobotHW.getFrontRightChassisDC();

        backLeftChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBL", backLeftChassisDC.getDirection());
        backLeftChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeBL:", backLeftChassisDC.getCurrentPosition());
        backLeftChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRightChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBR", backRightChassisDC.getDirection());
        backRightChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeBR:", backRightChassisDC.getCurrentPosition());
        backRightChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFL", frontLeftChassisDC.getDirection());
        frontLeftChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeFL:", frontLeftChassisDC.getCurrentPosition());
        frontLeftChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFR", frontRightChassisDC.getDirection());
        frontRightChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeFR:", frontRightChassisDC.getCurrentPosition());
        frontRightChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("DcMotor initiateChassisDCMotors:", "Configuration Completed");
        telemetry.update();
    }

    private void initiateOtherDCMotors() {
        telemetry.addData("DcMotor initiateOtherDCMotors:", "Configuration Started");

        leftShooterDC = myRobotHW.getLeftShooterDC();
        rightShooterDC = myRobotHW.getRightShooterDC();
        transferMechDC = myRobotHW.getTransferMechDC();
        intakeMechDC = myRobotHW.getIntakeMechDC();

        leftShooterDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionLeftShooter", leftShooterDC.getDirection());
        leftShooterDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("DcMotor ModeLeftShooter:", leftShooterDC.getCurrentPosition());


        rightShooterDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionRightShooter", rightShooterDC.getDirection());
        rightShooterDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("DcMotor ModeRightShooter:", rightShooterDC.getCurrentPosition());

        transferMechDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionTransferMech", transferMechDC.getDirection());
        transferMechDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("DcMotor ModeTransferMech:", transferMechDC.getCurrentPosition());

        intakeMechDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionIntakeMech", intakeMechDC.getDirection());
        intakeMechDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("DcMotor ModeIntakeMech:", intakeMechDC.getCurrentPosition());

        telemetry.addData("DcMotor initiateOtherDCMotors:", "Configuration Completed");
        telemetry.update();
    }

    private void initiateServoMotors() {
        telemetry.addData("Servo initiateServoMotors:", "Configuration Started");
        shooterMechRotatorServo = myRobotHW.getShooterMechRotatorServo();
        feederEnablerServo = myRobotHW.getFeederEnablerServo();
        specStopperServo = myRobotHW.getSpecStopperServo();
        // Initialize shooterMechRotatorServo & feederEnablerServo to the starting position
        specStopperServo.setDirection(Servo.Direction.FORWARD);
        shooterMechRotatorServo.setPosition(SHOOTER_ROTATOR_SERVO_START_POS);
        feederEnablerServo.setPosition(FEEDER_SERVO_POS_0_DEG);
        specStopperServo.setPosition(STOPPER_SERVO_POS_INITIAL_GATE);
        telemetry.addData("Servo initiateServoMotors:", "Configuration Completed");
        telemetry.addLine("Use triggers to move shooterMechRotatorServo");
        telemetry.addLine("Right Trigger → upward movement");
        telemetry.addLine("Left Trigger → downward movement");
        telemetry.addData("shooterMechRotatorServo init position", specStopperServo.getPosition());
        telemetry.addData("shooterMechRotatorServo init direction", specStopperServo.getDirection());
        telemetry.update();
    }

    private void mecanumDriveGamepadOneJoyStickControlled() {

        frontRightChassisDC.setDirection(REVERSE);
        backRightChassisDC.setDirection(REVERSE);
        double h = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double v1 = h * Math.sin(robotAngle) - rightX;
        final double v2 = h * Math.cos(robotAngle) + rightX;
        final double v3 = h * Math.cos(robotAngle) - rightX;
        final double v4 = h * Math.sin(robotAngle) + rightX;

        frontLeftChassisDC.setPower(v1);
        frontRightChassisDC.setPower(v2);
        backLeftChassisDC.setPower(v3);
        backRightChassisDC.setPower(v4);
    }

    // Function to run Shooter DC Motors at scaled power
    private void runShooterDCMotors() {
        // Reverse one motor so they spin opposite directions
        leftShooterDC.setDirection(DcMotor.Direction.FORWARD);
        rightShooterDC.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionLeftShooter", leftShooterDC.getDirection());
        telemetry.addData("DcMotor DirectionRightShooter", rightShooterDC.getDirection());
        telemetry.update();
        startShooterDCMotors();
    }

    // Function to give scaled power to both shooter DC motors(opposite rotation)
    private void startShooterDCMotors() {
        // Ensure shooterDCPowerScale stays between 0 and 1
        leftShooterDC.setPower(shooterDCMotorPowerScaleFactor);
        rightShooterDC.setPower(shooterDCMotorPowerScaleFactor);
        telemetry.addData("DcMotor shooterDCPowerScale", shooterDCMotorPowerScaleFactor);
        telemetry.addData("DcMotor LeftShooter Power", leftShooterDC.getPower());
        telemetry.addData("DcMotor RightShooter Power", rightShooterDC.getPower());
        telemetry.update();
    }

    // Function to run Transfer DC Motor at scaled power
    private void runTransferMechDCMotor() {
        transferMechDC.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionTransfer", transferMechDC.getDirection());
        telemetry.update();
        startTransferMechDCMotor(transferDCMotorPowerScale);
    }

    // Function to give scaled power to Transfer DC Motor
    private void startTransferMechDCMotor(double transferDCPowerScale) {
        // Ensure transferDCPowerScale stays between 0 and 1
        transferDCPowerScale = Math.max(0, Math.min(transferDCPowerScale, 1));
        transferMechDC.setPower(transferDCPowerScale);
        telemetry.addData("DcMotor transferDCPowerScale", transferDCPowerScale);
        telemetry.addData("DcMotor Transfer Power", transferMechDC.getPower());
        telemetry.update();
    }

    // Function to run Intake DC Motor at scaled power
    private void runIntakeMechDCMotor() {
        intakeMechDC.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionIntake", intakeMechDC.getDirection());
        telemetry.update();
        startIntakeMechDCMotor(intakeDCMotorPowerScale);
    }

    // Function to give scaled power to Intake DC Motor
    private void startIntakeMechDCMotor(double intakeDCPowerScale) {
        // Ensure intakeDCPowerScale stays between 0 and 1
        intakeDCPowerScale = Math.max(0, Math.min(intakeDCPowerScale, 1));
        intakeMechDC.setPower(intakeDCPowerScale);
        telemetry.addData("DcMotor intakeDCPowerScale", intakeDCPowerScale);
        telemetry.addData("DcMotor Intake Power", intakeMechDC.getPower());
        telemetry.update();
    }

    private void stopAllDCMotors() {
        telemetry.addData("stopAllDCMotors:  ", "started");
        stopShooterDCMotors();
        stopTransferMechDCMotor();
        stopIntakeMechDCMotor();
        telemetry.addData("stopAllDCMotors:  ", "ended");
    }

    // Function to stop both shooter DC motors (idle)
    private void stopShooterDCMotors() {
        leftShooterDC.setPower(0);
        rightShooterDC.setPower(0);
    }

    private void stopTransferMechDCMotor() {
        transferMechDC.setPower(0);
    }

    private void stopIntakeMechDCMotor() {
        intakeMechDC.setPower(0);
    }
}