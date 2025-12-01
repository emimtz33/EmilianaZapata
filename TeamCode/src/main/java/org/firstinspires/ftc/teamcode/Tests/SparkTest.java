package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "SparkTest by Mii")
public class SparkTest extends LinearOpMode {

    // Declare the motor variable
    public DcMotor intake;

    @Override
    public void runOpMode() {

        // Map the motor variable to the configuration name you set in the app
        intake = hardwareMap.get(DcMotor.class, "in1"); // Use your actual configuration name

        // Optional: set direction if needed
        intake.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Power storing variable
            double intakePower;


            //Intake functions
            if (gamepad1.left_trigger > 0.1) {
                intakePower = 0.8;
            } else {
                intakePower = 0.0;
            }

            if (gamepad1.left_bumper) {
                intakePower = -0.8;
            }


            //Set Power to intake
            intake.setPower(intakePower);


            // Add telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Motor Power", intake.getPower());
            telemetry.update();
        }
    }
}
