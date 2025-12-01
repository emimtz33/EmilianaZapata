package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "IMU test by Emiliano")

public class IMUTest extends LinearOpMode{
    //Variables globales
    public IMU imu;

    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Creación de variables locales
        imu = hardwareMap.get(IMU.class, "imu");

        //Inicializar IMU
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP))
        );

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Heading: ", obtenerAngulo());
            telemetry.update();
        }
    }

    // Creación de funciones adicionales //

    public double obtenerAngulo(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

}
