package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
//hello

@TeleOp(name = "MecanumDriveTrain by Emiliano")
public class MecanumDriveTest extends LinearOpMode{
    @Override
    public void runOpMode(){

        telemetry.addData("Estado", "Inicializado correctamente");
        telemetry.update();

        // Inicializar motores
        DcMotor frenteIzquierdo = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor frenteDerecho = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor atrasIzquierdo = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor atrasDerecho = hardwareMap.get(DcMotor.class, "rightBack");

        // Configurar direcciones
        frenteDerecho.setDirection(DcMotorSimple.Direction.FORWARD);
        atrasDerecho.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configurar encoders
        frenteIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frenteDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        atrasIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        atrasDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()){

            //configurar valores de joystick
            double drive = -gamepad1.left_stick_y; //Eje Y invertido
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_trigger - gamepad1.left_trigger;

            boolean modoLento = gamepad1.a; //Definir modo lento
            double maxPower = modoLento ? 0.2 : 0.8;    //Limitar motores

            //Declarar variables de poder para los motores
            double fiPower = Range.clip((drive + turn + strafe), -maxPower, maxPower);
            double fdPower = Range.clip((drive - turn - strafe), -maxPower, maxPower);
            double aiPower = Range.clip((drive + turn - strafe), -maxPower, maxPower);
            double adPower = Range.clip((drive - turn + strafe), -maxPower, maxPower);

            //Dar poder a los motores
            frenteIzquierdo.setPower(fiPower);
            frenteDerecho.setPower(fdPower);
            atrasIzquierdo.setPower(aiPower);
            atrasDerecho.setPower(adPower);

            // Telemetría
            telemetry.addData("Modo Lento", modoLento ? "ACTIVADO" : "Desactivado");
            telemetry.addData("FI Power", fiPower);
            telemetry.addData("FD Power", fdPower);
            telemetry.addData("AI Power", aiPower);
            telemetry.addData("AD Power", adPower);
            telemetry.update();
        }
    }
}
