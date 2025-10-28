//NOT FINISHED

package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Auto IMU Test by Emiliano")

public class AutoIMUMecanum extends LinearOpMode{
    //Variables globales
    public DcMotor frenteIzquierdo, frenteDerecho, atrasIzquierdo, atrasDerecho;

    public IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {


        // Inicializar 4 motores
        frenteIzquierdo = hardwareMap.get(DcMotor.class, "leftFront");
        frenteDerecho = hardwareMap.get(DcMotor.class, "rightFront");
        atrasIzquierdo = hardwareMap.get(DcMotor.class, "leftBack");
        atrasDerecho = hardwareMap.get(DcMotor.class, "rightBack");



        //Invertir motores para girar en el mismo sentido
        frenteDerecho.setDirection(DcMotorSimple.Direction.FORWARD);
        atrasDerecho.setDirection(DcMotorSimple.Direction.REVERSE);


        imu = hardwareMap.get(IMU.class, "imu");


        //Configuración para que los motores funcionen con encoder
        frenteIzquierdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frenteDerecho.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasIzquierdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasDerecho.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frenteIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frenteDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        atrasIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        atrasDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Inicializar IMU
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD))
        );

        waitForStart();

        while(opModeIsActive()){
            giroAngulo(180);
            sleep(3000);
            avanzarRecto(0.7,8);
            sleep(3000);
            giroAngulo(270);
            sleep(3000);
            avanzarRecto(0.7,8);
            sleep(3000);
            giroAngulo(0);
            sleep(3000);
            avanzarRecto(0.7,8);
            sleep(3000);
            giroAngulo(90);
            sleep(3000);
            avanzarRecto(0.7,8);
            sleep(3000);
            break;
        }
    }

// Creación de funciones adicionales //

    //Función para giro preciso
    public void giroAngulo(double angulo){
        double kp = 0.015;       // Ganancia proporcional (ajustable)
        double minPower = 0.12;  // Potencia mínima para vencer fricción estática
        double tolerancia = 0.5; // Error aceptable en grados

        while (opModeIsActive()) {
            double error = obtenerError(angulo);

            // Si el error es suficientemente pequeño, terminamos el giro
            if (Math.abs(error) <= tolerancia) break;

            // Calcular potencia proporcional
            double power = kp * error;

            // Asegurarse de que la potencia no sea demasiado baja
            if (Math.abs(power) < minPower) {
                power = minPower * Math.signum(error);
            }

            // Aplicar giro
            frenteIzquierdo.setPower(power);
            frenteDerecho.setPower(-power);
            atrasIzquierdo.setPower(power);
            atrasDerecho.setPower(-power);


            telemetry.addData("Ángulo actual", obtenerAngulo());
            telemetry.addData("Error", error);
            telemetry.addData("Potencia", power);
            telemetry.update();
        }

        // Detener motores al final del giro
        poderMotor(0);
    }

    //Función para obtener ángulo
    public double obtenerAngulo(){
        return -(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))+180;
    }

    //Funcionar para calcular error (posición objetivo - posición actual)
    public double obtenerError(double anguloObjetivo){
        //Calcular el error (distancia al ángulo objetivo)
        double error = anguloObjetivo - obtenerAngulo();
        if(error > 180)error -= 360;
        if(error < -180)error += 360;
        return error;
    }

    //Función para avanzar recto
    public void avanzarRecto(double poder, double distanciaCM){
        double targetAngle = obtenerAngulo();
        double kp = 0.015;
        // Calcular ticks necesarios (ajusta según tus motores y ruedas)
        double ticksPorVuelta = 336;
        double diametroRuedaCM = 7.5;
        double ticksPorCM = ticksPorVuelta / (Math.PI * diametroRuedaCM); //28.88
        int ticksObjetivo = (int)(distanciaCM * ticksPorCM);

        // Resetear encoders y configurar modo
        frenteIzquierdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frenteDerecho.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasIzquierdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasDerecho.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frenteIzquierdo.setTargetPosition(ticksObjetivo);
        frenteDerecho.setTargetPosition(ticksObjetivo);
        atrasIzquierdo.setTargetPosition(ticksObjetivo);
        atrasDerecho.setTargetPosition(ticksObjetivo);

        frenteIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frenteDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        atrasIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        atrasDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frenteIzquierdo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frenteDerecho.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        atrasIzquierdo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        atrasDerecho.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frenteIzquierdo.setPower(poder);
        frenteDerecho.setPower(poder);
        atrasIzquierdo.setPower(poder);
        atrasDerecho.setPower(poder);

        poderMotor(0);
        frenteIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frenteDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        atrasIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        atrasDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        while (opModeIsActive() &&
//                Math.abs(frenteIzquierdo.getCurrentPosition()) < ticksObjetivo &&
//                Math.abs(frenteDerecho.getCurrentPosition()) < ticksObjetivo &&
//                Math.abs(atrasIzquierdo.getCurrentPosition()) < ticksObjetivo &&
//                Math.abs(atrasDerecho.getCurrentPosition()) < ticksObjetivo) {
//
//            double error = obtenerError(targetAngle);
//            double correction = error * kp;
//
//            // Limitar la corrección para evitar sobresaturación
//            correction = Math.max(Math.min(correction, 0.3), -0.3);
//
//            // Aplicar corrección (izquierda o derecha)
//            frenteIzquierdo.setPower(poder + correction);
//            frenteDerecho.setPower(poder - correction);
//            atrasIzquierdo.setPower(poder + correction);
//            atrasDerecho.setPower(poder - correction);
//
//            telemetry.addData("Error", error);
//            telemetry.addData("Corrección", correction);
//            telemetry.addData("Heading: ", obtenerAngulo());
//            telemetry.addData("FI", frenteIzquierdo.getCurrentPosition());
//            telemetry.addData("FD", frenteDerecho.getCurrentPosition());
//            telemetry.addData("AD", atrasDerecho.getCurrentPosition());
//            telemetry.addData("AI", atrasIzquierdo.getCurrentPosition());
//            telemetry.update();
//        }
//
//        poderMotor(0);
    }

    //Funciones de giro
    public void enfrente(double poder){
        frenteIzquierdo.setPower(poder);
        frenteDerecho.setPower(poder);
        atrasIzquierdo.setPower(poder);
        atrasDerecho.setPower(poder);
    }
    public void atras(double poder){
        frenteIzquierdo.setPower(-poder);
        frenteDerecho.setPower(-poder);
        atrasIzquierdo.setPower(-poder);
        atrasDerecho.setPower(-poder);
    }
    public void derecha(double poder){
        frenteIzquierdo.setPower(poder);
        frenteDerecho.setPower(-poder);
        atrasIzquierdo.setPower(poder);
        atrasDerecho.setPower(-poder);
    }
    public void izquierda(double poder){
        frenteIzquierdo.setPower(-poder);
        frenteDerecho.setPower(poder);
        atrasIzquierdo.setPower(-poder);
        atrasDerecho.setPower(poder);
    }

    public void poderMotor(double poder){
        frenteIzquierdo.setPower(poder);
        frenteDerecho.setPower(poder);
        atrasIzquierdo.setPower(poder);
        atrasDerecho.setPower(poder);
    }
}

//
