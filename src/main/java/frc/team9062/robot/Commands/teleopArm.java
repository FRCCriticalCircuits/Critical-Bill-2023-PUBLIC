package frc.team9062.robot.Commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team9062.robot.Subsystems.ArmSubsystem;
import frc.team9062.robot.Subsystems.ArmSubsystem.ARM_STATE;
import frc.team9062.robot.Utils.ControllerBinds;
import frc.team9062.robot.Utils.IO;

public class teleopArm extends CommandBase{
    private ArmSubsystem armSubsystem;
    private ControllerBinds binds;
    private IO io;
    private boolean manualArmControl;
    private Debouncer toggleDebouncer = new Debouncer(0.2);

    public teleopArm() {
        armSubsystem = ArmSubsystem.getInstance();
        binds = ControllerBinds.getInstance();
        io = IO.getInstance();

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        armSubsystem.setShoulder(io.getOperatorRightY());

        if(toggleDebouncer.calculate(Math.abs(io.getOperatorLeftY()) > 0)) manualArmControl = true;

        if(manualArmControl) {
            armSubsystem.setArm(-io.getOperatorLeftY());
            armSubsystem.setCurrentArmState(ARM_STATE.MANUAL);
        }
    
        if(binds.setArmStowed()) {
            armSubsystem.setCurrentArmState(ARM_STATE.HOLD);
            manualArmControl = false;
        } else if(binds.setArmHigh()) {
            armSubsystem.setCurrentArmState(ARM_STATE.HIGH);
            manualArmControl = false;
        } else if(binds.setArmMid()) {
            armSubsystem.setCurrentArmState(ARM_STATE.MID);
            manualArmControl = false;
        } else if(binds.setArmLow()) {
            armSubsystem.setCurrentArmState(ARM_STATE.LOW);
            manualArmControl = false;
        } else if(binds.setArmDoubleSub()) {
            armSubsystem.setCurrentArmState(ARM_STATE.DOUBLE_SUB);
            manualArmControl = false;
        } else if(binds.setArmOrigin()) {
            armSubsystem.setCurrentArmState(ARM_STATE.HOME);
            manualArmControl = false;
        }

        if(binds.ConeIntake()) {
            armSubsystem.setConeIntake();
        } else if(binds.CubeIntake()) {
            armSubsystem.setCubeIntake();
        } else if(binds.Outtake()) {
            armSubsystem.setIntake(-0.8);
            //if(!armSubsystem.objectDetected()) armSubsystem.setCurrentIntakeState(INTAKE_STATE.IDLE);
        } else {
            armSubsystem.setIntake(0.05);
        }
    }

    @Override
    public void end(boolean interrupted) {}
}