package frc.team9062.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimedarSubsystem extends SubsystemBase{
    private Servo lime_servo = new Servo(0);
    
    private double origin_angle = 0.24;

    private PIDController controller = new PIDController(0.505, 0.05, 0.675);
    
    boolean inverted = true;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    /**
     * @param pos : between 0.24 to 1;
     */
    public void set(double pos){
        lime_servo.set(pos);
    }

    public void setAngle(double x){
        double progress = x / (lime_servo.getSpeed() - 0.24) * 360;
        set(progress);
    }


    public double getPosition(){
        return lime_servo.get();
    }

    public double getAngle(){
        double progress = (lime_servo.getSpeed() - 0.24) * 360;
        return progress;
    }

    public Boolean is_target(){
        double target = table.getEntry("tv").getDouble(0);
        if(target == 1) return true;
        else return false; 
    }

    public double get_tx(){
        return table.getEntry("tx").getDouble(0);
    }

    public double Pos2Angle(int position){
        double output = (355.2631578947368 * position) - 85.26315789473684;// angle = 355.26*position - 85.26.
        return output;
    }
    public double Angle2Pos(double d){
        double output = (d+85.26315789473684)/355.2631578947368; //position = (angle+85.26)/355.26
        return output;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("tx", get_tx());
        SmartDashboard.putNumber("angle", getAngle());

        if(is_target() && Math.abs(get_tx()) > 0.4){
            set(controller.calculate(getPosition(), 0.615 + Angle2Pos(get_tx())));
        }else{
            if(getPosition() >= 0.99){
                inverted = false;
            }else if (getPosition() <= 0.3){
                inverted = true;
            }
    
            SmartDashboard.putNumber("percentage of rotation", getPosition());
            SmartDashboard.putNumber("origin_angle", origin_angle);
            if(inverted){origin_angle += 0.0025;}
            else{origin_angle -= 0.0025;}
            
            set(origin_angle);
        }
    } 
}
