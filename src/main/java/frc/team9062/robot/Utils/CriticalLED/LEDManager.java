package frc.team9062.robot.Utils.CriticalLED;

public class LEDManager {
    private CriticalLED led;
    private LEDCommandBase activeCommand = null;
    private boolean init = false;
    private static LEDManager instance;

    public LEDManager (CriticalLED led) {
        this.led = led;
    }

    public static LEDManager getInstance(CriticalLED led) {
        if(instance == null) {
            instance = new LEDManager(led);
        }

        return instance;
    }

    public LEDCommandBase activeCommand() {
        return activeCommand;
    }

    public void endCurrentCommand() {
        activeCommand.end(false);
        activeCommand = null;
        init = false;
    }
    
    public void runActiveCommand() {
        activeCommand.execute();
    }

    public void setActiveCommand(LEDCommandBase command) {
        if(activeCommand != null) {
            //if(!command.compare(activeCommand)){
                endCurrentCommand();
            //}
        }
        
        activeCommand = command;
    }

    public void ManageCommands() {
        if(activeCommand != null) {
            if(init) {
                runActiveCommand();
            }else {
                activeCommand.initialize();
                init = true;
            }
        }else {
            led.setColourless();
        }
    }

    public Runnable getLEDRunnable() {
        return LEDRunnable;
    }

    Runnable LEDRunnable = new Runnable() {
        @Override
        public void run() {
            ManageCommands();
        }
    };

    Thread LEDManagerThread =
        new Thread(
            () -> {
                while (true) {
                    try {
                        ManageCommands();
                        Thread.sleep(20);
                    } catch (Exception e) {
                        e.printStackTrace();
                        endCurrentCommand();
                    }
                }
            }
    );     
}