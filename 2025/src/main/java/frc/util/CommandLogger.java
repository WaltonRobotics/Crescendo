package frc.util;

import java.util.Optional;
import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandLogger {
    public static String info(Command cmd) {
        String reqs = "";
        for (var req : cmd.getRequirements()) {
            reqs += req.getName() + " ";
        }

        return "Name: " + cmd.getName() + 
            "\nSubsys: " + cmd.getSubsystem() +
            "\nReqs: {" + reqs.trim() + "}";
    }

   
    public static BiConsumer<Command, Optional<Command>> commandInterruptLogger() {
        return (cmd, optIntCmd) -> {
            String dump = "INT'D CMD:\n" + info(cmd);
            if (optIntCmd.isPresent()) {
                dump += "\nINT'R CMD:\n" + info(optIntCmd.get()); 
            }
            System.out.println(dump);
        };
    }
}
