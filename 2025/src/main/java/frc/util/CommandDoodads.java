package frc.util;


import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public final class CommandDoodads {
    private CommandDoodads() {}

    public static Command printLater(Supplier<String> stringSup) {
		return defer(() -> {
			return print(stringSup.get());
		}, Set.of());
	}
    
}
