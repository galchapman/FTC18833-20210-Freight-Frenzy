package org.firstinspires.ftc.teamcode.lib.auto;

import org.firstinspires.ftc.teamcode.lib.auto.commands.SetCommand;

import java.io.BufferedReader;
import java.io.FileReader;

import edu.megiddo.lions.Interpreter;
import edu.megiddo.lions.execption.LanguageException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoLoader {
    public Interpreter<Command> interpreter = new Interpreter<>();

    public AutoLoader() {
        interpreter.registerCommand("set", new SetCommand());
    }

    public Command load(String file) throws LanguageException {
        StringBuilder sb = new StringBuilder();
        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line = br.readLine();

            while (line != null) {
                sb.append(line);
                sb.append(System.lineSeparator());
                line = br.readLine();
            }
            br.close();
        } catch (Exception ignored) {
            return null;
        }

        return new SequentialCommandGroup(interpreter.interpret(file, sb.toString()).toArray(new Command[0]));
    }
}
