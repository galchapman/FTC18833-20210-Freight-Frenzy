package org.firstinspires.ftc.teamcode.lib.auto.commands;

import android.content.pm.PackageManager;

import java.util.List;

import edu.megiddo.lions.Environment;
import edu.megiddo.lions.TokenType;
import edu.megiddo.lions.Tokenizer;
import edu.megiddo.lions.commands.Command;
import edu.megiddo.lions.tokens.TokenException;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetCommand implements Command<edu.wpi.first.wpilibj2.command.Command> {

    @Override
    public edu.wpi.first.wpilibj2.command.Command run(Environment env, List<Tokenizer.Token> args) throws PackageManager.NameNotFoundException, TokenException {
        StringBuilder name = new StringBuilder();
        // Join names
        for (int i = 0; i < args.size() - 1; i++) {
            if (i != 0) {
                name.append('.');
            }
            name.append(args.get(i).value);
        }

        Tokenizer.Token value = args.get(args.size()-1);
        if (value.type == TokenType.Value) {
            if (!env.hasDoubleVarWrite(name.toString()))
                throw new PackageManager.NameNotFoundException("name: `" + name + "` not found");
            return new InstantCommand(
                    () -> env.setDoubleVariable(name.toString(), env.parseValueToken(value.value))
            );
        } else {
            if (env.hasBooleanVarRead(value.value) && env.hasBooleanVarWrite(name.toString())) {
                return new InstantCommand(
                        () -> env.setBooleanVariable(name.toString(), env.getBooleanVariable(value.value))
                );
            } else if (env.hasDoubleVarRead(value.value) && env.hasDoubleVarRead(name.toString())) {
                return new InstantCommand(
                        () -> env.setDoubleVariable(name.toString(), env.getDoubleVariable(value.value))
                );
            } else {
                throw new PackageManager.NameNotFoundException("name: `" + name + "` not found");
            }
        }
    }
}
