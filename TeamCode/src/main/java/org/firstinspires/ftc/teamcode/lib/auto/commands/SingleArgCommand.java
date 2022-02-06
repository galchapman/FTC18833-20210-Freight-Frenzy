package org.firstinspires.ftc.teamcode.lib.auto.commands;

import java.util.List;
import java.util.function.Function;

import edu.megiddo.lions.Environment;
import edu.megiddo.lions.TokenType;
import edu.megiddo.lions.Tokenizer;
import edu.megiddo.lions.tokens.TokenException;
import edu.wpi.first.wpilibj2.command.Command;

public class SingleArgCommand implements edu.megiddo.lions.commands.Command<Command> {
    private final Function<Double, Command> function;

    public SingleArgCommand(Function<Double, Command> function) {
        this.function = function;
    }

    @Override
    public Command run(Environment env, List<Tokenizer.Token> args) throws TokenException {
        if (args.size() != 1) {
            throw new TokenException("Invalid arg amount");
        }

        if (args.get(0).type == TokenType.Variable) {
            if (env.hasDoubleVarRead(args.get(0).value)) {
                return function.apply(env.getDoubleVariable(args.get(0).value));
            } else {
                throw new TokenException("Arg name isn't defined");
            }
        } else if (args.get(0).type == TokenType.Value) {
            return function.apply(env.parseValueToken(args.get(0).value));
        } else {
            throw new TokenException("Arg must be value or variable");
        }
    }
}
