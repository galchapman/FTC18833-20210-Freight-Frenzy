package org.firstinspires.ftc.teamcode.lib.auto.commands;

import java.util.List;
import java.util.function.Function;

import edu.megiddo.lions.Environment;
import edu.megiddo.lions.TokenType;
import edu.megiddo.lions.Tokenizer;
import edu.megiddo.lions.execption.FunctionException;
import edu.megiddo.lions.execption.ObjectNotFoundException;
import edu.megiddo.lions.tokens.TokenFormatException;
import edu.wpi.first.wpilibj2.command.Command;

public class SingleArgCommand implements edu.megiddo.lions.commands.Command<Command> {
    private final Function<Double, Command> function;

    public SingleArgCommand(Function<Double, Command> function) {
        this.function = function;
    }

    @Override
    public Command run(Environment env, Tokenizer.Token token, List<Tokenizer.Token> args) throws FunctionException, ObjectNotFoundException, TokenFormatException {
        if (args.size() != 1) {
            throw new FunctionException("Invalid arg amount", token);
        }

        if (args.get(0).type == TokenType.Variable) {
            if (env.hasDoubleVarRead(args.get(0).value)) {
                return function.apply(env.getDoubleVariable(args.get(0).value));
            } else {
                throw new ObjectNotFoundException(args.get(0));
            }
        } else if (args.get(0).type == TokenType.Value) {
            return function.apply(env.parseValueToken(args.get(0)));
        } else {
            throw new TokenFormatException("Arg must be value or variable", args.get(0));
        }
    }
}
