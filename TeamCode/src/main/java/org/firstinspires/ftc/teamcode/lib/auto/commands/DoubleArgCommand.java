package org.firstinspires.ftc.teamcode.lib.auto.commands;

import java.util.List;

import edu.megiddo.lions.Environment;
import edu.megiddo.lions.TokenType;
import edu.megiddo.lions.Tokenizer;
import edu.megiddo.lions.execption.FunctionException;
import edu.megiddo.lions.execption.ObjectNotFoundException;
import edu.megiddo.lions.tokens.TokenFormatException;
import edu.wpi.first.wpilibj2.command.Command;
import kotlin.jvm.functions.Function2;

public class DoubleArgCommand implements edu.megiddo.lions.commands.Command<Command> {
    private final Function2<Double, Double, Command> function;

    public DoubleArgCommand(Function2<Double, Double, Command> function) {
        this.function = function;
    }

    @Override
    public Command run(Environment env, Tokenizer.Token token, List<Tokenizer.Token> args) throws FunctionException, ObjectNotFoundException, TokenFormatException {
        if (args.size() != 2) {
            throw new FunctionException("Invalid arg amount", token);
        }

        double[] func_args = new double[2];

        for (int i = 0; i < 2; i++) {
            if (args.get(i).type == TokenType.Variable) {
                if (env.hasDoubleVarRead(args.get(i).value)) {
                    func_args[i] = env.getDoubleVariable(args.get(i).value);
                } else {
                    throw new ObjectNotFoundException(args.get(i));
                }
            } else if (args.get(0).type == TokenType.Value) {
                func_args[i] = env.parseValueToken(args.get(i));
            } else {
                throw new FunctionException("Arg must be value or variable", token);
            }
        }

        return function.invoke(func_args[0], func_args[1]);
    }
}
