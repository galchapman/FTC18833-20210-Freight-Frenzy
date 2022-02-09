package org.firstinspires.ftc.teamcode.lib.auto.commands;

import java.util.List;
import java.util.Objects;
import java.util.function.Function;

import edu.megiddo.lions.Environment;
import edu.megiddo.lions.Tokenizer;
import edu.megiddo.lions.execption.FunctionException;
import edu.megiddo.lions.execption.ObjectNotFoundException;
import edu.wpi.first.wpilibj2.command.Command;

public class EnumArgCommand<T extends Enum<T>> implements edu.megiddo.lions.commands.Command<Command> {
    private final Class<T> enumType;
    private final Function<T, Command> function;

    public EnumArgCommand(Class<T> enumType, Function<T, Command> function) {
        this.enumType = enumType;
        this.function = function;
    }


    @Override
    public Command run(Environment env, Tokenizer.Token token, List<Tokenizer.Token> args) throws FunctionException, ObjectNotFoundException {
        if (args.size() != 1)
            throw new FunctionException("enum command require only one args.", token);

        T[] values = enumType.getEnumConstants();
        for (int i = 0; i < Objects.requireNonNull(values).length; i++) {
            if (values[i].name().equalsIgnoreCase(args.get(0).value))
                return function.apply(values[i]);
        }
        throw new ObjectNotFoundException(args.get(0));
    }
}
