package org.firstinspires.ftc.teamcode.lib.auto.commands;

import java.util.List;
import java.util.Objects;
import java.util.function.Function;

import edu.megiddo.lions.Environment;
import edu.megiddo.lions.Tokenizer;
import edu.megiddo.lions.execption.FunctionException;
import edu.megiddo.lions.execption.ObjectNotFoundException;

public class EnumArgCommand<T extends Enum<T>, F> implements edu.megiddo.lions.commands.Command<F> {
    private final Class<T> enumType;
    private final Function<T, F> function;

    public EnumArgCommand(Class<T> enumType, Function<T, F> function) {
        this.enumType = enumType;
        this.function = function;
    }


    @Override
    public F run(Environment env, Tokenizer.Token token, List<Tokenizer.Token> args) throws FunctionException, ObjectNotFoundException {
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
