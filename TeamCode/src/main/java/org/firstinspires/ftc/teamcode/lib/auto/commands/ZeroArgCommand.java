package org.firstinspires.ftc.teamcode.lib.auto.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.megiddo.lions.Environment;
import edu.megiddo.lions.Tokenizer;
import edu.megiddo.lions.commands.Command;
import edu.megiddo.lions.execption.FunctionException;
import edu.megiddo.lions.execption.LanguageException;

public class ZeroArgCommand<T> implements Command<T> {
    private final Supplier<T> supplier;

    public ZeroArgCommand(Supplier<T> supplier) {
        this.supplier = supplier;
    }

    @Override
    public T run(Environment env, Tokenizer.Token function, List<Tokenizer.Token> args) throws LanguageException {
        if (args.size() != 0) {
            throw new FunctionException("Invalid arg amount", function);
        }

        return supplier.get();
    }
}
