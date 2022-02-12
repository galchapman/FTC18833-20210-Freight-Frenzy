package org.firstinspires.ftc.teamcode.lib.auto.commands;

import java.util.List;

import edu.megiddo.lions.Environment;
import edu.megiddo.lions.TokenType;
import edu.megiddo.lions.Tokenizer;
import edu.megiddo.lions.commands.Command;
import edu.megiddo.lions.execption.LanguageException;
import edu.megiddo.lions.execption.ObjectNotFoundException;

public class StaticSetCommand<T> implements Command<T> {

    @Override
    public T run(Environment env, Tokenizer.Token function, List<Tokenizer.Token> args) throws LanguageException {
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
                throw new ObjectNotFoundException(value);
            env.setDoubleVariable(name.toString(), env.parseValueToken(value));
        } else if (value.type == TokenType.Variable) {
            if (env.hasBooleanVarRead(value.value) && env.hasBooleanVarWrite(name.toString())) {
                env.setBooleanVariable(name.toString(), env.getBooleanVariable(value.value));
            } else if (env.hasDoubleVarRead(value.value) && env.hasDoubleVarWrite(name.toString())) {
                env.setDoubleVariable(name.toString(), env.getDoubleVariable(value.value));
            } else {
                throw new ObjectNotFoundException(value);
            }
        }
        return null;
    }
}
