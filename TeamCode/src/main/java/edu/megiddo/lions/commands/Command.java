package edu.megiddo.lions.commands;

import java.util.List;

import edu.megiddo.lions.Environment;
import edu.megiddo.lions.Tokenizer;
import edu.megiddo.lions.execption.LanguageException;

public interface Command<T> {

    T run(Environment env, Tokenizer.Token function, List<Tokenizer.Token> args) throws LanguageException;
}
