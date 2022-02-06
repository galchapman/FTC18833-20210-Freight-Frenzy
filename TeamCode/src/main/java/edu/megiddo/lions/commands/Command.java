package edu.megiddo.lions.commands;

import android.content.pm.PackageManager;

import java.util.List;

import edu.megiddo.lions.Environment;
import edu.megiddo.lions.Tokenizer;
import edu.megiddo.lions.tokens.TokenException;

public interface Command<T> {

    T run(Environment env, List<Tokenizer.Token> args) throws PackageManager.NameNotFoundException, TokenException;
}
