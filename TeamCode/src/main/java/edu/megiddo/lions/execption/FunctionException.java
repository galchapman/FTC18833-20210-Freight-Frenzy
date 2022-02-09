package edu.megiddo.lions.execption;

import edu.megiddo.lions.Tokenizer;

public class FunctionException extends LanguageException{
    public FunctionException(String error, Tokenizer.Token function) {
        super("FunctionException", error, function.line, function.character);
    }
}
