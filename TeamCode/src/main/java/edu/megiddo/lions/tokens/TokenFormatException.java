package edu.megiddo.lions.tokens;

import edu.megiddo.lions.Tokenizer;
import edu.megiddo.lions.execption.LanguageException;

public class TokenFormatException extends LanguageException {
    public TokenFormatException(String error, Tokenizer.Token token) {
        super("TokenFormatException", error, token.line, token.character);
    }
}
