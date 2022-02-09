package edu.megiddo.lions.execption;

import androidx.annotation.NonNull;

import edu.megiddo.lions.Tokenizer;

public class InvalidTokenException extends SyntaxException {
    public InvalidTokenException(@NonNull Tokenizer.Token token) {
        super("Token " + token.value + " was unexpected at that point", token.line, token.character);
    }
}
