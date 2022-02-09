package edu.megiddo.lions;

import androidx.annotation.NonNull;

import java.nio.charset.StandardCharsets;
import java.util.Deque;
import java.util.LinkedList;
import java.util.Queue;

import edu.megiddo.lions.execption.LanguageException;
import edu.megiddo.lions.execption.TokenFormatException;

public class Tokenizer {

    public static class Token {
        public final TokenType type;
        public final String value;
        public final int line;
        public final int character;

        public Token(TokenType type, String value, int line, int character) {
            this.type = type;
            this.value = value;
            this.line = line;
            this.character = character;
        }

        @NonNull
        @Override
        public String toString() {
            return type + "(`" + value + "`)";
        }
    }

    static public Queue<Token> getTokens(String text) throws LanguageException {
        Deque<Token> tokens = new LinkedList<>();
        int line = 0;
        int character = 0;

        StringBuilder currentToken = new StringBuilder();
        TokenType token = TokenType.Empty;
        for (var c : text.getBytes(StandardCharsets.UTF_8)) {
            character++;
            if (c == '\r')
                continue;

            if (((c == ' ' || c == '\t') && token != TokenType.Comment) || c == '\n') {
                if (c == '\n') {
                    line++;
                    character = -1;
                }

                if (token == TokenType.Empty && tokens.getLast().type != TokenType.Indent) {
                    tokens.add(new Token(TokenType.Indent, "", line, character));
                } else if (token == TokenType.Logic) {
                    if (currentToken.toString().equalsIgnoreCase("if")) {
                        tokens.add(new Token(TokenType.If, "", line, character));
                    } else if (currentToken.toString().equalsIgnoreCase("elif")) {
                        tokens.add(new Token(TokenType.Elif, "", line, character));
                    } else if (currentToken.toString().equalsIgnoreCase("else")) {
                        tokens.add(new Token(TokenType.Else, "", line, character));
                    } else {
                        throw new TokenFormatException("Expected if else or elif after @ got " + currentToken, line, character);
                    }
                } else {
                    tokens.add(new Token(token, currentToken.toString(), line, character));
                }

                if (c == '\n') {
                    tokens.add(new Token(TokenType.NewLine, "", line, character));
                } else if (c == '\t'  && tokens.getLast().type != TokenType.Indent) {
                    tokens.add(new Token(TokenType.Indent, "", line, character));
                }

                token = TokenType.Empty;
                currentToken = new StringBuilder();
                continue;
            } else if (token == TokenType.Empty) {
                switch (c) {
                    case '-':
                    case '0':
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                    case '5':
                    case '6':
                    case '7':
                    case '8':
                    case '9':
                        token = TokenType.Value;
                        break;
                    case '#':
                        token = TokenType.Comment;
                        continue;
                    case '@':
                        token = TokenType.Logic;
                        continue;
                    default:
                        if (tokens.isEmpty() || tokens.getLast().type == TokenType.NewLine || tokens.getLast().type == TokenType.Indent) {
                            token = TokenType.Command;
                        } else {
                            token = TokenType.Variable;
                        }
                }
            }
            currentToken.append((char)c);
        }

        if (token != TokenType.Empty)
            tokens.add(new Token(token, currentToken.toString(), line, character));

        return tokens;
    }
}
