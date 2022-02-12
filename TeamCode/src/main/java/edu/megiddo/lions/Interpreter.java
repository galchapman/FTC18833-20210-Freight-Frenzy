package edu.megiddo.lions;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Queue;

import edu.megiddo.lions.commands.Command;
import edu.megiddo.lions.execption.InvalidTokenException;
import edu.megiddo.lions.execption.LanguageException;
import edu.megiddo.lions.execption.ObjectNotFoundException;
import edu.megiddo.lions.execption.SyntaxException;

public class Interpreter<T> {
    private final Map<String, Command<T>> commands;
    public final Environment env;

    public Interpreter() {
        this(DistanceUnit.Meter);
    }

    public Interpreter(DistanceUnit distanceUnit) {
        env = new Environment(distanceUnit);
        commands = new HashMap<>();
    }

    public List<T> interpret(String file, String text) throws LanguageException {
        try {
            Queue<Tokenizer.Token> tokens = Tokenizer.getTokens(text);
            List<T> results = new LinkedList<>();

            while (!tokens.isEmpty()) {
                var token = tokens.remove();
                switch (token.type) {
                    case Command:
                        T out = runCommand(token, tokens);
                        if (out != null) {
                            results.add(out);
                        }
                        break;
                    case If:
                        Tokenizer.Token condition_token = tokens.remove();
                        if (condition_token.type != TokenType.Variable) {
                            throw new SyntaxException("Condition must be a boolean variable", condition_token.line, condition_token.character);
                        }
                        // main if
                        boolean condition = env.getBooleanVariable(condition_token.value);
                        nextLine(tokens);
                        parseLogicBlock(tokens, condition, results);
                        // elif
                        while (!tokens.isEmpty() && Objects.requireNonNull(tokens.peek()).type == TokenType.Elif) {
                            tokens.remove();
                            if (condition) {
                                nextLine(tokens);
                                parseLogicBlock(tokens, false, results);
                            } else {
                                condition_token = tokens.remove();
                                nextLine(tokens);
                                if (condition_token.type != TokenType.Variable) {
                                    throw new SyntaxException("Condition must be a boolean variable", condition_token.line, condition_token.character);
                                }
                                condition = env.getBooleanVariable(condition_token.value);
                                parseLogicBlock(tokens, condition, results);
                            }
                        }
                        // else
                        if (!tokens.isEmpty() && Objects.requireNonNull(tokens.peek()).type == TokenType.Else) {
                            tokens.remove();
                            nextLine(tokens);
                            // Runs only if all ifs and elifs failed
                            parseLogicBlock(tokens, !condition, results);
                        }
                        break;
                    case NewLine:
                    case Indent:
                    case Comment:
                        nextLine(tokens);
                        break;
                    default:
                        throw new InvalidTokenException(token);
                }
            }
            return results;
        } catch (LanguageException e) {
            throw e.setFile(file);
        }
    }

    public void registerCommand(String name, Command<T> command) {
        commands.put(name, command);
    }

    private T runCommand(Tokenizer.Token command, Queue<Tokenizer.Token> tokens) throws LanguageException {
        try {
            return Objects.requireNonNull(commands.get(command.value)).run(env, command, getCommandArgs(tokens));
        } catch (NullPointerException e) {
            throw new ObjectNotFoundException(command);
        }
    }

    private List<Tokenizer.Token> getCommandArgs(Queue<Tokenizer.Token> tokens) throws SyntaxException {
        List<Tokenizer.Token> args = new ArrayList<>();

        while (!tokens.isEmpty()) {
            Tokenizer.Token arg = tokens.remove();
            if (arg.type == TokenType.NewLine
                    || arg.type == TokenType.Comment
                    || arg.type == TokenType.Indent) {
                break;
            } else if (arg.type == TokenType.Value || arg.type == TokenType.Variable) {
                args.add(arg);
            } else {
                throw new SyntaxException("unexpected token type `" + arg.value + '`', arg.line, arg.character);
            }
        }
        return args;
    }

    private void parseLogicBlock(Queue<Tokenizer.Token> tokens, boolean execute_block, List<T> results) throws LanguageException {
        while (!tokens.isEmpty() && (Objects.requireNonNull(tokens.peek()).type == TokenType.Indent
                || Objects.requireNonNull(tokens.peek()).type == TokenType.NewLine
                || Objects.requireNonNull(tokens.peek()).type == TokenType.Comment)) {
            Tokenizer.Token pre_token = tokens.remove();
            if (pre_token.type == TokenType.NewLine || pre_token.type == TokenType.Comment)
                continue;

            Tokenizer.Token token = tokens.remove();
            if (token.type == TokenType.Command) {
                if (execute_block) {
                    T out = runCommand(token, tokens);
                    if (out == null) {
                        throw new SyntaxException("General error", token.line, token.character);
                    } else {
                        results.add(out);
                    }
                } else {
                    nextLine(tokens); // skip command
                }
            } else if (token.type != TokenType.NewLine && token.type != TokenType.Comment) {
                throw new SyntaxException("Unexpected token: " + token, token.line, token.character);
            }
        }
    }

    private void nextLine(Queue<Tokenizer.Token> tokens) {
        do {
            while (!tokens.isEmpty() && Objects.requireNonNull(tokens.peek()).type != TokenType.NewLine) {
                tokens.remove();
            }
        } while(!tokens.isEmpty() && Objects.requireNonNull(tokens.peek()).type == TokenType.Comment);
        if (!tokens.isEmpty()) {
            tokens.remove();
        }
    }
}
