#pragma once
#include <string>
#include <stdexcept>
#include <vector>
#include <optional>

namespace planner {

// PDDL向けトークン
enum class TokenType {
    LPAR,       // (
    RPAR,       // )
    KEYWORD,    // :requirements, :types, :predicates, :action, :parameters, :precondition, :effect
    VARIABLE,   // ?x, ?y1, ?from などの変数
    NAME,       // ドメイン名、述語名、関数名、定数名、型名、アクション名など
    NUMBER,     // 123, 4.56 などの数値
    DASH,       // -
    EOF_TOKEN   // 入力終端
};

// 位置情報 (エラー対応用)
struct Location {
    int line = 1; // indexは1始まり
    int col  = 1;
};

// トークン情報を格納するStruct
struct Token {
    TokenType    type;
    std::string  lexeme;   // 表示用（NAME, KEYWORD, VARIABLE, NUMBERで使用）
    Location     loc;
};

// Lexerエラー用例外Class
class LexerError : public std::runtime_error {
public:
    explicit LexerError(const std::string& msg) : std::runtime_error(msg) {} // msg を基底クラスに渡す
};

// PDDL用のLexer本体
class Lexer {
public:
    explicit Lexer(const std::string& input);

    // 関数の宣言
    // 1トークン先読みする関数
    const Token& peek();

    // 1トークン読み進める関数
    Token next();

    // 期待する型であるか確認する関数
    Token expect(TokenType t, const char* what = nullptr);

    // 現在位置を取得し返す関数
    Location location() const { return loc_; }

private:
    const std::string input_; // 入力文字列
    std::size_t i_ = 0; // 現在位置
    Location loc_; // 位置情報
    std::optional<Token> lookahead_; // 先読みトークン

    // 内部メンバ関数の宣言
    void skipWhitespaceAndComments(); // 空白と';'コメントをスキップする関数

    bool eof() const { return i_ >= input_.size(); } // 入力終端

    char cur() const { return input_[i_]; } // 現在位置の文字を取得

    void advance(); // 現在位置を1つ進める関数
    void advanceLine(); // 改行処理を行う関数

    static bool isNameStart(char c); // NAMEから始まるか判定する関数
    static bool isNameChar(char c); // NAMEであるか判定する関数
    static bool isDigit(char c); // 数字の判定

    Token readNameLike(); // 名前の読み取り
    Token readNumber(); // 数値の読み取り
    Token make(TokenType t, std::string lex = ""); // トークンを生成する関数
    [[noreturn]] void errorHere(const std::string& msg); // エラー処理を行う関数
};

} // namespace planner
