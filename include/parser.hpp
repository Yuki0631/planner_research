#pragma once
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include "lexer.hpp"

namespace planner {

// ---ASTノードのデータ構造の定義---
// タイプ付き変数
struct TypedVar {
    std::string name, type;
};

// 命題
struct Atom {
    std::string pred;
    std::vector<std::string> args;
};

// 数値式で使う関数呼出部分
struct FuncTerm {
    std::string name;
    std::vector<std::string> args; // 変数名 or 定数名
};

// 数値定数または関数値用
struct NumExpr {
    enum Kind { CONST, FUNC, ADD, SUB, MUL, DIV } kind = CONST;
    double value = 0.0;           // CONST
    FuncTerm func;                // FUNC
    std::vector<NumExpr> args;    // ADD/SUB/MUL/DIV 用
};

// 論理式
struct Formula {
    enum Kind { ATOM, AND, NOT, INCREASE }

    kind = ATOM; // デフォルトではATOM

    Atom atom; // kind==ATOM
    std::vector<Formula> children; // kind==AND
    std::unique_ptr<Formula> child; // kind==NOT

    struct Inc
    {
        FuncTerm lhs; // 左辺
        NumExpr rhs;  // 右辺
    };
    Inc inc; // kind==INCREASE
};



// functions スキーマ
struct FunctionSchema {
    std::string name;
    std::vector<TypedVar> params;
    std::string rettype = "number"; 
};

// 述語スキーマ
struct PredicateSchema {
    std::string name;
    std::vector<TypedVar> params;
};

// アクション
struct Action {
    std::string name;
    std::vector<TypedVar> params;
    Formula precond;
    Formula effect;
};

// メトリック
struct Metric {
        enum Sense { MINIMIZE, MAXIMIZE } sense = MINIMIZE;
        NumExpr expr;
        bool present = false;
};

// ドメイン
struct Domain {
    std::string name;
    std::vector<std::string> requirements;  // strips, typing など
    std::vector<std::string> types;
    std::vector<PredicateSchema> predicates;
    std::vector<FunctionSchema> functions;
    std::vector<Action> actions;
    std::unordered_map<std::string, std::vector<std::string>> supertypes; // type の 下位から上位へのマップ (複数の親を許容)
};

// 初期化に数値が含まれる場合用の struct
struct NumericInit {
    FuncTerm lhs;   // 例: (total-cost)
    double   value; // 例: 0
};

// 問題インスタンス
struct Problem {
    std::string name;
    std::string domain_name;
    std::vector<std::pair<std::string,std::string>> objects; // (name,type)
    std::vector<Atom> init;
    std::vector<NumericInit> init_num;
    Formula goal; // and/not/atom のみ対応
    Metric metric; // メトリック
};

// ---Parser（再帰下降）---
class Parser {
public:
    explicit Parser(Lexer& lex) : lex_(lex) {}

    Domain  parseDomain(); // ドメインの解析を行う関数
    Problem parseProblem(); // 問題の解析を行う関数

    // デバッグ用表示関数
    static std::string to_string(const Formula& f);
    static std::string to_string(const Atom& a);
    static std::string to_string(const FuncTerm& ft);
    static std::string to_string(const NumExpr& ne);

private:
    Lexer& lex_;

    // 名前とキーワードを予想する関数
    std::string expectName(const char* what = "name");
    std::string expectKeyword(const char* what = "keyword"); // :xxx -> "xxx"

    // 式（再帰下降のコア）
    Formula parseFormula(); // (and ...)、(not ...)、(p t1 t2 ...)のいづれかを解析し、Formulaとして返す関数
    Atom    parseAtomWithHead(const std::string& head); // 命題の解析を行う関数
    Atom    parseAtom(); // 命題の解析をparseAtomWithHeadに委譲する関数

    // 関数項と数値式
    FuncTerm parseFuncTermInParens();
    NumExpr  parseNumericExpr();

    // Domainセクション
    std::vector<std::string> parseRequirementsSection(); // 'requirements' の後ろ
    void parseTypesSectionInto(Domain& d); // 'types' の後ろ
    std::vector<PredicateSchema> parsePredicatesSection(); // 'predicates' の後ろ
    std::vector<FunctionSchema> parseFunctionsSection(); // function 用の追加関数
    Action parseActionSection(); // :action ～ の塊

    // Problemセクション
    std::vector<std::pair<std::string,std::string>> parseObjectsSection(); // objects
    std::vector<Atom> parseInitSection(); // init
    void parseInitSectionInto(Problem& p); // 数値 init 用の追加関数
    void parseMetricSectionInto(Problem& p); // metric

    // 型付き変数列（(?x ?y - T ?z - U) など）
    std::vector<TypedVar> parseVarListInParens();
};

} // namespace planner
