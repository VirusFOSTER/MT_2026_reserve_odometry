#include "../../../../include/detection/common/json/json_common.hpp"

namespace sys { namespace json { namespace common {
bool isWhitespace (int ch) {
    return ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r';
}

bool isDigit (int ch) {
    return ch >= '0' && ch <= '9';
}

bool isDigitOrSign (int ch) {
    return isDigit(ch) || ch == '-';
}

bool isAlpha (int ch) {
    return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z');
}

bool isQuotelessStringChar (int ch) {
    return !(ch == ',' || ch == ':'
            || ch == '{' || ch == '}'
            || ch == '[' || ch == ']'
            || isWhitespace(ch));
}

bool isString (std::string text) {
    return text.length() > 1 && text[0] == '\"';
}

std::string stripString (std::string text) {
    return (text.length() > 1? text.substr (1, text.length() - 2) : text);
}

bool isNumber(std::string text) {
    return text.length() > 0 && (isDigitOrSign(text[0]) || text[0] == 'e' || text[0] == 'E');
}

bool isInteger (std::string text) {
    if (!isNumber(text))
        return false;
    return (text.find('.') == std::string::npos &&
            text.find('e') == std::string::npos &&
            text.find('E') == std::string::npos);
}

bool isKeyword (std::string text) {
    const int nWords = 3;
    const std::string keywords[nWords]= { "true", "false", "null" };
    for (int i = 0; i < nWords; i++)
        if (text == keywords[i])
            return true;
    return false;
}

bool isValue (std::string text) {
    return isString(text) || isNumber(text) || isKeyword(text);
}

std::string i2str (int x) {
    std::ostringstream s;
    s << x;
    return s.str();
}
}}}
