#ifndef SYSTEM_JSON_COMMON_HPP
#define SYSTEM_JSON_COMMON_HPP

#include <istream>
#include <sstream>
#include <fstream>
#include <iostream>

namespace sys {
namespace json {
namespace common {
bool isWhitespace (int ch);

bool isDigit (int ch);

bool isDigitOrSign (int ch);

bool isAlpha (int ch);

bool isQuotelessStringChar (int ch);

bool isString (std::string text);

std::string stripString (std::string text);

bool isNumber (std::string text);

bool isInteger (std::string text);

bool isKeyword (std::string text);

bool isValue (std::string text);

std::string i2str (int x);
}
}
}

#endif
