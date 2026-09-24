#include "../../../../include/detection/common/json/json_loader.hpp"
#include "../../../../include/detection/common/json/json_object.hpp"
#include "../../../../include/detection/common/json/json_array.hpp"
#include "../../../../include/detection/common/json/json_common.hpp"

using namespace sys;
using namespace json;
using namespace loader;

JsonLoader::JsonLoader (const std::string &str, int flags)
{
    strict = (flags & StrictnessMask) == Strict || (flags & StrictnessMask) == StrictComments;
    comments = (flags & StrictnessMask) == StrictComments || (flags & StrictnessMask) == NotStrict;

    if ((flags & SourceMask) == Auto)
    {
        int i = 0;
        while (common::isWhitespace(str[i])) i++;
        if (str[i] == '{' || str[i] == '[')
            flags |= String;
    }

    if ((flags & SourceMask) == String)
    {
        std::istringstream iss(str);
        stream = &iss;
        parse();
    }
    else
    {
        char *c_str = (char*)(str.c_str());
        std::ifstream ifs(c_str, std::ifstream::in | std::ifstream::binary);
        if (!ifs.good())
            throw std::string("JsonLoader: error opening file \"") + str + "\"";
        stream = &ifs;
        parse();
    }
}

//---------------------------------------------------------------------------

JsonLoader::JsonLoader (std::istream *aStream, int flags)
{
    strict = (flags & StrictnessMask) == Strict || (flags & StrictnessMask) == StrictComments;
    comments = (flags & StrictnessMask) == StrictComments || (flags & StrictnessMask) == NotStrict;
    stream = aStream;
    parse();
}

//---------------------------------------------------------------------------

JsonLoader::~JsonLoader ()
{
    for (int i = 0; i < (int) objs.size(); i++)
        delete objs[i];
    objs.clear();
    for (int i = 0; i < (int) arrs.size(); i++)
        delete arrs[i];
    arrs.clear();
}

//---------------------------------------------------------------------------

void JsonLoader::parse ()
{
    line = 0;
    offset = 0;
    arr = 0;
    obj = 0;
    if (!stream->good())
        throw std::string("JsonLoader: error opening stream");
    std::string token = readToken ();
    if (token == "[")
        arr = new array::JsonArray (this);
    else
    if (token == "{")
        obj = new object::JsonObject (this);
    else
        throwSyntax ("either object or array", token);
}

//---------------------------------------------------------------------------

void JsonLoader::throwString (std::string message)
{
    std::ostringstream s;
    s << "Error parsing JSON. Line " <<  (line+1) << ", offset " << offset << ": " << message << std::endl
        << "\"" << inputLine << "\"";
    throw s.str();
}

//---------------------------------------------------------------------------

void JsonLoader::throwSyntax (std::string expected, std::string got)
{
    throwString (std::string("Expected ") + expected + std::string(", got ") + got);
}

//---------------------------------------------------------------------------

void JsonLoader::skipToNewline ()
{
    while (stream->good() && stream->get() != '\n') {}
    line++;
    offset = 0;
    inputLine.clear();
}

//---------------------------------------------------------------------------

int JsonLoader::peekChar ()
{
    int ch = stream->peek();
    while (comments && ch == '/')
    {
        stream->get();
        if (stream->peek() == '/')
            skipToNewline ();
        else
            throwSyntax ("comment ('//')", std::string("/") + std::string(1, (char) ch));
        ch = stream->peek();
    }
    return ch;
}

//---------------------------------------------------------------------------

int JsonLoader::getChar (bool raw)
{
    int ch = stream->get();
    if (ch == '\n')
    {
        line++;
        offset = 0;
        inputLine.clear();
    }
    else
    if (ch == '\r')
        offset = 0;
    else
    if (comments && !raw && ch == '/' && stream->peek() == '/')
        skipToNewline ();
    else
    {
        offset++;
        inputLine += ch;
    }
    return ch;
}

//---------------------------------------------------------------------------

void JsonLoader::skipWhitespace ()
{
    int ch;
    while (stream->good() && common::isWhitespace(ch = peekChar()))
    {
        getChar();
    }
 }

//---------------------------------------------------------------------------

int JsonLoader::whatsNext ()
{
    skipWhitespace ();
    int ch = peekChar ();
    if (ch == '-' || common::isDigit(ch))
        ch = '0';
    else
    if (common::isAlpha(ch))
        ch = 'a';
    return ch;
}

//---------------------------------------------------------------------------

std::string JsonLoader::read ()
{
    int nch = whatsNext();
    switch (nch)
    {
    case '{':
    case '}':
    case '[':
    case ']':
    case ',':
    case ':':
        getChar();
        return std::string (1, (char) nch);
    case '\"':
        return readString ();
    case 'a':
        return readKeyword ();
    case '0':
        return readNumber ();
    case '=':
        if (!strict)
        {
            getChar();
            return std::string (1, (char) nch);
        } break;
    default:
        throwSyntax ("token or value", std::string(1, (char) nch));
    }
    return std::string();
}

//---------------------------------------------------------------------------

std::string JsonLoader::readToken ()
{
    skipWhitespace ();
    int ch = getChar();
    switch (ch)
    {
    case '{':
    case '}':
    case '[':
    case ']':
    case ',':
    case ':':
        return std::string (1, (char) ch);
    default:
        throwSyntax ("token", std::string(1, (char) ch));
    }
    return std::string();
}

//---------------------------------------------------------------------------

std::string JsonLoader::readValue ()
{
    int nch = whatsNext();
    switch (nch)
    {
    case '\"':
        return readString ();
    case 'a':
        return readKeyword ();
    case '0':
        return readNumber ();
    default:
        throwSyntax ("value", std::string(1, (char) nch));
    }
    return std::string();
}

//---------------------------------------------------------------------------

unsigned int JsonLoader::readHexCode ()
{
    unsigned int uCode = 0;
    for (int i = 0; i < 4; i++)
    {
        int ch = getChar(true);
        int n = 0;
        if (common::isDigit(ch))
            n = ch - '0';
        else
        if (ch >= 'a' && ch <= 'f')
            n = ch - 'a';
        else
        if (ch >= 'A' && ch <= 'F')
            n = ch - 'F';
        else
            throwSyntax ("hex unicode", std::string(1, (char) ch));
        uCode = (uCode << 4) + n;
    }
    return uCode;
}

//---------------------------------------------------------------------------

std::string JsonLoader::utf32ToString (unsigned int /*u32Code*/)
{
    return std::string("?");
}

//---------------------------------------------------------------------------

std::string JsonLoader::readString ()
{
    int ch;
    while (common::isWhitespace(ch = getChar())) {}
    if (ch != '\"')
        throwSyntax ("string", std::string(1, (char) ch));
    std::string res ("\"");
    while (stream->good())
    {
        ch = getChar(true);
        if (ch == '\\')
        {
            ch = getChar (true);
            switch (ch)
            {
            case '\"':
            case '\\':
            case '/':
                res += ch;
                break;
            case 'b':
                res += '\b';
                break;
            case 'f':
                res += '\f';
                break;
            case 'n':
                res += '\n';
                break;
            case 'r':
                res += '\r';
                break;
            case 't':
                res += '\t';
                break;
            case 'u':
                res += utf32ToString(readHexCode());
                break;
            default:
                throwString (std::string("Invalid escape character (") + std::string(1, (char) ch) + ") inside string");
            };
        }
        else
            res += ch;
        if (ch == '\"')
            break;
    }
    if (!stream->good())
        throwString (std::string("Error reading string"));
    return res;
}

//---------------------------------------------------------------------------

std::string JsonLoader::readNumber ()
{
    std::string res;
    bool wasDigit = true;
    bool waitSign = true;
    bool waitDec = true;
    bool waitExpSign = false;
    bool waitExp = true;
    while (true)
    {
        int ch = getChar();
        if (waitSign && ch == '-')
            waitSign = false;
        else
        if (waitDec && ch == '.')
        {
            waitSign = false;
            waitDec = false;
        }
        else
        if (waitExp && (ch == 'e' || ch == 'E'))
        {
            waitSign = false;
            waitDec = false;
            waitExp = false;
            int nch = peekChar();
            waitExpSign = (nch == '-' || nch == '+');
        }
        else
        if (waitExpSign && (ch == '-' || ch == '+'))
        {
            waitSign = false;
            waitDec = false;
            waitExpSign = false;
        }
        else
        if (common::isDigit(ch))
            wasDigit = true;
        else
        {
            throwSyntax ("number", res);
        }
        res += ch;
        ch = peekChar ();
        if (common::isWhitespace(ch) || ch == ',' || ch == '}' || ch == ']')
        {
            return res;
        }
    };
    (void)wasDigit;
}

//---------------------------------------------------------------------------

std::string JsonLoader::readKeyword ()
{
    int ch;
    std::string res;
    if (strict)
    {
        while (common::isAlpha(ch = peekChar()))
        {
            res += ch;
            getChar();
        }
        if (!common::isKeyword(res))
            throwSyntax ("keyword", res);
    }
    else
    {
        while (common::isQuotelessStringChar(ch = peekChar()))
        {
            res += ch;
            getChar();
        }
    }
    return res;
}

//---------------------------------------------------------------------------

object::JsonObject *JsonLoader::objectByTag (std::string tag)
{
    if (tag.length() < 1 || tag[0] != '{')
        return 0;
    std::istringstream is (tag.substr(1, tag.length() - 1));
    int n;
    is >> n;
    if (n < 0 || n >= (int) objs.size())
        return 0;
    return objs[n];
}

//---------------------------------------------------------------------------

array::JsonArray *JsonLoader::arrayByTag (std::string tag)
{
    if (tag.length() < 1 || tag[0] != '[')
        return 0;
    std::istringstream is (tag.substr(1, tag.length() - 1));
    int n;
    is >> n;
    if (n < 0 || n >= (int) arrs.size())
        return 0;
    return arrs[n];
}
