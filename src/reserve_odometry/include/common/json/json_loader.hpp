#ifndef SYSTEM_JSON_LOADER_HPP
#define SYSTEM_JSON_LOADER_HPP

#include <string>
#include <map>
#include <vector>
#include <iostream>

namespace sys {
namespace json {
namespace object { class JsonObject; }
namespace array { class JsonArray; }

namespace loader {
/*
Class for parsing JSON file or string

Usage example:
======================
    MyData data;
    try
    {
        JsonLoader jsl("filename.json");
        data.fromJson (jsl.rootObject());
        std::cout << "MyData config file loaded successfully." << std::endl;
    }
    catch (std::string ex)
    {
        std::cerr << "Error loading MyData config file." << std::endl;
        std::cerr << ex << std::endl;
        data.clear ();
    }
======================
*/
class JsonLoader {
    friend class object::JsonObject;
    friend class array::JsonArray;
public:
    enum
    {
        // Source for string constructor
        Auto = 0,
        String = 0x1,
        File = 0x2,
        SourceMask = 0x3,

        // Strictness
        Strict = 0x4,
        StrictComments = 0xc,
        NotStrict = 0x0,
        StrictnessMask = 0xc
    };

    JsonLoader (const std::string &str, int flags = Auto | NotStrict);
    JsonLoader (std::istream *aStream,  int flags = NotStrict);
    ~JsonLoader ();

    inline object::JsonObject *rootObject () { return obj; }
    inline array::JsonArray *rootArray () { return arr; }

    inline std::string input_line() const { return inputLine; }

private:
    std::istream *stream;
    object::JsonObject *obj;
    array::JsonArray *arr;
    std::vector<object::JsonObject *> objs;
    std::vector<array::JsonArray *> arrs;
    int line;
    int offset;
    std::string inputLine;
    bool strict;
    bool comments;

    void throwString (std::string message);
    void throwSyntax (std::string expected, std::string got);
    int peekChar ();
    int getChar (bool raw = false);
    void parse ();
    void skipWhitespace ();
    void skipToNewline ();
    int whatsNext ();
    std::string read ();
    std::string readToken ();
    std::string readValue ();
    std::string readString ();
    std::string readNumber ();
    std::string readKeyword ();
    unsigned int readHexCode ();
    std::string utf32ToString (unsigned int u32Code);
    object::JsonObject *objectByTag (std::string tag);
    array::JsonArray *arrayByTag (std::string tag);
};
}
}
}

#endif
