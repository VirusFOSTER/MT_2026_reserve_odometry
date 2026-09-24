#ifndef SYSTEM_JSON_OUTPUT_HPP
#define SYSTEM_JSON_OUTPUT_HPP

#include <cstring>
#include <iostream>
#include <vector>

namespace sys {
namespace json {
namespace out {
/*
Utility class for creating strings in JSON format

Usage example:
======================
    // use identation from level 0 (top level),
    // with 2 spaces identation size
    JsonStringOutput jso(0, 2);

    jso.startObject ();
    jso.integerProperty("number", 123);
    jso.stringProperty("name", "foo");

    jso.startArray ("values");
    jso.setCollapse (true);     // let the array collapse into one line
    jso.realValue(0.5);
    jso.realValue(1.3);
    jso.end(); // array "values"
    jso.setCollapse (false);    // turn collapsing off, after array is finished

    jso.end(); // top level object

    if (!jso.ok())
        throw std::string("Erroneous use of JsonStringOutput class");

    return jso.result ();
======================
*/
class JsonStringOutput {
public:
    JsonStringOutput (int identLevel = -1, int identSize = 4);

    void setCollapse (bool clps);

    void startObject (std::string key = std::string());
    void startArray (std::string key = std::string());
    void end ();

    void integerProperty (std::string key, int value);
    void realProperty (std::string key, double value, bool scient = false, int precision = 5);
    void stringProperty (std::string key, std::string value);
    void booleanProperty (std::string key, bool value);
    void nullProperty (std::string key);
    void rawProperty (std::string key, std::string value);


    void integerValue (int value);
    void realValue (double value);
    void stringValue (std::string value);
    void booleanValue (bool value);
    void nullValue ();
    void rawValue (std::string value);

    std::string &result ();
    bool ok ();
    std::string res;
private:
    void putSep ();
    bool checkLevelError();
    bool checkObjectError();
    bool checkArrayError();

    std::string idStr;
    std::string endStr;
    std::vector<char> levels;
    bool collapse;
    bool error;
    int idLevel;
    int idSize;
};
}
}
}

#endif
