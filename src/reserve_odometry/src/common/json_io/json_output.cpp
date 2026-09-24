#include "utils/json_io/json_output.hpp"
#include "utils/json_io/json_common.hpp"

using namespace mps;
using namespace json;
using namespace out;

enum
{
    FirstValueMask = 1,
    FirstValue = 0,
    NotFirstValue = FirstValueMask,

    TypeMask = 2,
    TypeObject = 0,
    TypeArray = TypeMask
};

//---------------------------------------------------------------------------

JsonStringOutput::JsonStringOutput (int identLevel, int identSize)
{
    error = false;
    idSize = identSize;
    idLevel = identLevel < 0? 0 : identLevel;
    collapse = (identLevel < 0);
    idStr = collapse? std::string() : std::string(idLevel * idSize, ' ');
    endStr = collapse? std::string() : std::string("\n");
}

//---------------------------------------------------------------------------

void JsonStringOutput::putSep ()
{
    if (levels.size() > 0)
    {
        bool sep = levels[levels.size()-1] & FirstValueMask;
        if (sep)
            res += ", ";
        res += endStr;
        levels[levels.size()-1] |= NotFirstValue;
    }
}
//---------------------------------------------------------------------------

bool JsonStringOutput::checkLevelError()
{
    error = error || (levels.size() < 1);
    return error;
}

//---------------------------------------------------------------------------

bool JsonStringOutput::checkObjectError()
{
    error = error || ((levels[levels.size()-1] & TypeMask) != TypeObject);
    return error;
}

//---------------------------------------------------------------------------

bool JsonStringOutput::checkArrayError()
{
    error = error || ((levels[levels.size()-1] & TypeMask) != TypeArray);
    return error;
}

//---------------------------------------------------------------------------

void JsonStringOutput::setCollapse (bool clps)
{
    collapse = clps;
    idStr = collapse? std::string() : std::string(idLevel * idSize, ' ');
    endStr = collapse? std::string() : std::string("\n");
}

//---------------------------------------------------------------------------

void JsonStringOutput::startObject (std::string key)
{
    putSep ();

    if (key.length() > 0)
    {
        checkObjectError();
        res += idStr + std::string("\"") + key + "\": " + endStr;
    }

    res += idStr + "{ ";

    levels.resize (levels.size() + 1);
    levels[levels.size()-1] = TypeObject | FirstValue; // object, no properties yet

    idLevel++;
    idStr = collapse? std::string() : std::string(idLevel * idSize, ' ');
}

//---------------------------------------------------------------------------

void JsonStringOutput::startArray (std::string key)
{
    putSep ();

    if (key.length() > 0)
    {
        checkObjectError();
        res += idStr + std::string("\"") + key + "\": " + endStr;
    }

    res += idStr + "[ ";

    levels.resize (levels.size() + 1);
    levels[levels.size()-1] = TypeArray | FirstValue; // array, no values yet

    idLevel++;
    idStr = collapse? std::string() : std::string(idLevel * idSize, ' ');
}

//---------------------------------------------------------------------------

void JsonStringOutput::end ()
{
    checkLevelError ();

    res += endStr;

    idLevel--;
    idStr = collapse? std::string() : std::string(idLevel * idSize, ' ');

    bool isObj = (levels[levels.size()-1] & TypeMask) == TypeObject;
    res += idStr + (collapse? " " : "") + (isObj? "}" : "]");

    levels.resize(levels.size()-1);
}

//---------------------------------------------------------------------------

void JsonStringOutput::integerProperty (std::string key, int value)
{
    checkLevelError ();
    checkObjectError ();

    putSep ();
    res += idStr + std::string("\"") + key + "\": " + common::i2str(value);
}

//---------------------------------------------------------------------------

void JsonStringOutput::realProperty (std::string key, double value, bool scient, int precision)
{
    checkLevelError ();
    checkObjectError ();

    putSep ();
    std::ostringstream s;
    if (scient)
    {
        s.setf(std::ios_base::scientific, std::ios_base::floatfield);
        s.precision(precision);
    }
    else
    {
        s.setf(std::ios::fixed);
        s.precision(precision);
    }
    s << value;
    res += idStr + std::string("\"") + key + "\": " + s.str();
}

//---------------------------------------------------------------------------

void JsonStringOutput::stringProperty (std::string key, std::string value)
{
    checkLevelError ();
    checkObjectError ();

    putSep ();
    res += idStr + std::string("\"") + key + "\": \"" + value + "\"";
}

//---------------------------------------------------------------------------

void JsonStringOutput::booleanProperty (std::string key, bool value)
{
    checkLevelError ();
    checkObjectError ();

    putSep ();
    res += idStr + std::string("\"") + key + "\": " + (value? "true" : "false");
}

//---------------------------------------------------------------------------

void JsonStringOutput::nullProperty (std::string key)
{
    checkLevelError ();
    checkObjectError ();

    putSep ();
    res += idStr + std::string("\"") + key + "\": null";
}

//---------------------------------------------------------------------------

void JsonStringOutput::rawProperty (std::string key, std::string value)
{
    checkLevelError ();
    checkObjectError ();

    putSep ();
    res += idStr + std::string("\"") + key + "\": " + value;
}

//---------------------------------------------------------------------------

void JsonStringOutput::integerValue (int value)
{
    checkLevelError ();
    checkArrayError ();

    putSep ();
    res += idStr + common::i2str(value);
}

//---------------------------------------------------------------------------

void JsonStringOutput::realValue (double value)
{
    checkLevelError ();
    checkArrayError ();

    putSep ();
    std::ostringstream s;
    s.setf(std::ios::fixed);
    s << value;
    res += idStr + s.str();
}

//---------------------------------------------------------------------------

void JsonStringOutput::stringValue (std::string value)
{
    checkLevelError ();
    checkArrayError ();

    putSep ();
    res += idStr + "\"" + value + "\"";
}

//---------------------------------------------------------------------------

void JsonStringOutput::booleanValue (bool value)
{
    checkLevelError ();
    checkArrayError ();

    putSep ();
    res += idStr + (value? "true" : "false");
}

//---------------------------------------------------------------------------

void JsonStringOutput::nullValue ()
{
    checkLevelError ();
    checkArrayError ();

    putSep ();
    res += idStr + "null";
}

//---------------------------------------------------------------------------

void JsonStringOutput::rawValue (std::string value)
{
    checkLevelError ();
    checkArrayError ();

    putSep ();
    res += idStr + value;
}

//---------------------------------------------------------------------------

std::string &JsonStringOutput::result ()
{
    return res;
}

//---------------------------------------------------------------------------

bool JsonStringOutput::ok ()
{
    return !error;
}
