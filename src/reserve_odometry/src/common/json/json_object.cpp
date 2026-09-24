#include "../../../../include/detection/common/json/json_object.hpp"
#include "../../../../include/detection/common/json/json_array.hpp"
#include "../../../../include/detection/common/json/json_output.hpp"
#include "../../../../include/detection/common/json/json_common.hpp"

using namespace sys;
using namespace json;
using namespace object;

JsonObject::JsonObject (loader::JsonLoader *aLoader)
{
    loader = aLoader;
    if (!loader)
        throw std::string ("Must not construct JsonObject with null loader");
    loaderId = loader->objs.size();
    loader->objs.push_back (this);

    while (true)
    {
        std::string key = loader->read ();
        if (!loader->strict)
        {
            if (key == "}")
                break;
            else
            if (key == ",")
                key = loader->read ();
        }

        if (loader->strict)
        {
            if (!common::isString(key))
                loader->throwSyntax("string (key name)", key);
            key = common::stripString (key);
        }
        else
        {
            if (common::isString(key))
                key = common::stripString (key);
            else
            if (key == ":" || key == "," || key == "="
                || key == "{" || key == "}"
                || key == "[" || key == "]")
                loader->throwSyntax("string (key name)", key);
        }

        std::string val;
        if (loader->strict)
        {
            val = loader->readToken ();
            if (val != ":")
                loader->throwSyntax("\":\"", val);
            val = loader->read ();
        }
        else
        {
            val = loader->read ();
            if (val == ":" || val == "=")
                val = loader->read ();
        }

        if (val == "{")
        {
            val = (new object::JsonObject (loader))->tag();
        }
        else if (val == "[")
        {
            val = (new array::JsonArray (loader))->tag();
        }
        else if (!common::isValue(val))
            loader->throwSyntax("value (property)", val);
        addProperty (key, val);

        if (!loader->strict)
        {
            int ch = loader->whatsNext();
            if (ch != ',' && ch != '}')
                continue;
        }
        val = loader->read ();
        if (val == "}")
            break;
        if (val != ",")
            loader->throwSyntax("\",\" or \"}\"", val);
    }
}

//---------------------------------------------------------------------------

std::string JsonObject::tag () const
{
    return std::string("{") + common::i2str(loaderId);
}

//---------------------------------------------------------------------------

bool JsonObject::hasProperty (std::string key) const
{
    return properties.find(key) != properties.end();
}

//---------------------------------------------------------------------------

void JsonObject::ensureProperty (std::string key) const
{
    if (!hasProperty(key))
        throw std::string("No such object property: ") + key;
}

//---------------------------------------------------------------------------

void JsonObject::addProperty (std::string key, std::string val)
{
    if (hasProperty (key))
        throw std::string ("Duplicate property name for JSON object: ") + key;
    properties[key] = val;
}

//---------------------------------------------------------------------------

bool JsonObject::isString (std::string key) const
{
    if (!hasProperty (key))
        return false;
    return common::isString(property(key));
}

//---------------------------------------------------------------------------

bool JsonObject::isInteger (std::string key) const
{
    if (!hasProperty (key))
        return false;
    return common::isInteger(property(key));
}

//---------------------------------------------------------------------------

bool JsonObject::isReal (std::string key) const
{
    if (!hasProperty (key))
        return false;
    return common::isNumber(property(key));
}

//---------------------------------------------------------------------------

bool JsonObject::isBoolean (std::string key) const
{
    if (!hasProperty (key))
        return false;
    return property(key) == "true" || property(key) == "false";
}

//---------------------------------------------------------------------------

bool JsonObject::isNull (std::string key) const
{
    if (!hasProperty (key))
        return false;
    return property(key) == "null";
}

//---------------------------------------------------------------------------

bool JsonObject::isObject (std::string key) const
{
    if (!hasProperty (key))
        return false;
    return property(key)[0] == '{';
}

//---------------------------------------------------------------------------

bool JsonObject::isArray (std::string key) const
{
    if (!hasProperty (key))
        return false;
    return property(key)[0] == '[';
}

//---------------------------------------------------------------------------

std::string JsonObject::asString (std::string key, std::string fallback) const
{
    if (!hasProperty (key) || !common::isString(property(key)))
        return fallback;
    return common::stripString(property(key));
}

//---------------------------------------------------------------------------

int JsonObject::asInteger (std::string key, int fallback) const
{
    if (!hasProperty (key) || !common::isInteger(property(key)))
        return fallback;
    int res;
    std::istringstream is(property(key));
    is >> res;
    return res;
}

//---------------------------------------------------------------------------

double JsonObject::asReal (std::string key, double fallback) const
{
    if (!hasProperty (key) || !common::isNumber(property(key)))
        return fallback;
    double res;
    std::istringstream is(property(key));
    is >> res;
    return res;
}

//---------------------------------------------------------------------------

bool JsonObject::asBoolean (std::string key, bool fallback) const
{
    if (!hasProperty (key) || (property(key) != "true" && property(key) != "false"))
        return fallback;
    return (property(key) == "true");
}

//---------------------------------------------------------------------------

JsonObject *JsonObject::asObject (std::string key) const
{
    if (!hasProperty (key))
        return 0;
    return loader->objectByTag (property(key));
}

//---------------------------------------------------------------------------

array::JsonArray *JsonObject::asArray (std::string key) const
{
    if (!hasProperty (key))
        return 0;
    return loader->arrayByTag (property(key));
}

//---------------------------------------------------------------------------

std::string JsonObject::property (std::string key) const
{
    if (!hasProperty (key))
        return std::string();
    return properties.find(key)->second;
}

//---------------------------------------------------------------------------

std::string JsonObject::asString (std::string key) const
{
    ensureProperty (key);
    if (!common::isString(property(key)))
        throw std::string("Value is not a string. ") + key + ": " + property(key);
    return common::stripString(property(key));
}

//---------------------------------------------------------------------------

int JsonObject::asInteger (std::string key) const
{
    ensureProperty (key);
    if (!common::isInteger(property(key)))
        throw std::string("Value is not an integer. ") + key + ": " + property(key);
    int res;
    std::istringstream is(property(key));
    is >> res;
    return res;
}

//---------------------------------------------------------------------------

double JsonObject::asReal (std::string key) const
{
    ensureProperty (key);
    if (!common::isNumber(property(key)))
        throw std::string("Value is not a number. ") + key + ": " + property(key);
    double res;
    std::istringstream is(property(key));
    is >> res;
    return res;
}

//---------------------------------------------------------------------------

bool JsonObject::asBoolean (std::string key) const
{
    ensureProperty (key);
    if (property(key) != "true" && property(key) != "false")
        throw std::string("Value is not a boolean. ") + key + ": " + property(key);
    return (property(key) == "true");
}

//---------------------------------------------------------------------------

int JsonObject::size () const
{
    return properties.size();
}

//---------------------------------------------------------------------------

std::string JsonObject::keyAt (int index) const
{
    if (!(index >= 0 && index < size()))
        return std::string();
    std::map<std::string, std::string>::const_iterator it = properties.begin();
    for (int i = 0; i < index; i++)
        it++;
    return it->first;
}

//---------------------------------------------------------------------------

std::string JsonObject::toString (int identLevel, int identSize) const
{
    out::JsonStringOutput jso(identLevel, identSize);
    jso.startObject ();
    std::string endStr = identLevel < 0? "" : "\n";
    for (int i = 0; i < size(); i++)
    {
        std::string key = keyAt (i);
        JsonObject *obj = asObject (key);
        array::JsonArray *arr = asArray (key);
        if (obj)
            jso.rawProperty (key, endStr + obj->toString((identLevel < 0? identLevel : identLevel + 1), identSize));
        else
        if (arr)
            jso.rawProperty (key, endStr + arr->toString((identLevel < 0? identLevel : identLevel + 1), identSize));
        else
            jso.rawProperty (key, property (key));
    }
    jso.end();
    return jso.result();
}
