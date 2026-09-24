#include "utils/json_io/json_array.hpp"
#include "utils/json_io/json_object.hpp"
#include "utils/json_io/json_output.hpp"
#include "utils/json_io/json_common.hpp"

using namespace mps;
using namespace json;
using namespace array;

JsonArray::JsonArray (loader::JsonLoader *aLoader)
{
    loader = aLoader;
    if (!loader)
        throw std::string ("Must not construct JsonArray with null loader");
    loaderId = loader->arrs.size();
    loader->arrs.push_back (this);

    std::string val;
    while (true)
    {
        std::string val = loader->read ();
        if (!loader->strict)
        {
            if (val == "]")
                break;
            else
            if (val == ",")
                val = loader->read ();
        }
        if (val == "{")
        {
            val = (new object::JsonObject (loader))->tag();
        }
        else
        if (val == "[")
        {
            val = (new JsonArray (loader))->tag();
        }
        else
        if (!common::isValue(val))
            loader->throwSyntax("value (element)", val);
        addValue (val);

        if (!loader->strict)
        {
            int ch = loader->whatsNext();
            if (ch != ',' && ch != ']')
                continue;
        }
        val = loader->read ();
        if (val == "]")
            break;
        if (val != ",")
            loader->throwSyntax("\",\" or \"]\"", val);
    }
}

//---------------------------------------------------------------------------

std::string JsonArray::tag () const
{
    return std::string("[") + common::i2str(loaderId);
}

//---------------------------------------------------------------------------

void JsonArray::addValue (std::string val)
{
    values.push_back (val);
}

//---------------------------------------------------------------------------

bool JsonArray::isIndexValid (int index) const
{
    return (index >= 0 && index < (int) values.size());
}

//---------------------------------------------------------------------------

void JsonArray::ensureIndex (int index) const
{
    if (!isIndexValid (index))
        throw std::string("Array index is out of range ") + common::i2str(index) + "/" + common::i2str (values.size());
}

//---------------------------------------------------------------------------

int JsonArray::size () const
{
    return values.size();
}

//---------------------------------------------------------------------------

bool JsonArray::isString (int index) const
{
    if (!isIndexValid (index))
        return false;
    return common::isString(values.at(index));
}

//---------------------------------------------------------------------------

bool JsonArray::isInteger (int index) const
{
    if (!isIndexValid (index))
        return false;
    return common::isInteger(values.at(index));
}

//---------------------------------------------------------------------------

bool JsonArray::isReal (int index) const
{
    if (!isIndexValid (index))
        return false;
    return common::isNumber(values.at(index));
}

//---------------------------------------------------------------------------

bool JsonArray::isBoolean (int index) const
{
    if (!isIndexValid (index))
        return false;
    return values.at(index) == "true" || values.at(index) == "false";
}

//---------------------------------------------------------------------------

bool JsonArray::isNull (int index) const
{
    if (!isIndexValid (index))
        return false;
    return values.at(index) == "null";
}

//---------------------------------------------------------------------------

bool JsonArray::isObject (int index) const
{
    if (!isIndexValid (index))
        return false;
    return values.at(index)[0] == '{';
}

//---------------------------------------------------------------------------

bool JsonArray::isArray (int index) const
{
    if (!isIndexValid (index))
        return false;
    return values.at(index)[0] == '[';
}

//---------------------------------------------------------------------------

std::string JsonArray::asString (int index, std::string fallback) const
{
    if (!isIndexValid (index) || !common::isString(values.at(index)))
        return fallback;
    return common::stripString(values.at(index));
}

//---------------------------------------------------------------------------

int JsonArray::asInteger (int index, int fallback) const
{
    if (!isIndexValid (index) || !common::isInteger(values.at(index)))
        return fallback;
    int res;
    std::istringstream is(values.at(index));
    is >> res;
    return res;
}

//---------------------------------------------------------------------------

double JsonArray::asReal (int index, double fallback) const
{
    if (!isIndexValid (index) || !common::isNumber(values.at(index)))
        return fallback;
    double res;
    std::istringstream is(values.at(index));
    is >> res;
    return res;
}

//---------------------------------------------------------------------------

bool JsonArray::asBoolean (int index, bool fallback) const
{
    if (!isIndexValid (index) || (values.at(index) != "true" && values.at(index) != "false"))
        return fallback;
    return (values.at(index) == "true");
}

//---------------------------------------------------------------------------

object::JsonObject *JsonArray::asObject (int index) const
{
    if (!isIndexValid (index))
        return 0;
    return loader->objectByTag (values.at(index));
}

//---------------------------------------------------------------------------

JsonArray *JsonArray::asArray (int index) const
{
    if (!isIndexValid (index))
        return 0;
    return loader->arrayByTag (values.at(index));
}

//---------------------------------------------------------------------------

std::string JsonArray::value (int index) const
{
    if (!isIndexValid (index))
        return std::string();
    return values.at(index);
}

//---------------------------------------------------------------------------

std::string JsonArray::asString (int index) const
{
    ensureIndex (index);
    if (!common::isString(values.at(index)))
        throw std::string("Value is not a string. [") + common::i2str(index) + "]:" + values.at(index);
    return common::stripString(values.at(index));
}

//---------------------------------------------------------------------------

int JsonArray::asInteger (int index) const
{
    ensureIndex (index);
    if (!common::isInteger(values.at(index)))
        throw std::string("Value is not an integer. [") + common::i2str(index) + "]:" + values.at(index);
    int res;
    std::istringstream is(values.at(index));
    is >> res;
    return res;
}

//---------------------------------------------------------------------------

double JsonArray::asReal (int index) const
{
    ensureIndex (index);
    if (!common::isNumber(values.at(index)))
        throw std::string("Value is not a number. [") + common::i2str(index) + "]:" + values.at(index);
    double res;
    std::istringstream is(values.at(index));
    is >> res;
    return res;
}

//---------------------------------------------------------------------------

bool JsonArray::asBoolean (int index) const
{
    ensureIndex (index);
    if (values.at(index) != "true" && values.at(index) != "false")
        throw std::string("Value is not a boolean. [") + common::i2str(index) + "]:" + values.at(index);
    return (values.at(index) == "true");
}

//---------------------------------------------------------------------------

std::string JsonArray::toString (int identLevel, int identSize) const
{
    out::JsonStringOutput jso(identLevel, identSize);
    jso.startArray ();
    std::string endStr = identLevel < 0? "" : "\n";
    for (int i = 0; i < size(); i++)
    {
        object::JsonObject *obj = asObject (i);
        JsonArray *arr = asArray (i);
        if (obj)
            jso.rawValue (endStr + obj->toString((identLevel < 0? identLevel : identLevel + 1), identSize));
        else
        if (arr)
            jso.rawValue (endStr + arr->toString((identLevel < 0? identLevel : identLevel + 1), identSize));
        else
            jso.rawValue (value (i));
    }
    jso.end();
    return jso.result();
}
