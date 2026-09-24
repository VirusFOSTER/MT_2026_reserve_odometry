#ifndef SYSTEM_JSON_ARRAY_HPP
#define SYSTEM_JSON_ARRAY_HPP

#include "json_loader.hpp"

namespace sys {
namespace json {
namespace array {
/*
Usage example:
======================
void MyData::fromJson(JsonObject *obj)
{
    this.number = obj->asInteger ("number", 0);
    this.name = obj->asString ("name", "<unnamed>");
    JsonArray *arr = obj->asArray ("values");
    if (!arr)
        throw std::string ("array 'values' expected");
    for (int i = 0; i < arr->size(); i++)
        this.values.push_back(arr->asReal(i, 0.0));
}
======================
*/
class JsonArray {
    friend class loader::JsonLoader;
    friend class object::JsonObject;
public:
    JsonArray (loader::JsonLoader *aLoader);

    // v-- no throw
    int size () const;
    bool isString (int index) const;
    bool isInteger (int index) const;
    bool isReal (int index) const;
    bool isBoolean (int index) const;
    bool isNull (int index) const;
    bool isObject (int index) const;
    bool isArray (int index) const;
    std::string asString (int index, std::string fallback) const;
    int asInteger (int index, int fallback) const;
    double asReal (int index, double fallback) const;
    bool asBoolean (int index, bool fallback) const;
    object::JsonObject *asObject (int index) const;
    JsonArray *asArray (int index) const;

    // return raw string value
    std::string value (int index) const;

    // form JSON output string from array
    std::string toString (int identLevel = -1, int identSize = 4) const;
    // ^--

    // v-- throws std::string
    std::string asString (int index) const;
    int asInteger (int index) const;
    double asReal (int index) const;
    bool asBoolean (int index) const;
    // ^--
private:
    loader::JsonLoader *loader;
    int loaderId;
    std::vector<std::string> values;

    std::string tag () const;
    void addValue (std::string val);
    void ensureIndex (int index) const;
    bool isIndexValid (int index) const;
};
}
}
}

#endif
