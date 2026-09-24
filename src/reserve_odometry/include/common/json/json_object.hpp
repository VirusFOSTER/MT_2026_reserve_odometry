#ifndef SYSTEM_JSON_OBJECT_HPP
#define SYSTEM_JSON_OBJECT_HPP

#include "json_loader.hpp"

namespace sys{
namespace json {
namespace object {
/*
Class, representing JSON object

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
class JsonObject {
    friend class loader::JsonLoader;
    friend class array::JsonArray;
public:
    JsonObject (loader::JsonLoader *aLoader);

    // v-- no throw
    bool hasProperty (std::string key) const;
    bool isString (std::string key) const;
    bool isInteger (std::string key) const;
    bool isReal (std::string key) const;
    bool isBoolean (std::string key) const;
    bool isNull (std::string key) const;
    bool isObject (std::string key) const;
    bool isArray (std::string key) const;
    std::string asString (std::string key, std::string fallback) const;
    int asInteger (std::string key, int fallback) const;
    double asReal (std::string key, double fallback) const;
    bool asBoolean (std::string key, bool fallback) const;
    JsonObject *asObject (std::string key) const;
    array::JsonArray *asArray (std::string key) const;

    // return raw string value
    std::string property (std::string key) const;

    // return keys count (number of indeces)
    int size () const;

    // return key at given index
    std::string keyAt (int index) const;

    // form JSON output string from object
    std::string toString (int identLevel = -1, int identSize = 4) const;
    // ^--

    // v-- throws std::string
    std::string asString (std::string key) const;
    int asInteger (std::string key) const;
    double asReal (std::string key) const;
    bool asBoolean (std::string key) const;
    // ^--
private:
    loader::JsonLoader *loader;
    int loaderId;
    std::map<std::string, std::string> properties;

    void ensureProperty (std::string key) const;
    std::string tag () const;
    void addProperty (std::string key, std::string val);
};
}
}
}

#endif
