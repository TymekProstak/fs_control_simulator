#ifndef CONE_DESCRIPTOR_H
#define CONE_DESCRIPTOR_H

#include "../common/Vec2.h"
#include "ConeType.h"

struct Cone
{
    Cone() {}
    Cone(ConeType type, Vec2 vec) : type(type), position(vec) {}
    Cone(ConeType type, float x, float y) : type(type), position(x, y) {}
    Cone(ConeType type) : type(type), position() {}

    ConeType getType() const { return type; }
    Vec2 getPosition() const { return position; }

private:
    ConeType type = ConeType::UNDEFINED;
    Vec2 position;
};

bool operator==(const Cone &cone1, const Cone &cone2);

struct ConeHashFunction
{
    size_t operator()(const Cone &cone) const
    {
        size_t xHash = std::hash<int>()(cone.getPosition().x);
        size_t yHash = std::hash<int>()(cone.getPosition().y) << 1;
        return xHash ^ yHash;
    }
};
#endif