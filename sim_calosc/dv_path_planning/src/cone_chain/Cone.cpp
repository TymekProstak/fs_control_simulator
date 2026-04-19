#include "../../include/cone_chain/Cone.h"

bool operator==(const Cone &cone1, const Cone &cone2)
{
    return cone1.getPosition() == cone2.getPosition() && cone1.getType() == cone2.getType();
}