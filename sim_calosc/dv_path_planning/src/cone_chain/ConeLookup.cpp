#include "../../include/cone_chain/ConeLookup.h"

ConeType getTypeFromName(const std::string &class_name);

inline Cone Convert2Cone(const pcl::PointXYZL& msg) {
    return Cone(static_cast<ConeType>(msg.label), {msg.x, msg.y});
}

inline Cone Convert2Cone(const dv_interfaces::Cone_<std::allocator<void>> & msg) {
    return Cone(getTypeFromName(msg.class_name), {msg.x, msg.y});
}

ConeLookup::ConeLookup(pcl::PointCloud<pcl::PointXYZL> &cones_msg) {
    leftCones.clear();
    rightCones.clear();

    InsertCones(cones_msg);
}

ConeLookup::ConeLookup(const dv_interfaces::Cones &cones_msg) {
    leftCones.clear();
    rightCones.clear();

    InsertCones(cones_msg);
}

void ConeLookup::InsertCones(const pcl::PointCloud<pcl::PointXYZL> &cones_msg) {
    InsertAnyCones<const pcl::PointCloud<pcl::PointXYZL>, Cone
        (const pcl::PointXYZL &)>(cones_msg, Convert2Cone, cones_msg.points.size());
}

void ConeLookup::InsertCones(const dv_interfaces::Cones &cones_msg) {
    InsertAnyCones<const std::vector<dv_interfaces::Cone_<std::allocator<void> > >, Cone (
        const dv_interfaces::Cone_<std::allocator<void> > &)>(cones_msg.cones, Convert2Cone, cones_msg.cones.size());
}

inline uint32_t Hilbert2D(uint32_t x, uint32_t y)
{
    // Standard 16-bit Hilbert curve
    uint32_t h = 0;
    for (uint32_t s = 1u << 15; s; s >>= 1)
    {
        uint32_t rx = (x & s) != 0;
        uint32_t ry = (y & s) != 0;
        h += s * s * ((3 * rx) ^ ry);

        if (!ry) {
            if (rx) {
                x = ((1u << 16) - 1) - x;
                y = ((1u << 16) - 1) - y;
            }
            std::swap(x, y);
        }
    }
    return h;
}

template<typename T_msg, typename F>
void ConeLookup::InsertAnyCones(T_msg cones, F insertFunc, size_t size) {
    std::vector<Cone> orangeCones; // if any
    BoundingBox leftBB, rightBB;

    leftCones.reserve(leftCones.size() + size / 2);
    rightCones.reserve(rightCones.size() + size / 2);
    for (int i = 0; i < size; i++) {
        const Cone cone = insertFunc(cones[i]);
        if (cone.getType() == BLUE_LEFT)
            leftCones.push_back(cone);
        else if (cone.getType() == YELLOW_RIGHT)
            rightCones.push_back(cone);
        else if (cone.getType() == ORANGE_BIG || cone.getType() == ORANGE_SMALL)
            orangeCones.push_back(cone);
        else
            continue;

        leftBB.minX = std::min(leftBB.minX, cone.getPosition().x);
        leftBB.maxX = std::max(leftBB.maxX, cone.getPosition().x);
        leftBB.minY = std::min(leftBB.minY, cone.getPosition().y);
        leftBB.maxY = std::max(leftBB.maxY, cone.getPosition().y);

        rightBB.minX = std::min(rightBB.minX, cone.getPosition().x);
        rightBB.maxX = std::max(rightBB.maxX, cone.getPosition().x);
        rightBB.minY = std::min(rightBB.minY, cone.getPosition().y);
        rightBB.maxY = std::max(rightBB.maxY, cone.getPosition().y);
    }

    // Assign orange cones to the correct sides (requires the lookup to work (but can for now be naive))
    for (auto orangeCone : orangeCones) {
        if ((LookupClosestLeftCone(orangeCone.getPosition())->getPosition() - orangeCone.getPosition()).length_squared() <
            (LookupClosestRightCone(orangeCone.getPosition())->getPosition() - orangeCone.getPosition()).length_squared()) {
            if (orangeCone.getType() == ORANGE_BIG)
                leftFinishCones.push_back(orangeCone);
            leftCones.push_back(orangeCone);
            }
        else {
            if (orangeCone.getType() == ORANGE_BIG)
                rightFinishCones.push_back(orangeCone);
            rightCones.push_back(orangeCone);
        }
    }

    hilbertSort(leftCones, leftXs, leftYs, leftBB);
    hilbertSort(rightCones, rightXs, rightYs, rightBB);
    //rebuildSoA();
}

void ConeLookup::hilbertSort(std::vector<Cone> &cones, std::vector<float>& xs, std::vector<float>& ys, const BoundingBox &bb) {
    if (cones.empty())
        return;

    const float range = std::max(bb.maxX - bb.minX, bb.maxY - bb.minY);
    const float scale = 65535.0f / range;

    struct HC {
        uint32_t key;
        Cone cone;
    };
    std::vector<HC> tmp(cones.size());
    std::generate(tmp.begin(), tmp.end(), [i = 0, cones, scale, bb]() mutable {
        const Cone& c = cones[i++];
        const Vec2 p = c.getPosition();
        const uint32_t ix = static_cast<uint32_t>((p.x - bb.minX) * scale);
        const uint32_t iy = static_cast<uint32_t>((p.y - bb.minY) * scale);
        return HC{Hilbert2D(ix, iy), c};
    });

    std::sort(tmp.begin(), tmp.end(), [](auto &a, auto &b) {
        return a.key < b.key;
    });

    cones.clear();
    cones.reserve(tmp.size());
    xs.resize(tmp.size());
    ys.resize(tmp.size());
    for (int i = 0; i < tmp.size(); i++) {
        auto cone = tmp[i].cone;
        cones.push_back(cone);
        const Vec2 p = cone.getPosition();
        xs[i] = p.x;
        ys[i] = p.y;
    }
}

void ConeLookup::rebuildSoA() {
    // Left side
    leftXs.resize(leftCones.size());
    leftYs.resize(leftCones.size());
    for (size_t i = 0; i < leftCones.size(); i++) {
        const Vec2 p = leftCones[i].getPosition();
        leftXs[i] = p.x;
        leftYs[i] = p.y;
    }

    // Right side
    rightXs.resize(rightCones.size());
    rightYs.resize(rightCones.size());
    for (size_t i = 0; i < rightCones.size(); i++) {
        const Vec2 p = rightCones[i].getPosition();
        rightXs[i] = p.x;
        rightYs[i] = p.y;
    }
}

const Cone *ConeLookup::LookupClosestLeftCone(const Vec2 pos) const {
    const float px = pos.x;
    const float py = pos.y;

    float bestDist = std::numeric_limits<float>::max();
    size_t bestIdx = 0;

    const size_t N = leftXs.size();

    for (size_t i = 0; i < N; i++) {
        const float dx = leftXs[i] - px;
        const float dy = leftYs[i] - py;

        if (const float d2 = dx * dx + dy * dy; d2 < bestDist) {
            bestDist = d2;
            bestIdx = i;
        }
    }

    return &leftCones[bestIdx];

    // STL version (slower)
    // auto it = std::min_element(leftCones.begin(), leftCones.end(),
    //     [&pos](const Cone& a, const Cone& b) {
    //         float distA = (pos - a.getPosition()).length_squared();
    //         float distB = (pos - b.getPosition()).length_squared();
    //         return distA < distB;  // Compare distances
    //     });
    // if (it != leftCones.end())
    //     return &*it;
    // return nullptr;
}

const Cone *ConeLookup::LookupClosestRightCone(const Vec2 pos) const {
    const float px = pos.x;
    const float py = pos.y;

    float bestDist = std::numeric_limits<float>::max();
    size_t bestIdx = 0;

    const size_t N = rightXs.size();

    for (size_t i = 0; i < N; i++) {
        const float dx = rightXs[i] - px;
        const float dy = rightYs[i] - py;

        if (const float d2 = dx * dx + dy * dy; d2 < bestDist) {
            bestDist = d2;
            bestIdx = i;
        }
    }

    return &rightCones[bestIdx];

    // STL version (slower)
    // auto it = std::min_element(rightCones.begin(), rightCones.end(),
    // [&pos](const Cone& a, const Cone& b) {
    //     float distA = (pos - a.getPosition()).length_squared();
    //     float distB = (pos - b.getPosition()).length_squared();
    //     return distA < distB;  // Compare distances
    // });
    // if (it != rightCones.end())
    //     return &*it;
    // return nullptr;
}

inline ConeType getTypeFromName(const std::string &class_name) {
    if (class_name == "blue")
    {
        return ConeType::BLUE_LEFT;
    }
    if (class_name == "yellow")
    {
        return ConeType::YELLOW_RIGHT;
    }
    if (class_name == "orange_big")
    {
        return ConeType::ORANGE_BIG;
    }
    if (class_name == "orange_small")
    {
        return ConeType::ORANGE_SMALL;
    }
    return ConeType::UNDEFINED;
}
