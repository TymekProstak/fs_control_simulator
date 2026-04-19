#ifndef CONELOOKUP_H
#define CONELOOKUP_H


#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include "dv_interfaces/Cones.h"
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>


#include "Cone.h"

// Class that aggregates cones in an optimal way for later lookup
class ConeLookup {
    std::vector<Cone> leftCones;
    std::vector<Cone> rightCones;

    // Split x/y for better cache locality
    std::vector<float> leftXs, leftYs;
    std::vector<float> rightXs, rightYs;

    std::vector<Cone> leftFinishCones;
    std::vector<Cone> rightFinishCones;

public:
    // Constructors convert any format to something more optimal for lookup
    explicit ConeLookup(pcl::PointCloud<pcl::PointXYZL> &cones_msg); // Slam cones
    explicit ConeLookup(const dv_interfaces::Cones &cones_msg); // Fake cones

    ConeLookup() = default;

    void InsertCones(const pcl::PointCloud<pcl::PointXYZL> &cones_msg);
    void InsertCones(const dv_interfaces::Cones &cones_msg);

    // Returns a pointer to internal structure, dont change them, pwetty pwease 🥺👉🏻👈🏻
    const Cone *LookupClosestLeftCone(Vec2 pos) const;
    const Cone *LookupClosestRightCone(Vec2 pos) const;

    static const Cone *LookupClosestCone(Vec2 pos, const std::vector<Cone>& cones, const std::vector<float>& xs, const std::vector<float>& ys);

    const std::vector<Cone>& GetLeftCones() const { return leftCones; };
    const std::vector<Cone>& GetRightCones() const { return rightCones; };

    const std::vector<Cone>& GetLeftFinishCones() const { return leftFinishCones; };
    const std::vector<Cone>& GetRightFinishCones() const { return rightFinishCones; };

private:
    template<class T_msg, class F>
    void InsertAnyCones(T_msg cones, F insertFunc, size_t size);

    void rebuildSoA();

    struct BoundingBox {
        float
                minX = std::numeric_limits<float>::max(),
                maxX = -std::numeric_limits<float>::max(),
                minY = std::numeric_limits<float>::max(),
                maxY = -std::numeric_limits<float>::max();
        // *I didn't even know you could format code like that
    };

    static void hilbertSort(std::vector<Cone> &cones, std::vector<float> &xs, std::vector<float> &ys, const BoundingBox &bb);
};


#endif //CONELOOKUP_H
