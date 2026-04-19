#ifndef TRACKDRIVE_CONE_CHAIN_CREATOR_H
#define TRACKDRIVE_CONE_CHAIN_CREATOR_H

#include "ConeChainCreator.h"

class TrackdriveConeChainCreator : public ConeChainCreator
{
public:
    /**
     * Create cone chains and save them in leftConeChain and rightConeChain.
     */
    void createLeftAndRightConeChains(BolideDescriptor &bolide) override;

    /**
     * Update cone chain on one side based on current detections and bolide position.
     */
    void updateChain(BolideDescriptor &bolide, ConeChain &chain) override;

private:
    /**
     * Update the costs of cones in candidates based on the current chain.
     */
    void updateCosts(conesPriorityQueue &candidates,
                     ConeChain &coneChain,
                     BolideDescriptor &bolide);

    /**
     * Create rating instances of given cones and update them to candidates queue.
     */
    void appendConesToCandidates(conesPriorityQueue &candidates,
                                 conesMap &cones,
                                 ConeChain &chain,
                                 BolideDescriptor bolide);

    /**
     * Perform linear interpolation on cones in cone chain.
     */
    void interpolateCones(ConeChain &chain);

    /**
     * When the lengths of chains on both sides differ significantly, extrapolate the shorter one.
     */
    void extrapolateShorterChain();

    /**
     * Extrapolate shorter chain based on corresponding cone in the other cone chain.
     */
    void extrapolateWithCorrespondingCones(ConeChain &longerChain, ConeChain &shorterChain);

    /**
     * Computes cost of cone_desc accorging to the chain and assigns it to cost variable.
     */
    float computeCost(const Cone &cone, const ConeChain &chain, const BolideDescriptor &bolide);
};

#endif // TRACKDRIVE_CONE_CHAIN_CREATOR_H