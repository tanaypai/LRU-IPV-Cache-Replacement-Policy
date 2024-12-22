/**
 * Copyright (c) 2018-2020 Inria
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Authors: Tanay Pai
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_LRU_VARIATION_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_LRU_VARIATION_HH__

#include "mem/cache/replacement_policies/base.hh"

// Forward declaration for LRU_VariationParams
struct LRU_VariationParams;
namespace ReplacementPolicy {

// Class declaration for LRU_Variation, inheriting from Base class
class LRU_Variation : public Base {
  protected:
    const int numWays;           // Number of cache ways
    int k;                        // Counter for instantiation
    typedef std::vector<int> LRUIPVvector;  // Type definition for vector
    LRUIPVvector *vectInstance;    // Instance of the vector

    // Struct for replacement data, inheriting from ReplacementData
    struct LRUReplData : ReplacementData {
        Tick lastTouchTick;         // Timestamp of the last touch
        mutable int index;          // Mutable index for replacement
        std::shared_ptr<LRUIPVvector> Vector;  // Shared pointer to the vector
        LRUReplData(int assoc, std::shared_ptr<LRUIPVvector> Vector);
    };

    // Initialization of the InsertionVector with predefined values
    const std::vector<int> InsertionVector{0, 0, 1, 0, 2, 0, 1, 2, 1, 0, 6, 1, 0, 0, 1, 11, 12};

  public:
    // Type definition for LRU_VariationParams
    typedef LRU_VariationParams Params;
    LRU_Variation(const Params &p);  // Constructor for LRU_Variation
    ~LRU_Variation() {}              // Destructor

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Sets its last touch tick as the starting tick.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data) const override;

    /**
     * Touch an entry to update its replacement data.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be touched.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const override;

    /**
     * Reset replacement data. Used when an entry is inserted.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be reset.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const override;

    /**
     * Find replacement victim using LRU timestamps.
     *
     * @param candidates Replacement candidates, selected by indexing policy.
     * @return Replacement entry to be replaced.
     */
    ReplaceableEntry* getVictim(const ReplacementCandidates& candidates) const override;

    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;
};

}  // namespace ReplacementPolicy

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_LRU_IPV_HH__

