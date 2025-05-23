/*
 * filtersBase.hpp
 *
 *  Created on: May 23, 2025
 *      Author: louis
 */

#pragma once




// Base class for all filters
template<typename T>
class FiltersBase
{
public:
    virtual ~FiltersBase() = default;

    /*
     * Initialize the filter with sampling rate and parameters
     * fs: sampling rate (Hz)
     * params: optional filter-specific parameters (e.g., cutoff frequency)
     */
    virtual void init(const T& fs, const T* pParams = nullptr) = 0;

    /*
     * Process one input sample and return the filtered output
     * x: input sample
     * Returns: filtered output
     */
    virtual T apply(const T& x0) = 0;

    /*
     * Reset filter state to a given initial value
     * initial: initial state value (default 0.0)
     */
    virtual void reset(const T& initial = T(0.0)) = 0;
};
