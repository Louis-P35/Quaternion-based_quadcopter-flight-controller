/*
 * lpfBiquadButterworth.hpp
 *
 *  Created on: May 23, 2025
 *      Author: louis
 */

#pragma once

// Includes from project
#include "Filters/filtersBase.hpp"

// Includes from STL
#include <cmath>



/*
 * 2nd-order Butterworth low-pass biquad filter
 */
template<typename T>
class BiquadLPF : public FiltersBase<T>
{
private:
    // Filter coefficients
    T m_b0 = T(1.0);
    T m_b1 = T(0.0);
    T m_b2 = T(0.0);
    T m_a1 = T(0.0);
    T m_a2 = T(0.0);

    // State: x[n-1], x[n-2], y[n-1], y[n-2]
    T m_x1 = T(0.0);
    T m_x2 = T(0.0);
    T m_y1 = T(0.0);
    T m_y2 = T(0.0);

    static constexpr T pi = T(3.14159265358979323846);
    // Butterworth Q-factor for 2nd order
    static constexpr T Q = T(1.0) / std::sqrt(T(2.0));

public:
    BiquadLPF() = default;

    /*
     * Initialize the filter:
     *  fs = sampling rate (Hz)
     *  fc = cutoff frequency (Hz)
     */
    virtual void init(const T& fs, const T* pParams = nullptr) override
    {
    	if (pParams == nullptr)
    	{
    		return;
    	}
    	T fc = pParams[0];

        T w0    = T(2.0) * pi * fc / fs;
        T cosw0 = std::cos(w0);
        T sinw0 = std::sin(w0);
        T alpha = sinw0 / (T(2.0) * Q);

        // Biquad “low-pass” coefficients (Direct Form I)
        T b0 =  (T(1.0) - cosw0) / T(2.0);
        T b1 =   T(1.0) - cosw0;
        T b2 =  (T(1.0) - cosw0) / T(2.0);
        T a0 =   T(1.0) + alpha;
        T a1 =  -T(2.0) * cosw0;
        T a2 =   T(1.0) - alpha;

        // Normalize so that a0 == 1
        m_b0 = b0 / a0;
        m_b1 = b1 / a0;
        m_b2 = b2 / a0;
        m_a1 = a1 / a0;
        m_a2 = a2 / a0;

        // Reset state
        m_x1 = m_x2 = m_y1 = m_y2 = T(0.0);
    }

    /*
     * Apply the filter to one sample x[n] → returns y[n]
     */
    virtual T apply(const T& x0) override
    {
        T y0 = m_b0 * x0
             + m_b1 * m_x1
             + m_b2 * m_x2
             - m_a1 * m_y1
             - m_a2 * m_y2;

        // Shift delay line
        m_x2 = m_x1;
        m_x1 = x0;
        m_y2 = m_y1;
        m_y1 = y0;

        return y0;
    }

    /*
     * Reset internal state to a given initial value
     */
    virtual void reset(const T& initial = T(0.0)) override
    {
        m_x1 = m_x2 = m_y1 = m_y2 = initial;
    }
};

