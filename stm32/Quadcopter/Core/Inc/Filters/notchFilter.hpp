/*
 * notchFilter.hpp
 *
 *  Created on: May 12, 2025
 *      Author: louis
 */

#pragma once

// Includes from project
#include "Filters/notchFilter.hpp"

// Include from STL
#include <cmath>


/*
 * Frequency cut-off filter
 */
template<typename T>
class NotchFilter
{
private:
    // filter coefficients
    T m_b0 = T(1.0);
	T m_b1 = T(0.0);
    T m_b2 = T(0.0);
    T m_a1 = T(0.0);
    T m_a2 = T(0.0);

    // state: x[n-1], x[n-2], y[n-1], y[n-2]
    T m_x1 = T(0.0);
    T m_x2 = T(0.0);
    T m_y1 = T(0.0);
    T m_y2 = T(0.0);

    static constexpr T pi = T(3.14159265358979323846);

public:
    NotchFilter() = default;

    /*
     * Initialize the notch: fs = sampling rate (Hz), f0 = notch center (Hz), Q = quality factor.
     */
    void init(const T& fs, const T& f0, const T& Q)
    {
        const T w0  = T(2.0) * pi * f0 / fs;
        const T alpha = std::sin(w0) / (T(2.0) * Q);

        // Biquad “notch” coefficients (Direct Form I)
        T b0 =  T(1.0);
        T b1 = T(-2.0) * std::cos(w0);
        T b2 =  T(1.0);
        T a0 =  T(1.0) + alpha;
        T a1 = T(-2.0) * std::cos(w0);
        T a2 =  T(1.0) - alpha;

        // Normalize so a0 == 1
        m_b0 = b0 / a0;
        m_b1 = b1 / a0;
        m_b2 = b2 / a0;
        m_a1 = a1 / a0;
        m_a2 = a2 / a0;

        // Reset state
        m_x1 = T(0.0);
        m_x2 = T(0.0);
        m_y1 = T(0.0);
        m_y2 = T(0.0);
    }

    /*
     * Apply the filter to one sample (x[n]) → returns y[n]
     */
    T apply(const T& x0)
    {
        T y0 = m_b0*x0 + m_b1*m_x1 + m_b2*m_x2 - m_a1*m_y1 - m_a2*m_y2;

        // Shift delay line
        m_x2 = m_x1;
        m_x1 = x0;

        m_y2 = m_y1;
        m_y1 = y0;

        return y0;
    }
};

