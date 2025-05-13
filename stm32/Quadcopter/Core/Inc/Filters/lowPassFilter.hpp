/*
 * lowPassFilter.hpp
 *
 *  Created on: May 12, 2025
 *      Author: louis
 */

#pragma once


template<typename T>
class LPF
{
private:
	T m_alpha = T(0.0);
	T m_previousVal = T(0.0);

public:
	LPF() = default;

	void init(const T& alpha)
	{
		if (alpha < T(0.0))
		{
			m_alpha = T(0.0);
			return;
		}
		else if (alpha > T(1.0))
		{
			m_alpha = T(1.0);
			return;
		}

		m_alpha = alpha;
	};

	void init(const T& signalFrequency, const T& cutFrequency)
	{
		const T dt = T(1.0) / signalFrequency;
		const T rc = T(1.0) / (T(2.0) * M_PI * cutFrequency);

		T alpha = rc / (rc + dt);

		if (alpha < T(0.0))
		{
			m_alpha = T(0.0);
			return;
		}
		else if (alpha > T(1.0))
		{
			m_alpha = T(1.0);
			return;
		}

		m_alpha = alpha;
	};

	T apply(const T& val)
	{
		T filtered = m_alpha * m_previousVal + (T(1.0) - m_alpha) * val;
		m_previousVal = filtered;

		return filtered;
	};

	void reset(const T& initial = T(0.0))
	{
		m_previousVal = initial;
	}
};
