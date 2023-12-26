/*
 * OversamplingHelpers.h - helper classes for upsampling and downsampling by any power of 2
 *
 * Copyright (c) 2023 Lost Robot <r94231/at/gmail/dot/com>
 *
 * This file is part of LMMS - https://lmms.io
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program (see COPYING); if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301 USA.
 *
 */

#ifndef OVERSAMPLING_HELPERS_H
#define OVERSAMPLING_HELPERS_H
#include "hiir/Downsampler2xFpu.h"
#include "hiir/PolyphaseIir2Designer.h"
#include "hiir/Upsampler2xFpu.h"

namespace lmms
{

class Upsampler
{
public:
	Upsampler(int stages, float sampleRate, int coefCount = 8, float passband = 20534.f)
	{
		setup(stages, sampleRate, coefCount, passband);
	}
	
	void reset()
	{
		m_upsampleIn.clear();
		m_upsampleIn.resize((1 << m_stages) - 1);
		float bw = 0.5f - m_passband / m_sampleRate;
		for (int i = 0, offset = 1; i < m_stages; ++i, offset <<= 1)
		{
			double coefs[m_coefCount];
			hiir::PolyphaseIir2Designer::compute_coefs_spec_order_tbw(coefs, m_coefCount, bw);
			for (int j = 0; j < offset; ++j)
			{
				m_upsampleIn[offset - 1 + j].set_coefs(coefs);
			}
			bw = (bw + 0.5f) * 0.5f;
		}
	}
	
	void setup(int stages, float sampleRate, int coefCount = 8, float passband = 20534.f)
	{
		m_stages = stages;
		m_sampleRate = sampleRate;
		m_coefCount = coefCount;
		m_passband = passband;
		reset();
	}
	
	inline void process(float* outSamples, float inSample)
	{
		int total = 1 << m_stages;
		outSamples[0] = inSample;
		for (int i = 0; i < m_stages; ++i)
		{
			int count = 1 << i;
			int gap = total / count;
			for (int j = 0; j < count; ++j)
			{
				int temp = j * gap;
				m_upsampleIn[j + count - 1].process_sample(outSamples[temp], outSamples[temp + gap / 2], outSamples[temp]);
			}
		}
	}
	
	inline int getStages() { return m_stages; }
	inline float getSampleRate() { return m_sampleRate; }
	inline int getCoefCount() { return m_coefCount; }
	inline float getPassband() { return m_passband; }

	inline void setStages(int stages) { m_stages = stages; reset(); }
	inline void setSampleRate(float sampleRate) { m_sampleRate = sampleRate; reset(); }
	inline void setCoefCount(int coefCount) { m_coefCount = coefCount; reset(); }
	inline void setPassband(float passband) { m_passband = passband; reset(); }

private:
	std::vector<hiir::Upsampler2xFpu<8>> m_upsampleIn;
	int m_stages;
	float m_sampleRate;
	int m_coefCount;
	float m_passband;
};


class Downsampler
{
public:
	Downsampler(int stages, float sampleRate, int coefCount = 8, float passband = 20534.f)
	{
		setup(stages, sampleRate, coefCount, passband);
	}
	
	void reset()
	{
		m_downsampleIn.clear();
		m_downsampleIn.resize((1 << m_stages) - 1);
		float bw = 0.5f - m_passband / m_sampleRate;
		for (int i = 0, offset = 1; i < m_stages; ++i, offset <<= 1)
		{
			double coefs[m_coefCount];
			hiir::PolyphaseIir2Designer::compute_coefs_spec_order_tbw(coefs, m_coefCount, bw);
			for (int j = 0; j < offset; ++j)
			{
				m_downsampleIn[offset - 1 + j].set_coefs(coefs);
			}
			bw = (bw + 0.5f) * 0.5f;
		}
	}
	
	void setup(int stages, float sampleRate, int coefCount = 8, float passband = 20534.f)
	{
		m_stages = stages;
		m_sampleRate = sampleRate;
		m_coefCount = coefCount;
		m_passband = passband;
		reset();
	}
	
	inline float process(float* inSamples)
	{
		for (int i = m_stages - 1; i >= 0; --i)
		{
			int count = 1 << i;
			for (int j = 0; j < count; ++j)
			{
				inSamples[j] = m_downsampleIn[j + count - 1].process_sample(&inSamples[j * 2]);
			}
		}
		return inSamples[0];
	}
	
	inline int getStages() { return m_stages; }
	inline float getSampleRate() { return m_sampleRate; }
	inline int getCoefCount() { return m_coefCount; }
	inline float getPassband() { return m_passband; }

	inline void setStages(int stages) { m_stages = stages; reset(); }
	inline void setSampleRate(float sampleRate) { m_sampleRate = sampleRate; reset(); }
	inline void setCoefCount(int coefCount) { m_coefCount = coefCount; reset(); }
	inline void setPassband(float passband) { m_passband = passband; reset(); }

private:
	std::vector<hiir::Downsampler2xFpu<8>> m_downsampleIn;
	int m_stages;
	float m_sampleRate;
	int m_coefCount;
	float m_passband;
};

} // namespace lmms

#endif
