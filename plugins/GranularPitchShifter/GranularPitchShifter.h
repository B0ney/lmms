/*
 * GranularPitchShifter.h - granularpitchshifter-effect-plugin
 *
 * Copyright (c) 2014 Vesa Kivimäki <contact/dot/diizy/at/nbl/dot/fi>
 * Copyright (c) 2006-2014 Tobias Doerffel <tobydox/at/users.sourceforge.net>
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

#ifndef GRANULARPITCHSHIFTER_H
#define GRANULARPITCHSHIFTER_H

#include "Effect.h"
#include "GranularPitchShifterControls.h"

#include "BasicFilters.h"
#include "interpolation.h"

constexpr float GPS_POSRAND_MAX = 0.5f;

namespace lmms
{

// adapted from signalsmith's crossfade approximation:
// https://signalsmith-audio.co.uk/writing/2021/cheap-energy-crossfade
inline float cosHalfWindowApprox(float x, float k) {
	float A = x * (1 - x);
	float B = A * (1 + k * A);
	float C = (B + x);
	return C * C;
}
// 1-2 fades between equal-gain and equal-power
inline float cosWindowApproxK(float p) {
	return -6.0026608f + p * (6.8773512f - 1.5838104f * p);
}

class GranularPitchShifterGrain final {
public:
	GranularPitchShifterGrain(float phase, double readPointL, double readPointR) :
		m_readPoint({readPointL, readPointR}),
		m_phaseSpeed({1, 1}),
		m_phase(phase) {}
	std::array<double, 2> m_readPoint;
	std::array<double, 2> m_phaseSpeed;
	float m_phase;
};

class GranularPitchShifterEffect : public Effect
{
public:
	GranularPitchShifterEffect(Model* parent, const Descriptor::SubPluginFeatures::Key* key);
	~GranularPitchShifterEffect() override = default;
	bool processAudioBuffer(sampleFrame* buf, const fpp_t frames) override;

	EffectControls* controls() override
	{
		return &m_granularpitchshifterControls;
	}
	
	inline float getHermiteSample(float index, int ch) {
		const int index_floor = static_cast<int>(index);
		const float fraction = index - index_floor;
		
		float v0, v1, v2, v3;

		if (index_floor == 0) {
			v0 = m_ringBuf[m_ringBuf.size() - 1][ch];
		} else {
			v0 = m_ringBuf[index_floor - 1][ch];
		}
		
		v1 = m_ringBuf[index_floor][ch];
		
		if(index_floor >= m_ringBuf.size() - 2) {
			v2 = m_ringBuf[(index_floor + 1) % m_ringBuf.size()][ch];
			v3 = m_ringBuf[(index_floor + 2) % m_ringBuf.size()][ch];
		} else {
			v2 = m_ringBuf[index_floor + 1][ch];
			v3 = m_ringBuf[index_floor + 2][ch];
		}
		
		return hermiteInterpolate(v0, v1, v2, v3, fraction);
	}

public slots:
	void changeSampleRate();

private:
	GranularPitchShifterControls m_granularpitchshifterControls;
	
	std::vector<std::array<float, 2>> m_ringBuf;
	int m_ringBufLength = 0;
	int m_writePoint = 0;
	
	std::vector<GranularPitchShifterGrain> m_grains;
	int m_grainCount = 0;
	
	int m_timeSinceLastGrain = 0;
	float m_nextWaitRandomization = 1;
	
	float m_sampleRate;
	float m_nyquist;
	
	std::array<double, 2> m_speed = {1, 1};
	
	std::array<BasicFilters<1>, 2> m_prefilter;
	
	std::array<float, 2> m_truePitch = {0, 0};
	
	float m_oldGlide = -1;
	float m_oldPitchSpread = -1;
	float m_glideCoef = 0.f;
	float m_targetRatio = 1.001f;
	float m_targetRatioInv = 1.f / 1.001f;
	
	friend class GranularPitchShifterControls;
};

} // namespace lmms

#endif
