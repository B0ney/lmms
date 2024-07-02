/*
 * GranularPitchShifter.cpp
 *
 * Copyright (c) 2024 Lost Robot <r94231/at/gmail/dot/com>
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

#include "GranularPitchShifter.h"

#include <cmath>
#include "embed.h"
#include "plugin_export.h"

















#include <iostream>
















namespace lmms
{

extern "C"
{

Plugin::Descriptor PLUGIN_EXPORT granularpitchshifter_plugin_descriptor =
{
	LMMS_STRINGIFY(PLUGIN_NAME),
	"Granular Pitch Shifter",
	QT_TRANSLATE_NOOP("PluginBrowser", "Granular pitch shifter"),
	"Lost Robot <r94231/at/gmail/dot/com>",
	0x0100,
	Plugin::Type::Effect,
	new PluginPixmapLoader("logo"),
	nullptr,
	nullptr,
} ;

}


GranularPitchShifterEffect::GranularPitchShifterEffect(Model* parent, const Descriptor::SubPluginFeatures::Key* key) :
	Effect(&granularpitchshifter_plugin_descriptor, parent, key),
	m_granularpitchshifterControls(this),
	m_prefilter({BasicFilters<1>(44100), BasicFilters<1>(44100)})
{
	autoQuitModel()->setValue(autoQuitModel()->maxValue());
	
	m_prefilter[0].setFilterType(BasicFilters<1>::FilterType::DoubleLowPass);
	m_prefilter[1].setFilterType(BasicFilters<1>::FilterType::DoubleLowPass);
	
	m_grains.reserve(8);// arbitrary
	
	changeSampleRate();
}


bool GranularPitchShifterEffect::processAudioBuffer(sampleFrame* buf, const fpp_t frames)
{
	if (!isEnabled() || !isRunning()) { return false; }

	const float d = dryLevel();
	const float w = wetLevel();
	
	const ValueBuffer * pitchBuf = m_granularpitchshifterControls.m_pitchModel.valueBuffer();
	const ValueBuffer * pitchSpreadBuf = m_granularpitchshifterControls.m_pitchSpreadModel.valueBuffer();

	const float size = m_granularpitchshifterControls.m_sizeModel.value();
	const float shape = m_granularpitchshifterControls.m_shapeModel.value();
	const float jitter = m_granularpitchshifterControls.m_jitterModel.value();
	const float posrand = m_granularpitchshifterControls.m_posrandModel.value();
	const float posrandSpread = m_granularpitchshifterControls.m_posrandSpreadModel.value();
	const float prefilter = m_granularpitchshifterControls.m_prefilterModel.value();
	const float density = m_granularpitchshifterControls.m_densityModel.value();
	const float glide = m_granularpitchshifterControls.m_glideModel.value();
	const int minLatency = m_granularpitchshifterControls.m_minLatencyModel.value() * m_sampleRate;
	
	if (glide != m_oldGlide)
	{
		m_oldGlide = glide;
		m_glideCoef = std::exp(-std::log((1.f + m_targetRatio) / m_targetRatio) / (glide * m_sampleRate));
	}
	
	m_shapeK = cosWindowApproxK(shape);

	for (fpp_t f = 0; f < frames; ++f)
	{
		const float pitch = pitchBuf ? pitchBuf->value(f) : m_granularpitchshifterControls.m_pitchModel.value();
		const float pitchSpread = (pitchSpreadBuf ? pitchSpreadBuf->value(f) : m_granularpitchshifterControls.m_pitchSpreadModel.value()) * 0.5f;
		
		bool pitchChanged = false;
		for (int i = 0; i < 2; ++i)
		{
			float targetVal = pitch + pitchSpread * (i ? 1 : -1);
			
			if (targetVal == m_truePitch[i]) { continue; }
			pitchChanged = true;
			bool initialSide = targetVal > m_truePitch[i];

			// 1-pole lowpass with overshoot to interpolate value
			m_truePitch[i] = m_glideCoef * m_truePitch[i] + (1.f - m_glideCoef) * targetVal * (initialSide ? m_targetRatio : m_targetRatioInv);
			
			// when it overshoots, set it to the exact value
			bool finalSide = targetVal > m_truePitch[i];
			if (initialSide != finalSide) { m_truePitch[i] = targetVal; }
		}
		
		if (pitchChanged)
		{
			const double speed[2] = {std::exp2(m_truePitch[0] * (1.f / 1200.f)), std::exp2(m_truePitch[1] * (1.f / 1200.f))};
			std::array<double, 2> ratio = {speed[0] / m_speed[0], speed[1] / m_speed[1]};
			for (int i = 0; i < m_grainCount; ++i)
			{
				m_grains[i].m_phaseSpeed[0] *= ratio[0];
				m_grains[i].m_phaseSpeed[1] *= ratio[1];
			}
			m_speed[0] = speed[0];
			m_speed[1] = speed[1];
			m_prefilter[0].calcFilterCoeffs(std::min(m_nyquist / (float)speed[0], m_nyquist) * 0.96f, 0.70710678118);
			m_prefilter[1].calcFilterCoeffs(std::min(m_nyquist / (float)speed[1], m_nyquist) * 0.96f, 0.70710678118);
		}
		
		const int sizeSamples = static_cast<int>((m_sampleRate / size) * 0.5) * 2;
		const float phaseInc = 1.f / sizeSamples;
		
		std::array<float, 2> s = {0, 0};
		std::array<float, 2> filtered = {buf[f][0], buf[f][1]};
		
		if (++m_timeSinceLastGrain >= m_nextWaitRandomization * sizeSamples / (density * 2))
		{
			m_timeSinceLastGrain = 0;
			m_nextWaitRandomization = std::exp2((fast_rand()/(float)FAST_RAND_MAX * 2 - 1.f) * jitter);

			std::array<float, 2> posrandResult = {0, 0};
			if (posrand > 0)
			{
				posrandResult[0] = (fast_rand() / (float)FAST_RAND_MAX) * posrand * m_sampleRate * GPS_POSRAND_MAX;
				posrandResult[1] = linearInterpolate(posrandResult[0],
									(fast_rand() / (float)FAST_RAND_MAX) * posrand * m_sampleRate * GPS_POSRAND_MAX,
									posrandSpread);
			}
			
			std::array<int, 2> placeThing;
			int latency = std::max(int(std::max(sizeSamples * (std::max(m_speed[0], m_speed[1]) - 1.), 0.) + 3.f), minLatency);// few extra samples of latency for safety
			for (int i = 0; i < 2; ++i)
			{
				placeThing[i] = m_writePoint - latency - posrandResult[i];
				if (placeThing[i] < 0) {placeThing[i] += m_ringBufLength;}
			}
			m_grains.push_back(GranularPitchShifterGrain(0, placeThing[0], placeThing[1]));
			++m_grainCount;
		}
		
		for (int i = 0; i < m_grainCount; ++i)
		{
			m_grains[i].m_phase += phaseInc * std::max(m_grains[i].m_phaseSpeed[0], m_grains[i].m_phaseSpeed[1]);
			if (m_grains[i].m_phase >= 1)
			{
				std::swap(m_grains[i], m_grains[m_grainCount-1]);
				m_grains.pop_back();
				--i;
				--m_grainCount;
				continue;
			}
			
			m_grains[i].m_readPoint[0] += m_speed[0];
			m_grains[i].m_readPoint[1] += m_speed[1];
			if (m_grains[i].m_readPoint[0] >= m_ringBufLength) { m_grains[i].m_readPoint[0] -= m_ringBufLength; }
			if (m_grains[i].m_readPoint[1] >= m_ringBufLength) { m_grains[i].m_readPoint[1] -= m_ringBufLength; }
			
			const float windowVal = cosHalfWindowApprox(-std::abs(-2*m_grains[i].m_phase+1)+1, m_shapeK);
			s[0] += getHermiteSample(m_grains[i].m_readPoint[0], 0) * windowVal;
			s[1] += getHermiteSample(m_grains[i].m_readPoint[1], 1) * windowVal;
		}
		
		if (++m_writePoint >= m_ringBufLength)
		{
			m_writePoint = 0;
		}
		if (prefilter)
		{
			filtered[0] = m_prefilter[0].update(filtered[0], 0);
			filtered[1] = m_prefilter[1].update(filtered[1], 0);
		}
		m_ringBuf[m_writePoint][0] = filtered[0];
		m_ringBuf[m_writePoint][1] = filtered[1];
			
		buf[f][0] = d * buf[f][0] + w * s[0];
		buf[f][1] = d * buf[f][1] + w * s[1];
	}

	return isRunning();
}

void GranularPitchShifterEffect::changeSampleRate()
{
	m_sampleRate = Engine::audioEngine()->outputSampleRate();
	m_nyquist = m_sampleRate / 2;
	
	m_ringBufLength = m_sampleRate;
	m_ringBuf.resize(m_ringBufLength);
	for (size_t i = 0; i < m_ringBufLength; ++i)
	{
		m_ringBuf[i][0] = 0;
		m_ringBuf[i][1] = 0;
	}
	m_writePoint = 0;
	
	m_prefilter[0].setSampleRate(m_sampleRate);
	m_prefilter[1].setSampleRate(m_sampleRate);
	
	m_targetRatio = 1.001f;
	m_targetRatioInv = 1.f / m_targetRatio;
	
	m_oldGlide = -1;
	
	const float pitch = m_granularpitchshifterControls.m_pitchModel.value();
	const float pitchSpread = m_granularpitchshifterControls.m_pitchSpreadModel.value();
	
	m_truePitch[0] = pitch - pitchSpread;
	m_truePitch[1] = pitch + pitchSpread;
	m_speed[0] = std::exp2(m_truePitch[0] * (1.f / 1200.f));
	m_speed[1] = std::exp2(m_truePitch[1] * (1.f / 1200.f));
	m_prefilter[0].calcFilterCoeffs(std::min(m_nyquist / (float)m_speed[0], m_nyquist) * 0.96f, 0.70710678118);
	m_prefilter[1].calcFilterCoeffs(std::min(m_nyquist / (float)m_speed[1], m_nyquist) * 0.96f, 0.70710678118);
}


extern "C"
{

// necessary for getting instance out of shared lib
PLUGIN_EXPORT Plugin* lmms_plugin_main(Model* parent, void* data)
{
	return new GranularPitchShifterEffect(parent, static_cast<const Plugin::Descriptor::SubPluginFeatures::Key*>(data));
}

}

} // namespace lmms
