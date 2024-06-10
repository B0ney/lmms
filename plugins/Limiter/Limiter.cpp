/*
 * Limiter.cpp
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

#include "Limiter.h"

#include "embed.h"
#include "interpolation.h"
#include "plugin_export.h"







#include <iostream>














namespace lmms
{

extern "C"
{

Plugin::Descriptor PLUGIN_EXPORT limiter_plugin_descriptor =
{
	LMMS_STRINGIFY(PLUGIN_NAME),
	"Limiter",
	QT_TRANSLATE_NOOP("PluginBrowser", "An absurdly clean and transparent limiter designed for use as the last effect on your master bus."),
	"Lost Robot <r94231/at/gmail/dot/com>",
	0x0100,
	Plugin::Type::Effect,
	new PluginPixmapLoader("logo"),
	nullptr,
	nullptr,
} ;

}


LimiterEffect::LimiterEffect(Model* parent, const Descriptor::SubPluginFeatures::Key* key) :
	Effect(&limiter_plugin_descriptor, parent, key),
	m_limiterControls(this),
	m_sampleRate(Engine::audioEngine()->outputSampleRate()),
	m_lufsProcessor(m_sampleRate),
	m_displayHandler(m_sampleRate, 120),
	m_peakHold{signalsmith::envelopes::PeakHold<float>(1),
			   signalsmith::envelopes::PeakHold<float>(1)},
	m_boxStackFilter{signalsmith::envelopes::BoxStackFilter<float>(1, 6),
					 signalsmith::envelopes::BoxStackFilter<float>(1, 6)},
	m_peakHold2{signalsmith::envelopes::PeakHold<float>(1),
			   signalsmith::envelopes::PeakHold<float>(1)},
	m_boxStackFilter2{signalsmith::envelopes::BoxStackFilter<float>(1, 6),
					 signalsmith::envelopes::BoxStackFilter<float>(1, 6)},
	m_ispFinder(),
	m_upsampler{Upsampler(1, m_sampleRate), Upsampler(1, m_sampleRate)},
	m_downsampler{Downsampler(1, m_sampleRate), Downsampler(1, m_sampleRate)},
	m_kWeightForCrest{m_sampleRate, m_sampleRate}
{
	emit changeSampleRate();
}


bool LimiterEffect::processAudioBuffer(sampleFrame* buf, const fpp_t frames)
{
	if (!isEnabled() || !isRunning()) { return false ; }

	double outSum = 0.0;
	const float d = dryLevel();
	const float w = wetLevel();
	
	const int oversampling = m_limiterControls.m_oversampleModel.value();
	const int oversampleMult = 1 << oversampling;
	if (oversampling != m_oldOversampling)
	{
		m_oldOversampling = oversampling;
		emit changeSampleRate();
	}
	
	const int smooth = m_limiterControls.m_smoothStagesModel.value();
	if (smooth != m_oldSmooth)
	{
		m_oldSmooth = smooth;
		changeSmooth();
	}
	
	const int lookahead = m_limiterControls.m_lookaheadModel.value() * 0.001f * m_sampleRate + 1;
	const int lookaheadOversamp = m_limiterControls.m_lookaheadModel.value() * 0.001f * m_sampleRate * oversampleMult + 1;
	const int lookReleaseStages = m_limiterControls.m_lookReleaseStagesModel.value();
	const float lookRelease = m_limiterControls.m_lookReleaseModel.value() / lookReleaseStages;
	const float adaptRelease = m_limiterControls.m_adaptReleaseModel.value();
	const float adaptSlow = 1.f - m_limiterControls.m_adaptSlowModel.value() * 0.99f;
	const float stereoLinkTransient = m_limiterControls.m_stereoLinkTransientModel.value();
	const float stereoLinkRelease = m_limiterControls.m_stereoLinkReleaseModel.value();
	const float dcFreq = m_limiterControls.m_dcFreqModel.value();
	const bool truePeak = m_limiterControls.m_truePeakModel.value();
	const float threshold = dbfsToAmp(m_limiterControls.m_thresholdModel.value());
	const float invThreshold = 1.f / threshold;
	const float inGain = dbfsToAmp(m_limiterControls.m_inGainModel.value());
	const float outGain = dbfsToAmp(m_limiterControls.m_outGainModel.value());
	const float hold = m_limiterControls.m_holdModel.value() * 0.001f * m_sampleRate * oversampleMult;
	const bool ditherEnabled = m_limiterControls.m_ditherEnabledModel.value();
	const int ditherBitDepth = m_limiterControls.m_ditherBitDepthModel.value();
	const int ditherType = m_limiterControls.m_ditherTypeModel.value();
	const float cleanClipAmount = m_limiterControls.m_cleanClipAmountModel.value();
	const float cleanClipStart = dbfsToAmp(m_limiterControls.m_cleanClipStartModel.value());
	const float cleanClipCeiling = dbfsToAmp(m_limiterControls.m_cleanClipCeilingModel.value());
	const float cleanClipCeilingReal = (cleanClipCeiling - cleanClipStart) * 2;
	const bool kWeightCrest = m_limiterControls.m_kWeightCrestModel.value();
	
	if (lookahead != m_oldLookahead)
	{
		m_oldLookahead = lookahead;
		m_oldHold = hold;
		for (int i = 0; i < 2; ++i)
		{
			m_peakHold[i].set(lookaheadOversamp + hold);
			m_boxStackFilter[i].set(lookaheadOversamp);
			
			m_peakHold2[i].set(lookahead);
			m_boxStackFilter2[i].set(lookahead);
		}
	}
	else if (hold != m_oldHold)
	{
		m_oldHold = hold;
		for (int i = 0; i < 2; ++i)
		{
			m_peakHold[i].set(lookaheadOversamp + hold);
		}
	}
	
	const float dcCoeff = (F_2PI * dcFreq) / m_sampleRate;
	
	const int oversampleVal = 1 << oversampling;
	
	float inVolDisplayNext[2] = {};
	float outVolDisplayNext[2] = {};
	float gainDisplayNext[2] = {};
	for (fpp_t f = 0; f < frames; ++f)
	{
		for (int i = 0; i < 2; ++i)
		{
			buf[f][i] *= inGain;
			
			inVolDisplayNext[i] = std::abs(buf[f][i]);
			
			// DC Offset removal (first-order highpass filter).
			if (dcFreq) { buf[f][i] -= (m_dcVal[i] = dcCoeff * buf[f][i] + (1.f - dcCoeff) * m_dcVal[i]); }
			
			// I cheat with the threshold implementation by applying it as input/output gain and
			// keeping the real threshold locked at 0 dBFS, for both code simplicity and performance.
			buf[f][i] *= invThreshold;
		}
		
		float lookReleaseVal[2];
		float lookReleaseValOversamp[2];
		for (int i = 0; i < 2; ++i)
		{
			if (lookRelease == 0)
			{
				lookReleaseVal[i] = 0;
				lookReleaseValOversamp[i] = 0;
				continue;
			}
		
			// My adaptive release algorithm.  It's far easier to get away with massive gain changes
			// with minimal audibility when there are transients, so I decrease the release time
			// whenever the crest factor is high.
			// The user also has the option to decrease the release time when the crest factor is low,
			// which prompts the limiter to wait more patiently for transients before making significant
			// gain increases.
			float crestIn = buf[f][i];
			if (kWeightCrest)
			{
				// Excessive low frequencies were having larger impacts on the crest factor measurements
				// than I was happy with.  I decided to take the K-Weighting filter, normally meant
				// solely for LUFS measurement, and re-use it here so the crest factor is measured
				// based on something closer to the audio's actual loudness rather than just its digital
				// volume.  This'll make it less sensitive to frequencies the human ear is less sensitive
				// to, and vice-versa.
				crestIn = m_kWeightForCrest[i].process(crestIn);
			}
			const float inSquared = crestIn * crestIn;
			m_crestPeak[i] = std::max(inSquared, m_crestPeak[i] * m_crestMeanCoeff + inSquared * (1.f - m_crestMeanCoeff));
			m_crestMean[i] = m_crestMean[i] * m_crestMeanCoeff + inSquared * (1.f - m_crestMeanCoeff);
			m_crestFactor[i] = m_crestPeak[i] / m_crestMean[i];
			if (adaptRelease <= 1)
			{
				m_releaseDivide[i] = linearInterpolate(1.f, std::min(5.f, std::max(adaptSlow, m_crestFactor[i] * 0.2f)), adaptRelease);
			}
			else
			{
				float temp = adaptRelease - 1.f;
				m_releaseDivide[i] = std::min(5.f + temp * 20.f, std::max(adaptSlow, m_crestFactor[i] * (0.2f + temp * 0.3f) - temp));
			}
			
			// Values are needed for both oversampled and non-oversampled audio
			// due to the second post-oversampling limiter not being oversampled.
			lookReleaseVal[i] = exp(m_coeffPrecalc / (lookRelease / m_releaseDivide[i]));
			lookReleaseValOversamp[i] = (oversampleVal > 1) ? exp(m_coeffPrecalcOversamp / (lookRelease / m_releaseDivide[i])) : lookReleaseVal[i];
		}
		
		// The crest factor is once again used to determine whether we're at a transient
		// for the stereo linking.  It's generally best for transients to have less stereo
		// linking than non-transients, so dips in volume to catch a transient in one ear
		// don't create weird gain fluctuations in the other ear.  This allows us to have
		// all the upsides of full stereo linking (preservation of relative channel volumes
		// to avoid losing stereo width), while removing the largest downsides.
		const float crestMono = (m_crestFactor[0] + m_crestFactor[1]) * 0.5f;
		const float linkAmountType = std::min(std::max(0.f, (crestMono - 5.f) / 12.f), 1.f);
		m_linkAmount = linearInterpolate(stereoLinkRelease, stereoLinkTransient, linkAmountType);
		
		// float overOuts[2][oversampleVal] = {{}};
		std::vector<float> overOuts[2] = { 
			std::vector<float>(oversampleVal), 
			std::vector<float>(oversampleVal),
		};

		if (oversampleVal > 1)
		{
			// Upsample the audio with HIIR via OversamplingHelpers.h.
			m_upsampler[0].process(overOuts[0].data(), buf[f][0]);
			m_upsampler[1].process(overOuts[1].data(), buf[f][1]);
		}
		else
		{
			overOuts[0][0] = buf[f][0];
			overOuts[1][0] = buf[f][1];
		}
		
		// True peaks are only detected here if there's no oversampling.
		// Otherwise, the task of true peak limiting is given to the second post-oversampling limiter.
		int truePeakDelay = (truePeak && oversampleVal < 2) ? LIMITER_TRUE_PEAK_LATENCY : 0;
		
		for (int overSamp = 0; overSamp < oversampleVal; ++overSamp)
		{
			std::array<float, 2> releasePeak = {0, 0};

			for (int i = 0; i < 2; ++i)
			{
				if (cleanClipAmount >= 0)
				{
					/*
					My "clean clip" algorithm.
					f(x)=c(1/(1+e^(-4c(|x|-s)/((c+(|x|-s))*(c-(|x|-s)))))-0.5)+s
					
					         /                    1                         \
					f(x) = c |---------------------------------------- - 0.5| + s {x>s}∪{x<-s}
							 |     /          -4c(|x| - s)           \      |
							 |     |---------------------------------|      |
							 |     \(c + (|x| - s)) . (c - (|x| - s))/      |
							 \1 + e                                         /

					With `x` being the input, `c` being the hard clip ceiling, and `s` being the "Start", the absolute
					lowest possible volume the distortion can impact in any way.
					
					It's a very close closed-form approximation of this function:
					f(x) = int((1/(1+e^(1/(1-t)-1/t))) * dt ,0,x)
					
					        x                        
							.- /       1       \     
					f(x) =  |  |---------------| . dt
							|  |     / 2t - 1 \|     
							|  |     |--------||     
							|  |     \t(1 - t)/|     
						   -'  \1 + e          /     
							0                        

					I designed this function in a way that causes it to have no discontinuities in *any* derivative,
					making it perfectly smooth.  A lack of waveshaping is equivalent to y=x, so this function starts with
					a first derivative of 1 to match, but all other derivatives start off as exactly equal to zero,
					and the end of the function has all derivatives exactly equal to zero as well, resulting in the
					mathematically cleanest possible transitions from no waveshaping to "clean clipping" to hard clipping.
					
					This should minimize audible distortion, and as a result, maximize how hard this can be pushed
					before the distortion becomes audible.  This allows taming upper peaks in the audio with
					minimal distortion before the limiter is left to deal with it.  Use of oversampling is highly recommended.
					
					The waveshaping is equivalent to y=x while the volume is underneath the "Start" parameter,
					meaning everything below that volume is completely untouched.  Note that while this algorithm
					is designed to be as clean as it can be, the overall distortion level is mainly determined by the
					amount of space between the Start and the Ceiling, so having a very low Start value can be beneficial.
					*/
					const float temp1 = std::abs(overOuts[i][overSamp]) - cleanClipStart;
					if (temp1 > 0)
					{
						if (temp1 >= cleanClipCeilingReal)
						{
							overOuts[i][overSamp] = linearInterpolate(overOuts[i][overSamp], std::copysign(cleanClipCeiling, overOuts[i][overSamp]), cleanClipAmount);
						}
						else
						{
							const float temp2 = cleanClipCeilingReal * (1.f / (1.f + std::exp((-4.f * cleanClipCeilingReal * temp1) / ((cleanClipCeilingReal + temp1) * (cleanClipCeilingReal - temp1)))) - 0.5f) + cleanClipStart;
							overOuts[i][overSamp] = linearInterpolate(overOuts[i][overSamp], std::copysign(temp2, overOuts[i][overSamp]), cleanClipAmount);
						}
					}
				}

				float scIn = (truePeakDelay != 0) ? m_ispFinder[i].processPeak(overOuts[i][overSamp]) : std::abs(overOuts[i][overSamp]);
				
				// Release
				applyRelease(1.f / std::max(1.f, m_peakHold[i](scIn)), lookReleaseValOversamp[i], lookReleaseStages, m_yL[i]);
				
				releasePeak[i] = std::min(1.f, m_yL[i][lookReleaseStages-1]);
			}
			
			// Stereo linking (must be done before smoothing)
			const int temp = releasePeak[0] < releasePeak[1] ? 1 : 0;
			releasePeak[temp] = linearInterpolate(releasePeak[temp], releasePeak[1 - temp], m_linkAmount);
			
			for (int i = 0; i < 2; ++i)
			{
				// Plugin latency
				std::swap(overOuts[i][overSamp], m_inLookBuf[i][m_lookWrite]);
				
				// Smoothing delay
				const float temp2 = m_scLookBuf[i][(m_lookBufLength + m_lookWrite - lookaheadOversamp - truePeakDelay + 1) % m_lookBufLength];
				overOuts[i][overSamp] *= temp2;
				if (overSamp == 0) { gainDisplayNext[i] = temp2; }// for visualizer
				
				// Smooth gain reduction
				m_scLookBuf[i][m_lookWrite] = m_boxStackFilter[i](releasePeak[i]);
			}
			
			if (--m_lookWrite < 0) { m_lookWrite = m_lookBufLength - 1; }
		}
		
		std::array<float, 2> s;
		if (oversampleVal > 1)
		{
			// Downsample the audio with HIIR via OversamplingHelpers.h.
			s[0] = m_downsampler[0].process(overOuts[0].data());
			s[1] = m_downsampler[1].process(overOuts[1].data());
		}
		else
		{
			s[0] = overOuts[0][0];
			s[1] = overOuts[1][0];
		}
		
		// The changes in sample values from the downsampling filters can easily
		// push things above the threshold, which is unacceptable for a limiter.
		// The solution I came up with is to have a second non-oversampled limiter
		// after the first oversampled one.  This second limiter copies nearly all
		// of the settings from the first one, so it's all effectively just one limiter,
		// which does most of the processing with oversampling and the tiny remainder
		// without it.
		if (oversampleVal > 1)
		{
			truePeakDelay = truePeak ? LIMITER_TRUE_PEAK_LATENCY : 0;
			std::array<float, 2> releasePeak = {0, 0};
			for (int i = 0; i < 2; ++i)
			{
				float scIn = truePeak ? m_ispFinder[i].processPeak(s[i]) : std::abs(s[i]);
				applyRelease(1.f / std::max(1.f, m_peakHold2[i](scIn)), lookReleaseVal[i], lookReleaseStages, m_yL2[i]);
				releasePeak[i] = std::min(1.f, m_yL2[i][lookReleaseStages-1]);
			}
			const int temp = releasePeak[0] < releasePeak[1] ? 1 : 0;
			releasePeak[temp] = linearInterpolate(releasePeak[temp], releasePeak[1 - temp], m_linkAmount);
			for (int i = 0; i < 2; ++i)
			{
				std::swap(s[i], m_inLookBuf2[i][m_lookWrite2]);
				const float temp2 = m_scLookBuf2[i][(m_lookBufLength2 + m_lookWrite2 - lookahead - truePeakDelay + 1) % m_lookBufLength2];
				s[i] *= temp2;
				gainDisplayNext[i] *= temp2;
				m_scLookBuf2[i][m_lookWrite2] = m_boxStackFilter2[i](releasePeak[i]);
			}
			if (--m_lookWrite2 < 0) { m_lookWrite2 = m_lookBufLength2 - 1; }
		}
		
		buf[f][0] = d * buf[f][0] + w * s[0] * outGain * threshold * 0.9999f;
		buf[f][1] = d * buf[f][1] + w * s[1] * outGain * threshold * 0.9999f;
		
		/* 
		Dithering removes the quantization distortion from converting audio to a lower
		bit depth.  Since it absolutely must be the last process done to an audio signal,
		it's very common for the feature to be built into mastering limiters, which are
		always the last effect in the master bus effect chain.
		
		As an example, say audio is being quantized to 0 or 1 and the input audio value is 0.8.
		Without dithering, this 0.8 value would be rounded to 1, along with other nearby values,
		resulting in quantization distortion.  With dithering applied (assume rectangular dithering
		for simplicity), there's an 80% chance it'll be rounded to 1 and a 20% chance it'll be
		rounded to 0.  This statistically encodes the true audio signal into the limited bit depth,
		resulting in what is effectively an *infinite* bit depth and dynamic range, with the downside
		being the addition of extremely quiet (completely inaudible) white noise.
		
		This limiter removes the maximum possible volume of the dithering noise from the signal
		in order to guarantee the resulting signal does not clip.
		*/
		if (ditherEnabled && w)
		{
			float ditherGen = 0.f;
			float ditherPeak = 0.f;
			switch (ditherType)
			{
				case 0:// RPDF, Rectangular
					// 1 LSB white noise, which does remove all quantization distortion, but
					// may suffer from intermodulation distortion with low-frequency signals.
					ditherGen = (fast_rand() / (float)FAST_RAND_MAX) - 0.5f;
					ditherPeak = 0.5f;
					break;
				case 1:// TPDF, Triangular
					// 2 LSB blue noise which doesn't have the intermodulation distortion issue.
					// This is the correct choice in the enormous majority of cases.
					// 2 LSB TPDF is strictly superior to 2 LSB RPDF due to it having a lower RMS value.
					ditherGen = (fast_rand() + fast_rand()) / (float)FAST_RAND_MAX - 1.f;
					ditherPeak = 1.f;
					break;
				case 2:// GPDF, Gaussian
					// 3 LSB gaussian noise approximation.  It sounds similar to the noise generated by analog devices.
					ditherGen = ((fast_rand() + fast_rand() + fast_rand() + fast_rand() + fast_rand() + fast_rand()) / (float)FAST_RAND_MAX - 3.f) * 0.5f;
					ditherPeak = 1.5f;
					break;
			}
			
			const float ditherVol = 1.f / float(1 << (ditherBitDepth - 1));
			const float temp1 = (1.f - ditherVol * ditherPeak);
			const float temp2 = ditherGen * ditherVol;
			buf[f][0] = buf[f][0] * temp1 + temp2;
			buf[f][1] = buf[f][1] * temp1 + temp2;
		}
		
		m_lufsProcessor.process(buf[f][0], buf[f][1]);
		
		outVolDisplayNext[0] = std::abs(buf[f][0]);
		outVolDisplayNext[1] = std::abs(buf[f][1]);
		
		m_displayHandler.process(std::abs(inVolDisplayNext[0]), std::abs(inVolDisplayNext[1]), std::abs(outVolDisplayNext[0]), std::abs(outVolDisplayNext[1]), std::abs(gainDisplayNext[0]), std::abs(gainDisplayNext[1]));
		
		outSum += buf[f][0] * buf[f][0] + buf[f][1] * buf[f][1];
	}

	checkGate(outSum / frames);

	return isRunning();
}


void LimiterEffect::changeSampleRate()
{
	m_sampleRate = Engine::audioEngine()->outputSampleRate();
	const int oversampleStages = m_limiterControls.m_oversampleModel.value();
	const int oversampleMult = 1 << oversampleStages;
	
	const int smooth = m_limiterControls.m_smoothStagesModel.value();
	
	m_lufsProcessor.setSampleRate(m_sampleRate);
	m_displayHandler.setSampleRate(m_sampleRate);
	
	m_coeffPrecalc = -1.f / (m_sampleRate * 0.001f);
	m_coeffPrecalcOversamp = -1.f / (m_sampleRate * oversampleMult * 0.001f);
	m_crestMeanCoeff = exp(-1.f / (0.1f * m_sampleRate));

	m_lookBufLength2 = std::ceil(LIMITER_MAX_LOOKAHEAD * 0.001f * m_sampleRate) + LIMITER_TRUE_PEAK_LATENCY + 2;
	m_lookBufLength = m_lookBufLength2 * oversampleMult;
	for (int i = 0; i < 2; ++i)
	{
		m_inLookBuf[i].resize(m_lookBufLength);
		m_scLookBuf[i].resize(m_lookBufLength);
		m_inLookBuf2[i].resize(m_lookBufLength2);
		m_scLookBuf2[i].resize(m_lookBufLength2);
		m_upsampler[i].setup(oversampleStages, m_sampleRate);
		m_downsampler[i].setup(oversampleStages, m_sampleRate);
		
		const float temp = m_lookBufLength2 - 1;
		m_peakHold[i].resize((temp + std::ceil(LIMITER_MAX_HOLD * 0.001f * m_sampleRate)) * oversampleMult);
		m_boxStackFilter[i].resize(temp * oversampleMult, smooth);
		
		m_peakHold2[i].resize(temp);// no hold for the second limiter
		m_boxStackFilter2[i].resize(temp, smooth);
		
		m_kWeightForCrest[i].setSampleRate(m_sampleRate);
	}
	m_oldLookahead = -1;
	m_oldHold = -1;
	
	m_lookWrite = 0;
	m_lookWrite2 = 0;
}

void LimiterEffect::changeSmooth()
{
	const int oversampleStages = m_limiterControls.m_oversampleModel.value();
	const int oversampleMult = 1 << oversampleStages;
	const int smooth = m_limiterControls.m_smoothStagesModel.value();
	for (int i = 0; i < 2; ++i)
	{
		const float temp = m_lookBufLength2 - 1;
		m_boxStackFilter[i].resize(temp * oversampleMult, smooth);
		m_boxStackFilter2[i].resize(temp, smooth);
	}
	
	m_oldLookahead = -1;
}






extern "C"
{

// necessary for getting instance out of shared lib
PLUGIN_EXPORT Plugin* lmms_plugin_main(Model* parent, void* data)
{
	return new LimiterEffect(parent, static_cast<const Plugin::Descriptor::SubPluginFeatures::Key*>(data));
}

}

} // namespace lmms
