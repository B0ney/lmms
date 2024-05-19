/*
 * Mercury2.cpp
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


#include <QDomElement>

#include "Mercury2.h"
#include "AudioEngine.h"
#include "base64.h"
#include "Engine.h"
#include "Graph.h"
#include "InstrumentTrack.h"
#include "Knob.h"
#include "LedCheckBox.h"
#include "NotePlayHandle.h"
#include "PixmapButton.h"
#include "Song.h"

#include "embed.h"

#include "plugin_export.h"















#include <iostream>
#include <ostream>







/*
pushing up (slow)
push down occurs when push up minus signal surpasses grip strength
STORE this location
pushing down (fast)
push up occurs when passing negative of last stored location
repeat



push up speed
push down speed
grip strength
catch randomness
output volume
*/



















namespace lmms
{


extern "C"
{

Plugin::Descriptor PLUGIN_EXPORT mercury2_plugin_descriptor =
{
	LMMS_STRINGIFY( PLUGIN_NAME ),
	"Mercury2",
	QT_TRANSLATE_NOOP( "PluginBrowser",
				"Modal physical modelling synthesizer" ),
	"Lost Robot <r94231/at/gmail/dot/com>",
	0x0100,
	Plugin::Type::Instrument,
	new PluginPixmapLoader( "logo" ),
	nullptr,
	nullptr,
} ;

}


MSynth::MSynth( NotePlayHandle * nph, float sampleRate, int chordScatterDelay, float impulseLP, float impulseLPFalloff, float partialRand ) :
	m_sample_index( 0 ),
	m_sample_realindex( 0 ),
	m_nph( nph ),
	m_sampleRate( sampleRate ),
	m_noteDuration(0),
	m_noteReleaseTime(0),
	m_noteFreqChange(1),
	m_vibratoPhase(0),
	m_impulseCount(0),
	m_impulseNext(0),
	m_chordScatterDelay(chordScatterDelay),
	m_playPositionRandValue(fast_rand() / (float)FAST_RAND_MAX),
	m_fractalNoise(1),
	m_envExciterGainStorage(1),
	m_envVolumeGainStorage(1),
	m_coeffPrecalc(1),
	m_currentFreqInterp(440.f),
	m_currentFreqInterpCoef(44100.f / (50.f * m_sampleRate)),
	m_erodeLP(m_sampleRate),
	m_erodeHP(m_sampleRate)
{
	if (m_nph)
	{
		float velocityExciterLPChange = m_nph->getVolume() / 300.f + 0.333f;
		for (int i = 0; i < IMPULSE_MAX; ++i)
		{
			setExciterFilterCutoff(i, impulseLP * velocityExciterLPChange * powf(impulseLPFalloff, i));
		}
		
		m_currentFreqInterp = m_nph->frequency();
	}
	
	m_coeffPrecalc = -2.2 / (m_sampleRate * 0.001f);// -2.2 is slope constant
	
	m_noiseDelayBufSize = m_sampleRate + 4;
	m_noiseDelayBuf.resize(m_noiseDelayBufSize);
	
	for (int i = 0; i < 256; ++i)
	{
	    m_randFreqMult[i] = ((fast_rand() / (float)FAST_RAND_MAX) - 0.5f) * partialRand + 1.f;
	}
	m_randFreqMult[0] = 1;
	
	m_erodeBuf.resize(m_sampleRate);
}


MSynth::~MSynth()
{
}


void MSynth::nextNoteBuffer(sampleFrame * working_buffer, NotePlayHandle * currentNote, float decay, int partialNum, float partialFreqMax, float partialStretch, float partialSine, float partialFlatOdd, float partialInharmEnabled, float partialInharmConst, float partialInharmTrack, float partialInharmLows, bool partialInharmRetune, float partialDamp, float partialDampSlope, float partialFreqDamp, float partialFreqDampSlope, float partialRounding, float partialRoundingDivide, float partialRoundingCutoff, float partialComb1, float partialComb2, float partialCombShift, float partialCombBal, float partialCombInvert, float partialHarm2, float partialHarm3, float partialHarm5, float partialHarm7, float partialHarm11, float partialLP, float partialLPSlope, float partialRand, float playPosition, float playPositionRand, float brightness, float impulseNum, float impulseSpacing, float impulseLP, float impulseLPFalloff, float impulsePeriodicity, float impulseVol, float noiseVol, float noiseLP, float noiseHP, float noiseSat, float noisePeriodVol, float noisePeriodSpread, float noisePeriodShift, float noisePeriodDamp, float noiseNormalize, float exciterAttack, float exciterDecay1, float exciterBreakpoint, float exciterDecay2, float exciterSustain, float exciterRelease, float volumeAttack, float volumeDecay1, float volumeBreakpoint, float volumeDecay2, float volumeSustain, float volumeRelease, float tremoloAmount, float tremoloRate, float vibratoAmount, float vibratoRate, float vibratoShape, float vibratoTremolo, float vibratoFade, float vibratoFadeShape, float vibratoLengthMin, float erodeSelf, float erodeLP, float erodeHP, float erodeSine, float erodeFreq, float erodeEnable, float exciterFade, float release, float chordScatter, float chordScatterUniformity)
{
	const fpp_t frames = currentNote->framesLeftForCurrentPeriod();
	//const fpp_t fpp = Engine::audioEngine()->framesPerPeriod();
	const f_cnt_t offset = currentNote->noteOffset();
	
	setNoiseLP(0, noiseLP);
	setNoiseLP(1, noisePeriodDamp);
	setNoiseHP(0, noiseHP);
	
	m_erodeLP.setLowpass(erodeLP);
	m_erodeHP.setHighpass(erodeHP);

	for( fpp_t frame = offset; frame < frames + offset; ++frame )
	{
		if (m_chordScatterDelay > 0)
		{
			--m_chordScatterDelay;
			continue;
		}
	
		if( m_nph->isReleased() && frame >= m_nph->framesBeforeRelease() && !m_noteReleaseTime )
		{
			m_noteReleaseTime = m_noteDuration;
		}
		
		m_currentFreqInterp = m_currentFreqInterp * (1.f - m_currentFreqInterpCoef) + m_nph->frequency() * m_noteFreqChange * m_currentFreqInterpCoef;
		
		float exciterSamp = 0;
		
		if (noiseVol)
		{
			float noiseSamp = ((fast_rand() / (float)FAST_RAND_MAX) * 2.f - 1.f);
			
			float readLoc = m_noiseDelayWrite - qMax(1.f, m_sampleRate / (m_currentFreqInterp * noisePeriodShift));
			
			if (readLoc < 0) {readLoc += m_noiseDelayBufSize;}
			float readLocFrac = fraction(readLoc);

			// Read value from delay buffer
			float delayOut;
			if (readLoc < m_noiseDelayBufSize - 1)
			{
				delayOut = m_noiseDelayBuf[floor(readLoc)] * (1 - readLocFrac) + m_noiseDelayBuf[ceil(readLoc)] * readLocFrac;
			}
			else// For when the interpolation wraps around to the beginning of the buffer
			{
				delayOut = m_noiseDelayBuf[m_noiseDelayBufSize - 1] * (1 - readLocFrac) + m_noiseDelayBuf[0] * readLocFrac;
			}
			
			m_noiseDelayBuf[m_noiseDelayWrite] = noiseSamp + delayOut * noisePeriodSpread;
			runNoiseLP(1, m_noiseDelayBuf[m_noiseDelayWrite], m_noiseDelayBuf[m_noiseDelayWrite]);
			if (noiseSat > 0)
			{
				m_noiseDelayBuf[m_noiseDelayWrite] = fastSigmoid(m_noiseDelayBuf[m_noiseDelayWrite], 1.f / noiseSat);
			}
			
			noiseSamp += delayOut * noisePeriodVol;
			
			runNoiseLP(0, noiseSamp, noiseSamp);
			runNoiseHP(0, noiseSamp, noiseSamp);
			
			if (noiseSat > 0)
			{
				noiseSamp = fastSigmoid(noiseSamp, 1.f / noiseSat);
			}
			
			if (noiseNormalize && noisePeriodSpread != 0)
			{
				float temp = 1.f / (1.f - std::abs(noisePeriodSpread)) - 1.f;
				temp *= noisePeriodVol;
				temp += 1.f;
				exciterSamp += noiseSamp * (noiseVol / temp);
			}
			else
			{
				exciterSamp += noiseSamp * noiseVol;
			}
			
			++m_noiseDelayWrite;
			if (m_noiseDelayWrite >= m_noiseDelayBufSize)
			{
				m_noiseDelayWrite -= m_noiseDelayBufSize;
			}
		}
		
		float impulseSamp = 0;
		// Run all active impulse filters
		for (int i = 0; i < m_impulseCount; ++i)
		{
			float filtOut = 0;
			exciterFilter(i, 0, filtOut);
			impulseSamp += filtOut;
		}
		
		// Check if a new impulse should be added
		if (m_impulseCount < impulseNum)
		{
			if (m_noteDuration >= m_impulseNext)
			{
				++m_impulseCount;
				float filtOut = 0;
				// Send the impulse into the filter
				exciterFilter(m_impulseCount - 1, 1, filtOut);
				impulseSamp += filtOut;
				
				m_impulseNext = m_noteDuration + linearInterpolate(impulseSpacing * m_sampleRate * 0.001f, m_sampleRate / (m_nph->frequency() * m_noteFreqChange), impulsePeriodicity);
			}
		}
		exciterSamp += impulseSamp * impulseVol;
		
		
		
		
		
		
		
		//exciterSamp += m_fractalNoise.nextSample();
		
		
		
		
		exciterSamp *= calcEnvelope(m_noteDuration, exciterAttack * 0.001f * m_sampleRate, exciterDecay1 * 0.001f * m_sampleRate, exciterBreakpoint, exciterDecay2 * 0.001f * m_sampleRate, exciterSustain, exciterRelease * 0.001f * m_sampleRate, m_envExciterGainStorage);
		
		
		
		
		float resonatorOutput = 0;
		
		calcResonator(exciterSamp, resonatorOutput, qMin(partialNum, m_lastEnabled));
		
		float outVal = linearInterpolate(resonatorOutput, exciterSamp, exciterFade);
		
		// Apply release fade-out
		if( m_noteReleaseTime )
		{
			float r = release * 0.001f * m_sampleRate;
			if( m_noteDuration < r + m_noteReleaseTime )
			{
				float outMultThingy = (-1 / r) * (m_noteDuration - m_noteReleaseTime) + 1;
				outVal *= outMultThingy * outMultThingy;
			}
			else
			{
				outVal = 0;
			}
		}
		
		if (erodeEnable)
		{
			float filteredVal = m_erodeHP.update(m_erodeLP.update(outVal, 0), 0);
			float selfMove = qBound(0.f, (filteredVal + 1.f) * erodeSelf * erodeSelf * 4000.f * (m_sampleRate / 44100.f), m_sampleRate - 1.f);
			float sineMove = (std::sin(m_erodeSinePhase * F_2PI) + 1.f) * 0.5f * erodeSine * erodeSine * 2000.f * (m_sampleRate / 44100.f);
			float delayRead = m_erodeWrite - selfMove - sineMove - 2.f;
			float readFloor = floor(delayRead);
			float v0 = m_erodeBuf[realmod(readFloor - 1, m_sampleRate)];
			float v1 = m_erodeBuf[realmod(readFloor    , m_sampleRate)];
			float v2 = m_erodeBuf[realmod(readFloor + 1, m_sampleRate)];
			float v3 = m_erodeBuf[realmod(readFloor + 2, m_sampleRate)];
			float readValue = optimal4pInterpolate(v0, v1, v2, v3, fraction(delayRead));
			m_erodeBuf[m_erodeWrite] = outVal;
			outVal = readValue;
			if (++m_erodeWrite >= m_sampleRate) {m_erodeWrite -= m_sampleRate;}
			m_erodeSinePhase += m_currentFreqInterp * erodeFreq / m_sampleRate;
			if (m_erodeSinePhase >= 1) {m_erodeSinePhase -= 1;}
		}
		
		
		if (vibratoTremolo > 0)
		{
			float currentPhase = m_vibratoPhase + frame * (vibratoRate / m_sampleRate);
			float vibratoFadeResult = 1;
			float vibratoFadeProgress = m_noteDuration / (vibratoFade * m_sampleRate);
			if (vibratoFadeProgress < 1)
			{
				float vibratoFadeShapeSquared = vibratoFadeShape * vibratoFadeShape;
				vibratoFadeResult = -(vibratoFadeShapeSquared * vibratoFadeProgress) / (vibratoFadeProgress - vibratoFadeShapeSquared - 1);
			}
			float realVibratoTremolo = vibratoTremolo * vibratoFadeResult;
			outVal *= (qFastSin(currentPhase * 2 * 2 * M_PI) * 0.5 + 0.5) * realVibratoTremolo + (1.f - realVibratoTremolo);
		}
		
		outVal *= calcEnvelope(m_noteDuration, volumeAttack * 0.001f * m_sampleRate, volumeDecay1 * 0.001f * m_sampleRate, volumeBreakpoint, volumeDecay2 * 0.001f * m_sampleRate, volumeSustain, volumeRelease * 0.001f * m_sampleRate, m_envVolumeGainStorage);
		
		working_buffer[frame][0] += outVal;
		working_buffer[frame][1] += outVal;
		
		++m_noteDuration;
	}
	
	m_vibratoPhase += frames * (vibratoRate / m_sampleRate);
	m_vibratoPhase = fraction(m_vibratoPhase);
	
	float vibratoFadeResult = 1;
	float vibratoFadeProgress = m_noteDuration / (vibratoFade * m_sampleRate);
	if (vibratoFadeProgress < 1)
	{
		float vibratoFadeShapeSquared = vibratoFadeShape * vibratoFadeShape;
		vibratoFadeResult = -(vibratoFadeShapeSquared * vibratoFadeProgress) / (vibratoFadeProgress - vibratoFadeShapeSquared - 1);
	}
	
	float vibratoShapeSquared = vibratoShape * vibratoShape;
	m_noteFreqChange = (std::tanh(qFastSin(m_vibratoPhase * 2 * M_PI) * vibratoShapeSquared) / std::tanh(vibratoShapeSquared)) * vibratoFadeResult * vibratoAmount * vibratoAmount;
	m_noteFreqChange = std::pow(2, m_noteFreqChange);
}

#ifdef __SSE2__
void MSynth::calcResonator(float input, float &output, int partialNum)
{
    const int numIterations = partialNum / 4;

    __m128 inputVec = _mm_set_ps1(input);

    for (int i = 0; i < numIterations; ++i)
    {
        __m128 N1Vec, N2Vec;

        __m128 delay1Vec = _mm_load_ps(&m_partialDelay1[i * 4]);
        __m128 delay2Vec = _mm_load_ps(&m_partialDelay2[i * 4]);
        __m128 cosWFeedVec = _mm_load_ps(&m_partialCosWFeed[i * 4]);
        __m128 sinWFeedVec = _mm_load_ps(&m_partialSinWFeed[i * 4]);
        __m128 sinWVec = _mm_load_ps(&m_partialSinW[i * 4]);

        N1Vec = _mm_add_ps(_mm_sub_ps(_mm_mul_ps(cosWFeedVec, delay1Vec), _mm_mul_ps(sinWFeedVec, delay2Vec)), _mm_div_ps(inputVec, sinWVec));
        N2Vec = _mm_add_ps(_mm_mul_ps(sinWFeedVec, delay1Vec), _mm_mul_ps(cosWFeedVec, delay2Vec));

        _mm_store_ps(&m_partialDelay1[i * 4], N1Vec);
        _mm_store_ps(&m_partialDelay2[i * 4], N2Vec);

        __m128 gainVec = _mm_load_ps(&m_partialGain[i * 4]);
        __m128 N2GainVec = _mm_mul_ps(N2Vec, gainVec);
        output += N2GainVec[0] + N2GainVec[1] + N2GainVec[2] + N2GainVec[3];
    }

    // Process the remaining partials using regular floating point arithmetic
    for (int i = numIterations * 4; i < partialNum; ++i)
    {
        float N1 = m_partialCosWFeed[i] * m_partialDelay1[i] + -m_partialSinWFeed[i] * m_partialDelay2[i] + input / m_partialSinW[i];
        float N2 = m_partialSinWFeed[i] * m_partialDelay1[i] + m_partialCosWFeed[i] * m_partialDelay2[i];
        m_partialDelay1[i] = N1;
        m_partialDelay2[i] = N2;

        output += N2 * m_partialGain[i];
    }
}
#else
void MSynth::calcResonator(float input, float &output, int partialNum)
{
	for (int i = 0; i < partialNum; ++i)
	{
		float N1 = m_partialCosWFeed[i] * m_partialDelay1[i] + -m_partialSinWFeed[i] * m_partialDelay2[i] + (input * m_partialGain[i]) / m_partialSinW[i];
		float N2 = m_partialSinWFeed[i] * m_partialDelay1[i] + m_partialCosWFeed[i] * m_partialDelay2[i];
		m_partialDelay1[i] = N1;
		m_partialDelay2[i] = N2;

		output += N2;
	}
}
#endif

float MSynth::calcInharmConstant(float partialInharmConst, float partialInharmTrack, float partialInharmLows, float freq)
{
	const float temp = std::log2(freq / 440.f) - partialInharmLows;
	const float curve = 1;
	const float y = partialInharmTrack * (std::sqrt(temp * temp + curve) + partialInharmLows) + partialInharmConst;
	return std::pow(10.f, y);
}

float MSynth::calcInharmResult(float partialInharmConst, float partialInharmTrack, float partialInharmLows, float freq, float inharmConstant)
{
	if (freq > 220)
	{
		if (freq <= 440)
		{
			return freq;
		}
		const float otherConstant = calcInharmConstant(partialInharmConst, partialInharmTrack, partialInharmLows, freq * 0.5f);
		const float downOne = calcInharmResult(partialInharmConst, partialInharmTrack, partialInharmLows, freq * 0.5f, otherConstant);
		return (2.f * downOne * std::sqrt((1.f + 4.f * otherConstant) * (1.f + otherConstant))) / (1.f + otherConstant);
	}
	const float otherConstant = calcInharmConstant(partialInharmConst, partialInharmTrack, partialInharmLows, freq * 2.f);
	const float upOne = calcInharmResult(partialInharmConst, partialInharmTrack, partialInharmLows, freq * 2.f, otherConstant);
	return (upOne * std::sqrt((1.f + 4.f * inharmConstant) * (1.f + inharmConstant))) / (2.f * (1.f + 4.f * inharmConstant));
}

void MSynth::updatePartials(float noteFrequency, float decay, float partialStretch, float partialSine, float partialFlatOdd, float partialInharmEnabled, float partialInharmConst, float partialInharmTrack, float partialInharmLows, bool partialInharmRetune, float partialDamp, float partialDampSlope, float partialFreqDamp, float partialFreqDampSlope, float partialRounding, float partialRoundingDivide, float partialRoundingCutoff, float partialComb1, float partialComb2, float partialCombShift, float partialCombBal, float partialCombInvert, float partialHarm2, float partialHarm3, float partialHarm5, float partialHarm7, float partialHarm11, float partialLP, float partialLPSlope, float partialRand, float playPosition, float playPositionRand, float brightness, float partialFreqMax, int partialNum)
{
	float trueNoteFreq = noteFrequency * m_noteFreqChange;
	
	float inharmConstant;
	float inharmFundamentalLock;
	
	if (partialInharmEnabled)
	{
		inharmConstant = calcInharmConstant(partialInharmConst, partialInharmTrack, partialInharmLows, trueNoteFreq);
		inharmFundamentalLock = std::sqrt(1 + inharmConstant);
		if (partialInharmRetune)
		{
			trueNoteFreq = calcInharmResult(partialInharmConst, partialInharmTrack, partialInharmLows, trueNoteFreq, inharmConstant);
		}
	}

	float lastPartialFreq = 0;
	for (int i = 0; i < PARTIAL_MAX; ++i)
	{
		float partialFreq = trueNoteFreq;
		float partialOffsetRatio = i;
		if (partialInharmEnabled)
		{
			partialOffsetRatio = (i + 1) * std::sqrt(1 + inharmConstant * (i + 1) * (i + 1)) / inharmFundamentalLock;
			partialOffsetRatio -= 1;
		}
		partialOffsetRatio *= partialStretch;
		if (partialSine)
		{
			partialOffsetRatio *= qFastSin(i * partialSine * F_2PI) * 0.5f + 1.f;
		}
		partialFreq += trueNoteFreq * partialOffsetRatio;
		if (i % 2 == 0)
		{
			partialFreq -= partialFlatOdd * trueNoteFreq;
		}
		if (partialFreq > partialFreqMax || i >= partialNum)
		{
			m_partialFreq[i] = partialFreq;
			m_partialGain[i] = 0;
			m_partialRadius[i] = 0;
			
			const float w = F_2PI * partialFreq / m_sampleRate;
			m_partialCosW[i] = qFastCos(w);
			m_partialSinW[i] = qFastSin(w);
			m_partialCosWFeed[i] = 0;
			m_partialSinWFeed[i] = 0;
			m_partialDelay1[i] = 0;
			m_partialDelay2[i] = 0;
			continue;
		}
		if (partialRounding && partialFreq < partialRoundingCutoff)
		{
			float octaveDif = std::log2(partialFreq / noteFrequency);
			float octaveRounding = partialRounding / 1200.f;
			if (partialRoundingDivide <= partialFreq)
			{
				octaveRounding /= std::floor(partialFreq / partialRoundingDivide);
			}
			octaveDif = std::round(octaveDif / octaveRounding) * octaveRounding;
			partialFreq = noteFrequency * std::exp2(octaveDif) * m_noteFreqChange;
		}

		float partialGain = trueNoteFreq / m_sampleRate;
		float comb1Gain;
		float comb2Gain;
		if (partialCombBal < 0)
		{
			comb1Gain = 1.f;
			comb2Gain = partialCombBal + 1.f;
		}
		else
		{
			comb1Gain = -partialCombBal + 1.f;
			comb2Gain = 1.f;
		}
		partialGain *= comb1Gain * m_comb1GainPre[i] + comb2Gain * m_comb2GainPre[i] * (partialCombInvert ? -1 : 1);
		if (playPosition > 0 && playPosition < 1)
		{
			float effectivePlayPosition = linearInterpolate(1.f, m_playPositionRandValue, playPositionRand) * playPosition;
		
			// Find partial amplitude depending on play position, multiplied by pi/2 for normalization assuming playPosition of 0
			// We multiply the end result by the harmonic number (i+1) because the amplitudes already follow a saw wave by default
			partialGain *= F_PI / ((i+1) * (i+1) * F_PI * F_PI * effectivePlayPosition * (1.f - effectivePlayPosition)) * qFastSin((i+1) * F_PI * effectivePlayPosition) * (i+1);
		}
		//partialGain *= std::pow(partialLPSlope, std::log2(partialFreq / partialLP));
		partialGain *= 1.f / std::sqrt(1.f + std::pow(partialFreq / partialLP, 2 * partialLPSlope));
		partialGain *= std::pow(i + 1, brightness);

		float partialRadius = decay;
		if (partialDamp < 1.f)
		{
			float partialDampingMult = powf(partialDamp, i / partialDampSlope + 1.f);
			partialRadius *= partialDampingMult;
		}
		if (partialFreqDampSlope < 1 && partialFreqDamp < 20000)// Magic number alert, fix this
		{
			float octaveDif = std::log2(partialFreq / partialFreqDamp);
			float partialDampingMult = powf(partialFreqDampSlope, octaveDif + 1.f);
			partialRadius *= partialDampingMult;
		}
		partialRadius = expf((-1000.f / m_sampleRate) / partialRadius);
		
		if (partialRounding && partialFreq == lastPartialFreq)
		{
			m_partialGain[i] = 0;
			m_partialRadius[i] = 0;
		}
		
		partialFreq *= m_randFreqMult[i];

		m_partialFreq[i] = partialFreq;
		m_partialGain[i] = partialGain;
		m_partialRadius[i] = partialRadius;
		
		const float w = F_2PI * partialFreq / m_sampleRate;
		m_partialCosW[i] = qFastCos(w);
		m_partialSinW[i] = qFastSin(w);
		m_partialCosWFeed[i] = m_partialCosW[i] * partialRadius;
		m_partialSinWFeed[i] = m_partialSinW[i] * partialRadius;
		
		lastPartialFreq = partialFreq;
		
		if (m_partialGain[i] > 0)
		{
			m_lastEnabled = i + 1;
		}
	}
	
	if (partialHarm2 != 1 || partialHarm3 != 1 || partialHarm5 != 1 || partialHarm7 != 1 || partialHarm11 != 1)
	{
		for (int i = 1; i <= PARTIAL_MAX; ++i)// Start at 1 instead of 0 so we have the right harmonic number
		{
			if (i % 2 == 0) {m_partialGain[i-1] *= partialHarm2;}
			if (i % 3 == 0) {m_partialGain[i-1] *= partialHarm3;}
			if (i % 5 == 0) {m_partialGain[i-1] *= partialHarm5;}
			if (i % 7 == 0) {m_partialGain[i-1] *= partialHarm7;}
			if (i % 11 == 0) {m_partialGain[i-1] *= partialHarm11;}
		}
	}
}

void MSynth::updateComb(int partialNum, float partialFreqMax, float partialComb1, float partialComb2, float partialCombShift, float partialCombBal, float partialCombInvert)
{
	float combCombine = partialComb1 + partialComb2;
	float trueCombShift = fmod(partialCombShift * combCombine, combCombine);
	float combTimeLeft = combCombine - trueCombShift;
	
	for (int i = 0; i < partialNum; ++i)
	{
		// Ignore if partial frequency is too high
		if (m_partialFreq[i] > partialFreqMax)
		{
			++combTimeLeft;
			if (combTimeLeft >= combCombine) {combTimeLeft -= combCombine;}

			continue;
		}
		
		if (combTimeLeft < partialComb1)// Comb 1
		{
			float tempDiff = partialComb1 - combTimeLeft;
			if (tempDiff < 1)// Moving into Comb 2
			{
				m_comb1GainPre[i] = tempDiff;
				m_comb2GainPre[i] = -tempDiff + 1.f;
			}
			else
			{
				m_comb1GainPre[i] = 1.f;
				m_comb2GainPre[i] = 0.f;
			}
		}
		else// Comb 2
		{
			float tempDiff = combCombine - combTimeLeft;
			if (tempDiff < 1)// Moving into Comb 1
			{
				m_comb1GainPre[i] = -tempDiff + 1.f;
				m_comb2GainPre[i] = tempDiff;
			}
			else
			{
				m_comb1GainPre[i] = 0.f;
				m_comb2GainPre[i] = 1.f;
			}
		}
		++combTimeLeft;
		if (combTimeLeft >= combCombine) {combTimeLeft -= combCombine;}
		
	}
}

void MSynth::exciterFilter(int filterNum, float input, float &output)
{
	output = m_exciterFiltB[filterNum][0] * input
		+ m_exciterFiltB[filterNum][1] * m_exciterFiltX[filterNum][0]
		+ m_exciterFiltB[filterNum][2] * m_exciterFiltX[filterNum][1]
		- m_exciterFiltA[filterNum][1] * m_exciterFiltY[filterNum][0]
		- m_exciterFiltA[filterNum][2] * m_exciterFiltY[filterNum][1];

	m_exciterFiltX[filterNum][1] = m_exciterFiltX[filterNum][0];
	m_exciterFiltX[filterNum][0] = input;
	m_exciterFiltY[filterNum][1] = m_exciterFiltY[filterNum][0];
	m_exciterFiltY[filterNum][0] = output;
}

void MSynth::setExciterFilterCutoff(int filterNum, float cutoffFrequency)
{
	float omega = M_PI * 2 * cutoffFrequency / m_sampleRate;
	float sn = sin(omega);
	float cs = cos(omega);
	float alpha = sn / 1.414;
	
	m_exciterFiltA[filterNum][0] = 1.0 + alpha;

	m_exciterFiltB[filterNum][0] = ((1.0 - cs) / 2.0) / m_exciterFiltA[filterNum][0];
	m_exciterFiltB[filterNum][1] = (1.0 - cs) / m_exciterFiltA[filterNum][0];
	m_exciterFiltB[filterNum][2] = ((1.0 - cs) / 2.0) / m_exciterFiltA[filterNum][0];
	m_exciterFiltA[filterNum][1] = (-2.0 * cs) / m_exciterFiltA[filterNum][0];
	m_exciterFiltA[filterNum][2] = (1.0 - alpha) / m_exciterFiltA[filterNum][0];
}

void MSynth::runNoiseLP(int index, float input, float &output)
{
	output = m_noiseLPB[index][0] * input
		+ m_noiseLPB[index][1] * m_noiseLPX[index][0]
		+ m_noiseLPB[index][2] * m_noiseLPX[index][1]
		- m_noiseLPA[index][1] * m_noiseLPY[index][0]
		- m_noiseLPA[index][2] * m_noiseLPY[index][1];

	m_noiseLPX[index][1] = m_noiseLPX[index][0];
	m_noiseLPX[index][0] = input;
	m_noiseLPY[index][1] = m_noiseLPY[index][0];
	m_noiseLPY[index][0] = output;
}

void MSynth::setNoiseLP(int index, float cutoffFrequency)
{
	float omega = M_PI * 2 * cutoffFrequency / m_sampleRate;
	float sn = sin(omega);
	float cs = cos(omega);
	float alpha = sn / 1.414;
	
	m_noiseLPA[index][0] = 1.0 + alpha;

	m_noiseLPB[index][0] = ((1.0 - cs) / 2.0) / m_noiseLPA[index][0];
	m_noiseLPB[index][1] = (1.0 - cs) / m_noiseLPA[index][0];
	m_noiseLPB[index][2] = ((1.0 - cs) / 2.0) / m_noiseLPA[index][0];
	m_noiseLPA[index][1] = (-2.0 * cs) / m_noiseLPA[index][0];
	m_noiseLPA[index][2] = (1.0 - alpha) / m_noiseLPA[index][0];
}

void MSynth::runNoiseHP(int index, float input, float &output)
{
	output = m_noiseHPB[index][0] * input
		+ m_noiseHPB[index][1] * m_noiseHPX[index][0]
		+ m_noiseHPB[index][2] * m_noiseHPX[index][1]
		- m_noiseHPA[index][1] * m_noiseHPY[index][0]
		- m_noiseHPA[index][2] * m_noiseHPY[index][1];

	m_noiseHPX[index][1] = m_noiseHPX[index][0];
	m_noiseHPX[index][0] = input;
	m_noiseHPY[index][1] = m_noiseHPY[index][0];
	m_noiseHPY[index][0] = output;
}

void MSynth::setNoiseHP(int index, float cutoffFrequency)
{
	float omega = M_PI * 2 * cutoffFrequency / m_sampleRate;
	float sn = sin(omega);
	float cs = cos(omega);
	float alpha = sn / 1.414;
	
	m_noiseHPA[index][0] = 1.0 + alpha;

	m_noiseHPB[index][0] = ((1.0 + cs) / 2.0) / m_noiseHPA[index][0];
	m_noiseHPB[index][1] = -(1.0 + cs) / m_noiseHPA[index][0];
	m_noiseHPB[index][2] = ((1.0 + cs) / 2.0) / m_noiseHPA[index][0];
	m_noiseHPA[index][1] = (-2.0 * cs) / m_noiseHPA[index][0];
	m_noiseHPA[index][2] = (1.0 - alpha) / m_noiseHPA[index][0];
}

/***********************************************************************
*
*	class Mercury2
*
*	lmms - plugin 
*
***********************************************************************/


Mercury2::Mercury2( InstrumentTrack * _instrument_track ) :
	Instrument( _instrument_track, &mercury2_plugin_descriptor ),
	m_decay( 100, 1, 10000, 0.01, this, tr( "Decay" ) ),
	m_partialNum( PARTIAL_MAX, 1, PARTIAL_MAX, 1, this, tr( "Number of Partials" ) ),
	m_partialFreqMax( 20000, 8000, 20000, 0.01, this, tr( "Maximum Partial Frequency" ) ),
	m_partialStretch( 1, 0, 8, 0.001, this, tr( "Partial Stretch" ) ),
	m_partialSine( 0, 0, 1, 0.0001, this, tr( "Partial Sine" ) ),
	m_partialFlatOdd( 0, -1, 1, 0.0001, this, tr( "Flatten Odd Partials" ) ),
	m_partialInharmEnabled( 0, 0, 1, 1, this, tr( "Inharmonicity Enabled" ) ),
	m_partialInharmConst( -3.5, -8, 0, 0.00001, this, tr( "Inharmonicity Constant" ) ),
	m_partialInharmTrack( 0.48, -1, 2, 0.00001, this, tr( "Inharmonicity Tracking" ) ),
	m_partialInharmLows( -2.4, -5, 5, 0.00001, this, tr( "Low Note Inharmonicity" ) ),
	m_partialInharmRetune( 1, 0, 1, 1, this, tr( "Inharmonicity Retune" ) ),
	m_partialDamp( 1, 0, 1, 0.001, this, tr( "Partial Damping" ) ),
	m_partialDampSlope( 2, 1, 8, 0.001, this, tr( "Partial Damping Slope" ) ),
	m_partialFreqDamp( 20000, 20, 20000, 0.01, this, tr( "Partial Frequency Damping" ) ),
	m_partialFreqDampSlope( 1, 0, 1, 0.001, this, tr( "Partial Frequency Damping Slope" ) ),
	m_partialRounding( 0, 0, 1200, 0.001, this, tr( "Partial Rounding" ) ),
	m_partialRoundingDivide( 880, 220, 20000, 0.001, this, tr( "Partial Rounding Division Frequency" ) ),
	m_partialRoundingCutoff( 20000, 0, 20000, 0.001, this, tr( "Partial Rounding Cutoff" ) ),
	m_partialComb1( 4, 1, 16, 0.0001, this, tr( "Comb 1" ) ),
	m_partialComb2( 4, 1, 16, 0.0001, this, tr( "Comb 2" ) ),
	m_partialCombShift( 0, 0, 1, 0.0001, this, tr( "Comb Shift" ) ),
	m_partialCombBal( 0, -1, 1, 0.0001, this, tr( "Comb Balance" ) ),
	m_partialCombInvert( 0, 0, 1, 1, this, tr( "Comb Invert" ) ),
	m_partialHarm2( 1, 0, 1, 0.0001, this, tr( "2nd Harmonic" ) ),
	m_partialHarm3( 1, 0, 1, 0.0001, this, tr( "3rd Harmonic" ) ),
	m_partialHarm5( 1, 0, 1, 0.0001, this, tr( "5th Harmonic" ) ),
	m_partialHarm7( 1, 0, 1, 0.0001, this, tr( "7th Harmonic" ) ),
	m_partialHarm11( 1, 0, 1, 0.0001, this, tr( "11th Harmonic" ) ),
	m_partialLP( 24000, 0, 24000, 0.0001, this, tr( "Partial Lowpass" ) ),
	m_partialLPSlope( 3, 0, 24, 0.0001, this, tr( "Partial Lowpass Slope" ) ),
	m_partialRand( 0, 0, 0.1, 0.0001, this, tr( "Partial Randomness" ) ),
	m_playPosition( 0, 0, 1, 0.0001, this, tr( "Play Position" ) ),
	m_playPositionRand( 0, 0, 1, 0.0001, this, tr( "Play Position Randomness" ) ),
	m_brightness( 0, -1, 1, 0.0001, this, tr( "Brightness" ) ),
	m_impulseNum( 1, 1, IMPULSE_MAX, 1, this, tr( "Number of Impulses" ) ),
	m_impulseSpacing( 4, 0, 40, 0.0001, this, tr( "Impulse Spacing" ) ),
	m_impulseLP( 20000, 20, 20000, 0.1, this, tr( "Impulse Lowpass" ) ),
	m_impulseLPFalloff( 0.8, 0.25, 1, 0.0001, this, tr( "Impulse Lowpass Falloff" ) ),
	m_impulsePeriodicity( 1, 0, 1, 0.0001, this, tr( "Impulse Periodicity" ) ),
	m_impulseVol( 1, 0, 1, 0.0001, this, tr( "Impulse Volume" ) ),
	m_noiseVol( 0, 0, 1, 0.0001, this, tr( "Noise Volume" ) ),
	m_noiseLP( 20000, 10, 20000, 0.001, this, tr( "Noise Lowpass" ) ),
	m_noiseHP( 10, 10, 20000, 0.001, this, tr( "Noise Highpass" ) ),
	m_noiseSat( 0, 0, 10, 0.0001, this, tr( "Noise Saturation" ) ),
	m_noisePeriodVol( 0, 0, 1, 0.0001, this, tr( "Noise Periodicity Volume" ) ),
	m_noisePeriodSpread( 0, -0.999, 0.999, 0.0001, this, tr( "Noise Periodicity Spread" ) ),
	m_noisePeriodShift( 1, 0.25, 4, 0.0001, this, tr( "Noise Periodicity Shift" ) ),
	m_noisePeriodDamp( 20000, 10, 20000, 0.001, this, tr( "Noise Periodicity Damping" ) ),
	m_noiseNormalize( 1, 0, 1, 1, this, tr( "Noise Normalization" ) ),
	m_exciterAttack( 0, 0, 4000, 0.01, this, tr( "Exciter Attack" ) ),
	m_exciterDecay1( 0, 0, 2000, 0.01, this, tr( "Exciter Decay 1" ) ),
	m_exciterBreakpoint( 1, 0, 1, 0.0001, this, tr( "Exciter Breakpoint" ) ),
	m_exciterDecay2( 1, 0, 50, 0.0001, this, tr( "Exciter Decay 2" ) ),
	m_exciterSustain( 0, 0, 1, 0.0001, this, tr( "Exciter Sustain" ) ),
	m_exciterRelease( 50, 0, 4000, 0.01, this, tr( "Exciter Release" ) ),
	m_volumeAttack( 0, 0, 4000, 0.01, this, tr( "Volume Attack" ) ),
	m_volumeDecay1( 0, 0, 2000, 0.01, this, tr( "Volume Decay 1" ) ),
	m_volumeBreakpoint( 1, 0, 1, 0.0001, this, tr( "Volume Breakpoint" ) ),
	m_volumeDecay2( 0, 0, 50, 0.0001, this, tr( "Volume Decay 2" ) ),
	m_volumeSustain( 1, 0, 1, 0.0001, this, tr( "Volume Sustain" ) ),
	m_volumeRelease( 200, 0, 4000, 0.01, this, tr( "Volume Release" ) ),
	m_tremoloAmount( 0, 0, 1, 0.0001, this, tr( "Tremolo Amount" ) ),
	m_tremoloRate( 0, 0, 40, 0.0001, this, tr( "Tremolo Rate" ) ),
	m_vibratoAmount( 0, 0, 1, 0.0001, this, tr( "Vibrato Amount" ) ),
	m_vibratoRate( 7, 0.25, 20, 0.0001, this, tr( "Vibrato Rate" ) ),
	m_vibratoShape( 1.0, 0.1, 3, 0.0001, this, tr( "Vibrato Shape" ) ),
	m_vibratoTremolo( 0, 0, 1, 0.0001, this, tr( "Vibrato Tremolo" ) ),
	m_vibratoFade( 0.15, 0, 4, 0.0001, this, tr( "Vibrato Fade" ) ),
	m_vibratoFadeShape( 0.7, 0.01, 2, 0.0001, this, tr( "Vibrato Fade Shape" ) ),
	m_vibratoLengthMin( 0, 0, 1, 0.0001, this, tr( "Vibrato Note Length Minimum" ) ),
	m_erodeSelf( 0, 0, 1, 0.0001, this, tr( "Erode Self" ) ),
	m_erodeLP( 20000, 20, 20000, 0.01, this, tr( "Erosion Lowpass" ) ),
	m_erodeHP( 20, 20, 20000, 0.01, this, tr( "Erosion Highpass" ) ),
	m_erodeSine( 0, 0, 1, 0.0001, this, tr( "Erode Sine" ) ),
	m_erodeFreq( 1, 0.125, 8, 0.0001, this, tr( "Erosion Sine Frequency" ) ),
	m_erodeEnable( 0, 0, 1, 1, this, tr( "Enable Erosion" ) ),
	m_exciterFade( 0, 0, 1, 0.0001, this, tr( "Exciter Fade" ) ),
	m_release( 50, 0, 8000, 0.01, this, tr( "Release" ) ),
	m_chordScatter( 0, -0.1, 0.1, 0.0001, this, tr( "Chord Scatter" ) ),
	m_chordScatterUniformity( 0, 0, 1, 0.0001, this, tr( "Chord Scatter Uniformity" ) ),
	m_volume(100, 0, 200, 0.0001, this, tr("Volume")),
	m_tremoloCurrentPhase(0),
	m_lastNoteFreq(1.f)
{
	m_decay.setScaleLogarithmic(true);
	m_partialStretch.setScaleLogarithmic(true);
	m_partialFreqDamp.setScaleLogarithmic(true);
	m_partialRoundingDivide.setScaleLogarithmic(true);
	m_partialRoundingCutoff.setScaleLogarithmic(true);
	m_partialComb1.setScaleLogarithmic(true);
	m_partialComb2.setScaleLogarithmic(true);
	m_partialLP.setScaleLogarithmic(true);
	m_partialLPSlope.setScaleLogarithmic(true);
	m_partialRand.setScaleLogarithmic(true);
	m_playPositionRand.setScaleLogarithmic(true);
	m_brightness.setScaleLogarithmic(true);
	m_impulseSpacing.setScaleLogarithmic(true);
	m_release.setScaleLogarithmic(true);
	m_impulseLP.setScaleLogarithmic(true);
	m_noiseLP.setScaleLogarithmic(true);
	m_noiseHP.setScaleLogarithmic(true);
	m_noisePeriodShift.setScaleLogarithmic(true);
	m_chordScatter.setScaleLogarithmic(true);
	m_tremoloRate.setScaleLogarithmic(true);
	m_vibratoAmount.setScaleLogarithmic(true);
	m_vibratoShape.setScaleLogarithmic(true);
	m_vibratoTremolo.setScaleLogarithmic(true);
	m_vibratoFade.setScaleLogarithmic(true);
	m_vibratoLengthMin.setScaleLogarithmic(true);
	m_erodeLP.setScaleLogarithmic(true);
	m_erodeHP.setScaleLogarithmic(true);
	m_erodeFreq.setScaleLogarithmic(true);
	m_exciterAttack.setScaleLogarithmic(true);
	m_exciterDecay1.setScaleLogarithmic(true);
	m_exciterDecay2.setScaleLogarithmic(true);
	m_exciterRelease.setScaleLogarithmic(true);
	m_volumeAttack.setScaleLogarithmic(true);
	m_volumeDecay1.setScaleLogarithmic(true);
	m_volumeDecay2.setScaleLogarithmic(true);
	m_volumeRelease.setScaleLogarithmic(true);
	
	/*connect(&m_decay, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialStretch, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialSine, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialDamp, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialRounding, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialNum, SIGNAL(dataChanged()), this, SLOT(updatePartials()));*/
	
	InstrumentPlayHandle * iph = new InstrumentPlayHandle( this, _instrument_track );
	Engine::audioEngine()->addPlayHandle( iph );
}







void Mercury2::saveSettings( QDomDocument & _doc, QDomElement & _this )
{
	// Save plugin version
	_this.setAttribute( "version", "1.0.0" );

	m_volume.saveSettings( _doc, _this, "volume" );
	m_decay.saveSettings( _doc, _this, "decay" );
	m_partialNum.saveSettings( _doc, _this, "partialNum" );
	m_partialFreqMax.saveSettings( _doc, _this, "partialFreqMax" );
	m_partialStretch.saveSettings( _doc, _this, "partialStretch" );
	m_partialSine.saveSettings( _doc, _this, "partialSine" );
	m_partialFlatOdd.saveSettings( _doc, _this, "partialFlatOdd" );
	m_partialInharmEnabled.saveSettings( _doc, _this, "partialInharmEnabled" );
	m_partialInharmConst.saveSettings( _doc, _this, "partialInharmConst" );
	m_partialInharmTrack.saveSettings( _doc, _this, "partialInharmTrack" );
	m_partialInharmLows.saveSettings( _doc, _this, "partialInharmLows" );
	m_partialInharmRetune.saveSettings( _doc, _this, "partialInharmRetune" );
	m_partialDamp.saveSettings( _doc, _this, "partialDamp" );
	m_partialDampSlope.saveSettings( _doc, _this, "partialDampSlope" );
	m_partialFreqDamp.saveSettings( _doc, _this, "partialFreqDamp" );
	m_partialFreqDampSlope.saveSettings( _doc, _this, "partialFreqDampSlope" );
	m_partialRounding.saveSettings( _doc, _this, "partialRounding" );
	m_partialRoundingDivide.saveSettings( _doc, _this, "partialRoundingDivide" );
	m_partialRoundingCutoff.saveSettings( _doc, _this, "partialRoundingCutoff" );
	m_partialComb1.saveSettings( _doc, _this, "partialComb1" );
	m_partialComb2.saveSettings( _doc, _this, "partialComb2" );
	m_partialCombShift.saveSettings( _doc, _this, "partialCombShift" );
	m_partialCombBal.saveSettings( _doc, _this, "partialCombBal" );
	m_partialCombInvert.saveSettings( _doc, _this, "partialCombInvert" );
	m_partialHarm2.saveSettings( _doc, _this, "partialHarm2" );
	m_partialHarm3.saveSettings( _doc, _this, "partialHarm3" );
	m_partialHarm5.saveSettings( _doc, _this, "partialHarm5" );
	m_partialHarm7.saveSettings( _doc, _this, "partialHarm7" );
	m_partialHarm11.saveSettings( _doc, _this, "partialHarm11" );
	m_partialLP.saveSettings( _doc, _this, "partialLP" );
	m_partialLPSlope.saveSettings( _doc, _this, "partialLPSlope" );
	m_partialRand.saveSettings( _doc, _this, "partialRand" );
	m_playPosition.saveSettings( _doc, _this, "playPosition" );
	m_playPositionRand.saveSettings( _doc, _this, "playPositionRand" );
	m_brightness.saveSettings( _doc, _this, "brightness" );
	m_impulseNum.saveSettings( _doc, _this, "impulseNum" );
	m_impulseSpacing.saveSettings( _doc, _this, "impulseSpacing" );
	m_impulseLP.saveSettings( _doc, _this, "impulseLP" );
	m_impulseLPFalloff.saveSettings( _doc, _this, "impulseLPFalloff" );
	m_impulsePeriodicity.saveSettings( _doc, _this, "impulsePeriodicity" );
	m_impulseVol.saveSettings( _doc, _this, "impulseVol" );
	m_noiseVol.saveSettings( _doc, _this, "noiseVol" );
	m_noiseLP.saveSettings( _doc, _this, "noiseLP" );
	m_noiseHP.saveSettings( _doc, _this, "noiseHP" );
	m_noiseSat.saveSettings( _doc, _this, "noiseSat" );
	m_noisePeriodVol.saveSettings( _doc, _this, "noisePeriodVol" );
	m_noisePeriodSpread.saveSettings( _doc, _this, "noisePeriodSpread" );
	m_noisePeriodShift.saveSettings( _doc, _this, "noisePeriodShift" );
	m_noisePeriodDamp.saveSettings( _doc, _this, "noisePeriodDamp" );
	m_noiseNormalize.saveSettings( _doc, _this, "noiseNormalize" );
	m_exciterAttack.saveSettings( _doc, _this, "exciterAttack" );
	m_exciterDecay1.saveSettings( _doc, _this, "exciterDecay1" );
	m_exciterBreakpoint.saveSettings( _doc, _this, "exciterBreakpoint" );
	m_exciterDecay2.saveSettings( _doc, _this, "exciterDecay2" );
	m_exciterSustain.saveSettings( _doc, _this, "exciterSustain" );
	m_exciterRelease.saveSettings( _doc, _this, "exciterRelease" );
	m_volumeAttack.saveSettings( _doc, _this, "volumeAttack" );
	m_volumeDecay1.saveSettings( _doc, _this, "volumeDecay1" );
	m_volumeBreakpoint.saveSettings( _doc, _this, "volumeBreakpoint" );
	m_volumeDecay2.saveSettings( _doc, _this, "volumeDecay2" );
	m_volumeSustain.saveSettings( _doc, _this, "volumeSustain" );
	m_volumeRelease.saveSettings( _doc, _this, "volumeRelease" );
	m_tremoloAmount.saveSettings( _doc, _this, "tremoloAmount" );
	m_tremoloRate.saveSettings( _doc, _this, "tremoloRate" );
	m_vibratoAmount.saveSettings( _doc, _this, "vibratoAmount" );
	m_vibratoRate.saveSettings( _doc, _this, "vibratoRate" );
	m_vibratoShape.saveSettings( _doc, _this, "vibratoShape" );
	m_vibratoTremolo.saveSettings( _doc, _this, "vibratoTremolo" );
	m_vibratoFade.saveSettings( _doc, _this, "vibratoFade" );
	m_vibratoFadeShape.saveSettings( _doc, _this, "vibratoFadeShape" );
	m_vibratoLengthMin.saveSettings( _doc, _this, "vibratoLengthMin" );
	m_erodeSelf.saveSettings( _doc, _this, "erodeSelf" );
	m_erodeLP.saveSettings( _doc, _this, "erodeLP" );
	m_erodeHP.saveSettings( _doc, _this, "erodeHP" );
	m_erodeSine.saveSettings( _doc, _this, "erodeSine" );
	m_erodeFreq.saveSettings( _doc, _this, "erodeFreq" );
	m_erodeEnable.saveSettings( _doc, _this, "erodeEnable" );
	m_exciterFade.saveSettings( _doc, _this, "exciterFade" );
	m_release.saveSettings( _doc, _this, "release" );
	m_chordScatter.saveSettings( _doc, _this, "chordScatter" );
	m_chordScatterUniformity.saveSettings( _doc, _this, "chordScatterUniformity" );
}




void Mercury2::loadSettings( const QDomElement & _this )
{
	m_volume.loadSettings( _this, "volume" );
	m_decay.loadSettings( _this, "decay" );
	m_partialNum.loadSettings( _this, "partialNum" );
	m_partialFreqMax.loadSettings( _this, "partialFreqMax" );
	m_partialStretch.loadSettings( _this, "partialStretch" );
	m_partialSine.loadSettings( _this, "partialSine" );
	m_partialFlatOdd.loadSettings( _this, "partialFlatOdd" );
	m_partialInharmEnabled.loadSettings( _this, "partialInharmEnabled" );
	m_partialInharmConst.loadSettings( _this, "partialInharmConst" );
	m_partialInharmTrack.loadSettings( _this, "partialInharmTrack" );
	m_partialInharmLows.loadSettings( _this, "partialInharmLows" );
	m_partialInharmRetune.loadSettings( _this, "partialInharmRetune" );
	m_partialDamp.loadSettings( _this, "partialDamp" );
	m_partialDampSlope.loadSettings( _this, "partialDampSlope" );
	m_partialFreqDamp.loadSettings( _this, "partialFreqDamp" );
	m_partialFreqDampSlope.loadSettings( _this, "partialFreqDampSlope" );
	m_partialRounding.loadSettings( _this, "partialRounding" );
	m_partialRoundingDivide.loadSettings( _this, "partialRoundingDivide" );
	m_partialRoundingCutoff.loadSettings( _this, "partialRoundingCutoff" );
	m_partialComb1.loadSettings( _this, "partialComb1" );
	m_partialComb2.loadSettings( _this, "partialComb2" );
	m_partialCombShift.loadSettings( _this, "partialCombShift" );
	m_partialCombBal.loadSettings( _this, "partialCombBal" );
	m_partialCombInvert.loadSettings( _this, "partialCombInvert" );
	m_partialHarm2.loadSettings( _this, "partialHarm2" );
	m_partialHarm3.loadSettings( _this, "partialHarm3" );
	m_partialHarm5.loadSettings( _this, "partialHarm5" );
	m_partialHarm7.loadSettings( _this, "partialHarm7" );
	m_partialHarm11.loadSettings( _this, "partialHarm11" );
	m_partialLP.loadSettings( _this, "partialLP" );
	m_partialLPSlope.loadSettings( _this, "partialLPSlope" );
	m_partialRand.loadSettings( _this, "partialRand" );
	m_playPosition.loadSettings( _this, "playPosition" );
	m_playPositionRand.loadSettings( _this, "playPositionRand" );
	m_brightness.loadSettings( _this, "brightness" );
	m_impulseNum.loadSettings( _this, "impulseNum" );
	m_impulseSpacing.loadSettings( _this, "impulseSpacing" );
	m_impulseLP.loadSettings( _this, "impulseLP" );
	m_impulseLPFalloff.loadSettings( _this, "impulseLPFalloff" );
	m_impulsePeriodicity.loadSettings( _this, "impulsePeriodicity" );
	m_impulseVol.loadSettings( _this, "impulseVol" );
	m_noiseVol.loadSettings( _this, "noiseVol" );
	m_noiseLP.loadSettings( _this, "noiseLP" );
	m_noiseHP.loadSettings( _this, "noiseHP" );
	m_noiseSat.loadSettings( _this, "noiseSat" );
	m_noisePeriodVol.loadSettings( _this, "noisePeriodVol" );
	m_noisePeriodSpread.loadSettings( _this, "noisePeriodSpread" );
	m_noisePeriodShift.loadSettings( _this, "noisePeriodShift" );
	m_noisePeriodDamp.loadSettings( _this, "noisePeriodDamp" );
	m_noiseNormalize.loadSettings( _this, "noiseNormalize" );
	m_exciterAttack.loadSettings( _this, "exciterAttack" );
	m_exciterDecay1.loadSettings( _this, "exciterDecay1" );
	m_exciterBreakpoint.loadSettings( _this, "exciterBreakpoint" );
	m_exciterDecay2.loadSettings( _this, "exciterDecay2" );
	m_exciterSustain.loadSettings( _this, "exciterSustain" );
	m_exciterRelease.loadSettings( _this, "exciterRelease" );
	m_volumeAttack.loadSettings( _this, "volumeAttack" );
	m_volumeDecay1.loadSettings( _this, "volumeDecay1" );
	m_volumeBreakpoint.loadSettings( _this, "volumeBreakpoint" );
	m_volumeDecay2.loadSettings( _this, "volumeDecay2" );
	m_volumeSustain.loadSettings( _this, "volumeSustain" );
	m_volumeRelease.loadSettings( _this, "volumeRelease" );
	m_tremoloAmount.loadSettings( _this, "tremoloAmount" );
	m_tremoloRate.loadSettings( _this, "tremoloRate" );
	m_vibratoAmount.loadSettings( _this, "vibratoAmount" );
	m_vibratoRate.loadSettings( _this, "vibratoRate" );
	m_vibratoShape.loadSettings( _this, "vibratoShape" );
	m_vibratoTremolo.loadSettings( _this, "vibratoTremolo" );
	m_vibratoFade.loadSettings( _this, "vibratoFade" );
	m_vibratoFadeShape.loadSettings( _this, "vibratoFadeShape" );
	m_vibratoLengthMin.loadSettings( _this, "vibratoLengthMin" );
	m_erodeSelf.loadSettings( _this, "erodeSelf" );
	m_erodeLP.loadSettings( _this, "erodeLP" );
	m_erodeHP.loadSettings( _this, "erodeHP" );
	m_erodeSine.loadSettings( _this, "erodeSine" );
	m_erodeFreq.loadSettings( _this, "erodeFreq" );
	m_erodeEnable.loadSettings( _this, "erodeEnable" );
	m_exciterFade.loadSettings( _this, "exciterFade" );
	m_release.loadSettings( _this, "release" );
	m_chordScatter.loadSettings( _this, "chordScatter" );
	m_chordScatterUniformity.loadSettings( _this, "chordScatterUniformity" );
}


QString Mercury2::nodeName() const
{
	return( mercury2_plugin_descriptor.name );
}



void Mercury2::play( sampleFrame * working_buffer )
{
	const fpp_t frames = Engine::audioEngine()->framesPerPeriod();

	ConstNotePlayHandleList m_nphList = NotePlayHandle::nphsOfInstrumentTrack(instrumentTrack(), true);
	
	if (m_chordScatter.value() != 0)
	{
		std::vector<float> newNoteFreqs;
		
		for (int i = 0; i < m_nphList.size(); ++i)
		{
			NotePlayHandle * currentNote = const_cast<NotePlayHandle *>(m_nphList[i]);

			if ( currentNote->totalFramesPlayed() == 0 || currentNote->m_pluginData == NULL )
			{
				newNoteFreqs.push_back(currentNote->frequency());
			}
		}
		
		if (newNoteFreqs.size() > 1)
		{
			std::sort(newNoteFreqs.begin(), newNoteFreqs.end());
		}
		
		for (int i = 0; i < m_nphList.size(); ++i)
		{
			NotePlayHandle * currentNote = const_cast<NotePlayHandle *>(m_nphList[i]);

			if ( currentNote->totalFramesPlayed() == 0 || currentNote->m_pluginData == NULL )
			{
				int newNoteFreqIndex = std::distance(newNoteFreqs.begin(), std::find(newNoteFreqs.begin(), newNoteFreqs.end(), currentNote->frequency()));
				float effectiveIndex = newNoteFreqIndex;
				if (m_chordScatter.value() < 0)
				{
					effectiveIndex = newNoteFreqs.size() - effectiveIndex - 1;
				}
				float uniformIndex = linearInterpolate(effectiveIndex, effectiveIndex / (newNoteFreqs.size() - 1), m_chordScatterUniformity.value());
				
				int chordScatterDelay = uniformIndex * std::abs(m_chordScatter.value()) * Engine::audioEngine()->outputSampleRate();
				
				currentNote->m_pluginData = new MSynth( currentNote, Engine::audioEngine()->outputSampleRate(), chordScatterDelay, m_impulseLP.value(), m_impulseLPFalloff.value(), m_partialRand.value() );
				m_lastNoteFreq = currentNote->frequency();
				if (m_mercury2View) {emit m_mercury2View->updatePartialPixmap();}
			}
		}
	}
	else
	{
		for (int i = 0; i < m_nphList.size(); ++i)
		{
			NotePlayHandle * currentNote = const_cast<NotePlayHandle *>(m_nphList[i]);

			if ( currentNote->totalFramesPlayed() == 0 || currentNote->m_pluginData == NULL )
			{
				currentNote->m_pluginData = new MSynth( currentNote, Engine::audioEngine()->outputSampleRate(), 0, m_impulseLP.value(), m_impulseLPFalloff.value(), m_partialRand.value() );
				m_lastNoteFreq = currentNote->frequency();
				if (m_mercury2View) {emit m_mercury2View->updatePartialPixmap();}
			}
		}
	}	
	
	emit updatePartials();

	for (int i = 0; i < m_nphList.size(); ++i)
	{
		NotePlayHandle * currentNote = const_cast<NotePlayHandle *>(m_nphList[i]);
		
		const fpp_t frames = currentNote->framesLeftForCurrentPeriod();
		//const fpp_t fpp = Engine::audioEngine()->framesPerPeriod();
		const f_cnt_t offset = currentNote->noteOffset();

		MSynth * ps = static_cast<MSynth *>( currentNote->m_pluginData );
		
		ps->nextNoteBuffer(working_buffer, currentNote, m_decay.value(), m_partialNum.value(), m_partialFreqMax.value(), m_partialStretch.value(), m_partialSine.value(), m_partialFlatOdd.value(), m_partialInharmEnabled.value(), m_partialInharmConst.value(), m_partialInharmTrack.value(), m_partialInharmLows.value(), m_partialInharmRetune.value(), m_partialDamp.value(), m_partialDampSlope.value(), m_partialFreqDamp.value(), m_partialFreqDampSlope.value(), m_partialRounding.value(), m_partialRoundingDivide.value(), m_partialRoundingCutoff.value(), m_partialComb1.value(), m_partialComb2.value(), m_partialCombShift.value(), m_partialCombBal.value(), m_partialCombInvert.value(), m_partialHarm2.value(), m_partialHarm3.value(), m_partialHarm5.value(), m_partialHarm7.value(), m_partialHarm11.value(), m_partialLP.value(), m_partialLPSlope.value(), m_partialRand.value(), m_playPosition.value(), m_playPositionRand.value(), m_brightness.value(), m_impulseNum.value(), m_impulseSpacing.value(), m_impulseLP.value(), m_impulseLPFalloff.value(), m_impulsePeriodicity.value(), m_impulseVol.value(), m_noiseVol.value(), m_noiseLP.value(), m_noiseHP.value(), m_noiseSat.value(), m_noisePeriodVol.value(), m_noisePeriodSpread.value(), m_noisePeriodShift.value(), m_noisePeriodDamp.value(), m_noiseNormalize.value(), m_exciterAttack.value(), m_exciterDecay1.value(), m_exciterBreakpoint.value(), m_exciterDecay2.value(), m_exciterSustain.value(), m_exciterRelease.value(), m_volumeAttack.value(), m_volumeDecay1.value(), m_volumeBreakpoint.value(), m_volumeDecay2.value(), m_volumeSustain.value(), m_volumeRelease.value(), m_tremoloAmount.value(), m_tremoloRate.value(), m_vibratoAmount.value(), m_vibratoRate.value(), m_vibratoShape.value(), m_vibratoTremolo.value(), m_vibratoFade.value(), m_vibratoFadeShape.value(), m_vibratoLengthMin.value(), m_erodeSelf.value(), m_erodeLP.value(), m_erodeHP.value(), m_erodeSine.value(), m_erodeFreq.value(), m_erodeEnable.value(), m_exciterFade.value(), m_release.value(), m_chordScatter.value(), m_chordScatterUniformity.value());
	}
	
	if (m_tremoloAmount.value() > 0)
	{
		for( fpp_t frame = 0; frame < frames; ++frame )
		{
			m_tremoloCurrentPhase += m_tremoloRate.value() / (float)Engine::audioEngine()->outputSampleRate();

			float tremoloVolumeChange = qFastSin(m_tremoloCurrentPhase * F_PI * 2.f) * m_tremoloAmount.value() + 1.f;
			working_buffer[frame][0] *= tremoloVolumeChange;
			working_buffer[frame][1] *= tremoloVolumeChange;
		}
		m_tremoloCurrentPhase = fmod(m_tremoloCurrentPhase, 1.f);
	}
	
	instrumentTrack()->processAudioBuffer( working_buffer, frames, NULL );
}

void Mercury2::updatePartials()
{
	ConstNotePlayHandleList m_nphList = NotePlayHandle::nphsOfInstrumentTrack(instrumentTrack(), true);

	for (int i = 0; i < m_nphList.size(); ++i)
	{
		if (static_cast<MSynth *>(m_nphList[i]->m_pluginData))
		{
			updatePartials(static_cast<MSynth *>(m_nphList[i]->m_pluginData), m_nphList[i]->frequency());
		}
	}
}

void Mercury2::updatePartials(MSynth * mSynthInstance, float noteFrequency)
{
	updateComb(mSynthInstance);
	mSynthInstance->updatePartials(noteFrequency, m_decay.value(), m_partialStretch.value(), m_partialSine.value(), m_partialFlatOdd.value(), m_partialInharmEnabled.value(), m_partialInharmConst.value(), m_partialInharmTrack.value(), m_partialInharmLows.value(), m_partialInharmRetune.value(), m_partialDamp.value(), m_partialDampSlope.value(), m_partialFreqDamp.value(), m_partialFreqDampSlope.value(), m_partialRounding.value(), m_partialRoundingDivide.value(), m_partialRoundingCutoff.value(), m_partialComb1.value(), m_partialComb2.value(), m_partialCombShift.value(), m_partialCombBal.value(), m_partialCombInvert.value(), m_partialHarm2.value(), m_partialHarm3.value(), m_partialHarm5.value(), m_partialHarm7.value(), m_partialHarm11.value(), m_partialLP.value(), m_partialLPSlope.value(), m_partialRand.value(), m_playPosition.value(), m_playPositionRand.value(), m_brightness.value(), m_partialFreqMax.value(), m_partialNum.value());
}

void Mercury2::updateComb(MSynth * mSynthInstance)
{
	mSynthInstance->updateComb(m_partialNum.value(), m_partialFreqMax.value(), m_partialComb1.value(), m_partialComb2.value(), m_partialCombShift.value(), m_partialCombBal.value(), m_partialCombInvert.value());
}


f_cnt_t Mercury2::desiredReleaseFrames() const
{
	return int(m_release.value() * 0.001f * Engine::audioEngine()->outputSampleRate()) + 256 + 1;
}


void Mercury2::deleteNotePluginData( NotePlayHandle * _n )
{
	delete static_cast<MSynth *>( _n->m_pluginData );
}




gui::PluginView * Mercury2::instantiateView( QWidget * _parent )
{
	m_mercury2View = new gui::Mercury2View( this, _parent );
	return m_mercury2View;
}




namespace gui
{


Mercury2View::Mercury2View( Instrument * _instrument,
					QWidget * _parent ) :
	InstrumentViewFixedSize( _instrument, _parent ),
	m_m(castModel<Mercury2>()),
	m_updatePartialPixmap(true)
{
	setAutoFillBackground( true );
	QPalette pal;

	pal.setBrush( backgroundRole(), PLUGIN_NAME::getIconPixmap(
								"artwork" ) );
	setPalette( pal );
	
	m_volumeKnob = new Knob( KnobType::Dark28, this );
	m_volumeKnob->move( 6, 201 );
	m_volumeKnob->setHintText( tr( "Volume" ), "" );
	m_volumeKnob->hide();
	m_volumeKnob->setModel( &m_m->m_volume );
	
	m_decayKnob = new Knob( KnobType::Small17, this );
	m_decayKnob->move( 5, 5 );
	m_decayKnob->setHintText( tr( "Decay" ), "" );
	m_decayKnob->setModel( &m_m->m_decay );
	
	m_partialNumKnob = new Knob( KnobType::Small17, this );
	m_partialNumKnob->move( 115, 5 );
	m_partialNumKnob->setHintText( tr( "Number of Partials" ), "" );
	m_partialNumKnob->setModel( &m_m->m_partialNum );

	m_partialFreqMaxKnob = new Knob( KnobType::Small17, this );
	m_partialFreqMaxKnob->move( 135, 5 );
	m_partialFreqMaxKnob->setHintText( tr( "Maximum Partial Frequency" ), "" );
	m_partialFreqMaxKnob->setModel( &m_m->m_partialFreqMax );

	m_partialStretchKnob = new Knob( KnobType::Small17, this );
	m_partialStretchKnob->move( 165, 5 );
	m_partialStretchKnob->setHintText( tr( "Partial Stretch" ), "" );
	m_partialStretchKnob->setModel( &m_m->m_partialStretch );
	
	m_partialSineKnob = new Knob( KnobType::Small17, this );
	m_partialSineKnob->move( 185, 5 );
	m_partialSineKnob->setHintText( tr( "Partial Sine" ), "" );
	m_partialSineKnob->setModel( &m_m->m_partialSine );
	
	m_partialFlatOddKnob = new Knob( KnobType::Small17, this );
	m_partialFlatOddKnob->move( 185, 25 );
	m_partialFlatOddKnob->setHintText( tr( "Flatten Odd Partials" ), "" );
	m_partialFlatOddKnob->setModel( &m_m->m_partialFlatOdd );
	
	m_partialInharmEnabledKnob = new Knob( KnobType::Small17, this );
	m_partialInharmEnabledKnob->move( 185, 45 );
	m_partialInharmEnabledKnob->setHintText( tr( "Enable Inharmonicity" ), "" );
	m_partialInharmEnabledKnob->setModel( &m_m->m_partialInharmEnabled );
	
	m_partialInharmConstKnob = new Knob( KnobType::Small17, this );
	m_partialInharmConstKnob->move( 205, 45 );
	m_partialInharmConstKnob->setHintText( tr( "Inharmonicity Constant" ), "" );
	m_partialInharmConstKnob->setModel( &m_m->m_partialInharmConst );
	
	m_partialInharmTrackKnob = new Knob( KnobType::Small17, this );
	m_partialInharmTrackKnob->move( 225, 45 );
	m_partialInharmTrackKnob->setHintText( tr( "Inharmonicity Tracking" ), "" );
	m_partialInharmTrackKnob->setModel( &m_m->m_partialInharmTrack );
	
	m_partialInharmLowsKnob = new Knob( KnobType::Small17, this );
	m_partialInharmLowsKnob->move( 245, 45 );
	m_partialInharmLowsKnob->setHintText( tr( "Low Note Inharmonicity" ), "" );
	m_partialInharmLowsKnob->setModel( &m_m->m_partialInharmLows );
	
	m_partialInharmRetuneKnob = new Knob( KnobType::Small17, this );
	m_partialInharmRetuneKnob->move( 205, 65 );
	m_partialInharmRetuneKnob->setHintText( tr( "Inharmonicity Dynamic Retuning" ), "" );
	m_partialInharmRetuneKnob->setModel( &m_m->m_partialInharmRetune );
	
	m_partialDampKnob = new Knob( KnobType::Small17, this );
	m_partialDampKnob->move( 25, 5 );
	m_partialDampKnob->setHintText( tr( "Partial Damp" ), "" );
	m_partialDampKnob->setModel( &m_m->m_partialDamp );
	
	m_partialDampSlopeKnob = new Knob( KnobType::Small17, this );
	m_partialDampSlopeKnob->move( 45, 5 );
	m_partialDampSlopeKnob->setHintText( tr( "Partial Damping Slope" ), "" );
	m_partialDampSlopeKnob->setModel( &m_m->m_partialDampSlope );
	
	m_partialFreqDampKnob = new Knob( KnobType::Small17, this );
	m_partialFreqDampKnob->move( 65, 5 );
	m_partialFreqDampKnob->setHintText( tr( "Partial Frequency Damping" ), " Hz" );
	m_partialFreqDampKnob->setModel( &m_m->m_partialFreqDamp );
	
	m_partialFreqDampSlopeKnob = new Knob( KnobType::Small17, this );
	m_partialFreqDampSlopeKnob->move( 85, 5 );
	m_partialFreqDampSlopeKnob->setHintText( tr( "Partial Frequency Damping Slope" ), "" );
	m_partialFreqDampSlopeKnob->setModel( &m_m->m_partialFreqDampSlope );
	
	m_partialRoundingKnob = new Knob( KnobType::Small17, this );
	m_partialRoundingKnob->move( 205, 85 );
	m_partialRoundingKnob->setHintText( tr( "Partial Rounding" ), "" );
	m_partialRoundingKnob->setModel( &m_m->m_partialRounding );
	
	m_partialRoundingDivideKnob = new Knob( KnobType::Small17, this );
	m_partialRoundingDivideKnob->move( 225, 85 );
	m_partialRoundingDivideKnob->setHintText( tr( "Partial Rounding Division Frequency" ), " Hz" );
	m_partialRoundingDivideKnob->setModel( &m_m->m_partialRoundingDivide );
	
	m_partialRoundingCutoffKnob = new Knob( KnobType::Small17, this );
	m_partialRoundingCutoffKnob->move( 245, 85 );
	m_partialRoundingCutoffKnob->setHintText( tr( "Partial Rounding Cutoff" ), " Hz" );
	m_partialRoundingCutoffKnob->setModel( &m_m->m_partialRoundingCutoff );
	
	m_partialComb1Knob = new Knob( KnobType::Small17, this );
	m_partialComb1Knob->move( 285, 85 );
	m_partialComb1Knob->setHintText( tr( "Partial Comb 1" ), "" );
	m_partialComb1Knob->setModel( &m_m->m_partialComb1 );
	
	m_partialComb2Knob = new Knob( KnobType::Small17, this );
	m_partialComb2Knob->move( 305, 85 );
	m_partialComb2Knob->setHintText( tr( "Partial Comb 2" ), "" );
	m_partialComb2Knob->setModel( &m_m->m_partialComb2 );
	
	m_partialCombShiftKnob = new Knob( KnobType::Small17, this );
	m_partialCombShiftKnob->move( 325, 85 );
	m_partialCombShiftKnob->setHintText( tr( "Partial Comb Shift" ), "" );
	m_partialCombShiftKnob->setModel( &m_m->m_partialCombShift );
	
	m_partialCombBalKnob = new Knob( KnobType::Small17, this );
	m_partialCombBalKnob->move( 345, 85 );
	m_partialCombBalKnob->setHintText( tr( "Partial Comb Balance" ), "" );
	m_partialCombBalKnob->setModel( &m_m->m_partialCombBal );
	
	m_partialCombInvertKnob = new Knob( KnobType::Small17, this );
	m_partialCombInvertKnob->move( 365, 85 );
	m_partialCombInvertKnob->setHintText( tr( "Partial Comb Invert" ), "" );
	m_partialCombInvertKnob->setModel( &m_m->m_partialCombInvert );
	
	m_partialHarm2Knob = new Knob( KnobType::Small17, this );
	m_partialHarm2Knob->move( 245, 5 );
	m_partialHarm2Knob->setHintText( tr( "2nd Harmonic" ), "" );
	m_partialHarm2Knob->setModel( &m_m->m_partialHarm2 );
	
	m_partialHarm3Knob = new Knob( KnobType::Small17, this );
	m_partialHarm3Knob->move( 265, 5 );
	m_partialHarm3Knob->setHintText( tr( "3rd Harmonic" ), "" );
	m_partialHarm3Knob->setModel( &m_m->m_partialHarm3 );
	
	m_partialHarm5Knob = new Knob( KnobType::Small17, this );
	m_partialHarm5Knob->move( 285, 5 );
	m_partialHarm5Knob->setHintText( tr( "5th Harmonic" ), "" );
	m_partialHarm5Knob->setModel( &m_m->m_partialHarm5 );
	
	m_partialHarm7Knob = new Knob( KnobType::Small17, this );
	m_partialHarm7Knob->move( 305, 5 );
	m_partialHarm7Knob->setHintText( tr( "7th Harmonic" ), "" );
	m_partialHarm7Knob->setModel( &m_m->m_partialHarm7 );
	
	m_partialHarm11Knob = new Knob( KnobType::Small17, this );
	m_partialHarm11Knob->move( 325, 5 );
	m_partialHarm11Knob->setHintText( tr( "11th Harmonic" ), "" );
	m_partialHarm11Knob->setModel( &m_m->m_partialHarm11 );
	
	m_partialLPKnob = new Knob( KnobType::Small17, this );
	m_partialLPKnob->move( 365, 5 );
	m_partialLPKnob->setHintText( tr( "Partial Lowpass" ), "" );
	m_partialLPKnob->setModel( &m_m->m_partialLP );
	
	m_partialLPSlopeKnob = new Knob( KnobType::Small17, this );
	m_partialLPSlopeKnob->move( 385, 5 );
	m_partialLPSlopeKnob->setHintText( tr( "Partial Lowpass Slope" ), "" );
	m_partialLPSlopeKnob->setModel( &m_m->m_partialLPSlope );
	
	m_partialRandKnob = new Knob( KnobType::Small17, this );
	m_partialRandKnob->move( 385, 25 );
	m_partialRandKnob->setHintText( tr( "Partial Randomness" ), "" );
	m_partialRandKnob->setModel( &m_m->m_partialRand );
	
	m_playPositionKnob = new Knob( KnobType::Small17, this );
	m_playPositionKnob->move( 305, 45 );
	m_playPositionKnob->setHintText( tr( "Play Position" ), "" );
	m_playPositionKnob->setModel( &m_m->m_playPosition );
	
	m_playPositionRandKnob = new Knob( KnobType::Small17, this );
	m_playPositionRandKnob->move( 325, 45 );
	m_playPositionRandKnob->setHintText( tr( "Play Position Randomness" ), "" );
	m_playPositionRandKnob->setModel( &m_m->m_playPositionRand );
	
	m_brightnessKnob = new Knob( KnobType::Small17, this );
	m_brightnessKnob->move( 365, 45 );
	m_brightnessKnob->setHintText( tr( "Brightness" ), "" );
	m_brightnessKnob->setModel( &m_m->m_brightness );
	
	m_impulseNumKnob = new Knob( KnobType::Small17, this );
	m_impulseNumKnob->move( 25, 45 );
	m_impulseNumKnob->setHintText( tr( "Number of Impulses" ), "" );
	m_impulseNumKnob->setModel( &m_m->m_impulseNum );
	
	m_impulseSpacingKnob = new Knob( KnobType::Small17, this );
	m_impulseSpacingKnob->move( 45, 45 );
	m_impulseSpacingKnob->setHintText( tr( "Impulse Spacing" ), "" );
	m_impulseSpacingKnob->setModel( &m_m->m_impulseSpacing );
	
	m_impulseLPKnob = new Knob( KnobType::Small17, this );
	m_impulseLPKnob->move( 65, 45 );
	m_impulseLPKnob->setHintText( tr( "Impulse Lowpass" ), "" );
	m_impulseLPKnob->setModel( &m_m->m_impulseLP );
	
	m_impulseLPFalloffKnob = new Knob( KnobType::Small17, this );
	m_impulseLPFalloffKnob->move( 85, 45 );
	m_impulseLPFalloffKnob->setHintText( tr( "Impulse Lowpass Falloff" ), "" );
	m_impulseLPFalloffKnob->setModel( &m_m->m_impulseLPFalloff );
	
	m_impulsePeriodicityKnob = new Knob( KnobType::Small17, this );
	m_impulsePeriodicityKnob->move( 105, 45 );
	m_impulsePeriodicityKnob->setHintText( tr( "Impulse Periodicity" ), "" );
	m_impulsePeriodicityKnob->setModel( &m_m->m_impulsePeriodicity );
	
	m_impulseVolKnob = new Knob( KnobType::Small17, this );
	m_impulseVolKnob->move( 125, 45 );
	m_impulseVolKnob->setHintText( tr( "Impulse Volume" ), "" );
	m_impulseVolKnob->setModel( &m_m->m_impulseVol );
	
	m_noiseVolKnob = new Knob( KnobType::Small17, this );
	m_noiseVolKnob->move( 5, 105 );
	m_noiseVolKnob->setHintText( tr( "Noise Volume" ), "" );
	m_noiseVolKnob->setModel( &m_m->m_noiseVol );
	
	m_noiseLPKnob = new Knob( KnobType::Small17, this );
	m_noiseLPKnob->move( 25, 105 );
	m_noiseLPKnob->setHintText( tr( "Noise Lowpass" ), " Hz" );
	m_noiseLPKnob->setModel( &m_m->m_noiseLP );
	
	m_noiseHPKnob = new Knob( KnobType::Small17, this );
	m_noiseHPKnob->move( 45, 105 );
	m_noiseHPKnob->setHintText( tr( "Noise Highpass" ), " Hz" );
	m_noiseHPKnob->setModel( &m_m->m_noiseHP );
	
	m_noiseSatKnob = new Knob( KnobType::Small17, this );
	m_noiseSatKnob->move( 65, 105 );
	m_noiseSatKnob->setHintText( tr( "Noise Saturation" ), "" );
	m_noiseSatKnob->setModel( &m_m->m_noiseSat );
	
	m_noisePeriodVolKnob = new Knob( KnobType::Small17, this );
	m_noisePeriodVolKnob->move( 85, 105 );
	m_noisePeriodVolKnob->setHintText( tr( "Noise Periodicity Volume" ), "" );
	m_noisePeriodVolKnob->setModel( &m_m->m_noisePeriodVol );
	
	m_noisePeriodSpreadKnob = new Knob( KnobType::Small17, this );
	m_noisePeriodSpreadKnob->move( 105, 105 );
	m_noisePeriodSpreadKnob->setHintText( tr( "Noise Periodicity Spread" ), "" );
	m_noisePeriodSpreadKnob->setModel( &m_m->m_noisePeriodSpread );
	
	m_noisePeriodShiftKnob = new Knob( KnobType::Small17, this );
	m_noisePeriodShiftKnob->move( 125, 105 );
	m_noisePeriodShiftKnob->setHintText( tr( "Noise Periodicity Shift" ), "x" );
	m_noisePeriodShiftKnob->setModel( &m_m->m_noisePeriodShift );
	
	m_noisePeriodDampKnob = new Knob( KnobType::Small17, this );
	m_noisePeriodDampKnob->move( 145, 105 );
	m_noisePeriodDampKnob->setHintText( tr( "Noise Periodicity Damping" ), " Hz" );
	m_noisePeriodDampKnob->setModel( &m_m->m_noisePeriodDamp );
	
	m_noiseNormalizeKnob = new Knob( KnobType::Small17, this );
	m_noiseNormalizeKnob->move( 165, 105 );
	m_noiseNormalizeKnob->setHintText( tr( "Noise Normalization" ), "" );
	m_noiseNormalizeKnob->setModel( &m_m->m_noiseNormalize );
	
	m_exciterAttackKnob = new Knob( KnobType::Small17, this );
	m_exciterAttackKnob->move( 5, 65 );
	m_exciterAttackKnob->setHintText( tr( "Exciter Attack:" ), "" );
	m_exciterAttackKnob->setModel( &m_m->m_exciterAttack );

	m_exciterDecay1Knob = new Knob( KnobType::Small17, this );
	m_exciterDecay1Knob->move( 25, 65 );
	m_exciterDecay1Knob->setHintText( tr( "Exciter Decay 1:" ), "" );
	m_exciterDecay1Knob->setModel( &m_m->m_exciterDecay1 );

	m_exciterBreakpointKnob = new Knob( KnobType::Small17, this );
	m_exciterBreakpointKnob->move( 45, 65 );
	m_exciterBreakpointKnob->setHintText( tr( "Exciter Breakpoint:" ), "" );
	m_exciterBreakpointKnob->setModel( &m_m->m_exciterBreakpoint );

	m_exciterDecay2Knob = new Knob( KnobType::Small17, this );
	m_exciterDecay2Knob->move( 65, 65 );
	m_exciterDecay2Knob->setHintText( tr( "Exciter Decay 2:" ), "" );
	m_exciterDecay2Knob->setModel( &m_m->m_exciterDecay2 );

	m_exciterSustainKnob = new Knob( KnobType::Small17, this );
	m_exciterSustainKnob->move( 85, 65 );
	m_exciterSustainKnob->setHintText( tr( "Exciter Sustain:" ), "" );
	m_exciterSustainKnob->setModel( &m_m->m_exciterSustain );

	m_exciterReleaseKnob = new Knob( KnobType::Small17, this );
	m_exciterReleaseKnob->move( 105, 65 );
	m_exciterReleaseKnob->setHintText( tr( "Exciter Release:" ), "" );
	m_exciterReleaseKnob->setModel( &m_m->m_exciterRelease );
	
	m_volumeAttackKnob = new Knob( KnobType::Small17, this );
	m_volumeAttackKnob->move( 5, 85 );
	m_volumeAttackKnob->setHintText( tr( "Volume Attack:" ), "" );
	m_volumeAttackKnob->setModel( &m_m->m_volumeAttack );

	m_volumeDecay1Knob = new Knob( KnobType::Small17, this );
	m_volumeDecay1Knob->move( 25, 85 );
	m_volumeDecay1Knob->setHintText( tr( "Volume Decay 1:" ), "" );
	m_volumeDecay1Knob->setModel( &m_m->m_volumeDecay1 );

	m_volumeBreakpointKnob = new Knob( KnobType::Small17, this );
	m_volumeBreakpointKnob->move( 45, 85 );
	m_volumeBreakpointKnob->setHintText( tr( "Volume Breakpoint:" ), "" );
	m_volumeBreakpointKnob->setModel( &m_m->m_volumeBreakpoint );

	m_volumeDecay2Knob = new Knob( KnobType::Small17, this );
	m_volumeDecay2Knob->move( 65, 85 );
	m_volumeDecay2Knob->setHintText( tr( "Volume Decay 2:" ), "" );
	m_volumeDecay2Knob->setModel( &m_m->m_volumeDecay2 );

	m_volumeSustainKnob = new Knob( KnobType::Small17, this );
	m_volumeSustainKnob->move( 85, 85 );
	m_volumeSustainKnob->setHintText( tr( "Volume Sustain:" ), "" );
	m_volumeSustainKnob->setModel( &m_m->m_volumeSustain );

	m_volumeReleaseKnob = new Knob( KnobType::Small17, this );
	m_volumeReleaseKnob->move( 105, 85 );
	m_volumeReleaseKnob->setHintText( tr( "Volume Release:" ), "" );
	m_volumeReleaseKnob->setModel( &m_m->m_volumeRelease );
	
	m_tremoloAmountKnob = new Knob( KnobType::Small17, this );
	m_tremoloAmountKnob->move( 145, 85 );
	m_tremoloAmountKnob->setHintText( tr( "Tremolo Amount" ), "" );
	m_tremoloAmountKnob->setModel( &m_m->m_tremoloAmount );
	
	m_tremoloRateKnob = new Knob( KnobType::Small17, this );
	m_tremoloRateKnob->move( 165, 85 );
	m_tremoloRateKnob->setHintText( tr( "Tremolo Rate" ), "" );
	m_tremoloRateKnob->setModel( &m_m->m_tremoloRate );
	
	m_vibratoAmountKnob = new Knob( KnobType::Small17, this );
	m_vibratoAmountKnob->move( 285, 65 );
	m_vibratoAmountKnob->setHintText( tr( "Vibrato Amount" ), "" );
	m_vibratoAmountKnob->setModel( &m_m->m_vibratoAmount );
	
	m_vibratoRateKnob = new Knob( KnobType::Small17, this );
	m_vibratoRateKnob->move( 305, 65 );
	m_vibratoRateKnob->setHintText( tr( "Vibrato Rate" ), "" );
	m_vibratoRateKnob->setModel( &m_m->m_vibratoRate );
	
	m_vibratoShapeKnob = new Knob( KnobType::Small17, this );
	m_vibratoShapeKnob->move( 325, 65 );
	m_vibratoShapeKnob->setHintText( tr( "Vibrato Shape" ), "" );
	m_vibratoShapeKnob->setModel( &m_m->m_vibratoShape );
	
	m_vibratoTremoloKnob = new Knob( KnobType::Small17, this );
	m_vibratoTremoloKnob->move( 345, 65 );
	m_vibratoTremoloKnob->setHintText( tr( "Vibrato Tremolo" ), "" );
	m_vibratoTremoloKnob->setModel( &m_m->m_vibratoTremolo );
	
	m_vibratoFadeKnob = new Knob( KnobType::Small17, this );
	m_vibratoFadeKnob->move( 365, 65 );
	m_vibratoFadeKnob->setHintText( tr( "Vibrato Fade" ), "" );
	m_vibratoFadeKnob->setModel( &m_m->m_vibratoFade );
	
	m_vibratoFadeShapeKnob = new Knob( KnobType::Small17, this );
	m_vibratoFadeShapeKnob->move( 385, 65 );
	m_vibratoFadeShapeKnob->setHintText( tr( "Vibrato Fade Shape" ), "" );
	m_vibratoFadeShapeKnob->setModel( &m_m->m_vibratoFadeShape );
	
	m_vibratoLengthMinKnob = new Knob( KnobType::Small17, this );
	m_vibratoLengthMinKnob->move( 405, 65 );
	m_vibratoLengthMinKnob->setHintText( tr( "Vibrato Note Length Minimum" ), "" );
	m_vibratoLengthMinKnob->setModel( &m_m->m_vibratoLengthMin );
	
	m_erodeSelfKnob = new Knob( KnobType::Small17, this );
	m_erodeSelfKnob->move( 5, 25 );
	m_erodeSelfKnob->setHintText( tr( "Erode Self" ), "" );
	m_erodeSelfKnob->setModel( &m_m->m_erodeSelf );
	
	m_erodeLPKnob = new Knob( KnobType::Small17, this );
	m_erodeLPKnob->move( 25, 25 );
	m_erodeLPKnob->setHintText( tr( "Erosion Lowpass" ), "" );
	m_erodeLPKnob->setModel( &m_m->m_erodeLP );
	
	m_erodeHPKnob = new Knob( KnobType::Small17, this );
	m_erodeHPKnob->move( 45, 25 );
	m_erodeHPKnob->setHintText( tr( "Erosion Highpass" ), "" );
	m_erodeHPKnob->setModel( &m_m->m_erodeHP );
	
	m_erodeSineKnob = new Knob( KnobType::Small17, this );
	m_erodeSineKnob->move( 65, 25 );
	m_erodeSineKnob->setHintText( tr( "Erode Sine" ), "" );
	m_erodeSineKnob->setModel( &m_m->m_erodeSine );
	
	m_erodeFreqKnob = new Knob( KnobType::Small17, this );
	m_erodeFreqKnob->move( 85, 25 );
	m_erodeFreqKnob->setHintText( tr( "Erosion Sine Frequency" ), "" );
	m_erodeFreqKnob->setModel( &m_m->m_erodeFreq );
	
	m_erodeEnableKnob = new Knob( KnobType::Small17, this );
	m_erodeEnableKnob->move( 105, 25 );
	m_erodeEnableKnob->setHintText( tr( "Erosion Enabled" ), "" );
	m_erodeEnableKnob->setModel( &m_m->m_erodeEnable );
	
	m_exciterFadeKnob = new Knob( KnobType::Small17, this );
	m_exciterFadeKnob->move( 265, 45 );
	m_exciterFadeKnob->setHintText( tr( "Exciter Fade" ), "" );
	m_exciterFadeKnob->setModel( &m_m->m_exciterFade );
	
	m_releaseKnob = new Knob( KnobType::Small17, this );
	m_releaseKnob->move( 400, 85 );
	m_releaseKnob->setHintText( tr( "Release" ), "" );
	m_releaseKnob->setModel( &m_m->m_release );
	
	m_chordScatterKnob = new Knob( KnobType::Small17, this );
	m_chordScatterKnob->move( 400, 45 );
	m_chordScatterKnob->setHintText( tr( "Chord Scatter" ), "" );
	m_chordScatterKnob->setModel( &m_m->m_chordScatter );
	
	m_chordScatterUniformityKnob = new Knob( KnobType::Small17, this );
	m_chordScatterUniformityKnob->move( 420, 45 );
	m_chordScatterUniformityKnob->setHintText( tr( "Chord Scatter Uniformity" ), "" );
	m_chordScatterUniformityKnob->setModel( &m_m->m_chordScatterUniformity );
	
	m_partialPixmap.fill(QColor("transparent"));
	
	connect(getGUI()->mainWindow(), SIGNAL(periodicUpdate()), this, SLOT(updateDisplay()));
	
	connect(&m_m->m_decay, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialStretch, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialSine, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialFlatOdd, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialInharmEnabled, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialInharmConst, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialInharmTrack, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialInharmLows, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialInharmRetune, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialDamp, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialRounding, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialRoundingDivide, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialRoundingCutoff, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialNum, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialDampSlope, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialFreqDamp, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialFreqDampSlope, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialComb1, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialComb2, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialCombShift, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialCombBal, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialCombInvert, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialHarm2, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialHarm3, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialHarm5, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialHarm7, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialHarm11, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialLP, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialLPSlope, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialRand, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_playPosition, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_playPositionRand, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_brightness, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	connect(&m_m->m_partialFreqMax, SIGNAL(dataChanged()), this, SLOT(updatePartialPixmap()));
	
	m_dummyNote = new MSynth( nullptr, Engine::audioEngine()->outputSampleRate(), 0, m_m->m_impulseLP.value(), m_m->m_impulseLPFalloff.value(), m_m->m_partialRand.value() );
}


void Mercury2View::paintEvent(QPaintEvent *event)
{
	if (!isVisible())
	{
		return;
	}
	
	updateDisplay();

	m_p.begin(this);

	m_p.setCompositionMode(QPainter::CompositionMode_Source);
	m_p.fillRect(0, 0, MERCURY_WINDOWSIZE_X, MERCURY_WINDOWSIZE_Y, QColor(12, 14, 15, 255));
	m_p.setCompositionMode(QPainter::CompositionMode_SourceOver);

	m_p.drawPixmap((MERCURY_WINDOWSIZE_X - MERCURY_PARTIALVIEWSIZE_X) * 0.5, MERCURY_WINDOWSIZE_Y - MERCURY_PARTIALVIEWSIZE_Y - 20, m_partialPixmap);

	m_p.end();
}


void Mercury2View::updateDisplay()
{
	if (!isVisible())
	{
		return;
	}
	
	if (m_updatePartialPixmap)
	{
		m_updatePartialPixmap = false;
		drawPartialPixmap();
		
		update();
	}
}


void Mercury2View::drawPartialPixmap()
{
	float sampleRate = Engine::audioEngine()->outputSampleRate();

	float noteFreq = m_m->m_lastNoteFreq;
	m_m->updatePartials(m_dummyNote, noteFreq);
	
	m_p.begin(&m_partialPixmap);

	m_p.setRenderHint(QPainter::Antialiasing, false);

	m_p.setCompositionMode(QPainter::CompositionMode_Source);
	m_p.fillRect(0, 0, MERCURY_PARTIALVIEWSIZE_X, MERCURY_PARTIALVIEWSIZE_Y, QColor("transparent"));
	m_p.setCompositionMode(QPainter::CompositionMode_SourceOver);
	
	
	float fundamentalDecay = (sampleRate * 1000.f) / std::log(m_dummyNote->m_partialRadius[0]);
	for (int i = 0; i < m_m->m_partialNum.value(); ++i)
	{
		int lineX = m_dummyNote->m_partialFreq[i] / noteFreq / (float)m_m->m_partialNum.value() * (float)MERCURY_PARTIALVIEWLENGTH + MERCURY_PARTIALVIEWBORDER;
		int lineHeight = std::abs(m_dummyNote->m_partialGain[i]) * (float)MERCURY_PARTIALVIEWHEIGHT * sampleRate / noteFreq;
		if (lineX < MERCURY_PARTIALVIEWSIZE_X - MERCURY_PARTIALVIEWBORDER)
		{
			float temp = ((sampleRate * 1000.f) / std::log(m_dummyNote->m_partialRadius[i])) / fundamentalDecay;
			temp = std::sqrt(temp);
			int lineOpacity = qBound(25.f, (temp * 230.f) + 25.f, 255.f);
			int lineRed = linearInterpolate(255, 255, std::abs(m_dummyNote->m_comb1GainPre[i]));
			int lineGreen = linearInterpolate(255, 45, std::abs(m_dummyNote->m_comb1GainPre[i]));
			int lineBlue = linearInterpolate(255, 46, std::abs(m_dummyNote->m_comb1GainPre[i]));
			m_p.setPen(QPen(QColor(lineRed, lineGreen, lineBlue, lineOpacity), 1));
			
			m_p.drawLine(lineX, MERCURY_PARTIALVIEWSIZE_Y - MERCURY_PARTIALVIEWBORDER, lineX, MERCURY_PARTIALVIEWSIZE_Y - MERCURY_PARTIALVIEWBORDER - lineHeight);
		}
	}

	m_p.end();
}


} // namespace gui


extern "C"
{

// necessary for getting instance out of shared lib
PLUGIN_EXPORT Plugin * lmms_plugin_main( Model *m, void * )
{
	return( new Mercury2( static_cast<InstrumentTrack *>( m ) ) );
}


}


} // namespace lmms
