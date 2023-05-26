/*
 * Mercury.cpp - [Put a very brief description of the effect here (not more than just a few words usually)]
 *
 * Copyright (c) [the year] [your name] [<youremail/at/gmailormaybenotgmail/dot/com>]
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

#include "Mercury.h"
#include "base64.h"
#include "Engine.h"
#include "InstrumentPlayHandle.h"
#include "InstrumentTrack.h"
#include "Knob.h"
#include "lmms_math.h"
#include "Mixer.h"
#include "NotePlayHandle.h"
#include "Oscillator.h"
#include "Song.h"

#include "embed.h"

#include "plugin_export.h"

#include "lmms_math.h"
#include <qmath.h>
#include <interpolation.h>









#include <iostream>









namespace lmms
{



extern "C"
{

Plugin::Descriptor PLUGIN_EXPORT mercury_plugin_descriptor =
{
	LMMS_STRINGIFY( PLUGIN_NAME ),
	"Mercury",
	QT_TRANSLATE_NOOP( "pluginBrowser", "[PUT YOUR PLUGIN DESCRIPTION HERE]" ),
	"[your name] [<youremail/at/gmailormaybenotgmail/dot/com>]",
	0x0100,
	Plugin::Instrument,
	new PluginPixmapLoader( "logo" ),
	nullptr,
	nullptr
} ;

}


sSynth::sSynth( NotePlayHandle * _nph, const sample_rate_t _sample_rate ):
	m_nph( _nph ),
	m_sampleRate(_sample_rate),
	m_exciterLPF(_sample_rate),
	m_exciterHPF(_sample_rate)
{
	m_exciterBuf.resize(m_sampleRate*10 + 1);
	m_exciterBufSize = m_sampleRate*10 + 1;
	
	m_coeffPrecalc = -2.2 / (m_sampleRate * 0.001f);// -2.2 is slope constant
}


sSynth::~sSynth()
{
}


void sSynth::nextStringSample( sampleFrame &outputSample, int frame, float decay, int partialNum, float partialFreqMax, float partialStretch, float partialSine, float partialLP, float partialLPSlope, float partialDamp, float fundamental, float partialComb1, float partialComb2, float partialCombShift, float partialCombBal, float partialRing, float partialRound, float partialRoundMax, float partialFreqBal, float exciterLP, float exciterHP, float exciterImpulse, float exciterNoise, float partialProtect, float partialProtectSlope, float exciterFeedback, float exciterFeedbackDelay, float modalVol, float exciterFeedbackPhase, float exciterFade, float exciterDelayFade, float exciterAttack, float exciterDecay1, float exciterBreakpoint, float exciterDecay2, float exciterSustain, float exciterRelease, float exciterSaturation, float exciterAsym, float exciterPulseTrain, float exciterFeedbackLP, float reverbMix, float reverbDecay, float reverbSize, float reverbDiffusion, float reverbHighDamp, float reverbLowDamp, float reverbMod, float reverbModFreq, float reverbWidth, float flangerMix, float flangerDelay, float flangerDepth, float flangerRate, float flangerFeedback, float flangerDamping, float flangerPhase, float delayMix, float delayTime, float delayStereo, float delayHighDamp, float delayLowDamp, float delayFeedback, float exciterLeak )
{
	if( m_nph->isReleased() && frame >= m_nph->framesBeforeRelease() && !m_noteReleaseTime )
	{
		m_noteReleaseTime = m_noteDuration;
	}

	if (exciterLP != m_exciterLPlast)
	{
		m_exciterLPlast = exciterLP;
		m_exciterLPF.setLowpass(exciterLP);
	}
	if (exciterHP != m_exciterHPlast)
	{
		m_exciterHPlast = exciterHP;
		m_exciterHPF.setHighpass(exciterHP);
	}

	float exciterSamp;

	float impulseSamp = (m_noteDuration == 0);

	float noiseSamp = fast_rand() / (float)FAST_RAND_MAX * 2.f - 1;
	noiseSamp *= calcEnvelope(m_noteDuration, exciterAttack * 0.001f * m_sampleRate, exciterDecay1 * 0.001f * m_sampleRate, exciterBreakpoint, exciterDecay2 * 0.001f * m_sampleRate, exciterSustain, exciterRelease * 0.001f * m_sampleRate, m_envGainStorage);

	const float pulseTrainExciterDelayOutput = m_exciterDelayOutput ? (m_exciterDelayOutput > 0 ? 1 : -1) : 0;
	exciterSamp = noiseSamp * exciterNoise + impulseSamp * exciterImpulse + linearInterpolate(m_exciterDelayOutput, pulseTrainExciterDelayOutput, exciterPulseTrain) * exciterFeedback;

	exciterSamp = m_exciterLPF.update(exciterSamp, 0);
	exciterSamp = m_exciterHPF.update(exciterSamp, 0);

	//exciterSamp = sin(m_noteDuration * F_2PI * m_nph->frequency() / m_sampleRate);// FOR TESTING ONLY

	outputSample[0] = 0;

	// Prepare for comb calculations
	float comb1GainPost;
	float comb2GainPost;

	// Calculate the gain of each partial group
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

	float combSample1 = 0;
	float combSample2 = 0;
	float combSample1PostBal = 0;
	float combSample2PostBal = 0;

	// Loop through every partial
	for (int i = 0; i < partialNum; ++i)
	{
		// Ignore if partial frequency is too high
		if (m_partialFreq[i] > partialFreqMax)
		{
			continue;
		}

		comb1GainPost = m_comb1GainPre[i] * comb1Gain;
		comb2GainPost = m_comb2GainPre[i] * comb2Gain;

		// Process actual resonator
		const float resonatorResult = calcResonator(exciterSamp, i, m_partialFreq[i], m_partialRadius[i], exciterLeak) * m_partialGain[i];
		combSample1 += resonatorResult * m_comb1GainPre[i];
		combSample2 += resonatorResult * m_comb2GainPre[i];
		combSample1PostBal += resonatorResult * comb1GainPost;
		combSample2PostBal += resonatorResult * comb2GainPost;
	}

	// Ring modulation multiplication by 2.f is due to the fact that the ring modulated signal tends to be quite a bit
	// quieter than the original signals
	float modalOutput = linearInterpolate(combSample1PostBal + -combSample2PostBal, combSample1 * -combSample2 * 2.f, partialRing) * modalVol * 0.01f;

	//YOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO, we do be inverting that B comb group tho

	const float w0 = (F_2PI / m_sampleRate) * exciterFeedbackPhase;
	const float a0 = 1 + (qFastSin(w0) / (0.707 * 2.f));// 0.707 = resonance
	const float b0 = (1 - (a0 - 1)) / a0;
	const float b1 = (-2*cos(w0)) / a0;
	const float allpassOutput = calcAllpassFilter(modalOutput, b0, b1);
	// To protect against infinite feedback loops
	float saturatedAllpassOutput = tanh(allpassOutput + exciterAsym);
	m_saturatedAllpassOutputDC = saturatedAllpassOutput * 0.001f + 0.999f * m_saturatedAllpassOutputDC;
	saturatedAllpassOutput -= m_saturatedAllpassOutputDC;

	const float a = 1.f - expf(-(F_2PI * m_nph->frequency() / m_sampleRate));
	m_saturatedAllpassOutputLP = saturatedAllpassOutput * a + (1.f - a) * m_saturatedAllpassOutputLP;

	++m_exciterBufIndex;
	if (m_exciterBufIndex >= m_exciterBufSize)
	{
		m_exciterBufIndex = 0;
	}

	float newExciterBufIndex = realfmod(m_exciterBufIndex - (exciterFeedbackDelay * 0.001f * m_sampleRate), m_exciterBufSize);
	float newExciterBufIndexFrac = fraction(newExciterBufIndex);

	if (newExciterBufIndex < m_exciterBufSize - 1)
	{
		m_exciterDelayOutput = m_exciterBuf[floor(newExciterBufIndex)] * (1.f - newExciterBufIndexFrac) + m_exciterBuf[ceil(newExciterBufIndex)] * newExciterBufIndexFrac;
	}
	else// For when the interpolation wraps around to the beginning of the buffer
	{
		m_exciterDelayOutput = m_exciterBuf[m_exciterBufSize - 1] * (1.f - newExciterBufIndexFrac) + m_exciterBuf[0] * newExciterBufIndexFrac;
	}

	m_exciterBuf[m_exciterBufIndex] = m_saturatedAllpassOutputLP;

	const float trueAllpassOutput = linearInterpolate(allpassOutput, m_saturatedAllpassOutputLP, exciterSaturation);
	float trueModalOutput = linearInterpolate(trueAllpassOutput, sign(exciterDelayFade) * m_exciterDelayOutput, abs(exciterDelayFade));

	outputSample[0] = linearInterpolate(trueModalOutput, exciterSamp, exciterFade);
	
	if( m_noteReleaseTime )
	{
		float r = exciterRelease * 0.001f * m_sampleRate;
		if( m_noteDuration < r + m_noteReleaseTime )
		{
			outputSample[0] *= (-1 / r) * (m_noteDuration - m_noteReleaseTime) + 1;
		}
		else
		{
			outputSample[0] = 0;
		}
	}
	
	outputSample[1] = outputSample[0];

	++m_noteDuration;
}



inline void sSynth::updatePartials(float decay, float partialStretch, float partialSine, float partialLP, float partialLPSlope, float partialDamp, float fundamental, float partialComb1, float partialComb2, float partialCombShift, float partialCombBal, float partialRing, float partialRound, float partialRoundMax, float partialFreqBal, float exciterLP, float exciterHP, float exciterImpulse, float exciterNoise, float partialProtect, float partialProtectSlope)
{
	for (int i = 0; i < PARTIAL_MAX; ++i)
	{
		float partialFreq = m_nph->frequency();
		float partialOffsetRatio;
		if (partialSine)
		{
			partialOffsetRatio = i * partialStretch * (qFastSin(i * partialSine * F_2PI) * 0.5f + 1.f);
		}
		else
		{
			partialOffsetRatio = i * partialStretch;
		}
		partialFreq += m_nph->frequency() * partialOffsetRatio;
		if (partialProtect > 0)
		{
			float partialFreqProtection;
			if (i <= partialProtect)
			{
				partialFreqProtection = 1.f;
			}
			else if (i <= partialProtect + partialProtectSlope)
			{
				partialFreqProtection = cosinusInterpolate(1, 0, (i - partialProtect) / partialProtectSlope);
			}
			else
			{
				partialFreqProtection = 0.f;
			}
			partialFreq = linearInterpolate(partialFreq, m_nph->frequency() * (i + 1), partialFreqProtection);
		}

		if (partialFreq > PARTIAL_FREQ_TRUE_MAX)
		{
			m_partialFreq[i] = partialFreq;
			m_partialGain[i] = 0;
			m_partialRadius[i] = 0;
			continue;
		}

		float partialGain = m_nph->frequency() / m_sampleRate;
		partialGain *= powf(1.f / (i + 1.f), partialFreqBal);
		float partialLPGain = -calcLogistic(i, 1.f, partialLPSlope, partialLP) + 1.f;
		partialGain *= partialLPGain;
		if (i != 0) {partialGain *= fundamental;}

		float partialRadius = decay;
		if (partialDamp < 1.f)
		{
			float partialDampingMult = powf(partialDamp, i / 2.f + 1.f);
			partialRadius *= partialDampingMult;
		}
		partialRadius = expf((-1000.f / m_sampleRate) / partialRadius);
		
		if (partialRound && i <= partialRoundMax)
		{
			float octaveDif = std::log2(partialFreq / m_nph->frequency());
			float octaveRound = partialRound / 1200.f;
			octaveDif = std::round(octaveDif / octaveRound) * octaveRound;
			partialFreq = m_nph->frequency() * std::exp2(octaveDif);
		}

		m_partialFreq[i] = partialFreq;
		m_partialGain[i] = partialGain;
		m_partialRadius[i] = partialRadius;
	}
}


inline void sSynth::updateComb(int partialNum, float partialFreqMax, float partialComb1, float partialComb2, float partialCombShift)
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


inline float sSynth::calcResonator(float input, int index, float frequency, float feedback, float leak)
{
	if (frequency != m_partialLastFreq[index] || feedback != m_partialLastFeedback[index])
	{
		m_partialLastFreq[index] = frequency;
		m_partialLastFeedback[index] = feedback;
		
		const float w = F_2PI * frequency / m_sampleRate;
		m_partialCosW[index] = qFastCos(w);
		m_partialSinWAbs[index] = qFastSin(w);
		m_partialCosWFeed[index] = m_partialCosW[index] * feedback;
		m_partialSinWFeed[index] = m_partialSinWAbs[index] * feedback;
		//m_partialSinWAbs[index] = m_partialSinWAbs[index];
	}

	float N1 = m_partialCosWFeed[index] * m_partialDelay1[index] + -m_partialSinWFeed[index] * m_partialDelay2[index] + input / m_partialSinWAbs[index];
	float N2 = m_partialSinWFeed[index] * m_partialDelay1[index] + m_partialCosWFeed[index] * m_partialDelay2[index] + input / m_partialSinWAbs[index] * leak;
	m_partialDelay1[index] = N1;
	m_partialDelay2[index] = N2;

	return N2 + N1 * leak;
	
	/*
	const float w = F_2PI * frequency / m_sampleRate;
	const float c1 = feedback * qFastCos(w);
	const float sinw = qFastSin(w);
	const float c2 = feedback * sinw;
	float N1 = c1 * m_partialDelay1[index] + -c2 * m_partialDelay2[index] + input / abs(sinw);
	float N2 = c2 * m_partialDelay1[index] + c1 * m_partialDelay2[index] + input / abs(sinw);
	m_partialDelay1[index] = N1;
	m_partialDelay2[index] = N2;

	return N1 + N2;
	*/
}


inline float sSynth::calcLogistic(float input, float max, float slope, float midpoint)
{
	return max / (1.f + expf(-slope * (input - midpoint)));
}


// Handles negative values properly, unlike fmod.
inline float sSynth::realfmod(float k, float n)
{
	float r = fmod(k, n);
	return r < 0 ? r + n : r;
}

// Handles negative values properly, unlike regular modulo.
inline int sSynth::realmod(int k, int n)
{
	int r = k % n;
	return r < 0 ? r + n : r;
}


inline sample_t sSynth::calcAllpassFilter(sample_t inSamp, float b0, float b1)
{
	float filterOutput = b0 * (inSamp - m_APFy[1]) + b1 * (m_APFx[0] - m_APFy[0]) + m_APFx[1];

	m_APFx[1] = m_APFx[0];
	m_APFx[0] = inSamp;
	m_APFy[1] = m_APFy[0];
	m_APFy[0] = filterOutput;

	return filterOutput;
}


inline float sSynth::calcEnvelope(float x, float a, float d1, float b, float d2, float s, float r, float &gainStorage)
{
	float y = 0;

	if (x < a)
	{
		y = expCurve(x / a, 0.f, 1.f);
	}
	else if (x < a + d1)
	{
		y = linearInterpolate(1.f, b, (x - a) / d1);
	}
	else //if (x < a + d1 + d2)
	{
		//y = expCurve((x - a - d1) / d2, b, s);
		y = gainStorage * (b - s) + s;
		gainStorage *= msToCoeff(d2);
	}
	/*else
	{
		y = s;
	}*/

	if( m_noteReleaseTime )
	{
		if( x < r + m_noteReleaseTime )
		{
			y *= (-1 / r) * (x - m_noteReleaseTime) + 1;
		}
		else
		{
			y = 0;
		}
	}

	return y;
}


inline float sSynth::expCurve(float x, float a, float b)
{
	return (-powf(x, 1.f / F_E) + 1.f) * (a - b) + b;
}


inline float sSynth::msToCoeff(float ms)
{
	// Convert time in milliseconds to applicable lowpass coefficient
	return exp(m_coeffPrecalc / ms);
}


Mercury::Mercury( InstrumentTrack * _instrument_track ) :
	Instrument( _instrument_track, &mercury_plugin_descriptor ),
	m_decay( 100, 1, 10000, 0.01, this, tr( "Decay" ) ),
	m_partialNum( 100, 1, PARTIAL_MAX, 1, this, tr( "Number of Partials" ) ),
	m_partialFreqMax( 16000, 8000, PARTIAL_FREQ_TRUE_MAX, 0.01, this, tr( "Maximum Partial Frequency" ) ),
	m_partialStretch( 1, 0, 8, 0.001, this, tr( "Partial Stretch" ) ),
	m_partialSine( 0, 0, 1, 0.0001, this, tr( "Partial Sine" ) ),
	m_partialLP( PARTIAL_MAX, 0, PARTIAL_MAX, 0.001, this, tr( "Partial Lowpass" ) ),
	m_partialLPSlope( 1, 0, 10, 0.001, this, tr( "Partial Lowpass Slope" ) ),
	m_partialDamp( 1, 0, 1, 0.001, this, tr( "Partial Damping" ) ),
	m_fundamental( 1, 0, 1, 0.0001, this, tr( "Fundamental" ) ),
	m_partialComb1( 4, 1, 20, 0.0001, this, tr( "Partial Comb 1" ) ),
	m_partialComb2( 4, 1, 20, 0.0001, this, tr( "Partial Comb 2" ) ),
	m_partialCombShift( 0, 0, 1, 0.0001, this, tr( "Partial Comb Shift" ) ),
	m_partialCombBal( 0, -1, 1, 0.0001, this, tr( "Partial Comb Bal" ) ),
	m_partialRing( 0, 0, 1, 0.0001, this, tr( "Partial Comb Ring Modulation" ) ),
	m_partialRound( 0, 0, 1200, 0.0001, this, tr( "Partial Rounding" ) ),
	m_partialRoundMax( 128, 0, 128, 1.0, this, tr( "Partial Rounding Maximum" ) ),
	m_partialFreqBal( 0, -1, 2, 0.0001, this, tr( "Partial Frequency Balance" ) ),
	m_exciterLP( 20000, 20, 20000, 0.01, this, tr( "Exciter Lowpass" ) ),
	m_exciterHP( 20, 20, 20000, 0.01, this, tr( "Exciter Highpass" ) ),
	m_exciterImpulse( 0, 0, 1, 0.0001, this, tr( "Exciter Impulse" ) ),
	m_exciterNoise( 0.33, 0, 1, 0.0001, this, tr( "Exciter Noise" ) ),
	m_partialProtect( 0, 0, PARTIAL_MAX, 0.0001, this, tr( "Partial Protection" ) ),
	m_partialProtectSlope( 1, 0, 10, 0.0001, this, tr( "Partial Protection Slope" ) ),
	m_exciterFeedback( 0, -0.1, 0.1, 0.000001, this, tr( "Exciter Feedback" ) ),
	m_exciterFeedbackDelay( 10, 1, 10000, 0.0001, this, tr( "Exciter Feedback Delay" ) ),
	m_modalVol( 50, 0, 200, 0.0001, this, tr( "Modal Output Volume" ) ),
	m_exciterFeedbackPhase( 200, 20, 20000, 0.1, this, tr( "Exciter Feedback Phase Shift" ) ),
	m_exciterFade( 0, 0, 1, 0.0001, this, tr( "Exciter/Modal Crossfade" ) ),
	m_exciterDelayFade( 0, -1, 1, 0.0001, this, tr( "Exciter Delay Crossfade" ) ),
	m_exciterAttack( 0, 0, 4000, 0.01, this, tr( "Exciter Attack" ) ),
	m_exciterDecay1( 0, 0, 2000, 0.01, this, tr( "Exciter Decay 1" ) ),
	m_exciterBreakpoint( 1, 0, 1, 0.0001, this, tr( "Exciter Breakpoint" ) ),
	m_exciterDecay2( 1, 0, 50, 0.0001, this, tr( "Exciter Decay 2" ) ),
	m_exciterSustain( 0, 0, 1, 0.0001, this, tr( "Exciter Sustain" ) ),
	m_exciterRelease( 50, 0, 4000, 0.01, this, tr( "Exciter Release" ) ),
	m_exciterSaturation( 0, 0, 1, 0.0001, this, tr( "Exciter Saturation" ) ),
	m_exciterAsym( 0, 0, 1, 0.0001, this, tr( "Exciter Saturation Asymmetry" ) ),
	m_exciterPulseTrain( 0, 0, 1, 0.0001, this, tr( "Exciter Feedback Pulse Train" ) ),
	m_exciterFeedbackLP( 20000, 20, 20000, 0.1, this, tr( "Exciter Feedback Lowpass" ) ),
	m_reverbMix( 0, 0, 1, 0.001, this, tr( "Reverb Mix" ) ),
	m_reverbDecay( 0.9, 0, 1, 0.001, this, tr( "Reverb Decay" ) ),
	m_reverbSize( 50, 1, 200, 0.001, this, tr( "Reverb Size" ) ),
	m_reverbDiffusion( 0.8, 0, 1, 0.001, this, tr( "Reverb Diffusion" ) ),
	m_reverbHighDamp( 20000, 20, 20000, 0.1, this, tr( "Reverb High-Frequency Damping" ) ),
	m_reverbLowDamp( 20, 20, 20000, 0.1, this, tr( "Reverb Low-Frequency Damping" ) ),
	m_reverbMod( 0.2, 0, 1, 0.001, this, tr( "Reverb Modulation Amount" ) ),
	m_reverbModFreq( 9, 0.125, 40, 0.001, this, tr( "Reverb Modulation Frequency" ) ),
	m_reverbWidth( 1, 0, 2, 0.001, this, tr( "Reverb Stereo Width" ) ),
	m_flangerMix( 0, 0, 1, 0.001, this, tr( "Flanger Mix" ) ),
	m_flangerDelay( 2000, 20, 20000, 0.001, this, tr( "Flanger Delay" ) ),
	m_flangerDepth( 0.5, 0, 1, 0.001, this, tr( "Flanger LFO Depth" ) ),
	m_flangerRate( 0.1, 0.001, 40, 0.00001, this, tr( "Flanger LFO Rate" ) ),
	m_flangerFeedback( 0.8, -1, 1, 0.001, this, tr( "Flanger Feedback" ) ),
	m_flangerDamping( 4000, 20, 20000, 0.001, this, tr( "Flanger Damping" ) ),
	m_flangerPhase( 0.25, 0, 1, 0.0001, this, tr( "Flanger Stereo Phase" ) ),
	m_delayMix( 0, 0, 1, 0.0001, this, tr( "Delay Mix" ) ),
	m_delayTime( 200, 1, MAX_DELAY_TIME, 0.0001, this, tr( "Delay Time" ) ),
	m_delayStereo( 0, -1, 1, 0.0001, this, tr( "Delay Stereo" ) ),
	m_delayHighDamp( 20000, 20, 20000, 0.1, this, tr( "Delay High-Frequency Damping" ) ),
	m_delayLowDamp( 20, 20, 20000, 0.1, this, tr( "Delay Low-Frequency Damping" ) ),
	m_delayFeedback( 0.5, 0, 1, 0.0001, this, tr( "Delay Feedback" ) ),
	m_exciterLeak( 0, 0, 1, 0.0001, this, tr( "Exciter Leakage" ) )
{
	m_sampleRate = Engine::audioEngine()->processingSampleRate();

	for (int i = 0; i < 4; ++i)
	{
		m_reverbBuf[i][0].resize(m_sampleRate + 1);
		m_reverbBuf[i][1].resize(m_sampleRate + 1);
		m_reverbBufSize[i] = m_sampleRate + 1;
		m_reverbSizeMult[i] = powf(0.86379373, i);// Completely arbitrary constant
	}

	m_flangerBuf[0].resize(m_sampleRate + 1);
	m_flangerBuf[1].resize(m_sampleRate + 1);
	m_flangerBufSize = m_sampleRate + 1;

	m_delayBuf[0].resize(MAX_DELAY_TIME * 0.001 * m_sampleRate * 2 + 1);
	m_delayBuf[1].resize(MAX_DELAY_TIME * 0.001 * m_sampleRate * 2 + 1);
	m_delayBufSize = MAX_DELAY_TIME * 0.001 * m_sampleRate * 2 + 1;

	m_decay.setScaleLogarithmic(true);
	m_partialStretch.setScaleLogarithmic(true);
	m_partialLP.setScaleLogarithmic(true);
	m_partialLPSlope.setScaleLogarithmic(true);
	m_partialRound.setScaleLogarithmic(true);
	m_partialRoundMax.setScaleLogarithmic(true);
	m_exciterLP.setScaleLogarithmic(true);
	m_exciterHP.setScaleLogarithmic(true);
	m_partialProtect.setScaleLogarithmic(true);
	m_partialProtectSlope.setScaleLogarithmic(true);
	m_exciterFeedbackDelay.setScaleLogarithmic(true);
	m_exciterFeedbackPhase.setScaleLogarithmic(true);
	m_exciterAttack.setScaleLogarithmic(true);
	m_exciterDecay1.setScaleLogarithmic(true);
	m_exciterDecay2.setScaleLogarithmic(true);
	m_exciterRelease.setScaleLogarithmic(true);
	m_reverbSize.setScaleLogarithmic(true);
	m_flangerDelay.setScaleLogarithmic(true);
	m_flangerRate.setScaleLogarithmic(true);
	m_flangerDamping.setScaleLogarithmic(true);

	connect(&m_decay, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialStretch, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialSine, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialLP, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialLPSlope, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialDamp, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_fundamental, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialProtect, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialProtectSlope, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialRound, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialRoundMax, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	connect(&m_partialFreqBal, SIGNAL(dataChanged()), this, SLOT(updatePartials()));
	
	connect(&m_partialNum, SIGNAL(dataChanged()), this, SLOT(updateComb()));
	connect(&m_partialFreqMax, SIGNAL(dataChanged()), this, SLOT(updateComb()));
	connect(&m_partialComb1, SIGNAL(dataChanged()), this, SLOT(updateComb()));
	connect(&m_partialComb2, SIGNAL(dataChanged()), this, SLOT(updateComb()));
	connect(&m_partialCombShift, SIGNAL(dataChanged()), this, SLOT(updateComb()));

	InstrumentPlayHandle * iph = new InstrumentPlayHandle( this, _instrument_track );
	Engine::audioEngine()->addPlayHandle( iph );
}



Mercury::~Mercury()
{
}


void Mercury::updatePartials()
{
	ConstNotePlayHandleList m_nphList = NotePlayHandle::nphsOfInstrumentTrack(instrumentTrack(), true);

	for (int i = 0; i < m_nphList.size(); ++i)
	{
		if (static_cast<sSynth *>(m_nphList[i]->m_pluginData))
		{
			static_cast<sSynth *>(m_nphList[i]->m_pluginData)->updatePartials(m_decay.value(), m_partialStretch.value(), m_partialSine.value(), m_partialLP.value(), m_partialLPSlope.value(), m_partialDamp.value(), m_fundamental.value(), m_partialComb1.value(), m_partialComb2.value(), m_partialCombShift.value(), m_partialCombBal.value(), m_partialRing.value(), m_partialRound.value(), m_partialRoundMax.value(), m_partialFreqBal.value(), m_exciterLP.value(), m_exciterHP.value(), m_exciterImpulse.value(), m_exciterNoise.value(), m_partialProtect.value(), m_partialProtectSlope.value());
		}
	}
}


void Mercury::updateComb()
{
	ConstNotePlayHandleList m_nphList = NotePlayHandle::nphsOfInstrumentTrack(instrumentTrack(), true);

	for (int i = 0; i < m_nphList.size(); ++i)
	{
		if (static_cast<sSynth *>(m_nphList[i]->m_pluginData))
		{
			static_cast<sSynth *>(m_nphList[i]->m_pluginData)->updateComb(m_partialNum.value(), m_partialFreqMax.value(), m_partialComb1.value(), m_partialComb2.value(), m_partialCombShift.value());
		}
	}
}


void Mercury::play( sampleFrame * _working_buffer )
{
	const fpp_t frames = Engine::audioEngine()->framesPerPeriod();

	ConstNotePlayHandleList m_nphList = NotePlayHandle::nphsOfInstrumentTrack(instrumentTrack(), true);

	for (int i = 0; i < m_nphList.size(); ++i)
	{
		/*for( fpp_t frame = 0; frame < frames; ++frame )
		{
			for( ch_cnt_t chnl = 0; chnl < DEFAULT_CHANNELS; ++chnl )
			{
				_working_buffer[frame][chnl] += m_nphList[i]->buffer()[frame][chnl];
			}
		}*/

		NotePlayHandle * currentNote = const_cast<NotePlayHandle *>(m_nphList[i]);

		if ( currentNote->totalFramesPlayed() == 0 || currentNote->m_pluginData == NULL )
		{
			currentNote->m_pluginData = new sSynth( currentNote, Engine::audioEngine()->processingSampleRate() );
		}
	}
	
	emit updatePartials();
	emit updateComb();

	for (int i = 0; i < m_nphList.size(); ++i)
	{
		NotePlayHandle * currentNote = const_cast<NotePlayHandle *>(m_nphList[i]);
		
		const fpp_t frames = currentNote->framesLeftForCurrentPeriod();
		//const fpp_t fpp = Engine::audioEngine()->framesPerPeriod();
		const f_cnt_t offset = currentNote->noteOffset();

		sSynth * ps = static_cast<sSynth *>( currentNote->m_pluginData );
		for( fpp_t frame = offset; frame < frames + offset; ++frame )
		{
			sampleFrame outputSample = {0,0};

			ps->nextStringSample(outputSample, frame, m_decay.value(), m_partialNum.value(), m_partialFreqMax.value(), m_partialStretch.value(), m_partialSine.value(), m_partialLP.value(), m_partialLPSlope.value(), m_partialDamp.value(), m_fundamental.value(), m_partialComb1.value(), m_partialComb2.value(), m_partialCombShift.value(), m_partialCombBal.value(), m_partialRing.value(), m_partialRound.value(), m_partialRoundMax.value(), m_partialFreqBal.value(), m_exciterLP.value(), m_exciterHP.value(), m_exciterImpulse.value(), m_exciterNoise.value(), m_partialProtect.value(), m_partialProtectSlope.value(), m_exciterFeedback.value(), m_exciterFeedbackDelay.value(), m_modalVol.value(), m_exciterFeedbackPhase.value(), m_exciterFade.value(), m_exciterDelayFade.value(), m_exciterAttack.value(), m_exciterDecay1.value(), m_exciterBreakpoint.value(), m_exciterDecay2.value(), m_exciterSustain.value(), m_exciterRelease.value(), m_exciterSaturation.value(), m_exciterAsym.value(), m_exciterPulseTrain.value(), m_exciterFeedbackLP.value(), m_reverbMix.value(), m_reverbDecay.value(), m_reverbSize.value(), m_reverbDiffusion.value(), m_reverbHighDamp.value(), m_reverbLowDamp.value(), m_reverbMod.value(), m_reverbModFreq.value(), m_reverbWidth.value(), m_flangerMix.value(), m_flangerDelay.value(), m_flangerDepth.value(), m_flangerRate.value(), m_flangerFeedback.value(), m_flangerDamping.value(), m_flangerPhase.value(), m_delayMix.value(), m_delayTime.value(), m_delayStereo.value(), m_delayHighDamp.value(), m_delayLowDamp.value(), m_delayFeedback.value(), m_exciterLeak.value());

			_working_buffer[frame][0] += outputSample[0];
			_working_buffer[frame][1] = _working_buffer[frame][0];
		}
	}


	// FLANGER
	if (m_flangerMix.value() > 0)
	{
		for( fpp_t frame = 0; frame < frames; ++frame )
		{
			++m_flangerBufIndex;
			if (m_flangerBufIndex >= m_flangerBufSize)
			{
				m_flangerBufIndex = 0;
			}

			float flangerDelayOutput[2] = {0};
			for (int j = 0; j < 2; ++j)
			{
				float samplesBack = m_flangerDelay.value();
				samplesBack *= std::exp2(qFastSin(F_2PI * ((j ? 0 : m_flangerPhase.value()) + m_lifetime * m_flangerRate.value() / m_sampleRate)) * 4.f * m_flangerDepth.value());
				samplesBack = m_sampleRate / samplesBack;

				float newFlangerBufIndex = realfmod(m_flangerBufIndex - samplesBack, m_flangerBufSize);
				float newFlangerBufIndexFrac = fraction(newFlangerBufIndex);

				if (newFlangerBufIndex < m_flangerBufSize - 1)
				{
					flangerDelayOutput[j] = m_flangerBuf[j][floor(newFlangerBufIndex)] * (1.f - newFlangerBufIndexFrac) + m_flangerBuf[j][ceil(newFlangerBufIndex)] * newFlangerBufIndexFrac;
				}
				else// For when the interpolation wraps around to the beginning of the buffer
				{
					flangerDelayOutput[j] = m_flangerBuf[j][m_flangerBufSize - 1] * (1.f - newFlangerBufIndexFrac) + m_flangerBuf[j][0] * newFlangerBufIndexFrac;
				}

				const float a = 1.f - expf(-(F_2PI * m_flangerDamping.value() / m_sampleRate));
				m_flangerDampingOut[j] = flangerDelayOutput[j] * a + (1.f - a) * m_flangerDampingOut[j];
			}

			for (int j = 0; j < 2; ++j)
			{
				m_flangerBuf[j][m_flangerBufIndex] = _working_buffer[frame][j] + m_flangerFeedback.value() * m_flangerDampingOut[j];
			}

			_working_buffer[frame][0] = linearInterpolate(_working_buffer[frame][0], flangerDelayOutput[0], m_flangerMix.value() * 0.5f);
			_working_buffer[frame][1] = linearInterpolate(_working_buffer[frame][1], flangerDelayOutput[1], m_flangerMix.value() * 0.5f);
		}
	}


	// DELAY
	if (m_delayMix.value() > 0)
	{
		for( fpp_t frame = 0; frame < frames; ++frame )
		{
			++m_delayBufIndex;
			if (m_delayBufIndex >= m_delayBufSize)
			{
				m_delayBufIndex = 0;
			}

			float delayDelayOutput[2] = {0};
			for (int j = 0; j < 2; ++j)
			{
				float samplesBack = (m_delayTime.value() * (j ? m_delayStereo.value() + 1 : 1)) * 0.001f * m_sampleRate;

				float newDelayBufIndex = realfmod(m_delayBufIndex - samplesBack, m_delayBufSize);
				float newDelayBufIndexFrac = fraction(newDelayBufIndex);

				if (newDelayBufIndex < m_delayBufSize - 1)
				{
					delayDelayOutput[j] = m_delayBuf[j][floor(newDelayBufIndex)] * (1.f - newDelayBufIndexFrac) + m_delayBuf[j][ceil(newDelayBufIndex)] * newDelayBufIndexFrac;
				}
				else// For when the interpolation wraps around to the beginning of the buffer
				{
					delayDelayOutput[j] = m_delayBuf[j][m_delayBufSize - 1] * (1.f - newDelayBufIndexFrac) + m_delayBuf[j][0] * newDelayBufIndexFrac;
				}

				const float lastHighDampOut = m_delayHighDampOut[j];

				const float a = 1.f - expf(-(F_2PI * m_delayHighDamp.value() / m_sampleRate));
				m_delayHighDampOut[j] = delayDelayOutput[j] * a + (1.f - a) * m_delayHighDampOut[j];

				const float a2 = expf(-(F_2PI * m_delayLowDamp.value() / m_sampleRate));
				m_delayLowDampOut[j] = a2 * m_delayLowDampOut[j] + a2 * (m_delayHighDampOut[j] - lastHighDampOut);
			}

			for (int j = 0; j < 2; ++j)
			{
				m_delayBuf[j][m_delayBufIndex] = _working_buffer[frame][j] + m_delayFeedback.value() * m_delayLowDampOut[j];
			}

			// We send the damped signal to the output here, unlike the flanger and reverb
			_working_buffer[frame][0] = linearInterpolate(_working_buffer[frame][0], m_delayLowDampOut[0], m_delayMix.value());
			_working_buffer[frame][1] = linearInterpolate(_working_buffer[frame][1], m_delayLowDampOut[1], m_delayMix.value());
		}
	}


	// REVERB
	if (m_reverbMix.value() > 0)
	{
		for( fpp_t frame = 0; frame < frames; ++frame )
		{
			float reverbDelayOutput[4][2] = {{0}};
			for (int i = 0; i < 4; ++i)
			{
				++m_reverbBufIndex[i];
				if (m_reverbBufIndex[i] >= m_reverbBufSize[i])
				{
					m_reverbBufIndex[i] = 0;
				}
				for (int j = 0; j < 2; ++j)
				{
					
					float samplesBack = m_reverbSizeMult[i] * m_reverbSize.value() * 0.001f * m_sampleRate;
					samplesBack *= qFastSin(F_2PI * ((i * 0.25f) + (j * 0.25f) + m_lifetime * m_reverbModFreq.value() / m_sampleRate)) * 0.0125f * m_reverbMod.value() + 1.f;

					float newReverbBufIndex = realfmod(m_reverbBufIndex[i] - samplesBack, m_reverbBufSize[i]);
					float newReverbBufIndexFrac = fraction(newReverbBufIndex);

					if (newReverbBufIndex < m_reverbBufSize[i] - 1)
					{
						reverbDelayOutput[i][j] = m_reverbBuf[i][j][floor(newReverbBufIndex)] * (1.f - newReverbBufIndexFrac) + m_reverbBuf[i][j][ceil(newReverbBufIndex)] * newReverbBufIndexFrac;
					}
					else// For when the interpolation wraps around to the beginning of the buffer
					{
						reverbDelayOutput[i][j] = m_reverbBuf[i][j][m_reverbBufSize[i] - 1] * (1.f - newReverbBufIndexFrac) + m_reverbBuf[i][j][0] * newReverbBufIndexFrac;
					}
				}
			}

			for (int j = 0; j < 2; ++j)
			{
				m_reverbBuf[0][j][m_reverbBufIndex[0]] = _working_buffer[frame][j] + m_reverbDecay.value() * 0.5f * ( reverbDelayOutput[0][j] + reverbDelayOutput[1][j] + reverbDelayOutput[2][j] + reverbDelayOutput[3][j]);
				m_reverbBuf[1][j][m_reverbBufIndex[1]] = _working_buffer[frame][j] + m_reverbDecay.value() * 0.5f * (-reverbDelayOutput[0][j] + reverbDelayOutput[1][j] - reverbDelayOutput[2][j] + reverbDelayOutput[3][j]);
				m_reverbBuf[2][j][m_reverbBufIndex[2]] = _working_buffer[frame][j] + m_reverbDecay.value() * 0.5f * (-reverbDelayOutput[0][j] - reverbDelayOutput[1][j] + reverbDelayOutput[2][j] + reverbDelayOutput[3][j]);
				m_reverbBuf[3][j][m_reverbBufIndex[3]] = _working_buffer[frame][j] + m_reverbDecay.value() * 0.5f * ( reverbDelayOutput[0][j] - reverbDelayOutput[1][j] - reverbDelayOutput[2][j] + reverbDelayOutput[3][j]);
			}

			float reverbOutput[2] = {0};
			for (int j = 0; j < 2; ++j)
			{
				reverbOutput[j] = reverbDelayOutput[0][j] + reverbDelayOutput[1][j] + reverbDelayOutput[2][j] + reverbDelayOutput[3][j];
			}

			const float reverbMonoVal = (reverbOutput[0] + reverbOutput[1]) * 0.5f;
			reverbOutput[0] = linearInterpolate(reverbMonoVal, reverbOutput[0], m_reverbWidth.value());
			reverbOutput[1] = linearInterpolate(reverbMonoVal, reverbOutput[1], m_reverbWidth.value());

			for (int i = 0; i < 4; ++i)
			{
				for (int j = 0; j < 2; ++j)
				{
					const float a = 1.f - expf(-(F_2PI * m_reverbHighDamp.value() / m_sampleRate));
					const float lastHighDampOut = m_reverbHighDampOut[i][j];
					m_reverbHighDampOut[i][j] = m_reverbBuf[i][j][m_reverbBufIndex[i]] * a + (1.f - a) * m_reverbHighDampOut[i][j];
					const float a2 = expf(-(F_2PI * m_reverbLowDamp.value() / m_sampleRate));
					m_reverbLowDampOut[i][j] = a2 * m_reverbLowDampOut[i][j] + a2 * (m_reverbHighDampOut[i][j] - lastHighDampOut);

					const float w0 = (F_2PI / m_sampleRate) * linearInterpolate(2000, 20, m_reverbDiffusion.value());
					const float a0 = 1 + (qFastSin(w0) / (0.707 * 2.f));// 0.707 = resonance
					const float b0 = (1 - (a0 - 1)) / a0;
					const float b1 = (-2*cos(w0)) / a0;
					const float allpassOutput = calcAllpassFilter(m_reverbLowDampOut[i][j], b0, b1, i, j);

					m_reverbBuf[i][j][m_reverbBufIndex[i]] = allpassOutput;
				}
			}

			_working_buffer[frame][0] = linearInterpolate(_working_buffer[frame][0], reverbOutput[0], m_reverbMix.value());
			_working_buffer[frame][1] = linearInterpolate(_working_buffer[frame][1], reverbOutput[1], m_reverbMix.value());

			++m_lifetime;
		}
	}



	/*
	for( ch_cnt_t chnl = 0; chnl < DEFAULT_CHANNELS; ++chnl )
	{
		_working_buffer[frame][chnl] += outputSample[chnl];
	}
	*/

	instrumentTrack()->processAudioBuffer( _working_buffer, frames, NULL );
}


f_cnt_t Mercury::desiredReleaseFrames() const
{
	return int(m_exciterRelease.value() * 0.001f * Engine::audioEngine()->processingSampleRate()) + 256 + 1;
}


// Handles negative values properly, unlike fmod.
inline float Mercury::realfmod(float k, float n)
{
	float r = fmod(k, n);
	return r < 0 ? r + n : r;
}


inline sample_t Mercury::calcAllpassFilter(sample_t inSamp, float b0, float b1, int num, int chnl)
{
	float filterOutput = b0 * (inSamp - m_APFy[num][chnl][1]) + b1 * (m_APFx[num][chnl][0] - m_APFy[num][chnl][0]) + m_APFx[num][chnl][1];

	m_APFx[num][chnl][1] = m_APFx[num][chnl][0];
	m_APFx[num][chnl][0] = inSamp;
	m_APFy[num][chnl][1] = m_APFy[num][chnl][0];
	m_APFy[num][chnl][0] = filterOutput;

	return filterOutput;
}


void Mercury::saveSettings( QDomDocument & doc, QDomElement & _this )
{
	// Save plugin version
	_this.setAttribute( "version", "1.0.2" );

	m_decay.saveSettings( doc, _this, "decay" );
	m_partialNum.saveSettings( doc, _this, "partialNum" );
	m_partialFreqMax.saveSettings( doc, _this, "partialFreqMax" );
	m_partialStretch.saveSettings( doc, _this, "partialStretch" );
	m_partialSine.saveSettings( doc, _this, "partialSine" );
	m_partialLP.saveSettings( doc, _this, "partialLP" );
	m_partialLPSlope.saveSettings( doc, _this, "partialLPSlope" );
	m_partialDamp.saveSettings( doc, _this, "partialDamp" );
	m_fundamental.saveSettings( doc, _this, "fundamental" );
	m_partialComb1.saveSettings( doc, _this, "partialComb1" );
	m_partialComb2.saveSettings( doc, _this, "partialComb2" );
	m_partialCombShift.saveSettings( doc, _this, "partialCombShift" );
	m_partialCombBal.saveSettings( doc, _this, "partialCombBal" );
	m_partialRing.saveSettings( doc, _this, "partialRing" );
	m_partialRound.saveSettings( doc, _this, "partialRound" );
	m_partialRoundMax.saveSettings( doc, _this, "partialRoundMax" );
	m_partialFreqBal.saveSettings( doc, _this, "partialFreqBal" );
	m_exciterLP.saveSettings( doc, _this, "exciterLP" );
	m_exciterHP.saveSettings( doc, _this, "exciterHP" );
	m_exciterImpulse.saveSettings( doc, _this, "exciterImpulse" );
	m_exciterNoise.saveSettings( doc, _this, "exciterNoise" );
	m_partialProtect.saveSettings( doc, _this, "partialProtect" );
	m_partialProtectSlope.saveSettings( doc, _this, "partialProtectSlope" );
	m_exciterFeedback.saveSettings( doc, _this, "exciterFeedback" );
	m_exciterFeedbackDelay.saveSettings( doc, _this, "exciterFeedbackDelay" );
	m_modalVol.saveSettings( doc, _this, "modalVol" );
	m_exciterFeedbackPhase.saveSettings( doc, _this, "exciterFeedbackPhase" );
	m_exciterFade.saveSettings( doc, _this, "exciterFade" );
	m_exciterDelayFade.saveSettings( doc, _this, "exciterDelayFade" );
	m_exciterAttack.saveSettings( doc, _this, "exciterAttack" );
	m_exciterDecay1.saveSettings( doc, _this, "exciterDecay1" );
	m_exciterBreakpoint.saveSettings( doc, _this, "exciterBreakpoint" );
	m_exciterDecay2.saveSettings( doc, _this, "exciterDecay2" );
	m_exciterSustain.saveSettings( doc, _this, "exciterSustain" );
	m_exciterRelease.saveSettings( doc, _this, "exciterRelease" );
	m_exciterSaturation.saveSettings( doc, _this, "exciterSaturation" );
	m_exciterAsym.saveSettings( doc, _this, "exciterAsym" );
	m_exciterPulseTrain.saveSettings( doc, _this, "exciterPulseTrain" );
	m_exciterFeedbackLP.saveSettings( doc, _this, "exciterFeedbackLP" );
	m_reverbMix.saveSettings( doc, _this, "reverbMix" );
	m_reverbDecay.saveSettings( doc, _this, "reverbDecay" );
	m_reverbSize.saveSettings( doc, _this, "reverbSize" );
	m_reverbDiffusion.saveSettings( doc, _this, "reverbDiffusion" );
	m_reverbHighDamp.saveSettings( doc, _this, "reverbHighDamp" );
	m_reverbLowDamp.saveSettings( doc, _this, "reverbLowDamp" );
	m_reverbMod.saveSettings( doc, _this, "reverbMod" );
	m_reverbModFreq.saveSettings( doc, _this, "reverbModFreq" );
	m_reverbWidth.saveSettings( doc, _this, "reverbWidth" );
	m_flangerMix.saveSettings( doc, _this, "flangerMix" );
	m_flangerDelay.saveSettings( doc, _this, "flangerDelay" );
	m_flangerDepth.saveSettings( doc, _this, "flangerDepth" );
	m_flangerRate.saveSettings( doc, _this, "flangerRate" );
	m_flangerFeedback.saveSettings( doc, _this, "flangerFeedback" );
	m_flangerDamping.saveSettings( doc, _this, "flangerDamping" );
	m_flangerPhase.saveSettings( doc, _this, "flangerPhase" );
	m_delayMix.saveSettings( doc, _this, "delayMix" );
	m_delayTime.saveSettings( doc, _this, "delayTime" );
	m_delayStereo.saveSettings( doc, _this, "delayStereo" );
	m_delayHighDamp.saveSettings( doc, _this, "delayHighDamp" );
	m_delayLowDamp.saveSettings( doc, _this, "delayLowDamp" );
	m_delayFeedback.saveSettings( doc, _this, "delayFeedback" );
	m_exciterLeak.saveSettings( doc, _this, "exciterLeak" );
}



void Mercury::loadSettings( const QDomElement & _this )
{
	m_decay.loadSettings( _this, "decay" );
	m_partialNum.loadSettings( _this, "partialNum" );
	m_partialFreqMax.loadSettings( _this, "partialFreqMax" );
	m_partialStretch.loadSettings( _this, "partialStretch" );
	m_partialSine.loadSettings( _this, "partialSine" );
	m_partialLP.loadSettings( _this, "partialLP" );
	m_partialLPSlope.loadSettings( _this, "partialLPSlope" );
	m_partialDamp.loadSettings( _this, "partialDamp" );
	m_fundamental.loadSettings( _this, "fundamental" );
	m_partialComb1.loadSettings( _this, "partialComb1" );
	m_partialComb2.loadSettings( _this, "partialComb2" );
	m_partialCombShift.loadSettings( _this, "partialCombShift" );
	m_partialCombBal.loadSettings( _this, "partialCombBal" );
	m_partialRing.loadSettings( _this, "partialRing" );
	m_partialRound.loadSettings( _this, "partialRound" );
	m_partialRoundMax.loadSettings( _this, "partialRoundMax" );
	m_partialFreqBal.loadSettings( _this, "partialFreqBal" );
	m_exciterLP.loadSettings( _this, "exciterLP" );
	m_exciterHP.loadSettings( _this, "exciterHP" );
	m_exciterImpulse.loadSettings( _this, "exciterImpulse" );
	m_exciterNoise.loadSettings( _this, "exciterNoise" );
	m_partialProtect.loadSettings( _this, "partialProtect" );
	m_partialProtectSlope.loadSettings( _this, "partialProtectSlope" );
	m_exciterFeedback.loadSettings( _this, "exciterFeedback" );
	m_exciterFeedbackDelay.loadSettings( _this, "exciterFeedbackDelay" );
	m_modalVol.loadSettings( _this, "modalVol" );
	m_exciterFeedbackPhase.loadSettings( _this, "exciterFeedbackPhase" );
	m_exciterFade.loadSettings( _this, "exciterFade" );
	m_exciterDelayFade.loadSettings( _this, "exciterDelayFade" );
	m_exciterAttack.loadSettings( _this, "exciterAttack" );
	m_exciterDecay1.loadSettings( _this, "exciterDecay1" );
	m_exciterBreakpoint.loadSettings( _this, "exciterBreakpoint" );
	m_exciterDecay2.loadSettings( _this, "exciterDecay2" );
	m_exciterSustain.loadSettings( _this, "exciterSustain" );
	m_exciterRelease.loadSettings( _this, "exciterRelease" );
	m_exciterSaturation.loadSettings( _this, "exciterSaturation" );
	m_exciterAsym.loadSettings( _this, "exciterAsym" );
	m_exciterPulseTrain.loadSettings( _this, "exciterPulseTrain" );
	m_exciterFeedbackLP.loadSettings( _this, "exciterFeedbackLP" );
	m_reverbMix.loadSettings( _this, "reverbMix" );
	m_reverbDecay.loadSettings( _this, "reverbDecay" );
	m_reverbSize.loadSettings( _this, "reverbSize" );
	m_reverbDiffusion.loadSettings( _this, "reverbDiffusion" );
	m_reverbHighDamp.loadSettings( _this, "reverbHighDamp" );
	m_reverbLowDamp.loadSettings( _this, "reverbLowDamp" );
	m_reverbMod.loadSettings( _this, "reverbMod" );
	m_reverbModFreq.loadSettings( _this, "reverbModFreq" );
	m_reverbWidth.loadSettings( _this, "reverbWidth" );
	m_flangerMix.loadSettings( _this, "flangerMix" );
	m_flangerDelay.loadSettings( _this, "flangerDelay" );
	m_flangerDepth.loadSettings( _this, "flangerDepth" );
	m_flangerRate.loadSettings( _this, "flangerRate" );
	m_flangerFeedback.loadSettings( _this, "flangerFeedback" );
	m_flangerDamping.loadSettings( _this, "flangerDamping" );
	m_flangerPhase.loadSettings( _this, "flangerPhase" );
	m_delayMix.loadSettings( _this, "delayMix" );
	m_delayTime.loadSettings( _this, "delayTime" );
	m_delayStereo.loadSettings( _this, "delayStereo" );
	m_delayHighDamp.loadSettings( _this, "delayHighDamp" );
	m_delayLowDamp.loadSettings( _this, "delayLowDamp" );
	m_delayFeedback.loadSettings( _this, "delayFeedback" );
	m_exciterLeak.loadSettings( _this, "exciterLeak" );
	
	auto thingy = _this.attribute( "version" );
	if (thingy == "1.0.0" || thingy == "1.0.1")
	{
		m_exciterLeak.setValue(1.f);
	}
}




QString Mercury::nodeName() const
{
	return( mercury_plugin_descriptor.name );
}




void Mercury::playNote( NotePlayHandle * _n, sampleFrame * _working_buffer )
{
	/*if ( _n->totalFramesPlayed() == 0 || _n->m_pluginData == NULL )
	{
		_n->m_pluginData = new sSynth( _n, Engine::audioEngine()->processingSampleRate() );
	}

	const fpp_t frames = _n->framesLeftForCurrentPeriod();
	const f_cnt_t offset = _n->noteOffset();

	sSynth * ps = static_cast<sSynth *>( _n->m_pluginData );
	for( fpp_t frame = offset; frame < frames + offset; ++frame )
	{
		sampleFrame outputSample = {0,0};

		ps->nextStringSample(outputSample, m_decay.value(), m_partialNum.value(), m_partialFreqMax.value(), m_partialStretch.value(), m_partialSine.value(), m_partialLP.value(), m_partialLPSlope.value(), m_partialDamp.value(), m_fundamental.value(), m_partialComb1.value(), m_partialComb2.value(), m_partialCombShift.value(), m_partialCombBal.value(), m_partialRing.value(), m_exciterLP.value(), m_exciterHP.value(), m_exciterImpulse.value(), m_exciterNoise.value(), m_partialProtect.value(), m_partialProtectSlope.value(), m_exciterFeedback.value(), m_exciterFeedbackDelay.value(), m_modalVol.value(), m_exciterFeedbackPhase.value(), m_exciterFade.value(), m_exciterDelayFade.value(), m_exciterAttack.value(), m_exciterDecay1.value(), m_exciterBreakpoint.value(), m_exciterDecay2.value(), m_exciterSustain.value(), m_exciterRelease.value(), m_exciterSaturation.value(), m_exciterAsym.value(), m_exciterPulseTrain.value(), m_exciterFeedbackLP.value());

		for( ch_cnt_t chnl = 0; chnl < DEFAULT_CHANNELS; ++chnl )
		{
			_working_buffer[frame][chnl] = outputSample[chnl];
		}
	}*/

	//applyRelease( _working_buffer, _n );

	//instrumentTrack()->processAudioBuffer( _working_buffer, frames + offset, _n );
}




void Mercury::deleteNotePluginData( NotePlayHandle * _n )
{
	delete static_cast<sSynth *>( _n->m_pluginData );
}




gui::PluginView * Mercury::instantiateView( QWidget * _parent )
{
	return( new gui::MercuryView( this, _parent ) );
}




namespace gui
{


MercuryView::MercuryView( Instrument * _instrument,
					QWidget * _parent ) :
	InstrumentView( _instrument, _parent )
{
	setAutoFillBackground( true );
	QPalette pal;

	pal.setBrush( backgroundRole(), PLUGIN_NAME::getIconPixmap(
								"artwork" ) );
	setPalette( pal );
	
	m_decayKnob = new Knob( knobSmall_17, this );
	m_decayKnob->move( 5, 5 );
	m_decayKnob->setHintText( tr( "Decay" ), "" );

	m_partialNumKnob = new Knob( knobSmall_17, this );
	m_partialNumKnob->move( 25, 5 );
	m_partialNumKnob->setHintText( tr( "Number of Partials" ), "" );

	m_partialFreqMaxKnob = new Knob( knobSmall_17, this );
	m_partialFreqMaxKnob->move( 45, 5 );
	m_partialFreqMaxKnob->setHintText( tr( "Maximum Partial Frequency" ), "" );

	m_partialStretchKnob = new Knob( knobSmall_17, this );
	m_partialStretchKnob->move( 65, 5 );
	m_partialStretchKnob->setHintText( tr( "Partial Stretch" ), "" );

	m_partialSineKnob = new Knob( knobSmall_17, this );
	m_partialSineKnob->move( 85, 5 );
	m_partialSineKnob->setHintText( tr( "Partial Sine" ), "" );

	m_partialLPKnob = new Knob( knobSmall_17, this );
	m_partialLPKnob->move( 105, 5 );
	m_partialLPKnob->setHintText( tr( "Partial Lowpass" ), "" );

	m_partialLPSlopeKnob = new Knob( knobSmall_17, this );
	m_partialLPSlopeKnob->move( 125, 5 );
	m_partialLPSlopeKnob->setHintText( tr( "Partial Lowpass Slope" ), "" );

	m_partialDampKnob = new Knob( knobSmall_17, this );
	m_partialDampKnob->move( 145, 5 );
	m_partialDampKnob->setHintText( tr( "Partial Damping" ), "" );

	m_fundamentalKnob = new Knob( knobSmall_17, this );
	m_fundamentalKnob->move( 165, 5 );
	m_fundamentalKnob->setHintText( tr( "Fundamental" ), "" );

	m_partialComb1Knob = new Knob( knobSmall_17, this );
	m_partialComb1Knob->move( 185, 5 );
	m_partialComb1Knob->setHintText( tr( "Comb 1:" ), "" );

	m_partialComb2Knob = new Knob( knobSmall_17, this );
	m_partialComb2Knob->move( 205, 5 );
	m_partialComb2Knob->setHintText( tr( "Comb 2:" ), "" );

	m_partialCombShiftKnob = new Knob( knobSmall_17, this );
	m_partialCombShiftKnob->move( 225, 5 );
	m_partialCombShiftKnob->setHintText( tr( "Comb Shift:" ), "" );

	m_partialCombBalKnob = new Knob( knobSmall_17, this );
	m_partialCombBalKnob->move( 5, 25 );
	m_partialCombBalKnob->setHintText( tr( "Comb Balance:" ), "" );

	m_partialRingKnob = new Knob( knobSmall_17, this );
	m_partialRingKnob->move( 25, 25 );
	m_partialRingKnob->setHintText( tr( "Comb Ring Modulation:" ), "" );
	
	m_partialRoundKnob = new Knob( knobSmall_17, this );
	m_partialRoundKnob->move( 30, 220 );
	m_partialRoundKnob->setHintText( tr( "Partial Rounding" ), " cents" );
	
	m_partialRoundMaxKnob = new Knob( knobSmall_17, this );
	m_partialRoundMaxKnob->move( 50, 220 );
	m_partialRoundMaxKnob->setHintText( tr( "Partial Rounding Maximum" ), " partials" );

	m_partialFreqBalKnob = new Knob( knobSmall_17, this );
	m_partialFreqBalKnob->move( 150, 220 );
	m_partialFreqBalKnob->setHintText( tr( "Partial Frequency Balance" ), "" );

	m_exciterLPKnob = new Knob( knobSmall_17, this );
	m_exciterLPKnob->move( 45, 25 );
	m_exciterLPKnob->setHintText( tr( "Exciter Lowpass:" ), " Hz" );

	m_exciterHPKnob = new Knob( knobSmall_17, this );
	m_exciterHPKnob->move( 65, 25 );
	m_exciterHPKnob->setHintText( tr( "Exciter Highpass:" ), " Hz" );

	m_exciterImpulseKnob = new Knob( knobSmall_17, this );
	m_exciterImpulseKnob->move( 85, 25 );
	m_exciterImpulseKnob->setHintText( tr( "Exciter Impulse:" ), "" );

	m_exciterNoiseKnob = new Knob( knobSmall_17, this );
	m_exciterNoiseKnob->move( 105, 25 );
	m_exciterNoiseKnob->setHintText( tr( "Exciter Noise:" ), "" );

	m_partialProtectKnob = new Knob( knobSmall_17, this );
	m_partialProtectKnob->move( 125, 25 );
	m_partialProtectKnob->setHintText( tr( "Partial Protection:" ), "" );

	m_partialProtectSlopeKnob = new Knob( knobSmall_17, this );
	m_partialProtectSlopeKnob->move( 145, 25 );
	m_partialProtectSlopeKnob->setHintText( tr( "Partial Protection Slope:" ), "" );

	m_exciterFeedbackKnob = new Knob( knobSmall_17, this );
	m_exciterFeedbackKnob->move( 165, 25 );
	m_exciterFeedbackKnob->setHintText( tr( "Exciter Feedback:" ), "" );

	m_exciterFeedbackDelayKnob = new Knob( knobSmall_17, this );
	m_exciterFeedbackDelayKnob->move( 185, 25 );
	m_exciterFeedbackDelayKnob->setHintText( tr( "Exciter Feedback Delay:" ), "" );

	m_modalVolKnob = new Knob( knobSmall_17, this );
	m_modalVolKnob->move( 205, 25 );
	m_modalVolKnob->setHintText( tr( "Modal Output Volume:" ), "" );

	m_exciterFeedbackPhaseKnob = new Knob( knobSmall_17, this );
	m_exciterFeedbackPhaseKnob->move( 225, 25 );
	m_exciterFeedbackPhaseKnob->setHintText( tr( "Exciter Feedback Phase:" ), "" );

	m_exciterFadeKnob = new Knob( knobSmall_17, this );
	m_exciterFadeKnob->move( 5, 45 );
	m_exciterFadeKnob->setHintText( tr( "Exciter/Modal Crossfade:" ), "" );

	m_exciterDelayFadeKnob = new Knob( knobSmall_17, this );
	m_exciterDelayFadeKnob->move( 25, 45 );
	m_exciterDelayFadeKnob->setHintText( tr( "Exciter Delay Crossfade:" ), "" );

	m_exciterAttackKnob = new Knob( knobSmall_17, this );
	m_exciterAttackKnob->move( 5, 65 );
	m_exciterAttackKnob->setHintText( tr( "Exciter Attack:" ), "" );

	m_exciterDecay1Knob = new Knob( knobSmall_17, this );
	m_exciterDecay1Knob->move( 25, 65 );
	m_exciterDecay1Knob->setHintText( tr( "Exciter Decay 1:" ), "" );

	m_exciterBreakpointKnob = new Knob( knobSmall_17, this );
	m_exciterBreakpointKnob->move( 45, 65 );
	m_exciterBreakpointKnob->setHintText( tr( "Exciter Breakpoint:" ), "" );

	m_exciterDecay2Knob = new Knob( knobSmall_17, this );
	m_exciterDecay2Knob->move( 65, 65 );
	m_exciterDecay2Knob->setHintText( tr( "Exciter Decay 2:" ), "" );

	m_exciterSustainKnob = new Knob( knobSmall_17, this );
	m_exciterSustainKnob->move( 85, 65 );
	m_exciterSustainKnob->setHintText( tr( "Exciter Sustain:" ), "" );

	m_exciterReleaseKnob = new Knob( knobSmall_17, this );
	m_exciterReleaseKnob->move( 105, 65 );
	m_exciterReleaseKnob->setHintText( tr( "Exciter Release:" ), "" );

	m_exciterSaturationKnob = new Knob( knobSmall_17, this );
	m_exciterSaturationKnob->move( 145, 65 );
	m_exciterSaturationKnob->setHintText( tr( "Exciter Saturation:" ), "" );

	m_exciterAsymKnob = new Knob( knobSmall_17, this );
	m_exciterAsymKnob->move( 165, 65 );
	m_exciterAsymKnob->setHintText( tr( "Exciter Saturation Asymmetry:" ), "" );

	m_exciterPulseTrainKnob = new Knob( knobSmall_17, this );
	m_exciterPulseTrainKnob->move( 185, 65 );
	m_exciterPulseTrainKnob->setHintText( tr( "Exciter Feedback Pulse Train:" ), "" );

	m_exciterFeedbackLPKnob = new Knob( knobSmall_17, this );
	m_exciterFeedbackLPKnob->move( 205, 65 );
	m_exciterFeedbackLPKnob->setHintText( tr( "Exciter Feedback Lowpass:" ), "" );

	m_reverbMixKnob = new Knob( knobSmall_17, this );
	m_reverbMixKnob->move( 5, 95 );
	m_reverbMixKnob->setHintText( tr( "Reverb Mix:" ), "" );

	m_reverbDecayKnob = new Knob( knobSmall_17, this );
	m_reverbDecayKnob->move( 25, 95 );
	m_reverbDecayKnob->setHintText( tr( "Reverb Decay:" ), "" );

	m_reverbSizeKnob = new Knob( knobSmall_17, this );
	m_reverbSizeKnob->move( 45, 95 );
	m_reverbSizeKnob->setHintText( tr( "Reverb Size:" ), "" );

	m_reverbDiffusionKnob = new Knob( knobSmall_17, this );
	m_reverbDiffusionKnob->move( 65, 95 );
	m_reverbDiffusionKnob->setHintText( tr( "Reverb Diffusion:" ), "" );

	m_reverbHighDampKnob = new Knob( knobSmall_17, this );
	m_reverbHighDampKnob->move( 85, 95 );
	m_reverbHighDampKnob->setHintText( tr( "Reverb High-Frequency Damping:" ), "" );

	m_reverbLowDampKnob = new Knob( knobSmall_17, this );
	m_reverbLowDampKnob->move( 105, 95 );
	m_reverbLowDampKnob->setHintText( tr( "Reverb Low-Frequency Damping:" ), "" );

	m_reverbModKnob = new Knob( knobSmall_17, this );
	m_reverbModKnob->move( 125, 95 );
	m_reverbModKnob->setHintText( tr( "Reverb Modulation Amount:" ), "" );

	m_reverbModFreqKnob = new Knob( knobSmall_17, this );
	m_reverbModFreqKnob->move( 145, 95 );
	m_reverbModFreqKnob->setHintText( tr( "Reverb Modulation Frequency:" ), "" );

	m_reverbWidthKnob = new Knob( knobSmall_17, this );
	m_reverbWidthKnob->move( 165, 95 );
	m_reverbWidthKnob->setHintText( tr( "Reverb Stereo Width:" ), "" );

	m_flangerMixKnob = new Knob( knobSmall_17, this );
	m_flangerMixKnob->move( 5, 115 );
	m_flangerMixKnob->setHintText( tr( "Flanger Mix:" ), "" );

	m_flangerDelayKnob = new Knob( knobSmall_17, this );
	m_flangerDelayKnob->move( 25, 115 );
	m_flangerDelayKnob->setHintText( tr( "Flanger Delay:" ), "" );

	m_flangerDepthKnob = new Knob( knobSmall_17, this );
	m_flangerDepthKnob->move( 45, 115 );
	m_flangerDepthKnob->setHintText( tr( "Flanger LFO Depth:" ), "" );

	m_flangerRateKnob = new Knob( knobSmall_17, this );
	m_flangerRateKnob->move( 65, 115 );
	m_flangerRateKnob->setHintText( tr( "Flanger LFO Rate:" ), "" );

	m_flangerFeedbackKnob = new Knob( knobSmall_17, this );
	m_flangerFeedbackKnob->move( 85, 115 );
	m_flangerFeedbackKnob->setHintText( tr( "Flanger Feedback:" ), "" );

	m_flangerDampingKnob = new Knob( knobSmall_17, this );
	m_flangerDampingKnob->move( 105, 115 );
	m_flangerDampingKnob->setHintText( tr( "Flanger Damping:" ), "" );

	m_flangerPhaseKnob = new Knob( knobSmall_17, this );
	m_flangerPhaseKnob->move( 125, 115 );
	m_flangerPhaseKnob->setHintText( tr( "Flanger Phase:" ), "" );

	m_delayMixKnob = new Knob( knobSmall_17, this );
	m_delayMixKnob->move( 5, 135 );
	m_delayMixKnob->setHintText( tr( "Delay Mix:" ), "" );

	m_delayTimeKnob = new Knob( knobSmall_17, this );
	m_delayTimeKnob->move( 25, 135 );
	m_delayTimeKnob->setHintText( tr( "Delay Time:" ), "" );

	m_delayStereoKnob = new Knob( knobSmall_17, this );
	m_delayStereoKnob->move( 45, 135 );
	m_delayStereoKnob->setHintText( tr( "Delay Stereo:" ), "" );

	m_delayHighDampKnob = new Knob( knobSmall_17, this );
	m_delayHighDampKnob->move( 65, 135 );
	m_delayHighDampKnob->setHintText( tr( "Delay High-Frequency Damping:" ), "" );

	m_delayLowDampKnob = new Knob( knobSmall_17, this );
	m_delayLowDampKnob->move( 85, 135 );
	m_delayLowDampKnob->setHintText( tr( "Delay Low-Frequency Damping:" ), "" );

	m_delayFeedbackKnob = new Knob( knobSmall_17, this );
	m_delayFeedbackKnob->move( 105, 135 );
	m_delayFeedbackKnob->setHintText( tr( "Delay Feedback:" ), "" );

	m_exciterLeakKnob = new Knob( knobSmall_17, this );
	m_exciterLeakKnob->move( 200, 220 );
	m_exciterLeakKnob->setHintText( tr( "Exciter Leakage:" ), "" );
}




void MercuryView::modelChanged()
{
	Mercury * b = castModel<Mercury>();

	m_decayKnob->setModel( &b->m_decay );
	m_partialNumKnob->setModel( &b->m_partialNum );
	m_partialFreqMaxKnob->setModel( &b->m_partialFreqMax );
	m_partialStretchKnob->setModel( &b->m_partialStretch );
	m_partialSineKnob->setModel( &b->m_partialSine );
	m_partialLPKnob->setModel( &b->m_partialLP );
	m_partialLPSlopeKnob->setModel( &b->m_partialLPSlope );
	m_partialDampKnob->setModel( &b->m_partialDamp );
	m_fundamentalKnob->setModel( &b->m_fundamental );
	m_partialComb1Knob->setModel( &b->m_partialComb1 );
	m_partialComb2Knob->setModel( &b->m_partialComb2 );
	m_partialCombShiftKnob->setModel( &b->m_partialCombShift );
	m_partialCombBalKnob->setModel( &b->m_partialCombBal );
	m_partialRingKnob->setModel( &b->m_partialRing );
	m_partialRoundKnob->setModel( &b->m_partialRound );
	m_partialRoundMaxKnob->setModel( &b->m_partialRoundMax );
	m_partialFreqBalKnob->setModel( &b->m_partialFreqBal );
	m_exciterLPKnob->setModel( &b->m_exciterLP );
	m_exciterHPKnob->setModel( &b->m_exciterHP );
	m_exciterImpulseKnob->setModel( &b->m_exciterImpulse );
	m_exciterNoiseKnob->setModel( &b->m_exciterNoise );
	m_partialProtectKnob->setModel( &b->m_partialProtect );
	m_partialProtectSlopeKnob->setModel( &b->m_partialProtectSlope );
	m_exciterFeedbackKnob->setModel( &b->m_exciterFeedback );
	m_exciterFeedbackDelayKnob->setModel( &b->m_exciterFeedbackDelay );
	m_modalVolKnob->setModel( &b->m_modalVol );
	m_exciterFeedbackPhaseKnob->setModel( &b->m_exciterFeedbackPhase );
	m_exciterFadeKnob->setModel( &b->m_exciterFade );
	m_exciterDelayFadeKnob->setModel( &b->m_exciterDelayFade );
	m_exciterAttackKnob->setModel( &b->m_exciterAttack );
	m_exciterDecay1Knob->setModel( &b->m_exciterDecay1 );
	m_exciterBreakpointKnob->setModel( &b->m_exciterBreakpoint );
	m_exciterDecay2Knob->setModel( &b->m_exciterDecay2 );
	m_exciterSustainKnob->setModel( &b->m_exciterSustain );
	m_exciterReleaseKnob->setModel( &b->m_exciterRelease );
	m_exciterSaturationKnob->setModel( &b->m_exciterSaturation );
	m_exciterAsymKnob->setModel( &b->m_exciterAsym );
	m_exciterPulseTrainKnob->setModel( &b->m_exciterPulseTrain );
	m_exciterFeedbackLPKnob->setModel( &b->m_exciterFeedbackLP );
	m_reverbMixKnob->setModel( &b->m_reverbMix );
	m_reverbSizeKnob->setModel( &b->m_reverbSize );
	m_reverbDecayKnob->setModel( &b->m_reverbDecay );
	m_reverbDiffusionKnob->setModel( &b->m_reverbDiffusion );
	m_reverbHighDampKnob->setModel( &b->m_reverbHighDamp );
	m_reverbLowDampKnob->setModel( &b->m_reverbLowDamp );
	m_reverbModKnob->setModel( &b->m_reverbMod );
	m_reverbModFreqKnob->setModel( &b->m_reverbModFreq );
	m_reverbWidthKnob->setModel( &b->m_reverbWidth );
	m_flangerMixKnob->setModel( &b->m_flangerMix );
	m_flangerDelayKnob->setModel( &b->m_flangerDelay );
	m_flangerDepthKnob->setModel( &b->m_flangerDepth );
	m_flangerRateKnob->setModel( &b->m_flangerRate );
	m_flangerFeedbackKnob->setModel( &b->m_flangerFeedback );
	m_flangerDampingKnob->setModel( &b->m_flangerDamping );
	m_flangerPhaseKnob->setModel( &b->m_flangerPhase );
	m_delayMixKnob->setModel( &b->m_delayMix );
	m_delayTimeKnob->setModel( &b->m_delayTime );
	m_delayStereoKnob->setModel( &b->m_delayStereo );
	m_delayHighDampKnob->setModel( &b->m_delayHighDamp );
	m_delayLowDampKnob->setModel( &b->m_delayLowDamp );
	m_delayFeedbackKnob->setModel( &b->m_delayFeedback );
	m_exciterLeakKnob->setModel( &b->m_exciterLeak );
}

}



extern "C"
{

// necessary for getting instance out of shared lib
PLUGIN_EXPORT Plugin * lmms_plugin_main(Model *m, void *)
{
	return new Mercury(static_cast<InstrumentTrack *>(m));
}


}





}

