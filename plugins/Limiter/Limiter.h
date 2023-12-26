/*
 * Limiter.h
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

#ifndef LIMITER_H
#define LIMITER_H

#include "Effect.h"
#include "LimiterControls.h"

#include "lmms_math.h"
#include "OversamplingHelpers.h"
#include "SignalsmithEnvelopes.h"

#ifdef __SSE2__
#include <emmintrin.h>
#endif

namespace lmms
{

class IntersamplePeakFinder {
public:
	IntersamplePeakFinder()
	{
		m_buffer.resize(12, 0.f);
	}

#ifdef __SSE2__
	// With this conveniently being a 4-stage FIR filter, this is a perfect
	// opportunity to use SSE2 instructions to process all four filters
	// simultaneously within a single thread.
	void processSamples(float input, float* outputs)
    {
        for (int i = 11; i > 0; --i) { m_buffer[i] = m_buffer[i - 1]; }
        m_buffer[0] = input;

        __m128 results = _mm_setzero_ps(); // Initialize a 4-element vector with zeros
        for (int i = 0; i < 12; ++i)
        {
            __m128 bufferElement = _mm_set1_ps(m_buffer[i]); // Broadcast buffer[i] to a 4-element vector
            __m128 coefVector = _mm_loadu_ps(m_coefs[i]); // Load coefficients into a vector
            // Multiply coefficients with buffer element and add to results
            results = _mm_add_ps(results, _mm_mul_ps(coefVector, bufferElement));
        }

        _mm_storeu_ps(outputs, results);
    }
#else
	void processSamples(float input, float* outputs)
	{
		for (int i = 11; i > 0; --i) { m_buffer[i] = m_buffer[i - 1]; }
		m_buffer[0] = input;
		
		for (int i = 0; i < 12; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				outputs[j] += m_coefs[i][j] * m_buffer[i];
			}
		}
	}
#endif

	float processPeak(float input)
	{
		float outputs[4] = {0, 0, 0, 0};
		processSamples(input, outputs);
		// The upsampling coefficients aren't centered on an integer sample,
		// so we need to take the maximum of both the upsampled and non-upsampled values.
		return std::max({std::abs(outputs[0]), std::abs(outputs[1]), std::abs(outputs[2]), std::abs(outputs[3]), std::abs(m_buffer[6])});
	}

private:
	// This needs to be aligned to a 16-byte boundary for SSE2 instructions to work properly.
	alignas(16) std::vector<float> m_buffer;
	
	// Coefficients from the official ITU-R BS.1770-4 standard (page 17):
	// https://www.itu.int/dms_pubrec/itu-r/rec/bs/R-REC-BS.1770-4-201510-I!!PDF-E.pdf
	// This is a 4-phase FIR filter for 4x upsampling.  Each filter provides one of the
	// four samples resulting from the upsampling.  Since the goal is simply to filter
	// at a fixed percentage of the total bandwidth, these coefficients work equally
	// well at all sample rates.
	// While the standard generally recommends upsampling to a minimum of 192 kHz
	// from the 48 kHz standard, upsampling to 176.4 kHz from LMMS's default
	// 44.1 kHz doesn't seem to be noticeably inferior in the resulting measurements.
	alignas(16) float m_coefs[12][4] = {{ 0.0017089843750f, -0.0291748046875f, -0.0189208984375f, -0.0083007812500f },
									  { 0.0109863281250f,  0.0292968750000f,  0.0330810546875f,  0.0148925781250f },
									  {-0.0196533203125f, -0.0517578125000f, -0.0582275390625f, -0.0266113281250f },
									  { 0.0332031250000f,  0.0891113281250f,  0.1015625000000f,  0.0476074218750f },
									  {-0.0594482421875f, -0.1665039062500f, -0.2003173828125f, -0.1022949218750f },
									  { 0.1373291015625f,  0.4650878906250f,  0.7797851562500f,  0.9721679687500f },
									  { 0.9721679687500f,  0.7797851562500f,  0.4650878906250f,  0.1373291015625f },
									  {-0.1022949218750f, -0.2003173828125f, -0.1665039062500f, -0.0594482421875f },
									  { 0.0476074218750f,  0.1015625000000f,  0.0891113281250f,  0.0332031250000f },
									  {-0.0266113281250f, -0.0582275390625f, -0.0517578125000f, -0.0196533203125f },
									  { 0.0148925781250f,  0.0330810546875f,  0.0292968750000f,  0.0109863281250f },
									  {-0.0083007812500f, -0.0189208984375f, -0.0291748046875f,  0.0017089843750f }};
};

// A version of the ITU-R BS.1770-4 K-weighting filter that
// can run at all sample rates rather than just 48 kHz.
class KWeight {
public:
    KWeight(float fs) : fs_(fs)
    {
        updateCoefficients();
    }

    inline float process(float input)
    {
        float hsf = pb0 * input + pb1 * x1 + pb2 * x2 - pa1 * y1 - pa2 * y2;
        x2 = x1; x1 = input; y2 = y1; y1 = hsf;

        float hpf = rb0 * hsf + rb1 * w1 + rb2 * w2 - ra1 * z1 - ra2 * z2;
        w2 = w1; w1 = hsf; z2 = z1; z1 = hpf;

        return hpf;
    }

    void setSampleRate(float fs)
    {
        fs_ = fs;
        updateCoefficients();
    }

private:
    float fs_;
    float pb0, pb1, pb2, pa1, pa2, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    float rb0, rb1, rb2, ra1, ra2, w1 = 0, w2 = 0, z1 = 0, z2 = 0;

    void updateCoefficients()
    {
        // High shelf
        float db = 3.999843853973347f;
        float f0_hsf = 1681.974450955533f;
        float Q_hsf = 0.7071752369554196f;
        float K_hsf = tan(M_PI * f0_hsf / fs_);
        float Vh = std::pow(10.f, db / 20.f);
        float Vb = std::pow(Vh, 0.4996667741545416f);
        float a0_hsf = 1.f + K_hsf / Q_hsf + K_hsf * K_hsf;
        pb0 = (Vh + Vb * K_hsf / Q_hsf + K_hsf * K_hsf) / a0_hsf;
        pb1 = 2.f * (K_hsf * K_hsf - Vh) / a0_hsf;
        pb2 = (Vh - Vb * K_hsf / Q_hsf + K_hsf * K_hsf) / a0_hsf;
        pa1 = 2.f * (K_hsf * K_hsf - 1.f) / a0_hsf;
        pa2 = (1.f - K_hsf / Q_hsf + K_hsf * K_hsf) / a0_hsf;

        // Highpass
        float f0_hpf = 38.13547087602444f;
        float Q_hpf = 0.5003270373238773f;
        float K_hpf = tan(M_PI * f0_hpf / fs_);
        float a0_hpf = 1.f + K_hpf / Q_hpf + K_hpf * K_hpf;
        rb0 = 1.f;
        rb1 = -2.f;
        rb2 = 1.f;
        ra1 = 2.f * (K_hpf * K_hpf - 1.f) / a0_hpf;
        ra2 = (1.f - K_hpf / Q_hpf + K_hpf * K_hpf) / a0_hpf;
    }
};



class LufsProcessor {
public:
	LufsProcessor(float sampleRate) :
		m_sampleRate(sampleRate),
		m_kWeight{m_sampleRate, m_sampleRate},
		m_integratedVal(0),
		m_bufferSize(1),
		m_bufIndex(0),
		m_momentaryBox(1),
		m_momentaryBoxResult(0),
		m_absoluteSum(0),
		m_shortTermBox(1),
		m_shortTermBoxResult(0)
	{
		setSampleRate(m_sampleRate);
	}
	
	void reset()
	{
		m_bufIndex = 0;
		m_momentaryBox.resize(std::round(m_sampleRate * 0.4f));
		m_shortTermBox.resize(std::round(m_sampleRate * 3.f));
		m_integratedVal = 0;
		m_momentary.clear();
		m_momentary.push_back(-999.f);
		m_absoluteSum = 0;
		m_shortTerm.clear();
		m_shortTerm.push_back(-999.f);
	}
	
	void process(float in1, float in2)
	{
		float kResults[2] = {m_kWeight[0].process(in1), m_kWeight[1].process(in2)};
		float newSquare = kResults[0] * kResults[0] + kResults[1] * kResults[1];
		m_momentaryBoxResult = m_momentaryBox(newSquare);
		m_shortTermBoxResult = m_shortTermBox(newSquare);
		
		if (++m_bufIndex >= m_bufferSize)
		{
			m_bufIndex = 0;
			
			float momentaryTemp = toLUsafe(m_momentaryBoxResult);
			if (momentaryTemp >= -70) { m_momentary.push_back(momentaryTemp); m_absoluteSum += momentaryTemp; }// -70 LKFS absolute threshold 
			float shortTermTemp = toLUsafe(m_shortTermBoxResult);
			if (shortTermTemp >= -70) { m_shortTerm.push_back(shortTermTemp); }
			
			calcIntegrated();
		}
	}
	
	void calcIntegrated()
	{
		float absoluteAvg = m_absoluteSum / static_cast<float>(m_momentary.size());
		
		float relativeAvg = 0;
		int relativeCount = 0;
		for (float val : m_momentary)
		{
			if (val >= absoluteAvg - 10)// -10 dB relative threshold
			{
				relativeAvg += val;
				++relativeCount;
			}
		}
		
		relativeAvg /= (float)relativeCount;
		
		m_integratedVal = relativeAvg;
	}
	
	inline float toLU(float in) {return -0.691f + 10.f * std::log10(in);}
	inline float toLUsafe(float in) {return in > 0 ? -0.691f + 10.f * std::log10(in) : -999.f;}
	
	float getMomentary() { return toLUsafe(m_momentaryBoxResult); }
	float getShortTerm() { return toLUsafe(m_shortTermBoxResult); }
	float getIntegrated() { return m_integratedVal; }
	
	void setSampleRate(float sampleRate)
	{
		m_sampleRate = sampleRate;
		m_bufferSize = std::round(sampleRate * 0.1f);
		m_kWeight[0].setSampleRate(m_sampleRate);
		m_kWeight[1].setSampleRate(m_sampleRate);
		reset();
	}

private:
	float m_sampleRate;
	
	std::array<KWeight, 2> m_kWeight;
	float m_integratedVal;

	int m_bufferSize;
	int m_bufIndex;
	std::vector<float> m_buffer;
	signalsmith::envelopes::BoxFilter<float> m_momentaryBox;
	float m_momentaryBoxResult;
	float m_absoluteSum;
	signalsmith::envelopes::BoxFilter<float> m_shortTermBox;
	float m_shortTermBoxResult;
	
	std::vector<float> m_results;
	std::vector<float> m_momentary;
	std::vector<float> m_shortTerm;
};


// For collecting and calculating the proper information for the display
// in a thread-safe (and lockless) manner.
// The audio thread will provide and process the data while the display
// thread will track which info it's actually processed so far.
class DisplayHandler {
public:
	DisplayHandler(float sampleRate, int pointsPerSecond) :
		m_sampleRate(sampleRate)
	{
		setPointsPerSecond(pointsPerSecond);
	}
	
	inline void setPointsPerSecond(int pointsPerSecond)
	{
		m_pointsPerSecond = pointsPerSecond;
		m_timerGoal = m_sampleRate / pointsPerSecond;
	}
	
	inline void setSampleRate(int sampleRate)
	{
		m_sampleRate = sampleRate;
		setPointsPerSecond(m_pointsPerSecond);
	}
	
	inline void process(float inPeakL, float inPeakR, float outPeakL, float outPeakR, float gainPeakL, float gainPeakR)
	{
		m_inPeakLVal = std::max(m_inPeakLVal, inPeakL);
		m_inPeakRVal = std::max(m_inPeakRVal, inPeakR);
		m_outPeakLVal = std::max(m_outPeakLVal, outPeakL);
		m_outPeakRVal = std::max(m_outPeakRVal, outPeakR);
		m_gainPeakLVal = std::min(m_gainPeakLVal, gainPeakL);
		m_gainPeakRVal = std::min(m_gainPeakRVal, gainPeakR);

		if (++m_timer >= m_timerGoal)
		{
			m_timer = 0;
			
			m_inPeakLBuf[m_index] = m_inPeakLVal;
			m_inPeakLVal = 0;
			m_inPeakRBuf[m_index] = m_inPeakRVal;
			m_inPeakRVal = 0;
			m_outPeakLBuf[m_index] = m_outPeakLVal;
			m_outPeakLVal = 0;
			m_outPeakRBuf[m_index] = m_outPeakRVal;
			m_outPeakRVal = 0;
			m_gainPeakLBuf[m_index] = m_gainPeakLVal;
			m_gainPeakLVal = 1;
			m_gainPeakRBuf[m_index] = m_gainPeakRVal;
			m_gainPeakRVal = 1;
			
			// We don't increment the index variable until the end to ensure the display
			// thread doesn't try accessing the new data too early
			m_index = (m_index + 1) % LIMITER_DISPLAY_BUFSIZE;
		}
	}
private:
	int m_sampleRate;
	
	float m_inPeakLVal = 0;
	float m_inPeakRVal = 0;
	float m_outPeakLVal = 0;
	float m_outPeakRVal = 0;
	float m_gainPeakLVal = 0;
	float m_gainPeakRVal = 0;
	float m_inPeakLBuf[LIMITER_DISPLAY_BUFSIZE] = {};
	float m_inPeakRBuf[LIMITER_DISPLAY_BUFSIZE] = {};
	float m_outPeakLBuf[LIMITER_DISPLAY_BUFSIZE] = {};
	float m_outPeakRBuf[LIMITER_DISPLAY_BUFSIZE] = {};
	float m_gainPeakLBuf[LIMITER_DISPLAY_BUFSIZE] = {};
	float m_gainPeakRBuf[LIMITER_DISPLAY_BUFSIZE] = {};
	int m_index = 0;
	
	int m_pointsPerSecond = 0;
	int m_timer = 0;
	int m_timerGoal = 0;
	
	// making getter functions is against my religion
	friend class gui::LimiterControlDialog;
};


class LimiterEffect : public Effect
{
	Q_OBJECT
public:
	LimiterEffect(Model* parent, const Descriptor::SubPluginFeatures::Key* key);
	~LimiterEffect() override = default;
	bool processAudioBuffer(sampleFrame* buf, const fpp_t frames) override;

	EffectControls* controls() override
	{
		return &m_limiterControls;
	}
	
	// Apply multiple release stages.  Each successive iteration of this
	// smooths out one more derivative, making the result cleaner and more "S" shaped.
	inline void applyRelease(float boundPeak, float releaseVal, int releaseStages, std::array<float, LIMITER_MAX_RELEASE_STAGES>& yL)
	{
		if (boundPeak <= yL[0]) { yL[0] = boundPeak; }
		else { yL[0] = std::min(yL[0] * releaseVal + (1 - releaseVal) * boundPeak, boundPeak); }
		yL[0] = std::max(yL[0], LIMITER_MIN_FLOOR);
		for (int i = 1; i < releaseStages; ++i)
		{
			if (yL[i-1] <= yL[i]) { yL[i] = yL[i-1]; }
			else { yL[i] = std::min(yL[i] * releaseVal + (1 - releaseVal) * yL[i-1], yL[i-1]); }
			yL[i] = std::max(yL[i], LIMITER_MIN_FLOOR);
		}
	}
	
	void changeSmooth();
	
	inline float getMomentary() {return m_lufsProcessor.getMomentary();}
	inline float getShortTerm() {return m_lufsProcessor.getShortTerm();}
	inline float getIntegrated() {return m_lufsProcessor.getIntegrated();}
	inline float getReleaseDivide(int channel) {return m_releaseDivide[channel];}
	inline float getLinkAmount() {return m_linkAmount;}
	inline float getCrestFactor(int channel) {return m_crestFactor[channel];}
	inline float getDcOffset(int channel) {return m_dcVal[channel];}
	
	inline DisplayHandler* displayHandler() {return &m_displayHandler;}

private slots:
	void changeSampleRate();

private:
	LimiterControls m_limiterControls;
	
	float m_sampleRate;
	
	LufsProcessor m_lufsProcessor;
	DisplayHandler m_displayHandler;
	
	// Signalsmith's gems.
	std::array<signalsmith::envelopes::PeakHold<float>, 2> m_peakHold;
	std::array<signalsmith::envelopes::BoxStackFilter<float>, 2> m_boxStackFilter;
	std::array<signalsmith::envelopes::PeakHold<float>, 2> m_peakHold2;
	std::array<signalsmith::envelopes::BoxStackFilter<float>, 2> m_boxStackFilter2;
	
	std::array<std::vector<float>, 2> m_inLookBuf;
	std::array<std::vector<float>, 2> m_scLookBuf;
	int m_lookWrite;
	int m_lookBufLength;
	std::vector<float> m_inLookBuf2[2];
	std::vector<float> m_scLookBuf2[2];
	int m_lookWrite2;
	int m_lookBufLength2;
	
	std::array<std::array<float, LIMITER_MAX_RELEASE_STAGES>, 2> m_yL = {{}};
	std::array<std::array<float, LIMITER_MAX_RELEASE_STAGES>, 2> m_yL2 = {{}};
	
	int m_oldLookahead = -1;
	int m_oldHold = -1;
	int m_oldOversampling = -1;
	int m_oldSmooth = -1;
	
	std::array<float, 2> m_dcVal = {0, 0};
	
	std::array<float, 2> m_crestPeak = {1, 1};
	std::array<float, 2> m_crestMean = {1, 1};
	float m_crestMeanCoeff = 0;
	
	float m_coeffPrecalc;
	float m_coeffPrecalcOversamp;
	
	IntersamplePeakFinder m_ispFinder[2];
	
	std::array<float, 2> m_holdTimer = {0, 0};
	
	std::array<Upsampler, 2> m_upsampler;
	std::array<Downsampler, 2> m_downsampler;
	
	int m_oversampleMultMax = -1;
	
	float m_releaseDivide[2] = {1, 1};
	float m_linkAmount = 0;
	float m_crestFactor[2] = {0, 0};
	
	std::array<KWeight, 2> m_kWeightForCrest;
	
	friend class LimiterControls;
};

} // namespace lmms

#endif
