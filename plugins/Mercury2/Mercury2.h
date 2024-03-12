/*
 * Mercury2.h
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


#ifndef MERCURY2_H
#define MERCURY2_H

#include "AutomatableModel.h"
#include "BasicFilters.h"
#include "Instrument.h"
#include "InstrumentPlayHandle.h"
#include "InstrumentTrack.h"
#include "InstrumentView.h"
#include "interpolation.h"
#include "Graph.h"
#include "GuiApplication.h"
#include "MainWindow.h"
#include <QPainter>
#include "qmath.h"
#include <xmmintrin.h>
#include <random>

namespace lmms
{

constexpr int PARTIAL_MAX = 256;
constexpr float PARTIAL_FREQ_TRUE_MAX = 16000;

constexpr int IMPULSE_MAX = 10;

constexpr int MERCURY_WINDOWSIZE_X = 450;
constexpr int MERCURY_WINDOWSIZE_Y = 250;

constexpr int MERCURY_PARTIALVIEWSIZE_X = 420;
constexpr int MERCURY_PARTIALVIEWSIZE_Y = 100;
constexpr int MERCURY_PARTIALVIEWBORDER = 2;
constexpr int MERCURY_PARTIALVIEWLENGTH = MERCURY_PARTIALVIEWSIZE_X - MERCURY_PARTIALVIEWBORDER * 2;
constexpr int MERCURY_PARTIALVIEWHEIGHT = MERCURY_PARTIALVIEWSIZE_Y - MERCURY_PARTIALVIEWBORDER * 2;

namespace gui
{
class Mercury2View;
class Knob;
class LedCheckBox;
class PixmapButton;
}


class ColoredNoiseGenerator {
public:
    ColoredNoiseGenerator(float H) : m_H(H), m_pos(0) {
        m_buffer = new float[256];
        generateNoise();
    }

    ~ColoredNoiseGenerator() {
        delete[] m_buffer;
    }

    float nextSample() {
        float sample = m_buffer[m_pos++];
        if (m_pos >= 256) {
            generateNoise();
            m_pos = 0;
        }
        return sample;
    }
    
    inline void setDimension(float H)
    {
		m_H = H;
    }

private:
    float m_H;
    float* m_buffer;
    int m_pos;

    void generateNoise() {
        fractal(m_buffer, 256, m_H);
    }

    void fractal(float* v, int N, float H) {
        int l = N;
        int k;
        float r = 2.0f * H*H + 0.3f;
        int c;

        v[0] = 0;
        while (l > 1) {
            k = N / l;
            for (c = 0; c < k; c++) {
                v[c*l + l/2] = (v[c*l] + v[((c+1) * l) % N]) / 2.0f +
                    2.0f * r * (rand() - (float)RAND_MAX/2.0f) / (float)RAND_MAX;
                v[c*l + l/2] = fmin(fmax(v[c*l + l/2], -1.0f), 1.0f);
            }
            l /= 2;
            r /= powf(2, H);
        }
    }
};




class MSynth
{
public:
	MSynth( NotePlayHandle * nph, float sampleRate, int chordScatterDelay, float impulseLP, float impulseLPFalloff, float partialRand );
	virtual ~MSynth();
	
	void nextNoteBuffer(sampleFrame * working_buffer, NotePlayHandle * currentNote, float decay, int partialNum, float partialFreqMax, float partialStretch, float partialSine, float partialFlatOdd, float partialInharmEnabled, float partialInharmConst, float partialInharmTrack, float partialInharmLows, bool partialInharmRetune, float partialDamp, float partialDampSlope, float partialFreqDamp, float partialFreqDampSlope, float partialRounding, float partialRoundingDivide, float partialRoundingCutoff, float partialComb1, float partialComb2, float partialCombShift, float partialCombBal, float partialCombInvert, float partialHarm2, float partialHarm3, float partialHarm5, float partialHarm7, float partialHarm11, float partialLP, float partialLPSlope, float partialRand, float playPosition, float playPositionRand, float brightness, float impulseNum, float impulseSpacing, float impulseLP, float impulseLPFalloff, float impulsePeriodicity, float impulseVol, float noiseVol, float noiseLP, float noiseHP, float noiseSat, float noisePeriodVol, float noisePeriodSpread, float noisePeriodShift, float noisePeriodDamp, float noiseNormalize, float exciterAttack, float exciterDecay1, float exciterBreakpoint, float exciterDecay2, float exciterSustain, float exciterRelease, float volumeAttack, float volumeDecay1, float volumeBreakpoint, float volumeDecay2, float volumeSustain, float volumeRelease, float tremoloAmount, float tremoloRate, float vibratoAmount, float vibratoRate, float vibratoShape, float vibratoTremolo, float vibratoFade, float vibratoFadeShape, float vibratoLengthMin, float erodeSelf, float erodeLP, float erodeHP, float erodeSine, float erodeFreq, float erodeEnable, float exciterFade, float release, float chordScatter, float chordScatterUniformity);
	
	void calcResonator(float input, float &output, int partialMax);
	
	void updatePartials(float noteFrequency, float decay, float partialStretch, float partialSine, float partialFlatOdd, float partialInharmEnabled, float partialInharmConst, float partialInharmTrack, float partialInharmLows, bool partialInharmRetune, float partialDamp, float partialDampSlope, float partialFreqDamp, float partialFreqDampSlope, float partialRounding, float partialRoundingDivide, float partialRoundingCutoff, float partialComb1, float partialComb2, float partialCombShift, float partialCombBal, float partialCombInvert, float partialHarm2, float partialHarm3, float partialHarm5, float partialHarm7, float partialHarm11, float partialLP, float partialLPSlope, float partialRand, float playPosition, float playPositionRand, float brightness, float partialFreqMax, int partialNum);
	void updateComb(int partialNum, float partialFreqMax, float partialComb1, float partialComb2, float partialCombShift, float partialCombBal, float partialCombInvert);
	
	void exciterFilter(int filter_num, float input, float &output);
	void setExciterFilterCutoff(int filter_num, float cutoff_frequency);
	
	void runNoiseLP(int index, float input, float &output);
	void setNoiseLP(int index, float cutoffFrequency);
	void runNoiseHP(int index, float input, float &output);
	void setNoiseHP(int index, float cutoffFrequency);
	
	inline float realmod(int k, int n)
	{
		return ((k %= n) < 0) ? k+n : k;
	}
	
	inline float calcEnvelope(float x, float a, float d1, float b, float d2, float s, float r, float &gainStorage)
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

	inline float expCurve(float x, float a, float b)
	{
		return (-powf(x, 1.f / F_E) + 1.f) * (a - b) + b;
	}
	
	inline float msToCoeff(float ms)
	{
		// Convert time in milliseconds to applicable lowpass coefficient
		return exp(m_coeffPrecalc / ms);
	}
	
	inline float fastSigmoid(float input, float slope)
	{
		return (input / (slope + std::abs(input))) * (slope + 1);
	}
	
	float calcInharmConstant(float partialInharmConst, float partialInharmTrack, float partialInharmLows, float freq);
	float calcInharmResult(float partialInharmConst, float partialInharmTrack, float partialInharmLows, float freq, float inharmConstant);

private:
	int m_sample_index;
	float m_sample_realindex;
	NotePlayHandle* m_nph;
	float m_sampleRate;
	
	int m_noteDuration;
	int m_noteReleaseTime;
	
	float m_noteFreqChange;
	float m_vibratoPhase;
	
	int m_impulseCount;
	int m_impulseNext;
	
	float m_partialDelay1[PARTIAL_MAX] __attribute__((aligned(16))) = {0};
	float m_partialDelay2[PARTIAL_MAX] __attribute__((aligned(16))) = {0};
	
	// For performance
	float m_partialSinW[PARTIAL_MAX] __attribute__((aligned(16))) = {0};
	float m_partialCosW[PARTIAL_MAX] __attribute__((aligned(16))) = {0};
	float m_partialSinWFeed[PARTIAL_MAX] __attribute__((aligned(16))) = {0};
	float m_partialCosWFeed[PARTIAL_MAX] __attribute__((aligned(16))) = {0};
	
	float m_partialFreq[PARTIAL_MAX] = {0};
	float m_partialGain[PARTIAL_MAX] = {0};
	float m_partialRadius[PARTIAL_MAX] = {0};
	
	float m_comb1GainPre[PARTIAL_MAX] = {0};
	float m_comb2GainPre[PARTIAL_MAX] = {0};
	
	float m_exciterFiltA[IMPULSE_MAX][3] = {{0}};
	float m_exciterFiltB[IMPULSE_MAX][3] = {{0}};
	float m_exciterFiltX[IMPULSE_MAX][2] = {{0}};
	float m_exciterFiltY[IMPULSE_MAX][2] = {{0}};
	
	float m_noiseLPA[2][3] = {{0}};
	float m_noiseLPB[2][3] = {{0}};
	float m_noiseLPX[2][2] = {{0}};
	float m_noiseLPY[2][2] = {{0}};
	float m_noiseHPA[2][3] = {{0}};
	float m_noiseHPB[2][3] = {{0}};
	float m_noiseHPX[2][2] = {{0}};
	float m_noiseHPY[2][2] = {{0}};
	
	int m_chordScatterDelay;
	
	float m_playPositionRandValue;
	
	ColoredNoiseGenerator m_fractalNoise;
	
	float m_envExciterGainStorage;
	float m_envVolumeGainStorage;
	
	float m_coeffPrecalc;
	
	std::vector<float> m_noiseDelayBuf;
	int m_noiseDelayWrite;
	int m_noiseDelayBufSize;
	
	float m_currentFreqInterp;
	float m_currentFreqInterpCoef;
	
	float m_randFreqMult[256] __attribute__((aligned(16))) = {};
	
	std::vector<float> m_erodeBuf;
	int m_erodeWrite = 0;
	LinkwitzRiley<1> m_erodeLP;
	LinkwitzRiley<1> m_erodeHP;
	float m_erodeSinePhase = 0;
	
	int m_lastEnabled = 256;
	
	friend class gui::Mercury2View;
} ;

class Mercury2 : public Instrument
{
	Q_OBJECT
public:
	Mercury2(InstrumentTrack * _instrument_track );
	~Mercury2() override = default;

	/*void playNote( NotePlayHandle * _n,
						sampleFrame * _working_buffer ) override;*/
	void play( sampleFrame * _working_buffer ) override;
	
	void deleteNotePluginData( NotePlayHandle * _n ) override;


	void saveSettings( QDomDocument & _doc,
							QDomElement & _parent ) override;
	void loadSettings( const QDomElement & _this ) override;

	QString nodeName() const override;

	virtual f_cnt_t desiredReleaseFrames() const;

	gui::PluginView * instantiateView( QWidget * _parent ) override;
	
	void updatePartials(MSynth * mSynthInstance, float noteFrequency);
	void updateComb(MSynth * mSynthInstance);

private slots:
	void updatePartials();

private:
	gui::Mercury2View * m_mercury2View;

	FloatModel m_volume;
	FloatModel m_decay;
	FloatModel m_partialNum;
	FloatModel m_partialFreqMax;
	FloatModel m_partialStretch;
	FloatModel m_partialSine;
	FloatModel m_partialFlatOdd;
	FloatModel m_partialInharmEnabled;
	FloatModel m_partialInharmConst;
	FloatModel m_partialInharmTrack;
	FloatModel m_partialInharmLows;
	FloatModel m_partialInharmRetune;
	FloatModel m_partialDamp;
	FloatModel m_partialDampSlope;
	FloatModel m_partialFreqDamp;
	FloatModel m_partialFreqDampSlope;
	FloatModel m_partialRounding;
	FloatModel m_partialRoundingDivide;
	FloatModel m_partialRoundingCutoff;
	FloatModel m_partialComb1;
	FloatModel m_partialComb2;
	FloatModel m_partialCombShift;
	FloatModel m_partialCombBal;
	FloatModel m_partialCombInvert;
	FloatModel m_partialHarm2;
	FloatModel m_partialHarm3;
	FloatModel m_partialHarm5;
	FloatModel m_partialHarm7;
	FloatModel m_partialHarm11;
	FloatModel m_partialLP;
	FloatModel m_partialLPSlope;
	FloatModel m_partialRand;
	FloatModel m_playPosition;
	FloatModel m_playPositionRand;
	FloatModel m_brightness;
	FloatModel m_impulseNum;
	FloatModel m_impulseSpacing;
	FloatModel m_impulseLP;
	FloatModel m_impulseLPFalloff;
	FloatModel m_impulsePeriodicity;
	FloatModel m_impulseVol;
	FloatModel m_noiseVol;
	FloatModel m_noiseLP;
	FloatModel m_noiseHP;
	FloatModel m_noiseSat;
	FloatModel m_noisePeriodVol;
	FloatModel m_noisePeriodSpread;
	FloatModel m_noisePeriodShift;
	FloatModel m_noisePeriodDamp;
	FloatModel m_noiseNormalize;
	FloatModel m_wavePushUp;
	FloatModel m_wavePushDown;
	FloatModel m_waveGrip;
	FloatModel m_waveCatchRand;
	FloatModel m_waveOutVol;
	FloatModel m_exciterAttack;
	FloatModel m_exciterDecay1;
	FloatModel m_exciterBreakpoint;
	FloatModel m_exciterDecay2;
	FloatModel m_exciterSustain;
	FloatModel m_exciterRelease;
	FloatModel m_volumeAttack;
	FloatModel m_volumeDecay1;
	FloatModel m_volumeBreakpoint;
	FloatModel m_volumeDecay2;
	FloatModel m_volumeSustain;
	FloatModel m_volumeRelease;
	FloatModel m_tremoloAmount;
	FloatModel m_tremoloRate;
	FloatModel m_vibratoAmount;
	FloatModel m_vibratoRate;
	FloatModel m_vibratoShape;
	FloatModel m_vibratoTremolo;
	FloatModel m_vibratoFade;
	FloatModel m_vibratoFadeShape;
	FloatModel m_vibratoLengthMin;
	FloatModel m_erodeSelf;
	FloatModel m_erodeLP;
	FloatModel m_erodeHP;
	FloatModel m_erodeSine;
	FloatModel m_erodeFreq;
	FloatModel m_erodeEnable;
	FloatModel m_exciterFade;
	FloatModel m_release;
	FloatModel m_chordScatter;
	FloatModel m_chordScatterUniformity;
	
	float m_tremoloCurrentPhase;
	
	float m_lastNoteFreq;
	
	friend class gui::Mercury2View;
} ;


namespace gui
{

class Mercury2View : public InstrumentViewFixedSize
{
	Q_OBJECT
public:
	Mercury2View( Instrument * _instrument,
					QWidget * _parent );

	~Mercury2View() override = default;
	
	QSize sizeHint() const override { return QSize(MERCURY_WINDOWSIZE_X, MERCURY_WINDOWSIZE_Y); }
	
	void drawPartialPixmap();

public slots:
	void updateDisplay();
	void updatePartialPixmap() {m_updatePartialPixmap = true;}
	
protected slots:
	void paintEvent(QPaintEvent *event) override;

private:
	Mercury2 * m_m;
	MSynth * m_dummyNote;
	
	bool m_updatePartialPixmap;

	Knob * m_volumeKnob;
	Knob * m_decayKnob;
	Knob * m_partialNumKnob;
	Knob * m_partialFreqMaxKnob;
	Knob * m_partialStretchKnob;
	Knob * m_partialSineKnob;
	Knob * m_partialFlatOddKnob;
	Knob * m_partialInharmEnabledKnob;
	Knob * m_partialInharmConstKnob;
	Knob * m_partialInharmTrackKnob;
	Knob * m_partialInharmLowsKnob;
	Knob * m_partialInharmRetuneKnob;
	Knob * m_partialDampKnob;
	Knob * m_partialDampSlopeKnob;
	Knob * m_partialFreqDampKnob;
	Knob * m_partialFreqDampSlopeKnob;
	Knob * m_partialRoundingKnob;
	Knob * m_partialRoundingDivideKnob;
	Knob * m_partialRoundingCutoffKnob;
	Knob * m_partialComb1Knob;
	Knob * m_partialComb2Knob;
	Knob * m_partialCombShiftKnob;
	Knob * m_partialCombBalKnob;
	Knob * m_partialCombInvertKnob;
	Knob * m_partialHarm2Knob;
	Knob * m_partialHarm3Knob;
	Knob * m_partialHarm5Knob;
	Knob * m_partialHarm7Knob;
	Knob * m_partialHarm11Knob;
	Knob * m_partialLPKnob;
	Knob * m_partialLPSlopeKnob;
	Knob * m_partialRandKnob;
	Knob * m_playPositionKnob;
	Knob * m_playPositionRandKnob;
	Knob * m_brightnessKnob;
	Knob * m_impulseNumKnob;
	Knob * m_impulseSpacingKnob;
	Knob * m_impulseLPKnob;
	Knob * m_impulseLPFalloffKnob;
	Knob * m_impulsePeriodicityKnob;
	Knob * m_impulseVolKnob;
	Knob * m_noiseVolKnob;
	Knob * m_noiseLPKnob;
	Knob * m_noiseHPKnob;
	Knob * m_noiseSatKnob;
	Knob * m_noisePeriodVolKnob;
	Knob * m_noisePeriodSpreadKnob;
	Knob * m_noisePeriodShiftKnob;
	Knob * m_noisePeriodDampKnob;
	Knob * m_noiseNormalizeKnob;
	Knob * m_wavePushUpKnob;
	Knob * m_wavePushDownKnob;
	Knob * m_waveGripKnob;
	Knob * m_waveCatchRandKnob;
	Knob * m_waveOutVolKnob;
	Knob * m_exciterAttackKnob;
	Knob * m_exciterDecay1Knob;
	Knob * m_exciterBreakpointKnob;
	Knob * m_exciterDecay2Knob;
	Knob * m_exciterSustainKnob;
	Knob * m_exciterReleaseKnob;
	Knob * m_volumeAttackKnob;
	Knob * m_volumeDecay1Knob;
	Knob * m_volumeBreakpointKnob;
	Knob * m_volumeDecay2Knob;
	Knob * m_volumeSustainKnob;
	Knob * m_volumeReleaseKnob;
	Knob * m_tremoloAmountKnob;
	Knob * m_tremoloRateKnob;
	Knob * m_vibratoAmountKnob;
	Knob * m_vibratoRateKnob;
	Knob * m_vibratoShapeKnob;
	Knob * m_vibratoTremoloKnob;
	Knob * m_vibratoFadeKnob;
	Knob * m_vibratoFadeShapeKnob;
	Knob * m_vibratoLengthMinKnob;
	Knob * m_erodeSelfKnob;
	Knob * m_erodeLPKnob;
	Knob * m_erodeHPKnob;
	Knob * m_erodeSineKnob;
	Knob * m_erodeFreqKnob;
	Knob * m_erodeEnableKnob;
	Knob * m_exciterFadeKnob;
	Knob * m_releaseKnob;
	Knob * m_chordScatterKnob;
	Knob * m_chordScatterUniformityKnob;
	
	QPainter m_p;
	
	QPixmap m_partialPixmap = QPixmap(MERCURY_WINDOWSIZE_X, MERCURY_WINDOWSIZE_Y);

	static QPixmap * s_artwork;
} ;


} // namespace gui

} // namespace lmms

#endif
