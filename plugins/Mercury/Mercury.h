/*
 * Mercury.h
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


#ifndef MERCURY_H
#define MERCURY_H

#include "Instrument.h"
#include "InstrumentView.h"
#include "Knob.h"
#include "BasicFilters.h"
#include "RingBuffer.h"

namespace lmms
{

constexpr int PARTIAL_MAX = 100;
constexpr int PARTIAL_FREQ_TRUE_MAX = 16000;

constexpr int MAX_DELAY_TIME = 2000;


class oscillator;

namespace gui
{
class MercuryView;
}


class sSynth
{
public:
	sSynth( NotePlayHandle * _nph, const sample_rate_t _sample_rate );
	virtual ~sSynth();
	
	inline void nextStringSample( sampleFrame &outputSample, int frame, float decay, int partialNum, float partialFreqMax, float partialStretch, float partialSine, float partialLP, float partialLPSlope, float partialDamp, float fundamental, float partialComb1, float partialComb2, float partialCombShift, float partialCombBal, float partialRing, float partialRound, float partialRoundMax, float partialFreqBal, float exciterLP, float exciterHP, float exciterImpulse, float exciterNoise, float partialProtect, float partialProtectSlope, float exciterFeedback, float exciterFeedbackDelay, float modalVol, float exciterFeedbackPhase, float exciterFade, float exciterDelayFade, float exciterAttack, float exciterDecay1, float exciterBreakpoint, float exciterDecay2, float exciterSustain, float exciterRelease, float exciterSaturation, float exciterAsym, float exciterPulseTrain, float exciterFeedbackLP, float reverbMix, float reverbDecay, float reverbSize, float reverbDiffusion, float reverbHighDamp, float reverbLowDamp, float reverbMod, float reverbModFreq, float reverbWidth, float flangerMix, float flangerDelay, float flangerDepth, float flangerRate, float flangerFeedback, float flangerDamping, float flangerPhase, float delayMix, float delayTime, float delayStereo, float delayHighDamp, float delayLowDamp, float delayFeedback, float exciterLeak );
	inline void updatePartials(float decay, float partialStretch, float partialSine, float partialLP, float partialLPSlope, float partialDamp, float fundamental, float partialComb1, float partialComb2, float partialCombShift, float partialCombBal, float partialRing, float partialRound, float partialRoundMax, float partialFreqBal, float exciterLP, float exciterHP, float exciterImpulse, float exciterNoise, float partialProtect, float partialProtectSlope);
	inline void updateComb(int partialNum, float partialFreqMax, float partialComb1, float partialComb2, float partialCombShift);

private:
	inline float calcResonator(float input, int index, float frequency, float feedback, float leak);
	inline float calcLogistic(float input, float max, float slope, float midpoint);
	inline float realfmod(float k, float n);
	inline int realmod(int k, int n);
	inline sample_t calcAllpassFilter(sample_t inSamp, float b0, float b1);
	inline float calcEnvelope(float x, float a, float d1, float b, float d2, float s, float r, float &gainStorage);
	inline float expCurve(float x, float minimum, float maximum);
	inline float msToCoeff(float ms);

	NotePlayHandle * m_nph;

	float m_APFx[2] = {0};
	float m_APFy[2] = {0};

	float m_sampleRate;

	/*float m_partialX[PARTIAL_MAX][2] = {{0}};
	float m_partialY[PARTIAL_MAX][2] = {{0}};*/
	float m_partialDelay1[PARTIAL_MAX] = {0};
	float m_partialDelay2[PARTIAL_MAX] = {0};
	int m_noteDuration = 0;
	int m_noteReleaseTime = 0;
	
	// For performance
	float m_partialLastFreq[PARTIAL_MAX] = {0};
	float m_partialLastFeedback[PARTIAL_MAX] = {0};
	float m_partialSinWAbs[PARTIAL_MAX] = {0};
	float m_partialCosW[PARTIAL_MAX] = {0};
	float m_partialSinWFeed[PARTIAL_MAX] = {0};
	float m_partialCosWFeed[PARTIAL_MAX] = {0};
	
	float m_comb1GainPre[PARTIAL_MAX] = {0};
	float m_comb2GainPre[PARTIAL_MAX] = {0};

	LinkwitzRiley<1> m_exciterLPF;
	LinkwitzRiley<1> m_exciterHPF;
	float m_exciterLPlast = 0;
	float m_exciterHPlast = 0;

	std::vector<float> m_exciterBuf;
	int m_exciterBufIndex = 0;
	float m_exciterBufSize;
	float m_exciterDelayOutput = 0;
	float m_saturatedAllpassOutputLP = 0;
	float m_saturatedAllpassOutputDC = 0;

	float m_partialFreq[PARTIAL_MAX] = {0};
	float m_partialGain[PARTIAL_MAX] = {0};
	float m_partialRadius[PARTIAL_MAX] = {0};
	
	float m_envGainStorage = 1;
	float m_coeffPrecalc = 1;
} ;

class Mercury : public Instrument
{
	Q_OBJECT
public:
	Mercury(InstrumentTrack * _instrument_track );
	virtual ~Mercury();

	virtual void playNote( NotePlayHandle * _n,
						sampleFrame * _working_buffer );
	virtual void deleteNotePluginData( NotePlayHandle * _n );


	virtual void saveSettings( QDomDocument & _doc,
							QDomElement & _parent );
	virtual void loadSettings( const QDomElement & _this );

	virtual QString nodeName() const;

	virtual f_cnt_t desiredReleaseFrames() const;

	virtual gui::PluginView * instantiateView( QWidget * _parent );

	virtual void play( sampleFrame * _working_buffer );

private slots:
	void updatePartials();
	void updateComb();

private:
	inline sample_t calcAllpassFilter(sample_t inSamp, float b0, float b1, int num, int chnl);
	inline float realfmod(float k, float n);

	FloatModel m_decay;
	FloatModel m_partialNum;
	FloatModel m_partialFreqMax;
	FloatModel m_partialStretch;
	FloatModel m_partialSine;
	FloatModel m_partialLP;
	FloatModel m_partialLPSlope;
	FloatModel m_partialDamp;
	FloatModel m_fundamental;
	FloatModel m_partialComb1;
	FloatModel m_partialComb2;
	FloatModel m_partialCombShift;
	FloatModel m_partialCombBal;
	FloatModel m_partialRing;
	FloatModel m_partialRound;
	FloatModel m_partialRoundMax;
	FloatModel m_partialFreqBal;
	FloatModel m_exciterLP;
	FloatModel m_exciterHP;
	FloatModel m_exciterImpulse;
	FloatModel m_exciterNoise;
	FloatModel m_partialProtect;
	FloatModel m_partialProtectSlope;
	FloatModel m_exciterFeedback;
	FloatModel m_exciterFeedbackDelay;
	FloatModel m_modalVol;
	FloatModel m_exciterFeedbackPhase;
	FloatModel m_exciterFade;
	FloatModel m_exciterDelayFade;
	FloatModel m_exciterAttack;
	FloatModel m_exciterDecay1;
	FloatModel m_exciterBreakpoint;
	FloatModel m_exciterDecay2;
	FloatModel m_exciterSustain;
	FloatModel m_exciterRelease;
	FloatModel m_exciterSaturation;
	FloatModel m_exciterAsym;
	FloatModel m_exciterPulseTrain;
	FloatModel m_exciterFeedbackLP;
	FloatModel m_reverbMix;
	FloatModel m_reverbDecay;
	FloatModel m_reverbSize;
	FloatModel m_reverbDiffusion;
	FloatModel m_reverbHighDamp;
	FloatModel m_reverbLowDamp;
	FloatModel m_reverbMod;
	FloatModel m_reverbModFreq;
	FloatModel m_reverbWidth;
	FloatModel m_flangerMix;
	FloatModel m_flangerDelay;
	FloatModel m_flangerDepth;
	FloatModel m_flangerRate;
	FloatModel m_flangerFeedback;
	FloatModel m_flangerDamping;
	FloatModel m_flangerPhase;
	FloatModel m_delayMix;
	FloatModel m_delayTime;
	FloatModel m_delayStereo;
	FloatModel m_delayHighDamp;
	FloatModel m_delayLowDamp;
	FloatModel m_delayFeedback;
	FloatModel m_exciterLeak;

	std::vector<float> m_reverbBuf[4][2];
	int m_reverbBufIndex[4] = {0};
	float m_reverbBufSize[4] = {0};
	float m_reverbSizeMult[4] = {0};
	float m_reverbHighDampOut[4][2] = {{0}};
	float m_reverbLowDampOut[4][2] = {{0}};

	std::vector<float> m_flangerBuf[2];
	int m_flangerBufIndex = 0;
	float m_flangerBufSize = 0;
	float m_flangerDampingOut[2] = {0};

	std::vector<float> m_delayBuf[2];
	int m_delayBufIndex = 0;
	float m_delayBufSize = 0;
	float m_delayHighDampOut[2] = {0};
	float m_delayLowDampOut[2] = {0};

	float m_sampleRate;

	double m_lifetime = 0;

	float m_APFx[4][2][2] = {0};
	float m_APFy[4][2][2] = {0};

	friend class gui::MercuryView;
} ;

namespace gui
{

class MercuryView : public InstrumentView
{
	Q_OBJECT
public:
	MercuryView( Instrument * _instrument,
					QWidget * _parent );

	virtual ~MercuryView() {};

protected slots:


private:
	virtual void modelChanged();

	Knob * m_decayKnob;
	Knob * m_partialNumKnob;
	Knob * m_partialFreqMaxKnob;
	Knob * m_partialStretchKnob;
	Knob * m_partialSineKnob;
	Knob * m_partialLPKnob;
	Knob * m_partialLPSlopeKnob;
	Knob * m_partialDampKnob;
	Knob * m_fundamentalKnob;
	Knob * m_partialComb1Knob;
	Knob * m_partialComb2Knob;
	Knob * m_partialCombShiftKnob;
	Knob * m_partialCombBalKnob;
	Knob * m_partialRingKnob;
	Knob * m_partialRoundKnob;
	Knob * m_partialRoundMaxKnob;
	Knob * m_partialFreqBalKnob;
	Knob * m_exciterLPKnob;
	Knob * m_exciterHPKnob;
	Knob * m_exciterImpulseKnob;
	Knob * m_exciterNoiseKnob;
	Knob * m_partialProtectKnob;
	Knob * m_partialProtectSlopeKnob;
	Knob * m_exciterFeedbackKnob;
	Knob * m_exciterFeedbackDelayKnob;
	Knob * m_modalVolKnob;
	Knob * m_exciterFeedbackPhaseKnob;
	Knob * m_exciterFadeKnob;
	Knob * m_exciterDelayFadeKnob;
	Knob * m_exciterAttackKnob;
	Knob * m_exciterDecay1Knob;
	Knob * m_exciterBreakpointKnob;
	Knob * m_exciterDecay2Knob;
	Knob * m_exciterSustainKnob;
	Knob * m_exciterReleaseKnob;
	Knob * m_exciterSaturationKnob;
	Knob * m_exciterAsymKnob;
	Knob * m_exciterPulseTrainKnob;
	Knob * m_exciterFeedbackLPKnob;
	Knob * m_reverbMixKnob;
	Knob * m_reverbDecayKnob;
	Knob * m_reverbSizeKnob;
	Knob * m_reverbDiffusionKnob;
	Knob * m_reverbHighDampKnob;
	Knob * m_reverbLowDampKnob;
	Knob * m_reverbModKnob;
	Knob * m_reverbModFreqKnob;
	Knob * m_reverbWidthKnob;
	Knob * m_flangerMixKnob;
	Knob * m_flangerDelayKnob;
	Knob * m_flangerDepthKnob;
	Knob * m_flangerRateKnob;
	Knob * m_flangerFeedbackKnob;
	Knob * m_flangerDampingKnob;
	Knob * m_flangerPhaseKnob;
	Knob * m_delayMixKnob;
	Knob * m_delayTimeKnob;
	Knob * m_delayStereoKnob;
	Knob * m_delayHighDampKnob;
	Knob * m_delayLowDampKnob;
	Knob * m_delayFeedbackKnob;
	Knob * m_exciterLeakKnob;

	static QPixmap * s_artwork;
} ;

}


}

#endif
