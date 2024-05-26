/*
 * Architect2.h
 *
 * Copyright (c) 2021 Lost Robot [r94231@gmail.com]
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


#ifndef ARCHITECT2_H
#define ARCHITECT2_H

#include "Instrument.h"
#include "InstrumentView.h"
#include "AutomatableModel.h"
#include "Knob.h"
#include "MemoryManager.h"
#include "PixmapButton.h"
#include "base64.h"
#include "Engine.h"
#include "InstrumentTrack.h"
#include "Knob.h"
#include "lmms_math.h"
#include "Mixer.h"
#include "Oscillator.h"
#include "Song.h"
#include "interpolation.h"
#include "lmms_basics.h"


namespace lmms
{


class oscillator;

namespace gui
{
class Architect2View;
}

class aSynth
{
	MM_OPERATORS
public:
	aSynth( NotePlayHandle * _nph, const sample_rate_t _sample_rate, std::vector<float> (&soundSample)[2], float position );
	virtual ~aSynth();
	
	void nextStringSample( sampleFrame &outputSample, std::vector<float> (&soundSample)[2], float volume, float position, float grainSize, float grainSizeKeytrack, float speed, float spray, float fmAmount, float fmFreq, float pitchRand, float ampRand, float dropRand, float dropAmp, float grainSpread, float grainRand, int windowMode, float windowCurve, float windowSlice, float voiceNum, float formant );


private:
	inline float realfmod( float k, float n );
	inline double realdmod( double k, double n );

	NotePlayHandle* nph;

	const double m_sample_rate;

	double m_pos1[2] = {0, 0};
	double m_pos2[2] = {0, 0};

	double m_phase[2] = {1, 1};
	double m_trueGrainSize[2] = {0, 0};

	double m_noteLifetime = 0;
	double m_speedOffset = 0;

	bool m_firstGrain[2] = {true, true};

	double m_fmPhase;

	double m_posAdd[2] = {0, 0};

	double m_pitchMult1[2] = {1, 1};
	double m_pitchMult2[2] = {1, 1};

	double m_ampMult1[2] = {1, 1};
	double m_ampMult2[2] = {1, 1};
};

class aSynthGroup
{
public:
	aSynthGroup( NotePlayHandle * _nph, const sample_rate_t _sample_rate, std::vector<float> (&soundSample)[2], float position );
	virtual ~aSynthGroup();

	std::vector<aSynth*> m_aSynths;
};

class Architect2 : public Instrument
{
	Q_OBJECT
public:
	Architect2(InstrumentTrack * _instrument_track );
	virtual ~Architect2();

	virtual void playNote( NotePlayHandle * _n,
						sampleFrame * _working_buffer );
	virtual void deleteNotePluginData( NotePlayHandle * _n );


	virtual void saveSettings( QDomDocument & _doc,
							QDomElement & _parent );
	virtual void loadSettings( const QDomElement & _this );

	virtual QString nodeName() const;

	virtual f_cnt_t desiredReleaseFrames() const
	{
		return( 64 );
	}

	gui::PluginView * instantiateView( QWidget * _parent );

protected slots:


private:
	std::vector<float> m_soundSample[2];

	FloatModel  m_volume;
	FloatModel  m_position;
	FloatModel  m_grainSize;
	FloatModel  m_grainSizeKeytrack;
	FloatModel  m_speed;
	FloatModel  m_spray;
	FloatModel  m_fmAmount;
	FloatModel  m_fmFreq;
	FloatModel  m_pitchRand;
	FloatModel  m_ampRand;
	FloatModel  m_dropRand;
	FloatModel  m_dropAmp;
	FloatModel  m_grainSpread;
	FloatModel  m_grainRand;
	FloatModel  m_windowMode;
	FloatModel  m_windowCurve;
	FloatModel  m_windowSlice;
	FloatModel  m_voiceNum;
	FloatModel  m_formant;
	
	QString m_sampleLocation;

	friend class gui::Architect2View;
};

namespace gui
{

class Architect2View : public InstrumentView
{
	Q_OBJECT
public:
	Architect2View( Instrument * _instrument,
					QWidget * _parent );

	virtual ~Architect2View() {};

	QSize sizeHint() const override { return QSize(405, 250); }

private slots:
	void usrWaveClicked();


private:
	virtual void modelChanged();

	Knob * m_volumeKnob;
	Knob * m_positionKnob;
	Knob * m_grainSizeKnob;
	Knob * m_grainSizeKeytrackKnob;
	Knob * m_speedKnob;
	Knob * m_sprayKnob;
	Knob * m_fmAmountKnob;
	Knob * m_fmFreqKnob;
	Knob * m_pitchRandKnob;
	Knob * m_ampRandKnob;
	Knob * m_dropRandKnob;
	Knob * m_dropAmpKnob;
	Knob * m_grainSpreadKnob;
	Knob * m_grainRandKnob;
	Knob * m_windowModeKnob;
	Knob * m_windowCurveKnob;
	Knob * m_windowSliceKnob;
	Knob * m_voiceNumKnob;
	Knob * m_formantKnob;

	PixmapButton * usrWaveBtn;

	static QPixmap * s_artwork;
};

}


}


#endif
