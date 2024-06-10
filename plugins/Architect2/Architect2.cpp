/*
 * Architect2.cpp
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


#include <QDomElement>
#include <qmath.h>
#include "Architect2.h"
#include "SampleLoader.h"
#include "PathUtil.h"
#include "embed.h"
#include "plugin_export.h"

#include <iostream>
using namespace std;

namespace lmms
{

extern "C"
{

Plugin::Descriptor PLUGIN_EXPORT architect2_plugin_descriptor =
{
	LMMS_STRINGIFY( PLUGIN_NAME ),
	"Architect2",
	QT_TRANSLATE_NOOP( "pluginBrowser", "granular synthesizer" ),
	"Lost Robot <r94231/at/gmail/dot/com>",
	0x0100,
	Plugin::Type::Instrument,
	new PluginPixmapLoader( "logo" ),
	nullptr,
	nullptr
} ;

}


aSynthGroup::aSynthGroup( NotePlayHandle * _nph, const sample_rate_t _sample_rate, std::vector<float> (&soundSample)[2], float position )
{
	for (int i = 0; i < 16; ++i)
	{
		m_aSynths.push_back(new aSynth(_nph, _sample_rate, soundSample, position));
	}
}

aSynthGroup::~aSynthGroup()
{
	for (int i = 0; i < 16; ++i)
	{
		delete m_aSynths[i];
	}
}


aSynth::aSynth( NotePlayHandle * _nph, const sample_rate_t _sample_rate, std::vector<float> (&soundSample)[2], float position ) :
	nph( _nph ),
	m_sample_rate( _sample_rate )
{
	for (int i = 0; i < 2; ++i)
	{
		m_pos1[i] = position * soundSample[0].size();
		m_pos2[i] = m_pos1[i];
	}
}


aSynth::~aSynth()
{
}


void aSynth::nextStringSample( sampleFrame &outputSample, std::vector<float> (&soundSample)[2], float volume, float position, float grainSize, float grainSizeKeytrack, float speed, float spray, float fmAmount, float fmFreq, float pitchRand, float ampRand, float dropRand, float dropAmp, float grainSpread, float grainRand, int windowMode, float windowCurve, float windowSlice, float voiceNum, float formant )
{
	m_fmPhase = realdmod(m_fmPhase + fmFreq / m_sample_rate, 1.);

	float grainOutput[2] = {0, 0};

	float randNumSpray = fast_rand() / (float)FAST_RAND_MAX;
	float randNumPitch = fast_rand() / (float)FAST_RAND_MAX;
	float randNumAmp = fast_rand() / (float)FAST_RAND_MAX;
	float randNumDrop = fast_rand() / (float)FAST_RAND_MAX;
	float randNumGrainSize = fast_rand() / (float)FAST_RAND_MAX;
	for (int i = 0; i < 2; ++i)
	{
		m_phase[i] += 1.0;
		m_posAdd[i] += (nph->frequency() + qFastSin(m_fmPhase * D_2PI) * fmAmount) / 440.;
		if (m_phase[i] >= m_trueGrainSize[i])
		{
			m_pos1[i] = m_pos2[i] + m_posAdd[i] * m_pitchMult2[i] - m_trueGrainSize[i] * m_pitchMult2[i];
			m_pos2[i] = realdmod((position + ((spray / m_sample_rate) * (randNumSpray * 2 - 1))) * soundSample[0].size() + m_speedOffset, soundSample[0].size());
			m_phase[i] -= m_trueGrainSize[i];
			float keyGrainSize = grainSize * linearInterpolate(nph->frequency() / 440.f, 1.f, grainSizeKeytrack);
			m_trueGrainSize[i] = m_sample_rate / abs(keyGrainSize + (i ? -grainSpread * 0.5 : grainSpread * 0.5) + ((randNumGrainSize * 2. - 1.) * grainRand));
			m_firstGrain[i] = (m_noteLifetime == 0);
			m_posAdd[i] = 0.;
			m_pitchMult1[i] = m_pitchMult2[i];
			m_pitchMult2[i] = exp2(pitchRand / 1200. * (randNumPitch * 2. - 1.));
			m_ampMult1[i] = m_ampMult2[i];
			m_ampMult2[i] = (ampRand * 0.01 * (randNumAmp * 2. - 1.) + 1.) * (randNumDrop < (dropRand * 0.01) ? dropAmp * 0.01: 1.);
		}

		const double truePos1 = realdmod(m_pos1[i] + m_posAdd[i] * m_pitchMult1[i], soundSample[0].size());
		const double truePos2 = realdmod(m_pos2[i] + m_posAdd[i] * m_pitchMult2[i] - m_trueGrainSize[i] * m_pitchMult2[i], soundSample[0].size());

		double grain1 = linearInterpolate(soundSample[i][floor(truePos1)], soundSample[i][ceil(truePos1)], fmod(truePos1, 1.)) * m_ampMult1[i];
		double grain2 = linearInterpolate(soundSample[i][floor(truePos2)], soundSample[i][ceil(truePos2)], fmod(truePos2, 1.)) * m_ampMult2[i];

		grain1 = m_firstGrain[i] ? 0. : grain1;

		float plainWindowX = m_phase[i] / m_trueGrainSize[i];
		switch (windowMode)
		{
			case 0:
			{
				double windowX = plainWindowX * 0.5 + 0.5;
				float windowMult1 = 0;
				float windowMult2 = 0;
				if (windowX > (windowSlice - 1.) / (2. * windowSlice) && windowX < 1. - ((windowSlice - 1.) / (2. * windowSlice)))
				{
					windowMult1 = qFastCos(F_2PI * (windowX - 0.5) * windowSlice) * 0.5 + 0.5;
				}
				if (windowX < 0.5 && windowX < 1. / (2. * windowSlice))
				{
					windowMult2 = qFastCos(F_2PI * windowX * windowSlice) * 0.5 + 0.5;
				}
				else if (windowX > 0.5 && windowX > 1. - (1. / (2 * windowSlice)))
				{
					windowMult2 = qFastCos(F_2PI * (windowX - 1.) * windowSlice) * 0.5 + 0.5;
				}
				windowMult1 = pow(windowMult1, windowCurve);
				windowMult2 = pow(windowMult2, windowCurve);
				grainOutput[i] = grain1 * windowMult1 + grain2 * windowMult2;
				break;
			}
			case 1:
			{
				double windowX = plainWindowX * 0.5 + 0.5;
				float windowMult1 = 0;
				float windowMult2 = 0;
				if (windowX < windowSlice)
				{
					windowMult2 = qFastCos((windowX * (0.5 / windowSlice)) * F_2PI) * 0.5 + 0.5;
				}
				double modWindowX = fmod(windowX + 0.5f, 1.f);
				if (modWindowX < windowSlice)
				{
					windowMult1 = qFastCos((modWindowX * (0.5 / windowSlice)) * F_2PI) * 0.5 + 0.5;
				}
				windowMult1 = pow(windowMult1, windowCurve);
				windowMult2 = pow(windowMult2, windowCurve);
				grainOutput[i] = grain1 * windowMult1 + grain2 * windowMult2;
				break;
			}
			case 2:
			{
				double windowX = plainWindowX * 0.5 + 0.5;
				float windowMult1 = 0;
				float windowMult2 = 0;
				if (windowX < windowSlice)
				{
					windowMult2 = qFastCos((windowX * (0.5 / windowSlice)) * F_2PI) * 0.5 + 0.5;
					windowMult2 = -windowMult2 + 1.f;
				}
				double modWindowX = fmod(windowX + 0.5f, 1.f);
				if (modWindowX < windowSlice)
				{
					windowMult1 = qFastCos((modWindowX * (0.5 / windowSlice)) * F_2PI) * 0.5 + 0.5;
					windowMult1 = -windowMult1 + 1.f;
				}
				windowMult1 = pow(windowMult1, windowCurve);
				windowMult2 = pow(windowMult2, windowCurve);
				grainOutput[i] = grain1 * windowMult1 + grain2 * windowMult2;
				break;
			}
			case 3:
			{
				double windowX = plainWindowX * 0.5 + 0.5;
				float windowMult1 = 0;
				float windowMult2 = 0;
				windowMult1 = (fast_rand() / (float)FAST_RAND_MAX) * 2 - 1;
				windowMult2 = (fast_rand() / (float)FAST_RAND_MAX) * 2 - 1;
				windowMult1 = pow(windowMult1, windowCurve);
				windowMult2 = pow(windowMult2, windowCurve);
				grainOutput[i] = grain1 * windowMult1 + grain2 * windowMult2;
				break;
			}
		}

		outputSample[i] = grainOutput[i];
	}

	if (speed < 10000)
	{
		m_speedOffset += 1. / (speed * 0.01);
	}

	++m_noteLifetime;
}


// Handles negative values properly, unlike fmod.
inline float aSynth::realfmod( float k, float n )
{
	return ((k = fmod(k,n)) < 0) ? k+n : k;
}

inline double aSynth::realdmod( double k, double n )
{
	return ((k = fmod(k,n)) < 0) ? k+n : k;
}



Architect2::Architect2( InstrumentTrack * _instrument_track ) :
	Instrument( _instrument_track, &architect2_plugin_descriptor ),
	m_volume( 100, 0, 200, 0.00001, this, tr( "Volume" ) ),
	m_position( 0, 0, 1, 0.00001, this, tr( "Position" ) ),
	m_grainSize( 10, 0.5, 150, 0.00001, this, tr( "Grain Size" ) ),
	m_grainSizeKeytrack( 0, 0, 1, 0.00001, this, tr( "Grain Size Keytracking" ) ),
	m_speed( 100, 25, 10000, 0.00001, this, tr( "Speed" ) ),
	m_spray( 0, 0, 10000, 0.00001, this, tr( "Spray" ) ),
	m_fmAmount( 0, 0, 1000, 0.00001, this, tr( "FM Amount" ) ),
	m_fmFreq( 440, 0.25, 1000, 0.00001, this, tr( "FM Frequency" ) ),
	m_pitchRand( 0, 0, 2400, 0.00001, this, tr( "Pitch Randomness" ) ),
	m_ampRand( 0, 0, 100, 0.00001, this, tr( "Amplitude Randomness" ) ),
	m_dropRand( 0, 0, 100, 0.00001, this, tr( "Drop Chance" ) ),
	m_dropAmp( 0, 0, 100, 0.00001, this, tr( "Drop Amplitude" ) ),
	m_grainSpread( 0, -150, 150, 0.00001, this, tr( "Grain Stereo Spread" ) ),
	m_grainRand( 0, 0, 150, 0.00001, this, tr( "Grain Size Randomness" ) ),
	m_windowMode( 0, 0, 3, 1, this, tr( "Window Mode" ) ),
	m_windowCurve( 1, 0.25, 4, 0.00001, this, tr( "Window Curve" ) ),
	m_windowSlice( 1, 0.25, 4, 0.00001, this, tr( "Window Slice" ) ),
	m_voiceNum( 1, 1, 16, 1, this, tr( "Voice Number" ) ),
	m_formant( 0, -1200, 1200, 0.001, this, tr( "Formant" ) ),
	m_sampleLocation("")
{
	m_soundSample[0].push_back( 0 );
	m_soundSample[1].push_back( 0 );

	m_grainSize.setScaleLogarithmic(true);
	m_speed.setScaleLogarithmic(true);
	m_spray.setScaleLogarithmic(true);
	m_fmFreq.setScaleLogarithmic(true);
	m_pitchRand.setScaleLogarithmic(true);
	m_grainSpread.setScaleLogarithmic(true);
	m_grainRand.setScaleLogarithmic(true);
	m_windowCurve.setScaleLogarithmic(true);
	m_windowSlice.setScaleLogarithmic(true);
	m_voiceNum.setScaleLogarithmic(true);
}




Architect2::~Architect2()
{
}




void Architect2::saveSettings( QDomDocument & _doc, QDomElement & _this )
{
	m_volume.saveSettings( _doc, _this, "volume" );
	m_position.saveSettings( _doc, _this, "position" );
	m_grainSize.saveSettings( _doc, _this, "grainSize" );
	m_grainSizeKeytrack.saveSettings( _doc, _this, "grainSizeKeytrack" );
	m_speed.saveSettings( _doc, _this, "speed" );
	m_spray.saveSettings( _doc, _this, "spray" );
	m_fmAmount.saveSettings( _doc, _this, "fmAmount" );
	m_fmFreq.saveSettings( _doc, _this, "fmFreq" );
	m_pitchRand.saveSettings( _doc, _this, "pitchRand" );
	m_ampRand.saveSettings( _doc, _this, "ampRand" );
	m_dropRand.saveSettings( _doc, _this, "dropRand" );
	m_dropAmp.saveSettings( _doc, _this, "dropAmp" );
	m_grainSpread.saveSettings( _doc, _this, "grainSpread" );
	m_grainRand.saveSettings( _doc, _this, "grainRand" );
	m_windowMode.saveSettings( _doc, _this, "windowMode" );
	m_windowCurve.saveSettings( _doc, _this, "windowCurve" );
	m_windowSlice.saveSettings( _doc, _this, "windowSlice" );
	m_voiceNum.saveSettings( _doc, _this, "voiceNum" );
	m_formant.saveSettings( _doc, _this, "formant" );

	// Save plugin version
	_this.setAttribute( "version", "0.1" );

	QString saveString;

	_this.setAttribute( "sampleLocation", m_sampleLocation );

	if (m_sampleLocation == "")
	{
		for( int i = 0; i < 2; ++i )
		{
			base64::encode( (const char *)m_soundSample[i].data(),
				m_soundSample[i].size() * sizeof(float), saveString );
			_this.setAttribute( "soundSample_"+QString::number(i), saveString );
		}
	}

	_this.setAttribute( "sampleSize", (int)m_soundSample[0].size() );
}




void Architect2::loadSettings( const QDomElement & _this )
{
	m_volume.loadSettings( _this, "volume" );
	m_position.loadSettings( _this, "position" );
	m_grainSize.loadSettings( _this, "grainSize" );
	m_grainSizeKeytrack.loadSettings( _this, "grainSizeKeytrack" );
	m_speed.loadSettings( _this, "speed" );
	m_spray.loadSettings( _this, "spray" );
	m_fmAmount.loadSettings( _this, "fmAmount" );
	m_fmFreq.loadSettings( _this, "fmFreq" );
	m_pitchRand.loadSettings( _this, "pitchRand" );
	m_ampRand.loadSettings( _this, "ampRand" );
	m_dropRand.loadSettings( _this, "dropRand" );
	m_dropAmp.loadSettings( _this, "dropAmp" );
	m_grainSpread.loadSettings( _this, "grainSpread" );
	m_grainRand.loadSettings( _this, "grainRand" );
	m_windowMode.loadSettings( _this, "windowMode" );
	m_windowCurve.loadSettings( _this, "windowCurve" );
	m_windowSlice.loadSettings( _this, "windowSlice" );
	m_voiceNum.loadSettings( _this, "voiceNum" );
	m_formant.loadSettings( _this, "formant" );

	int sampleSize = _this.attribute( "sampleSize" ).toInt();

	int size = 0;
	char * dst = 0;

	m_soundSample[0].clear();
	m_soundSample[1].clear();
	
	m_sampleLocation = _this.attribute( "sampleLocation" );
	if( m_sampleLocation != "" )
	{
		QString file_name = m_sampleLocation;

		// TODO
		if (QFileInfo(PathUtil::toAbsolute(file_name)).exists())
		{
			SampleBuffer sample_buffer = SampleBuffer(file_name);

			int filelength = sample_buffer.size();

			m_soundSample[0].clear();
			m_soundSample[1].clear();

			double lengthOfSample = sample_buffer.size();
			for( int i = 0; i < lengthOfSample; ++i )
			{
				m_soundSample[0].push_back(sample_buffer.data()[i][0]);
				m_soundSample[1].push_back(sample_buffer.data()[i][1]);
			}

		} else {
			
		}
		

		std::cout << "m_sampleLocation is not \"\"\n";
		
	}
	else
	{
		for( int i = 0; i < 2; ++i )
		{
			base64::decode( _this.attribute( "soundSample_"+QString::number(i) ), &dst, &size );
			for( int j = 0; j < sampleSize; ++j )
			{
				m_soundSample[i].push_back( ( (float*) dst )[j] );
			}
		}
	}
	
	if( !m_soundSample[0].size() )
	{
		m_soundSample[0].push_back( 0 );
		m_soundSample[1].push_back( 0 );
	}

}




QString Architect2::nodeName() const
{
	return( architect2_plugin_descriptor.name );
}



// TODO
void Architect2::playNote( NotePlayHandle * _n, sampleFrame * _working_buffer )
{
	if ( _n->totalFramesPlayed() == 0 || _n->m_pluginData == NULL )
	{
		_n->m_pluginData = new aSynthGroup( _n, Engine::audioEngine()->outputSampleRate(), m_soundSample, m_position.value() );
	}

	const fpp_t frames = _n->framesLeftForCurrentPeriod();
	const f_cnt_t offset = _n->noteOffset();

	aSynthGroup * ps = static_cast<aSynthGroup *>( _n->m_pluginData );
	for( fpp_t frame = offset; frame < frames + offset; ++frame )
	{
		sampleFrame outputSample = {0,0};
		sampleFrame voiceOutputSample = {0,0};

		const float currentVoiceNum = m_voiceNum.value();

		for (int i = 0; i < currentVoiceNum; ++i)
		{
			ps->m_aSynths[i]->nextStringSample(voiceOutputSample, m_soundSample, m_volume.value(), m_position.value(), m_grainSize.value(), m_grainSizeKeytrack.value(), m_speed.value(), m_spray.value(), m_fmAmount.value(), m_fmFreq.value(), m_pitchRand.value(), m_ampRand.value(), m_dropRand.value(), m_dropAmp.value(), m_grainSpread.value(), m_grainRand.value(), m_windowMode.value(), m_windowCurve.value(), m_windowSlice.value(), m_voiceNum.value(), m_formant.value());
			outputSample[0] += voiceOutputSample[0];
			outputSample[1] += voiceOutputSample[1];
		}

		outputSample[0] /= currentVoiceNum;
		outputSample[1] /= currentVoiceNum;

		for( ch_cnt_t chnl = 0; chnl < DEFAULT_CHANNELS; ++chnl )
		{
			_working_buffer[frame][chnl] = outputSample[chnl];
		}
	}

	applyRelease( _working_buffer, _n );

	instrumentTrack()->processAudioBuffer( _working_buffer, frames + offset, _n );
}




void Architect2::deleteNotePluginData( NotePlayHandle * _n )
{
	delete static_cast<aSynth *>( _n->m_pluginData );
}




gui::PluginView * Architect2::instantiateView( QWidget * _parent )
{
	return( new gui::Architect2View( this, _parent ) );
}





namespace gui
{

Architect2View::Architect2View( Instrument * _instrument,
					QWidget * _parent ) :
	InstrumentView( _instrument, _parent )
{
	setAutoFillBackground( true );
	QPalette pal;

	pal.setBrush( backgroundRole(), PLUGIN_NAME::getIconPixmap(
								"artwork" ) );
	setPalette( pal );
	
	m_volumeKnob = new Knob( KnobType::Dark28, this );
	m_volumeKnob->move( 6, 201 );
	m_volumeKnob->setHintText( tr( "Volume" ), "" );

	m_positionKnob = new Knob( KnobType::Dark28, this );
	m_positionKnob->move( 46, 201 );
	m_positionKnob->setHintText( tr( "Position" ), "" );

	m_grainSizeKnob = new Knob( KnobType::Dark28, this );
	m_grainSizeKnob->move( 86, 201 );
	m_grainSizeKnob->setHintText( tr( "Grain Size" ), "" );
	
	m_grainSizeKeytrackKnob = new Knob( KnobType::Dark28, this );
	m_grainSizeKeytrackKnob->move( 86, 01 );
	m_grainSizeKeytrackKnob->setHintText( tr( "Grain Size Keytracking" ), "" );

	m_speedKnob = new Knob( KnobType::Dark28, this );
	m_speedKnob->move( 126, 201 );
	m_speedKnob->setHintText( tr( "Speed" ), "" );

	m_sprayKnob = new Knob( KnobType::Dark28, this );
	m_sprayKnob->move( 166, 201 );
	m_sprayKnob->setHintText( tr( "Spray" ), "" );

	m_fmAmountKnob = new Knob( KnobType::Dark28, this );
	m_fmAmountKnob->move( 6, 161 );
	m_fmAmountKnob->setHintText( tr( "FM Amount" ), "" );

	m_fmFreqKnob = new Knob( KnobType::Dark28, this );
	m_fmFreqKnob->move( 46, 161 );
	m_fmFreqKnob->setHintText( tr( "FM Frequency" ), "" );

	m_pitchRandKnob = new Knob( KnobType::Dark28, this );
	m_pitchRandKnob->move( 86, 161 );
	m_pitchRandKnob->setHintText( tr( "Pitch Randomness" ), "" );

	m_ampRandKnob = new Knob( KnobType::Dark28, this );
	m_ampRandKnob->move( 126, 161 );
	m_ampRandKnob->setHintText( tr( "Amplitude Randomness" ), "" );

	m_dropRandKnob = new Knob( KnobType::Dark28, this );
	m_dropRandKnob->move( 166, 161 );
	m_dropRandKnob->setHintText( tr( "Drop Chance" ), "" );

	m_dropAmpKnob = new Knob( KnobType::Dark28, this );
	m_dropAmpKnob->move( 206, 161 );
	m_dropAmpKnob->setHintText( tr( "Drop Amplitude" ), "" );

	m_grainSpreadKnob = new Knob( KnobType::Dark28, this );
	m_grainSpreadKnob->move( 6, 121 );
	m_grainSpreadKnob->setHintText( tr( "Grain Stereo Spread" ), "" );

	m_grainRandKnob = new Knob( KnobType::Dark28, this );
	m_grainRandKnob->move( 46, 121 );
	m_grainRandKnob->setHintText( tr( "Grain Size Randomness" ), "" );

	m_windowModeKnob = new Knob( KnobType::Dark28, this );
	m_windowModeKnob->move( 86, 121 );
	m_windowModeKnob->setHintText( tr( "Window Mode" ), "" );

	m_windowCurveKnob = new Knob( KnobType::Dark28, this );
	m_windowCurveKnob->move( 126, 121 );
	m_windowCurveKnob->setHintText( tr( "Window Curve" ), "" );

	m_windowSliceKnob = new Knob( KnobType::Dark28, this );
	m_windowSliceKnob->move( 166, 121 );
	m_windowSliceKnob->setHintText( tr( "Window Slice" ), "" );

	m_voiceNumKnob = new Knob( KnobType::Dark28, this );
	m_voiceNumKnob->move( 16, 81 );
	m_voiceNumKnob->setHintText( tr( "Voice Number" ), "" );

	m_formantKnob = new Knob( KnobType::Dark28, this );
	m_formantKnob->move( 56, 81 );
	m_formantKnob->setHintText( tr( "Formant" ), "" );

	usrWaveBtn = new PixmapButton( this, tr( "User-defined wave" ) );
	usrWaveBtn->move( 131 + 14*5, 105 );
	usrWaveBtn->setActiveGraphic( embed::getIconPixmap(
						"usr_wave_active" ) );
	usrWaveBtn->setInactiveGraphic( embed::getIconPixmap(
						"usr_wave_inactive" ) );
	usrWaveBtn->setToolTip( tr( "User-defined wave" ) );


	connect( usrWaveBtn, SIGNAL ( clicked () ),
			this, SLOT ( usrWaveClicked() ) );

}




void Architect2View::modelChanged()
{
	Architect2 * b = castModel<Architect2>();

	m_volumeKnob->setModel( &b->m_volume );
	m_positionKnob->setModel( &b->m_position );
	m_grainSizeKnob->setModel( &b->m_grainSize );
	m_grainSizeKeytrackKnob->setModel( &b->m_grainSizeKeytrack );
	m_speedKnob->setModel( &b->m_speed );
	m_sprayKnob->setModel( &b->m_spray );
	m_fmAmountKnob->setModel( &b->m_fmAmount );
	m_fmFreqKnob->setModel( &b->m_fmFreq );
	m_pitchRandKnob->setModel( &b->m_pitchRand );
	m_ampRandKnob->setModel( &b->m_ampRand );
	m_dropRandKnob->setModel( &b->m_dropRand );
	m_dropAmpKnob->setModel( &b->m_dropAmp );
	m_grainSpreadKnob->setModel( &b->m_grainSpread );
	m_grainRandKnob->setModel( &b->m_grainRand );
	m_windowModeKnob->setModel( &b->m_windowMode );
	m_windowCurveKnob->setModel( &b->m_windowCurve );
	m_windowSliceKnob->setModel( &b->m_windowSlice );
	m_voiceNumKnob->setModel( &b->m_voiceNum );
	m_formantKnob->setModel( &b->m_formant );
}


// TODO
void Architect2View::usrWaveClicked()
{
	QString af = SampleLoader::openAudioFile();
	if (af.isEmpty()) { return; }

	Architect2 * b = castModel<Architect2>();

	const double sample_rate = Engine::audioEngine()->outputSampleRate();
	SampleBuffer sample_buffer = SampleBuffer(af);

	QString file_name = sample_buffer.audioFile();

	if (file_name.isEmpty()) {
		return;
	}
	
	b->m_sampleLocation = file_name;

	b->m_soundSample[0].clear();
	b->m_soundSample[1].clear();

	double lengthOfSample = sample_buffer.size();
	for( int i = 0; i < lengthOfSample; ++i )
	{
		b->m_soundSample[0].push_back(sample_buffer.data()[i][0]);
		b->m_soundSample[1].push_back(sample_buffer.data()[i][1]);
	}
}

}



extern "C"
{

// necessary for getting instance out of shared lib
PLUGIN_EXPORT Plugin * lmms_plugin_main(Model *m, void *)
{
	return new Architect2(static_cast<InstrumentTrack *>(m));
}


}


}

