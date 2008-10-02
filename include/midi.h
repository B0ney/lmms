/*
 * midi.h - constants, structs etc. concerning MIDI
 *
 * Copyright (c) 2005-2008 Tobias Doerffel <tobydox/at/users.sourceforge.net>
 * 
 * This file is part of Linux MultiMedia Studio - http://lmms.sourceforge.net
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


#ifndef _MIDI_H
#define _MIDI_H


#include "types.h"
#include <cstdlib>


enum MidiEventTypes
{
	// messages
	MidiNoteOff = 0x80,
	MidiNoteOn = 0x90,
	MidiKeyPressure = 0xA0,
	MidiControlChange = 0xB0,
	MidiProgramChange = 0xC0,
	MidiChannelPressure = 0xD0,
	MidiPitchBend = 0xE0,
	// system exclusive
	MidiSysEx= 0xF0,
	// system common - never in midi files
	MidiTimeCode= 0xF1,
	MidiSongPosition = 0xF2,
	MidiSongSelect = 0xF3,
	MidiTuneRequest = 0xF6,
	MidiEOX= 0xF7,
	// system real-time - never in midi files
	MidiSync = 0xF8,
	MidiTick = 0xF9,
	MidiStart = 0xFA,
	MidiContinue = 0xFB,
	MidiStop = 0xFC,
	MidiActiveSensing = 0xFE,
	MidiSystemReset = 0xFF,
	// meta event - for midi files only
	MidiMetaEvent = 0xFF
} ;

enum MidiMetaEvents
{
	MidiCopyright = 0x02,
	MidiTrackName = 0x03,
	MidiInstName = 0x04,
	MidiLyric = 0x05,
	MidiMarker = 0x06,
	MidiCuePoint = 0x07,
	MidiPortNumber = 0x21,
	MidiEOT = 0x2f,
	MidiSetTempo = 0x51,
	MidiSMPTEOffset = 0x54,
	MidiTimeSignature = 0x58,
	MidiKeySignature = 0x59,
	MidiSequencerEvent = 0x7f
} ;


const int MidiChannelCount = 16;
const int MidiControllerCount = 128;
const int MidiProgramCount = 128;
const int MidiMaxVelocity = 127;


struct midiEvent
{
	midiEvent( MidiEventTypes _type = MidiActiveSensing,
			Sint8 _channel = 0,
			Sint16 _param1 = 0,
			Sint16 _param2 = 0 ) :
		m_type( _type ),
		m_channel( _channel ),
		m_sysExData( NULL )
	{
		m_data.m_param[0] = _param1;
		m_data.m_param[1] = _param2;
	}
	midiEvent( MidiEventTypes _type, const char * _sysex_data,
							int _data_len ) :
		m_type( _type ),
		m_channel( 0 ),
		m_sysExData( _sysex_data )
	{
		m_data.m_sysExDataLen = _data_len;
	}

	midiEvent( const midiEvent & _copy ) :
		m_type( _copy.m_type ),
		m_channel( _copy.m_channel ),
		m_data( _copy.m_data ),
		m_sysExData( _copy.m_sysExData )
	{
	}

	inline int channel( void ) const
	{
		return m_channel;
	}

	inline Sint16 key( void ) const
	{
		return m_data.m_param[0];
	}

	inline Sint16 & key( void )
	{
		return m_data.m_param[0];
	}

	inline Sint16 velocity( void ) const
	{
		return m_data.m_param[1];
	}

	inline Sint16 & velocity( void )
	{
		return m_data.m_param[1];
	}

	inline volume getVolume( void ) const
	{
		return (volume)( velocity() * 100 / MidiMaxVelocity );
	}


	MidiEventTypes m_type;		// MIDI event type
	Sint8 m_channel;		// MIDI channel
	union
	{
		Sint16 m_param[2];	// first/second parameter (key/velocity)
		Uint8  m_bytes[4];		// raw bytes
		Sint32 m_sysExDataLen;	// len of m_sysExData
	} m_data;

	const char * m_sysExData;

} ;


#endif
