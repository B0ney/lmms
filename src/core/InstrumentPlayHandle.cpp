/*
 * InstrumentPlayHandle.cpp - play-handle for driving an instrument
 *
 * Copyright (c) 2005-2014 Tobias Doerffel <tobydox/at/users.sourceforge.net>
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


#include "InstrumentPlayHandle.h"
#include "Instrument.h"
#include "InstrumentTrack.h"
#include "Engine.h"
#include "AudioEngine.h"

namespace lmms
{


InstrumentPlayHandle::InstrumentPlayHandle( Instrument * instrument, InstrumentTrack* instrumentTrack ) :
		PlayHandle( Type::InstrumentPlayHandle ),
		m_instrument( instrument )
{
	setAudioPort( instrumentTrack->audioPort() );
}

void InstrumentPlayHandle::play( sampleFrame * _working_buffer )
{
	InstrumentTrack * instrumentTrack = m_instrument->instrumentTrack();

	// ensure that all our nph's have been processed first
	ConstNotePlayHandleList nphv = NotePlayHandle::nphsOfInstrumentTrack(instrumentTrack, true );
	
	bool nphsLeft;
	do
	{
		nphsLeft = false;
		for( const NotePlayHandle * constNotePlayHandle : nphv )
		{
			NotePlayHandle * notePlayHandle = const_cast<NotePlayHandle *>( constNotePlayHandle );
			if( notePlayHandle->state() != ThreadableJob::ProcessingState::Done &&
				!notePlayHandle->isFinished())
			{
				nphsLeft = true;
				notePlayHandle->process();
			}
		}
	}
	while( nphsLeft );
	
	m_instrument->play( _working_buffer );

	// Process the audio buffer that the instrument has just worked on...
	const fpp_t frames = Engine::audioEngine()->framesPerPeriod();
	instrumentTrack->processAudioBuffer(_working_buffer, frames, nullptr);
}

bool InstrumentPlayHandle::isFromTrack( const Track* _track ) const
{
	return m_instrument->isFromTrack( _track );
}


} // namespace lmms