/*
 * GranularPitchShifterControlDialog.cpp - control dialog for granularpitchshifter effect
 *
 * Copyright (c) 2014 Vesa Kivimäki <contact/dot/diizy/at/nbl/dot/fi>
 * Copyright (c) 2006-2014 Tobias Doerffel <tobydox/at/users.sourceforge.net>
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

#include "GranularPitchShifterControlDialog.h"
#include "GranularPitchShifterControls.h"
#include "embed.h"
#include "Knob.h"

namespace lmms::gui
{

GranularPitchShifterControlDialog::GranularPitchShifterControlDialog(GranularPitchShifterControls* controls) :
	EffectControlDialog(controls)
{
	setAutoFillBackground(true);
	QPalette pal;
	pal.setBrush(backgroundRole(), PLUGIN_NAME::getIconPixmap("artwork"));
	setPalette(pal);
	setFixedSize(350, 110);
	
	auto makeKnob = [this](int x, int y, const QString& label, const QString& hintText, const QString& unit, FloatModel* model)
	{
        Knob* newKnob = new Knob(KnobType::Bright26, this);
        newKnob->move(x, y);
        newKnob->setModel(model);
        newKnob->setLabel(label);
        newKnob->setHintText(hintText, unit);
        return newKnob;
    };

	makeKnob(16, 10, tr("PITCH"), tr("Pitch:"), " cents", &controls->m_pitchModel);
	makeKnob(156, 10, tr("PITCHSPREAD"), tr("Pitch Stereo Spread:"), " cents", &controls->m_pitchSpreadModel);
	makeKnob(57, 10, tr("SIZE"), tr("Grain Size:"), " Hz", &controls->m_sizeModel);
	makeKnob(16, 65, tr("SHAPE"), tr("Grain Shape:"), "", &controls->m_shapeModel);
	makeKnob(57, 65, tr("JITTER"), tr("Jitter:"), "", &controls->m_jitterModel);
	makeKnob(107, 65, tr("POS RAND"), tr("Position Randomization:"), "", &controls->m_posrandModel);
	makeKnob(157, 65, tr("POSRANDSPREAD"), tr("Position Randomization Stereo Spread:"), "", &controls->m_posrandSpreadModel);
	makeKnob(107, 15, tr("PREFILTER"), tr("Prefilter:"), "", &controls->m_prefilterModel);
	makeKnob(207, 15, tr("DENSITY"), tr("Density:"), "x", &controls->m_densityModel);
	makeKnob(257, 65, tr("GLIDE"), tr("Glide:"), " seconds", &controls->m_glideModel);
}

} // namespace lmms::gui
