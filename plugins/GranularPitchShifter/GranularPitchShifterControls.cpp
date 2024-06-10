/*
 * GranularPitchShifterControls.cpp
 *
 * Copyright (c) 2024 Lost Robot <r94231/at/gmail/dot/com>
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

#include "GranularPitchShifterControls.h"
#include "GranularPitchShifter.h"

namespace lmms
{

GranularPitchShifterControls::GranularPitchShifterControls(GranularPitchShifterEffect* effect) :
	EffectControls(effect),
	m_effect(effect),
	m_pitchModel(-100.f, -2400.f, 2400.f, 0.01f, this, tr("Pitch")),
	m_pitchSpreadModel(0.f, -2400.f, 2400.f, 0.01f, this, tr("Pitch Stereo Spread")),
	m_sizeModel(10.f, 4.f, 1000.f, 0.001f, this, tr("Grain Size")),
	m_shapeModel(2.f, 1.f, 2.f, 0.0001f, this, tr("Shape")),
	m_jitterModel(0.f, 0.f, 1.f, 0.0001f, this, tr("Jitter")),
	m_posrandModel(0.f, 0.f, 1.f, 0.0001f, this, tr("Position Randomization")),
	m_posrandSpreadModel(0.f, 0.f, 1.f, 0.0001f, this, tr("Position Randomization Stereo Spread")),
	m_prefilterModel(1.f, 0.f, 1.f, 1.f, this, tr("Prefilter")),
	m_densityModel(1.f, 1.f, 16.f, 0.0001f, this, tr("Density")),
	m_glideModel(0.01f, 0.f, 0.1f, 0.0001f, this, tr("Glide")),
	m_minLatencyModel(0.f, 0.f, 1.f, 0.00001f, this, tr("Minimum Latency"))
{
	m_pitchModel.setScaleLogarithmic(true);
	m_pitchSpreadModel.setScaleLogarithmic(true);
	m_sizeModel.setScaleLogarithmic(true);
	m_posrandModel.setScaleLogarithmic(true);
	m_posrandSpreadModel.setScaleLogarithmic(true);
	m_densityModel.setScaleLogarithmic(true);
	m_glideModel.setScaleLogarithmic(true);
	m_minLatencyModel.setScaleLogarithmic(true);
	
	m_pitchModel.setInitValue(0);
}


void GranularPitchShifterControls::loadSettings(const QDomElement& parent)
{
	m_pitchModel.loadSettings(parent, "pitch");
	m_pitchSpreadModel.loadSettings(parent, "pitchSpread");
	m_sizeModel.loadSettings(parent, "size");
	m_shapeModel.loadSettings(parent, "shape");
	m_jitterModel.loadSettings(parent, "jitter");
	m_posrandModel.loadSettings(parent, "posrand");
	m_posrandSpreadModel.loadSettings(parent, "posrandSpread");
	m_prefilterModel.loadSettings(parent, "prefilter");
	m_densityModel.loadSettings(parent, "density");
	m_glideModel.loadSettings(parent, "glide");
	m_minLatencyModel.loadSettings(parent, "minLatency");
}


void GranularPitchShifterControls::saveSettings(QDomDocument& doc, QDomElement& parent)
{
	m_pitchModel.saveSettings(doc, parent, "pitch");
	m_pitchSpreadModel.saveSettings(doc, parent, "pitchSpread");
	m_sizeModel.saveSettings(doc, parent, "size");
	m_shapeModel.saveSettings(doc, parent, "shape");
	m_jitterModel.saveSettings(doc, parent, "jitter");
	m_posrandModel.saveSettings(doc, parent, "posrand");
	m_posrandSpreadModel.saveSettings(doc, parent, "posrandSpread");
	m_prefilterModel.saveSettings(doc, parent, "prefilter");
	m_densityModel.saveSettings(doc, parent, "density");
	m_glideModel.saveSettings(doc, parent, "glide");
	m_minLatencyModel.saveSettings(doc, parent, "minLatency");
}


} // namespace lmms
