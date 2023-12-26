/*
 * LimiterControls.cpp
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

#include <QDomElement>

#include "LimiterControls.h"
#include "Limiter.h"

namespace lmms
{

LimiterControls::LimiterControls(LimiterEffect* effect) :
	EffectControls(effect),
	m_effect(effect),
	m_lookaheadModel(2.f, 0.0f, LIMITER_MAX_LOOKAHEAD, 0.0001f, this, tr("Lookahead")),
	m_lookReleaseModel(80.0f, 0.0f, 1000.f, 0.0001f, this, tr("Look Release")),
	m_lookReleaseStagesModel(2, 1, LIMITER_MAX_RELEASE_STAGES, 1, this, tr("Look Release Stages")),
	m_adaptReleaseModel(0.f, 0.f, 2.f, 0.0001f, this, tr("Adaptive Release")),
	m_adaptSlowModel(0.f, 0.f, 1.f, 0.0001f, this, tr("Adaptive Release Slowing")),
	m_stereoLinkTransientModel(0.75f, 0.f, 1.f, 0.0001f, this, tr("Stereo Link Transient")),
	m_stereoLinkReleaseModel(1.f, 0.f, 1.f, 0.0001f, this, tr("Stereo Link Release")),
	m_dcFreqModel(0.0f, 0.f, 20.f, 0.0001f, this, tr("DC Offset Removal Frequency")),
	m_truePeakModel(0, 0, 1, 1, this, tr("True Peak Limiting")),
	m_thresholdModel(0.f, -36.f, 0.f, 0.0001f, this, tr("Threshold")),
	m_inGainModel(0.f, -36.f, 36.f, 0.0001f, this, tr("Input Gain")),
	m_outGainModel(0.f, -36.f, 36.f, 0.0001f, this, tr("Output Gain")),
	m_holdModel(0.f, 0.f, LIMITER_MAX_HOLD, 0.0001f, this, tr("Hold")),
	m_oversampleModel(0, 0, 5, 1, this, tr("Oversample")),
	m_ditherEnabledModel(0, 0, 1, 1, this, tr("Dither Enabled")),
	m_ditherBitDepthModel(24, 4, 30, this, tr("Dither Bit Depth")),
	m_ditherTypeModel(this, tr("Dither Type")),
	m_smoothStagesModel(6, 1, 6, 1, this, tr("Smoothing Stages")),
	m_cleanClipAmountModel(0.f, 0.f, 1.f, 0.0001f, this, tr("Clean Clip Amount")),
	m_cleanClipCeilingModel(48.f, 0.f, 48.f, 0.0001f, this, tr("Clean Clip Ceiling")),
	m_cleanClipStartModel(0.f, -96.f, 48.f, 0.0001f, this, tr("Clean Clip Start Gain")),
	m_kWeightCrestModel(1, 0, 1, 1, this, tr("K-Weight Crest Factor Measurements"))
{
	m_lookReleaseModel.setScaleLogarithmic(true);
	m_thresholdModel.setScaleLogarithmic(true);
	m_inGainModel.setScaleLogarithmic(true);
	m_outGainModel.setScaleLogarithmic(true);
	m_holdModel.setScaleLogarithmic(true);
	m_cleanClipCeilingModel.setScaleLogarithmic(true);
	m_cleanClipStartModel.setScaleLogarithmic(true);
	
	m_ditherTypeModel.addItem(tr("RPDF"));
	m_ditherTypeModel.addItem(tr("TPDF"));
	m_ditherTypeModel.addItem(tr("GPDF"));
	m_ditherTypeModel.setInitValue(1);
	m_ditherTypeModel.setValue(1);
}


void LimiterControls::loadSettings(const QDomElement& parent)
{
	m_lookaheadModel.loadSettings(parent, "lookahead");
	m_lookReleaseModel.loadSettings(parent, "lookRelease");
	m_lookReleaseStagesModel.loadSettings(parent, "lookReleaseStages");
	m_adaptReleaseModel.loadSettings(parent, "adaptRelease");
	m_adaptSlowModel.loadSettings(parent, "adaptSlow");
	m_stereoLinkTransientModel.loadSettings(parent, "stereoLinkTransient");
	m_stereoLinkReleaseModel.loadSettings(parent, "stereoLinkRelease");
	m_dcFreqModel.loadSettings(parent, "dcFreq");
	m_truePeakModel.loadSettings(parent, "truePeak");
	m_thresholdModel.loadSettings(parent, "threshold");
	m_inGainModel.loadSettings(parent, "inGain");
	m_outGainModel.loadSettings(parent, "outGain");
	m_holdModel.loadSettings(parent, "hold");
	m_oversampleModel.loadSettings(parent, "oversample");
	m_ditherEnabledModel.loadSettings(parent, "ditherEnabled");
	m_ditherBitDepthModel.loadSettings(parent, "ditherBitDepth");
	m_ditherTypeModel.loadSettings(parent, "ditherType");
	m_smoothStagesModel.loadSettings(parent, "smoothStages");
	m_cleanClipAmountModel.loadSettings(parent, "cleanClipAmount");
	m_cleanClipCeilingModel.loadSettings(parent, "cleanClipCeiling");
	m_cleanClipStartModel.loadSettings(parent, "cleanClipStart");
	m_kWeightCrestModel.loadSettings(parent, "kWeightCrest");
}


void LimiterControls::saveSettings(QDomDocument& doc, QDomElement& parent)
{
	m_lookaheadModel.saveSettings(doc, parent, "lookahead");
	m_lookReleaseModel.saveSettings(doc, parent, "lookRelease");
	m_lookReleaseStagesModel.saveSettings(doc, parent, "lookReleaseStages");
	m_adaptReleaseModel.saveSettings(doc, parent, "adaptRelease");
	m_adaptSlowModel.saveSettings(doc, parent, "adaptSlow");
	m_stereoLinkTransientModel.saveSettings(doc, parent, "stereoLinkTransient");
	m_stereoLinkReleaseModel.saveSettings(doc, parent, "stereoLinkRelease");
	m_dcFreqModel.saveSettings(doc, parent, "dcFreq");
	m_truePeakModel.saveSettings(doc, parent, "truePeak");
	m_thresholdModel.saveSettings(doc, parent, "threshold");
	m_inGainModel.saveSettings(doc, parent, "inGain");
	m_outGainModel.saveSettings(doc, parent, "outGain");
	m_holdModel.saveSettings(doc, parent, "hold");
	m_oversampleModel.saveSettings(doc, parent, "oversample");
	m_ditherEnabledModel.saveSettings(doc, parent, "ditherEnabled");
	m_ditherBitDepthModel.saveSettings(doc, parent, "ditherBitDepth");
	m_ditherTypeModel.saveSettings(doc, parent, "ditherType");
	m_smoothStagesModel.saveSettings(doc, parent, "smoothStages");
	m_cleanClipAmountModel.saveSettings(doc, parent, "cleanClipAmount");
	m_cleanClipCeilingModel.saveSettings(doc, parent, "cleanClipCeiling");
	m_cleanClipStartModel.saveSettings(doc, parent, "cleanClipStart");
	m_kWeightCrestModel.saveSettings(doc, parent, "kWeightCrest");
}


} // namespace lmms
