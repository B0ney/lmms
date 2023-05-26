/*
 * LOMMControls.cpp
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


#include "LOMMControls.h"
#include "LOMM.h"

#include <QDomElement>

namespace lmms
{

LOMMControls::LOMMControls(LOMMEffect* effect) :
	EffectControls(effect),
	m_effect(effect),
	m_depthModel(0.4, 0, 1, 0.00001, this, tr("Depth")),
	m_timeModel(1, 0, 10, 0.00001, this, tr("Time")),
	m_inVolModel(0, -48, 48, 0.00001, this, tr("Input Volume")),
	m_outVolModel(8, -48, 48, 0.00001, this, tr("Output Volume")),
	m_upwardModel(1, 0, 2, 0.00001, this, tr("Upward Depth")),
	m_downwardModel(1, 0, 2, 0.00001, this, tr("Downward Depth")),
	m_split1Model(2500, 20, 20000, 0.01, this, tr("High/Mid Split")),
	m_split2Model(88.3, 20, 20000, 0.01, this, tr("Mid/Low Split")),
	m_split1EnabledModel(true, this, tr("Enable High/Mid Split")),
	m_split2EnabledModel(true, this, tr("Enable Mid/Low Split")),
	m_band1EnabledModel(true, this, tr("Enable High Band")),
	m_band2EnabledModel(true, this, tr("Enable Mid Band")),
	m_band3EnabledModel(true, this, tr("Enable Low Band")),
	m_inHighModel(0, -48, 48, 0.00001, this, tr("High Input Volume")),
	m_inMidModel(0, -48, 48, 0.00001, this, tr("Mid Input Volume")),
	m_inLowModel(0, -48, 48, 0.00001, this, tr("Low Input Volume")),
	m_outHighModel(4.6, -48, 48, 0.00001, this, tr("High Output Volume")),
	m_outMidModel(0.0, -48, 48, 0.00001, this, tr("Mid Output Volume")),
	m_outLowModel(4.6, -48, 48, 0.00001, this, tr("Low Output Volume")),
	m_aThreshHModel(-30.3, LOMM_DISPLAY_MIN, LOMM_DISPLAY_MAX, 0.001, this, tr("Above Threshold High")),
	m_aThreshMModel(-25.0, LOMM_DISPLAY_MIN, LOMM_DISPLAY_MAX, 0.001, this, tr("Above Threshold Mid")),
	m_aThreshLModel(-28.6, LOMM_DISPLAY_MIN, LOMM_DISPLAY_MAX, 0.001, this, tr("Above Threshold Low")),
	m_aRatioHModel(99.99, 1, 99.99, 0.01, this, tr("Above Ratio High")),
	m_aRatioMModel(66.7, 1, 99.99, 0.01, this, tr("Above Ratio Mid")),
	m_aRatioLModel(66.7, 1, 99.99, 0.01, this, tr("Above Ratio Low")),
	m_bThreshHModel(-35.6, LOMM_DISPLAY_MIN, LOMM_DISPLAY_MAX, 0.001, this, tr("Below Threshold High")),
	m_bThreshMModel(-36.6, LOMM_DISPLAY_MIN, LOMM_DISPLAY_MAX, 0.001, this, tr("Below Threshold Mid")),
	m_bThreshLModel(-35.6, LOMM_DISPLAY_MIN, LOMM_DISPLAY_MAX, 0.001, this, tr("Below Threshold Low")),
	m_bRatioHModel(4.17, 1, 99.99, 0.01, this, tr("Below Ratio High")),
	m_bRatioMModel(4.17, 1, 99.99, 0.01, this, tr("Below Ratio Mid")),
	m_bRatioLModel(4.17, 1, 99.99, 0.01, this, tr("Below Ratio Low")),
	m_atkHModel(13.5, 0.1, 1000, 0.001, this, tr("Attack High")),
	m_atkMModel(22.4, 0.1, 1000, 0.001, this, tr("Attack Mid")),
	m_atkLModel(47.8, 0.1, 1000, 0.001, this, tr("Attack Low")),
	m_relHModel(132, 0.1, 1000, 0.001, this, tr("Release High")),
	m_relMModel(282, 0.1, 1000, 0.001, this, tr("Release Mid")),
	m_relLModel(282, 0.1, 1000, 0.001, this, tr("Release Low")),
	m_rmsTimeModel(10, 0, 500, 0.00001, this, tr("RMS Time")),
	m_kneeModel(6, 0, 36, 0.00001, this, tr("Knee")),
	m_rangeModel(36, 0, 96, 0.00001, this, tr("Range")),
	m_depthScalingModel(true, this, tr("Scale output volume with Depth")),
	m_stereoLinkModel(false, this, tr("Stereo Link")),
	m_autoTimeModel(0, 0, 1, 0.00001, this, tr("Auto Time")),
	m_mixModel(1, 0, 1, 0.00001, this, tr("Mix")),
	m_feedbackModel(false, this, tr("Feedback")),
	m_midsideModel(false, this, tr("Mid/Side")),
	m_lookaheadEnableModel(false, this, tr("Lookahead")),
	m_lookaheadModel(0.f, 0.f, 20.f, 0.01, this, tr("Lookahead"))
{
	m_timeModel.setScaleLogarithmic(true);
	m_aRatioHModel.setScaleLogarithmic(true);
	m_aRatioMModel.setScaleLogarithmic(true);
	m_aRatioLModel.setScaleLogarithmic(true);
	m_bRatioHModel.setScaleLogarithmic(true);
	m_bRatioMModel.setScaleLogarithmic(true);
	m_bRatioLModel.setScaleLogarithmic(true);
	m_atkHModel.setScaleLogarithmic(true);
	m_atkMModel.setScaleLogarithmic(true);
	m_atkLModel.setScaleLogarithmic(true);
	m_relHModel.setScaleLogarithmic(true);
	m_relMModel.setScaleLogarithmic(true);
	m_relLModel.setScaleLogarithmic(true);
	m_rmsTimeModel.setScaleLogarithmic(true);
}


void LOMMControls::resetAllParameters()
{
	m_depthModel.setInitValue(1);
	m_depthModel.reset();
	m_timeModel.setInitValue(1);
	m_timeModel.reset();
	m_inVolModel.setInitValue(0);
	m_inVolModel.reset();
	m_outVolModel.setInitValue(0);
	m_outVolModel.reset();
	m_upwardModel.setInitValue(1);
	m_upwardModel.reset();
	m_downwardModel.setInitValue(1);
	m_downwardModel.reset();
	m_split1Model.setInitValue(2500);
	m_split1Model.reset();
	m_split2Model.setInitValue(88);
	m_split2Model.reset();
	m_split1EnabledModel.setInitValue(true);
	m_split1EnabledModel.reset();
	m_split2EnabledModel.setInitValue(true);
	m_split2EnabledModel.reset();
	m_band1EnabledModel.setInitValue(true);
	m_band1EnabledModel.reset();
	m_band2EnabledModel.setInitValue(true);
	m_band2EnabledModel.reset();
	m_band3EnabledModel.setInitValue(true);
	m_band3EnabledModel.reset();
	m_inHighModel.setInitValue(0);
	m_inHighModel.reset();
	m_inMidModel.setInitValue(0);
	m_inMidModel.reset();
	m_inLowModel.setInitValue(0);
	m_inLowModel.reset();
	m_outHighModel.setInitValue(0);
	m_outHighModel.reset();
	m_outMidModel.setInitValue(0);
	m_outMidModel.reset();
	m_outLowModel.setInitValue(0);
	m_outLowModel.reset();
	m_aThreshHModel.setInitValue(m_aThreshHModel.maxValue());
	m_aThreshHModel.reset();
	m_aThreshMModel.setInitValue(m_aThreshMModel.maxValue());
	m_aThreshMModel.reset();
	m_aThreshLModel.setInitValue(m_aThreshLModel.maxValue());
	m_aThreshLModel.reset();
	m_aRatioHModel.setInitValue(1);
	m_aRatioHModel.reset();
	m_aRatioMModel.setInitValue(1);
	m_aRatioMModel.reset();
	m_aRatioLModel.setInitValue(1);
	m_aRatioLModel.reset();
	m_bThreshHModel.setInitValue(m_bThreshHModel.minValue());
	m_bThreshHModel.reset();
	m_bThreshMModel.setInitValue(m_bThreshMModel.minValue());
	m_bThreshMModel.reset();
	m_bThreshLModel.setInitValue(m_bThreshLModel.minValue());
	m_bThreshLModel.reset();
	m_bRatioHModel.setInitValue(1);
	m_bRatioHModel.reset();
	m_bRatioMModel.setInitValue(1);
	m_bRatioMModel.reset();
	m_bRatioLModel.setInitValue(1);
	m_bRatioLModel.reset();
	m_atkHModel.setInitValue(13.5);
	m_atkHModel.reset();
	m_atkMModel.setInitValue(22.4);
	m_atkMModel.reset();
	m_atkLModel.setInitValue(47.8);
	m_atkLModel.reset();
	m_relHModel.setInitValue(132);
	m_relHModel.reset();
	m_relMModel.setInitValue(282);
	m_relMModel.reset();
	m_relLModel.setInitValue(282);
	m_relLModel.reset();
	m_rmsTimeModel.setInitValue(10);
	m_rmsTimeModel.reset();
	m_kneeModel.setInitValue(6);
	m_kneeModel.reset();
	m_rangeModel.setInitValue(36);
	m_rangeModel.reset();
	m_depthScalingModel.setInitValue(true);
	m_depthScalingModel.reset();
	m_stereoLinkModel.setInitValue(false);
	m_stereoLinkModel.reset();
	m_autoTimeModel.setInitValue(0);
	m_autoTimeModel.reset();
	m_mixModel.setInitValue(1);
	m_mixModel.reset();
	m_feedbackModel.setInitValue(true);
	m_feedbackModel.reset();
	m_midsideModel.setInitValue(false);
	m_midsideModel.reset();
	m_lookaheadEnableModel.setInitValue(false);
	m_lookaheadEnableModel.reset();
	m_lookaheadModel.setInitValue(0.f);
	m_lookaheadModel.reset();
}



void LOMMControls::loadSettings(const QDomElement& parent)
{
	m_depthModel.loadSettings(parent, "depth");
	m_timeModel.loadSettings(parent, "time");
	m_inVolModel.loadSettings(parent, "inVol");
	m_outVolModel.loadSettings(parent, "outVol");
	m_upwardModel.loadSettings(parent, "upward");
	m_downwardModel.loadSettings(parent, "downward");
	m_split1Model.loadSettings(parent, "split1");
	m_split2Model.loadSettings(parent, "split2");
	m_split1EnabledModel.loadSettings(parent, "split1Enabled");
	m_split2EnabledModel.loadSettings(parent, "split2Enabled");
	m_band1EnabledModel.loadSettings(parent, "band1Enabled");
	m_band2EnabledModel.loadSettings(parent, "band2Enabled");
	m_band3EnabledModel.loadSettings(parent, "band3Enabled");
	m_inHighModel.loadSettings(parent, "inHigh");
	m_inMidModel.loadSettings(parent, "inMid");
	m_inLowModel.loadSettings(parent, "inLow");
	m_outHighModel.loadSettings(parent, "outHigh");
	m_outMidModel.loadSettings(parent, "outMid");
	m_outLowModel.loadSettings(parent, "outLow");
	m_aThreshHModel.loadSettings(parent, "aThreshH");
	m_aThreshMModel.loadSettings(parent, "aThreshM");
	m_aThreshLModel.loadSettings(parent, "aThreshL");
	m_aRatioHModel.loadSettings(parent, "aRatioH");
	m_aRatioMModel.loadSettings(parent, "aRatioM");
	m_aRatioLModel.loadSettings(parent, "aRatioL");
	m_bThreshHModel.loadSettings(parent, "bThreshH");
	m_bThreshMModel.loadSettings(parent, "bThreshM");
	m_bThreshLModel.loadSettings(parent, "bThreshL");
	m_bRatioHModel.loadSettings(parent, "bRatioH");
	m_bRatioMModel.loadSettings(parent, "bRatioM");
	m_bRatioLModel.loadSettings(parent, "bRatioL");
	m_atkHModel.loadSettings(parent, "atkH");
	m_atkMModel.loadSettings(parent, "atkM");
	m_atkLModel.loadSettings(parent, "atkL");
	m_relHModel.loadSettings(parent, "relH");
	m_relMModel.loadSettings(parent, "relM");
	m_relLModel.loadSettings(parent, "relL");
	m_rmsTimeModel.loadSettings(parent, "rmsTime");
	m_kneeModel.loadSettings(parent, "knee");
	m_rangeModel.loadSettings(parent, "range");
	m_depthScalingModel.loadSettings(parent, "depthScaling");
	m_stereoLinkModel.loadSettings(parent, "stereoLink");
	m_autoTimeModel.loadSettings(parent, "autoTime");
	m_mixModel.loadSettings(parent, "mix");
	m_feedbackModel.loadSettings(parent, "feedback");
	m_midsideModel.loadSettings(parent, "midside");
	m_lookaheadEnableModel.loadSettings(parent, "lookaheadEnable");
	m_lookaheadModel.loadSettings(parent, "lookahead");
}




void LOMMControls::saveSettings(QDomDocument& doc, QDomElement& parent)
{
	m_depthModel.saveSettings(doc, parent, "depth");
	m_timeModel.saveSettings(doc, parent, "time");
	m_inVolModel.saveSettings(doc, parent, "inVol");
	m_outVolModel.saveSettings(doc, parent, "outVol");
	m_upwardModel.saveSettings(doc, parent, "upward");
	m_downwardModel.saveSettings(doc, parent, "downward");
	m_split1Model.saveSettings(doc, parent, "split1");
	m_split2Model.saveSettings(doc, parent, "split2");
	m_split1EnabledModel.saveSettings(doc, parent, "split1Enabled");
	m_split2EnabledModel.saveSettings(doc, parent, "split2Enabled");
	m_band1EnabledModel.saveSettings(doc, parent, "band1Enabled");
	m_band2EnabledModel.saveSettings(doc, parent, "band2Enabled");
	m_band3EnabledModel.saveSettings(doc, parent, "band3Enabled");
	m_inHighModel.saveSettings(doc, parent, "inHigh");
	m_inMidModel.saveSettings(doc, parent, "inMid");
	m_inLowModel.saveSettings(doc, parent, "inLow");
	m_outHighModel.saveSettings(doc, parent, "outHigh");
	m_outMidModel.saveSettings(doc, parent, "outMid");
	m_outLowModel.saveSettings(doc, parent, "outLow");
	m_aThreshHModel.saveSettings(doc, parent, "aThreshH");
	m_aThreshMModel.saveSettings(doc, parent, "aThreshM");
	m_aThreshLModel.saveSettings(doc, parent, "aThreshL");
	m_aRatioHModel.saveSettings(doc, parent, "aRatioH");
	m_aRatioMModel.saveSettings(doc, parent, "aRatioM");
	m_aRatioLModel.saveSettings(doc, parent, "aRatioL");
	m_bThreshHModel.saveSettings(doc, parent, "bThreshH");
	m_bThreshMModel.saveSettings(doc, parent, "bThreshM");
	m_bThreshLModel.saveSettings(doc, parent, "bThreshL");
	m_bRatioHModel.saveSettings(doc, parent, "bRatioH");
	m_bRatioMModel.saveSettings(doc, parent, "bRatioM");
	m_bRatioLModel.saveSettings(doc, parent, "bRatioL");
	m_atkHModel.saveSettings(doc, parent, "atkH");
	m_atkMModel.saveSettings(doc, parent, "atkM");
	m_atkLModel.saveSettings(doc, parent, "atkL");
	m_relHModel.saveSettings(doc, parent, "relH");
	m_relMModel.saveSettings(doc, parent, "relM");
	m_relLModel.saveSettings(doc, parent, "relL");
	m_rmsTimeModel.saveSettings(doc, parent, "rmsTime");
	m_kneeModel.saveSettings(doc, parent, "knee");
	m_rangeModel.saveSettings(doc, parent, "range");
	m_depthScalingModel.saveSettings(doc, parent, "depthScaling");
	m_stereoLinkModel.saveSettings(doc, parent, "stereoLink");
	m_autoTimeModel.saveSettings(doc, parent, "autoTime");
	m_mixModel.saveSettings(doc, parent, "mix");
	m_feedbackModel.saveSettings(doc, parent, "feedback");
	m_midsideModel.saveSettings(doc, parent, "midside");
	m_lookaheadEnableModel.saveSettings(doc, parent, "lookaheadEnable");
	m_lookaheadModel.saveSettings(doc, parent, "lookahead");
}


} // namespace lmms


