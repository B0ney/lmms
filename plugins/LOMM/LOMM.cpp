/*
 * LOMM.cpp
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

#include "LOMM.h"

#include "embed.h"
#include "plugin_export.h"













#include <iostream>
#include <ostream>






















/*


DO NOT FORGET ABOUT THAT BUG, it is not gone yet!


*/

























namespace lmms
{

extern "C"
{

Plugin::Descriptor PLUGIN_EXPORT lomm_plugin_descriptor =
{
	LMMS_STRINGIFY(PLUGIN_NAME),
	"LOMM",
	QT_TRANSLATE_NOOP("PluginBrowser", "LOMM Gaming"),
	"Lost Robot <r94231/at/gmail/dot/com>",
	0x0100,
	Plugin::Effect,
	new PluginPixmapLoader("logo"),
	nullptr,
	nullptr
};

}


LOMMEffect::LOMMEffect(Model* parent, const Descriptor::SubPluginFeatures::Key* key) :
	Effect(&lomm_plugin_descriptor, parent, key),
	m_lommControls(this),
	m_sampleRate(Engine::audioEngine()->processingSampleRate()),
	m_lp1(m_sampleRate),
	m_lp2(m_sampleRate),
	m_hp1(m_sampleRate),
	m_hp2(m_sampleRate)
{
	m_yL[0][0] = m_yL[0][1] = LOMM_MIN_FLOOR;
	m_yL[1][0] = m_yL[1][1] = LOMM_MIN_FLOOR;
	m_yL[2][0] = m_yL[2][1] = LOMM_MIN_FLOOR;
	
	connect(Engine::audioEngine(), SIGNAL(sampleRateChanged()), this, SLOT(changeSampleRate()));
	emit changeSampleRate();
}

void LOMMEffect::changeSampleRate()
{
	m_sampleRate = Engine::audioEngine()->processingSampleRate();
	m_lp1.setSampleRate(m_sampleRate);
	m_lp2.setSampleRate(m_sampleRate);
	m_hp1.setSampleRate(m_sampleRate);
	m_hp2.setSampleRate(m_sampleRate);
	
	m_coeffPrecalc = -2.2f / (m_sampleRate * 0.001f);
	m_needsUpdate = true;
	
	m_crestTimeConst = exp(-1.f / (0.2f * m_sampleRate));
	
	m_lookBufLength = std::ceil((20.f / 1000.f) * m_sampleRate) + 2;
	for (int i = 0; i < 2; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			m_inLookBuf[j][i].resize(m_lookBufLength);
			m_scLookBuf[j][i].resize(m_lookBufLength);
		}
	}
}

void LOMMEffect::clearFilterHistories()
{
	m_lp1.clearHistory();
	m_lp2.clearHistory();
	m_hp1.clearHistory();
	m_hp2.clearHistory();
}




bool LOMMEffect::processAudioBuffer(sampleFrame* buf, const fpp_t frames)
{
	if (!isEnabled() || !isRunning())
	{
		return false;
	}
	
	if (m_needsUpdate || m_lommControls.m_split1Model.isValueChanged())
	{
		m_lp1.setLowpass(m_lommControls.m_split1Model.value());
		m_hp1.setHighpass(m_lommControls.m_split1Model.value());
	}
	if (m_needsUpdate || m_lommControls.m_split2Model.isValueChanged())
	{
		m_lp2.setLowpass(m_lommControls.m_split2Model.value());
		m_hp2.setHighpass(m_lommControls.m_split2Model.value());
	}
	
	m_needsUpdate = false;

	double outSum = 0.0;
	const float d = dryLevel();
	const float w = wetLevel();
	
	const float depth = m_lommControls.m_depthModel.value();
	const float time = m_lommControls.m_timeModel.value();
	const float inVol = dbfsToAmp(m_lommControls.m_inVolModel.value());
	const float outVol = dbfsToAmp(m_lommControls.m_outVolModel.value());
	const float upward = m_lommControls.m_upwardModel.value();
	const float downward = m_lommControls.m_downwardModel.value();
	const float split1 = m_lommControls.m_split1Model.value();
	const float split2 = m_lommControls.m_split2Model.value();
	const bool split1Enabled = m_lommControls.m_split1EnabledModel.value();
	const bool split2Enabled = m_lommControls.m_split2EnabledModel.value();
	const bool band1Enabled = m_lommControls.m_band1EnabledModel.value();
	const bool band2Enabled = m_lommControls.m_band2EnabledModel.value();
	const bool band3Enabled = m_lommControls.m_band3EnabledModel.value();
	const float inHigh = dbfsToAmp(m_lommControls.m_inHighModel.value());
	const float inMid = dbfsToAmp(m_lommControls.m_inMidModel.value());
	const float inLow = dbfsToAmp(m_lommControls.m_inLowModel.value());
	float inBandVol[3] = {inHigh, inMid, inLow};
	const float outHigh = dbfsToAmp(m_lommControls.m_outHighModel.value());
	const float outMid = dbfsToAmp(m_lommControls.m_outMidModel.value());
	const float outLow = dbfsToAmp(m_lommControls.m_outLowModel.value());
	float outBandVol[3] = {outHigh, outMid, outLow};
	const float aThreshH = m_lommControls.m_aThreshHModel.value();
	const float aThreshM = m_lommControls.m_aThreshMModel.value();
	const float aThreshL = m_lommControls.m_aThreshLModel.value();
	float aThresh[3] = {aThreshH, aThreshM, aThreshL};
	const float aRatioH = m_lommControls.m_aRatioHModel.value();
	const float aRatioM = m_lommControls.m_aRatioMModel.value();
	const float aRatioL = m_lommControls.m_aRatioLModel.value();
	float aRatio[3] = {1.f / aRatioH, 1.f / aRatioM, 1.f / aRatioL};
	const float bThreshH = m_lommControls.m_bThreshHModel.value();
	const float bThreshM = m_lommControls.m_bThreshMModel.value();
	const float bThreshL = m_lommControls.m_bThreshLModel.value();
	float bThresh[3] = {bThreshH, bThreshM, bThreshL};
	const float bRatioH = m_lommControls.m_bRatioHModel.value();
	const float bRatioM = m_lommControls.m_bRatioMModel.value();
	const float bRatioL = m_lommControls.m_bRatioLModel.value();
	float bRatio[3] = {1.f / bRatioH, 1.f / bRatioM, 1.f / bRatioL};
	const float atkH = m_lommControls.m_atkHModel.value() * time;
	const float atkM = m_lommControls.m_atkMModel.value() * time;
	const float atkL = m_lommControls.m_atkLModel.value() * time;
	const float atkCoefH = msToCoeff(atkH);
	const float atkCoefM = msToCoeff(atkM);
	const float atkCoefL = msToCoeff(atkL);
	float atk[3] = {atkH, atkM, atkL};
	float atkCoef[3] = {atkCoefH, atkCoefM, atkCoefL};
	const float relH = m_lommControls.m_relHModel.value() * time;
	const float relM = m_lommControls.m_relMModel.value() * time;
	const float relL = m_lommControls.m_relLModel.value() * time;
	const float relCoefH = msToCoeff(relH);
	const float relCoefM = msToCoeff(relM);
	const float relCoefL = msToCoeff(relL);
	float rel[3] = {relH, relM, relL};
	float relCoef[3] = {relCoefH, relCoefM, relCoefL};
	const float rmsTime = m_lommControls.m_rmsTimeModel.value();
	if (rmsTime > 0)
	{
		m_rmsTimeConst = exp(-1.f / (rmsTime * 0.001f * m_sampleRate));
	}
	const float knee = m_lommControls.m_kneeModel.value() * 0.5f;
	const float range = m_lommControls.m_rangeModel.value();
	const double rangeAmp = dbfsToAmp(range);
	const bool depthScaling = m_lommControls.m_depthScalingModel.value();
	const bool stereoLink = m_lommControls.m_stereoLinkModel.value();
	const float autoTime = m_lommControls.m_autoTimeModel.value();
	const float mix = m_lommControls.m_mixModel.value();
	const bool feedback = m_lommControls.m_feedbackModel.value();
	const bool midside = m_lommControls.m_midsideModel.value();
	const bool lookaheadEnable = m_lommControls.m_lookaheadEnableModel.value();
	const int lookahead = std::ceil((m_lommControls.m_lookaheadModel.value() / 1000.f) * m_sampleRate);
	
	
	for (fpp_t f = 0; f < frames; ++f)
	{
		std::array<sample_t, 2> s = {buf[f][0], buf[f][1]};
		
		if (midside)
		{
			float tempS0 = s[0];
			s[0] = (s[0] + s[1]) * 0.5f;
			s[1] = s[1] - tempS0;
		}
		
		float bands[3][2] = {{}};
		float bandsDry[3][2] = {{}};
		
		for (int i = 0; i < 2; ++i)
		{
			s[i] *= inVol;
			
			m_crestPeakVal[i] = qMax(s[i] * s[i], m_crestTimeConst * m_crestPeakVal[i] + (1 - m_crestTimeConst) * (s[i] * s[i]));
			m_crestRmsVal[i] = m_crestTimeConst * m_crestRmsVal[i] + ((1 - m_crestTimeConst) * (s[i] * s[i]));
			m_crestFactorVal[i] = m_crestPeakVal[i] / m_crestRmsVal[i];
			float crestFactorValTemp = ((m_crestFactorVal[i] - 5.f) * autoTime) + 5.f;
		
			bands[0][i] = m_hp1.update(s[i], i);
			bands[1][i] = m_hp2.update(m_lp1.update(s[i], i), i);
			bands[2][i] = m_lp2.update(s[i], i);
			
			if (!split1Enabled)
			{
				bands[1][i] += bands[0][i];
				bands[0][i] = 0;
			}
			if (!split2Enabled)
			{
				bands[1][i] += bands[2][i];
				bands[2][i] = 0;
			}
			
			bands[0][i] *= band1Enabled;
			bands[1][i] *= band2Enabled;
			bands[2][i] *= band3Enabled;
			
			float detect[3] = {};
			for (int j = 0; j < 3; ++j)
			{
				if (feedback && !lookaheadEnable)
				{
					bands[j][i] = m_prevOut[j][i];
				}
				
				bandsDry[j][i] = bands[j][i];
				
				bands[j][i] *= inBandVol[j];
				
				
				float peakVal = qMax(LOMM_MIN_FLOOR, std::abs(bands[j][i]));
				if (rmsTime > 0)
				{
					m_rms[j][i] = m_rmsTimeConst * m_rms[j][i] + ((1 - m_rmsTimeConst) * (bands[j][i] * bands[j][i]));
					detect[j] = qMax(LOMM_MIN_FLOOR, std::sqrt(m_rms[j][i]));
				}
				else
				{
					detect[j] = peakVal;
				}
				
				// Reduce clicks and explosions
				if (peakVal / m_yL[j][i] > rangeAmp)
				{
					m_yL[j][i] = peakVal / rangeAmp;
				}
				
				
				if (detect[j] > m_yL[j][i])// Attack phase
				{
					// Calculate attack value depending on crest factor
					const float currentAttack = autoTime
						? msToCoeff(5.f * atk[j] / crestFactorValTemp)
						: atkCoef[j];
					
					m_yL[j][i] = m_yL[j][i] * currentAttack + (1 - currentAttack) * detect[j];
				}
				else// Release phase
				{
					// Calculate attack value depending on crest factor
					const float currentRelease = autoTime
						? msToCoeff(5.f * rel[j] / crestFactorValTemp)
						: relCoef[j];
					
					m_yL[j][i] = m_yL[j][i] * currentRelease + (1 - currentRelease) * detect[j];
				}
				
				m_yL[j][i] = qMax(LOMM_MIN_FLOOR, m_yL[j][i]);
				
				float yAmp = m_yL[j][i];
				if (lookaheadEnable)
				{
					float temp = yAmp;
					yAmp = qMax(m_scLookBuf[j][i][m_lookWrite], m_scLookBuf[j][i][(m_lookWrite + m_lookBufLength - lookahead) % m_lookBufLength]);
					m_scLookBuf[j][i][m_lookWrite] = temp;
				}
				
				const float yDbfs = ampToDbfs(yAmp);
				
				float aboveGain = 0;
				float belowGain = 0;
				if (yDbfs - aThresh[j] < -knee)// Below knee
				{
					aboveGain = yDbfs;
				}
				else if (yDbfs - aThresh[j] < knee)// Within knee
				{
					const float temp = yDbfs - aThresh[j] + knee;
					aboveGain = yDbfs + (aRatio[j] - 1) * temp * temp / (4 * knee);
				}
				else// Above knee
				{
					aboveGain = aThresh[j] + (yDbfs - aThresh[j]) * aRatio[j];
				}
				if (aboveGain < yDbfs)
				{
					if (downward * depth <= 1)
					{
						aboveGain = linearInterpolate(yDbfs, aboveGain, downward * depth);
					}
					else
					{
						aboveGain = linearInterpolate(aboveGain, aThresh[j], downward * depth - 1);
					}
				}
				
				if (yDbfs - bThresh[j] > knee)// Above knee
				{
					belowGain = yDbfs;
				}
				else if (bThresh[j] - yDbfs < knee)// Within knee
				{
					const float temp = bThresh[j] - yDbfs + knee;
					belowGain = yDbfs + (1 - bRatio[j]) * temp * temp / (4 * knee);
				}
				else// Below knee
				{
					belowGain = bThresh[j] + (yDbfs - bThresh[j]) * bRatio[j];
				}
				if (belowGain > yDbfs)
				{
					if (upward * depth <= 1)
					{
						belowGain = linearInterpolate(yDbfs, belowGain, upward * depth);
					}
					else
					{
						belowGain = linearInterpolate(belowGain, bThresh[j], upward * depth - 1);
					}
				}
				
				m_displayIn[j][i] = yDbfs;
				m_gainResult[j][i] = (dbfsToAmp(aboveGain) / yAmp) * (dbfsToAmp(belowGain) / yAmp);
				/*if (m_gainResult[j][i] >= rangeAmp)
				{
					float ampAdjust = m_gainResult[j][i] / rangeAmp;
					m_yL[j][i] *= ampAdjust;
					m_gainResult[j][i] = rangeAmp;
				}*/
				m_gainResult[j][i] = qMin(m_gainResult[j][i], rangeAmp);
				m_displayOut[j][i] = ampToDbfs(yAmp * m_gainResult[j][i]);
				
				if (stereoLink && i == 1)
				{
					if (m_gainResult[j][1] < m_gainResult[j][0])
					{
						m_gainResult[j][0] = m_gainResult[j][1];
						m_displayOut[j][0] = m_displayIn[j][0] - (m_displayIn[j][1] - m_displayOut[j][1]);
					}
					else
					{
						m_gainResult[j][1] = m_gainResult[j][0];
						m_displayOut[j][1] = m_displayIn[j][1] - (m_displayIn[j][0] - m_displayOut[j][0]);
					}
				}
			}
		}
		
		for (int i = 0; i < 2; ++i)
		{
			if (lookaheadEnable)
			{
				for (int j = 0; j < 3; ++j)
				{
					float temp = bands[j][i];
					bands[j][i] = m_inLookBuf[j][i][m_lookWrite];
					m_inLookBuf[j][i][m_lookWrite] = temp;
				}
			}
			else if (feedback)
			{
				bands[0][i] = bandsDry[0][i];
				bands[1][i] = bandsDry[1][i];
				bands[2][i] = bandsDry[2][i];
			}
			
			bands[0][i] *= m_gainResult[0][i];
			bands[1][i] *= m_gainResult[1][i];
			bands[2][i] *= m_gainResult[2][i];
			
			bands[0][i] *= outBandVol[0];
			bands[1][i] *= outBandVol[1];
			bands[2][i] *= outBandVol[2];
			
			m_prevOut[0][i] = bands[0][i];
			m_prevOut[1][i] = bands[1][i];
			m_prevOut[2][i] = bands[2][i];
			
			bands[0][i] = linearInterpolate(bandsDry[0][i], bands[0][i], mix);
			bands[1][i] = linearInterpolate(bandsDry[1][i], bands[1][i], mix);
			bands[2][i] = linearInterpolate(bandsDry[2][i], bands[2][i], mix);
			
			s[i] = bands[0][i] + bands[1][i] + bands[2][i];
			
			s[i] *= linearInterpolate(1.f, outVol, mix * (depthScaling ? depth : 1));
		}
		
		if (midside)
		{
			float tempS0 = s[0];
			s[0] = s[0] + s[1];
			s[1] = tempS0 - s[1];
		}
		
		if (--m_lookWrite < 0) {m_lookWrite = m_lookBufLength - 1;}

		buf[f][0] = d * buf[f][0] + w * s[0];
		buf[f][1] = d * buf[f][1] + w * s[1];
		outSum += buf[f][0] * buf[f][0] + buf[f][1] * buf[f][1];
	}

	checkGate(outSum / frames);
	return isRunning();
}


extern "C"
{

// necessary for getting instance out of shared lib
PLUGIN_EXPORT Plugin * lmms_plugin_main(Model* parent, void* data)
{
	return new LOMMEffect(parent, static_cast<const Plugin::Descriptor::SubPluginFeatures::Key *>(data));
}

}

} // namespace lmms
