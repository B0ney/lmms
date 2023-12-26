/*
 * LimiterControls.h
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

#ifndef LIMITER_CONTROLS_H
#define LIMITER_CONTROLS_H

#include "EffectControls.h"
#include "LimiterControlDialog.h"

#include "ComboBox.h"

constexpr inline float LIMITER_MAX_LOOKAHEAD = 10.f;
constexpr inline float LIMITER_MAX_HOLD = 500.f;
constexpr inline int LIMITER_MAX_RELEASE_STAGES = 6;
constexpr inline float LIMITER_MIN_FLOOR = 0.00012589;// -72 dBFS
constexpr inline int LIMITER_TRUE_PEAK_LATENCY = 6;

constexpr inline int LIMITER_DISPLAY_BUFSIZE = 128;

namespace lmms
{

class LimiterEffect;

namespace gui
{
class LimiterControlDialog;
}

class LimiterControls : public EffectControls
{
	Q_OBJECT
public:
	LimiterControls(LimiterEffect* effect);
	~LimiterControls() override = default;

	void saveSettings(QDomDocument& doc, QDomElement& parent) override;
	void loadSettings(const QDomElement& parent) override;
	inline QString nodeName() const override
	{
		return "LimiterControls";
	}
	gui::EffectControlDialog* createView() override
	{
		return new gui::LimiterControlDialog(this);
	}
	int controlCount() override { return 4; }

private:
	LimiterEffect* m_effect;
	FloatModel m_lookaheadModel;
	FloatModel m_lookReleaseModel;
	FloatModel m_lookReleaseStagesModel;
	FloatModel m_adaptReleaseModel;
	FloatModel m_adaptSlowModel;
	FloatModel m_stereoLinkTransientModel;
	FloatModel m_stereoLinkReleaseModel;
	FloatModel m_dcFreqModel;
	FloatModel m_truePeakModel;
	FloatModel m_thresholdModel;
	FloatModel m_inGainModel;
	FloatModel m_outGainModel;
	FloatModel m_holdModel;
	FloatModel m_oversampleModel;
	FloatModel m_ditherEnabledModel;
	IntModel m_ditherBitDepthModel;
	ComboBoxModel m_ditherTypeModel;
	FloatModel m_smoothStagesModel;
	FloatModel m_cleanClipAmountModel;
	FloatModel m_cleanClipCeilingModel;
	FloatModel m_cleanClipStartModel;
	FloatModel m_kWeightCrestModel;

	friend class gui::LimiterControlDialog;
	friend class LimiterEffect;
};

} // namespace lmms

#endif
