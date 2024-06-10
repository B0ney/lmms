/*
 * GranularPitchShifterControls.h - controls for bassboosterx -effect
 *
 * Copyright (c) 2014 Vesa Kivimäki <contact/dot/diizy/at/nbl/dot/fi>
 * Copyright (c) 2008-2014 Tobias Doerffel <tobydox/at/users.sourceforge.net>
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

#ifndef GRANULARPITCHSHIFTER_CONTROLS_H
#define GRANULARPITCHSHIFTER_CONTROLS_H

#include "EffectControls.h"
#include "GranularPitchShifterControlDialog.h"

namespace lmms
{

class GranularPitchShifterEffect;

namespace gui
{
class GranularPitchShifterControlDialog;
}

class GranularPitchShifterControls : public EffectControls
{
	Q_OBJECT
public:
	GranularPitchShifterControls(GranularPitchShifterEffect* effect);
	~GranularPitchShifterControls() override = default;

	void saveSettings(QDomDocument& doc, QDomElement& parent) override;
	void loadSettings(const QDomElement& parent) override;
	inline QString nodeName() const override
	{
		return "GranularPitchShifterControls";
	}
	gui::EffectControlDialog* createView() override
	{
		return new gui::GranularPitchShifterControlDialog(this);
	}
	int controlCount() override { return 4; }

private:
	GranularPitchShifterEffect* m_effect;
	FloatModel m_pitchModel;
	FloatModel m_pitchSpreadModel;
	FloatModel m_sizeModel;
	FloatModel m_shapeModel;
	FloatModel m_jitterModel;
	FloatModel m_posrandModel;
	FloatModel m_posrandSpreadModel;
	FloatModel m_prefilterModel;
	FloatModel m_densityModel;
	FloatModel m_glideModel;

	friend class gui::GranularPitchShifterControlDialog;
	friend class GranularPitchShifterEffect;
};

} // namespace lmms

#endif
