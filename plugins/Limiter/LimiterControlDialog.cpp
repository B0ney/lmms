/*
 * LimiterControlDialog.cpp
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

#include "LimiterControlDialog.h"
#include "LimiterControls.h"
#include "Limiter.h"
#include "embed.h"
//#include "BarModelEditor.h"
#include "Knob.h"
#include "LcdSpinBox.h"
#include "GuiApplication.h"
#include "gui_templates.h"
#include "MainWindow.h"

namespace lmms::gui
{

LimiterControlDialog::LimiterControlDialog(LimiterControls* controls) :
	EffectControlDialog(controls),
	m_controls(controls),
	m_mainFont("Arial", 18),
	m_pixmap(1, 1),
	m_scrollerPixmap(1, 1)
{
	setAutoFillBackground(false);
	setAttribute(Qt::WA_OpaquePaintEvent, true);
	setAttribute(Qt::WA_NoSystemBackground, true);
	
	setFixedSize(700, 400);
	m_pixmap = m_pixmap.scaled(size());
	m_scrollerPixmap = m_scrollerPixmap.scaled(LIMITER_WAVEGRAPH_WIDTH, LIMITER_WAVEGRAPH_HEIGHT);
	
	auto makeKnob = [this](int x, int y, const QString& label, const QString& hintText, const QString& unit, FloatModel* model)
	{
        Knob* newKnob = new Knob(KnobType::Bright26, this);
        newKnob->move(x, y);
        newKnob->setModel(model);
        newKnob->setLabel(label);
        newKnob->setHintText(hintText, unit);
        return newKnob;
    };

	makeKnob(157, 65, tr("LOOKAHEAD"), tr("Lookahead:"), " ms", &controls->m_lookaheadModel);
	makeKnob(227, 155, tr("LOOKREL"), tr("Look Release:"), " ms", &controls->m_lookReleaseModel);
	makeKnob(227, 55, tr("LOOKREL STAGES"), tr("Look Release Stages:"), "", &controls->m_lookReleaseStagesModel);
	makeKnob(87, 85, tr("ADAPT"), tr("Adaptive Release:"), "", &controls->m_adaptReleaseModel);
	makeKnob(137, 115, tr("ADAPT SLOW"), tr("Adaptive Release Slowing:"), "", &controls->m_adaptSlowModel);
	makeKnob(37, 165, tr("LINK TRANSIENT"), tr("Stereo Link Transients:"), "", &controls->m_stereoLinkTransientModel);
	makeKnob(87, 165, tr("LINK RELEASE"), tr("Stereo Link Release:"), "", &controls->m_stereoLinkReleaseModel);
	makeKnob(27, 55, tr("DC FREQ"), tr("DC Offset Removal Frequency:"), " Hz", &controls->m_dcFreqModel);
	makeKnob(127, 55, tr("TRUE PEAK"), tr("True Peak:"), "", &controls->m_truePeakModel);
	makeKnob(27, 5, tr("THRESH"), tr("Threshold"), " dBFS", &controls->m_thresholdModel);
	makeKnob(127, 5, tr("IN"), tr("Input Gain"), " dB", &controls->m_inGainModel);
	makeKnob(167, 5, tr("OUT"), tr("Output Gain"), " dB", &controls->m_outGainModel);
	makeKnob(227, 5, tr("HOLD"), tr("Hold"), " ms", &controls->m_holdModel);
	makeKnob(227, 105, tr("OVERSAMPLE"), tr("Oversample"), " stages", &controls->m_oversampleModel);
	makeKnob(327, 5, tr("DITHER"), tr("Dither Enabled"), "", &controls->m_ditherEnabledModel);
	makeKnob(357, 25, tr("SMOOTH"), tr("Smoothing Stages"), " stages", &controls->m_smoothStagesModel);
	makeKnob(357, 85, tr("CLEANCLIP"), tr("Clean Clip Amount"), "", &controls->m_cleanClipAmountModel);
	makeKnob(357, 125, tr("CEILING"), tr("Clean Clip Ceiling"), " dBFS", &controls->m_cleanClipCeilingModel);
	makeKnob(357, 165, tr("START"), tr("Clean Clip Start"), " dBFS", &controls->m_cleanClipStartModel);
	makeKnob(427, 55, tr("KWEIGHT CREST"), tr("K-Weight Crest Factor Measurement:"), "", &controls->m_kWeightCrestModel);
	
	LcdSpinBox* ditherBitDepthBox = new LcdSpinBox(2, this, "Dithering Bit Depth");
	ditherBitDepthBox->setModel(&controls->m_ditherBitDepthModel);
	ditherBitDepthBox->move(327, 35);
	ditherBitDepthBox->setLabel(tr("BIT DEPTH"));
	ditherBitDepthBox->setToolTip(tr("Dithering Bit Depth"));
	
	ComboBox* ditherTypeBox = new ComboBox(this);
	ditherTypeBox->setGeometry(327, 65, 60, 22);
	ditherTypeBox->setFont(adjustedToPixelSize(ditherTypeBox->font(), 10)); // 8
	//ditherTypeBox->move(327, 65);
	ditherTypeBox->setModel(&m_controls->m_ditherTypeModel);
	
	connect(getGUI()->mainWindow(), SIGNAL(periodicUpdate()), this, SLOT(updateDisplay()));
	
	m_p.begin(&m_scrollerPixmap);
	m_p.fillRect(rect(), QColor(9, 9, 9, 255));
	m_p.end();
	
	m_dH = m_controls->m_effect->displayHandler();
}

void LimiterControlDialog::updateDisplay()
{
	if (!isVisible())
	{
		return;
	}
	
	const float momentary = m_controls->m_effect->getMomentary();
	const float shortTerm = m_controls->m_effect->getShortTerm();
	const float integrated = m_controls->m_effect->getIntegrated();
	
	float releaseDivide[2] = {m_controls->m_effect->getReleaseDivide(0), m_controls->m_effect->getReleaseDivide(1)};
	float crestFactor[2] = {m_controls->m_effect->getCrestFactor(0), m_controls->m_effect->getCrestFactor(1)};
	float dcOffset[2] = {m_controls->m_effect->getDcOffset(0), m_controls->m_effect->getDcOffset(1)};
	
	int theirIndex = m_dH->m_index;
	int ourDifference = theirIndex - m_dhIndex;
	if (ourDifference < 0) {ourDifference += LIMITER_DISPLAY_BUFSIZE;}
	m_scrollerPixmap.scroll(-ourDifference, 0, m_scrollerPixmap.rect());
	m_p.begin(&m_scrollerPixmap);
	m_p.setRenderHint(QPainter::Antialiasing, false);
	m_p.fillRect(LIMITER_WAVEGRAPH_WIDTH - ourDifference, 0, ourDifference, LIMITER_WAVEGRAPH_HEIGHT, QColor(9, 9, 9, 255));
	for (int i = 0; i < ourDifference; ++i)
	{
		float inVolPeak = (m_dH->m_inPeakLBuf[m_dhIndex] + m_dH->m_inPeakRBuf[m_dhIndex]) * 0.5f;
		float outVolPeak = (m_dH->m_outPeakLBuf[m_dhIndex] + m_dH->m_outPeakRBuf[m_dhIndex]) * 0.5f;
		
		m_p.fillRect(LIMITER_WAVEGRAPH_WIDTH - i - 1, LIMITER_WAVEGRAPH_HEIGHT, 1, -inVolPeak * LIMITER_WAVEGRAPH_0DBFS_HEIGHT, QColor(94, 94, 94, 255));
		m_p.fillRect(LIMITER_WAVEGRAPH_WIDTH - i - 1, LIMITER_WAVEGRAPH_HEIGHT, 1, -outVolPeak * LIMITER_WAVEGRAPH_0DBFS_HEIGHT, QColor(194, 194, 194, 255));
		
		m_dhIndex = (m_dhIndex + 1) % LIMITER_DISPLAY_BUFSIZE;
	}
	m_p.end();
	
	/*m_scrollerPixmap.scroll(-4, 0, m_scrollerPixmap.rect());
    m_p.begin(&m_scrollerPixmap);
    m_p.setRenderHint(QPainter::Antialiasing, false);
    m_p.fillRect(LIMITER_WAVEGRAPH_WIDTH - 4, 0, 4, 150, QColor(9, 9, 9, 255));
    const float volStart = 75 - inVolMax[0] * 75.f;
    const float volEnd = 75 - inVolMin[0] * 75.f;
    m_p.fillRect(LIMITER_WAVEGRAPH_WIDTH - 2, LIMITER_WAVEGRAPH_HEIGHT, 2, -(safeAmpToDbfs(inVol[0]) + 32) * ((float)LIMITER_WAVEGRAPH_HEIGHT / 32.f), QColor(194, 194, 194, 255));
    //m_p.fillRect(296, 0, 2, 100 - gainChange[0] * 100.f, QColor(40, 120, 40, 255));
    m_p.end();*/
	
	m_pixmap.fill(Qt::transparent);
	m_p.begin(&m_pixmap);
	m_p.setRenderHint(QPainter::Antialiasing, false);
    
	m_p.setFont(m_mainFont);
	m_p.setPen(QPen(QColor(220, 220, 220), 1));
	m_p.drawText(QRectF(100, 250, 300, 100), Qt::AlignCenter, QString::number(momentary));
	m_p.drawText(QRectF(100, 275, 300, 100), Qt::AlignCenter, QString::number(shortTerm));
	m_p.drawText(QRectF(100, 300, 300, 100), Qt::AlignCenter, QString::number(integrated));
	
	m_p.fillRect(250, 210, std::log2(releaseDivide[0]) * 100.f, 12, QColor(127, 63, 255, 127));
	m_p.fillRect(250, 210, std::log2(releaseDivide[1]) * 100.f, 12, QColor(255, 63, 127, 127));
	m_p.fillRect(250, 230, m_controls->m_effect->getLinkAmount() * 100.f, 12, QColor(63, 255, 127, 200));
	m_p.fillRect(250, 250, crestFactor[0] * 25.f, 12, QColor(200, 100, 200, 127));
	m_p.fillRect(250, 250, crestFactor[1] * 25.f, 12, QColor(255, 127, 63, 127));
	m_p.fillRect(70, 280 + dcOffset[0] * 100.f, 18, 2, QColor(126, 254, 0, 127));
	m_p.fillRect(70, 280 + dcOffset[1] * 100.f, 18, 2, QColor(254, 126, 0, 127));
	
	m_p.fillRect(650, 380, 8, -(momentary + 72) * 3.f, QColor(237, 112, 159, 255));
	m_p.fillRect(660, 380, 8, -(shortTerm + 72) * 3.f, QColor(206, 148, 233, 255));
	m_p.fillRect(670, 380, 8, -(integrated + 72) * 3.f, QColor(149, 148, 234, 255));
	
	/*m_p.fillRect(500, 380, 12, -(safeAmpToDbfs(inVol[0]) + 72) * 3.f, QColor(255, 255, 63, 255));
	m_p.fillRect(512, 380, 12, -(safeAmpToDbfs(inVol[1]) + 72) * 3.f, QColor(255, 127, 63, 255));
	m_p.fillRect(530, 380, 12, -(safeAmpToDbfs(outVol[0]) + 72) * 3.f, QColor(63, 255, 255, 255));
	m_p.fillRect(542, 380, 12, -(safeAmpToDbfs(outVol[1]) + 72) * 3.f, QColor(63, 127, 255, 255));
	m_p.fillRect(50, 210, 12, safeAmpToDbfs(gainChange[0]) * -10.f, QColor(63, 127, 255, 127));
	m_p.fillRect(50, 210, 12, safeAmpToDbfs(gainChange[1]) * -10.f, QColor(127, 63, 255, 127));*/
	
    m_p.end();
    
	update();
}

void LimiterControlDialog::paintEvent(QPaintEvent* event)
{
    if (!isVisible())
    {
        return;
    }
    
    EffectControlDialog::paintEvent(event);
    
    m_p.begin(this);
    m_p.fillRect(rect(), QColor("black"));
    m_p.drawPixmap(150, 400 - LIMITER_WAVEGRAPH_HEIGHT, m_scrollerPixmap);
    m_p.drawPixmap(0, 0, m_pixmap);
    m_p.end();
}

} // namespace lmms::gui
