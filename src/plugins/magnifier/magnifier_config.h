/*
    KWin - the KDE window manager
    This file is part of the KDE project.

    SPDX-FileCopyrightText: 2007 Christian Nitschkowski <christian.nitschkowski@kdemail.net>

    SPDX-License-Identifier: GPL-2.0-or-later
*/

#pragma once

#include <kcmodule.h>

#include "ui_magnifier_config.h"

class KActionCollection;

namespace KWin
{

class MagnifierEffectConfig : public KCModule
{
    Q_OBJECT
public:
    explicit MagnifierEffectConfig(QObject *parent, const KPluginMetaData &data);

    void save() override;
    void defaults() override;

private:
    Ui::MagnifierEffectConfigForm m_ui;
    KActionCollection *m_actionCollection;
};

} // namespace
