/*
    SPDX-FileCopyrightText: 2021-2022 Xaver Hugl <xaver.hugl@gmail.com>

    SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL
 */
#pragma once

#include <QHash>
#include <QObject>
#include <map>
#include <memory>

namespace KWin
{

class DrmBackend;
class DrmGpu;
class DrmLeaseDeviceV1Interface;
class DrmLeaseConnectorV1Interface;
class Display;

class DrmLeaseManagerV1 : public QObject
{
    Q_OBJECT
public:
    DrmLeaseManagerV1(DrmBackend *backend, Display *display, QObject *parent = nullptr);
    ~DrmLeaseManagerV1();

private:
    void addGpu(DrmGpu *gpu);
    void removeGpu(DrmGpu *gpu);
    void handleOutputsQueried();

    DrmBackend *const m_backend;
    Display *const m_display;
    QHash<DrmGpu *, DrmLeaseDeviceV1Interface *> m_leaseDevices;
};
}
