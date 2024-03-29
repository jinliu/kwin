/*
    SPDX-FileCopyrightText: 2021 Vlad Zahorodnii <vlad.zahorodnii@kde.org>

    SPDX-License-Identifier: GPL-2.0-or-later
*/

#pragma once

#include <QByteArray>
#include <QList>
#include <QString>

namespace KWin
{

class XwaylandSocket
{
public:
    enum class OperationMode {
        CloseFdsOnExec,
        TransferFdsOnExec
    };

    XwaylandSocket(OperationMode operationMode);
    ~XwaylandSocket();

    bool isValid() const;
    int display() const;
    QString name() const;

    QList<int> fileDescriptors() const;

private:
    QList<int> m_fileDescriptors;
    int m_display = -1;
    QString m_socketFilePath;
    QString m_lockFilePath;
};

} // namespace KWin
