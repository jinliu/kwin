#SPDX-FileCopyrightText: 2020 Aleix Pol Gonzalzez <aleixpol@kde.org>
#SPDX-License-Identifier: BSD-3-Clause

add_definitions(-DTRANSLATION_DOMAIN=\"kcm_virtualkeyboard\")

kcmutils_generate_module_data(
    kcm_virtualkeyboard_PART_SRCS
    MODULE_DATA_HEADER virtualkeyboarddata.h
    MODULE_DATA_CLASS_NAME VirtualKeyboardData
    SETTINGS_HEADERS virtualkeyboardsettings.h
    SETTINGS_CLASSES VirtualKeyboardSettings
)

kconfig_add_kcfg_files(kcm_virtualkeyboard_PART_SRCS virtualkeyboardsettings.kcfgc GENERATE_MOC)
kcmutils_add_qml_kcm(kcm_virtualkeyboard SOURCES kcmvirtualkeyboard.cpp ${kcm_virtualkeyboard_PART_SRCS})

target_link_libraries(kcm_virtualkeyboard PRIVATE
    KF6::I18n
    KF6::KCMUtils
    KF6::Service
    KF6::KCMUtilsQuick
)
