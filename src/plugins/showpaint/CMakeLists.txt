#######################################
# Effect

set(showpaint_SOURCES
    main.cpp
    showpaint.cpp
)

kwin_add_builtin_effect(showpaint ${showpaint_SOURCES})
target_link_libraries(showpaint PRIVATE
    kwin

    KF6::GlobalAccel
    KF6::I18n
)

#######################################
# Config
if (KWIN_BUILD_KCMS)
    set(kwin_showpaint_config_SRCS showpaint_config.cpp)
    ki18n_wrap_ui(kwin_showpaint_config_SRCS showpaint_config.ui)

    kwin_add_effect_config(kwin_showpaint_config ${kwin_showpaint_config_SRCS})

    target_link_libraries(kwin_showpaint_config
        KF6::KCMUtils
        KF6::CoreAddons
        KF6::GlobalAccel
        KF6::I18n
        KF6::XmlGui
    )
endif()
