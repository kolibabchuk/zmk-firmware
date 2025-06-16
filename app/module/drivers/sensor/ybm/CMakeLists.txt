zephyr_include_directories(.)
zephyr_library()
zephyr_library_sources(ybm.c)
zephyr_library_sources_ifdef(CONFIG_YBM_TRIGGER ybm_trigger.c)