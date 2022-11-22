idf_component_register(SRCS "real_time_stats_example_main.c"
							"GPS.c"
							"AT_Function.c"
							"MQTT.c"
							"Sim7070G_General_Control.c"
							"Sim7070G_Battery.c"
							"FOTA.c"
							"sensor.c"
							"SPIFFS_user.c"
							"led_indicator.c"
							"string_user.c"
                    INCLUDE_DIRS "."
                     # Embed the server root certificate into the final binary
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
#
# ULP support additions to component CMakeLists.txt.
#
# 1. The ULP app name must be unique (if multiple components use ULP).
set(ulp_app_name ulp_${COMPONENT_NAME})
#
# 2. Specify all assembly source files.
#    Files should be placed into a separate directory (in this case, ulp/),
#    which should not be added to COMPONENT_SRCS.
set(ulp_s_sources "ulp/blink.S")
#
# 3. List all the component source files which include automatically
#    generated ULP export file, ${ulp_app_name}.h:
set(ulp_exp_dep_srcs "real_time_stats_example_main.c")
#
# 4. Call function to build ULP binary and embed in project using the argument
#    values above.
ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")