# Add libraries
add_library(
	biped_sensor
	sensor.cpp
	camera.cpp
)

# Set library link targets
target_link_libraries(
	biped_sensor PRIVATE
	biped_common
	biped_platform
)



target_link_arduino_libraries(
	biped_sensor PRIVATE
	core
	WiFi
	WebServer
)